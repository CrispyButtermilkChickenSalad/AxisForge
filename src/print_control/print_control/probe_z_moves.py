"""Operator-requested Z positioning and contact release inside an owned routine."""
from dataclasses import replace
import math
import statistics
import time

import numpy as np

from print_control.load_cell_contact import (
    ContactDetected, ContactDetector, ContactMeasurementRejected, LoadCellOverload,
    stable_baseline,
)
from print_control.load_calibration_policy import CalibrationDetector
from print_control.probe_control import Z_STEP_SIZES
from print_control.probe_settling import LATEST_AGE_S, MAX_VELOCITY_RAD_S


class ZReleaseLoadRise(RuntimeError):
    """Stop the current release step; retain the monitored recovery session."""


class ProbeZMoves:
    """Guarded Z controls, shared with the mesh routine's automatic recovery."""

    def run_routine(self):
        try:
            return self._run_routine()
        except Exception as error:
            if getattr(self, '_mesh_phase_active', False):
                self.recover_mesh_failure(error)
            if not isinstance(error, (ContactMeasurementRejected, LoadCellOverload)):
                raise
            alignment = getattr(self, 'start_alignment', None)
            if alignment and alignment['required']:
                raise RuntimeError('Upright alignment is not verified; Z controls remain closed: '+str(error)) from error
            self.manual_z_session(recovery=True, failure=str(error))
            raise

    def _z_samples(self):
        samples = self.source.snapshot()
        if not samples or self.source.clock()-samples[-1].observed_at > self.homing.sample_age_s:
            raise RuntimeError('Z controls require fresh ALPS samples')
        newest = samples[-1].source_time
        return [s for s in samples if s.source_time >= newest-self.contact_options.window_s]

    def _z_unloaded(self, samples):
        if (len(samples) < 10 or
                samples[-1].source_time-samples[0].source_time < self.contact_options.window_s-.005 or
                max(abs(self.detector.load(s.counts)) for s in samples) >= self.contact_options.threshold*.25):
            return False
        try:
            stable_baseline(samples, self.source.sensor, self.contact_options, self.detector.baseline)
        except ValueError:
            return False
        return True

    def _z_require_stationary(self):
        self.control.check()
        if getattr(self, 'stop_unconfirmed', False):
            raise RuntimeError('Z controls require a confirmed robot stop')
        self.check_interrupted()
        self.check_z_stop_travel()
        if self.config.dry_run:
            return
        if not self.joints_fresh():
            raise RuntimeError('Z controls require fresh robot feedback')
        # Baseline/status collection can return between ROS callbacks. Wait for
        # a fresh observation while stationary instead of treating a previously
        # valid but aged sample as a new robot fault. Never accept that old sample
        # or send a movement while waiting; the same source-age bound applies.
        feedback_deadline = time.monotonic()+min(.25, self.config.feedback_timeout_s)
        observation_anchor = self.planner.nozzle_position(self.joints).copy()
        while True:
            feedback = getattr(self, '_homing_feedback', ())
            last = feedback[-1] if feedback else None
            if last is not None and (np.shape(last.velocity) != (6,) or not np.all(np.isfinite(last.velocity))):
                raise RuntimeError('Z controls require fresh finite joint velocities')
            if (last is not None and last.received_at == self.joint_received and
                    0 <= time.monotonic()-(last.received_at-last.source_age_s) <= LATEST_AGE_S):
                break
            if time.monotonic() >= feedback_deadline:
                raise RuntimeError('Z controls require fresh finite joint velocities')
            self.pump()
            self.check_z_stop_travel()
            if np.linalg.norm(self.planner.nozzle_position(self.joints)-observation_anchor) > self.homing.max_stop_travel_mm:
                raise RuntimeError('Robot moved too far while waiting for fresh Z-control feedback')
        deadline = time.monotonic()+2.
        anchor = self.planner.nozzle_position(self.joints).copy()
        while np.max(np.abs(last.velocity)) > MAX_VELOCITY_RAD_S:
            if time.monotonic() >= deadline:
                raise RuntimeError('Robot did not stop within the Z-control observation deadline')
            self.pump()
            self.check_z_stop_travel()
            if np.linalg.norm(self.planner.nozzle_position(self.joints)-anchor) > self.homing.max_stop_travel_mm:
                raise RuntimeError('Robot moved too far while waiting for Z-control termination')
            feedback = getattr(self, '_homing_feedback', ())
            last = feedback[-1] if feedback else None
            if (last is None or last.received_at != self.joint_received or
                    not 0 <= time.monotonic()-(last.received_at-last.source_age_s) <= LATEST_AGE_S or
                    np.shape(last.velocity) != (6,) or not np.all(np.isfinite(last.velocity))):
                raise RuntimeError('Z controls require fresh finite joint velocities')
        self.check_z_stop_travel()

    def reanchor_z_start(self):
        """Capture a new search origin without resetting the simulated/current pose."""
        self._z_require_stationary()
        nozzle, joints = self.nozzle_now()
        self.planner.check_pose(joints, nozzle, nozzle, rotation=self._nominal_probe_rotation)
        self.freeze_probe_orientation(joints)
        self.target, self.seed = nozzle.copy(), joints.copy()
        self.start_xyz, self.start_joints = nozzle.copy(), joints.copy()

    def z_waiting_anchor(self):
        """Fix the idle guard's reference from fresh stopped observations.

        Using one noisy encoder sample as the reference doubles the apparent
        excursion between opposite noise extremes. Keep checking every later
        raw sample against the same 0.1 mm bound, but centre it on a short median
        rather than that arbitrary first sample. No rolling/reanchored guard.
        """
        if self.config.dry_run:
            return self.nozzle_now()
        began=time.monotonic()
        origin=self.planner.nozzle_position(self.joints).copy()
        while time.monotonic()-began < 2.:
            self.pump()
            self.check_z_stop_travel()
            if np.linalg.norm(self.planner.nozzle_position(self.joints)-origin)>self.homing.max_stop_travel_mm:
                raise RuntimeError('Robot moved too far while establishing the Z-control idle reference')
            recent=max(began,time.monotonic()-.15)
            samples=[s for s in getattr(self, '_homing_feedback', ())
                     if s.received_at-s.source_age_s > recent]
            if not samples:
                continue
            if any(np.shape(s.joints)!=(6,) or not np.all(np.isfinite(s.joints)) or
                   np.shape(s.velocity)!=(6,) or not np.all(np.isfinite(s.velocity)) for s in samples):
                raise RuntimeError('Finite joint feedback is required for the Z-control idle reference')
            if any(np.max(np.abs(s.velocity))>MAX_VELOCITY_RAD_S for s in samples):
                continue  # Require a complete clean window after residual deceleration.
            if (len(samples)<20 or samples[-1].received_at-samples[0].received_at<.1 or
                    not 0<=time.monotonic()-(samples[-1].received_at-samples[-1].source_age_s)<=LATEST_AGE_S):
                continue
            gaps=np.diff([s.source_time for s in samples])
            arrivals=np.diff([s.received_at for s in samples])
            if (any(not math.isfinite(s.source_age_s) or s.source_age_s<0 for s in samples) or
                    not np.all(np.isfinite(gaps)) or np.any(gaps<=0) or np.any(arrivals<=0)):
                raise RuntimeError('Fresh continuous joint feedback is required for the Z-control idle reference')
            if (any(s.source_age_s>LATEST_AGE_S for s in samples) or
                    np.max(gaps)>LATEST_AGE_S or np.max(arrivals)>LATEST_AGE_S):
                # Discard this whole candidate window, without moving or using
                # stale data. A clean replacement must fit the original 2 s
                # observation budget and the unchanged travel/load guards.
                continue
            joints=np.median(np.asarray([s.joints for s in samples]),axis=0)
            expected=self.planner.nozzle_position(joints)
            if any(np.linalg.norm(self.planner.nozzle_position(s.joints)-expected)>.1 or
                   np.max(np.abs(s.joints-joints))>math.radians(.1) for s in samples):
                raise RuntimeError('Arm moved while establishing the Z-control idle reference')
            return expected,joints
        raise RuntimeError('Timed out collecting fresh stationary Z-control feedback')

    def _z_release_guard(self, load):
        if load >= 250000.:
            raise RuntimeError('Z recovery hard load ceiling reached (250000 counts); manual recovery required')
        if load > self._z_release_ceiling:
            self._z_release_blocked = True
            if getattr(self, '_z_manual_active', False):
                raise ZReleaseLoadRise('Load increased during upward release; Z step stopped')

    def _z_release_limit(self, error):
        if not self._z_stop_confirmed or not self._z_recovery_active:
            raise error
        # The ordinary overload remains latched. Only this bounded +Z release
        # session may hold it, and every sample still passes the rise/hard guards.

    def _z_prepare_recovery(self):
        self.control.check()
        if (getattr(self, 'stop_unconfirmed', False) or self.detector is None or
                self.contact_options.trigger_mode != 'raw' or
                not math.isfinite(self.detector.baseline)):
            raise RuntimeError('Z recovery requires a retained unloaded reference and confirmed stop')
        # Establish termination before enabling the special upward release guard.
        self.confirm_motion_stopped()
        self._z_stop_confirmed = True
        native = getattr(self.source, 'native_trigger', None)
        if native is not None:
            native.end_contact()
        # snapshot() requires fresh validated printer health from the existing
        # background poller as well as fresh force data. A synchronous HTTP
        # refresh here can starve ROS long enough to destroy the recovery session.
        samples = self._z_samples()
        peak = max(abs(self.detector.load(s.counts)) for s in samples)
        self._z_release_ceiling = min(250000., peak+self.contact_options.threshold*.25)
        self._z_release_blocked = False
        self._z_release_guard(peak)
        old = self.detector
        detector = CalibrationDetector(old.baseline, self.source.sensor.get('counts_per_gram'),
            self.contact_options, self._z_release_limit, self._z_release_guard)
        detector.last_sequence = old.last_sequence
        detector.peak_load = old.peak_load
        self.detector = detector
        self._z_recovery_active = True
        self.seeking = False
        # Service a new ROS sample before evaluating stationary feedback.
        nozzle, joints = self.nozzle_now()
        self.target, self.seed = nozzle.copy(), joints.copy()
        self._z_require_stationary()

    def manual_z_values(self):
        samples = self._z_samples()
        unloaded = self._z_unloaded(samples)
        if getattr(self, '_z_recovery_active', False):
            # Never increase the original ceiling or tare away a rise. Re-enable
            # operator steps only after a full fresh window is below that ceiling.
            if (len(samples) >= 10 and
                    samples[-1].source_time-samples[0].source_time >= self.contact_options.window_s-.005 and
                    max(abs(self.detector.load(s.counts)) for s in samples) <= self._z_release_ceiling):
                self._z_release_blocked = False
        if unloaded and getattr(self, '_z_recovery_active', False):
            # Release is a one-way exception. Once the original reference is
            # unloaded again, restore the ordinary overload stop before allowing
            # larger upward positioning; never reuse the old loaded allowance.
            old = self.detector
            self.detector = ContactDetector(old.baseline, self.source.sensor.get('counts_per_gram'),
                                            self.contact_options)
            self.detector.last_sequence, self.detector.peak_load = old.last_sequence, old.peak_load
            self._z_recovery_active = False
        blocked = getattr(self, '_z_release_blocked', False)
        return dict(phase=self._z_phase, position_mm=[float(v) for v in self.target],
            lower_z_mm=self._z_lower, upper_z_mm=self._z_upper,
            delta_counts=float(statistics.median(self.detector.load(s.counts) for s in samples)),
            upward_only=self._z_phase == 'recovery' or not unloaded, loaded=not unloaded,
            can_continue=self._z_phase == 'positioning' and unloaded and not blocked,
            can_finish=self._z_phase == 'recovery' and unloaded and not blocked,
            release_blocked=blocked,
            step_sizes=list(Z_STEP_SIZES), failure=self._z_failure)

    def manual_z_step(self, delta):
        if type(delta) not in (int, float) or not math.isfinite(delta) or abs(delta) not in Z_STEP_SIZES:
            raise ValueError('Choose a supported Z step size')
        self._z_require_stationary()
        values = self.manual_z_values()
        if values['release_blocked']:
            return 'Load rose during recovery. Wait for fresh readings below the original release ceiling before another Up step.'
        if values['upward_only'] and delta < 0:
            return 'Only Up is allowed until the nozzle is unloaded; recovery never permits Down.'
        if values['loaded'] and delta > .1:
            return 'Use Up 0.1 mm or finer while load is applied.'
        # Tiny release steps must not start from one noisy encoder extreme:
        # correcting that arbitrary starting pose can briefly increase contact
        # load before the requested Up move. Use the same fresh stopped median
        # and retained load guards as the idle reference.
        nozzle, joints = self.z_waiting_anchor()
        self.planner.check_pose(joints, nozzle, nozzle, rotation=self._nominal_probe_rotation)
        self.freeze_probe_orientation(joints)
        self.target, self.seed = nozzle.copy(), joints.copy()
        end = nozzle.copy()
        end[2] += delta
        if not self._z_lower-1e-8 <= end[2] <= self._z_upper+1e-8:
            return 'This Z step exceeds the current positioning range; choose a smaller step.'
        speed = .1 if values['loaded'] else min(1., self.homing.first_speed_mm_s) if delta < 0 else self.homing.lift_speed_mm_s
        # Plan before setting the moving state. A rejected path keeps the session
        # available without catching any failure from actual motion execution.
        try:
            motion = self.planner.sampled_line('dashboard Z step', nozzle, end, joints, speed,
                                               pump=self.pump)
        except ValueError as error:
            return 'Z step was not sent: '+str(error)
        latest = self.manual_z_values()
        if latest['release_blocked']:
            return 'Load rose while preparing the step; no motion sent. Wait for the release guard to clear.'
        if latest['loaded'] != values['loaded']:
            return 'Load changed while preparing the Z step; review the fresh reading and request it again.'
        if latest['upward_only'] and delta < 0:
            return 'Only Up is allowed until the nozzle is unloaded.'
        if delta < 0:
            # Every downward size is a contact search, including 0.05 mm. Keep
            # the original reference rather than taring at the new position.
            detector = ContactDetector(self.detector.baseline,
                self.source.sensor.get('counts_per_gram'), self.contact_options)
            samples = self._z_samples()
            if not self._z_unloaded(samples):
                return 'The nozzle must remain unloaded before any downward positioning step.'
            detector.last_sequence = samples[-1].sequence
            self.detector = detector
        self.control.emit('z_stepping', 'Moving Z by %+.3f mm' % delta)
        self.seeking = delta < 0
        self._z_manual_active = True
        original_config = self.config
        self.config = replace(self.config, cancel_timeout_s=self.homing.cancel_timeout_s)
        try:
            self.check_interrupted()
            self.run_motion(motion)
            nozzle, joints = self.nozzle_now()
            self.target, self.seed = nozzle.copy(), joints.copy()
            self._z_require_stationary()
        except ContactDetected:
            self._z_stop_anchor = self.planner.nozzle_position(self.joints).copy()
            self.cancel_contact()
            self.check_z_stop_travel()
            self._z_require_stationary()
            return 'ALPS detected contact. Downward motion stopped; use small Up steps to unload.'
        except ZReleaseLoadRise:
            # Keep every-sample rise detection, but stop only this requested
            # step. Idle monitoring can then observe the load clearing without
            # terminating the session or automatically repeating the movement.
            self._z_manual_active = False
            self._z_stop_anchor = self.planner.nozzle_position(self.joints).copy()
            self.cancel_contact()
            self.check_z_stop_travel()
            self._z_require_stationary()
            return ('Up step stopped because ALPS load increased. Recovery remains open; '
                    'wait for the load guard to clear before requesting another Up step. '
                    'If it stays blocked, stop and inspect the toolhead/contact direction.')
        finally:
            self.seeking = False
            self._z_manual_active = False
            self._z_stop_anchor = None
            self.config = original_config
        return ''

    def check_z_stop_travel(self):
        anchor = getattr(self, '_z_stop_anchor', None)
        if anchor is not None and np.linalg.norm(self.planner.nozzle_position(self.joints)-anchor) > self.homing.max_stop_travel_mm:
            raise RuntimeError('Dashboard Z contact stop exceeded its travel limit; manual recovery required')

    def manual_z_session(self, *, recovery=False, failure=''):
        self._z_recovery_active = False
        self._z_release_blocked = False
        self._z_stop_confirmed = False
        if recovery:
            self._z_prepare_recovery()
        else:
            self._z_require_stationary()
        self._z_phase = 'recovery' if recovery else 'positioning'
        self._z_failure = failure
        origin = float(self.target[2])
        self._z_lower = origin-.1 if recovery else origin-self.homing.max_descent_mm
        self._z_upper = origin+(25. if recovery else self.homing.max_descent_mm)
        try:
            while True:
                expected, expected_joints = self.z_waiting_anchor()
                def guarded_pump():
                    self.pump()
                    actual = self.planner.nozzle_position(self.joints)
                    if (np.linalg.norm(actual-expected) > .1 or
                            np.max(np.abs(self.joints-expected_joints)) > math.radians(.1)):
                        raise RuntimeError('Arm moved outside dashboard Z controls; restart the routine')
                action = self.control.wait_z_action(self._z_phase, guarded_pump, self.manual_z_values)
                guarded_pump()
                values = self.manual_z_values()
                if action['command'] in ('continue', 'finish_recovery'):
                    allowed = values['can_finish'] if recovery else values['can_continue']
                    if not allowed:
                        self.control.emit('z_step_rejected', 'Use Up until fresh readings confirm the nozzle is unloaded.')
                        continue
                    self._z_require_stationary()
                    if not recovery:
                        self.reanchor_z_start()
                    return
                error = self.manual_z_step(action['delta_mm'])
                if error:
                    self.control.emit('z_step_rejected', error)
                self.window()  # Fresh readings determine loaded/unloaded after this step.
        finally:
            self._z_manual_active = False
            self._z_recovery_active = False
