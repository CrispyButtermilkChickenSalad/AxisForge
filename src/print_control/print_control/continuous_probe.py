"""Continuous, supervised homing and mesh contacts; scale contacts stay stepped."""
from dataclasses import replace
from collections import deque
import statistics
import math
import time

import numpy as np

from print_control.load_cell_contact import ContactDetected, ContactMeasurementRejected
from print_control.probe_motion import plan_probe_descent
from print_control.motion_planner import JOINT_NAMES
from print_control.probe_settling import (JointFeedback, StationaryError, StationaryMotionError, estimate_stationary,
                                        LATEST_AGE_S, MAX_VELOCITY_RAD_S, RAW_POSITION_BOUND_MM,
                                        JOINT_BOUND_RAD)


# This is a stationary measurement observation budget, after acknowledged
# cancellation. Cancellation, overload, velocity and stop-travel limits remain
# separate and active throughout; never extend this deadline on each retry.
SETTLING_TIMEOUT_S = 4.


class ContinuousHoming:
    """Mixin for the two guided homing passes, with no per-waypoint sample waits."""

    def on_joints(self, message):
        """Retain positions, velocities and source freshness from one accepted sample."""
        previous = self.joints
        super().on_joints(message)
        if self.joints is previous:
            return
        if not hasattr(self, '_homing_feedback'):
            self._homing_feedback = deque(maxlen=2048)
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        velocity = dict(zip(message.name, message.velocity))
        self._homing_feedback.append(JointFeedback(
            received_at=self.joint_received, source_time=stamp/1e9,
            joints=self.joints.copy(),
            velocity=np.array([velocity.get(name, float('nan')) for name in JOINT_NAMES]),
            source_age_s=age))

    def check_continuous_homing(self):
        context = getattr(self, '_continuous_homing', None)
        if context is None:
            return
        samples = self.source.snapshot()
        if not samples or self.source.clock()-samples[-1].observed_at > self.homing.sample_age_s:
            raise RuntimeError('Continuous homing ALPS samples are older than %.0f ms' %
                               (self.homing.sample_age_s*1000.))
        if not self.config.dry_run and not self.joints_fresh():
            raise RuntimeError('Robot joint feedback is stale during continuous homing')
        nozzle = self.planner.nozzle_position(self.joints)
        if nozzle[2] < context['floor_z']-.05:
            raise RuntimeError('Continuous homing exceeded its descent floor')
        if context.get('trigger_xyz') is not None:
            travel = float(np.linalg.norm(nozzle-context['trigger_xyz']))
            context['stop_travel_mm'] = max(context['stop_travel_mm'], travel)
            if travel > self.homing.max_stop_travel_mm:
                raise RuntimeError('Continuous homing stop travel exceeds %.3f mm; contact rejected' %
                                   self.homing.max_stop_travel_mm)
            if (getattr(self, '_precision_landing_executing', False) and
                    time.monotonic()-context['trigger_at'] > self.homing.cancel_timeout_s):
                raise RuntimeError('Precision contact did not finish within its stop deadline')
        if context.get('terminal_confirmed') and time.monotonic() >= context['settling_deadline']:
            reason = context.get('settling_reason')
            raise ContactMeasurementRejected('Robot did not settle after continuous homing cancellation; '
                               'contact rejected: %g second observation deadline exceeded' % SETTLING_TIMEOUT_S +
                               ('; last observation: '+reason if reason else ''))
        anchor = context.get('settled_anchor')
        if anchor is not None and not self.config.dry_run:
            feedback = getattr(self, '_homing_feedback', ())
            latest = feedback[-1] if feedback else None
            if (latest is None or latest.received_at != self.joint_received or
                    not 0 <= time.monotonic()-(latest.received_at-latest.source_age_s) <= LATEST_AGE_S):
                raise ContactMeasurementRejected('Stationary joint feedback became stale during contact measurement')
            if not np.all(np.isfinite(latest.velocity)) or np.max(np.abs(latest.velocity)) > MAX_VELOCITY_RAD_S:
                raise ContactMeasurementRejected('Robot reports joint motion during contact measurement')
            # The force snapshot must be paired with this accepted pose, without
            # another wait or a shifted estimation window. Inspect any callbacks
            # received since the anchor too: a transient must not be hidden by
            # a subsequent zero-velocity sample.
            anchor_end = anchor.get('observed_end', latest.received_at)
            for sample in feedback:
                if sample.received_at > anchor_end:
                    if (not np.all(np.isfinite(sample.velocity)) or
                            np.max(np.abs(sample.velocity)) > MAX_VELOCITY_RAD_S):
                        raise ContactMeasurementRejected('Robot reports joint motion during contact measurement')
                    if (np.linalg.norm(self.planner.nozzle_position(sample.joints)-anchor['nozzle']) > RAW_POSITION_BOUND_MM or
                            np.max(np.abs(sample.joints-anchor['joints'])) > JOINT_BOUND_RAD):
                        raise ContactMeasurementRejected('Robot left its settled contact pose during measurement')
            if time.monotonic()-context.get('measurement_started_at', anchor_end) > LATEST_AGE_S:
                raise ContactMeasurementRejected('Stationary contact measurement exceeded its fresh pose interval')
            if (np.linalg.norm(nozzle-anchor['nozzle']) > RAW_POSITION_BOUND_MM or
                    np.max(np.abs(self.joints-anchor['joints'])) > JOINT_BOUND_RAD):
                raise ContactMeasurementRejected('Robot left its settled contact pose during measurement')

    def capture_continuous_trigger(self):
        context = getattr(self, '_continuous_homing', None)
        if context is not None and context.get('trigger_xyz') is None:
            context['trigger_xyz'] = self.planner.nozzle_position(self.joints).copy()
            context['trigger_at'] = time.monotonic()

    def finish_precision_contact_step(self):
        """Latch contact only while a validated, bounded landing is executing."""
        if not getattr(self, '_precision_landing_executing', False):
            return False
        self.capture_continuous_trigger()
        self.check_continuous_homing()
        return True

    def execute_motion(self, motion, extrusion=None):
        """Permit natural completion only for this exact precision segment."""
        if motion is not getattr(self, '_precision_landing_motion', None):
            return super().execute_motion(motion, extrusion)
        # Robot safety queries and planning ran with ordinary immediate contact
        # cancellation. Recheck before granting the bounded finishing interval.
        self.check_interrupted()
        self._precision_landing_executing = True
        self._continuous_homing['precision_landing_started'] = True
        try:
            return super().execute_motion(motion, extrusion)
        finally:
            self._precision_landing_executing = False

    def run_precision_descent(self, *, floor_z, reference_z, speed_mm_s, search_step_mm=None):
        """Approach, then locate with cancellable steps or measure with small landings.

        Successful action completion holds its planned endpoint; cancellation
        instead holds a noisy instantaneous encoder sample on the UR controller.
        Finishing an already active landing is bounded by distance and deadline,
        with no further move after contact and no suspension of fault guards.
        Larger search steps never receive this finishing permission.
        """
        if not math.isfinite(reference_z) or not floor_z < reference_z < self.target[2]:
            raise ValueError('Precision contact requires a surface reference inside its search range')
        if search_step_mm is not None and (not math.isfinite(search_step_mm) or
                                          not .01 <= search_step_mm <= .1):
            raise ValueError('Near-contact search steps must be 0.01..0.1 mm')
        approach_z = reference_z + .3
        if self.target[2] > approach_z + .001:
            end = self.target.copy()
            end[2] = approach_z
            approach = plan_probe_descent(self.planner, self.target, end, self.seed,
                speed_mm_s, self.homing.acceleration_mm_s2, pump=self.pump)
            self.run_homing_motion(approach)
            self.target, self.seed = end, approach.positions[-1].copy()
        step = search_step_mm or min(.01, speed_mm_s*self.homing.cancel_timeout_s/3.)
        if step < .00001:
            raise ValueError('Precision probing speed is too low for a bounded landing')
        while self.target[2] > floor_z + 1e-8:
            self.check_interrupted()
            end = self.target.copy()
            end[2] = max(floor_z, end[2]-step)
            motion = self.planner.sampled_line(
                'near-contact search' if search_step_mm is not None else 'precision contact landing', self.target,
                end, self.seed, speed_mm_s, pump=self.pump)
            if search_step_mm is None and (float(motion.times[-1]) > self.homing.cancel_timeout_s*.75 or
                    np.linalg.norm(end-self.target) > .010001):
                raise ValueError('Precision landing exceeds its distance or stop-time budget')
            # Larger locator steps always cancel immediately on contact. Only
            # <=10 micrometre precision landings may finish their active step.
            self._precision_landing_motion = motion if search_step_mm is None else None
            if search_step_mm is not None:
                self._continuous_homing['stepped_locator'] = True
            try:
                self.run_homing_motion(motion)
            finally:
                self._precision_landing_motion = None
            self.target, self.seed = end, motion.positions[-1].copy()
            # Inspect samples from this completed endpoint before another step.
            # A latched trigger raises here if it did not raise during execution.
            self.check_interrupted()
            self.window()
        raise RuntimeError('No contact before the configured homing floor; check bed placement')

    def run_homing_motion(self, motion):
        if not self.config.dry_run:
            self.run_motion(motion)
            return
        # Simulate contact along the path instead of jumping through the bed to
        # the floor. Never construct or send a ROS action in simulation.
        for joints in motion.positions:
            self.joints = joints.copy()
            self.seed = joints.copy()
            self.target = self.planner.nozzle_position(joints)
            self.check_interrupted()

    def wait_homing_stationary(self):
        if self.config.dry_run:
            return dict(nozzle=self.planner.nozzle_position(self.joints), joints=self.joints.copy())
        context = getattr(self, '_continuous_homing', None) or {}
        deadline = context.get('settling_deadline', time.monotonic()+SETTLING_TIMEOUT_S)
        next_check = time.monotonic()
        reason = 'No fresh joint feedback after cancellation'
        while time.monotonic() < deadline:
            self.pump()
            if time.monotonic() < next_check:
                continue
            next_check = time.monotonic()+.05
            try:
                settled = self.stationary_homing_pose()
                context.pop('settling_reason', None)
                return settled
            except StationaryError as error:
                reason = str(error)
                context['settling_reason'] = reason
                # Controller cancellation may precede the last deceleration or
                # pose-drift samples, even with zero reported joint velocity.
                # Wait for an entire clean window without accepting a failed
                # estimate or extending the original deadline. Motion
                # after the first settled pose remains a fatal measurement fault.
                initial_motion = (isinstance(error, StationaryMotionError) and
                                  'settled_anchor' not in context)
                if not (error.waitable or initial_motion):
                    break
        raise ContactMeasurementRejected('Robot did not settle after continuous homing cancellation; '
                           'contact rejected: '+reason)

    def stationary_homing_pose(self):
        if self.config.dry_run:
            return dict(nozzle=self.planner.nozzle_position(self.joints), joints=self.joints.copy())
        context = getattr(self, '_continuous_homing', None) or {}
        return estimate_stationary(getattr(self, '_homing_feedback', ()), time.monotonic(),
            self.planner.nozzle_position, stopped_at=context.get('stopped_at', float('nan')),
            terminal_confirmed=context.get('terminal_confirmed') is True)

    def homing_load_window(self, settled):
        """Read force already captured inside the validated stationary interval.

        Waiting for another force window after accepting the pose used to shift
        the joint estimate unnecessarily. Both sensors are already buffered;
        pair their observations once, while all freshness/load guards stay live.
        Never reuse pre-stop data or accept an incomplete/stale force window.
        """
        if self.config.dry_run:
            return self.window()
        context = self._continuous_homing
        samples = self.source.snapshot()
        now = self.source.clock()
        start, end = settled['observed_start'], settled['observed_end']
        after = max(start, context['stopped_at']+self.contact_options.settle_s)
        samples = [s for s in samples if after <= s.observed_at <= end]
        if not samples or not 0 <= now-samples[-1].observed_at <= self.homing.sample_age_s:
            raise ContactMeasurementRejected('No fresh ALPS window inside the settled robot interval')
        cutoff = samples[-1].source_time-self.contact_options.window_s
        samples = [s for s in samples if s.source_time >= cutoff]
        if (len(samples) < 10 or
                samples[-1].source_time-samples[0].source_time < self.contact_options.window_s-.005):
            raise ContactMeasurementRejected('Incomplete ALPS window inside the settled robot interval')
        if (any(not all(math.isfinite(v) for v in (s.source_time, s.observed_at, s.counts)) for s in samples) or
                any(b.sequence <= a.sequence or not 0 < b.source_time-a.source_time <= .05
                    or b.observed_at < a.observed_at for a, b in zip(samples, samples[1:]))):
            raise ContactMeasurementRejected('Invalid ALPS timestamps inside the settled robot interval')
        return samples

    def commit_homing_pose(self, settled):
        """Service feedback queued during estimation before committing its pose.

        The force is still paired with the validated historical interval. The
        current robot guard needs a new source-fresh sample, not the callback
        that preceded the finite computation. Any intervening motion, missing
        interval or stale source data rejects the candidate without moving it.
        """
        context = self._continuous_homing
        if not self.config.dry_run:
            end = settled['observed_end']
            deadline = time.monotonic()+LATEST_AGE_S
            while True:
                self.pump()
                feedback = list(getattr(self, '_homing_feedback', ()))
                recent = [s for s in feedback if s.received_at >= end]
                now = time.monotonic()
                if (recent and recent[-1].received_at > end and
                        0 <= now-(recent[-1].received_at-recent[-1].source_age_s) <= LATEST_AGE_S):
                    if (recent[0].received_at != end or
                            any(not 0 <= s.source_age_s <= LATEST_AGE_S for s in recent) or
                            any(not 0 < b.received_at-a.received_at or
                                not 0 < b.source_time-a.source_time <= LATEST_AGE_S
                                for a, b in zip(recent, recent[1:]))):
                        raise ContactMeasurementRejected('Joint feedback gap while committing the settled contact pose')
                    # Estimation briefly delays callback delivery. Continuity
                    # comes from the robot sample timestamps; every delivered
                    # sample still has to satisfy the unchanged 20 ms source
                    # age and source-gap bounds. A 20 ms callback interval alone
                    # is not missing feedback when its source interval is 12 ms.
                    break
                if now >= deadline:
                    raise ContactMeasurementRejected('No fresh joint feedback after stationary pose estimation')
        context['settled_anchor'] = settled
        context['measurement_started_at'] = time.monotonic()
        self.check_continuous_homing()

    def homing_touch(self, *, floor_z, speed_mm_s, locating_only=False, precision_reference_z=None,
                     allow_approach_locator=False, near_contact_step_mm=None):
        if near_contact_step_mm is not None and (not allow_approach_locator or precision_reference_z is None):
            raise ValueError('Larger near-contact steps require a locator followed by precision refinement')
        end = self.target.copy()
        end[2] = floor_z
        # Service ROS and sensor/watchdog checks throughout IK. Planning cannot
        # move the robot; only the validated motion below reaches the executor.
        try:
            motion = plan_probe_descent(self.planner, self.target, end, self.seed,
                                        speed_mm_s, self.homing.acceleration_mm_s2, pump=self.pump)
        except ValueError as error:
            detail = ('Cannot plan the full Z search from current nozzle Base Z %.3f mm '
                      'through %.3f mm downward (end Z %.3f mm): %s. '
                      'No motion was sent for this search.' %
                      (self.target[2], self.target[2]-end[2], end[2], error))
            if 'min_link_z_mm' in str(error):
                detail += (' Set Maximum search descent for the CURRENT nozzle-to-surface gap '
                           'plus a margin, rather than a previous home pose. '
                           'The entire search range must pass the arm clearance checks; '
                           'reduce the range or reposition the arm before retrying.')
            raise ValueError(detail) from error
        original_config = self.config
        self.config = replace(self.config, cancel_timeout_s=self.homing.cancel_timeout_s)
        self._continuous_homing = context = dict(floor_z=floor_z, stop_travel_mm=0.)
        native = getattr(self.source, 'native_trigger', None)
        try:
            if native is not None:
                timeout = float(motion.times[-1]-motion.times[0])+5.
                if precision_reference_z is not None:
                    step = near_contact_step_mm or min(.01, speed_mm_s*self.homing.cancel_timeout_s/3.)
                    if step < .00001:
                        raise ValueError('Precision probing speed is too low for a bounded landing')
                    # Include finite per-step safety queries and post-goal sensor
                    # windows. Sensor/robot freshness deadlines stay unchanged.
                    count = math.ceil(max(0., precision_reference_z+.3-floor_z)/step)
                    timeout += count*5.
                if not 0 < timeout <= 7200.:
                    raise RuntimeError('Planned homing duration exceeds the native probe search deadline')
                native.begin_contact(self.pump, search_timeout_s=timeout)
            self.seeking = True
            # Refresh status after planning and reject an old latest sample even
            # if its enclosing batch was received recently.
            self.check_interrupted()
            try:
                if precision_reference_z is None or self.config.dry_run:
                    self.run_homing_motion(motion)
                else:
                    kwargs = {} if near_contact_step_mm is None else {'search_step_mm': near_contact_step_mm}
                    self.run_precision_descent(floor_z=floor_z, reference_z=precision_reference_z,
                                               speed_mm_s=speed_mm_s, **kwargs)
                self.check_interrupted()
                raise RuntimeError('No contact before the configured homing floor; check bed placement')
            except ContactDetected:
                self.capture_continuous_trigger()
                self.cancel_contact()
                context['terminal_confirmed'] = True
                context['stopped_at'] = time.monotonic()
                context['settling_deadline'] = context['stopped_at']+SETTLING_TIMEOUT_S
                # Exclude the earlier descent. Source ages still distinguish
                # genuinely post-stop samples from queued pre-stop messages.
                self._homing_feedback = deque(maxlen=2048)
                context['stop_ack_s'] = time.monotonic()-context['trigger_at']
                if not self.config.dry_run and context['stop_ack_s'] > self.homing.cancel_timeout_s+.04:
                    raise RuntimeError('Continuous homing cancellation exceeded its deadline; contact rejected')
                if native is not None: native.confirm_contact(self.pump)
            settled = self.wait_homing_stationary()
            self.commit_homing_pose(settled)
            samples = self.homing_load_window(settled)
            self.check_continuous_homing()
            self.detector.update(samples)
            load = statistics.median(abs(self.detector.load(s.counts)) for s in samples)
            # The first pass only bounds the subsequent precision search. Its
            # latched trigger remains a useful locator if contact unloads after
            # stopping. Never use that approximate pass as a measured bed height.
            # A first mesh approach may also locate a higher surface before its
            # precision band. It must be refined after retracting; precision
            # landings and repeated contacts always retain the persistence test.
            locator = locating_only or (allow_approach_locator and
                                       not context.get('precision_landing_started'))
            if not locator and load < self.contact_options.threshold*.7:
                raise ContactMeasurementRejected(
                    'Contact did not persist after stopping: %.1f %s remains; %.1f %s required; point rejected' %
                    (load, self.contact_options.unit, self.contact_options.threshold*.7, self.contact_options.unit))
            nozzle, joints = settled['nozzle'].copy(), settled['joints'].copy()
            self.check_continuous_homing()
            self.target, self.seed = nozzle.copy(), joints.copy()
            return dict(nozzle=nozzle, joints=joints, load=load,
                        peak_load=self.detector.peak_load, tare_counts=self.detector.baseline,
                        contact_persistence_checked=not locator,
                        contact_method=('bounded_precision_steps' if context.get('precision_landing_started')
                                        else 'stepped_locator_cancel' if context.get('stepped_locator')
                                        else 'continuous_cancel'),
                        speed_mm_s=speed_mm_s, stop_ack_s=context['stop_ack_s'],
                        stop_travel_mm=context['stop_travel_mm'])
        except BaseException as error:
            # Use the shortened cancellation deadline on sensor/watchdog faults
            # as well as normal contact. BridgeBedProbe.stop supplies the
            # independent UR dashboard stop if action termination is unresolved.
            self.seeking = False
            recorder = getattr(self, 'record_contact_failure', None)
            try:
                self.stop()
            except BaseException as stop_error:
                context['stop_failure'] = str(stop_error)
                if recorder is not None:
                    recorder(error)
                raise RuntimeError('%s; stop failure: %s' % (error, stop_error)) from stop_error
            else:
                if recorder is not None:
                    recorder(error)
            raise
        finally:
            self.seeking = False
            self._continuous_homing = None
            self.config = original_config
            if native is not None:
                native.end_contact()
