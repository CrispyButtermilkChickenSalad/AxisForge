"""Continuous, supervised homing contacts; mesh and scale contacts stay stepped."""
from dataclasses import replace
import statistics
import time

import numpy as np

from print_control.load_cell_contact import ContactDetected
from print_control.probe_motion import plan_probe_descent


class ContinuousHoming:
    """Mixin for the two guided homing passes, with no per-waypoint sample waits."""

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

    def capture_continuous_trigger(self):
        context = getattr(self, '_continuous_homing', None)
        if context is not None and context.get('trigger_xyz') is None:
            context['trigger_xyz'] = self.planner.nozzle_position(self.joints).copy()
            context['trigger_at'] = time.monotonic()

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
            return
        deadline = time.monotonic()+.25
        anchor = self.planner.nozzle_position(self.joints)
        anchor_joints = self.joints.copy()
        stable_since = time.monotonic()
        previous_sample = self.joint_received
        while time.monotonic() < deadline:
            self.pump()
            if self.joint_received == previous_sample:
                continue
            previous_sample = self.joint_received
            nozzle = self.planner.nozzle_position(self.joints)
            if (np.linalg.norm(nozzle-anchor) > .01 or
                    np.max(np.abs(self.joints-anchor_joints)) > .0001):
                anchor, anchor_joints = nozzle, self.joints.copy()
                stable_since = time.monotonic()
            elif time.monotonic()-stable_since >= .05:
                return
        raise RuntimeError('Robot did not settle after continuous homing cancellation; contact rejected')

    def homing_touch(self, *, floor_z, speed_mm_s):
        end = self.target.copy()
        end[2] = floor_z
        # Service ROS and sensor/watchdog checks throughout IK. Planning cannot
        # move the robot; only the validated motion below reaches the executor.
        motion = plan_probe_descent(self.planner, self.target, end, self.seed,
                                    speed_mm_s, self.homing.acceleration_mm_s2, pump=self.pump)
        original_config = self.config
        self.config = replace(self.config, cancel_timeout_s=self.homing.cancel_timeout_s)
        self._continuous_homing = context = dict(floor_z=floor_z, stop_travel_mm=0.)
        self.seeking = True
        try:
            # Refresh status after planning and reject an old latest sample even
            # if its enclosing batch was received recently.
            self.check_interrupted()
            try:
                self.run_homing_motion(motion)
                self.check_interrupted()
                raise RuntimeError('No contact before the configured homing floor; check bed placement')
            except ContactDetected:
                self.capture_continuous_trigger()
                self.cancel_contact()
                context['stop_ack_s'] = time.monotonic()-context['trigger_at']
                if not self.config.dry_run and context['stop_ack_s'] > self.homing.cancel_timeout_s+.04:
                    raise RuntimeError('Continuous homing cancellation exceeded its deadline; contact rejected')
            self.wait_homing_stationary()
            # Settling is required only once the robot has stopped, not after
            # every tiny increment during the descent.
            samples = self.window()
            self.detector.update(samples)
            load = statistics.median(abs(self.detector.load(s.counts)) for s in samples)
            if load < self.contact_options.threshold*.7:
                raise RuntimeError('Contact did not persist after stopping; point rejected')
            nozzle, joints = self.nozzle_now()
            self.check_continuous_homing()
            self.target, self.seed = nozzle.copy(), joints.copy()
            return dict(nozzle=nozzle, joints=joints, load=load,
                        peak_load=self.detector.peak_load, tare_counts=self.detector.baseline,
                        speed_mm_s=speed_mm_s, stop_ack_s=context['stop_ack_s'],
                        stop_travel_mm=context['stop_travel_mm'])
        except BaseException:
            # Use the shortened cancellation deadline on sensor/watchdog faults
            # as well as normal contact. BridgeBedProbe.stop supplies the
            # independent UR dashboard stop if action termination is unresolved.
            self.seeking = False
            self.stop()
            raise
        finally:
            self.seeking = False
            self._continuous_homing = None
            self.config = original_config
