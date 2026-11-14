"""Continuous homing lifecycle with synthetic feedback; no ROS node or robot."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from action_msgs.msg import GoalStatus

from print_control.bridge_bed_probe import HomingOptions
from print_control.continuous_probe import ContinuousHoming
from print_control.load_cell_bed_probe import LoadCellBedProbe
from print_control.load_cell_contact import ContactDetected, ContactDetector, ContactOptions, ForceSample
from print_control.motion_executor import MotionExecutor
from print_control.motion_planner import MotionConfig


class Clock:
    def __init__(self): self.now = 100.
    def __call__(self): return self.now
    def advance(self, seconds=.01): self.now += seconds


class Future:
    def __init__(self, result=None, done=True): self.value, self.ready = result, done
    def done(self): return self.ready
    def result(self): return self.value


class Source:
    def __init__(self, clock):
        self.clock = clock
        self.counts, self.age, self.sequence, self.error = 0., 0., 0, None
        self.received_at = clock()

    def snapshot(self):
        if self.error: raise RuntimeError(self.error)
        self.sequence += 5
        return [ForceSample(self.sequence+i, self.clock()-self.age, self.clock(), self.counts)
                for i in range(5)]


class ContactHarness(ContinuousHoming):
    """Actual contact detection/cancel lifecycle, with a fake trajectory sender."""
    def __init__(self, clock):
        self.clock = clock
        self.config = MotionConfig(dry_run=False, cancel_timeout_s=3.)
        self.homing = HomingOptions()
        self.contact_options = ContactOptions(trigger_mode='raw')
        self.source = Source(clock)
        self.detector = ContactDetector(0., None, self.contact_options)
        self.joints = np.array([1., 2., 105., 0., 0., 0.])
        self.seed = self.joints.copy()
        self.target = self.joints[:3].copy()
        self.planner = SimpleNamespace(nozzle_position=lambda q: q[:3].copy())
        self.events = []
        self.mode = 'contact'
        self.fresh = True
        self.seeking = False
        self.goal_future = self.goal_handle = self.result_future = self.motion_token = None
        self.stop_shift_mm = 0.
        self.stationarity_error = None
        self.cancel_delay = .01
        self.joint_received = clock()

    def joints_fresh(self): return self.fresh

    def check_interrupted(self):
        self.check_continuous_homing()
        if self.detector.update(self.source.snapshot()) and self.seeking:
            self.capture_continuous_trigger()
            raise ContactDetected('synthetic sustained contact')

    def pump(self):
        self.clock.advance()
        self.joint_received = self.clock()
        self.check_interrupted()

    def run_motion(self, motion):
        self.events.append(('send', self.config.cancel_timeout_s))
        self.goal_handle = SimpleNamespace(accepted=True)
        self.goal_future = Future(self.goal_handle)
        self.result_future = Future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED,
                                                    result=SimpleNamespace(error_code=0)))
        self.joints[2] = motion.end[2] if self.mode == 'no-contact' else 100.
        if self.mode == 'contact': self.source.counts = 25000.
        elif self.mode == 'overload': self.source.counts = 70000.
        elif self.mode == 'sensor-error': self.source.error = 'sensor disconnected'
        elif self.mode == 'stale': self.source.age = .151
        elif self.mode == 'unsettled-contact': self.source.counts = 25000.
        self.check_interrupted()

    def stop(self):
        self.events.append(('stop', self.config.cancel_timeout_s))
        self.clock.advance(self.cancel_delay)
        self.joints[2] -= self.stop_shift_mm

    def cancel_contact(self):
        self.events.append(('cancel', None))
        LoadCellBedProbe.cancel_contact(self)

    def nozzle_now(self):
        self.events.append(('measure', None))
        return self.joints[:3].copy(), self.joints.copy()

    def wait_homing_stationary(self):
        self.events.append(('stationary', None))
        if self.stationarity_error: raise RuntimeError(self.stationarity_error)
        self.check_continuous_homing()

    def window(self):
        self.events.append(('window', None))
        if self.mode == 'unsettled-contact': self.source.counts = 0.
        return self.source.snapshot()


class ContinuousContactTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.node = ContactHarness(self.clock)
        self.original_config = self.node.config
        self.addCleanup(patch.stopall)
        patch('print_control.continuous_probe.time.monotonic', self.clock).start()
        self.planned = []
        def plan(planner, start, end, seed, speed, acceleration, *, pump):
            self.planned.append((start.copy(), end.copy(), speed, acceleration))
            pump()
            return SimpleNamespace(end=end.copy())
        patch('print_control.continuous_probe.plan_probe_descent', side_effect=plan).start()

    def run_touch(self): return self.node.homing_touch(floor_z=85., speed_mm_s=10.)

    def assert_cleaned(self):
        self.assertIs(self.node.config, self.original_config)
        self.assertIsNone(self.node._continuous_homing)
        self.assertFalse(self.node.seeking)

    def test_single_descent_cancels_before_settling_and_measuring_contact(self):
        result = self.run_touch()
        names = [event[0] for event in self.node.events]
        self.assertEqual(names.count('send'), 1)
        self.assertEqual(names.count('window'), 1)
        self.assertLess(names.index('cancel'), names.index('measure'))
        self.assertLess(names.index('stop'), names.index('stationary'))
        self.assertLess(names.index('stationary'), names.index('window'))
        np.testing.assert_equal(self.planned[0][0][:2], self.planned[0][1][:2])
        self.assertEqual(self.planned[0][1][2], 85.)
        self.assertEqual(self.planned[0][2], 10.)
        self.assertEqual(result['speed_mm_s'], 10.)
        self.assertAlmostEqual(result['stop_ack_s'], .01)
        self.assertEqual(result['stop_travel_mm'], 0.)
        self.assertEqual(result['load'], 25000.)
        self.assert_cleaned()

    def test_no_contact_reaches_floor_and_fails_without_measurement_or_retraction(self):
        self.node.mode = 'no-contact'
        with self.assertRaisesRegex(RuntimeError, 'No contact.*floor'): self.run_touch()
        self.assertEqual(self.node.joints[2], 85.)
        self.assertEqual([e[0] for e in self.node.events], ['send', 'stop'])
        self.assert_cleaned()

    def test_fresh_batch_cannot_hide_old_latest_sample(self):
        self.node.mode = 'stale'
        with self.assertRaisesRegex(RuntimeError, 'ALPS samples.*150 ms'): self.run_touch()
        self.assertEqual(self.node.source.received_at, 100.)
        self.assertEqual(self.node.events[-1], ('stop', .2))
        self.assert_cleaned()

    def test_sensor_and_overload_faults_use_short_cancellation_and_restore_config(self):
        for mode, reason in (('sensor-error', 'sensor disconnected'), ('overload', 'overload')):
            self.node = ContactHarness(self.clock)
            self.original_config = self.node.config
            self.node.mode = mode
            with self.subTest(mode=mode), self.assertRaisesRegex(RuntimeError, reason): self.run_touch()
            self.assertEqual(self.node.events[-1], ('stop', .2))
            self.assertNotIn('window', [e[0] for e in self.node.events])
            self.assert_cleaned()

    def test_post_trigger_travel_rejects_contact_before_settled_window(self):
        self.node.stop_shift_mm = .51
        with self.assertRaisesRegex(RuntimeError, 'stop travel'): self.run_touch()
        self.assertNotIn('window', [e[0] for e in self.node.events])
        self.assert_cleaned()

    def test_cancellation_that_acknowledges_too_late_rejects_contact(self):
        self.node.cancel_delay = .25
        with self.assertRaisesRegex(RuntimeError, 'cancellation exceeded'): self.run_touch()
        self.assertNotIn('window', [e[0] for e in self.node.events])
        self.assert_cleaned()

    def test_contact_that_does_not_persist_is_rejected(self):
        self.node.mode = 'unsettled-contact'
        with self.assertRaisesRegex(RuntimeError, 'did not persist'): self.run_touch()
        self.assertEqual([e[0] for e in self.node.events].count('window'), 1)
        self.assert_cleaned()

    def test_stationarity_failure_prevents_contact_acceptance(self):
        self.node.stationarity_error = 'Robot did not settle'
        with self.assertRaisesRegex(RuntimeError, 'did not settle'): self.run_touch()
        self.assertNotIn('window', [e[0] for e in self.node.events])
        self.assert_cleaned()

    def test_actual_floor_and_joint_freshness_guards(self):
        self.node._continuous_homing = dict(floor_z=99., stop_travel_mm=0.)
        self.node.joints[2] = 98.9
        with self.assertRaisesRegex(RuntimeError, 'descent floor'): self.node.check_continuous_homing()
        self.node.joints[2] = 100.
        self.node.fresh = False
        with self.assertRaisesRegex(RuntimeError, 'joint feedback'): self.node.check_continuous_homing()

    def test_stationarity_requires_multiple_new_stable_feedback_samples(self):
        self.node.wait_homing_stationary = lambda: ContinuousHoming.wait_homing_stationary(self.node)
        self.node.wait_homing_stationary()
        self.assertGreaterEqual(self.clock(), 100.05)
        self.assertLess(self.clock(), 100.25)

    def test_stationarity_rejects_continuously_changing_feedback(self):
        def moving():
            self.clock.advance(.01)
            self.node.joint_received = self.clock()
            self.node.joints[2] -= .02
        self.node.pump = moving
        with self.assertRaisesRegex(RuntimeError, 'did not settle'):
            ContinuousHoming.wait_homing_stationary(self.node)

    def test_pending_goal_acceptance_is_resolved_and_cancelled_before_measurement(self):
        node = self.node
        node.config = replace(node.config, cancel_timeout_s=.2)
        node.goal_future = Future(done=False)
        terminal = Future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED,
                                         result=SimpleNamespace(error_code=0)), done=False)
        events = []
        def cancel():
            events.append('cancel request')
            terminal.ready = True
            return Future(SimpleNamespace(goals_canceling=[1]))
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: terminal,
                                 cancel_goal_async=cancel)
        def spin(*args, **kwargs):
            self.clock.advance(.01)
            node.goal_future.value = handle
            node.goal_future.ready = True
            events.append('acceptance')
        node.speed_publisher = None
        node.set_extrusion = lambda *args, **kwargs: None
        node.get_logger = lambda: SimpleNamespace(error=lambda message: events.append(message),
                                                  info=lambda message: events.append(message))
        node.stop = lambda: MotionExecutor.stop(node)
        with patch('print_control.motion_executor.rclpy.ok', return_value=True), \
                patch('print_control.motion_executor.rclpy.spin_once', side_effect=spin):
            node.cancel_contact()
        self.assertLess(events.index('acceptance'), events.index('cancel request'))
        self.assertTrue(terminal.done())
        self.assertEqual(node.events[-1][0], 'measure')
        self.assertIsNone(node.goal_handle)


if __name__ == '__main__': unittest.main()
