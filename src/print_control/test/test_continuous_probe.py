"""Continuous homing lifecycle with synthetic feedback; no ROS node or robot."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
from action_msgs.msg import GoalStatus

from print_control.bridge_bed_probe import BridgeBedProbe, HomingOptions
from print_control.continuous_probe import ContinuousHoming, SETTLING_TIMEOUT_S
from print_control.load_cell_bed_probe import LoadCellBedProbe
from print_control.load_cell_contact import ContactDetected, ContactDetector, ContactOptions, ForceSample
from print_control.motion_executor import MotionExecutor
from print_control.motion_planner import JOINT_NAMES, MotionConfig
from print_control.probe_settling import JointFeedback, StationaryError, WINDOW_S


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
    spin_feedback = MotionExecutor.spin_feedback
    """Actual contact detection/cancel lifecycle, with a fake trajectory sender."""
    confirm_motion_stopped = LoadCellBedProbe.confirm_motion_stopped
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
        self._homing_feedback = []

    def joints_fresh(self): return self.fresh

    def check_interrupted(self):
        self.check_continuous_homing()
        if self.detector.update(self.source.snapshot()) and self.seeking:
            self.capture_continuous_trigger()
            raise ContactDetected('synthetic sustained contact')

    def pump(self):
        self.clock.advance(.002)
        self.joint_received = self.clock()
        self._homing_feedback.append(JointFeedback(self.clock(), self.clock(), self.joints.copy(), np.zeros(6), 0.))
        self.check_interrupted()

    def run_motion(self, motion):
        self.events.append(('send', self.config.cancel_timeout_s))
        self.goal_handle = SimpleNamespace(accepted=True)
        self.goal_future = Future(self.goal_handle)
        self.result_future = Future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED,
                                                    result=SimpleNamespace(error_code=0)))
        self.joints[2] = motion.end[2] if self.mode == 'no-contact' else 100.
        if self.mode == 'contact': self.source.counts = 25000.
        elif self.mode == 'overload': self.source.counts = self.detector.options.maximum + 10000.
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
        self.pump()
        self.check_continuous_homing()
        return self.stationary_homing_pose()

    def stationary_homing_pose(self):
        return dict(nozzle=self.joints[:3].copy(), joints=self.joints.copy(),
                    observed_start=self.clock()-1.4, observed_end=self.clock())

    def homing_load_window(self, settled):
        return self.window()

    def window(self):
        self.events.append(('window', None))
        if self.mode == 'unsettled-contact': self.source.counts = 0.
        self.pump()
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
            return SimpleNamespace(end=end.copy(), times=np.array([0., 50.]))
        patch('print_control.continuous_probe.plan_probe_descent', side_effect=plan).start()

    def run_touch(self): return self.node.homing_touch(floor_z=85., speed_mm_s=10.)

    def test_rejected_full_search_reports_current_range_without_sending_motion(self):
        error = ValueError('A link origin is below min_link_z_mm: DH link 5 (wrist_2_joint) '
                           'has Base Z -20.100 mm, below the configured -20.000 mm limit')
        with patch('print_control.continuous_probe.plan_probe_descent', side_effect=error):
            with self.assertRaises(ValueError) as caught:
                self.node.homing_touch(floor_z=-245., speed_mm_s=5.)
        message = str(caught.exception)
        self.assertIn('current nozzle Base Z 105.000 mm', message)
        self.assertIn('350.000 mm downward (end Z -245.000 mm)', message)
        self.assertIn('DH link 5 (wrist_2_joint)', message)
        self.assertIn('CURRENT nozzle-to-surface gap', message)
        self.assertIn('No motion was sent for this search', message)
        self.assertEqual(self.node.events, [])
        self.assertFalse(self.node.seeking)
        self.assertIs(self.node.config, self.original_config)

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

    def test_failed_stop_preserves_original_fault_in_diagnostic_and_interlock_error(self):
        self.node.mode='sensor-error'
        self.node.stop=Mock(side_effect=RuntimeError('Robot stop unconfirmed'))
        recorded=[]
        self.node.record_contact_failure=lambda error:recorded.append(
            (str(error),dict(self.node._continuous_homing)))
        with self.assertRaisesRegex(RuntimeError,'sensor disconnected; stop failure: Robot stop unconfirmed'):
            self.run_touch()
        self.assertEqual(recorded[0][0],'sensor disconnected')
        self.assertEqual(recorded[0][1]['stop_failure'],'Robot stop unconfirmed')
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

    def test_locator_retains_trigger_after_release_but_never_claims_measurement_quality(self):
        self.node.mode = 'unsettled-contact'
        result = self.node.homing_touch(floor_z=85., speed_mm_s=10., locating_only=True)
        self.assertFalse(result['contact_persistence_checked'])
        self.assertEqual(result['load'], 0.)
        self.assertTrue(any(name == 'cancel' for name, _ in self.node.events))
        self.assertTrue(any(name == 'stationary' for name, _ in self.node.events))

    def test_locator_still_rejects_overload_missing_trigger_and_unsettled_pose(self):
        for mode in ('overload', 'no-contact'):
            with self.subTest(mode=mode):
                self.node = ContactHarness(self.clock)
                self.node.mode = mode
                with self.assertRaises(RuntimeError):
                    self.node.homing_touch(floor_z=85., speed_mm_s=10., locating_only=True)
        self.node = ContactHarness(self.clock)
        self.node.stationarity_error = 'position did not settle'
        with self.assertRaisesRegex(RuntimeError, 'position did not settle'):
            self.node.homing_touch(floor_z=85., speed_mm_s=10., locating_only=True)

    def test_mesh_approach_locator_requires_refinement_when_load_relaxes(self):
        self.node.mode = 'unsettled-contact'
        def approach(**kwargs):
            self.node.run_homing_motion(SimpleNamespace(end=np.array([1.,2.,85.])))
        self.node.run_precision_descent = approach
        result = self.node.homing_touch(floor_z=85., speed_mm_s=.2,
            precision_reference_z=99., allow_approach_locator=True)
        self.assertEqual(result['contact_method'], 'continuous_cancel')
        self.assertFalse(result['contact_persistence_checked'])
        self.assertEqual(result['load'], 0.)

    def test_mesh_precision_landing_never_uses_locator_persistence_exemption(self):
        self.node.mode = 'unsettled-contact'
        def landing(**kwargs):
            self.node._continuous_homing['precision_landing_started'] = True
            self.node.run_homing_motion(SimpleNamespace(end=np.array([1.,2.,85.])))
        self.node.run_precision_descent = landing
        with self.assertRaisesRegex(RuntimeError, 'did not persist'):
            self.node.homing_touch(floor_z=85., speed_mm_s=.2,
                precision_reference_z=99., allow_approach_locator=True)
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
        self.node.stationary_homing_pose = lambda: ContinuousHoming.stationary_homing_pose(self.node)
        self.node._continuous_homing = dict(floor_z=90., stop_travel_mm=0.,
            terminal_confirmed=True, stopped_at=self.clock(), settling_deadline=self.clock()+2.)
        result = self.node.wait_homing_stationary()
        self.assertGreaterEqual(self.clock(), 100.+WINDOW_S-.01)
        self.assertLess(self.clock(), 102.)
        self.assertGreaterEqual(result['sample_count'], int(200*WINDOW_S))

    def use_actual_settling_with_residual_motion(self, *, moving_until=.15, fault=None):
        self.node.wait_homing_stationary = lambda: ContinuousHoming.wait_homing_stationary(self.node)
        self.node.stationary_homing_pose = lambda: ContinuousHoming.stationary_homing_pose(self.node)
        original_pump = self.node.pump
        observed = {}
        def pump():
            context = getattr(self.node, '_continuous_homing', None)
            stopping = context is not None and context.get('terminal_confirmed')
            velocity = 0.
            if stopping:
                observed['stopped_at'] = context['stopped_at']
                elapsed = self.clock()+.002-context['stopped_at']
                if elapsed <= moving_until:
                    velocity = .000379931
                    self.node.joints[0] += velocity*.002
                    observed['last_motion_at'] = self.clock()+.002
                if elapsed >= .9 and fault is not None:
                    fault()
            original_pump()
            if stopping:
                v = np.zeros(6)
                v[0] = velocity
                self.node._homing_feedback[-1] = replace(self.node._homing_feedback[-1], velocity=v)
                self.node.check_interrupted()
        self.node.pump = pump
        return observed

    def test_residual_motion_waits_for_an_entire_clean_window_before_accepting_contact(self):
        observed = self.use_actual_settling_with_residual_motion()
        result = self.run_touch()
        self.assertGreaterEqual(self.clock()-observed['last_motion_at'], WINDOW_S)
        self.assertLess(self.clock()-observed['stopped_at'], 2.)
        names = [e[0] for e in self.node.events]
        self.assertEqual(names.count('send'), 1)
        self.assertEqual(names.count('cancel'), 1)
        self.assertEqual(names.count('window'), 1)
        self.assertEqual(result['load'], 25000.)
        self.assert_cleaned()

    def test_persistent_reported_motion_expires_original_deadline_without_accepting_contact(self):
        observed = self.use_actual_settling_with_residual_motion(moving_until=float('inf'))
        with self.assertRaisesRegex(RuntimeError, 'observation deadline'):
            self.run_touch()
        elapsed = self.clock()-observed['stopped_at']
        self.assertGreaterEqual(elapsed, SETTLING_TIMEOUT_S)
        self.assertLess(elapsed, SETTLING_TIMEOUT_S+.02)
        names = [e[0] for e in self.node.events]
        self.assertEqual(names.count('send'), 1)
        self.assertEqual(names.count('cancel'), 1)
        self.assertNotIn('window', names)
        self.assertEqual(names[-1], 'stop')
        self.assert_cleaned()

    def test_late_servo_deceleration_requires_a_full_clean_window_before_measurement(self):
        observed=self.use_actual_settling_with_residual_motion(moving_until=1.65)
        result=self.run_touch()
        self.assertGreaterEqual(self.clock()-observed['last_motion_at'],WINDOW_S)
        self.assertLess(self.clock()-observed['stopped_at'],SETTLING_TIMEOUT_S)
        self.assertEqual(result['load'],25000.)
        self.assertEqual([e[0] for e in self.node.events].count('send'),1)

    def test_one_delayed_early_sample_requires_a_wholly_fresh_replacement_window(self):
        observed=self.use_actual_settling_with_residual_motion(moving_until=0.)
        original_pump=self.node.pump
        delayed=[]
        def pump():
            original_pump()
            context=getattr(self.node, '_continuous_homing', None)
            if context and context.get('terminal_confirmed'):
                elapsed=self.clock()-context['stopped_at']
                if elapsed >= .1 and not delayed:
                    self.node._homing_feedback[-1]=replace(
                        self.node._homing_feedback[-1], source_age_s=.0224)
                    delayed.append(self.clock())
        self.node.pump=pump
        result=self.run_touch()
        self.assertGreater(self.clock()-delayed[0], WINDOW_S)
        self.assertLess(self.clock()-observed['stopped_at'], SETTLING_TIMEOUT_S)
        self.assertEqual(result['load'],25000.)
        self.assertEqual([e[0] for e in self.node.events].count('send'),1)

    def test_early_raw_excursion_requires_complete_clean_window_without_relaxing_bound(self):
        observed=self.use_actual_settling_with_residual_motion(moving_until=0.)
        original_pump=self.node.pump
        disturbed=[]
        def pump():
            original_pump()
            context=getattr(self.node, '_continuous_homing', None)
            if context and context.get('terminal_confirmed'):
                elapsed=self.clock()-context['stopped_at']
                if elapsed >= .2 and not disturbed:
                    sample=self.node._homing_feedback[-1]
                    joints=sample.joints.copy();joints[2]+=.11
                    self.node._homing_feedback[-1]=replace(sample,joints=joints)
                    disturbed.append(self.clock())
        self.node.pump=pump
        result=self.run_touch()
        self.assertGreater(self.clock()-disturbed[0],WINDOW_S)
        self.assertLess(self.clock()-observed['stopped_at'],SETTLING_TIMEOUT_S)
        self.assertEqual(result['load'],25000.)
        self.assertEqual([e[0] for e in self.node.events].count('send'),1)

    def test_waiting_for_residual_motion_keeps_load_travel_and_sensor_guards_active(self):
        for fault, reason in (('overload', 'overload'), ('travel', 'stop travel'),
                              ('sensor', 'ALPS samples')):
            with self.subTest(fault=fault):
                self.node = ContactHarness(self.clock)
                self.original_config = self.node.config
                def inject():
                    if fault == 'overload':
                        self.node.source.counts = self.node.contact_options.maximum+1.
                    elif fault == 'travel':
                        self.node.joints[2] -= .51
                    else:
                        self.node.source.age = .151
                observed = self.use_actual_settling_with_residual_motion(
                    moving_until=float('inf'), fault=inject)
                with self.assertRaisesRegex(RuntimeError, reason):
                    self.run_touch()
                elapsed = self.clock()-observed['stopped_at']
                self.assertGreaterEqual(elapsed, .9)
                self.assertLess(elapsed, 1.)
                self.assertNotIn('window', [e[0] for e in self.node.events])
                self.assert_cleaned()

    def test_missing_early_feedback_requires_full_replacement_without_accepting_gap(self):
        observed=self.use_actual_settling_with_residual_motion(moving_until=0.)
        original_pump=self.node.pump
        gap=[]
        def pump():
            original_pump()
            context=getattr(self.node, '_continuous_homing', None)
            if context and context.get('terminal_confirmed'):
                if self.clock()-context['stopped_at'] >= .1 and not gap:
                    self.node._homing_feedback=list(self.node._homing_feedback)
                    del self.node._homing_feedback[-14:-1]
                    gap.append(self.clock())
        self.node.pump=pump
        self.run_touch()
        self.assertGreaterEqual(self.clock()-gap[0], WINDOW_S-.01)
        self.assertLess(self.clock()-observed['stopped_at'], SETTLING_TIMEOUT_S)
        self.assertEqual([e[0] for e in self.node.events].count('send'),1)

    def test_buffered_motion_after_first_settled_pose_is_fatal_even_when_latest_is_zero(self):
        self.use_actual_settling_with_residual_motion(moving_until=0.)
        original_window = self.node.window
        def window():
            samples = original_window()
            self.node.pump()
            # A transient in two newly received samples must not disappear just
            # because the latest sample is stationary at the next guard check.
            self.node._homing_feedback[-2] = replace(self.node._homing_feedback[-2],
                velocity=np.array([.000379931, 0., 0., 0., 0., 0.]))
            self.node.check_continuous_homing()
            return samples
        self.node.window = window
        with self.assertRaisesRegex(RuntimeError, '[Jj]oint motion'):
            self.run_touch()
        names = [e[0] for e in self.node.events]
        self.assertEqual(names.count('window'), 1)
        self.assertEqual(names.count('send'), 1)
        self.assertEqual(names[-1], 'stop')
        self.assert_cleaned()

    def use_actual_settling_with_pose_drift(self, *, drifting_until=.6, fault=None):
        """Sub-velocity-limit drift with a realistic millimetre/radian FK scale."""
        self.node.wait_homing_stationary = lambda: ContinuousHoming.wait_homing_stationary(self.node)
        self.node.stationary_homing_pose = lambda: ContinuousHoming.stationary_homing_pose(self.node)
        base = self.node.joints[:3].copy()
        self.node.planner.nozzle_position = lambda q: np.array([
            base[0]+1000.*(q[0]-base[0]), q[1], q[2]])
        original_pump = self.node.pump
        observed = {}
        noise = np.random.default_rng(20261117)
        def pump():
            context = getattr(self.node, '_continuous_homing', None)
            if context is not None and context.get('terminal_confirmed'):
                observed['stopped_at'] = context['stopped_at']
                elapsed = self.clock()+.002-context['stopped_at']
                if elapsed <= drifting_until:
                    self.node.joints[0] += .05/1000.*.002
                if elapsed >= .9 and fault is not None:
                    fault()
            original_pump()  # Deliberately zero reported velocity throughout.
            if context is not None and context.get('terminal_confirmed'):
                # Small independent encoder noise, below the recorded idle
                # noise, separates this gradual-drift case from a raw spike.
                sample = self.node._homing_feedback[-1]
                joints = sample.joints.copy()
                joints[0] += noise.uniform(-8e-6, 8e-6)
                self.node._homing_feedback[-1] = replace(sample, joints=joints)
        self.node.pump = pump
        return observed

    def test_initial_pose_drift_can_settle_without_changing_the_acceptance_bounds(self):
        observed = self.use_actual_settling_with_pose_drift()
        result = self.run_touch()
        elapsed = self.clock()-observed['stopped_at']
        self.assertGreater(elapsed, WINDOW_S)  # The first observation window contains drift.
        self.assertLess(elapsed, 2.)
        self.assertLess(abs(result['nozzle'][0]-1.03), .003)
        names = [e[0] for e in self.node.events]
        self.assertEqual(names.count('send'), 1)
        self.assertEqual(names.count('cancel'), 1)
        self.assertEqual(names.count('window'), 1)
        self.assert_cleaned()

    def test_persistent_pose_drift_expires_without_accepting_contact(self):
        observed = self.use_actual_settling_with_pose_drift(drifting_until=float('inf'))
        with self.assertRaisesRegex(RuntimeError, 'observation deadline.*drift.*measured'):
            self.run_touch()
        self.assertGreaterEqual(self.clock()-observed['stopped_at'], SETTLING_TIMEOUT_S)
        self.assertLess(self.clock()-observed['stopped_at'], SETTLING_TIMEOUT_S+.02)
        self.assertNotIn('window', [e[0] for e in self.node.events])
        self.assert_cleaned()

    def test_initial_drift_wait_preserves_overload_travel_and_freshness_guards(self):
        for fault, reason in (('overload', 'overload'), ('travel', 'stop travel'),
                              ('sensor', 'ALPS samples'), ('joints', 'joint feedback')):
            with self.subTest(fault=fault):
                self.node = ContactHarness(self.clock)
                self.original_config = self.node.config
                def inject():
                    if fault == 'overload':
                        self.node.source.counts = self.node.contact_options.maximum+1.
                    elif fault == 'travel':
                        self.node.joints[2] -= .51
                    elif fault == 'sensor':
                        self.node.source.age = .151
                    else:
                        self.node.fresh = False
                observed = self.use_actual_settling_with_pose_drift(
                    drifting_until=float('inf'), fault=inject)
                with self.assertRaisesRegex(RuntimeError, reason):
                    self.run_touch()
                elapsed = self.clock()-observed['stopped_at']
                self.assertGreaterEqual(elapsed, .9)
                self.assertLess(elapsed, 1.)
                self.assertNotIn('window', [e[0] for e in self.node.events])
                self.assert_cleaned()

    def test_pose_drift_after_first_settled_anchor_is_rejected_without_retry(self):
        self.use_actual_settling_with_pose_drift(drifting_until=0.)
        original_window = self.node.window
        def window():
            samples = original_window()
            for _ in range(200):
                self.node.joints[0] += .11/1000.*.002
                self.node.pump()
            return samples
        self.node.window = window
        with self.assertRaisesRegex(RuntimeError, 'fresh pose interval'):
            self.run_touch()
        self.assertEqual([e[0] for e in self.node.events].count('window'), 1)
        self.assert_cleaned()

    def test_stationarity_rejects_continuously_changing_feedback(self):
        self.node._continuous_homing = dict(floor_z=90., stop_travel_mm=0.,
            terminal_confirmed=True, stopped_at=self.clock(), settling_deadline=self.clock()+2.)
        self.node.stationary_homing_pose = lambda: ContinuousHoming.stationary_homing_pose(self.node)
        def moving():
            self.clock.advance(.002)
            self.node.joint_received = self.clock()
            self.node.joints[2] -= .02
            self.node._homing_feedback.append(JointFeedback(self.clock(), self.clock(), self.node.joints.copy(), np.zeros(6), 0.))
        self.node.pump = moving
        with self.assertRaisesRegex(RuntimeError, 'did not settle'):
            ContinuousHoming.wait_homing_stationary(self.node)

    def test_contact_records_the_final_estimated_joint_pose_not_latest_raw_sample(self):
        count = [0]
        def estimate():
            count[0] += 1
            q = self.node.joints.copy()
            q[2] -= .00005
            return dict(nozzle=q[:3].copy(), joints=q,
                        observed_start=self.clock()-1.4, observed_end=self.clock())
        self.node.stationary_homing_pose = estimate
        result = self.run_touch()
        self.assertEqual(count[0], 1)
        self.assertAlmostEqual(result['nozzle'][2], 99.99995)
        self.assertEqual(self.node.joints[2], 100.)
        np.testing.assert_array_equal(self.node.seed, result['joints'])
        np.testing.assert_array_equal(self.node.target, result['nozzle'])

    def test_one_accepted_pose_is_paired_without_a_second_sliding_estimate(self):
        estimate = self.node.stationary_homing_pose
        count = [0]
        def once():
            count[0] += 1
            if count[0] > 1:
                raise AssertionError('No second shifted window may replace the accepted pose')
            return estimate()
        self.node.stationary_homing_pose = once
        self.run_touch()
        names = [event[0] for event in self.node.events]
        self.assertEqual(names.count('send'), 1)
        self.assertEqual(names.count('cancel'), 1)
        self.assertEqual(names.count('window'), 1)
        self.assertEqual(count[0], 1)

    def test_estimator_computation_handoff_requires_new_source_fresh_feedback(self):
        estimate=self.node.stationary_homing_pose
        def computing():
            result=estimate()
            self.node._homing_feedback[-1]=replace(self.node._homing_feedback[-1],source_age_s=.005)
            self.clock.advance(.016)
            return result
        self.node.stationary_homing_pose=computing
        self.assertEqual(self.run_touch()['load'],25000.)
        self.assertLess(self.clock()-self.node.joint_received,.02)

    def test_computation_gap_or_motion_cannot_be_hidden_by_newer_feedback(self):
        for fault,reason in (('gap','feedback gap'),('motion','settled contact pose')):
            with self.subTest(fault=fault):
                self.node=ContactHarness(self.clock)
                self.original_config=self.node.config
                estimate=self.node.stationary_homing_pose
                def computing():
                    result=estimate()
                    if fault=='gap':self.clock.advance(.025)
                    else:self.node.joints[0]+=.0002
                    return result
                self.node.stationary_homing_pose=computing
                with self.assertRaisesRegex(RuntimeError,reason):self.run_touch()
                self.assert_cleaned()

    def test_callback_delay_keeps_fresh_contiguous_source_samples_valid(self):
        estimate=self.node.stationary_homing_pose
        original_pump=self.node.pump
        pending=[]
        def computing():
            result=estimate()
            self.clock.advance(.020)
            pending.append(True)
            return result
        def pump():
            original_pump()
            if pending:
                pending.pop()
                sample=self.node._homing_feedback[-1]
                self.node._homing_feedback[-1]=replace(sample,
                    source_time=sample.source_time-.010,source_age_s=.010)
        self.node.stationary_homing_pose=computing
        self.node.pump=pump
        result=self.run_touch()
        self.assertTrue(result['contact_persistence_checked'])
        self.assertEqual(result['load'],25000.)
        self.assertEqual([e[0] for e in self.node.events].count('send'),1)

    def test_callback_delivery_cannot_hide_an_old_source_sample(self):
        original_pump=self.node.pump
        estimate=self.node.stationary_homing_pose
        pending=[]
        def computing():
            result=estimate();pending.append(True);return result
        def pump():
            original_pump()
            if pending:
                pending.pop()
                sample=self.node._homing_feedback[-1]
                self.node._homing_feedback[-1]=replace(sample,source_age_s=.021)
        self.node.stationary_homing_pose=computing
        self.node.pump=pump
        with self.assertRaisesRegex(RuntimeError,'feedback gap'):
            self.run_touch()

    def test_force_snapshot_delay_rejects_contact_without_moving_anchor(self):
        original_window = self.node.window
        def delayed():
            self.clock.advance(.025)
            return original_window()
        self.node.window = delayed
        with self.assertRaisesRegex(RuntimeError, 'fresh pose interval'):
            self.run_touch()
        self.assertEqual([e[0] for e in self.node.events].count('send'), 1)
        self.assert_cleaned()

    def test_motion_or_stale_feedback_during_load_window_is_fatal(self):
        for fault, reason in (('velocity', 'joint motion'), ('drift', 'settled contact pose'),
                              ('stale', 'became stale')):
            with self.subTest(fault=fault):
                self.node = ContactHarness(self.clock)
                self.original_config = self.node.config
                original_window = self.node.window
                def window():
                    samples = original_window()
                    if fault == 'velocity':
                        self.node._homing_feedback[-1] = replace(self.node._homing_feedback[-1], velocity=np.ones(6)*.001)
                    elif fault == 'drift':
                        self.node.joints[0] += .0002
                    else:
                        self.clock.advance(.021)
                    self.node.check_continuous_homing()
                    return samples
                self.node.window = window
                with self.assertRaisesRegex(RuntimeError, reason): self.run_touch()
                self.assert_cleaned()

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


class MatchedLoadWindowTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.node = ContactHarness(self.clock)
        self.node._continuous_homing = {'stopped_at': 98.}
        self.settled = {'observed_start': 98.6, 'observed_end': 100.}
        # A complete 488 Hz force window delivered 40 ms behind joint feedback.
        self.samples = [ForceSample(i, 99.76+i*.002, 49.76+i*.002, 25000.)
                        for i in range(101)]
        self.node.source.snapshot = lambda: self.samples

    def window(self):
        return ContinuousHoming.homing_load_window(self.node, self.settled)

    def test_pairs_only_fresh_force_data_within_the_accepted_pose_interval(self):
        selected = self.window()
        self.assertGreaterEqual(len(selected), 49)
        self.assertLessEqual(len(selected), 51)
        self.assertTrue(all(self.settled['observed_start'] <= s.observed_at <=
                            self.settled['observed_end'] for s in selected))
        self.assertEqual(self.clock(), 100.)  # No extra wait or motion.
        self.assertEqual(selected[-1], self.samples[-1])

    def test_pre_stop_stale_short_or_outside_pose_windows_are_rejected(self):
        original = self.samples
        for mode in ('pre-stop', 'stale', 'short', 'outside-pose'):
            with self.subTest(mode=mode):
                self.samples = original
                self.node._continuous_homing['stopped_at'] = 98.
                self.settled['observed_start'] = 98.6
                if mode == 'pre-stop': self.node._continuous_homing['stopped_at'] = 99.9
                if mode == 'stale': self.samples = [replace(s, observed_at=s.observed_at-.2) for s in original]
                if mode == 'short': self.samples = original[-5:]
                if mode == 'outside-pose': self.settled['observed_start'] = 99.99
                with self.assertRaisesRegex(RuntimeError, '(fresh|Incomplete) ALPS window'):
                    self.window()

    def test_duplicate_source_timestamps_are_rejected(self):
        self.samples[-2] = replace(self.samples[-2], source_time=self.samples[-1].source_time)
        with self.assertRaisesRegex(RuntimeError, 'Invalid ALPS timestamps'):
            self.window()

    def test_new_force_data_after_pose_is_not_paired_with_an_older_pose(self):
        future = [replace(s, sequence=s.sequence+200, observed_at=s.observed_at+.3,
                          source_time=s.source_time+.3) for s in self.samples]
        self.samples += future
        self.assertLessEqual(self.window()[-1].observed_at, self.settled['observed_end'])


class FeedbackCaptureTests(unittest.TestCase):
    def test_callback_keeps_velocity_and_source_age_with_accepted_position(self):
        node = object.__new__(BridgeBedProbe)
        node.joints, node.joint_received = np.zeros(6), None
        node.config = SimpleNamespace(feedback_timeout_s=.25)
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000))
        def message(*, stamp=100_000_000_000, positions=None, velocity=None):
            sec, nanosec = divmod(stamp, 1_000_000_000)
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
                name=list(reversed(JOINT_NAMES)), position=list(reversed([1.]*6 if positions is None else positions)),
                velocity=list(reversed([1e-5]*6 if velocity is None else velocity)))
        with patch('print_control.motion_executor.time.monotonic', return_value=10.):
            node.on_joints(message(stamp=99_900_000_000))
            sample = node._homing_feedback[-1]
            self.assertEqual(sample.received_at, node.joint_received)
            self.assertAlmostEqual(sample.source_age_s, .1)
            self.assertAlmostEqual(sample.source_time, 99.9)
            np.testing.assert_array_equal(sample.joints, node.joints)
            np.testing.assert_array_equal(sample.velocity, np.full(6, 1e-5))
            # Base executor rejection must also reject its accompanying speed.
            node.on_joints(message(positions=[float('nan')]*6, velocity=[1.]*6))
            node.on_joints(message(stamp=99_000_000_000, velocity=[1.]*6))
            self.assertEqual(len(node._homing_feedback), 1)
            node.on_joints(message(velocity=[]))
            self.assertTrue(np.all(np.isnan(node._homing_feedback[-1].velocity)))


if __name__ == '__main__': unittest.main()
