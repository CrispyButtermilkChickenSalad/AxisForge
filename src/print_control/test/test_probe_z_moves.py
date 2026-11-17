"""Native dashboard Z controls using simulated feedback and no hardware."""
import io
from types import SimpleNamespace
from dataclasses import replace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from print_control.probe_z_moves import ProbeZMoves
from print_control.motion_planner import MotionConfig
from print_control.probe_control import ProbeControl, Z_STEP_SIZES
from print_control.load_calibration_control import LoadCalibrationControl
from print_control.load_cell_contact import ContactDetector, ContactOptions, ContactDetected, ContactMeasurementRejected, LoadCellOverload, ForceSample
from print_control.load_calibration_policy import CalibrationDetector
from print_control.probe_settling import JointFeedback


class Source:
    def __init__(self):
        self.value = 0.
        self.sequence = 0
        self.sensor = {'counts_per_gram': None}
        self.refresh_status = Mock()
    def clock(self): return 100.
    def snapshot(self):
        self.sequence += 60
        return [ForceSample(self.sequence+i, 99.9+i*.002, 99.9+i*.002, self.value) for i in range(51)]


class Harness(ProbeZMoves):
    def __init__(self):
        self.config = MotionConfig(dry_run=True)
        self.homing = SimpleNamespace(sample_age_s=.15, max_descent_mm=100., first_speed_mm_s=1., lift_speed_mm_s=10., max_stop_travel_mm=.5, cancel_timeout_s=.2)
        self.contact_options = ContactOptions(trigger_mode='raw')
        self.source = Source()
        self.detector = ContactDetector(0., None, self.contact_options)
        self.target = np.array([1., 2., 100.])
        self.seed = self.joints = np.array([1., 2., 100., 0., 0., 0.])
        self.start_xyz = self.target.copy()
        self.start_joints = self.joints.copy()
        self._nominal_probe_rotation = np.eye(3)
        self.planner = SimpleNamespace(check_pose=Mock(), nozzle_position=lambda q: q[:3].copy(),
            sampled_line=lambda name, start, end, q, speed, **kwargs: SimpleNamespace(end=end.copy(), speed=speed))
        self.control = SimpleNamespace(check=Mock(), emit=Mock())
        self.moves = []
        self.stop_unconfirmed = False
        self.seeking = False
        self.freeze_probe_orientation = Mock()
        self.on_move = None
        self.confirm_motion_stopped = Mock()
        self._z_phase, self._z_failure = 'positioning', ''
        self._z_lower, self._z_upper = 0., 200.
    def check_interrupted(self):
        self.control.check()
        contact = self.detector.update(self.source.snapshot())
        if self.seeking and contact: raise ContactDetected('contact')
    def pump(self): self.check_interrupted()
    def nozzle_now(self): return self.joints[:3].copy(), self.joints.copy()
    def window(self):
        self.check_interrupted()
        return self.source.snapshot()
    def run_motion(self, motion):
        self.moves.append(motion)
        self.joints = np.r_[motion.end, [0., 0., 0.]]
        if self.on_move: self.on_move()
    def cancel_contact(self):
        self.confirm_motion_stopped()
        self.seeking = False
        self.target, self.seed = self.nozzle_now()


class ZMoveTests(unittest.TestCase):
    def test_fine_up_step_uses_stopped_median_not_single_raw_encoder_extreme(self):
        node=Harness()
        median=np.array([1.,2.,100.,0.,0.,0.])
        node.joints=median.copy();node.joints[2]-=.06
        node.z_waiting_anchor=Mock(return_value=(median[:3].copy(),median.copy()))
        self.assertEqual(node.manual_z_step(.05),'')
        np.testing.assert_allclose(node.moves[0].end,[1.,2.,100.05])
        node.z_waiting_anchor.assert_called_once()

    def test_requested_steps_are_vertical_and_reanchor_measured_pose(self):
        for delta in (-10., -1., -.1, -.05, .05, .1, 1., 10.):
            with self.subTest(delta=delta):
                node=Harness()
                node.target += 10.  # Deliberately obsolete command target.
                self.assertEqual(node.manual_z_step(delta), '')
                np.testing.assert_array_equal(node.moves[0].end, [1., 2., 100.+delta])
                self.assertEqual(node.moves[0].speed, 1. if delta<0 else 10.)
                node.freeze_probe_orientation.assert_called_once()

    def test_every_downward_size_stops_on_contact_without_taring(self):
        for delta in (-10., -1., -.1, -.05):
            node=Harness()
            def contact():
                node.source.value=25000.
                node.check_interrupted()
            node.on_move=contact
            self.assertIn('contact', node.manual_z_step(delta))
            node.confirm_motion_stopped.assert_called_once()
            self.assertEqual(node.detector.baseline, 0.)
            self.assertTrue(node.manual_z_values()['upward_only'])
            self.assertIn('Only Up',node.manual_z_step(-.05))
            self.assertFalse(node.seeking)

    def test_invalid_range_and_plan_do_not_send_motion_or_close_session(self):
        node=Harness()
        node._z_upper=105.
        self.assertIn('range',node.manual_z_step(10.))
        node.planner.sampled_line=Mock(side_effect=ValueError('link clearance'))
        self.assertIn('not sent',node.manual_z_step(.05))
        self.assertFalse(node.moves)

    def test_load_change_during_planning_rejects_down_or_fast_up_before_sending(self):
        for delta in (-.05, 1.):
            node=Harness()
            plan=node.planner.sampled_line
            def changing(*args, **kwargs):
                node.source.value=25000.
                return plan(*args, **kwargs)
            node.planner.sampled_line=changing
            self.assertIn('Load changed',node.manual_z_step(delta))
            self.assertFalse(node.moves)
            self.assertEqual(node.detector.baseline,0.)

    def test_residual_velocity_waits_without_relaxing_threshold(self):
        node=Harness();node.config=replace(node.config,dry_run=False)
        now=[100.];node.joint_received=now[0];node.joints_fresh=lambda:True
        node._homing_feedback=[JointFeedback(now[0],now[0],node.joints.copy(),np.full(6,.000379931),0.)]
        def pump():
            now[0]+=.002;node.joint_received=now[0]
            speed=.000379931 if now[0]<100.02 else 0.
            node._homing_feedback.append(JointFeedback(now[0],now[0],node.joints.copy(),np.full(6,speed),0.))
        node.pump=pump
        with patch('print_control.probe_z_moves.time.monotonic',lambda:now[0]):
            node._z_require_stationary()
        self.assertGreaterEqual(now[0],100.02)
        self.assertFalse(node.moves)

    def test_aged_pre_positioning_sample_waits_for_new_feedback_without_moving(self):
        for deliver in (True, False):
            node=Harness();node.config=replace(node.config,dry_run=False)
            now=[100.];node.joint_received=99.97;node.joints_fresh=lambda:True
            node._homing_feedback=[JointFeedback(99.97,99.97,node.joints.copy(),np.zeros(6),0.)]
            def pump():
                now[0]+=.002
                if deliver and now[0]>=100.01:
                    node.joint_received=now[0]
                    node._homing_feedback.append(JointFeedback(now[0],now[0],node.joints.copy(),np.zeros(6),0.))
            node.pump=pump
            with patch('print_control.probe_z_moves.time.monotonic',lambda:now[0]):
                if deliver:
                    node._z_require_stationary()
                    self.assertGreaterEqual(now[0],100.01)
                else:
                    with self.assertRaisesRegex(RuntimeError,'fresh finite joint velocities'):
                        node._z_require_stationary()
                    self.assertGreaterEqual(now[0],100.25)
            self.assertFalse(node.moves)

    def test_idle_reference_uses_fresh_median_but_keeps_raw_excursion_bound(self):
        for noise_mm, accepted in ((.06,True),(.12,False)):
            node=Harness();node.config=replace(node.config,dry_run=False)
            now=[100.];node._homing_feedback=[]
            base=node.joints.copy();node.planner.nozzle_position=lambda q:(q-base)[:3]*1000.
            def pump():
                now[0]+=.002
                q=base.copy();q[0]+=(-1 if len(node._homing_feedback)%2 else 1)*noise_mm/1000.
                node._homing_feedback.append(JointFeedback(now[0],now[0],q,np.zeros(6),0.))
            node.pump=pump
            with patch('print_control.probe_z_moves.time.monotonic',lambda:now[0]):
                if accepted:
                    xyz,q=node.z_waiting_anchor()
                    np.testing.assert_allclose(xyz,np.zeros(3),atol=1e-9)
                else:
                    with self.assertRaisesRegex(RuntimeError,'Arm moved'):
                        node.z_waiting_anchor()
            self.assertFalse(node.moves)

    def test_idle_reference_waits_for_residual_motion_but_never_accepts_moving_window(self):
        for stops in (True,False):
            node=Harness();node.config=replace(node.config,dry_run=False)
            now=[100.];node._homing_feedback=[]
            def pump():
                now[0]+=.002
                speed=.0003 if not stops or now[0]<100.08 else 0.
                node._homing_feedback.append(JointFeedback(now[0],now[0],node.joints.copy(),np.full(6,speed),0.))
            node.pump=pump
            with patch('print_control.probe_z_moves.time.monotonic',lambda:now[0]):
                if stops:
                    node.z_waiting_anchor()
                    self.assertGreaterEqual(now[0],100.22)
                else:
                    with self.assertRaisesRegex(RuntimeError,'Timed out'):
                        node.z_waiting_anchor()
                    self.assertGreaterEqual(now[0],102.)
            self.assertFalse(node.moves)

    def test_idle_reference_replaces_delayed_window_without_accepting_stale_samples(self):
        for persistent in (False, True):
            node=Harness();node.config=replace(node.config,dry_run=False)
            now=[100.];node._homing_feedback=[]
            def pump():
                now[0]+=.002
                age=.025 if persistent or now[0]<100.08 else 0.
                node._homing_feedback.append(JointFeedback(
                    now[0],now[0],node.joints.copy(),np.zeros(6),age))
            node.pump=pump
            with patch('print_control.probe_z_moves.time.monotonic',lambda:now[0]):
                if persistent:
                    with self.assertRaisesRegex(RuntimeError,'Timed out'):
                        node.z_waiting_anchor()
                    self.assertGreaterEqual(now[0],102.)
                else:
                    node.z_waiting_anchor()
                    self.assertGreater(now[0],100.2)
            self.assertFalse(node.moves)

    def test_original_contact_anchor_limits_total_post_stop_travel(self):
        node=Harness();node.config=replace(node.config,dry_run=False)
        now=[100.];node.joint_received=now[0];node.joints_fresh=lambda:True
        node._z_stop_anchor=node.joints[:3].copy()
        node.joints[2]+=.4
        node._homing_feedback=[JointFeedback(now[0],now[0],node.joints.copy(),np.full(6,.000379931),0.)]
        def pump():
            now[0]+=.002;node.joint_received=now[0];node.joints[2]+=.15
            node._homing_feedback.append(JointFeedback(now[0],now[0],node.joints.copy(),np.zeros(6),0.))
        node.pump=pump
        with patch('print_control.probe_z_moves.time.monotonic',lambda:now[0]):
            with self.assertRaisesRegex(RuntimeError,'stop exceeded'):
                node._z_require_stationary()

    def test_recovery_preserves_baseline_and_rearms_ordinary_overload_after_release(self):
        node=Harness(); node.source.value=130000.
        node._z_prepare_recovery()
        node._z_phase='recovery'
        self.assertIsInstance(node.detector,CalibrationDetector)
        self.assertEqual(node.detector.baseline,0.)
        self.assertEqual(node._z_release_ceiling,135000.)
        self.assertIn('Only Up',node.manual_z_step(-.05))
        self.assertIn('0.1',node.manual_z_step(1.))
        node.manual_z_step(.05)
        self.assertEqual(node.moves[-1].speed,.1)
        node.source.value=0.
        values=node.manual_z_values()
        self.assertTrue(values['can_finish'])
        self.assertTrue(values['upward_only'])
        self.assertIs(type(node.detector),ContactDetector)
        self.assertEqual(node.detector.baseline,0.)
        node.source.value=120001.
        with self.assertRaises(LoadCellOverload): node.check_interrupted()

    def test_recovery_load_rise_stops_step_and_requires_fresh_clearance_before_retry(self):
        for delta in (.05, .1):
            node=Harness();node.source.value=130000.;node._z_prepare_recovery()
            node._z_phase='recovery'
            def rise():
                node.source.value=136000.
                node.check_interrupted()
            node.on_move=rise
            result=node.manual_z_step(delta)
            self.assertIn('Up step stopped',result)
            self.assertEqual(node.confirm_motion_stopped.call_count,2)
            self.assertTrue(node.manual_z_values()['release_blocked'])
            self.assertEqual(node._z_release_ceiling,135000.)
            self.assertEqual(node.detector.baseline,0.)
            self.assertIn('Load rose',node.manual_z_step(.05))
            self.assertEqual(len(node.moves),1)
            node.source.value=131000.
            node.on_move=None
            self.assertFalse(node.manual_z_values()['release_blocked'])
            self.assertEqual(len(node.moves),1, 'Guard clearing must not automatically retry')
            self.assertEqual(node.manual_z_step(.05),'')
            self.assertEqual(len(node.moves),2)

    def test_idle_load_rise_blocks_recovery_steps_without_killing_session(self):
        node=Harness();node.source.value=130000.;node._z_prepare_recovery()
        node._z_phase='recovery'
        node.source.value=136000.
        node.check_interrupted()
        self.assertTrue(node.manual_z_values()['release_blocked'])
        self.assertIn('Load rose',node.manual_z_step(.05))
        self.assertFalse(node.moves)

    def test_release_stop_failure_and_hard_ceiling_remain_fatal(self):
        node=Harness();node.source.value=130000.;node._z_prepare_recovery()
        def rise():
            node.source.value=136000.;node.check_interrupted()
        node.on_move=rise
        node.confirm_motion_stopped.side_effect=RuntimeError('stop unconfirmed')
        with self.assertRaisesRegex(RuntimeError,'stop unconfirmed'):node.manual_z_step(.05)
        node=Harness();node.source.value=130000.;node._z_prepare_recovery()
        node.source.value=250000.
        with self.assertRaisesRegex(RuntimeError,'hard load'):node.check_interrupted()
        node=Harness();node.source.value=250000.
        with self.assertRaisesRegex(RuntimeError,'hard load'):node._z_prepare_recovery()

    def test_unconfirmed_stop_abort_and_sensor_failure_never_enable_recovery(self):
        for fault in ('stop','abort','sensor'):
            node=Harness()
            if fault=='stop':node.confirm_motion_stopped.side_effect=RuntimeError('unconfirmed')
            elif fault=='abort':node.control.check.side_effect=RuntimeError('cancelled')
            else:node.source.snapshot=Mock(side_effect=RuntimeError('sensor stale'))
            with self.assertRaises(RuntimeError):node._z_prepare_recovery()
            self.assertNotIsInstance(node.detector,CalibrationDetector)
            self.assertFalse(node.moves)

    def test_recovery_uses_supervised_health_snapshot_without_blocking_http(self):
        node=Harness();node.source.value=25000.
        node.source.refresh_status.side_effect=AssertionError('Must not block ROS on HTTP')
        node._z_prepare_recovery()
        node.source.refresh_status.assert_not_called()
        self.assertIsInstance(node.detector,CalibrationDetector)
        self.assertEqual(node.detector.baseline,0.)

    def test_continue_recaptures_the_adjusted_search_and_return_origin(self):
        node=Harness()
        actions=iter([{'command':'step','delta_mm':-10.},{'command':'continue'}])
        node.control.wait_z_action=lambda *args:next(actions)
        node.manual_z_session()
        np.testing.assert_array_equal(node.start_xyz,[1.,2.,90.])
        np.testing.assert_array_equal(node.start_joints,node.joints)
        self.assertEqual(node.start_xyz[2]-node.homing.max_descent_mm,-10.)
        self.assertEqual(node.start_xyz[2]-20.,70.)

    def test_only_typed_contact_failures_offer_recovery_and_finish_does_not_resume(self):
        for error, expected in ((ContactMeasurementRejected('contact rejected'),True),
                                (LoadCellOverload(120001.,120000.,'counts'),True),
                                (RuntimeError('sensor failed'),False)):
            node=Harness();node._run_routine=Mock(side_effect=error)
            node.manual_z_session=Mock()
            with self.assertRaises(type(error)):node.run_routine()
            self.assertEqual(node.manual_z_session.called,expected)
            node._run_routine.assert_called_once()


class ZControlTests(unittest.TestCase):
    def test_single_use_phase_scoped_tokens_work_for_both_routine_controls(self):
        for cls in (ProbeControl,LoadCalibrationControl):
            for phase,finish in (('positioning','continue'),('recovery','finish_recovery')):
                control=cls(io.StringIO(),io.StringIO(),clock=lambda:100.)
                seen=[]
                def pump():
                    token=control.action_token
                    self.assertFalse(control.ingest({'command':finish,'token':'stale'}))
                    command={'command':finish,'token':token}
                    self.assertTrue(control.ingest(command))
                    self.assertFalse(control.ingest(command))
                    seen.append(command)
                self.assertEqual(control.wait_z_action(phase,pump,lambda:{}),seen[0])
                self.assertIsNone(control.action_token)
                self.assertIsNone(control.z_mode)

    def test_expired_token_and_queued_step_are_rejected(self):
        control=ProbeControl(io.StringIO(),io.StringIO(),clock=lambda:100.)
        control.z_mode='positioning';control.action_token='a'*32;control.action_deadline=100.
        self.assertFalse(control.ingest({'command':'step','delta_mm':1.,'token':'a'*32}))
        control.action_deadline=110.
        command={'command':'step','delta_mm':.05,'token':'a'*32}
        self.assertTrue(control.ingest(command));self.assertFalse(control.ingest(command))


if __name__=='__main__':unittest.main()
