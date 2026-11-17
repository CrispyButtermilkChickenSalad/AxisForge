"""Exercise scale orchestration and save handoff without hardware or calibration writes."""
import io
import sys
import tempfile
import unittest
from unittest.mock import patch, Mock
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from print_control.alps_load_calibration import LoadCalibration, sensitive_options
from print_control.load_calibration_control import LoadCalibrationControl
from print_control.load_cell_contact import ContactOptions, ContactDetected, LoadCellOverload, ForceSample, thermal_contact_options, stable_baseline
from print_control.load_calibration_policy import CalibrationPolicy
from print_control.motion_planner import JOINT_NAMES
from test_bridge_bed_probe import GuidedProbeHarness


class ScaleHarness(LoadCalibration, GuidedProbeHarness):
    def __init__(self, output):
        GuidedProbeHarness.__init__(self, output)
        self.control = LoadCalibrationControl(io.StringIO(),io.StringIO(),clock=lambda:100.)
        self.actions = [10., -10., 1., -1., .1, -.1]+[-.5]*9+[-.05]*9+['record',.5,'finish']
        self.capture_confirmations = None

    def capture_start(self):
        self.capture_confirmations = list(self.confirmed_stages)
        GuidedProbeHarness.capture_start(self)

    def pump(self):
        GuidedProbeHarness.pump(self)
        if self.control.action_token:
            action=self.actions.pop(0)
            self.control.ingest(dict(command=action if isinstance(action,str) else 'step',
                                     token=self.control.action_token, delta_mm=action, grams=150.))


class ScaleRoutineTests(unittest.TestCase):
    def test_sequence_is_vertical_and_calibration_and_mesh_are_not_written(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'bed.yaml';path.write_text('original mesh')
            probe=ScaleHarness(path)
            probe.run_routine()
            self.assertEqual(probe.confirmed_stages,['home','scale','tare'])
            self.assertEqual(probe.capture_confirmations,['home'])
            self.assertEqual(probe.touches[0]['confirmed'],['home','scale'])
            self.assertEqual(probe.touches[0]['speed'],.1)
            retract=next(m for m in probe.moves if m['name']=='contact retract')
            self.assertAlmostEqual(retract['to'][2]-retract['from'][2],5.)
            for move in probe.moves:
                np.testing.assert_equal(move['from'][:2],move['to'][:2])
                if move['name']=='scale baby step': self.assertIn('tare',move['confirmed'])
            self.assertFalse(probe.saved)
            self.assertEqual(path.read_text(),'original mesh')
            self.assertFalse(probe.source.sensor['is_calibrated'])
            self.assertEqual(probe.contact_options.max_delta_counts, 120000.)
            self.assertTrue(probe.load_status()['calibration_recorded'])
            self.assertTrue(probe.load_status()['unloaded'])

    def test_each_confirmation_blocks_following_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            for stage in ('home','scale','tare'):
                probe=ScaleHarness(Path(directory)/'unused')
                probe.reject_stage=stage
                with self.assertRaisesRegex(RuntimeError,'declined '+stage):probe.run_routine()
                if stage in ('home','scale'):self.assertFalse(probe.moves)
                else:self.assertFalse(any(m['name']=='scale baby step' for m in probe.moves))

    def test_step_limits_prevent_coarse_contact_or_unbounded_travel(self):
        probe=object.__new__(LoadCalibration)
        probe.contact_z=100.;probe.upper_z=105.;probe.lower_z=99.75
        probe.target=np.array([1.,2.,100.8])
        self.assertIn('0.1',probe.step_error(-.5))
        self.assertEqual(probe.step_error(-.05),'')
        probe.target[2]=99.76
        self.assertIn('travel limit',probe.step_error(-.05))
        probe.target[2]=105.
        self.assertIn('travel limit',probe.step_error(.001))
        for bad in (float('nan'),True,0.,-2.):self.assertTrue(probe.step_error(bad))

    def test_requested_steps_work_in_both_directions_with_known_clearance(self):
        probe=object.__new__(LoadCalibration)
        probe.contact_z=100.;probe.upper_z=125.;probe.lower_z=99.75
        probe.target=np.array([1.,2.,112.])
        for step in (10.,1.,.1,.05):
            for sign in (-1,1):self.assertEqual(probe.step_error(sign*step),'')
        probe.target[2]=100.1
        for step in (10.,1.):self.assertTrue(probe.step_error(-step))
        self.assertEqual(probe.step_error(-.1),'')
        self.assertEqual(probe.step_error(-.05),'')

    def test_point_one_step_builds_load_slowly_without_coarse_contact_rearm(self):
        probe=self.limited_probe()
        probe.source.snapshot=lambda:self.samples(90000.)
        probe.move_z=Mock()
        probe.detector.triggered=True
        detector=probe.detector
        probe.calibration_step(-.1)
        self.assertIs(probe.detector,detector)
        self.assertTrue(probe.detector.triggered)
        self.assertFalse(probe.seeking)
        self.assertEqual(probe.move_z.call_args.args[1:],('scale baby step',.1))

    def test_coarse_down_rearms_contact_at_original_unloaded_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            probe=ScaleHarness(Path(directory)/'unused')
            probe.capture_start();probe.contact_z=100.;probe.upper_z=125.;probe.lower_z=99.75
            probe.target[2]=115.;probe.set_baseline();baseline=probe.detector.baseline
            probe.detector.triggered=True
            probe.calibration_step(-10.)
            self.assertEqual(probe.target[2],105.)
            self.assertFalse(probe.detector.triggered)
            self.assertEqual(probe.detector.baseline,baseline)
            self.assertFalse(probe.seeking)
            self.assertEqual(probe.moves[-1]['speed'],1.)

    def test_coarse_down_rejects_load_and_stops_on_unexpected_contact(self):
        with tempfile.TemporaryDirectory() as directory:
            probe=ScaleHarness(Path(directory)/'unused');probe.capture_start()
            probe.contact_z=100.;probe.upper_z=125.;probe.lower_z=99.75
            probe.target[2]=115.;probe.set_baseline()
            with patch.object(probe.detector,'load',return_value=probe.contact_options.threshold):
                with self.assertRaisesRegex(RuntimeError,'Unload'):probe.calibration_step(-10.)
            self.assertFalse(probe.moves)
            with patch.object(probe,'move_z',side_effect=ContactDetected('contact')),patch.object(probe,'cancel_contact') as cancel:
                with self.assertRaisesRegex(RuntimeError,'Unexpected contact'):probe.calibration_step(-10.)
                cancel.assert_called_once()
            self.assertFalse(probe.seeking)

    def test_raw_trigger_adapts_to_noise_without_weakening_noise_guard(self):
        options=ContactOptions(trigger_mode='raw')
        for noise, expected in ((1000.,8000.),(4000.,16400.)):
            adjusted,_=sensitive_options(options,[SimpleNamespace(counts=v) for v in (0.,noise)])
            self.assertEqual(adjusted.threshold,expected)
            self.assertGreaterEqual(adjusted.threshold*.25,noise)
        with self.assertRaisesRegex(ValueError,'noise is too high'):
            sensitive_options(options,[SimpleNamespace(counts=v) for v in (0.,6000.)])

    def test_hot_scale_search_uses_hot_noise_margin_with_unchanged_threshold_ceiling(self):
        options=thermal_contact_options(ContactOptions(trigger_mode='raw'),hot_nozzle=True)
        samples=[SimpleNamespace(counts=v) for v in (-3500.,3500.)]*20
        adjusted,noise=sensitive_options(options,samples)
        self.assertLessEqual(adjusted.threshold,20000.)
        self.assertGreaterEqual(adjusted.baseline_noise_limit,noise)
        self.assertEqual(adjusted.maximum,120000.)
        self.assertEqual(stable_baseline(samples,{},adjusted),0.)
        with self.assertRaisesRegex(ValueError,'noise is too high'):
            sensitive_options(options,[SimpleNamespace(counts=v) for v in (-5000.,5000.)])

    def limited_probe(self):
        probe = object.__new__(LoadCalibration)
        probe.contact_options = ContactOptions(trigger_mode='raw', max_delta_counts=120000.)
        probe.contact_z, probe.lower_z, probe.upper_z = 100., 99.75, 125.
        probe.target = np.array([1., 2., 100.1]); probe.seed = np.zeros(6)
        probe.reference_counts = 0.
        probe.load_limit_reached = False
        probe._manual_loading = True
        probe._calibration_step_active = probe._release_step_active = False
        probe._load_limit_stop_confirmed = True
        probe._release_ceiling = None
        probe.source = SimpleNamespace(clock=lambda: 100., sensor={'counts_per_gram': None})
        probe.detector = probe._manual_detector(0.)
        probe.control = SimpleNamespace(emit=Mock())
        probe.check_interrupted = Mock()
        return probe

    @staticmethod
    def samples(counts, sequence=1, observed=100.):
        return [ForceSample(sequence+i, observed+i*.002, observed+i*.002, float(counts)) for i in range(61)]

    def test_release_only_uses_fixed_ceiling_and_never_retakes_loaded_baseline(self):
        probe = self.limited_probe()
        probe.detector.update(self.samples(130000.))
        self.assertTrue(probe.load_limit_reached)
        self.assertEqual(probe._release_ceiling, 135000.)
        for delta in (-.001, -.05, .5, 1., 10.):
            self.assertIn('only Up', probe.step_error(delta))
        for delta in (.001, .01, .05, .1):
            self.assertEqual(probe.step_error(delta), '')
        probe.source.snapshot = lambda: self.samples(134000., sequence=100)
        probe.move_z = Mock(side_effect=lambda height, *_: probe.target.__setitem__(2, height))
        probe.calibration_step(.1)
        probe.move_z.assert_called_once()
        self.assertAlmostEqual(probe.move_z.call_args.args[0], 100.2)
        self.assertEqual(probe.move_z.call_args.args[1:], ('scale baby step', .1))
        self.assertEqual(probe._release_ceiling, 135000.)
        self.assertEqual(probe.detector.baseline, 0.)
        with self.assertRaisesRegex(RuntimeError, 'Load increased'):
            probe.detector.update(self.samples(136000., sequence=200))

    def test_manual_overload_cancels_before_release_and_cancel_failure_remains_fatal(self):
        for stop_fails in (False, True):
            with self.subTest(stop_fails=stop_fails):
                probe = self.limited_probe()
                batch = self.samples(130000.)
                probe.source.snapshot = lambda: batch
                probe.move_z = Mock(side_effect=lambda *_: probe.detector.update(batch))
                probe.cancel_contact = Mock(side_effect=RuntimeError('stop unconfirmed') if stop_fails else None)
                if stop_fails:
                    with self.assertRaisesRegex(RuntimeError, 'stop unconfirmed'):
                        probe.calibration_step(-.05)
                    self.assertFalse(probe._load_limit_stop_confirmed)
                    with self.assertRaisesRegex(RuntimeError, 'termination'):
                        probe.calibration_step(.05)
                else:
                    probe.calibration_step(-.05)
                    self.assertTrue(probe._load_limit_stop_confirmed)
                    probe.check_interrupted.assert_called_once()
                    self.assertEqual(probe.control.emit.call_args.args[0], 'load_limit')
                    self.assertEqual(probe._release_ceiling, 135000.)
                probe.cancel_contact.assert_called_once()
                self.assertFalse(probe._calibration_step_active)

    def test_absolute_calibration_ceiling_and_unrelated_faults_are_not_recoverable(self):
        probe = self.limited_probe()
        with self.assertRaisesRegex(RuntimeError, 'hard load ceiling'):
            probe.detector.update(self.samples(250000.))
        probe.move_z = Mock(side_effect=RuntimeError('temperature feedback stale'))
        probe.cancel_contact = Mock()
        with self.assertRaisesRegex(RuntimeError, 'temperature feedback stale'):
            probe.calibration_step(-.05)
        probe.cancel_contact.assert_not_called()

    def test_only_fresh_stable_unloaded_window_clears_release_only_latch(self):
        probe = self.limited_probe()
        probe.detector.update(self.samples(130000.))
        probe._clear_released_limit(self.samples(0., observed=99.))
        self.assertTrue(probe.load_limit_reached)
        probe._clear_released_limit(self.samples(5000., observed=100.5))
        self.assertTrue(probe.load_limit_reached)
        probe._clear_released_limit(self.samples(100., observed=100.5))
        self.assertFalse(probe.load_limit_reached)
        self.assertIsNone(probe._release_ceiling)
        self.assertEqual(probe.detector.baseline, 0.)


class ActionControlTests(unittest.TestCase):
    def test_record_requires_valid_grams_and_fresh_single_use_token(self):
        for grams in (None, True, float('nan'), float('inf'), 49.9, 25000.1, '100'):
            control=LoadCalibrationControl(io.StringIO(),io.StringIO(),clock=lambda:100.)
            control.action_token='a'*32;control.action_deadline=110.
            with self.subTest(grams=grams),self.assertRaisesRegex(ValueError,'scale reading'):
                control.ingest(dict(command='record',token=control.action_token,grams=grams))
            self.assertIsNone(control.action)
        control=LoadCalibrationControl(io.StringIO(),io.StringIO(),clock=lambda:100.)
        control.action_token='a'*32;control.action_deadline=110.
        command=dict(command='record',token=control.action_token,grams=123.5)
        self.assertFalse(control.ingest(dict(command,token='stale')))
        self.assertTrue(control.ingest(command))
        self.assertFalse(control.ingest(command))
        self.assertEqual(control.wait_action(lambda:None,lambda:{}),command)

    def test_steps_are_single_use_and_cannot_be_queued_before_adjustment(self):
        control=LoadCalibrationControl(io.StringIO(),io.StringIO(),clock=lambda:100.)
        self.assertFalse(control.ingest(dict(command='step',token='a'*32,delta_mm=-.01)))
        def pump():
            data=dict(command='step',token=control.action_token,delta_mm=-.01)
            self.assertFalse(control.ingest(dict(data,token='stale')))
            self.assertTrue(control.ingest(data))
            self.assertFalse(control.ingest(data))
        result=control.wait_action(pump,lambda:{})
        self.assertEqual(result['delta_mm'],-.01)
        self.assertIsNone(control.action_token)
        self.assertFalse(control.ingest(result))

    def test_expired_step_and_lost_bridge_never_move(self):
        now=[100.]
        control=LoadCalibrationControl(io.StringIO(),io.StringIO(),clock=lambda:now[0])
        control.action_token='a'*32;control.action_deadline=110.
        now[0]=111.;control.last_heartbeat=111.
        self.assertFalse(control.ingest(dict(command='step',token='a'*32,delta_mm=-.01)))
        now[0]=117.
        with self.assertRaisesRegex(RuntimeError,'heartbeat'):
            control.ingest(dict(command='step',token='a'*32,delta_mm=-.01))


class NativeCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.probe=ScaleRoutineTests().limited_probe()
        probe=self.probe
        probe.config=SimpleNamespace(dry_run=False)
        probe.calibration_policy=Mock()
        probe.calibration_measurement=None
        probe.calibration_tare_counts=0.
        probe.joint_received=100.
        probe.joints_fresh=Mock(return_value=True)
        probe._calibration_velocity=np.zeros(6)
        probe._calibration_velocity_received=100.
        probe.nozzle_now=lambda:(probe.target.copy(),probe.seed.copy())
        probe.pump=Mock()
        probe.cancel_contact=Mock()
        self.samples=ScaleRoutineTests.samples(0.)
        probe.source.snapshot=lambda:self.samples
        probe.source.wait_window=lambda *args,**kwargs:self.samples
        self.receipt=dict(reference_tare_counts=0,counts_per_gram=600.,grams=150.)
        probe.calibration_client=Mock()
        probe.calibration_client.begin_unloaded.return_value={'reference_tare_counts':0}
        probe.calibration_client.record_load.side_effect=lambda grams,pump:(pump() or dict(self.receipt,grams=grams))
        probe.calibration_client.accept.side_effect=lambda pump:(pump() or dict(self.receipt,accepted=True,saved=False,config_fingerprint='a'*64))

    def loaded(self):
        self.samples=ScaleRoutineTests.samples(90000.,sequence=100)

    def unloaded(self):
        self.samples=ScaleRoutineTests.samples(0.,sequence=200)

    def test_native_zero_record_accept_preserve_original_guard_and_require_unload(self):
        probe=self.probe
        probe.begin_calibration()
        probe.calibration_client.begin_unloaded.assert_called_once()
        self.assertEqual(probe.detector.baseline,0.)
        self.loaded()
        probe.record_calibration(150.)
        self.assertEqual(probe.calibration_measurement,self.receipt)
        self.assertFalse(probe.is_unloaded(self.samples))
        with self.assertRaisesRegex(RuntimeError,'Release all load'):
            probe.accept_calibration()
        probe.calibration_client.accept.assert_not_called()
        self.unloaded()
        result=probe.accept_calibration()
        self.assertTrue(result['accepted'])
        self.assertFalse(result['saved'])
        self.assertEqual(result['config_fingerprint'],'a'*64)
        self.assertEqual(probe.cancel_contact.call_count,2)
        self.assertEqual(probe.detector.baseline,0.)
        self.assertEqual(probe.reference_counts,0.)

    def test_record_rejects_unloaded_noisy_repeated_or_load_limit_measurements(self):
        probe=self.probe
        probe.record_calibration(150.)
        probe.calibration_client.record_load.assert_not_called()
        self.loaded()
        self.samples[0]=ForceSample(100,100.,100.,96000.)
        probe.record_calibration(150.)
        probe.calibration_client.record_load.assert_not_called()
        self.loaded()
        probe.load_limit_reached=True
        probe.record_calibration(150.)
        probe.calibration_client.record_load.assert_not_called()
        probe.load_limit_reached=False
        probe.record_calibration(150.)
        probe.record_calibration(200.)
        probe.calibration_client.record_load.assert_called_once()
        self.assertEqual(probe.calibration_measurement['grams'],150.)

    def test_native_minimum_is_checked_before_command_and_during_collection(self):
        probe=self.probe
        self.samples=ScaleRoutineTests.samples(83000.,sequence=100)
        probe.record_calibration(150.)
        probe.calibration_client.record_load.assert_not_called()
        self.assertIsNone(probe.calibration_measurement)
        self.samples=ScaleRoutineTests.samples(85000.,sequence=200)
        def command(grams,pump):
            self.samples=ScaleRoutineTests.samples(83886.,sequence=300)
            pump()
        probe.calibration_client.record_load.side_effect=command
        with self.assertRaisesRegex(RuntimeError,'native calibration minimum'):
            probe.record_calibration(150.)
        self.assertIsNone(probe.calibration_measurement)

    def test_new_overload_in_record_window_returns_to_release_controls(self):
        probe=self.probe
        self.samples=ScaleRoutineTests.samples(130000.,sequence=100)
        probe.record_calibration(150.)
        self.assertTrue(probe.load_limit_reached)
        self.assertEqual(probe._release_ceiling,135000.)
        self.assertEqual(probe.control.emit.call_args.args[0],'step_rejected')
        probe.calibration_client.record_load.assert_not_called()

    def test_no_accept_without_record_or_confirmed_stop(self):
        probe=self.probe
        with self.assertRaisesRegex(RuntimeError,'Record a valid'):
            probe.accept_calibration()
        probe.cancel_contact.assert_not_called()
        probe.calibration_measurement=self.receipt
        probe.cancel_contact.side_effect=RuntimeError('stop unconfirmed')
        with self.assertRaisesRegex(RuntimeError,'stop unconfirmed'):
            probe.accept_calibration()
        probe.calibration_client.accept.assert_not_called()
        self.assertFalse(probe._load_limit_stop_confirmed)

    def test_record_keeps_stationary_load_and_thermal_checks_during_native_command(self):
        for fault in ('robot','load','thermal'):
            with self.subTest(fault=fault):
                self.setUp();probe=self.probe;self.loaded()
                def command(grams,pump):
                    if fault=='robot':probe.target[2]+=.11
                    elif fault=='load':self.unloaded()
                    else:probe.calibration_policy.require_ready.side_effect=RuntimeError('temperature changed')
                    pump()
                probe.calibration_client.record_load.side_effect=command
                with self.assertRaises((RuntimeError,ValueError)):
                    probe.record_calibration(150.)
                self.assertIsNone(probe.calibration_measurement)

    def test_native_failure_does_not_create_record_or_success(self):
        probe=self.probe;self.loaded()
        probe.calibration_client.record_load.side_effect=RuntimeError('HTTP outcome unknown')
        with self.assertRaisesRegex(RuntimeError,'outcome unknown'):
            probe.record_calibration(150.)
        self.assertIsNone(probe.calibration_measurement)

    def test_measurement_requires_stationary_velocity_and_matching_fresh_feedback(self):
        probe=self.probe
        for fault in ('moving','missing','nonfinite','stale','mismatched'):
            with self.subTest(fault=fault):
                self.setUp();probe=self.probe;self.loaded()
                if fault=='moving':probe._calibration_velocity[2]=.00011
                elif fault=='missing':probe._calibration_velocity=None
                elif fault=='nonfinite':probe._calibration_velocity[2]=float('nan')
                elif fault=='stale':probe.joints_fresh.return_value=False
                else:probe._calibration_velocity_received=99.
                with self.assertRaisesRegex(RuntimeError,'motion|velocity|stale'):
                    probe.record_calibration(150.)
                probe.calibration_client.record_load.assert_not_called()

    def test_measurement_checks_terminal_action_before_native_request(self):
        for phase in ('acceptance','unconsumed_acceptance','running','controller_fault'):
            with self.subTest(phase=phase):
                self.setUp();probe=self.probe;self.loaded()
                probe.goal_future=Mock()
                probe.goal_future.done.return_value=phase!='acceptance'
                probe.goal_handle=None if phase in ('acceptance','unconsumed_acceptance') else SimpleNamespace(accepted=True)
                probe.result_future=Mock()
                probe.result_future.done.return_value=phase!='running'
                probe.result_future.result.return_value=SimpleNamespace(status=6,result=SimpleNamespace(error_code=-1))
                with self.assertRaisesRegex(RuntimeError,'trajectory|controller fault'):
                    probe.record_calibration(150.)
                probe.calibration_client.record_load.assert_not_called()

    def test_stationary_pose_noise_remains_contained_without_changing_bed_settling(self):
        self.loaded();probe=self.probe
        origin=probe.target.copy();seed=probe.seed.copy()
        # Extremes from the 1502-sample unloaded UR trace captured 2026-11-16.
        offsets=[([.0237993,-.0512796,.0157348],[.000085086,-.000015497,-.000019073,.000007153,-.000020027,0.]),
                 ([-.0437097,.0345322,.0309351],[-.000055283,-.000019133,-.000046730,.000003815,-.000012398,0.]),
                 ([-.0289611,.0107917,.0373731],[-.000012517,-.000034630,-.000034809,-.000011444,-.000027657,.000019073])]
        def record(grams,pump):
            for xyz,q in offsets:
                probe.target=origin+xyz;probe.seed=seed+q
                pump()
            return self.receipt
        probe.calibration_client.record_load.side_effect=record
        probe.record_calibration(150.)
        self.assertEqual(probe.calibration_measurement,self.receipt)
        # A materially different pose must still fail, even with zero reported velocity.
        self.setUp();probe=self.probe;self.loaded()
        def drift(grams,pump):
            probe.seed[0]=.002
            pump()
        probe.calibration_client.record_load.side_effect=drift
        with self.assertRaisesRegex(RuntimeError,'Robot moved'):
            probe.record_calibration(150.)

    def test_joint_callback_accepts_velocity_only_with_valid_base_position_sample(self):
        probe=object.__new__(LoadCalibration)
        probe.joints=np.zeros(6);probe.joint_received=None
        probe.config=SimpleNamespace(feedback_timeout_s=.25)
        probe.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100000000000))
        def message(stamp=100,velocity=None,positions=None):
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp,nanosec=0)),
                name=list(reversed(JOINT_NAMES)),position=list(reversed(positions or [0.]*6)),
                velocity=list(reversed([0.]*6 if velocity is None else velocity)))
        with patch('print_control.motion_executor.time.monotonic',return_value=10.):
            probe.on_joints(message(velocity=[.00001,.00002,0.,0.,0.,0.]))
            np.testing.assert_allclose(probe._calibration_velocity,[.00001,.00002,0.,0.,0.,0.])
            self.assertEqual(probe._calibration_velocity_received,10.)
            probe.on_joints(message(stamp=99,velocity=[1.]*6))
            self.assertEqual(probe._calibration_velocity[0],.00001)
            probe.on_joints(message(positions=[float('nan')]*6,velocity=[1.]*6))
            self.assertEqual(probe._calibration_velocity[0],.00001)
            probe.on_joints(message(velocity=[]))
            self.assertIsNone(probe._calibration_velocity)

    def test_dry_run_never_calls_native_calibration_client(self):
        with tempfile.TemporaryDirectory() as directory:
            probe=ScaleHarness(Path(directory)/'unused')
            probe.calibration_client=Mock()
            self.assertIsNone(probe.run_routine())
            self.assertEqual(probe.calibration_client.mock_calls,[])


class CalibrationHandoffTests(unittest.TestCase):
    def test_save_handoff_requires_all_cleanup_and_never_runs_save_config(self):
        import print_control.alps_load_calibration as module
        for failed_stage in (None,'stop','close','cancel'):
            with self.subTest(failed_stage=failed_stage):
                order=[]
                receipt=dict(reference_tare_counts=0,counts_per_gram=600.,grams=150.,
                             accepted=True,saved=False,config_fingerprint='a'*64)
                args=SimpleNamespace(config='/tmp/test-ur.yaml',dry_run=False,load_cell='load_cell_probe',
                                     calibration_max_counts=120000.,calibration_hot_target=None)
                control=Mock(reason=None)
                control.start.return_value=control
                control.emit.side_effect=lambda state,*args,**kwargs:order.append(state)
                source=Mock(moonraker_url='http://test.invalid:7125',api_key=None)
                node=Mock(stop_unconfirmed=False)
                node.run_routine.return_value=receipt
                def cleanup(stage):
                    order.append(stage)
                    if failed_stage==stage:raise RuntimeError(stage+' failed')
                    if failed_stage=='cancel' and stage=='close':
                        control.check.side_effect=RuntimeError('Stop requested during cleanup')
                node.stop.side_effect=lambda:cleanup('stop')
                node.destroy_node.side_effect=lambda:cleanup('destroy')
                source.close.side_effect=lambda:cleanup('close')
                client=Mock()
                client.prepare.side_effect=lambda pump:order.append('prepare')
                factory=Mock(return_value=client)
                module_stub=SimpleNamespace(KlipperLoadCalibration=factory)
                with patch.dict(sys.modules,{'print_control.klipper_load_calibration':module_stub}), \
                        patch.object(module,'LoadCalibrationControl',return_value=control), \
                        patch.object(module,'parse_args',return_value=args), \
                        patch.object(module,'load_profile',return_value=(ContactOptions(trigger_mode='raw'),None)), \
                        patch.object(module,'load_configs',return_value=(None,None)), \
                        patch.object(module,'sensor_setup',return_value=(None,source)) as sensor_setup, \
                        patch.object(module,'CalibrationPolicy'), \
                        patch.object(module,'LoadCalibration',side_effect=lambda *args:(order.append('node') or node)), \
                        patch.object(module,'signal'),patch.object(module,'rclpy'):
                    if failed_stage:
                        with self.assertRaises(SystemExit):module.main([])
                    else:module.main([])
                self.assertEqual(sensor_setup.call_args.args[1].max_delta_counts, 120000.)
                self.assertLess(order.index('prepare'),order.index('node'))
                if failed_stage:
                    self.assertNotIn('ready_to_save',order)
                    self.assertIn('failed',order)
                else:
                    self.assertLess(order.index('stop'),order.index('close'))
                    self.assertLess(order.index('close'),order.index('ready_to_save'))
                    self.assertEqual(control.emit.call_args.kwargs['calibration_values'],receipt)
                client.verify_and_save.assert_not_called()


if __name__=='__main__':unittest.main()
