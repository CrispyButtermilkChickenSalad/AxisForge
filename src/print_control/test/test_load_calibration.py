"""Exercise scale orchestration without ROS hardware or any calibration writes."""
import io
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from print_control.alps_load_calibration import LoadCalibration, sensitive_options
from print_control.load_calibration_control import LoadCalibrationControl
from print_control.load_cell_contact import ContactOptions
from test_bridge_bed_probe import GuidedProbeHarness


class ScaleHarness(LoadCalibration, GuidedProbeHarness):
    def __init__(self, output):
        GuidedProbeHarness.__init__(self, output)
        self.control = LoadCalibrationControl(io.StringIO(),io.StringIO(),clock=lambda:100.)
        self.actions = [-.5]*9+[-.05]*9+[.5,'finish']
        self.capture_confirmations = None

    def capture_start(self):
        self.capture_confirmations = list(self.confirmed_stages)
        GuidedProbeHarness.capture_start(self)

    def pump(self):
        GuidedProbeHarness.pump(self)
        if self.control.action_token:
            action=self.actions.pop(0)
            self.control.ingest(dict(command='finish' if action=='finish' else 'step',
                                     token=self.control.action_token, delta_mm=action))


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
        self.assertIn('0.05',probe.step_error(-.5))
        self.assertEqual(probe.step_error(-.05),'')
        probe.target[2]=99.76
        self.assertIn('travel limit',probe.step_error(-.05))
        probe.target[2]=105.
        self.assertIn('travel limit',probe.step_error(.001))
        for bad in (float('nan'),True,0.,-1.):self.assertTrue(probe.step_error(bad))

    def test_raw_trigger_adapts_to_noise_without_weakening_noise_guard(self):
        options=ContactOptions(trigger_mode='raw')
        for noise, expected in ((1000.,8000.),(4000.,16400.)):
            adjusted,_=sensitive_options(options,[SimpleNamespace(counts=v) for v in (0.,noise)])
            self.assertEqual(adjusted.threshold,expected)
            self.assertGreaterEqual(adjusted.threshold*.25,noise)
        with self.assertRaisesRegex(ValueError,'noise is too high'):
            sensitive_options(options,[SimpleNamespace(counts=v) for v in (0.,6000.)])


class ActionControlTests(unittest.TestCase):
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


if __name__=='__main__':unittest.main()
