"""Exercise contact/retract/save and cancellation using a synthetic bed only."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml
from action_msgs.msg import GoalStatus

from print_control.bed_probe import grid_points
from print_control.bed_surface import BedSurface
from print_control.gcode_path import GCodeConfig
from print_control.load_cell_bed_probe import LoadCellBedProbe, SimulatedAlps
from print_control.load_cell_contact import ContactDetector, ContactOptions, ContactDetected
from print_control.motion_planner import MotionConfig
from print_control.motion_executor import MotionExecutor


class ProbeHarness(LoadCellBedProbe):
    """No ROS node, publishers or action clients; measured pose differs from target."""
    def __init__(self, output):
        self.settings = GCodeConfig(z_offset=100., bed_probe_file='')
        self.config = MotionConfig(dry_run=True, clearance_mm=.12, tcp_offset_mm=(-100., 0., 117.06))
        self.planner = SimpleNamespace(config=self.config)
        self.args = SimpleNamespace(config='unused', floor_mm=.2, paper_mm=0., load_cell='load_cell_probe')
        self.output = Path(output)
        self.points = grid_points(self.settings.bed_size_mm, 30.)
        self.records = {}; self.index = 0; self.promoted = False
        self.target = self.hover(self.points[0]); self.seed = np.zeros(6)
        self.contact_options = ContactOptions()
        self.source = SimulatedAlps(self)
        self.detector = ContactDetector(0., 100., self.contact_options)
        self.seeking = False; self.moves = []; self.stops = 0

    def check_interrupted(self):
        if self.detector.update(self.source.snapshot()) and self.seeking:
            raise ContactDetected('simulated contact')

    def line_to(self, end, name, speed):
        self.moves.append((self.target.copy(), np.array(end), name))
        self.target = np.asarray(end).copy()

    def nozzle_now(self):
        measured = self.target.copy(); measured[2] += .003
        return measured, np.arange(6, dtype=float)*.01


class BedContactTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)/'bed_probe.yaml'
        self.output.write_text('previous map\n')
        self.probe = ProbeHarness(self.output)

    def test_real_probe_planner_accepts_tight_contact_tolerance_without_print_blending(self):
        args = SimpleNamespace(margin_mm=30., paper_mm=0., floor_mm=3., jog_speed=5., xy_step_mm=10.,
                               output=str(self.output.parent/'simulated.yaml'), config='unused')
        # Suppress only ROS node creation; use the real planner and dry executor.
        with patch.object(MotionExecutor, '__init__', return_value=None):
            node = LoadCellBedProbe(GCodeConfig(), MotionConfig(dry_run=True), args, None, ContactOptions())
        point = node.hover(node.points[0])
        q = node.planner.solve(point, node.joints)
        end = point-np.array([0., 0., .02])
        motion = node.planner.sampled_line('contact', point, end, q, .2)
        self.assertLess(np.linalg.norm(node.planner.nozzle_position(motion.positions[-1])-end), .005)
        self.assertIsNone(node.action); self.assertIsNone(node.speed_publisher)

    def test_repeated_contact_records_measured_tip_then_retracts_and_lifts(self):
        self.probe.probe_current()
        record = self.probe.records[0]
        self.assertEqual(len(record['contacts']), 3)
        self.assertLessEqual(record['repeatability_mm'], .05)
        self.assertEqual(record['joints_rad'], [0., .01, .02, .03, .04, .05])
        self.assertGreaterEqual(self.probe.target[2], 100.12)
        for start, end, name in self.probe.moves:
            if name == 'contact step':
                self.assertLessEqual(start[2]-end[2], .020001)
                np.testing.assert_equal(start[:2], end[:2])
        # No paper subtraction; the recorded tip comes from nozzle_now(), not a command.
        self.assertEqual(record['bed_z_mm'], record['nozzle_base_mm'][2])
        self.assertEqual(self.output.read_text(), 'previous map\n')
        data = yaml.safe_load(self.output.with_suffix('.pending.yaml').read_text())
        self.assertFalse(data['complete']); self.assertTrue(data['simulated'])

    def test_complete_grid_atomically_replaces_and_backs_up_previous_map(self):
        self.probe.probe_all()
        data = yaml.safe_load(self.output.read_text())
        self.assertTrue(data['complete']); self.assertEqual(len(data['points']), 9)
        backups = list(self.output.parent.glob('bed_probe.yaml.*.bak'))
        self.assertEqual(len(backups), 1); self.assertEqual(backups[0].read_text(), 'previous map\n')
        self.assertFalse(self.output.with_suffix('.pending.yaml').exists())
        # Simulation is deliberately rejected by the printing loader.
        with self.assertRaisesRegex(ValueError, 'simulated'):
            BedSurface.load(self.output, self.probe.config.tcp_offset_mm, self.probe.config.nozzle_rpy_deg, 1.)
        data['simulated'] = False; self.output.write_text(yaml.safe_dump(data))
        surface = BedSurface.load(self.output, self.probe.config.tcp_offset_mm, self.probe.config.nozzle_rpy_deg, 1.)
        for record in data['points']:
            self.assertAlmostEqual(surface.height(record['nozzle_base_mm'][:2]), record['bed_z_mm'])

    def test_no_contact_stops_at_floor_without_changing_active_map(self):
        original = self.probe.source.snapshot
        self.probe.source.snapshot = lambda: [replace(s, counts=0.) for s in original()]
        with self.assertRaisesRegex(RuntimeError, 'probe floor'): self.probe.probe_current()
        self.assertGreaterEqual(self.probe.target[2], 99.8-1e-8)
        self.assertFalse(self.probe.records); self.assertEqual(self.output.read_text(), 'previous map\n')

    def test_overload_prevents_another_step_or_save(self):
        original = self.probe.source.snapshot
        def overloaded():
            return [replace(s, counts=21000.) if self.probe.target[2] < 100.1 else s for s in original()]
        self.probe.source.snapshot = overloaded
        with self.assertRaisesRegex(RuntimeError, 'overload'): self.probe.probe_current()
        self.assertFalse(self.probe.records)
        self.assertGreaterEqual(self.probe.target[2], 100.08-1e-8)

    def test_sensor_failure_during_descent_stops_without_retracting_blindly(self):
        original = self.probe.source.snapshot
        def stale():
            if self.probe.target[2] < 100.1: raise RuntimeError('ALPS samples are stale')
            return original()
        self.probe.source.snapshot = stale
        with self.assertRaisesRegex(RuntimeError, 'stale'): self.probe.probe_current()
        self.assertFalse(self.probe.records)
        self.assertEqual(self.probe.moves[-1][2], 'contact step')

    def test_persistent_load_after_retract_prevents_save(self):
        with patch.object(self.probe, 'retract_contact', side_effect=RuntimeError('load did not clear')):
            with self.assertRaisesRegex(RuntimeError, 'did not clear'): self.probe.probe_current()
        self.assertFalse(self.probe.records)

    def test_failed_repeatability_does_not_replace_previous_map(self):
        with patch('print_control.load_cell_bed_probe.repeat_contact', side_effect=ValueError('repeatability failed')):
            with self.assertRaisesRegex(RuntimeError, 'repeatability'): self.probe.probe_current()
        self.assertEqual(self.output.read_text(), 'previous map\n')
        self.assertFalse(self.probe.records)

    def test_cancellation_must_finish_before_measuring_or_retracting(self):
        self.probe.config = replace(self.probe.config, dry_run=False)
        self.probe.stop = lambda: None
        self.probe.goal_future = SimpleNamespace(done=lambda: True)
        self.probe.goal_handle = SimpleNamespace(accepted=True)
        self.probe.result_future = SimpleNamespace(done=lambda: False)
        with patch.object(self.probe, 'nozzle_now') as measured:
            with self.assertRaisesRegex(RuntimeError, 'stop unconfirmed'): self.probe.cancel_contact()
            measured.assert_not_called()
        self.probe.result_future = SimpleNamespace(done=lambda: True,
            result=lambda: SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        self.probe.cancel_contact()
        self.assertIsNone(self.probe.goal_handle); self.assertFalse(self.probe.aborting)
        np.testing.assert_equal(self.probe.seed, np.arange(6)*.01)

    def test_manual_downward_jog_is_disabled_in_contact_mode(self):
        before = self.probe.target.copy()
        self.probe.on_key('s'); self.probe.on_key('j')
        np.testing.assert_equal(before, self.probe.target)


if __name__ == '__main__': unittest.main()
