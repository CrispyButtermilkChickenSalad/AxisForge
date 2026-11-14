"""Bed limits use resolved coordinates and apply before robot planning."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from print_control.bed_probe import grid_points
from print_control.bed_surface import BedSurface
from print_control.gcode_parser import GCodeParser
from print_control.gcode_path import GCodeConfig, GCodePlanner
from print_control.motion_planner import MotionConfig
from print_control.sync_path import TimedPath, canonical_hash
from test_sync_path import plan


class BedBoundsTests(unittest.TestCase):
    def test_resolved_absolute_relative_inch_and_g92_coordinates_reject_purge_and_park(self):
        settings = GCodeConfig(bed_size_mm=(100.,80.), bed_probe_file='')
        cases = ('G0 X101 Y40 F600\nG1 X90',
                 'G0 X90 Y40 F600\nG91\nG1 X11',
                 'G0 X90 Y40 F600\nG92 X0\nG1 X11',
                 'G20\nG0 X4 Y1 F60\nG1 X3',
                 'G0 X50 Y40 F600\nG1 Y-1',
                 'G0 X50 Y40 F600\nG1 Y81')
        for text in cases:
            with self.subTest(text=text):
                parser = GCodeParser()
                planner = GCodePlanner(settings, MotionConfig())
                with patch.object(planner, 'solve', side_effect=AssertionError('IK must not run')):
                    with self.assertRaisesRegex(ValueError, 'G-code line .*bed bounds'):
                        planner.plan_segments(parser.parse(text.splitlines()), np.zeros(6))

    def test_limits_are_in_gcode_space_and_do_not_depend_on_base_offsets(self):
        for centered in (False, True):
            cfg = GCodeConfig(bed_size_mm=(100.,80.), origin_at_center=centered, x_offset=-700.,y_offset=-300.)
            for point in ((0.,0.,0.), (100.,80.,50.), (50.,40.,0.)):
                cfg.check_bed_bounds(point)
            for point in ((-.01,0.,0.), (100.01,80.,0.), (50.,80.01,0.)):
                with self.assertRaisesRegex(ValueError, 'bed bounds'):
                    cfg.check_bed_bounds(point)

    def test_timed_master_schedule_is_rejected_before_ik_even_with_valid_hash(self):
        config = Path(__file__).resolve().parents[2]/'axisforge_bringup/config/ur_master.yaml'
        value = plan()
        value['operations'][0]['moves'][0]['end'][0] = 236.
        value['plan_hash'] = canonical_hash({k:v for k,v in value.items() if k!='plan_hash'})
        with patch('print_control.sync_path.GCodePlanner', side_effect=AssertionError('IK must not run')):
            with self.assertRaisesRegex(ValueError, 'line 2.*bed bounds'):
                TimedPath(config, value, np.zeros(6))

    def test_mesh_for_a_previous_bed_size_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'bed.yaml'
            path.write_text(yaml.safe_dump({'transform':{'bed_size_mm':[235.,235.]}}))
            with self.assertRaisesRegex(ValueError, 'dimensions changed'):
                BedSurface.load(path, (0.,0.,0.), (180.,0.,90.), 5., expected_bed_size=(150.,100.))

    def test_resized_mesh_fits_each_axis_and_margin_without_changing_point_count(self):
        for size, margin in (((150.,100.), 10.), ((20.,10.), 0.)):
            points = np.asarray(grid_points(size, margin, 5, 9))
            self.assertEqual(len(points),45)
            np.testing.assert_allclose(points.min(axis=0), [margin,margin])
            np.testing.assert_allclose(points.max(axis=0), np.asarray(size)-margin)
