"""Regression checks for rotary coordinates and calibrated nozzle motion."""

import unittest

import numpy as np

from print_control.bed_surface import subdivide
from print_control.gcode_parser import GCodeParser
from print_control.gcode_path import GCodeConfig, GCodePlanner
from print_control.motion_planner import MotionConfig


class RotaryAxesTest(unittest.TestCase):
    """Check the externally visible rotary convention and motion geometry."""

    def test_angles_stay_absolute_degrees_in_relative_inch_mode(self):
        """Changing linear units or mode must not scale or accumulate angles."""
        segments = GCodeParser().parse([
            'G1 X10 A5 B-10 C15 F600', 'G91', 'G20',
            'G1 X1 A2 B3 C4', 'G4 P100', 'G1 X1 C0',
        ])
        self.assertEqual(segments[1].orientation_deg, (2.0, 3.0, 4.0))
        np.testing.assert_allclose(segments[1].end, (35.4, 0.0, 0.0))
        self.assertEqual(segments[2].orientation_deg, (2.0, 3.0, 4.0))
        self.assertEqual(segments[3].orientation_deg, (2.0, 3.0, 0.0))
        np.testing.assert_allclose(segments[3].end, (60.8, 0.0, 0.0))

    def test_bed_subdivision_preserves_rotary_continuity_and_material(self):
        """Splitting a move must preserve its endpoints, angles and total E."""
        segment = GCodeParser().parse([
            'G1 X25 A10 B-20 C30 E5 F600',
        ])[0]
        pieces = subdivide([segment], max_length_mm=10)
        self.assertEqual(len(pieces), 3)
        self.assertEqual(pieces[0].start, segment.start)
        self.assertEqual(pieces[-1].end, segment.end)
        self.assertEqual(pieces[0].start_orientation_deg, (0.0, 0.0, 0.0))
        self.assertEqual(pieces[-1].orientation_deg, (10.0, -20.0, 30.0))
        self.assertAlmostEqual(sum(piece.delta_e for piece in pieces), 5.0)
        for left, right in zip(pieces, pieces[1:]):
            self.assertEqual(left.end, right.start)
            self.assertEqual(left.orientation_deg, right.start_orientation_deg)

    def test_positive_axes_follow_right_hand_rule(self):
        """At a zero home orientation, A/B/C turn about X/Y/Z respectively."""
        planner = GCodePlanner(GCodeConfig(
            nozzle_roll_deg=0.0, nozzle_pitch_deg=0.0, wrist_angle=0.0,
        ), MotionConfig())
        for angles, vector, expected in (
            ((90, 0, 0), (0, 1, 0), (0, 0, 1)),
            ((0, 90, 0), (0, 0, 1), (1, 0, 0)),
            ((0, 0, 90), (1, 0, 0), (0, 1, 0)),
        ):
            with self.subTest(angles=angles):
                np.testing.assert_allclose(
                    planner.rotation_for(angles) @ vector, expected, atol=1e-12,
                )

    def test_rotation_keeps_calibrated_nozzle_tip_at_target(self):
        """Actual UR16e IK must keep the offset nozzle fixed through each turn."""
        config = MotionConfig(tcp_offset_mm=(-100.0, 0.0, 117.06))
        planner = GCodePlanner(GCodeConfig(), config)
        point = np.array([-627.13, -269.78, 110.0])
        seed = np.deg2rad([-3.73, -90.0, -120.0, -60.0, 90.0, 0.0])
        joints = planner.solve(point, seed)
        home_rotation = planner.rotation.copy()
        for angles in ((5, 0, 0), (0, -5, 0), (0, 0, 5), (0, 0, 0)):
            with self.subTest(angles=angles):
                motion = planner.sampled_line(
                    'orientation', point, point, joints, 10.0, angles,
                )
                for sample in motion.positions:
                    self.assertLessEqual(
                        np.linalg.norm(planner.nozzle_position(sample) - point),
                        config.path_tolerance_mm,
                    )
                self.assertGreater(motion.times[-1], 0.0)
                joints = motion.positions[-1]
        np.testing.assert_allclose(
            planner.robot.fkine(joints).R, home_rotation, atol=1e-5,
        )


if __name__ == '__main__':
    unittest.main()
