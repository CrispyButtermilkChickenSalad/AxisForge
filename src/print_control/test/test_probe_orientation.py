"""Probe Z moves preserve the measured upright tool orientation without ROS I/O."""
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np
from spatialmath import SE3

from print_control.bridge_bed_probe import BridgeBedProbe
from print_control.bed_probe import grid_points, load_configs
from print_control.gcode_path import GCodeConfig, GCodePlanner
from print_control.motion_planner import MotionConfig, cubic_joint_state


class OrientationHarness(BridgeBedProbe):
    """Use real capture/line planning with fake joint feedback and execution."""
    def __init__(self):
        self.settings = GCodeConfig(lookahead_enabled=False, z_offset=100.)
        self.config = MotionConfig(dry_run=False, tcp_offset_mm=(-100., 0., 117.06))
        self.planner = GCodePlanner(self.settings, self.config)
        self.measured_rotation = SE3.RPY((180.1, 0., 90.), unit='deg', order='zyx').R
        self.initial_tip = np.array([-627.13, -269.78, 120.])
        self.joints = self.planner.solve(self.initial_tip,
            np.deg2rad([-3.73, -90., -120., -60., 90., 0.]), self.measured_rotation)
        self.motions = []

    def robot_reason(self): return ''

    def nozzle_now(self): return self.planner.nozzle_position(self.joints), self.joints.copy()

    def run_motion(self, motion):
        self.motions.append(motion)
        self.joints = motion.positions[-1].copy()
        self.seed = self.joints.copy()


class ProbeOrientationTests(unittest.TestCase):
    def test_fast_retract_and_longest_grid_travels_pass_real_ik_and_path_limits(self):
        node = OrientationHarness()
        path = Path(__file__).resolve().parents[2]/'axisforge_bringup/config/ur_master.yaml'
        settings, config = load_configs(path, True)
        node.settings = replace(settings, bed_probe_file='', lookahead_enabled=False)
        node.config = replace(config, path_tolerance_mm=.0025)
        node.planner = GCodePlanner(node.settings, node.config)
        node.joints = node.planner.solve(node.initial_tip, node.joints, node.measured_rotation)
        # Use the already known fake feedback, without capture_start's dry-run positioning.
        node.config = replace(node.config, dry_run=False)
        node.capture_start()
        for distance in (.5, 3., 10.):
            before = node.target.copy()
            node.move_z(float(before[2]+distance), 'fast retract', 20.)
            fast = node.motions[-1]
            # Must actually finish well before an equally long 0.1 mm/s descent.
            self.assertLess(fast.times[-1], distance/.1)
            np.testing.assert_allclose(node.planner.nozzle_position(node.joints),
                                       before+[0., 0., distance], atol=.0025)
        # A 2x2 grid exercises the longest horizontal moves in the same bed area.
        height = float(node.target[2])
        for point in grid_points(node.settings.bed_size_mm, 30., 2, 2):
            target = node.bed_point(point)
            target[2] = height
            node.line_to(target, 'fast mesh XY', node.settings.max_travel_speed_mm_s)
            np.testing.assert_allclose(node.planner.nozzle_position(node.joints), target, atol=.0025)
        self.assertEqual(len(node.motions), 7)

    def test_capture_then_ten_mm_up_and_down_preserve_measured_rotation(self):
        node = OrientationHarness()
        # Exercise the actual capture_start(), not a mock of its settings update.
        node.capture_start()
        start = node.target.copy()
        measured = node.planner.kinematics.eval(node.joints)[:3, :3].copy()
        self.assertIs(node.planner.settings, node.settings)
        np.testing.assert_allclose(SE3.RPY((node.settings.nozzle_roll_deg,
            node.settings.nozzle_pitch_deg, node.settings.wrist_angle),
            unit='deg', order='zyx').R, measured, atol=1e-10)
        for dz in (10., -10.):
            before = node.target.copy()
            node.move_z(float(before[2]+dz), 'calibration Z step', 1.)
            motion = node.motions[-1]
            for index, dt in enumerate(np.diff(motion.times)):
                for fraction in (0., .25, .5, .75, 1.):
                    joints, _, _ = cubic_joint_state(*motion.positions[index:index+2],
                        *motion.velocities[index:index+2], dt, fraction)
                    rotation = node.planner.kinematics.eval(joints)[:3, :3]
                    np.testing.assert_allclose(rotation, measured, atol=2e-5)
                    np.testing.assert_allclose(node.planner.nozzle_position(joints)[:2],
                                               start[:2], atol=.005)
            np.testing.assert_allclose(node.planner.nozzle_position(node.joints),
                                       before+[0., 0., dz], atol=.005)
        np.testing.assert_allclose(node.target, start, atol=1e-9)
        self.assertEqual(len(node.motions), 2)


if __name__ == '__main__': unittest.main()
