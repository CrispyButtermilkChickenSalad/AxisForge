"""Cached kinematics must preserve the reference model and rejection checks."""

from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from spatialmath import SE3

from print_control.motion_planner import MotionConfig, MotionPlanner, PlannedMotion


class MotionPlannerTests(unittest.TestCase):
    def setUp(self):
        self.config = MotionConfig(tcp_offset_mm=(-100., 0., 117.06))
        self.planner = MotionPlanner(self.config)
        self.seed = np.deg2rad([-3.73, -90., -120., -60., 90., 0.])

    def test_corner_tolerance_can_only_tighten_existing_path_bound(self):
        pose=self.planner.kinematics.eval(self.seed)
        tip=self.planner.nozzle_position(self.seed)
        shifted=tip+np.array([.08,0.,0.])
        self.planner.check_pose(self.seed,shifted,shifted,pose[:3,:3])
        with self.assertRaisesRegex(ValueError,'TCP path error'):
            self.planner.check_pose(self.seed,shifted,shifted,pose[:3,:3],path_tolerance_mm=.05)
        far=tip+np.array([.3,0.,0.])
        with self.assertRaisesRegex(ValueError,'TCP path error'):
            self.planner.check_pose(self.seed,far,far,pose[:3,:3],path_tolerance_mm=1.)

    def test_cached_frames_match_dh_at_random_and_singular_poses(self):
        """Compare every link, not just the tip, including full joint revolutions."""
        planner = self.planner
        rng = np.random.default_rng(145)
        poses = np.vstack((np.zeros(6), self.seed, planner.robot.qr,
                           rng.uniform(*planner.robot.qlim, size=(100, 6))))
        accepted = rejected = 0
        for joints in poses:
            reference = planner.robot.fkine(joints)
            np.testing.assert_allclose(planner.kinematics.eval(joints), reference.A,
                                       atol=1e-12, rtol=0)
            links = planner.robot.fkine_all(joints)
            for chain, frame in zip(planner.link_kinematics, links[1:]):
                np.testing.assert_allclose(chain.eval(joints), frame.A, atol=1e-12, rtol=0)
            tip = 1000 * (reference.t + reference.R @ planner.offset_m)
            np.testing.assert_allclose(planner.nozzle_position(joints), tip, atol=1e-9, rtol=0)
            planner.check_pose(joints, tip, tip, reference.R)
            if min(frame.t[2] for frame in links) * 1000 < self.config.min_link_z_mm:
                with self.assertRaisesRegex(ValueError, 'link origin'):
                    planner.check_joints(joints)
                rejected += 1
            else:
                planner.check_joints(joints)
                accepted += 1
        self.assertGreater(accepted, 0)
        self.assertGreater(rejected, 0)

    def test_seeded_solution_matches_full_search_for_rotary_targets(self):
        planner = self.planner
        seed = self.seed.copy()
        for angles in ((180., 0., 90.), (185., 0., 90.), (180., -5., 90.),
                       (180., 0., 95.), (180., 0., 90.)):
            rotation = SE3.RPY(angles, unit='deg', order='zyx').R
            point = np.array([-627.13, -269.78, 110.])
            reference = planner.robot.ikine_LM(planner.tool_pose(point, rotation),
                                              q0=seed, joint_limits=True)
            self.assertTrue(reference.success)
            self.assertEqual(reference.searches, 1)
            expected = seed + (reference.q - seed + np.pi) % (2*np.pi) - np.pi
            original_seed = seed.copy()
            result = planner.solve(point, seed, rotation)
            np.testing.assert_array_equal(seed, original_seed)
            np.testing.assert_allclose(result, expected, atol=1e-12, rtol=0)
            seed = result

    def test_failed_seed_search_retries_and_validates_the_result(self):
        planner = self.planner
        point = np.array([-627.13, -269.78, 110.])
        full_search = planner.kinematics.ikine_LM

        def retry(target, **kwargs):
            if kwargs.get('slimit') == 1:
                return SimpleNamespace(success=False)
            return full_search(target, **kwargs)

        with patch.object(planner.kinematics, 'ikine_LM', side_effect=retry) as solver:
            joints = planner.solve(point, self.seed)
        self.assertEqual(solver.call_count, 2)
        np.testing.assert_allclose(planner.nozzle_position(joints), point, atol=.01, rtol=0)
        planner.check_joints(joints)

    def test_unsolved_target_still_fails(self):
        with patch.object(self.planner.kinematics, 'ikine_LM',
                          return_value=SimpleNamespace(success=False)):
            with self.assertRaisesRegex(ValueError, 'IK failed'):
                self.planner.solve(np.array([10000., 0., 0.]), self.seed)

    def test_custom_joint_limits_apply_to_search_and_postcheck(self):
        limits = (170., 150., 140., 130., 120., 110.)
        planner = MotionPlanner(replace(self.config, joint_limits_deg=limits))
        np.testing.assert_allclose(planner.kinematics.qlim,
                                   np.deg2rad([-np.array(limits), limits]))
        for joint in range(6):
            for sign in (-1., 1.):
                joints = self.seed.copy()
                joints[joint] = sign * np.deg2rad(limits[joint] + .01)
                with self.assertRaisesRegex(ValueError, 'Joint position'):
                    planner.check_joints(joints)

    def test_invalid_joints_rejected_before_compiled_kinematics(self):
        for joints in (np.zeros(5), np.zeros((2, 6)), [np.nan]*6, [np.inf]*6):
            with self.assertRaisesRegex(ValueError, 'Invalid IK/joint state'):
                self.planner.check_joints(joints)

    def test_tcp_and_orientation_errors_are_still_rejected(self):
        planner = self.planner
        pose = planner.robot.fkine(self.seed)
        tip = 1000 * (pose.t + pose.R @ planner.offset_m)
        wrong_point = tip + [self.config.path_tolerance_mm + .01, 0., 0.]
        with self.assertRaisesRegex(ValueError, 'TCP path error'):
            planner.check_pose(self.seed, wrong_point, wrong_point, pose.R)
        wrong_rotation = pose.R @ SE3.Rx(self.config.orientation_tolerance_deg + .01,
                                        unit='deg').R
        with self.assertRaisesRegex(ValueError, 'Orientation error'):
            planner.check_pose(self.seed, tip, tip, wrong_rotation)

    def test_interpolation_speed_and_acceleration_limits_still_apply(self):
        for name, dt, end_velocity, expected_error in (
                ('speed', .1, 1., 'joint speed'),
                ('acceleration', .01, 0., 'joint acceleration')):
            positions = np.array([self.seed, self.seed + [0., 0., 0., 0., 0., .001]])
            velocities = np.zeros((2, 6))
            velocities[:, -1] = end_velocity
            motion = PlannedMotion(name, positions, velocities, np.array([0., dt]))
            with self.assertRaisesRegex(ValueError, expected_error):
                self.planner.check_motion(motion)


if __name__ == '__main__':
    unittest.main()
