"""Validate actual continuous nozzle speed with the calibrated-offset UR model."""

from dataclasses import replace
import unittest
from unittest.mock import patch

import numpy as np

from print_control.gcode_path import GCodeConfig, GCodePlanner
from print_control.motion_planner import MotionConfig, cubic_joint_state
from print_control.probe_motion import plan_probe_descent


class ContinuousProbeMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = MotionConfig(dry_run=True, tcp_offset_mm=(-100., 0., 117.06),
                                  path_tolerance_mm=.005)
        cls.planner = GCodePlanner(GCodeConfig(lookahead_enabled=False), cls.config)
        cls.start = np.array([-627.13, -269.78, 120.])
        cls.seed = cls.planner.solve(cls.start, np.deg2rad([-3.73, -90., -120., -60., 90., 0.]))

    def test_cruise_is_actual_requested_nozzle_speed_between_waypoints(self):
        for speed, distance in ((10., 20.), (5., 3.5), (.2, 3.5), (.1, 1.), (.02, .2)):
            with self.subTest(speed_mm_s=speed):
                end = self.start - [0., 0., distance]
                with patch.object(self.planner, 'retime', side_effect=AssertionError('must not retime')):
                    motion = plan_probe_descent(self.planner, self.start, end, self.seed, speed)
                ramp = max(.1, 1.5*speed/50.)
                cruise_end = motion.times[-1]-ramp
                np.testing.assert_array_equal(motion.velocities[[0, -1]], np.zeros((2, 6)))
                np.testing.assert_allclose(self.planner.nozzle_position(motion.positions[-1]), end, atol=.001)
                self.assertLessEqual(np.max(np.diff(motion.times)), min(.05, .1/speed)+1e-9)
                samples, trace = [], []
                for index in range(len(motion.times)-1):
                    dt = motion.times[index+1]-motion.times[index]
                    for u in (.25, .5, .75):
                        t = motion.times[index]+u*dt
                        q, qd, _ = cubic_joint_state(*motion.positions[index:index+2],
                                                    *motion.velocities[index:index+2], dt, u)
                        tip = self.planner.nozzle_position(q)
                        np.testing.assert_allclose(tip[:2], self.start[:2], atol=.005)
                        self.planner.check_pose(q, self.start, end)
                        twist = self.planner.nozzle_jacobian(q) @ qd
                        trace.append((t, -twist[2]*1000.))
                        np.testing.assert_allclose(twist[:2]*1000., [0., 0.], atol=.005)
                        np.testing.assert_allclose(twist[3:], np.zeros(3), atol=1e-5)
                        if ramp < t < cruise_end:
                            samples.append(-twist[2]*1000.)
                self.assertGreater(len(samples), 20)
                np.testing.assert_allclose(samples, speed, atol=.01, rtol=0.)
                trace = np.asarray(trace)
                acceleration = np.diff(trace[:, 1])/np.diff(trace[:, 0])
                self.assertLessEqual(np.max(np.abs(acceleration)), 50.1)

    def test_short_path_is_rejected_instead_of_silently_lowering_speed(self):
        for speed, distance in ((10., 2.9), (10., 3.), (5., .75)):
            with self.subTest(speed=speed, distance=distance), self.assertRaisesRegex(ValueError, 'too short'):
                plan_probe_descent(self.planner, self.start, self.start-[0., 0., distance], self.seed, speed)

    def test_excessively_long_time_grid_is_rejected_before_allocation(self):
        with patch('print_control.probe_motion.np.linspace', side_effect=AssertionError('must not allocate')):
            with self.assertRaisesRegex(ValueError, '10000'):
                plan_probe_descent(self.planner, self.start, self.start-[0., 0., 20.], self.seed, 1e-8)

    def test_only_finite_bounded_vertical_descent_is_accepted(self):
        end = self.start-[0., 0., 20.]
        for speed in (0., -1., 10.01, float('nan'), float('inf'), True):
            with self.subTest(speed=speed), self.assertRaisesRegex(ValueError, 'speed'):
                plan_probe_descent(self.planner, self.start, end, self.seed, speed)
        for acceleration in (0., -1., 100.01, float('nan'), True):
            with self.subTest(acceleration=acceleration), self.assertRaisesRegex(ValueError, 'acceleration'):
                plan_probe_descent(self.planner, self.start, end, self.seed, 10., acceleration)
        for bad_end in (self.start, self.start+[0., 0., 1.], end+[1., 0., 0.], [0., 0., float('nan')]):
            with self.subTest(end=bad_end), self.assertRaises(ValueError):
                plan_probe_descent(self.planner, self.start, bad_end, self.seed, 10.)

    def test_singular_jacobian_is_rejected_before_motion(self):
        with patch.object(self.planner, 'nozzle_jacobian', return_value=np.zeros((6, 6))):
            with self.assertRaisesRegex(ValueError, 'singular'):
                plan_probe_descent(self.planner, self.start, self.start-[0., 0., 20.], self.seed, 10.)

    def test_joint_limits_fail_instead_of_retiming(self):
        for option, value, error in (('max_joint_speed_deg_s', .01, 'joint speed'),
                                     ('max_joint_acceleration_deg_s2', .01, 'joint acceleration')):
            planner = GCodePlanner(GCodeConfig(lookahead_enabled=False), replace(self.config, **{option:value}))
            with self.subTest(limit=option), self.assertRaisesRegex(ValueError, error):
                plan_probe_descent(planner, self.start, self.start-[0., 0., 20.], self.seed, 10.)

    def test_planning_cancellation_unwinds_before_action_creation(self):
        def cancelled():
            raise RuntimeError('operator abort')
        with self.assertRaisesRegex(RuntimeError, 'operator abort'):
            plan_probe_descent(self.planner, self.start, self.start-[0., 0., 20.], self.seed, 10.,
                               pump=cancelled)

    def test_supervision_is_pumped_during_ik_and_interval_validation(self):
        phases = []
        original = self.planner.check_motion
        def validate(*args, **kwargs):
            self.assertEqual(phases[-1], 'pump')
            phases.append('validate')
            return original(*args, **kwargs)
        with patch.object(self.planner, 'check_motion', side_effect=validate):
            motion = plan_probe_descent(self.planner, self.start, self.start-[0., 0., 20.], self.seed, 10.,
                                        pump=lambda: phases.append('pump'))
        self.assertGreater(phases.count('pump'), len(motion.times))
        self.assertGreater(phases.count('validate'), 10)


if __name__ == '__main__':
    unittest.main()
