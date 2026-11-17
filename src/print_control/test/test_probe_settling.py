"""Stationary contact estimates from synthetic feedback; no ROS or hardware I/O."""
from dataclasses import replace
import math
import unittest

import numpy as np

from print_control.probe_settling import (
    JointFeedback, StationaryError, StationaryMotionError, estimate_stationary,
    MAX_VELOCITY_RAD_S, WINDOW_S,
)


# A deterministic local linear FK with the same mm/radian scale as a robot arm.
BASE_JOINTS = np.array([.1686, -.9386, 1.6347, -2.2678, -1.5706, .1717])
BASE_XYZ = np.array([-640.4, -385.7, 102.3])
JACOBIAN = np.array([[500., -200., 350., 0., 75., 0.],
                     [-100., 400., 0., 250., 0., 100.],
                     [0., 200., -400., -50., 180., 0.]])
Z_DIRECTION = np.linalg.lstsq(JACOBIAN, np.array([0., 0., 1.]), rcond=None)[0]


def nozzle_fk(joints):
    return BASE_XYZ + JACOBIAN @ (np.asarray(joints) - BASE_JOINTS)


def trace(*, noise=True):
    """500 Hz, zero reported velocity, bounded approximately 0.01 mm XYZ noise."""
    rng = np.random.default_rng(20261116)
    rows = []
    for index in range(801):
        elapsed = .002 * (index + 1)
        perturbation = rng.normal(0., 1.5e-5, 6) if noise else np.zeros(6)
        rows.append(JointFeedback(received_at=10. + elapsed,
            source_time=2000. + elapsed, joints=BASE_JOINTS + perturbation,
            velocity=np.zeros(6), source_age_s=.001))
    return rows


def perturb_z(rows, displacement):
    return [replace(row, joints=np.asarray(row.joints) +
                    Z_DIRECTION * displacement(row.received_at - rows[0].received_at))
            for row in rows]


class StationaryEstimateTests(unittest.TestCase):
    def estimate(self, rows, **kwargs):
        options = dict(now=rows[-1].received_at + .005, stopped_at=10.,
                       terminal_confirmed=True)
        options.update(kwargs)
        return estimate_stationary(rows, nozzle_fk=nozzle_fk, **options)

    def test_realistic_noise_accepts_a_consistent_joint_and_fk_estimate(self):
        rows = trace()
        points = np.asarray([nozzle_fk(row.joints) for row in rows])
        # This trace deliberately exceeds the old individual-sample 0.01 mm gate.
        self.assertGreater(np.max(np.linalg.norm(points - points[0], axis=1)), .01)
        result = self.estimate(rows)
        self.assertGreaterEqual(result['sample_count'], math.ceil(200*WINDOW_S))
        self.assertGreaterEqual(result['duration_s'], WINDOW_S-.01)
        self.assertLessEqual(result['duration_s'], WINDOW_S+.001)
        self.assertLessEqual(result['position_radius_mm'], .01)
        self.assertLessEqual(result['joint_radius_rad'], .0001)
        np.testing.assert_allclose(result['nozzle'], nozzle_fk(result['joints']), atol=1e-12)
        self.assertLess(np.linalg.norm(result['nozzle'] - BASE_XYZ), .01)

    def test_exactly_constant_feedback_is_valid_when_timestamps_advance(self):
        result = self.estimate(trace(noise=False))
        np.testing.assert_allclose(result['nozzle'], BASE_XYZ, atol=1e-12)
        np.testing.assert_allclose(result['joints'], BASE_JOINTS, atol=1e-12)

    def test_zero_velocity_does_not_hide_slow_position_drift(self):
        rows = perturb_z(trace(), lambda t: .05 * t)
        with self.assertRaises(StationaryMotionError) as caught:
            self.estimate(rows)
        self.assertFalse(caught.exception.waitable)
        self.assertIn('measured ', str(caught.exception))
        self.assertIn('microradians)', str(caught.exception))

    def test_zero_median_coherent_oscillation_is_rejected(self):
        for frequency in (5., 35.):
            with self.subTest(frequency=frequency):
                rows = perturb_z(trace(), lambda t: .03 * math.sin(2 * math.pi * frequency * t))
                with self.assertRaises(StationaryMotionError):
                    self.estimate(rows)

    def test_balanced_large_jitter_cannot_hide_inside_block_medians(self):
        rows = trace(noise=False)
        # Keep every joint excursion below 100 microradians so the independent
        # raw Cartesian bound, not the raw joint bound, must catch this case.
        direction = np.array([1. / 3000., 0., 0., 0., 0., 0.])
        rows = [replace(row, joints=np.asarray(row.joints) +
                        direction * (0. if index % 25 == 24 else -.2 if index % 25 < 12 else .2))
                for index, row in enumerate(rows)]
        # All 50 ms block medians can be near zero while every raw pose is wrong.
        def stretched_fk(joints):
            return BASE_XYZ + np.array([3000. * (joints[0] - BASE_JOINTS[0]), 0., 0.])
        with self.assertRaisesRegex(StationaryError, 'Raw stationary pose excursion'):
            estimate_stationary(rows, rows[-1].received_at + .005, stretched_fk,
                                stopped_at=10., terminal_confirmed=True)

    def test_single_position_spike_is_rejected(self):
        rows = trace()
        rows[-100] = replace(rows[-100], joints=rows[-100].joints + Z_DIRECTION * .2)
        with self.assertRaises(StationaryError):
            self.estimate(rows)

    def test_reported_motion_is_rejected_even_with_constant_positions(self):
        rows = trace(noise=False)
        rows[-100] = replace(rows[-100], velocity=np.array([0., .000379931, 0., 0., 0., 0.]))
        with self.assertRaises(StationaryMotionError) as caught:
            self.estimate(rows)
        self.assertFalse(caught.exception.waitable)
        self.assertIn('peak 0.000379931 rad/s; latest 0 rad/s', str(caught.exception))

    def test_velocity_limit_is_not_relaxed_for_initial_settling(self):
        rows = [replace(row, velocity=np.full(6, MAX_VELOCITY_RAD_S))
                for row in trace(noise=False)]
        self.estimate(rows)
        rows[-1] = replace(rows[-1], velocity=np.full(6, MAX_VELOCITY_RAD_S*1.001))
        with self.assertRaises(StationaryMotionError):
            self.estimate(rows)

    def test_missing_malformed_and_nonfinite_joint_or_velocity_data_is_rejected(self):
        for field, value in (('velocity', None), ('velocity', []), ('velocity', [0.] * 5),
                             ('velocity', [0., 0., float('nan'), 0., 0., 0.]),
                             ('joints', [0.] * 5),
                             ('joints', [0., 0., float('inf'), 0., 0., 0.])):
            with self.subTest(field=field, value=value), self.assertRaises(StationaryError):
                rows = trace()
                rows[-100] = replace(rows[-100], **{field: value})
                self.estimate(rows)

    def test_duplicate_reversed_and_invalid_source_or_arrival_times_are_rejected(self):
        for field in ('received_at', 'source_time'):
            for kind in ('duplicate', 'backward', 'nonfinite'):
                with self.subTest(field=field, kind=kind), self.assertRaises(StationaryError):
                    rows = trace()
                    value = getattr(rows[-101], field)
                    value = value if kind == 'duplicate' else value - .001 if kind == 'backward' else float('nan')
                    rows[-100] = replace(rows[-100], **{field: value})
                    self.estimate(rows)

    def test_missing_feedback_interval_is_rejected(self):
        rows = trace()
        del rows[-160:-130]
        with self.assertRaises(StationaryError):
            self.estimate(rows)

    def test_underpopulated_block_rejects_whole_window_until_replacement_is_available(self):
        rows=trace(noise=False)
        start=rows[-1].received_at-WINDOW_S
        block=[s for s in rows if start+.3<=s.received_at<start+.35]
        kept={s.received_at for s in block[::4]}
        sparse=[s for s in rows if not start+.3<=s.received_at<start+.35 or s.received_at in kept]
        with self.assertRaisesRegex(StationaryError, 'Too few distinct observations') as caught:
            self.estimate(sparse)
        self.assertTrue(caught.exception.waitable)

    def test_source_delivery_skew_is_rejected(self):
        rows = trace()
        # Source clock advances differently while receive times continue normally.
        rows = [replace(row, source_time=row.source_time + (0.03 if index > 250 else 0.))
                for index, row in enumerate(rows)]
        with self.assertRaises(StationaryError):
            self.estimate(rows)

    def test_old_source_samples_cannot_look_fresh_from_recent_arrival_times(self):
        for source_age in (.2, -.001, float('nan'), None):
            with self.subTest(source_age=source_age), self.assertRaises(StationaryError):
                rows = [replace(row, source_age_s=source_age) for row in trace()]
                self.estimate(rows)

    def test_stale_future_or_insufficient_feedback_is_rejected(self):
        rows = trace()
        for now in (rows[-1].received_at+.1, rows[-1].received_at-.1):
            with self.subTest(now=now), self.assertRaises(StationaryError):
                self.estimate(trace(), now=now)
        with self.assertRaises(StationaryError):
            self.estimate(trace()[-40:])

    def test_terminal_stop_proof_and_post_stop_observations_are_required(self):
        with self.assertRaises(StationaryError):
            self.estimate(trace(), terminal_confirmed=False)
        with self.assertRaises(StationaryError):
            self.estimate(trace(), stopped_at=10.3)


if __name__ == '__main__':
    unittest.main()
