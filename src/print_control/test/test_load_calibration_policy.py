"""Calibration thermal readiness and typed load limits without hardware or ROS."""
from types import SimpleNamespace
import unittest

from print_control.load_calibration_policy import CalibrationPolicy, CalibrationDetector
from print_control.load_cell_contact import ContactOptions, ForceSample, LoadCellOverload, validate_printer


def status(temperature=25., target=0.):
    return {'webhooks': {'state': 'ready'}, 'load_cell_probe': {'is_calibrated': False},
            'extruder': {'temperature': temperature, 'target': target}, 'heater_bed': {'target': 0.},
            'axisforge_sync': {}}


class CalibrationPolicyTests(unittest.TestCase):
    def setUp(self):
        self.now = [0.]
        self.options = ContactOptions(trigger_mode='raw')
        self.policy = CalibrationPolicy(hot_target=210., clock=lambda: self.now[0])

    def feed(self, temperature=210., target=210.):
        return self.policy.validate_status(status(temperature, target), 'load_cell_probe', self.options)

    def stabilize(self):
        for i in range(121):
            self.now[0] = i*.25
            self.feed()
        self.policy.require_ready()

    def test_cold_remains_default_and_global_bed_probe_thermal_guard_is_unchanged(self):
        cold = CalibrationPolicy(clock=lambda: self.now[0])
        cold.validate_status(status(), 'load_cell_probe', self.options)
        self.assertTrue(cold.ready())
        with self.assertRaisesRegex(RuntimeError, 'heater targets off'):
            cold.validate_status(status(210., 210.), 'load_cell_probe', self.options)
        with self.assertRaisesRegex(RuntimeError, 'heater targets off'):
            validate_printer(status(210., 210.), 'load_cell_probe', self.options)
        with self.assertRaisesRegex(RuntimeError, 'cool'):
            CalibrationPolicy().validate_status(status(51., 0.), 'load_cell_probe', self.options)

    def test_hot_requires_thirty_seconds_of_fresh_in_band_status(self):
        self.feed(205.)
        self.assertFalse(self.policy.ready())
        for i in range(120):
            self.now[0] = i*.25
            self.feed()
        self.assertFalse(self.policy.ready())
        self.now[0] = 30.
        self.feed()
        self.policy.require_ready()
        self.assertEqual(self.policy.status()['target_c'], 210.)
        self.assertEqual(self.policy.status()['stable_for_s'], 30.)
        self.assertTrue(self.policy.status()['hot_calibration'])

    def test_old_or_interrupted_status_cannot_establish_stability(self):
        self.feed()
        self.now[0] = 31.
        self.assertFalse(self.policy.ready())
        self.feed()
        self.assertFalse(self.policy.ready())
        self.assertEqual(self.policy.status()['stable_for_s'], 0.)
        self.now[0] += .25
        self.feed()
        self.now[0] += .25
        self.feed(213.)
        self.assertFalse(self.policy.ready())
        self.assertEqual(self.policy.status()['stable_for_s'], 0.)

    def test_locked_policy_latches_target_temperature_and_stale_health_faults(self):
        for change in ('temperature', 'target', 'stale'):
            with self.subTest(change=change):
                self.policy = CalibrationPolicy(hot_target=210., clock=lambda: self.now[0])
                self.stabilize()
                self.now[0] += 1.1 if change == 'stale' else .25
                with self.assertRaises(RuntimeError):
                    self.feed(213. if change == 'temperature' else 210.,
                              209. if change == 'target' else 210.)
                with self.assertRaises(RuntimeError):
                    self.policy.require_ready()

    def test_print_ownership_sensor_and_bed_heater_interlocks_remain(self):
        for key, value in (('axisforge_sync', {'leased': True}), ('print_stats', {'state': 'printing'}),
                           ('load_cell_probe', {'errors': 1}), ('webhooks', {'state': 'shutdown'}),
                           ('heater_bed', {'target': 60.})):
            state = status(210., 210.); state[key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                self.policy.validate_status(state, 'load_cell_probe', self.options)

    def test_policy_bounds_reject_unsafe_or_ambiguous_inputs(self):
        for value in (True, 89999., 250001., float('nan'), float('inf')):
            with self.subTest(counts=value), self.assertRaises(ValueError):
                CalibrationPolicy(max_counts=value)
        for value in (False, 49.9, 300.1, float('nan')):
            with self.subTest(target=value), self.assertRaises(ValueError):
                CalibrationPolicy(hot_target=value)
        self.assertEqual(CalibrationPolicy().max_counts, 120000.)
        self.assertEqual(ContactOptions(trigger_mode='raw').maximum, 120000.)

    def test_readiness_wait_services_control_and_sensor_without_motion(self):
        self.feed()
        events, checks, samples = [], [], []
        def advance(_):
            self.now[0] += .25
            self.feed()
        control = SimpleNamespace(check=lambda: checks.append(True),
                                  emit=lambda *args, **kwargs: events.append((args, kwargs)))
        self.policy.wait_ready(SimpleNamespace(snapshot=lambda: samples.append(True)), control, sleep=advance)
        self.assertGreaterEqual(self.now[0], 30.)
        self.assertEqual(len(checks), len(samples))
        self.assertTrue(events)
        self.assertEqual(events[0][0][0], 'waiting_temperature')
        self.assertTrue(self.policy.locked)

    def test_recoverable_detector_scans_remaining_batch_without_retaring(self):
        options = ContactOptions(trigger_mode='raw', max_delta_counts=120000.)
        limits, loads = [], []
        detector = CalibrationDetector(1000., None, options, limits.append, loads.append)
        samples = [ForceSample(i, float(i), float(i), v) for i, v in enumerate((122000., 124000., 126000.), 1)]
        detector.update(samples)
        self.assertEqual([e.load for e in limits], [121000., 123000., 125000.])
        self.assertTrue(all(isinstance(e, LoadCellOverload) for e in limits))
        self.assertEqual(detector.last_sequence, 3)
        self.assertEqual(detector.baseline, 1000.)
        self.assertEqual(detector.peak_load, 125000.)
        detector.update(samples)
        self.assertEqual(len(limits), 3)
        self.assertEqual(loads, [121000., 123000., 125000.])

    def test_unrelated_detector_failure_is_not_recoverable(self):
        def fault(load): raise RuntimeError('thermal failure')
        detector = CalibrationDetector(0., None, self.options, lambda error: None, fault)
        with self.assertRaisesRegex(RuntimeError, 'thermal failure'):
            detector.update([ForceSample(1, 1., 1., 200000.)])


if __name__ == '__main__':
    unittest.main()
