"""Exercise primary contact selection and motion ordering without hardware."""
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest

from print_control.load_cell_bed_probe import LoadCellBedProbe
from print_control.load_cell_contact import ContactDetected, ContactDetector, ContactOptions, ForceSample
from print_control.motion_executor import MotionExecutor
from test_continuous_probe import ContinuousContactTests


class NativeSelectionTests(unittest.TestCase):
    def setUp(self):
        self.probe = object.__new__(LoadCellBedProbe)
        self.probe.seeking = True
        self.probe.detector = ContactDetector(0., 100., ContactOptions(
            trigger_mode='raw', trigger_source='klipper_probe'))
        self.native = Mock()
        self.native.check.return_value = False
        self.samples = [ForceSample(i, i*.002, i*.002, 25000.) for i in range(5)]
        self.probe.source = SimpleNamespace(snapshot=lambda: self.samples, native_trigger=self.native)
        self.addCleanup(patch.stopall)
        patch.object(MotionExecutor, 'check_interrupted').start()

    def test_raw_contact_does_not_substitute_for_native_event(self):
        self.probe.check_interrupted()
        self.assertTrue(self.probe.detector.triggered)
        self.native.check.assert_called_once()

    def test_native_event_stops_before_raw_batch_reaches_threshold(self):
        self.samples = [ForceSample(1, 1., 1., 0.)]
        self.native.check.return_value = True
        with self.assertRaises(ContactDetected): self.probe.check_interrupted()

    def test_dashboard_positioning_uses_raw_contact_even_when_native_probe_is_selected(self):
        self.probe._z_manual_active = True
        self.native.check.return_value = False
        with self.assertRaises(ContactDetected): self.probe.check_interrupted()
        self.native.check.assert_not_called()

    def test_overload_takes_precedence_even_when_native_contact_arrives(self):
        self.samples.append(ForceSample(6, 2., 2., 121000.))
        self.native.check.return_value = True
        with self.assertRaisesRegex(RuntimeError, 'overload'): self.probe.check_interrupted()
        self.native.check.assert_not_called()

    def test_native_connection_failure_never_falls_back_to_raw_contact(self):
        self.native.check.side_effect = RuntimeError('native connection stalled')
        with self.assertRaisesRegex(RuntimeError, 'stalled'): self.probe.check_interrupted()

    def test_raw_overload_guard_stays_active_after_contact_cancellation(self):
        self.probe.seeking = False
        self.samples.append(ForceSample(6, 2., 2., 121000.))
        with self.assertRaisesRegex(RuntimeError, 'overload'): self.probe.check_interrupted()


class NativeMotionOrderingTests(unittest.TestCase):
    run_touch = ContinuousContactTests.run_touch
    assert_cleaned = ContinuousContactTests.assert_cleaned

    def setUp(self):
        ContinuousContactTests.setUp(self)
        self.native = Mock()
        self.native.begin_contact.side_effect = lambda *a, **kw: self.node.events.append(('arm', None))
        self.native.confirm_contact.side_effect = lambda *a: self.node.events.append(('confirm', None))
        self.native.end_contact.side_effect = lambda: self.node.events.append(('disarm', None))
        self.node.source.native_trigger = self.native

    def test_native_arms_before_motion_and_classifies_only_after_cancel(self):
        self.run_touch()
        names = [x[0] for x in self.node.events]
        self.assertLess(names.index('arm'), names.index('send'))
        self.assertLess(names.index('cancel'), names.index('confirm'))
        self.assertLess(names.index('confirm'), names.index('window'))
        self.assertEqual(names[-1], 'disarm')
        self.assertEqual(self.native.begin_contact.call_args.kwargs['search_timeout_s'], 55.)

    def test_arm_failure_prevents_motion_and_cleans_up(self):
        self.native.begin_contact.side_effect = RuntimeError('not calibrated')
        with self.assertRaisesRegex(RuntimeError, 'not calibrated'): self.run_touch()
        self.assertNotIn('send', [x[0] for x in self.node.events])
        self.native.end_contact.assert_called_once()
        self.assert_cleaned()

    def test_fault_classification_after_immediate_stop_rejects_measurement(self):
        self.native.confirm_contact.side_effect = RuntimeError('MCU sensor fault')
        with self.assertRaisesRegex(RuntimeError, 'sensor fault'): self.run_touch()
        self.assertNotIn('window', [x[0] for x in self.node.events])
        self.native.end_contact.assert_called_once()
        self.assert_cleaned()
