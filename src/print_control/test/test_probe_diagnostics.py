"""A slow diagnostic disk write must never block guarded contact recovery."""
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from print_control.bridge_bed_probe import BridgeBedProbe
from print_control.load_cell_contact import ForceSample
from print_control.probe_settling import JointFeedback


class DiagnosticTests(unittest.TestCase):
    def test_disk_write_does_not_block_recovery_and_preserves_failure_observations(self):
        with tempfile.TemporaryDirectory() as directory:
            node = object.__new__(BridgeBedProbe)
            node.config = SimpleNamespace(dry_run=False)
            node.output = Path(directory)/'bed_probe.yaml'
            node._homing_feedback = [JointFeedback(1., 1., np.zeros(6), np.zeros(6), 0.)]
            node.source = SimpleNamespace(lock=threading.Lock(),
                samples=[ForceSample(1, 1., 1., -175000.)],
                received_at=1., health_at=1., error=None)
            node.detector = SimpleNamespace(baseline=-200000.)
            node._continuous_homing = {'terminal_confirmed': True}
            node._point_readings = [{'nozzle':np.array([1.,2.,3.]),'load':19000.},
                                    {'nozzle':np.array([1.,2.,3.04]),'load':21000.}]
            entered, release = threading.Event(), threading.Event()
            write = Path.write_text
            def slow_write(path, contents, *args, **kwargs):
                entered.set()
                if not release.wait(2.):
                    raise TimeoutError('Test did not release background writer')
                return write(path, contents, *args, **kwargs)
            with patch.object(Path, 'write_text', slow_write):
                try:
                    node.record_contact_failure(RuntimeError('contact rejected'))
                    self.assertTrue(entered.wait(1.))
                    self.assertTrue(node._diagnostic_thread.is_alive())
                    self.assertFalse(node.output.with_suffix('.contact-failure.json').exists())
                    # The call has returned while disk I/O is still blocked.
                finally:
                    release.set()
                    node._diagnostic_thread.join(2.)
            self.assertFalse(node._diagnostic_thread.is_alive())
            data = json.loads(node.output.with_suffix('.contact-failure.json').read_text())
            self.assertEqual(data['error'], 'contact rejected')
            self.assertEqual(data['baseline_counts'], -200000.)
            self.assertEqual(data['joints'][0]['joints'], [0.]*6)
            self.assertEqual(data['alps'][0]['counts'], -175000.)
            self.assertEqual([r['nozzle'][2] for r in data['point_readings']],[3.,3.04])
            self.assertFalse(node.output.exists())
