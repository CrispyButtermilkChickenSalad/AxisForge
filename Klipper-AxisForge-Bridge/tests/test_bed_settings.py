"""Bed resizing preserves the calibrated corner, ownership and configuration revision."""
import fcntl
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

import yaml

from bridge_dashboard.bed_settings import BedSettings, BedSettingsError, BedSettingsConflict

CONFIG = '''gcode_interpreter:
  ros__parameters:
    origin_at_center: true
    bed_size_mm: [235.0, 235.0] # physical bed
    bed_probe_margin_mm: 30.0
    x_offset: -854.63
    y_offset: -497.28
    z_offset: 100.0
    tcp_offset_mm: [-100.0, 0.0, 117.06]
    bed_probe_file: ""
'''


class BedSettingsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root/'config.yaml'; self.path.write_text(CONFIG)
        self.store = BedSettings(self.path, lock_path=self.root/'robot.lock')
        self.invalidate = Mock()

    def save(self, width=150., depth=100., margin=10., revision=None):
        return self.store.save(width, depth, margin, revision or self.store.status()['revision'], self.invalidate)

    def test_resize_keeps_physical_front_left_and_preserves_unrelated_parameters(self):
        self.path.chmod(0o640)
        before = self.store.status()
        result = self.save()
        self.assertEqual(result['front_left_base_mm'], before['front_left_base_mm'])
        self.assertEqual(result['probe_area'], {'x':[10.,140.], 'y':[10.,90.]})
        previous = yaml.safe_load(CONFIG)['gcode_interpreter']['ros__parameters']
        current = yaml.safe_load(self.path.read_text())['gcode_interpreter']['ros__parameters']
        changed = {k for k in previous if current[k] != previous[k]}
        self.assertEqual(changed, {'bed_size_mm','bed_probe_margin_mm','x_offset','y_offset'})
        self.assertIn('# physical bed', self.path.read_text())
        self.assertEqual(self.path.stat().st_mode & 0o777,0o640)
        self.assertEqual(self.path.with_name('config.yaml.bed-backup').read_text(),CONFIG)
        self.invalidate.assert_called_once()

    def test_corner_origin_needs_no_offset_adjustment_and_zero_margin_reaches_edges(self):
        self.path.write_text(CONFIG.replace('origin_at_center: true','origin_at_center: false'))
        before = self.store.status()
        result = self.save(20.,10.,0.)
        self.assertEqual(result['front_left_base_mm'],before['front_left_base_mm'])
        self.assertEqual(result['probe_area'],{'x':[0.,20.], 'y':[0.,10.]})
        self.assertIn('x_offset: -854.63',self.path.read_text())

    def test_revision_and_external_robot_owner_prevent_lost_writes(self):
        revision = self.store.status()['revision']
        self.path.write_text(CONFIG+'# changed elsewhere\n')
        with self.assertRaises(BedSettingsConflict): self.save(revision=revision)
        with self.store.lock_path.open('w') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaises(BedSettingsConflict): self.save()
        self.invalidate.assert_not_called()

    def test_noop_and_invalid_values_never_change_configuration(self):
        self.assertFalse(self.save(235.,235.,30.)['changed'])
        for values in ((True,100.,10.),(100.,float('nan'),10.),(9.,10.,0.),
                       (150.,100.,50.),(150.,100.,-1.),('150',100.,10.)):
            with self.subTest(values=values), self.assertRaises(BedSettingsError): self.save(*values)
        self.assertEqual(self.path.read_text(),CONFIG)
        self.invalidate.assert_not_called()

    def test_dimension_mismatch_recommends_reprobe_without_touching_existing_map(self):
        mesh = self.root/'mesh.yaml'
        mesh.write_text(yaml.safe_dump({'transform':{'bed_size_mm':[235.,235.]}}))
        original = mesh.read_bytes()
        self.path.write_text(CONFIG.replace('bed_probe_file: ""','bed_probe_file: '+str(mesh)))
        self.assertFalse(self.store.status()['bed_mesh']['requires_probe'])
        result = self.save()
        self.assertTrue(result['bed_mesh']['requires_probe'])
        self.assertEqual(mesh.read_bytes(),original)

    def test_malformed_or_aliased_dimensions_fail_closed(self):
        for text in (CONFIG.replace('bed_size_mm:', 'bed_size_mm: &bed'),
                     CONFIG.replace('bed_size_mm: [235.0, 235.0]', 'bed_size_mm: [235]'),
                     CONFIG.replace('    bed_probe_margin_mm: 30.0','    bed_probe_margin_mm: 30.0\n    bed_probe_margin_mm: 10.0')):
            self.path.write_text(text)
            with self.assertRaises(BedSettingsError): self.store.status()
