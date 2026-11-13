"""TCP changes preserve configuration, obey robot ownership and invalidate old geometry."""
import fcntl
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch
import yaml

from bridge_dashboard.tcp_settings import TcpSettings,TcpSettingsError,TcpSettingsConflict

CONFIG='''gcode_interpreter:
  ros__parameters:
    # keep this comment
    tcp_offset_mm: [-100.00, 0.00, 117.06] # nozzle offset
    bed_probe_file: ""
    wrist_angle: 90.0 # never change this
    nozzle_roll_deg: 180.0
other_node:
  ros__parameters:
    tcp_offset_mm: [1, 2, 3]
'''


class TcpSettingsTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.config=self.root/'config.yaml';self.config.write_text(CONFIG)
        self.store=TcpSettings(self.config,lock_path=self.root/'robot.lock')
        self.invalidate=Mock()

    def save(self,values=None,revision=None):
        return self.store.save(values or [-101.,2.,118.],revision or self.store.status()['revision'],self.invalidate)

    def test_save_changes_only_planner_offset_and_preserves_comments_backup_and_mode(self):
        self.config.chmod(0o640)
        before=self.store.status()
        result=self.save()
        self.assertTrue(result['changed']);self.assertFalse(result['controller_tcp_changed'])
        self.assertEqual(result['tcp_offset_mm'],[-101.,2.,118.])
        self.assertNotEqual(result['revision'],before['revision'])
        self.assertEqual(self.config.read_text(),CONFIG.replace('[-100.00, 0.00, 117.06]','[-101.0, 2.0, 118.0]'))
        self.assertEqual(self.config.stat().st_mode & 0o777,0o640)
        self.assertEqual(self.config.with_name('config.yaml.tcp-backup').read_text(),CONFIG)
        self.invalidate.assert_called_once()
        self.assertFalse(list(self.root.glob('.config*')))

    def test_noop_preserves_preflight_and_stale_editor_cannot_overwrite_changes(self):
        revision=self.store.status()['revision']
        result=self.save([-100.,0.,117.06]);self.assertFalse(result['changed'])
        self.invalidate.assert_not_called()
        self.config.write_text(CONFIG+'# external editor\n')
        with self.assertRaisesRegex(TcpSettingsConflict,'changed since'):self.save(revision=revision)
        self.assertIn('external editor',self.config.read_text())
        self.invalidate.assert_not_called()

    def test_external_robot_owner_blocks_save_even_without_dashboard_operation(self):
        with open(self.store.lock_path,'w') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaisesRegex(TcpSettingsConflict,'owned'):self.save()
        self.assertEqual(self.config.read_text(),CONFIG)
        self.invalidate.assert_not_called()

    def test_bad_numbers_duplicate_keys_and_aliases_are_rejected(self):
        for values in ([True,0.,0.],[float('nan'),0.,0.],[0.,float('inf'),0.],[1001.,0.,0.],[1.,2.]):
            with self.subTest(values=values),self.assertRaises(TcpSettingsError):self.save(values)
        for text in (CONFIG.replace('tcp_offset_mm: [-100.00','tcp_offset_mm: &offset [-100.00'),
                     CONFIG.replace('    wrist_angle:', '    tcp_offset_mm: [0,0,0]\n    wrist_angle:')):
            self.config.write_text(text)
            with self.assertRaises(TcpSettingsError):self.store.status()
        self.invalidate.assert_not_called()

    def test_block_list_is_updated_without_touching_other_parameters(self):
        self.config.write_text(CONFIG.replace('[-100.00, 0.00, 117.06] # nozzle offset','\n      - -100.0\n      - 0.0\n      - 117.06'))
        before=yaml.safe_load(self.config.read_text());before['gcode_interpreter']['ros__parameters']['tcp_offset_mm']=[-101.,2.,118.]
        self.save()
        self.assertEqual(yaml.safe_load(self.config.read_text()),before)
        self.assertIn('# never change this',self.config.read_text())

    def test_existing_mesh_is_preserved_and_mismatched_tcp_requests_reprobe(self):
        mesh=self.root/'bed.yaml';mesh.write_text('tcp_offset_mm: [-100, 0, 117.06]\ncomplete: true\n')
        self.config.write_text(CONFIG.replace('bed_probe_file: ""',f'bed_probe_file: {mesh}'))
        original=mesh.read_bytes()
        self.assertFalse(self.store.status()['bed_mesh']['requires_probe'])
        result=self.save();self.assertTrue(result['bed_mesh']['requires_probe'])
        self.assertEqual(mesh.read_bytes(),original)
        self.save([-100.,0.,117.06]);self.assertFalse(self.store.status()['bed_mesh']['requires_probe'])

    def test_failed_replacement_keeps_original_config_and_preflight_invalidated(self):
        actual=self.store._atomic_write
        def write(path,*args):
            if path==self.config:raise OSError('read-only disk')
            return actual(path,*args)
        with patch.object(self.store,'_atomic_write',side_effect=write):
            with self.assertRaisesRegex(TcpSettingsError,'Could not save'):self.save()
        self.assertEqual(self.config.read_text(),CONFIG)
        self.invalidate.assert_called_once()


if __name__=='__main__':unittest.main()
