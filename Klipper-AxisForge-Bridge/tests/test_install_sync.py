"""Install into temporary directories; never touch a real Klipper host."""
from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('install_sync', ROOT/'scripts/install_sync.py')
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class InstallSyncTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.klipper = self.root/'klipper'
        self.extras = self.klipper/'klippy/extras'
        self.extras.mkdir(parents=True)
        self.config = self.root/'printer.cfg'
        self.original = '[printer]\nkinematics: none\n#*# <---------------------- SAVE_CONFIG\n#*# saved\n'
        self.config.write_text(self.original)

    def install(self, **kwargs):
        with redirect_stdout(io.StringIO()):
            installer.install(ROOT/'klipper_extras', self.klipper, self.config, **kwargs)

    def test_native_module_is_copied_but_not_enabled_without_flag(self):
        self.install()
        self.assertTrue((self.extras/'axisforge_probe.py').is_file())
        self.assertNotIn('[axisforge_probe]', self.config.read_text())
        backup = list(self.root.glob('axisforge-sync-backup-*/printer.cfg'))
        self.assertEqual(len(backup), 1)
        self.assertEqual(backup[0].read_text(), self.original)

    def test_native_section_is_opt_in_idempotent_and_before_save_config(self):
        self.install(native_probe=True)
        self.install(native_probe=True)
        data = self.config.read_text()
        self.assertEqual(data.count('[axisforge_probe]'), 1)
        self.assertEqual(data.count('[axisforge_sync]'), 1)
        self.assertLess(data.index('[axisforge_probe]'), data.index('#*# <'))

    def test_existing_sections_and_operator_settings_are_preserved(self):
        data = '[axisforge_sync]\nmax_velocity: 8\n[axisforge_probe]\n'+self.original
        self.config.write_text(data)
        self.install(native_probe=True)
        self.assertEqual(self.config.read_text(), data)
