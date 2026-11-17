from pathlib import Path
import tempfile
import unittest
import yaml

from bridge_dashboard.probe_options import SavedMesh


class SavedMeshTests(unittest.TestCase):
    def test_replacing_active_mesh_refreshes_status_and_temperature_metadata(self):
        with tempfile.TemporaryDirectory() as folder:
            config=Path(folder)/'config.yaml';active=Path(folder)/'bed.yaml'
            config.write_text(yaml.safe_dump({'gcode_interpreter':{'ros__parameters':{'bed_probe_file':str(active)}}}))
            saved=SavedMesh(config)
            self.assertFalse(saved.status()['available'])
            active.write_text('points: [1, 2, 3, 4]\nstamp: original\ncomplete: true\n')
            self.assertEqual(saved.status()['points'],4)
            replacement=active.with_suffix('.new')
            replacement.write_text(yaml.safe_dump({'points':[1]*9,'stamp':'heated mesh','complete':True,
                'probing_temperature':{'mode':'heated','options':{'nozzle_target_c':210.,'bed_target_c':60.,'soak_s':60.}}}))
            replacement.replace(active)
            current=saved.status()
            self.assertEqual(current['points'],9)
            self.assertEqual(current['temperature']['options']['bed_target_c'],60.)
