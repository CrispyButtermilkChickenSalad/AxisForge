"""The next planner/probe reads saved TCP and rejects geometry made with the old TCP."""
from pathlib import Path
import tempfile
import unittest
import yaml
from print_control.bed_probe import load_configs
from print_control.bed_surface import BedSurface
from print_control.sync_path import configuration


class TcpReloadTests(unittest.TestCase):
    def test_next_job_and_probe_load_saved_offset_without_restarting_worker(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'config.yaml'
            def save(tcp):path.write_text(yaml.safe_dump({'gcode_interpreter':{'ros__parameters':{'tcp_offset_mm':tcp,'bed_probe_file':''}}}))
            save([-100.,0.,117.06]);before=configuration(path)
            save([-101.,2.,118.]);after=configuration(path)
            self.assertNotEqual(before[2],after[2])
            self.assertEqual(list(after[1].tcp_offset_mm),[-101.,2.,118.])
            self.assertEqual(list(load_configs(path,True)[1].tcp_offset_mm),[-101.,2.,118.])

    def test_previous_mesh_is_rejected_for_changed_tcp(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'bed.yaml'
            path.write_text(yaml.safe_dump({'tcp_offset_mm':[-100.,0.,117.06],'nozzle_rpy_deg':[180.,0.,90.],
                'points':[{'nozzle_base_mm':[x,y,100.],'bed_z_mm':100.} for x,y in ((0.,0.),(20.,0.),(0.,20.))]}))
            with self.assertRaisesRegex(ValueError,'probe the bed again'):
                BedSurface.load(path,[-101.,2.,118.],[180.,0.,90.],1.)
