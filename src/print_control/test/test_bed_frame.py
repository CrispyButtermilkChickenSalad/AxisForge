"""Centred probing and printing must share the same physical XY frame."""
from pathlib import Path
import tempfile
import unittest
import numpy as np
import yaml
from print_control.bed_frame import BedFrame
from print_control.bed_surface import BedSurface
from print_control.gcode_path import GCodeConfig,GCodePlanner
from print_control.motion_planner import MotionConfig


class BedFrameTests(unittest.TestCase):
    def test_size_margin_and_center_have_distinct_meanings(self):
        frame=BedFrame((0.,0.),(200.,200.))
        np.testing.assert_allclose(frame.to_base((0.,0.)),(-100.,-100.))
        np.testing.assert_allclose(frame.to_base((50.,50.)),(-50.,-50.))
        frame=BedFrame((-640.,-385.),(220.,220.))
        np.testing.assert_allclose(frame.to_base((30.,30.)),(-720.,-465.))
        np.testing.assert_allclose(frame.to_base((110.,110.)),(-640.,-385.))

    def test_rotated_grid_round_trips_without_moving_centre(self):
        for yaw in (-180.,-90.,0.,90.,180.,37.):
            frame=BedFrame((-640.,-385.),(220.,180.),yaw)
            for xy in ((0.,0.),(30.,30.),(110.,90.),(220.,180.)):
                np.testing.assert_allclose(frame.to_gcode(frame.to_base(xy)),xy,atol=1e-10)
            np.testing.assert_allclose(frame.to_base((110.,90.)),frame.center_base_mm)
        np.testing.assert_allclose(BedFrame((0.,0.),(200.,200.),90.).to_base((30.,30.)),(70.,-70.))

    def mesh(self,path,frame):
        points=[]
        for xy in ((30.,30.),(190.,30.),(190.,190.),(30.,190.)):
            base=frame.to_base(xy)
            points.append({'gcode_xy':list(xy),'nozzle_base_mm':[float(base[0]),float(base[1]),100.],
                           'bed_z_mm':100.})
        data={'complete':True,'simulated':False,'tcp_offset_mm':[0.,0.,0.],
              'nozzle_rpy_deg':[180.,0.,90.],'transform':{'bed_size_mm':[220.,220.]},
              'bed_frame':frame.as_dict(),'points':points}
        path.write_text(yaml.safe_dump(data));return data

    def test_print_planner_uses_measured_centre_instead_of_legacy_offsets(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'mesh.yaml';frame=BedFrame((-640.,-385.),(220.,220.),90.)
            self.mesh(path,frame)
            settings=GCodeConfig(bed_probe_file=str(path),bed_size_mm=(220.,220.),
                x_offset=9999.,y_offset=9999.,z_offset=-500.,bed_z_adjust_mm=.1)
            planner=GCodePlanner(settings,MotionConfig())
            np.testing.assert_allclose(planner.transform((110.,110.,.2)),(-640.,-385.,100.3))
            np.testing.assert_allclose(planner.transform((30.,30.,0.))[:2],frame.to_base((30.,30.)))

    def test_mismatched_point_placement_or_size_rejects_calibrated_frame(self):
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/'mesh.yaml';frame=BedFrame((-640.,-385.),(220.,220.))
            for fault in ('point','size','yaw'):
                data=self.mesh(path,frame)
                if fault=='point':data['points'][0]['gcode_xy'][0]+=1.
                elif fault=='size':data['bed_frame']['size_mm'][0]=200.
                else:data['bed_frame']['yaw_deg']=float('nan')
                path.write_text(yaml.safe_dump(data))
                with self.subTest(fault=fault),self.assertRaises(ValueError):
                    BedSurface.load(path,(0.,0.,0.),(180.,0.,90.),5.,expected_bed_size=(220.,220.))
