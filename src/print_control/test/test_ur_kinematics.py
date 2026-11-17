"""Calibrated UR transforms, Jacobian/IK and mesh provenance."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
import numpy as np
import yaml
import roboticstoolbox as rtb
from spatialmath import SE3
from print_control.motion_planner import MotionConfig,MotionPlanner
from print_control.ur_kinematics import LINKS,calibrated_model
from print_control.bed_surface import BedSurface


class CalibratedKinematicsTests(unittest.TestCase):
    def setUp(self):
        root=tempfile.TemporaryDirectory();self.addCleanup(root.cleanup)
        self.path=Path(root.name)/'calibration.yaml'
        h=np.pi/2
        self.data={'kinematics':dict(zip(LINKS,[dict(zip(('x','y','z','roll','pitch','yaw'),row)) for row in
            ((0,0,.1807,0,0,0),(0,0,0,h,0,0),(-.4784,0,0,0,0,0),
             (-.36,0,.17415,0,0,0),(0,-.11985,0,h,0,0),(0,.11655,0,h,np.pi,np.pi))]))}
        self.data['kinematics']['hash']='calib_test'
        self.write()

    def write(self):self.path.write_text(yaml.safe_dump(self.data))

    def planner(self):return MotionPlanner(MotionConfig(kinematics_file=str(self.path),tcp_offset_mm=(-100.,0.,117.06)))

    def test_nominal_description_reproduces_dh_base_and_tool0(self):
        p=self.planner();reference=rtb.models.DH.UR16e();rng=np.random.default_rng(421)
        for q in rng.uniform(-np.pi,np.pi,(50,6)):
            np.testing.assert_allclose(p.kinematics.eval(q),reference.fkine(q).A,atol=1e-12)
            np.testing.assert_allclose(p.robot.jacob0(q),reference.jacob0(q),atol=1e-12)
        np.testing.assert_allclose(p.joint_limits,np.deg2rad([[-360,-360,-180,-360,-360,-360],[360,360,180,360,360,360]]))

    def test_calibration_changes_fk_and_preserves_ik_and_jacobian(self):
        self.data['kinematics']['shoulder']['yaw']=.003
        self.data['kinematics']['upper_arm']['x']=.0002
        self.write();p=self.planner();q=np.array([.1,-1.,1.6,-2.2,-1.57,.1])
        transform=p.kinematics.eval(q);reference=rtb.models.DH.UR16e().fkine(q).A
        self.assertGreater(np.max(abs(transform-reference)),1e-4)
        tip=p.nozzle_position(q);solution=p.solve(tip,q+.0001,transform[:3,:3])
        np.testing.assert_allclose(p.nozzle_position(solution),tip,atol=.001)
        eps=1e-6
        for j in range(6):
            dq=np.eye(6)[j]*eps
            finite=(p.kinematics.eval(q+dq)[:3,3]-p.kinematics.eval(q-dq)[:3,3])/(2*eps)
            np.testing.assert_allclose(p.robot.jacob0(q)[:3,j],finite,atol=1e-9)
        self.assertEqual(len(p.link_kinematics),7)

    def test_invalid_or_missing_calibration_never_falls_back_to_generic_model(self):
        self.data['kinematics']['wrist_1']['roll']=float('nan');self.write()
        with self.assertRaises(ValueError):self.planner()
        self.path.unlink()
        with self.assertRaises(OSError):self.planner()
        with self.assertRaises(ValueError):MotionConfig(kinematics_file=None).validate()

    def test_mesh_requires_the_same_model_identity(self):
        p=self.planner();mesh=self.path.with_name('mesh.yaml')
        data={'tcp_offset_mm':[0.,0.,0.],'nozzle_rpy_deg':[180.,0.,90.],
              'points':[{'nozzle_base_mm':[x,y,100.],'bed_z_mm':100.} for x,y in ((0,0),(50,0),(0,50),(50,50))]}
        def load():return BedSurface.load(mesh,(0.,0.,0.),(180.,0.,90.),5.,expected_model=p.kinematic_model)
        mesh.write_text(yaml.safe_dump(data))
        with self.assertRaisesRegex(ValueError,'kinematic model changed'):load()
        data['kinematic_model']=p.kinematic_model;mesh.write_text(yaml.safe_dump(data));load()
        self.data['kinematics']['shoulder']['yaw']=.0001;self.write();p=self.planner()
        with self.assertRaisesRegex(ValueError,'kinematic model changed'):load()
