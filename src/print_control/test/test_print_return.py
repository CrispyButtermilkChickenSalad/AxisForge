"""End-of-print geometry and measured tracking; no robot I/O."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
import numpy as np
from spatialmath import SE3
import yaml

from print_control.gcode_path import GCodeConfig, GCodePlanner
from print_control.motion_planner import MotionConfig, PlannedMotion
from print_control.print_return import saved_home, return_motions, lift_motion
from print_control.sync_tracking import PrintTracking


class PrintReturnTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.path=Path(temp.name)/'mesh.yaml'
        self.config=MotionConfig(tcp_offset_mm=(-100.,0.,117.06),clearance_mm=6.)
        self.home=np.array([-627.13,-269.78,120.])
        self.data={'complete':True,'simulated':False,'stamp':'test','tcp_offset_mm':list(self.config.tcp_offset_mm),
            'nozzle_rpy_deg':[180.,0.,90.],'transform':{'bed_size_mm':[235.,235.]},
            'homing':{'returned_to_start':True,'starting_nozzle_base_mm':self.home.tolist()},
            'points':[{'nozzle_base_mm':[x,y,100.],'bed_z_mm':100.} for x,y in
                      [(-700,-350),(-500,-350),(-700,-150),(-500,-150)]]}
        self.path.write_text(yaml.safe_dump(self.data))
        self.settings=GCodeConfig(bed_probe_file=str(self.path),return_to_homing_pose=True,
                                  lookahead_enabled=False,x_offset=-700,y_offset=-350,z_offset=9999.)
        self.planner=GCodePlanner(self.settings,self.config)
        self.seed=np.deg2rad([-3.73,-90,-120,-60,90,0])

    def test_ten_mm_lift_holds_xy_and_tilt_without_saved_home(self):
        settings=replace(self.settings,return_to_homing_pose=False,post_print_lift_mm=10.)
        # A finishing lift must not depend on Home being above the printed part.
        self.data.pop('homing')
        self.path.write_text(yaml.safe_dump(self.data))
        start=self.home+[20.,10.,-10.]
        rotation=SE3.RPY([183.,-2.,90.],unit='deg',order='zyx').R
        q=self.planner.solve(start,self.seed,rotation)
        motions=lift_motion(settings,self.config,q)
        self.assertEqual(len(motions),1)
        measured=self.planner.nozzle_position(q)
        for joints in motions[0].positions:
            xyz=self.planner.nozzle_position(joints)
            np.testing.assert_allclose(xyz[:2],measured[:2],atol=.0025)
            self.assertGreaterEqual(xyz[2],measured[2]-.0025)
            np.testing.assert_allclose(self.planner.kinematics.eval(joints)[:3,:3],rotation,atol=1e-5)
        np.testing.assert_allclose(self.planner.nozzle_position(motions[0].positions[-1]),
                                   measured+[0,0,10.],atol=.0025)
        self.assertIsNone(saved_home(settings))

    def test_lift_cancellation_and_configuration_guards(self):
        settings=replace(self.settings,return_to_homing_pose=False,post_print_lift_mm=10.)
        def cancel():raise RuntimeError('Operator abort')
        with self.assertRaisesRegex(RuntimeError,'Operator abort'):
            lift_motion(settings,self.config,self.seed,cancel)
        for distance in (-1.,51.,float('nan'),True):
            with self.subTest(distance=distance),self.assertRaises(ValueError):
                replace(settings,post_print_lift_mm=distance).validate()
        with self.assertRaisesRegex(ValueError,'either'):
            replace(settings,return_to_homing_pose=True).validate()

    def test_lift_travel_restore_orientation_then_lower_to_saved_home(self):
        start=self.home+[20.,10.,-10.]
        q=self.planner.solve(start,self.seed,SE3.RPY([183.,0.,90.],unit='deg',order='zyx').R)
        motions=return_motions(self.settings,self.config,saved_home(self.settings),q,110.)
        self.assertEqual(len(motions),4)
        ends=[self.planner.nozzle_position(m.positions[-1]) for m in motions]
        np.testing.assert_allclose(ends[0][:2],start[:2],atol=.0025)
        self.assertGreaterEqual(ends[0][2],125.9)
        np.testing.assert_allclose(ends[1],self.home+[0,0,6],atol=.0025)
        np.testing.assert_allclose(ends[2],ends[1],atol=.0025)
        self.assertGreaterEqual(motions[2].times[-1],3.)
        np.testing.assert_allclose(ends[-1],self.home,atol=.0025)
        np.testing.assert_allclose(self.planner.kinematics.eval(motions[-1].positions[-1])[:3,:3],
                                   self.planner.rotation,atol=1e-5)

    def test_low_home_or_untrusted_saved_pose_blocks_return_before_motion(self):
        q=self.planner.solve(self.home,self.seed)
        with self.assertRaisesRegex(ValueError,'too low'):
            return_motions(self.settings,self.config,saved_home(self.settings),q,115.)
        for update in ({'complete':False},{'homing':{'returned_to_start':False}},
                       {'homing':{'returned_to_start':True,'starting_nozzle_base_mm':[0,0,float('nan')]}}):
            self.path.write_text(yaml.safe_dump(dict(self.data,**update)))
            with self.subTest(update=update),self.assertRaises(ValueError):saved_home(self.settings)

    def test_cancelled_planning_does_not_produce_return_motion(self):
        q=self.planner.solve(self.home,self.seed)
        def cancel():raise RuntimeError('Operator abort')
        with self.assertRaisesRegex(RuntimeError,'Operator abort'):
            return_motions(self.settings,self.config,saved_home(self.settings),q,110.,cancel)
        self.assertIsNone(saved_home(replace(self.settings,return_to_homing_pose=False)))

    def test_zero_probe_offset_keeps_first_and_subsequent_layer_heights(self):
        for height in (.24,.44,.49,.54,.64,.74,1.14):
            # The old flat-bed estimate (9999 here) must not be added to a mesh.
            self.assertAlmostEqual(self.planner.transform([100.,100.,height])[2],100.+height)
        adjusted=GCodePlanner(replace(self.settings,bed_z_adjust_mm=.1),self.config)
        self.assertAlmostEqual(adjusted.transform([100.,100.,.44])[2],100.54)


class TrackingTests(unittest.TestCase):
    def test_measures_z_error_and_lag_at_observation_time(self):
        q=np.zeros((2,6));q[1,0]=.02
        v=np.zeros((2,6));v[:,0]=.02
        motion=PlannedMotion('test',q,v,np.array([0.,1.]))
        observed=np.zeros(6);observed[0]=.0094;observed[2]=.00005
        track=PrintTracking();track.observe(motion,.5,observed,lambda q:q[:3]*1000.)
        status=track.status()
        self.assertAlmostEqual(status['max_z_error_mm'],.05)
        self.assertAlmostEqual(status['mean_estimated_lag_ms'],30.)
        self.assertEqual(status['samples'],1)
        track.observe(motion,-.1,observed,lambda q:q[:3]*1000.)
        self.assertEqual(track.status()['samples'],1)


if __name__=='__main__':unittest.main()
