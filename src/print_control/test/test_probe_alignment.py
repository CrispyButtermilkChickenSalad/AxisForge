"""Real IK and guarded alignment orchestration, with no robot/network I/O."""
import io
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import numpy as np
from spatialmath import SE3

from test_probe_orientation import OrientationHarness
from test_probe_z_moves import Source
from print_control.bridge_bed_probe import BridgeBedProbe, HomingOptions
from print_control.bed_probe import BedProbe, load_configs
from print_control.gcode_path import GCodePlanner
from print_control.load_cell_bed_probe import LoadCellBedProbe
from print_control.load_cell_contact import ContactDetector, ContactOptions, ContactDetected, LoadCellOverload
from print_control.motion_planner import cubic_joint_state
from print_control.probe_alignment import rotation_error_deg
from print_control.probe_control import ProbeControl


class AlignmentHarness(OrientationHarness):
    def __init__(self, tilt=.255, *, calibrated=False):
        super().__init__()
        if calibrated:
            path=Path(__file__).resolve().parents[2]/'axisforge_bringup/config/ur_master.yaml'
            self.settings,self.config=load_configs(path,False)
            self.planner=GCodePlanner(self.settings,self.config)
        tilted=SE3.RPY((180.,-tilt,90.),unit='deg',order='zyx').R
        self.joints=self.planner.solve(self.initial_tip,self.joints,tilted)
        self.homing=HomingOptions()
        self.contact_options=ContactOptions(trigger_mode='raw')
        self.source=Source()
        self.detector=ContactDetector(0.,None,self.contact_options)
        self.control=ProbeControl(io.StringIO(),io.StringIO())
        self.records={};self.points=[(30.,30.)]
        self._z_require_stationary=Mock()
        self.seeking=False
        self._z_manual_active=False
        self.capture_start(allow_alignment=True)

    def z_waiting_anchor(self): return self.nozzle_now()

    def run_motion(self,motion):
        assert self._surface_confirmed and self.seeking and self._z_manual_active
        super().run_motion(motion)

    def confirm_surface(self):
        def confirm():
            if self.control.pending:
                self.control.ingest({'command':'confirm',**self.control.pending})
        with patch.object(self,'pump',side_effect=confirm):
            self.confirm_position('surface','Confirm surface and alignment')


class ProbeAlignmentTests(unittest.TestCase):
    def test_prepare_and_confirmation_only_never_move(self):
        node=AlignmentHarness()
        self.assertTrue(node.start_alignment['required'])
        self.assertAlmostEqual(node.start_alignment['angle_deg'],.255,places=3)
        self.assertEqual(node.motions,[])
        with self.assertRaisesRegex(RuntimeError,'fresh unloaded surface confirmation'):
            node.align_homing_start()
        self.assertFalse(node._alignment_attempted)
        node.confirm_surface()
        self.assertEqual(node.motions,[])
        self.assertTrue(node._surface_confirmed)

    def test_real_path_stays_at_tip_and_corrected_pose_is_return_anchor(self):
        node=AlignmentHarness();node.confirm_surface()
        pivot=node.start_xyz.copy();old_joints=node.start_joints.copy()
        node.align_homing_start()
        self.assertEqual(len(node.motions),1)
        motion=node.motions[0]
        self.assertGreaterEqual(motion.times[-1],3.)
        for index,dt in enumerate(np.diff(motion.times)):
            for fraction in (0.,.25,.5,.75,1.):
                joints,_,_=cubic_joint_state(*motion.positions[index:index+2],
                    *motion.velocities[index:index+2],dt,fraction)
                self.assertLessEqual(np.linalg.norm(node.planner.nozzle_position(joints)-pivot),.0025)
        self.assertEqual(node.start_alignment['state'],'aligned')
        self.assertLess(node.start_alignment['residual_deg'],.05)
        self.assertFalse(np.allclose(node.start_joints,old_joints))
        np.testing.assert_allclose(node.start_joints,node.joints)
        np.testing.assert_allclose(node.start_xyz,pivot,atol=.0025)
        # A subsequent Z move must preserve the corrected orientation.
        node.run_motion=lambda motion: OrientationHarness.run_motion(node,motion)
        node.move_z(node.target[2]+3.,'post-alignment Z',10.)
        self.assertLess(rotation_error_deg(node.planner.kinematics.eval(node.joints)[:3,:3],
                                          node._nominal_probe_rotation),.05)
        self.assertFalse(node.seeking or node._z_manual_active)

    def test_large_tilt_is_rejected_before_confirmation(self):
        with self.assertRaisesRegex(ValueError,'0.5 deg automatic alignment limit'):
            AlignmentHarness(.51)

    def test_calibrated_arm_model_also_reaches_upright(self):
        node=AlignmentHarness(calibrated=True);node.confirm_surface();node.align_homing_start()
        self.assertEqual(node.start_alignment['state'],'aligned')
        self.assertLess(node.start_alignment['measured_shift_mm'],.0025)
        self.assertLess(node.start_alignment['residual_deg'],.05)

    def test_near_upright_skips_rotation(self):
        node=AlignmentHarness(.01);node.confirm_surface();node.align_homing_start()
        self.assertEqual(node.start_alignment['state'],'not_needed')
        self.assertEqual(node.motions,[])

    def test_unloaded_baseline_required_before_planning(self):
        for mode in ('loaded','no_baseline','stale_sensor','stale_robot'):
            node=AlignmentHarness();node.confirm_surface()
            if mode=='loaded':node.source.value=6000.
            elif mode=='no_baseline':node.detector=None
            elif mode=='stale_sensor':node.source.clock=lambda:101.
            else:node._z_require_stationary.side_effect=RuntimeError('Robot joint feedback stale')
            with self.subTest(mode=mode),self.assertRaisesRegex(RuntimeError,'Z homing was not started'):
                node.align_homing_start()
            self.assertEqual(node.motions,[])
            self.assertEqual(node.start_alignment['state'],'failed')
            with self.assertRaisesRegex(RuntimeError,'fresh unloaded surface confirmation'):
                node.align_homing_start()

    def test_changed_pose_requires_restart(self):
        node=AlignmentHarness();node.confirm_surface()
        node.joints=node.planner.solve(node.start_xyz+[0.,0.,1.],node.joints,node.planner.rotation)
        with self.assertRaisesRegex(RuntimeError,'moved from the confirmed alignment position'):
            node.align_homing_start()
        self.assertEqual(node.motions,[])

    def test_planning_failure_clears_guards_without_motion_or_tolerance_changes(self):
        for error in (RuntimeError('Operator cancelled'),RuntimeError('Temperature changed'),
                      ContactDetected('Unexpected nozzle contact'),LoadCellOverload(121000.,120000.,'counts')):
            node=AlignmentHarness();node.confirm_surface()
            config=node.config
            def fail():
                self.assertTrue(node.seeking and node._z_manual_active)
                raise error
            with patch.object(node,'pump',side_effect=fail):
                with self.subTest(error=str(error)),self.assertRaisesRegex(RuntimeError,'Z homing was not started'):
                    node.align_homing_start()
            self.assertIs(node.planner.config,node.config)
            self.assertEqual(node.config.path_tolerance_mm,config.path_tolerance_mm)
            self.assertEqual(node.config.orientation_tolerance_deg,.2)
            self.assertFalse(node.seeking or node._z_manual_active)
            self.assertEqual(node.motions,[])

    def test_ur_safety_refusal_prevents_action_execution(self):
        node=AlignmentHarness();node.confirm_surface()
        node.robot_safety_checker=Mock()
        node.robot_safety_checker.validate.side_effect=RuntimeError('UR safety plane rejects pose')
        node.run_motion=lambda motion: BridgeBedProbe.run_motion(node,motion)
        with patch.object(LoadCellBedProbe,'run_motion') as execute:
            with self.assertRaisesRegex(RuntimeError,'UR safety plane rejects pose'):
                node.align_homing_start()
            execute.assert_not_called()
        node.robot_safety_checker.validate.assert_called_once()

    def test_incomplete_rotation_and_shift_and_load_prevent_homing(self):
        for fault in ('stopped_short','shift','load','contact'):
            node=AlignmentHarness();node.confirm_surface()
            execute=node.run_motion
            def faulty_motion(motion):
                if fault=='contact':raise ContactDetected('contact during alignment')
                if fault=='stopped_short':return
                execute(motion)
                if fault=='load':node.source.value=6000.
                if fault=='shift':
                    node.joints=node.planner.solve(node.start_xyz+[0.,0.,.2],node.joints,node._nominal_probe_rotation)
            node.run_motion=faulty_motion
            with self.subTest(fault=fault),self.assertRaisesRegex(RuntimeError,'Z homing was not started'):
                node.align_homing_start()
            self.assertEqual(node.start_alignment['state'],'failed')
            self.assertFalse(node.seeking or node._z_manual_active)

    def test_pending_alignment_error_never_opens_z_recovery(self):
        node=AlignmentHarness()
        node.manual_z_session=Mock()
        node._run_routine=Mock(side_effect=LoadCellOverload(121000.,120000.,'counts'))
        with self.assertRaisesRegex(RuntimeError,'Z controls remain closed'):
            node.run_routine()
        node.manual_z_session.assert_not_called()

    def test_native_probe_mode_still_guards_rotation_with_raw_contact(self):
        node=AlignmentHarness();node.confirm_surface()
        native=Mock();node.source.native_trigger=native
        def contact_during_motion(motion):
            node.source.value=21000.
            # Bypass only the ROS base monitor, exercising the real ALPS guard.
            with patch.object(BedProbe,'check_interrupted'):
                LoadCellBedProbe.check_interrupted(node)
        node.run_motion=contact_during_motion
        with self.assertRaisesRegex(RuntimeError,'Nozzle contact detected'):
            node.align_homing_start()
        native.check.assert_not_called()
        native.arm.assert_not_called()


if __name__=='__main__':unittest.main()
