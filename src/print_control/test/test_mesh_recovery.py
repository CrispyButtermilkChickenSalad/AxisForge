"""Exercise automatic mesh recovery using real load/release guards, no hardware."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import numpy as np

from test_probe_z_moves import Harness
from print_control.bridge_bed_probe import BridgeBedProbe
from print_control.mesh_recovery import MeshFailureRecovery
from print_control.load_cell_contact import LoadCellOverload


class RecoveryHarness(MeshFailureRecovery, Harness):
    return_to_start = BridgeBedProbe.return_to_start

    def __init__(self):
        super().__init__()
        self.start_xyz=np.array([1.,2.,110.])
        self.joints[:3]=[4.,5.,100.]
        self.target,self.seed=self.nozzle_now()
        self.homing.clearance_above_start_mm=10.
        self.homed_z=99.;self.records={};self._point_readings=[]
        self._home_established=self._mesh_phase_active=True
        self.returned=False;self.save=Mock();self.events=[]
        self.xy_travel_speed=20.
        self._run_routine=Mock(side_effect=RuntimeError('Contact repeatability failed'))
        self.manual_z_session=Mock()
        self.detector.baseline=1234.
        self.source.value=1234.

    def emit_stage(self,state,message,**values): self.events.append((state,message,values))

    def line_to(self,end,name,speed):
        self.check_interrupted()
        self.run_motion(SimpleNamespace(end=np.array(end),name=name,speed=speed))
        self.target,self.seed=self.nozzle_now()
        self.check_interrupted()

    def move_z(self,height,name,speed):self.line_to(np.r_[self.target[:2],height],name,speed)


class MeshRecoveryTests(unittest.TestCase):
    def test_unloaded_first_point_failure_returns_home_without_saving_or_retrying(self):
        node=RecoveryHarness()
        with self.assertRaisesRegex(RuntimeError,'Contact repeatability failed; returned to established Home'):
            node.run_routine()
        self.assertEqual([m.name for m in node.moves],['return clearance lift','return XY','return starting Z'])
        self.assertEqual(node.moves[0].end[0],4.)
        self.assertEqual(node.moves[0].end[1],5.)
        np.testing.assert_allclose(node.target,node.start_xyz)
        self.assertEqual(node.failure_recovery['state'],'returned')
        self.assertFalse(node.returned)
        node.save.assert_not_called();node.manual_z_session.assert_not_called()
        node._run_routine.assert_called_once()
        self.assertEqual(node.detector.baseline,1234.)

    def test_contact_is_released_upward_before_any_xy_or_downward_move(self):
        for load in (20000.,125000.):
            node=RecoveryHarness();node.source.value=node.detector.baseline+load
            node._run_routine.side_effect=LoadCellOverload(load,120000.,'counts')
            def release():
                if node.joints[2]>=100.19:node.source.value=node.detector.baseline
            node.on_move=release
            with self.subTest(load=load),self.assertRaisesRegex(RuntimeError,'returned to established Home'):
                node.run_routine()
            self.assertEqual(len(node.moves),5)
            for m in node.moves[:2]:
                np.testing.assert_allclose(m.end[:2],[4.,5.]);self.assertEqual(m.speed,.1)
            self.assertGreater(node.moves[1].end[2],node.moves[0].end[2])
            self.assertFalse(node._z_recovery_active)
            self.assertEqual(node.detector.options.maximum,120000.)
            self.assertEqual(node.detector.baseline,1234.)

    def test_stop_heartbeat_sensor_temperature_and_unconfirmed_stop_block_all_moves(self):
        for fault in ('Operator Stop','Bridge heartbeat lost','Sensor stale','Temperature changed','Stop unconfirmed','Robot feedback stale'):
            node=RecoveryHarness()
            if fault in ('Operator Stop','Bridge heartbeat lost'):node.control.check.side_effect=RuntimeError(fault)
            elif fault=='Sensor stale':node.source.snapshot=Mock(side_effect=RuntimeError(fault))
            elif fault=='Temperature changed':node.check_interrupted=Mock(side_effect=RuntimeError(fault))
            elif fault=='Stop unconfirmed':node.confirm_motion_stopped.side_effect=RuntimeError(fault)
            else:node._z_require_stationary=Mock(side_effect=RuntimeError(fault))
            with self.subTest(fault=fault),self.assertRaisesRegex(RuntimeError,'automatic Home return blocked'):
                node.run_routine()
            self.assertEqual(node.moves,[])
            self.assertEqual(node.failure_recovery['state'],'blocked')
            node.save.assert_not_called()

    def test_load_increase_stops_recovery_without_automatic_retry(self):
        node=RecoveryHarness();node.source.value=22000.
        node.on_move=lambda:setattr(node.source,'value',32000.)
        with self.assertRaisesRegex(RuntimeError,'Load increased'):
            node.run_routine()
        self.assertEqual(len(node.moves),1)
        self.assertFalse(node.seeking or node._z_manual_active)
        with self.assertRaisesRegex(RuntimeError,'already attempted'):
            node.recover_mesh_failure(RuntimeError('same fault'))
        self.assertEqual(len(node.moves),1)

    def test_unreleased_load_never_exceeds_five_mm_and_never_travels_xy(self):
        node=RecoveryHarness();node.source.value=22000.
        with self.assertRaisesRegex(RuntimeError,'automatic Home return blocked'):
            node.run_routine()
        self.assertLessEqual(node.target[2],105.+1e-8)
        self.assertLessEqual(len(node.moves),50)
        for motion in node.moves:np.testing.assert_allclose(motion.end[:2],[4.,5.])

    def test_home_clearance_and_return_path_rejection_remain_failures(self):
        node=RecoveryHarness();node._point_readings=[{'nozzle':np.array([4.,5.,109.8])}]
        with self.assertRaisesRegex(RuntimeError,'less than 0.5 mm'):
            node.run_routine()
        self.assertEqual(node.moves,[])
        node=RecoveryHarness();node.return_to_start=Mock(side_effect=ValueError('UR safety plane rejects path'))
        with self.assertRaisesRegex(RuntimeError,'UR safety plane rejects path'):
            node.run_routine()
        self.assertEqual(node.failure_recovery['state'],'blocked')
        node.return_to_start.assert_called_once()

    def test_unestablished_home_keeps_existing_contact_recovery_behavior(self):
        node=RecoveryHarness();node._home_established=node._mesh_phase_active=False
        node._run_routine.side_effect=LoadCellOverload(121000.,120000.,'counts')
        with self.assertRaises(LoadCellOverload):node.run_routine()
        node.manual_z_session.assert_called_once()
        self.assertEqual(node.moves,[])


if __name__=='__main__':unittest.main()
