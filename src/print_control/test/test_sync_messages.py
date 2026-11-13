"""ROS message preparation checks without nodes, controllers or robot commands."""
from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from print_control.sync_runtime import ROSRobot


class MessageTests(unittest.TestCase):
    def setUp(self):
        self.robot=object.__new__(ROSRobot)
        self.motion=SimpleNamespace(positions=np.array([[0.]*6,[.1]*6,[.2]*6]),
            velocities=np.array([[0.]*6,[.03]*6,[0.]*6]),times=np.array([0.,.123456789,2.]))

    def test_preparation_preserves_every_position_velocity_and_time(self):
        self.robot.prepare_motions([self.motion])
        goal=self.robot.prepared_motions[id(self.motion)][1]
        for point,q,v,t in zip(goal.trajectory.points,self.motion.positions,self.motion.velocities,self.motion.times):
            np.testing.assert_array_equal(point.positions,q)
            np.testing.assert_array_equal(point.velocities,v)
            self.assertAlmostEqual(point.time_from_start.sec+point.time_from_start.nanosec/1e9,t,places=9)
        self.assertEqual(len(goal.trajectory.points),3)
        self.assertTrue(all(abs(t.position-np.deg2rad(2))<1e-12 for t in goal.path_tolerance))
        self.assertTrue(all(abs(t.position-np.deg2rad(.2))<1e-12 for t in goal.goal_tolerance))

    def test_send_reuses_prepared_message_but_assigns_a_fresh_stamp(self):
        self.robot.prepare_motions([self.motion])
        self.robot.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000)))
        received=[]
        result=Future();result.set_result(None)
        handle=SimpleNamespace(accepted=True,get_result_async=lambda:result)
        accepted=Future();accepted.set_result(handle)
        def send(goal):
            received.append((goal,goal.trajectory.header.stamp.sec,goal.trajectory.header.stamp.nanosec))
            return accepted
        self.robot.action=SimpleNamespace(send_goal_async=send)
        with patch('print_control.sync_runtime.time.monotonic',return_value=50.),patch.object(self.robot,'_build_goal',side_effect=AssertionError('Rebuilt a prepared message')):
            self.robot.send(self.motion,52.)
            self.robot.send(self.motion,53.)
        self.assertIs(received[0][0],received[1][0])
        self.assertEqual(received[0][1:],(102,0))
        self.assertEqual(received[1][1:],(103,0))

    def test_cancelled_preparation_does_not_publish_a_partial_cache(self):
        self.robot.prepared_motions={}
        with self.assertRaisesRegex(RuntimeError,'Planning cancelled'):
            self.robot.prepare_motions([self.motion],cancelled=lambda:True)
        self.assertEqual(self.robot.prepared_motions,{})


if __name__=='__main__':unittest.main()
