"""Probe callback registration across polling, cancellation and cleanup."""
import unittest
from unittest.mock import Mock, patch

import rclpy

from print_control.alps_load_calibration import LoadCalibration
from print_control.bridge_bed_probe import BridgeBedProbe
from print_control.load_cell_bed_probe import LoadCellBedProbe
from print_control.motion_planner import MotionConfig


class Executor:
    def __init__(self):
        self.nodes = set()
        self.removed = []
        self.spins = 0

    def add_node(self, node):
        added = node not in self.nodes
        self.nodes.add(node)
        return added

    def remove_node(self, node):
        self.removed.append(node)
        self.nodes.discard(node)

    def spin_once(self, timeout_sec=None):
        self.spins += 1


class ProbeFeedbackExecutorTests(unittest.TestCase):
    def node(self, cls=BridgeBedProbe, *, dry_run=False):
        node = object.__new__(cls)
        node.config = MotionConfig(dry_run=dry_run)
        node._feedback_executor = None
        return node

    def test_homing_and_calibration_stay_registered_across_stock_spin_calls(self):
        for cls in (BridgeBedProbe, LoadCalibration):
            with self.subTest(routine=cls.__name__):
                node, executor = self.node(cls), Executor()
                with patch('rclpy.get_global_executor', return_value=executor), \
                     patch.object(LoadCellBedProbe, 'destroy_node', return_value=True) as cleanup:
                    node.attach_feedback_executor()
                    node.attach_feedback_executor()
                    for timeout in (.01, .02, .01):
                        # These are the real rclpy calls used by pump(),
                        # wait_until() and cancellation, not a replacement loop.
                        rclpy.spin_once(node, timeout_sec=timeout)
                        self.assertIn(node, executor.nodes)
                    self.assertEqual(executor.spins, 3)
                    self.assertEqual(executor.removed, [])
                    self.assertTrue(node.destroy_node())
                    self.assertEqual(executor.removed, [node])
                    self.assertIsNone(node._feedback_executor)
                    cleanup.assert_called_once()

    def test_dry_run_creates_no_feedback_executor(self):
        node = self.node(dry_run=True)
        with patch('rclpy.get_global_executor') as global_executor:
            node.attach_feedback_executor()
            global_executor.assert_not_called()

    def test_alternating_probe_waits_do_not_reset_callback_iterator(self):
        class ReadyExecutor(Executor):
            def __init__(self):
                super().__init__()
                self.timeout = None
                self.index = 0
                self.delivered = []

            def spin_once(self, timeout_sec=None):
                # Humble's wait_for_ready_callbacks recreates its generator if
                # kwargs change. The first subscription is continuously ready.
                if self.timeout != timeout_sec:
                    self.index = 0
                    self.timeout = timeout_sec
                self.delivered.append(('joints', 'speed', 'action')[self.index % 3])
                self.index += 1

        node, executor = self.node(), ReadyExecutor()
        with patch('rclpy.get_global_executor', return_value=executor):
            node.attach_feedback_executor()
            for timeout in (.01, .02)*15:
                node.spin_feedback(timeout)
        self.assertEqual(executor.timeout, .002)
        counts=[executor.delivered.count(signal) for signal in ('joints', 'speed', 'action')]
        self.assertGreaterEqual(min(counts), 10)
        self.assertLessEqual(max(counts)-min(counts), 1)
        self.assertLessEqual(len(executor.delivered), 30*8)

    def test_cleanup_does_not_remove_registration_owned_by_another_caller(self):
        node, executor = self.node(), Executor()
        executor.add_node(node)
        with patch('rclpy.get_global_executor', return_value=executor), \
             patch.object(LoadCellBedProbe, 'destroy_node', return_value=True):
            node.attach_feedback_executor()
            self.assertIsNone(node._feedback_executor)
            node.destroy_node()
            self.assertEqual(executor.removed, [])

    def test_node_and_robot_lock_cleanup_still_runs_if_detach_fails(self):
        node = self.node()
        node._feedback_executor = Mock()
        node._feedback_executor.remove_node.side_effect = RuntimeError('executor cleanup failed')
        with patch.object(LoadCellBedProbe, 'destroy_node', return_value=True) as cleanup:
            with self.assertRaisesRegex(RuntimeError, 'executor cleanup failed'):
                node.destroy_node()
            cleanup.assert_called_once()
            self.assertIsNone(node._feedback_executor)


if __name__ == '__main__':
    unittest.main()
