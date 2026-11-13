"""Standalone probing and the synchronization worker must exclude one another."""
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from print_control.robot_ownership import RobotControlLock
from print_control.sync_runtime import RobotWorker, ROSRobot


class OwnershipTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        path = Path(self.temp.name)/'robot.lock'
        self.probe, self.sync = RobotControlLock(path), RobotControlLock(path)
        self.addCleanup(self.probe.release); self.addCleanup(self.sync.release)
        backend = SimpleNamespace(feedback=lambda: {'ready': True, 'joints_fresh': True},
                                  joints=lambda: np.zeros(6), cancel=lambda goal: None)
        self.worker = RobotWorker(backend, 'unused', enabled=True, control_lock=self.sync)
        self.addCleanup(self.worker.close)
        patched = patch('print_control.sync_runtime.configuration', return_value=(None, None, 'hash'))
        patched.start(); self.addCleanup(patched.stop)
        self.data = {'session': 'session-000000000001'}
        self.worker.session = self.data['session']; self.worker.state = 'ready'

    def test_probe_ownership_blocks_worker_and_reports_busy(self):
        self.probe.acquire()
        with self.assertRaisesRegex(ValueError, 'owned'): self.worker.acquire(self.data)
        with self.assertRaisesRegex(ValueError, 'owned'): self.worker.prepare(dict(self.data, plan={}))
        self.assertFalse(self.worker.lease)
        self.assertTrue(self.worker.status()['external_control_busy'])
        self.assertFalse(self.worker.status()['control_ready'])
        self.probe.release(); self.worker.acquire(self.data)
        with self.assertRaisesRegex(ValueError, 'owned'): self.probe.acquire()
        self.worker.release(self.data); self.probe.acquire()

    def test_abort_keeps_lock_until_pending_execution_thread_has_finished(self):
        self.worker.acquire(self.data)
        gate = threading.Event()
        self.worker.thread = threading.Thread(target=lambda: gate.wait(2.))
        self.worker.thread.start()
        try:
            self.worker.abort(self.data)
            with self.assertRaisesRegex(ValueError, 'owned'): self.probe.acquire()
        finally:
            gate.set(); self.worker.thread.join(1.)
        deadline = time.monotonic()+.3
        while self.probe.external_busy() and time.monotonic() < deadline: time.sleep(.01)
        self.probe.acquire()

    def test_uncertain_stop_keeps_ownership(self):
        self.worker.acquire(self.data); self.worker.goal = 'moving'
        self.worker.backend.cancel = lambda goal: (_ for _ in ()).throw(RuntimeError('Disconnected'))
        self.worker.abort(self.data)
        time.sleep(.06)
        with self.assertRaisesRegex(ValueError, 'owned'): self.probe.acquire()
        with self.assertRaisesRegex(ValueError, 'busy'): self.worker.prepare(dict(self.data, plan={}))

    def test_ros_cancellation_waits_for_terminal_action_result(self):
        robot = object.__new__(ROSRobot)
        ack = SimpleNamespace(goals_canceling=['goal'])
        terminal = SimpleNamespace(status=5)
        result_future = SimpleNamespace(done=lambda: False)
        goal = (SimpleNamespace(cancel_goal_async=lambda: 'cancel future'), result_future)
        with patch.object(robot, '_wait', side_effect=[ack, terminal]) as wait:
            robot.cancel(goal)
            self.assertEqual(wait.call_args_list[1].args, (result_future, 2.))
        with patch.object(robot, '_wait', side_effect=[ack, RuntimeError('timeout')]):
            with self.assertRaisesRegex(RuntimeError, 'timeout'): robot.cancel(goal)


if __name__ == '__main__': unittest.main()
