"""Exercise worker ownership, clock deadlines and fail-stop behavior without ROS."""
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from print_control.sync_runtime import RobotWorker


class Backend:
    def __init__(self):
        self.ready=True; self.cancelled=[]; self.fail_cancel=False; self.stops=0
    def feedback(self): return {'ready':self.ready,'joints_fresh':self.ready,'reason':'Stale feedback'}
    def joints(self): return np.zeros(6)
    def cancel(self,goal):
        self.cancelled.append(goal)
        if self.fail_cancel: raise RuntimeError('Controller disconnected')
    def stop_program(self): self.stops+=1


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.backend=Backend()
        self.worker=RobotWorker(self.backend,'unused',enabled=True,clock=lambda:self.now)
        self.addCleanup(self.worker.close)
        self.patch=patch('print_control.sync_runtime.configuration',return_value=(None,None,'config'))
        self.patch.start(); self.addCleanup(self.patch.stop)
        self.worker.session='s'*24
        self.worker.compiled=SimpleNamespace(plan={'plan_hash':'plan'},config_hash='config',
            chunks=[SimpleNamespace(positions=np.zeros((2,6)))])
        self.worker.state='ready'
        self.data={'session':self.worker.session}

    def test_lease_ownership_and_disabled_motion(self):
        with self.assertRaisesRegex(ValueError,'Unknown'): self.worker.acquire({'session':'wrong'})
        self.worker.acquire(self.data)
        with self.assertRaisesRegex(ValueError,'already'): self.worker.acquire(self.data)
        self.worker.enabled=False
        with self.assertRaisesRegex(ValueError,'disabled'): self.worker.approach(self.data)
        self.assertFalse(self.backend.cancelled)

    def test_watchdog_cancels_an_active_goal(self):
        self.worker.acquire(self.data); self.worker.goal='goal'; self.worker.state='running'
        self.now+=2
        deadline=time.monotonic()+1
        while self.worker.state!='fault' and time.monotonic()<deadline: time.sleep(.01)
        self.assertEqual(self.worker.state,'fault')
        self.assertIn('heartbeat',self.worker.error)
        self.assertEqual(self.backend.cancelled,['goal'])

    def test_heartbeat_does_not_wait_for_feedback_or_configuration_io(self):
        self.worker.acquire(self.data)
        self.now += .5
        self.backend.feedback=lambda:(_ for _ in ()).throw(RuntimeError('ROS query busy'))
        with patch('print_control.sync_runtime.configuration',side_effect=OSError('Config I/O busy')):
            result=self.worker.heartbeat(self.data)
        self.assertEqual(result,{'state':'ready','session':self.worker.session,'leased':True})
        self.assertEqual(self.worker.deadline,self.now+1.)

    def test_heartbeat_cannot_revive_an_expired_lease(self):
        self.worker.acquire(self.data)
        self.now += 1.01
        with self.assertRaisesRegex(ValueError,'lease|heartbeat'):
            self.worker.heartbeat(self.data)

    def test_cancel_failure_uses_dashboard_stop(self):
        self.worker.goal='goal'; self.backend.fail_cancel=True
        result=self.worker.abort(dict(self.data,reason='operator'))
        self.assertEqual(self.backend.stops,1)
        self.assertIn('Controller disconnected',result['error'])

    def test_late_or_uncertain_schedule_is_rejected(self):
        self.worker.acquire(self.data); self.worker.state='at_boundary'
        data=dict(self.data,index=0,start_monotonic=self.now+.1,clock_uncertainty_s=.001)
        with self.assertRaisesRegex(ValueError,'deadline'): self.worker.schedule(data)
        data.update(start_monotonic=self.now+1,clock_uncertainty_s=.02)
        with self.assertRaisesRegex(ValueError,'uncertainty'): self.worker.schedule(data)
        data.update(index=1,clock_uncertainty_s=.001)
        with self.assertRaisesRegex(ValueError,'index'): self.worker.schedule(data)
        self.assertFalse(self.backend.cancelled)

    def test_stale_feedback_and_changed_calibration_block_motion(self):
        self.worker.acquire(self.data); self.backend.ready=False
        with self.assertRaisesRegex(ValueError,'Stale'): self.worker.approach(self.data)
        self.backend.ready=True; self.worker.compiled.config_hash='old'
        with self.assertRaisesRegex(ValueError,'calibration'): self.worker.approach(self.data)

    def test_late_acceptance_is_cancelled_before_commit(self):
        self.worker.acquire(self.data)
        self.backend.send=lambda motion,start:'late'
        motion=SimpleNamespace(times=np.array([0.,1.]))
        with self.assertRaisesRegex(RuntimeError,'deadline'):
            self.worker._execute(motion,self.now+.1,self.worker.generation,0)
        self.assertEqual(self.backend.cancelled,['late'])

    def test_completion_requires_every_chunk(self):
        self.worker.acquire(self.data); self.worker.state='at_boundary'
        with self.assertRaisesRegex(ValueError,'unfinished'): self.worker.complete(self.data)
        self.worker.next_chunk=1
        self.assertEqual(self.worker.complete(self.data)['state'],'complete')
        self.assertFalse(self.worker.lease)


if __name__=='__main__': unittest.main()
