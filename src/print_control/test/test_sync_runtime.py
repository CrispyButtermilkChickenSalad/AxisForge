"""Exercise worker ownership, clock deadlines and fail-stop behavior without ROS."""
import json
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from print_control.sync_runtime import RobotWorker, ROSRobot, serve


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

    def test_blocked_motion_validation_does_not_block_heartbeat_or_abort(self):
        self.worker.acquire(self.data)
        entered, release, renewed = threading.Event(), threading.Event(), threading.Event()
        errors=[]
        def blocked_feedback():
            entered.set(); release.wait(2.)
            return {'ready':True,'joints_fresh':True,'reason':''}
        self.backend.feedback=blocked_feedback
        def approach():
            try:self.worker.approach(self.data)
            except Exception as error:errors.append(str(error))
        task=threading.Thread(target=approach);task.start()
        self.assertTrue(entered.wait(1.))
        heartbeat=threading.Thread(target=lambda:(self.worker.heartbeat(self.data),renewed.set()))
        heartbeat.start()
        try:
            self.assertTrue(renewed.wait(.2),'Motion validation held the heartbeat lock')
            self.worker.abort(dict(self.data,reason='cancel during validation'))
        finally:
            release.set();task.join(2.);heartbeat.join(2.)
        self.assertEqual(self.worker.state,'fault')
        self.assertTrue(any('cancel during validation' in error for error in errors))

    def test_blocked_configuration_io_does_not_starve_the_watchdog(self):
        self.worker.acquire(self.data)
        entered,release=threading.Event(),threading.Event()
        errors=[]
        def blocked(*args):
            entered.set();release.wait(2.);return None,None,'config'
        def approach():
            try:self.worker.approach(self.data)
            except Exception as error:errors.append(str(error))
        with patch('print_control.sync_runtime.configuration',side_effect=blocked):
            task=threading.Thread(target=approach);task.start()
            self.assertTrue(entered.wait(1.))
            self.now+=2.
            try:
                deadline=time.monotonic()+.3
                while self.worker.state!='fault' and time.monotonic()<deadline:time.sleep(.01)
                self.assertEqual(self.worker.state,'fault','Watchdog was blocked by configuration I/O')
            finally:release.set();task.join(2.)
        self.assertIn('heartbeat',self.worker.error)
        self.assertTrue(errors)

    def test_stalled_motion_checks_abort_even_with_fresh_heartbeats(self):
        self.worker.acquire(self.data)
        self.worker.goal='goal';self.worker.state='running';self.worker.validation_deadline=self.now+.25
        self.now+=.3
        self.worker.heartbeat(self.data)
        deadline=time.monotonic()+.3
        while self.worker.state!='fault' and time.monotonic()<deadline:time.sleep(.01)
        self.assertEqual(self.worker.state,'fault')
        self.assertIn('supervision stalled',self.worker.error)
        self.assertEqual(self.backend.cancelled,['goal'])

    def test_first_fault_is_not_replaced_by_a_late_controller_error(self):
        self.worker.abort(dict(self.data,reason='Bridge heartbeat lost'))
        result=self.worker.abort(dict(self.data,reason='Cancelled while accepting goal'))
        self.assertEqual(result['error'],'Bridge heartbeat lost')

    def test_old_watchdog_or_execution_callback_cannot_abort_a_new_generation(self):
        self.worker.acquire(self.data)
        old_generation=self.worker.generation
        self.worker.generation+=1
        self.worker.abort(dict(self.data,reason='Stale callback'),_generation=old_generation)
        self.assertTrue(self.worker.lease)
        self.assertEqual(self.worker.state,'ready')

    def test_preflight_requires_message_preparation_to_finish(self):
        self.backend.prepare_motions=lambda *args:(_ for _ in ()).throw(ValueError('Message preparation failed'))
        # prepare clears the old plan before calling the compiler.
        compiled=self.worker.compiled;self.worker.compiler=lambda *args:compiled
        self.worker.prepare(dict(self.data,plan=compiled.plan));self.worker.thread.join(1.)
        self.assertEqual(self.worker.state,'fault')
        self.assertIn('Message preparation failed',self.worker.error)

    def test_http_heartbeat_reuses_its_connection(self):
        import http.client
        self.worker.acquire(self.data)
        server=serve(self.worker,0)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        connection=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=1.)
        try:
            socket=None
            for _ in range(3):
                connection.request('POST','/heartbeat',json.dumps(self.data),{'Content-Type':'application/json'})
                response=connection.getresponse();payload=json.loads(response.read())
                self.assertEqual(response.version,11)
                self.assertTrue(payload['leased'])
                if socket is not None:self.assertIs(connection.sock,socket)
                socket=connection.sock
        finally:connection.close();server.shutdown();server.server_close()

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

    def test_clock_policy_is_saved_during_prepare_and_resets_for_legacy_requests(self):
        compiled=self.worker.compiled
        self.worker.compiler=lambda *args:compiled
        timing={'max_uncertainty_ms':15.,'max_alignment_error_ms':20.}
        self.worker.prepare(dict(self.data,plan=compiled.plan,clock_options=timing))
        self.worker.thread.join(1.)
        timing['max_uncertainty_ms']=20.
        self.assertEqual(self.worker.status()['clock_options']['max_uncertainty_ms'],15.)
        self.worker.release(self.data)
        self.worker.prepare(dict(self.data,plan=compiled.plan))
        self.worker.thread.join(1.)
        self.assertEqual(self.worker.status()['clock_options'],
                         {'max_uncertainty_ms':10.,'max_alignment_error_ms':10.})

    def test_selected_uncertainty_boundary_and_schedule_cannot_raise_it(self):
        timing={'max_uncertainty_ms':15.,'max_alignment_error_ms':20.}
        self.worker.clock_options=dict(timing)
        with self.assertRaisesRegex(ValueError,'differ from robot preflight'):
            self.worker.acquire(self.data)
        self.worker.acquire(dict(self.data,clock_options=timing))
        self.worker.state='at_boundary'
        data=dict(self.data,index=0,start_monotonic=self.now+1,clock_options=timing,
                  clock_uncertainty_s=.01501)
        with self.assertRaisesRegex(ValueError,'exceeds 15 ms'):self.worker.schedule(data)
        with self.assertRaisesRegex(ValueError,'differ from robot preflight'):
            self.worker.schedule(dict(data,clock_options={'max_uncertainty_ms':20.,'max_alignment_error_ms':20.}))
        for value in (True,float('nan'),float('inf'),-.001):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'uncertainty'):
                self.worker.schedule(dict(data,clock_uncertainty_s=value))
        def accept(motion,start,generation,index,accepted):accepted.set_result(True)
        with patch.object(self.worker,'_execute',side_effect=accept):
            result=self.worker.schedule(dict(data,clock_uncertainty_s=.015))
            self.worker.thread.join(1.)
        self.assertTrue(result['accepted'])
        self.assertEqual(self.worker.lease_s,1.)

    def test_invalid_clock_options_rejected_before_preparation(self):
        from print_control.sync_clock import clock_options
        for timing in (True,[],{'unknown':10},{'max_alignment_error_ms':5},
                       *({'max_uncertainty_ms':v} for v in (True,'5',float('nan'),float('inf'),1.9,20.1))):
            with self.subTest(timing=timing),self.assertRaises(ValueError):
                self.worker.prepare(dict(self.data,plan={},clock_options=timing))
            self.assertFalse(self.worker.lease)
        self.assertEqual(clock_options({'max_uncertainty_ms':2})['max_uncertainty_ms'],2.)

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

    def test_rejected_goal_cancel_failure_stops_the_program(self):
        self.worker.acquire(self.data);self.backend.fail_cancel=True
        self.backend.send=lambda motion,start:'late'
        with self.assertRaisesRegex(RuntimeError,'Controller disconnected'):
            self.worker._execute(SimpleNamespace(times=np.array([0.,1.])),self.now+.1,self.worker.generation,0)
        self.assertEqual(self.backend.stops,1)

    def test_stop_fallback_preserves_both_failures(self):
        self.backend.fail_cancel=True
        self.backend.stop_program=lambda:(_ for _ in ()).throw(RuntimeError('Dashboard unavailable'))
        self.worker.goal='goal'
        result=self.worker.abort(dict(self.data,reason='Operator abort'))
        self.assertIn('Controller disconnected',result['error'])
        self.assertIn('Dashboard unavailable',result['error'])

    def test_completion_requires_every_chunk(self):
        self.worker.acquire(self.data); self.worker.state='at_boundary'
        with self.assertRaisesRegex(ValueError,'unfinished'): self.worker.complete(self.data)
        self.worker.next_chunk=1
        self.assertEqual(self.worker.complete(self.data)['state'],'complete')
        self.assertFalse(self.worker.lease)


class RobotFeedbackTests(unittest.TestCase):
    """Test the real readiness gate with the UR broadcaster's percentage units."""

    def setUp(self):
        self.now = 100.
        self.robot = object.__new__(ROSRobot)
        self.robot.lock = threading.RLock()
        self.robot.values = np.zeros(6)
        self.robot.joint_time = self.now
        self.robot.signals = {'speed':(100.,self.now), 'program':(True,self.now),
                              'safety':(1,self.now)}
        self.robot.action = SimpleNamespace(server_is_ready=lambda:True)
        self.robot.active_clock_offset = None
        clock = patch('print_control.sync_runtime.time.monotonic', side_effect=lambda:self.now)
        clock.start()
        self.addCleanup(clock.stop)

    def test_one_hundred_percent_enables_motion(self):
        result = self.robot.feedback()
        self.assertTrue(result['ready'], result['reason'])
        self.assertEqual(result['speed_scaling'], 1.)
        self.assertEqual(result['speed_scaling_percent'], 100.)

    def test_reduced_or_out_of_range_percent_blocks_motion(self):
        for percent in (0., .5, 1., 50., 99., -1., 101.):
            with self.subTest(percent=percent):
                self.robot.signals['speed'] = (percent,self.now)
                result = self.robot.feedback()
                self.assertFalse(result['ready'])
                self.assertIn('speed scaling', result['reason'])
                self.assertEqual(result['speed_scaling'], percent/100.)

    def test_small_measurement_tolerance_is_preserved(self):
        for percent in (99.95, 100.05):
            self.robot.signals['speed'] = (percent,self.now)
            self.assertTrue(self.robot.feedback()['ready'])

    def test_invalid_feedback_blocks_motion_and_remains_json_safe(self):
        for percent in (None, float('nan'), float('inf'), float('-inf')):
            with self.subTest(percent=percent):
                self.robot.signals['speed'] = (percent,self.now)
                result = self.robot.feedback()
                self.assertFalse(result['ready'])
                self.assertIn('invalid', result['reason'])
                self.assertIsNone(result['speed_scaling'])
                self.assertIsNone(result['speed_scaling_percent'])
                json.dumps(result, allow_nan=False)

    def test_stale_or_missing_speed_still_blocks_motion(self):
        self.robot.signals['speed'] = (100.,self.now-.3)
        self.assertIn('stale', self.robot.feedback()['reason'])
        del self.robot.signals['speed']
        self.assertFalse(self.robot.feedback()['ready'])

    def test_other_interlocks_still_apply_at_full_speed(self):
        self.robot.signals['program'] = (False,self.now)
        self.assertIn('External Control', self.robot.feedback()['reason'])
        self.robot.signals['program'] = (True,self.now)
        self.robot.signals['safety'] = (3,self.now)
        self.assertIn('NORMAL', self.robot.feedback()['reason'])
        self.robot.signals['safety'] = (1,self.now)
        self.robot.joint_time = self.now-.3
        self.assertFalse(self.robot.feedback()['ready'])


if __name__=='__main__': unittest.main()
