"""End-to-end orchestration using the real Klipper master and simulated robot."""

import asyncio
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

from bridge_dashboard.workflow import Workflow
from bridge_dashboard.sync_clients import ClockMap, ParticipantError, sample_clock
from test_master_session import make_master, call, Heater
from test_schedule import ENTRY


class MasterClient:
    def __init__(self,master,events): self.master,self.events=master,events
    async def call(self,action,**data):
        self.events.append(('master',action,data))
        return call(self.master,action,**data)
    async def close(self): pass


class RobotClient:
    def __init__(self,events,heater):
        self.events,self.heater=events,heater
        self.state='idle'; self.next=0; self.plan=None; self.session=None
        self.fail_plan=False; self.fail_run=False
    async def call(self,action,**data):
        self.events.append(('robot',action,data))
        if action=='prepare':
            assert self.heater.target == 0, 'Robot planning must never run with heaters on'
            self.plan,self.session=data['plan'],data['session']
            self.state='fault' if self.fail_plan else 'ready'
        elif action=='approach': self.state='at_boundary'
        elif action=='schedule':
            self.state='running'; self.next=data['index']
            async def finish():
                chunk=next(o for o in self.plan['operations'] if o['kind']=='chunk' and o['index']==self.next)
                await asyncio.sleep(max(0,data['start_monotonic']+chunk['duration_s']-time.monotonic()))
                self.state='fault' if self.fail_run else 'at_boundary'; self.next+=1
            self.motion=asyncio.create_task(finish())
            return {'accepted':True}
        elif action=='complete': self.state='complete'
        elif action=='abort':
            if self.session and data['session'] != self.session:
                raise ParticipantError('Robot sync abort failed: Unknown robot session')
            self.state='fault'
            if hasattr(self,'motion'): self.motion.cancel()
        return {'protocol':'axisforge.sync.v1','enabled':True,'control_ready':True,
                'state':self.state,'session':self.session,'error':'injected robot failure',
                'progress':1.,'next_chunk':self.next,'config_hash':'robot-config',
                'plan_hash':self.plan['plan_hash'] if self.plan else None,
                'feedback':{'ready':True,'reason':''}}
    async def close(self): pass


class WorkflowTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'part.gcode'
        self.path.write_text('M109 S200\n'+ENTRY+'G1 X110.1 E.01\nM104 S0\n')
        self.master,self.printer=make_master(self.path.parent)
        self.master.lead_s=.5
        self.events=[]
        self.robot=RobotClient(self.events,self.printer.heater)
        axisforge=SimpleNamespace(job_path=lambda name:self.path,
                                 status=lambda:{'running':False,'planning_check_running':False})
        self.stops=0
        async def stop(): self.stops+=1
        self.flow=Workflow(MasterClient(self.master,self.events),self.robot,axisforge,
            heat_timeout=.1,poll_s=.005,heat_stable_s=0.,robot_stop=stop)
    async def asyncTearDown(self): await self.flow.close()

    async def preflight(self, heater_checks=None):
        await self.flow.preflight('part.gcode', heater_checks)
        await asyncio.wait_for(self.flow.task,3.)

    async def test_upload_preflight_heat_off_then_synchronized_run(self):
        await self.preflight()
        self.assertEqual(self.flow.state,'ready',self.flow.message)
        self.assertEqual(self.printer.heater.target,0.)
        self.assertFalse(self.printer.queued)
        actions=[(who,action) for who,action,data in self.events]
        self.assertLess(actions.index(('master','heaters_off')),actions.index(('robot','prepare')))
        await self.flow.start('part.gcode')
        await asyncio.wait_for(self.flow.task,3.)
        self.assertEqual(self.flow.state,'complete',self.flow.message)
        self.assertEqual(self.printer.heater.target,0.)
        self.assertEqual(len(self.printer.queued),1)
        actions=[(who,action) for who,action,data in self.events]
        self.assertLess(actions.index(('robot','schedule')),actions.index(('master','commit')))
        scheduled=next(data for who,action,data in self.events if who=='robot' and action=='schedule')
        self.assertAlmostEqual(self.printer.queued[0][0]-1000,scheduled['start_monotonic'],places=2)
        self.assertEqual(self.stops,0)

    async def test_robot_preflight_failure_leaves_heaters_off(self):
        self.robot.fail_plan=True
        await self.preflight()
        self.assertEqual(self.flow.state,'failed')
        self.assertEqual(self.printer.heater.target,0.)
        self.assertFalse(self.printer.queued)
        self.assertIsNone(self.flow.record)

    async def test_heat_timeout_never_starts_robot_planning(self):
        self.printer.heater.follows=False
        await self.preflight()
        self.assertEqual(self.flow.state,'failed')
        self.assertIn('timed out',self.flow.message)
        self.assertEqual(self.printer.heater.target,0.)
        self.assertFalse(any(who=='robot' and action=='prepare' for who,action,data in self.events))

    async def test_clock_failure_does_not_abort_unprepared_robot_session(self):
        # Reproduce a worker retaining a previous successful preflight session.
        await self.preflight()
        old_session = self.robot.session
        self.events.clear()
        async def bad_clock(master):
            raise ParticipantError('Clock uncertainty exceeds 10 ms')
        self.flow.clock_sampler = bad_clock
        await self.preflight()
        self.assertEqual(self.flow.state,'failed')
        self.assertEqual(self.flow.message,'Clock uncertainty exceeds 10 ms')
        self.assertNotEqual(self.flow.session,old_session)
        self.assertEqual(self.robot.session,old_session)
        self.assertEqual(self.robot.state,'ready')
        self.assertFalse(any(w=='robot' and a=='abort' for w,a,_ in self.events))
        self.assertTrue(any(w=='master' and a=='abort' for w,a,_ in self.events))
        self.assertFalse(self.master.leased)
        self.assertEqual(self.printer.heater.target,0.)
        self.assertFalse(self.printer.queued)

    async def test_lost_robot_prepare_reply_still_aborts_that_session(self):
        original_call = self.robot.call
        async def lose_reply(action, **data):
            result = await original_call(action, **data)
            if action == 'prepare': raise ParticipantError('Prepare reply lost')
            return result
        self.robot.call = lose_reply
        await self.preflight()
        self.assertEqual(self.flow.state,'failed')
        self.assertEqual(self.flow.message,'Prepare reply lost')
        self.assertEqual(self.robot.session,self.flow.session)
        self.assertEqual(self.robot.state,'fault')
        self.assertTrue(any(w=='robot' and a=='abort' for w,a,_ in self.events))
        self.assertEqual(self.printer.heater.target,0.)

    async def test_real_robot_abort_failure_remains_visible(self):
        self.robot.fail_plan = True
        original_call = self.robot.call
        async def fail_abort(action, **data):
            if action == 'abort': raise ParticipantError('Worker unreachable during abort')
            return await original_call(action, **data)
        self.robot.call = fail_abort
        await self.preflight()
        self.assertEqual(self.flow.state,'failed')
        self.assertIn('stop errors: Worker unreachable during abort',self.flow.message)
        self.assertEqual(self.printer.heater.target,0.)

    async def test_start_clock_failure_aborts_both_existing_sessions(self):
        await self.preflight()
        self.events.clear()
        async def bad_clock(master):
            raise ParticipantError('Clock uncertainty exceeds 10 ms')
        self.flow.clock_sampler = bad_clock
        await self.flow.start('part.gcode')
        await self.flow.task
        self.assertEqual(self.flow.state,'failed')
        self.assertEqual(self.flow.message,'Clock uncertainty exceeds 10 ms')
        self.assertEqual({w for w,a,_ in self.events if a=='abort'},{'master','robot'})
        self.assertFalse(any(a in ('heat','approach','commit') for _,a,_ in self.events))
        self.assertEqual(self.printer.heater.target,0.)

    async def test_slow_klipper_heartbeat_does_not_starve_robot_lease(self):
        master_started=asyncio.Event()
        master_cancelled=asyncio.Event()
        robot_renewals=[]
        async def slow_master(action, **data):
            self.assertEqual(action,'heartbeat')
            master_started.set()
            try: await asyncio.sleep(60)
            finally: master_cancelled.set()
        async def robot_heartbeat(action, **data):
            self.assertEqual(action,'heartbeat')
            robot_renewals.append(data['session'])
            return {'state':'preflighting'}
        self.flow.master.call=slow_master
        self.robot.call=robot_heartbeat
        self.flow.session='test-session'
        self.flow.master_leased=self.flow.robot_leased=True
        await self.flow._begin_heartbeat()
        await asyncio.wait_for(master_started.wait(),.5)
        await asyncio.sleep(.04)
        await self.flow._end_heartbeat()
        self.assertGreaterEqual(len(robot_renewals),3)
        self.assertTrue(master_cancelled.is_set())
        self.assertIsNone(self.flow.heartbeat_task)

    async def test_stalled_heartbeat_fails_both_participants_before_watchdog(self):
        master_call=self.flow.master.call
        robot_call=self.robot.call
        async def stalled_master(action, **data):
            if action=='heartbeat': await asyncio.sleep(60)
            return await master_call(action, **data)
        async def planning_robot(action, **data):
            result=await robot_call(action, **data)
            if action=='prepare': self.robot.state='preflighting'
            return result
        self.flow.master.call=stalled_master
        self.robot.call=planning_robot
        await self.flow.preflight('part.gcode',{'extruder':False})
        await asyncio.wait_for(self.flow.task,.9)
        self.assertEqual(self.flow.state,'failed')
        self.assertIn('Klipper heartbeat response timed out',self.flow.message)
        self.assertEqual({w for w,a,_ in self.events if a=='abort'},{'master','robot'})
        self.assertEqual(self.printer.heater.target,0.)

    async def test_cancel_heating_turns_targets_off(self):
        self.printer.heater.follows=False
        await self.flow.preflight('part.gcode')
        while self.printer.heater.target==0: await asyncio.sleep(.001)
        await self.flow.abort()
        self.assertEqual(self.printer.heater.target,0.)
        self.assertEqual(self.flow.state,'aborted')
        self.assertEqual(self.stops,0)

    async def test_changed_file_cannot_start_and_concurrent_jobs_are_rejected(self):
        await self.flow.preflight('part.gcode')
        with self.assertRaises(ParticipantError): await self.flow.preflight('part.gcode')
        await self.flow.task
        self.path.write_text('changed')
        with self.assertRaises(ParticipantError): await self.flow.start('part.gcode')
        self.assertFalse(self.printer.queued)

    async def test_robot_fault_stops_extruder_and_uses_independent_program_stop(self):
        await self.preflight()
        self.robot.fail_run=True
        original_call = self.robot.call
        async def slow_abort(action, **data):
            result = await original_call(action, **data)
            if action == 'abort': await asyncio.sleep(.05)
            return result
        self.robot.call = slow_abort
        await self.flow.start('part.gcode')
        await asyncio.wait_for(self.flow.task,3.)
        self.assertEqual(self.flow.state,'failed')
        self.assertTrue(self.printer.shutdowns)
        self.assertEqual(self.printer.heater.target,0.)
        self.assertGreater(self.stops,0)
        self.assertIsNone(self.flow.heartbeat_task)

    async def test_unpreflighted_job_cannot_start(self):
        with self.assertRaises(ParticipantError): await self.flow.start('part.gcode')
        self.assertFalse(self.printer.queued)

    async def test_independent_heater_options_heat_only_selected_heaters(self):
        bed = Heater()
        self.master.heaters['heater_bed'] = bed
        self.master.heater_config['heater_bed'] = {'min_temp':0.,'max_temp':120.}
        self.path.write_text('M140 S60\nM109 S200\n'+ENTRY+'G1 X111 E.1\n')
        for extruder,bed_check in ((True,True),(True,False),(False,True),(False,False)):
            with self.subTest(extruder=extruder,bed=bed_check):
                self.events.clear()
                await self.preflight({'extruder':extruder,'heater_bed':bed_check})
                self.assertEqual(self.flow.state,'ready',self.flow.message)
                targets = [d['targets'] for who,a,d in self.events if who=='master' and a=='heat']
                expected = dict(([('extruder',200.)] if extruder else [])+
                                ([('heater_bed',60.)] if bed_check else []))
                self.assertEqual(targets,[expected] if expected else [])
                self.assertEqual((self.printer.heater.target,bed.target),(0.,0.))
                statuses = {c['name']:c['status'] for c in self.flow.checks}
                self.assertEqual(statuses['Extruder heater'],'pass' if extruder else 'skip')
                self.assertEqual(statuses['Bed heater'],'pass' if bed_check else 'skip')
                self.assertEqual(self.flow.view('part.gcode')['heater_checks'],self.flow.heater_checks)

    async def test_missing_disabled_bed_passes_but_start_still_heats_extruder(self):
        self.path.write_text('M140 S60\nM190 S60\nM109 S200\n'+ENTRY+'G1 X110.1 E.01\n')
        choices = {'extruder':False,'heater_bed':False}
        await self.preflight(choices)
        self.assertEqual(self.flow.state,'ready',self.flow.message)
        self.assertFalse(any(a=='heat' for _,a,_ in self.events))
        self.assertEqual(self.printer.heater.target,0.)
        await self.flow.start('part.gcode',choices)
        await asyncio.wait_for(self.flow.task,3.)
        self.assertEqual(self.flow.state,'complete',self.flow.message)
        self.assertEqual([d['targets'] for w,a,d in self.events if w=='master' and a=='heat'],
                         [{'extruder':200.}])
        self.assertTrue(self.printer.queued)

    async def test_enabled_missing_bed_fails_before_any_heating(self):
        self.path.write_text('M104 S200\nM190 S60\n'+ENTRY+'G1 X111 E.1')
        await self.preflight({'heater_bed':True})
        self.assertEqual(self.flow.state,'failed')
        self.assertIn('not configured: heater_bed',self.flow.message)
        self.assertFalse(any(a=='heat' for _,a,_ in self.events))

    async def test_zero_bed_target_does_not_require_a_bed_sensor(self):
        self.path.write_text('M140 S0\nM190 S0\n'+ENTRY+'G1 X111')
        await self.preflight()
        self.assertEqual(self.flow.state,'ready',self.flow.message)
        check=next(c for c in self.flow.checks if c['name']=='Bed heater')
        self.assertEqual(check['status'],'skip')
        self.assertIn('off',check['detail'])

    async def test_changed_options_cannot_start_a_saved_preflight(self):
        await self.preflight()
        with self.assertRaisesRegex(ParticipantError,'options changed'):
            await self.flow.start('part.gcode',{'extruder':False,'heater_bed':False})
        self.assertFalse(self.printer.queued)

    async def test_disabled_check_does_not_allow_cold_extrusion_at_start(self):
        self.printer.heater.follows=False
        await self.preflight({'extruder':False})
        self.assertEqual(self.flow.state,'ready',self.flow.message)
        await self.flow.start('part.gcode')
        await asyncio.wait_for(self.flow.task,3.)
        self.assertEqual(self.flow.state,'failed')
        self.assertFalse(self.printer.queued)
        self.assertEqual(self.printer.heater.target,0.)
        self.assertFalse(any(a=='approach' for _,a,_ in self.events))


class ClockTests(unittest.IsolatedAsyncioTestCase):
    async def test_mapping_and_stale_clock(self):
        master=SimpleNamespace(call=lambda action:asyncio.sleep(0,result={'print_time':110.}))
        times=iter([10.,10.002,10.1,10.104])
        mapping=await sample_clock(master,clock=lambda:next(times),samples=2)
        self.assertAlmostEqual(mapping.offset,99.999)
        self.assertAlmostEqual(mapping.local_time(111.,now=11.),11.001)
        with self.assertRaises(ParticipantError): mapping.local_time(111.,now=16.)

    async def test_retries_initial_jitter_without_relaxing_uncertainty(self):
        master=SimpleNamespace(call=lambda action:asyncio.sleep(0,result={'print_time':110.}))
        times=iter([10.,10.040,10.1,10.140,10.2,10.240,10.3,10.340,
                    10.4,10.440,10.5,10.508])
        mapping=await sample_clock(master,clock=lambda:next(times),interval=0)
        self.assertAlmostEqual(mapping.uncertainty,.005)
        self.assertAlmostEqual(mapping.offset,99.496)

    async def test_persistent_high_latency_is_rejected_with_measurements(self):
        master=SimpleNamespace(call=lambda action:asyncio.sleep(0,result={'print_time':110.}))
        times=iter(t for i in range(20) for t in (10.+i*.05,10.04+i*.05))
        with self.assertRaisesRegex(ParticipantError,
                r'21.00 ms exceeds 10 ms \(best round trip 40.00 ms across 20 samples\)'):
            await sample_clock(master,clock=lambda:next(times),interval=0)

    async def test_sampling_budget_bounds_slow_rpc(self):
        cancelled = asyncio.Event()
        async def stalled(action):
            try: await asyncio.sleep(60)
            finally: cancelled.set()
        with self.assertRaisesRegex(ParticipantError,'Unable to measure Klipper clock'):
            await asyncio.wait_for(sample_clock(SimpleNamespace(call=stalled),
                                                timeout=.03,interval=0),.5)
        self.assertTrue(cancelled.is_set())

    async def test_recovers_after_transient_connection_failure(self):
        count = 0
        async def clock_rpc(action):
            nonlocal count
            count += 1
            if count == 1: raise ParticipantError('Connection closed')
            return {'print_time':110.}
        times=iter([10.,10.1,10.102])
        mapping=await sample_clock(SimpleNamespace(call=clock_rpc),clock=lambda:next(times),
                                   samples=1,interval=0)
        self.assertAlmostEqual(mapping.uncertainty,.002)

    async def test_invalid_remote_clock_is_never_used(self):
        for remote in (None,True,'110',float('nan'),float('inf')):
            with self.subTest(remote=remote):
                master=SimpleNamespace(call=lambda action:asyncio.sleep(0,result={'print_time':remote}))
                with self.assertRaisesRegex(ParticipantError,'Invalid Klipper clock sample'):
                    await sample_clock(master)


if __name__=='__main__': unittest.main()
