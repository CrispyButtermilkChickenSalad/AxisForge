"""Exercise the dashboard supervisor using a pipe-only child, without ROS/hardware."""
import asyncio
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock, patch

from bridge_dashboard.bed_probe import BedProbeRunner, BedProbeError


CHILD = r'''
import json,sys,time
prefix='AXISFORGE_PROBE '
stage='surface'
def emit(state,**data):
    print(prefix+json.dumps(dict(state=state,**data)),flush=True)
emit('waiting_surface',confirmation={'stage':stage,'token':'a'*24},trigger_mode='raw',trigger_value=20000,trigger_unit='counts',max_value=60000,max_descent_mm=350.)
for line in sys.stdin:
    data=json.loads(line)
    if data['command']=='abort':
        emit('aborted',message='stopped');sys.exit(1)
    if data['command']=='confirm':
        if stage=='surface':
            stage='front_left'
            emit('waiting_front_left',confirmation={'stage':stage,'token':'b'*24},progress=.2)
        else:
            emit('complete',message='mesh saved',progress=1.,current_point=9)
            time.sleep(.15)
            sys.exit(0)
'''


class ProbeRunnerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name)/'config.yaml'; self.config.write_text('test: true\n')
        self.settings = SimpleNamespace(axisforge_config=self.config, ros2_command='ros2',
            moonraker_url='http://printer:7125', moonraker_api_key='do-not-put-this-in-argv')
        self.stop = AsyncMock(return_value={'stopped':True})
        self.invalidate = Mock()
        self.runner = BedProbeRunner(self.settings, robot_stop=self.stop,
            invalidate_preflight=self.invalidate, heartbeat_s=.03, stop_timeout_s=.3)
        self.runner._command = lambda dry_run: [sys.executable, '-u', '-c', CHILD]

    async def asyncTearDown(self):
        if self.runner._stop_unconfirmed:
            # The test has no robot; explicitly release only its fake process.
            self.runner._stop_unconfirmed = False
            if self.runner._process.returncode is None: self.runner._process.kill()
        await self.runner.close()
        if self.runner._failure_stop is not None:
            await self.runner._failure_stop

    async def state(self, expected):
        async def wait():
            while self.runner.status()['state'] != expected: await asyncio.sleep(.005)
        await asyncio.wait_for(wait(),2.)
        return self.runner.status()

    async def test_confirmations_are_explicit_single_use_and_stage_bound(self):
        await self.runner.start()
        first = await self.state('waiting_surface')
        self.assertTrue(first['busy']); self.assertEqual(first['trigger_value'],20000)
        self.assertEqual(first['max_descent_mm'],350.)
        await asyncio.sleep(.1)
        self.assertEqual(self.runner.status()['state'],'waiting_surface')
        with self.assertRaisesRegex(BedProbeError,'stale'): await self.runner.confirm('front_left','a'*24)
        await self.runner.confirm('surface','a'*24)
        with self.assertRaisesRegex(BedProbeError,'stale'): await self.runner.confirm('surface','a'*24)
        await self.state('waiting_front_left')
        await self.runner.confirm('front_left','b'*24)
        await self.state('complete')
        self.assertTrue(self.runner.busy(),'A completion event must not release ownership before exit')
        await asyncio.wait_for(self.runner._monitor,2.)
        self.assertFalse(self.runner.busy()); self.assertEqual(self.invalidate.call_count,2)
        self.stop.assert_not_awaited()

    async def test_abort_waiting_for_confirmation_has_no_robot_stop_fallback(self):
        await self.runner.start(); await self.state('waiting_surface')
        result = await self.runner.abort()
        self.assertEqual(result['state'],'aborted'); self.assertFalse(result['busy'])
        self.stop.assert_not_awaited()

    async def test_automatic_home_recovery_keeps_ownership_and_exposes_original_failure(self):
        child=CHILD.replace("stage='front_left'\n            emit('waiting_front_left',confirmation={'stage':stage,'token':'b'*24},progress=.2)",
            "stage='recovering'\n            emit('recovering_home',failure_recovery={'state':'returning','failure':'Repeatability failed'})")
        self.runner._command=lambda dry_run:[sys.executable,'-u','-c',child]
        await self.runner.start();await self.state('waiting_surface')
        await self.runner.confirm('surface','a'*24)
        status=await self.state('recovering_home')
        self.assertTrue(status['busy'])
        self.assertEqual(status['last_failure'],'Repeatability failed')
        self.assertEqual(status['failure_recovery']['state'],'returning')
        self.assertIsNone(status['confirmation']);self.assertIsNone(status['action_token'])
        with self.assertRaises(BedProbeError):await self.runner.action('step','a'*24,1.)
        with self.assertRaises(BedProbeError):await self.runner.start()
        await self.runner.abort()
        self.assertEqual(self.runner.status()['state'],'aborted')

    async def test_returned_home_failure_never_becomes_success(self):
        child='import json;print("AXISFORGE_PROBE "+json.dumps({"state":"failed", "message":"Repeatability failed; returned Home", "failure_recovery":{"state":"returned", "failure":"Repeatability failed"}}),flush=True);raise SystemExit(1)'
        self.runner._command=lambda dry_run:[sys.executable,'-u','-c',child]
        await self.runner.start();await asyncio.wait_for(self.runner._monitor,2.)
        status=self.runner.status()
        self.assertEqual(status['state'],'failed');self.assertFalse(status['busy'])
        self.assertEqual(status['failure_recovery']['state'],'returned')
        self.assertIn('Repeatability failed',status['last_failure'])
        self.stop.assert_not_awaited()

    async def test_alignment_keeps_ownership_and_blocks_z_and_repeat_confirmation(self):
        child=CHILD.replace("stage='front_left'\n            emit('waiting_front_left',confirmation={'stage':stage,'token':'b'*24},progress=.2)",
            "stage='aligning'\n            emit('aligning_nozzle',alignment={'required':True,'state':'aligning','angle_deg':.255})")
        self.runner._command=lambda dry_run:[sys.executable,'-u','-c',child]
        await self.runner.start();await self.state('waiting_surface')
        await self.runner.confirm('surface','a'*24)
        status=await self.state('aligning_nozzle')
        self.assertTrue(status['busy'])
        self.assertEqual(status['alignment']['angle_deg'],.255)
        self.assertIsNone(status['confirmation'])
        self.assertIsNone(status['action_token'])
        self.assertIsNone(status['z_controls'])
        with self.assertRaises(BedProbeError):await self.runner.action('step','a'*24,1.)
        with self.assertRaises(BedProbeError):await self.runner.confirm('surface','a'*24)
        await self.runner.abort()
        self.assertFalse(self.runner.busy())

    async def test_completed_mesh_can_be_redone_and_cancelled_without_deleting_saved_mesh(self):
        active=Path(self.temp.name)/'bed.yaml'
        old='complete: true\npoints: [{nozzle: [0, 0, 1]}]\nstamp: original\n'
        active.write_text(old)
        self.config.write_text('gcode_interpreter:\n  ros__parameters:\n    bed_probe_file: '+str(active)+'\n')
        self.assertTrue(self.runner.status()['saved_mesh']['available'])
        for _ in range(2):
            await self.runner.start()
            await self.state('waiting_surface')
            await self.runner.confirm('surface','a'*24)
            await self.state('waiting_front_left')
            await self.runner.confirm('front_left','b'*24)
            await asyncio.wait_for(self.runner._monitor,2.)
            self.assertEqual(self.runner.status()['state'],'complete')
            self.assertEqual(active.read_text(),old)
        await self.runner.start(thermal={'nozzle_target_c':210.,'bed_target_c':0.,'soak_s':60.})
        await self.state('waiting_surface');await self.runner.abort()
        self.assertEqual(active.read_text(),old)
        self.assertEqual(self.runner.status()['thermal_options']['nozzle_target_c'],210.)
        self.stop.assert_not_awaited()

    async def test_temperature_options_reach_child_and_invalid_request_never_launches(self):
        with self.assertRaises(BedProbeError):
            await self.runner.start(thermal={'nozzle_target_c':500,'bed_target_c':60,'soak_s':30})
        self.assertFalse(self.runner.busy())
        await self.runner.start(thermal={'nozzle_target_c':210.,'bed_target_c':60.,'soak_s':5.})
        command=BedProbeRunner._command(self.runner,False)
        self.assertEqual(command[command.index('--probe-nozzle-target')+1],'210.0')
        self.assertEqual(command[command.index('--probe-bed-target')+1],'60.0')
        self.assertEqual(command[command.index('--probe-soak-s')+1],'5.0')
        await self.state('waiting_surface');await self.runner.abort()

    async def test_retry_after_failure_or_abort_starts_fresh_and_requires_new_confirmation(self):
        failed_child = r'''
import json,sys,uuid
prefix='AXISFORGE_PROBE '
token=uuid.uuid4().hex
print('log from previous attempt',flush=True)
print(prefix+json.dumps(dict(state='waiting_surface',progress=.4,current_point=3,
    result={'obsolete':True},confirmation={'stage':'surface','token':token})),flush=True)
for line in sys.stdin:
    command=json.loads(line)['command']
    if command in ('confirm','abort'):
        print(prefix+json.dumps(dict(state='failed' if command=='confirm' else 'aborted',
            message='Previous attempt ended')),flush=True)
        sys.exit(1)
'''
        fresh_child = CHILD.replace("'token':'a'*24", "'token':__import__('uuid').uuid4().hex")
        for ending in ('failed', 'aborted'):
            with self.subTest(ending=ending):
                self.runner._command = lambda dry_run: [sys.executable, '-u', '-c', failed_child]
                await self.runner.start()
                old = (await self.state('waiting_surface'))['confirmation']
                if ending == 'failed':
                    await self.runner.confirm(old['stage'], old['token'])
                else:
                    await self.runner.abort()
                await asyncio.wait_for(self.runner._monitor, 2.)
                self.assertEqual(self.runner.status()['state'], ending)
                self.assertEqual(self.runner.status()['result'], {'obsolete': True})
                self.assertTrue(self.runner.status()['logs'])
                self.runner._command = lambda dry_run: [sys.executable, '-u', '-c', fresh_child]
                started = await self.runner.start()
                self.assertEqual(started['progress'], 0.)
                self.assertEqual(started['current_point'], 0)
                self.assertEqual(started['logs'], [])
                self.assertNotIn('result', started)
                self.assertIsNone(started['confirmation'])
                fresh = (await self.state('waiting_surface'))['confirmation']
                self.assertNotEqual(fresh['token'], old['token'])
                with self.assertRaisesRegex(BedProbeError, 'stale'):
                    await self.runner.confirm(old['stage'], old['token'])
                await asyncio.sleep(.05)
                self.assertEqual(self.runner.status()['confirmation'], fresh)
                self.assertEqual(self.runner.status()['state'], 'waiting_surface')
                await self.runner.abort()
        self.stop.assert_not_awaited()

    async def test_abrupt_child_exit_is_failure_even_with_zero_status(self):
        self.runner._command = lambda dry_run:[sys.executable,'-c','print("sensor disconnected")']
        await self.runner.start()
        await asyncio.wait_for(self.runner._monitor,2.)
        result = self.runner.status()
        self.assertEqual(result['state'],'failed'); self.assertIn('sensor disconnected',result['message'])
        self.stop.assert_awaited_once()

    async def test_complete_event_with_nonzero_exit_is_failure(self):
        self.runner._command = lambda dry_run:[sys.executable,'-c',
            'print(\'AXISFORGE_PROBE {"state":"complete"}\',flush=True); raise SystemExit(1)']
        await self.runner.start(); await asyncio.wait_for(self.runner._monitor,2.)
        self.assertEqual(self.runner.status()['state'],'failed')
        self.stop.assert_awaited_once()

    async def test_structured_child_failure_keeps_its_reason_after_ros2run_exit(self):
        self.runner._command = lambda dry_run:[sys.executable,'-c',
            'print(\'AXISFORGE_PROBE {"state":"failed","message":"Load cell is too noisy"}\',flush=True); '
            'print("[ros2run]: Process exited with failure 1"); raise SystemExit(1)']
        await self.runner.start(); await asyncio.wait_for(self.runner._monitor,2.)
        self.assertEqual(self.runner.status()['message'],'Load cell is too noisy')
        self.assertFalse(self.runner.busy()); self.stop.assert_not_awaited()

    async def test_unexpected_exit_with_unconfirmed_ur_stop_keeps_pipeline_blocked(self):
        self.runner._command = lambda dry_run:[sys.executable,'-c','raise SystemExit(7)']
        self.stop.side_effect = RuntimeError('UR unavailable')
        await self.runner.start(); await asyncio.wait_for(self.runner._monitor,2.)
        self.assertTrue(self.runner.busy()); self.assertTrue(self.runner.status()['stop_unconfirmed'])
        self.assertIn('pendant stop',self.runner.status()['message'])

    async def test_unexpected_simulation_exit_does_not_stop_real_robot(self):
        self.runner._command = lambda dry_run:[sys.executable,'-c','raise SystemExit(7)']
        await self.runner.start(dry_run=True); await asyncio.wait_for(self.runner._monitor,2.)
        self.assertFalse(self.runner.busy()); self.stop.assert_not_awaited()

    async def test_duplicate_start_and_stale_event_cannot_restore_consumed_token(self):
        await self.runner.start(); await self.state('waiting_surface')
        with self.assertRaisesRegex(BedProbeError,'already active'): await self.runner.start()
        await self.runner.confirm('surface','a'*24)
        self.runner._apply_event({'state':'waiting_surface','confirmation':{'stage':'surface','token':'a'*24}})
        self.assertIsNone(self.runner.status()['confirmation'])

    async def test_delayed_failure_stop_cannot_abort_retry_or_suppress_its_own_failure_stop(self):
        for new_failure in (False, True):
            with self.subTest(new_failure=new_failure):
                await self.runner.start()
                await self.state('waiting_surface')
                old_process = self.runner._process
                # Model a failure stop queued behind an earlier abort while
                # the old child independently completes its cleanup.
                async with self.runner._abort_lock:
                    self.runner._schedule_failure_stop('old attempt failed')
                    old_stop = self.runner._failure_stop
                    await self.runner._send({'command': 'abort'})
                    await asyncio.wait_for(self.runner._monitor, 2.)
                    self.assertFalse(old_stop.done())
                    await self.runner.start()
                    await self.state('waiting_surface')
                    self.assertIsNot(self.runner._process, old_process)
                    if new_failure:
                        self.runner._schedule_failure_stop('new attempt failed')
                        self.assertIsNot(self.runner._failure_stop, old_stop)
                await asyncio.wait_for(old_stop, 2.)
                if new_failure:
                    await asyncio.wait_for(self.runner._failure_stop, 2.)
                    self.assertFalse(self.runner.busy())
                    self.assertEqual(self.runner.status()['state'], 'failed')
                    self.assertEqual(self.runner.status()['message'], 'new attempt failed')
                else:
                    self.assertTrue(self.runner.busy())
                    self.assertEqual(self.runner.status()['state'], 'waiting_surface')
                    self.assertIsNone(self.runner._fatal_error)
                    await self.runner.abort()
        self.stop.assert_not_awaited()

    async def test_simulation_cannot_be_reported_as_live_mesh(self):
        await self.runner.start(dry_run=True); await self.state('waiting_surface')
        await self.runner.confirm('surface','a'*24); await self.state('waiting_front_left')
        await self.runner.confirm('front_left','b'*24)
        await asyncio.wait_for(self.runner._monitor,2.)
        self.assertTrue(self.runner.status()['dry_run'])
        self.assertIn('active bed map was not changed',self.runner.status()['message'])

    async def test_unresponsive_child_requires_ur_stop_before_process_termination(self):
        self.runner._command = lambda dry_run:[sys.executable,'-c','import time;time.sleep(60)']
        await self.runner.start()
        result = await self.runner.abort()
        self.stop.assert_awaited_once()
        self.assertFalse(result['busy']); self.assertEqual(result['state'],'aborted')

    async def test_unconfirmed_robot_stop_latches_interlock(self):
        self.runner._command = lambda dry_run:[sys.executable,'-c','import time;time.sleep(60)']
        self.stop.side_effect = RuntimeError('dashboard disconnected')
        await self.runner.start()
        with self.assertRaisesRegex(BedProbeError,'pendant stop'): await self.runner.abort()
        self.assertTrue(self.runner.busy()); self.assertTrue(self.runner.status()['stop_unconfirmed'])
        with self.assertRaisesRegex(BedProbeError,'already active'): await self.runner.start()

    async def test_api_key_is_only_in_child_environment(self):
        runner = BedProbeRunner(self.settings,robot_stop=self.stop,invalidate_preflight=self.invalidate)
        command = runner._command(False)
        self.assertNotIn(self.settings.moonraker_api_key,' '.join(command))
        spawn = AsyncMock(side_effect=OSError('missing executable'))
        runner.spawn = spawn
        with self.assertRaisesRegex(BedProbeError,'launch'): await runner.start()
        self.assertEqual(spawn.call_args.kwargs['env']['MOONRAKER_API_KEY'],self.settings.moonraker_api_key)
        self.assertFalse(runner.busy())

    async def test_local_probe_transport_profile_is_scoped_to_child_and_preserves_override(self):
        profile=self.config.with_name('probe_dds.xml');profile.write_text('<profiles/>')
        for override in (None, '/configured/operator-dds.xml'):
            runner=BedProbeRunner(self.settings,robot_stop=self.stop,invalidate_preflight=self.invalidate)
            runner.spawn=AsyncMock(side_effect=OSError('test executable absent'))
            with patch.dict('os.environ', {}, clear=True):
                if override is not None:
                    import os
                    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=override
                with self.assertRaises(BedProbeError):await runner.start()
                env=runner.spawn.call_args.kwargs['env']
                self.assertEqual(env['FASTRTPS_DEFAULT_PROFILES_FILE'],override or str(profile))
                self.assertEqual(env['OPENBLAS_NUM_THREADS'],'1')

    async def test_abort_waits_for_spawn_and_then_stops_that_process(self):
        entered, release = asyncio.Event(), asyncio.Event()
        async def spawn(*args, **kwargs):
            entered.set(); await release.wait()
            return await asyncio.create_subprocess_exec(*args, **kwargs)
        self.runner.spawn = spawn
        start = asyncio.create_task(self.runner.start())
        await entered.wait()
        stop = asyncio.create_task(self.runner.abort())
        await asyncio.sleep(.02); self.assertFalse(stop.done())
        release.set(); await start
        result = await asyncio.wait_for(stop, 2.)
        self.assertFalse(result['busy']); self.stop.assert_not_awaited()

    async def test_cancelled_start_request_does_not_orphan_the_child(self):
        entered, release = asyncio.Event(), asyncio.Event()
        async def spawn(*args, **kwargs):
            entered.set(); await release.wait()
            return await asyncio.create_subprocess_exec(*args, **kwargs)
        self.runner.spawn = spawn
        start = asyncio.create_task(self.runner.start())
        await entered.wait(); start.cancel(); release.set()
        with self.assertRaises(asyncio.CancelledError): await start
        await asyncio.wait_for(self.runner._failure_stop,2.)
        self.assertFalse(self.runner.busy())
        self.assertIsNotNone(self.runner._process.returncode)

    async def test_child_reported_unconfirmed_stop_remains_blocked_after_exit(self):
        self.runner._command = lambda dry_run:[sys.executable,'-c',
            'print(\'AXISFORGE_PROBE {"state":"failed","stop_unconfirmed":true,"message":"Use pendant stop"}\',flush=True)']
        await self.runner.start(); await asyncio.wait_for(self.runner._monitor,2.)
        self.assertTrue(self.runner.busy()); self.assertTrue(self.runner.status()['stop_unconfirmed'])
        with self.assertRaisesRegex(BedProbeError,'already active'): await self.runner.start()


if __name__ == '__main__': unittest.main()
