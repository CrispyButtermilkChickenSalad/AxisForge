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
emit('waiting_surface',confirmation={'stage':stage,'token':'a'*24},trigger_mode='raw',trigger_value=20000,trigger_unit='counts',max_value=60000)
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
