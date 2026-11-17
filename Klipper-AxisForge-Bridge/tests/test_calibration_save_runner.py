"""A child must exit safely before Klipper save; no printer or robot access."""
import asyncio
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock

from bridge_dashboard.load_calibration import LoadCalibrationRunner
from bridge_dashboard.bed_probe import BedProbeError


VALUES = dict(counts_per_gram=1000., reference_tare_counts=-200000, grams=100.,
              accepted=True, saved=False, config_fingerprint='a'*64,
              other_config_fingerprint='b'*64)


class CalibrationSaveRunnerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        config = Path(self.folder.name)/'ur.yaml'; config.write_text('test: true\n')
        self.save_started, self.save_release = asyncio.Event(), asyncio.Event()
        self.stop = AsyncMock()
        self.saved = []
        self.fail_save = False
        async def save(values, report):
            self.saved.append(values)
            self.assertEqual(self.runner._process.returncode, 0)
            self.assertTrue(self.runner.busy())
            report('verifying_config')
            self.save_started.set()
            await self.save_release.wait()
            if self.fail_save: raise RuntimeError('restart not observed')
            return dict(values, saved=True, restart_confirmed=True)
        self.runner = LoadCalibrationRunner(SimpleNamespace(axisforge_config=config,
            moonraker_url='http://not-used', moonraker_api_key=None), robot_stop=self.stop,
            invalidate_preflight=Mock(), save_calibration=save, heartbeat_s=.01, stop_timeout_s=.2)

    async def asyncTearDown(self):
        self.save_release.set()
        await self.runner.close()
        if self.runner._monitor: await self.runner._monitor

    async def launch(self, *, code=0, state='ready_to_save', dry_run=False):
        payload = dict(state=state, calibration_values=VALUES, unloaded=True, calibration_recorded=True)
        script = ('import time,sys\nprint('+repr('AXISFORGE_PROBE '+json.dumps(payload))+
                  ',flush=True)\ntime.sleep(.15)\nsys.exit('+str(code)+')\n')
        self.runner._command = lambda dry: [sys.executable, '-u', '-c', script]
        await self.runner.start(dry_run=dry_run)
        deadline = asyncio.get_running_loop().time()+2.
        while self.runner.status()['state'] == 'starting':
            self.assertLess(asyncio.get_running_loop().time(), deadline)
            await asyncio.sleep(.005)

    async def test_handoff_waits_for_exit_and_remains_busy_until_verified(self):
        await self.launch()
        self.assertIsNone(self.runner._process.returncode)
        self.assertFalse(self.saved)
        await asyncio.wait_for(self.save_started.wait(), 2.)
        self.assertEqual(self.runner.status()['state'], 'verifying_config')
        self.assertTrue(self.runner.busy())
        await self.runner.abort()
        self.assertTrue(self.runner.busy(), 'Save cannot be falsely reported as undone')
        self.save_release.set()
        await asyncio.wait_for(self.runner._monitor, 2.)
        self.assertEqual(self.runner.status()['state'], 'complete')
        self.assertTrue(self.runner.status()['result']['saved'])
        self.assertFalse(self.runner.busy())
        self.assertEqual(self.saved, [VALUES])
        self.stop.assert_not_awaited()

    async def test_nonzero_child_exit_never_saves(self):
        await self.launch(code=1)
        await asyncio.wait_for(self.runner._monitor, 2.)
        self.assertFalse(self.saved)
        self.assertEqual(self.runner.status()['state'], 'failed')
        self.stop.assert_awaited_once()

    async def test_unverified_save_is_visible_and_never_retried(self):
        self.fail_save = True
        self.save_release.set()
        await self.launch()
        await asyncio.wait_for(self.runner._monitor, 2.)
        self.assertEqual(self.runner.status()['state'], 'save_failed')
        self.assertIn('restart not observed', self.runner.status()['message'])
        self.assertEqual(len(self.saved), 1)

    async def test_cancel_before_child_exit_prevents_save(self):
        await self.launch()
        await self.runner.abort()
        self.assertFalse(self.saved)
        self.assertEqual(self.runner.status()['state'], 'aborted')

    async def test_simulation_never_saves_native_configuration(self):
        await self.launch(state='complete', dry_run=True)
        await asyncio.wait_for(self.runner._monitor, 2.)
        self.assertFalse(self.saved)
        self.assertEqual(self.runner.status()['state'], 'complete')

    async def test_old_completion_without_native_handoff_is_not_saved_success(self):
        await self.launch(state='complete')
        await asyncio.wait_for(self.runner._monitor, 2.)
        self.assertFalse(self.saved)
        self.assertEqual(self.runner.status()['state'], 'save_failed')

    async def test_contradictory_or_missing_unloaded_handoff_is_rejected(self):
        for values in ({'unloaded': False, 'calibration_recorded': True},
                       {'unloaded': True, 'calibration_recorded': False}, {}):
            with self.subTest(values=values), self.assertRaises(BedProbeError):
                self.runner._apply_event(dict(state='ready_to_save', calibration_values=VALUES, **values))
        self.assertFalse(self.saved)

    async def test_abort_during_heartbeat_teardown_does_not_start_save(self):
        async def heartbeat():
            try: await asyncio.Event().wait()
            finally:
                self.runner._aborting = True
                self.runner._snapshot['state'] = 'stopping'
        self.runner._heartbeat = heartbeat
        await self.launch()
        await asyncio.wait_for(self.runner._monitor, 2.)
        self.assertFalse(self.saved)
        self.assertEqual(self.runner.status()['state'], 'aborted')
