import asyncio
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock,Mock
from bridge_dashboard.load_calibration import LoadCalibrationRunner
from bridge_dashboard.bed_probe import BedProbeError


class CalibrationRunnerTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.runner=LoadCalibrationRunner(SimpleNamespace(axisforge_config=None),
            robot_stop=AsyncMock(),invalidate_preflight=Mock())
        self.runner._active=True
        self.runner._send=AsyncMock()
        self.token='a'*32
        self.runner._apply_event(dict(state='adjusting_load',action_token=self.token,delta_counts=12000.))

    async def test_duplicate_clicks_send_one_step_and_finish_requires_a_new_token(self):
        results=await asyncio.gather(*(self.runner.action('step',self.token,-.01) for _ in range(2)),return_exceptions=True)
        self.assertEqual(sum(isinstance(r,BedProbeError) for r in results),1)
        self.runner._send.assert_awaited_once()
        with self.assertRaises(BedProbeError):await self.runner.action('finish',self.token)
        self.runner._apply_event(dict(state='adjusting_load',action_token=self.token))
        self.assertIsNone(self.runner.status()['action_token'])

    async def test_expired_or_out_of_stage_or_invalid_steps_are_rejected(self):
        self.runner._action_received_at=time.monotonic()-10.
        with self.assertRaises(BedProbeError):await self.runner.action('step',self.token,.01)
        self.runner._action_received_at=time.monotonic()
        for delta in (True,float('nan'),1.,0.):
            with self.assertRaises(BedProbeError):await self.runner.action('step',self.token,delta)
        self.runner._apply_event(dict(state='waiting_tare',confirmation={'stage':'tare','token':'b'*32}))
        with self.assertRaises(BedProbeError):await self.runner.action('step',self.token,-.01)
        self.runner._send.assert_not_awaited()

    async def test_wire_failure_uses_existing_stop_supervision(self):
        self.runner._send.side_effect=BedProbeError('pipe broke')
        self.runner._schedule_failure_stop=Mock()
        with self.assertRaises(BedProbeError):await self.runner.action('step',self.token,-.01)
        self.runner._schedule_failure_stop.assert_called_once_with('pipe broke')
