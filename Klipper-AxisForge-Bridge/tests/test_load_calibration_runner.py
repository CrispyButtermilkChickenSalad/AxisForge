import asyncio
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock,Mock,patch
from bridge_dashboard.load_calibration import LoadCalibrationRunner
from bridge_dashboard.bed_probe import BedProbeError, BedProbeRunner


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
        for delta in (True,float('nan'),2.,11.,0.):
            with self.assertRaises(BedProbeError):await self.runner.action('step',self.token,delta)
        self.runner._apply_event(dict(state='waiting_tare',confirmation={'stage':'tare','token':'b'*32}))
        with self.assertRaises(BedProbeError):await self.runner.action('step',self.token,-.01)
        self.runner._send.assert_not_awaited()

    async def test_requested_steps_forward_once_with_fresh_tokens(self):
        for index,delta in enumerate((10.,-10.,1.,-1.,.1,-.1,.05,-.05)):
            token=('%032x' % (index+1))
            self.runner._apply_event(dict(state='adjusting_load',action_token=token))
            await self.runner.action('step',token,delta)
            self.runner._send.assert_awaited_with(dict(command='step',token=token,delta_mm=delta))
            with self.assertRaises(BedProbeError):await self.runner.action('step',token,delta)

    async def test_wire_failure_uses_existing_stop_supervision(self):
        self.runner._send.side_effect=BedProbeError('pipe broke')
        self.runner._schedule_failure_stop=Mock()
        with self.assertRaises(BedProbeError):await self.runner.action('step',self.token,-.01)
        self.runner._schedule_failure_stop.assert_called_once_with('pipe broke')

    async def test_launch_options_are_calibration_only_and_do_not_issue_heater_commands(self):
        self.runner._active=False
        with tempfile.TemporaryDirectory() as folder:
            config=Path(folder)/'printer.yaml';config.write_text('configuration unchanged\n')
            self.runner.settings=SimpleNamespace(axisforge_config=config,ros2_command='ros2',moonraker_url='http://printer:7125')
            with patch.object(BedProbeRunner,'start',new_callable=AsyncMock,return_value={}) as launch:
                await self.runner.start()
                command=self.runner._command(False)
                self.assertEqual(command[-2:],['--calibration-max-counts','120000.0'])
                self.assertNotIn('--calibration-hot-target',command)
                await self.runner.start(calibration_max_counts=150000,calibration_hot_target=210)
                command=self.runner._command(False)
                self.assertEqual(command[-4:],['--calibration-max-counts','150000.0','--calibration-hot-target','210.0'])
                self.assertEqual(launch.await_count,2)
            self.assertEqual(config.read_text(),'configuration unchanged\n')
            self.runner._send.assert_not_awaited()

    async def test_invalid_or_busy_launch_cannot_change_active_options(self):
        with self.assertRaises(BedProbeError):await self.runner.start(calibration_max_counts=200000)
        self.assertEqual(self.runner.status()['calibration_max_counts'],120000.)
        self.runner._active=False
        for kwargs in ({'calibration_max_counts':60000},{'calibration_max_counts':250001},
                       {'calibration_max_counts':True},{'calibration_max_counts':float('nan')},
                       {'calibration_hot_target':49},{'calibration_hot_target':301},
                       {'calibration_hot_target':True},{'calibration_hot_target':float('inf')}):
            with self.subTest(kwargs=kwargs),self.assertRaises(BedProbeError):await self.runner.start(**kwargs)

    async def test_load_limit_allows_only_fresh_small_upward_release(self):
        self.runner._apply_event(dict(state='adjusting_load',action_token=self.token,
                                      upward_only=True,load_limit_reached=True))
        for command,delta in (('step',-.01),('step',1.),('step',10.),('finish',None)):
            with self.subTest(command=command,delta=delta),self.assertRaises(BedProbeError):
                await self.runner.action(command,self.token,delta)
        self.runner._send.assert_not_awaited()
        await self.runner.action('step',self.token,.1)
        self.runner._send.assert_awaited_once_with(dict(command='step',token=self.token,delta_mm=.1))

    async def test_temperature_wait_and_safety_flags_are_validated(self):
        self.runner._apply_event(dict(state='waiting_temperature',temperature_c=209.,target_c=210.,
                                      stable_for_s=12.,required_stable_s=30.,hot_calibration=True))
        self.assertEqual(self.runner.status()['stable_for_s'],12.)
        self.assertIsNone(self.runner.status()['action_token'])
        for data in (dict(state='waiting_temperature',temperature_c=float('nan')),
                     dict(state='load_limit',upward_only='yes')):
            with self.assertRaises(BedProbeError):self.runner._apply_event(data)

    async def test_record_requires_numeric_weight_load_and_fresh_token(self):
        for value in (None, True, 0., 49.9, 25001., float('nan')):
            with self.subTest(grams=value), self.assertRaises(BedProbeError):
                await self.runner.action('record', self.token, grams=value)
        with self.assertRaises(BedProbeError):
            await self.runner.action('record', self.token, grams=100.)
        self.runner._apply_event(dict(state='adjusting_load', action_token=self.token,
                                      unloaded=False, calibration_recorded=False))
        await self.runner.action('record', self.token, grams=137.5)
        self.runner._send.assert_awaited_once_with(dict(command='record', token=self.token, grams=137.5))
        with self.assertRaises(BedProbeError): await self.runner.action('record', self.token, grams=137.5)

    async def test_save_requires_recorded_measurement_and_verified_unloading(self):
        for recorded,unloaded in ((False, True), (True, False), (True, None)):
            self.runner._snapshot.update(calibration_recorded=recorded, unloaded=unloaded)
            with self.assertRaises(BedProbeError): await self.runner.action('finish', self.token)
        self.runner._apply_event(dict(state='adjusting_load', action_token=self.token,
                                      calibration_recorded=True, unloaded=True))
        await self.runner.action('finish', self.token)
        self.runner._send.assert_awaited_once_with(dict(command='finish', token=self.token))
