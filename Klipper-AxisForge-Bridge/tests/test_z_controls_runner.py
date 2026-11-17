"""Shared dashboard Z controls with pipe mocks; no ROS or hardware commands."""
import asyncio
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock

from bridge_dashboard.bed_probe import BedProbeRunner, BedProbeError
from bridge_dashboard.load_calibration import LoadCalibrationRunner


def z_event(*, phase='positioning', token='a'*32, loaded=False, **changes):
    controls = dict(phase=phase, position_mm=[1., 2., 100.], lower_z_mm=90., upper_z_mm=110.,
        delta_counts=25000. if loaded else 0., upward_only=phase == 'recovery' or loaded,
        loaded=loaded, can_continue=phase == 'positioning' and not loaded,
        can_finish=phase == 'recovery' and not loaded, step_sizes=[.05, .1, 1., 10.])
    controls.update(changes)
    return dict(state='positioning_z' if phase == 'positioning' else 'recovering_z',
                action_token=token, z_controls=controls)


class ZControlsRunnerTests(unittest.IsolatedAsyncioTestCase):
    def runners(self):
        for runner_type in (BedProbeRunner, LoadCalibrationRunner):
            runner = runner_type(SimpleNamespace(axisforge_config=None),
                robot_stop=AsyncMock(), invalidate_preflight=Mock())
            runner._active = True
            runner._send = AsyncMock()
            yield runner

    async def test_duplicate_z_clicks_send_one_step_in_both_routines(self):
        for runner in self.runners():
            with self.subTest(runner=type(runner).__name__):
                runner._apply_event(z_event())
                results = await asyncio.gather(*(runner.action('step', 'a'*32, -.05)
                                                for _ in range(2)), return_exceptions=True)
                self.assertEqual(sum(isinstance(value, BedProbeError) for value in results), 1)
                runner._send.assert_awaited_once_with(dict(command='step', token='a'*32, delta_mm=-.05))
                runner._apply_event(z_event())
                self.assertIsNone(runner.status()['action_token'])
                self.assertTrue(runner.busy())

    async def test_positioning_and_recovery_have_distinct_completion_actions(self):
        for runner in self.runners():
            for phase, command, wrong in (('positioning', 'continue', 'finish_recovery'),
                                           ('recovery', 'finish_recovery', 'continue')):
                with self.subTest(runner=type(runner).__name__, phase=phase):
                    token = ('b' if phase == 'positioning' else 'c')*32
                    runner._apply_event(z_event(phase=phase, token=token))
                    with self.assertRaises(BedProbeError): await runner.action(wrong, token)
                    await runner.action(command, token)
                    runner._send.assert_awaited_with(dict(command=command, token=token))
                    self.assertTrue(runner.busy(), 'Finishing a recovery must not release ownership before child exit')

    async def test_loaded_recovery_allows_only_small_upward_steps(self):
        for runner in self.runners():
            runner._apply_event(z_event(phase='recovery', loaded=True))
            for command, delta in (('step', -10.), ('step', -.05), ('step', 1.),
                                    ('step', 10.), ('continue', None), ('finish_recovery', None)):
                with self.subTest(runner=type(runner).__name__, command=command, delta=delta):
                    with self.assertRaises(BedProbeError): await runner.action(command, 'a'*32, delta)
            runner._send.assert_not_awaited()
            await runner.action('step', 'a'*32, .05)
            runner._send.assert_awaited_once_with(dict(command='step', token='a'*32, delta_mm=.05))
            runner._apply_event(z_event(phase='recovery', token='b'*32))
            with self.assertRaises(BedProbeError): await runner.action('step', 'b'*32, -.05)
            await runner.action('step', 'b'*32, 10.)

    async def test_positioning_contact_also_requires_small_upward_release(self):
        for runner in self.runners():
            runner._apply_event(z_event(loaded=True))
            for command, delta in (('continue', None), ('step', -.05), ('step', 1.)):
                with self.assertRaises(BedProbeError): await runner.action(command, 'a'*32, delta)
            await runner.action('step', 'a'*32, .1)

    async def test_release_load_rise_blocks_steps_until_native_guard_clears(self):
        for runner in self.runners():
            runner._apply_event(z_event(phase='recovery', loaded=True, release_blocked=True))
            with self.assertRaisesRegex(BedProbeError,'Load rose'):
                await runner.action('step','a'*32,.05)
            runner._send.assert_not_awaited()
            self.assertTrue(runner.busy())
            runner._apply_event(z_event(phase='recovery', loaded=True, release_blocked=False, token='b'*32))
            await runner.action('step','b'*32,.05)
            runner._send.assert_awaited_once()

    async def test_prior_failure_survives_abort_and_fresh_status(self):
        for runner in self.runners():
            runner._apply_event(dict(state='failed',message='Load increased during upward release'))
            runner._apply_event(dict(state='aborted',message='Operator cancelled bed probing'))
            runner._apply_event(dict(state='starting',message='Starting checks'))
            self.assertEqual(runner.status()['last_failure'],'Load increased during upward release')

    async def test_recovery_reports_do_not_erase_the_upward_step_failure_history(self):
        for runner in self.runners():
            recovery=z_event(phase='recovery',loaded=True,failure='Contact height rejected')
            runner._apply_event(recovery)
            runner._apply_event(dict(state='z_step_rejected',message='Up step stopped: load increased'))
            runner._apply_event(recovery)
            runner._apply_event(dict(state='aborted',message='Operator cancelled'))
            self.assertEqual(runner.status()['failure_history'],
                ['Contact height rejected','Up step stopped: load increased'])

    async def test_step_distances_and_session_travel_bounds_are_enforced(self):
        for runner in self.runners():
            runner._apply_event(z_event(upper_z_mm=100.5, lower_z_mm=99.5))
            for delta in (1., -1., 10., -10., 2., 0., True, float('nan'), None):
                with self.subTest(runner=type(runner).__name__, delta=delta):
                    with self.assertRaises(BedProbeError): await runner.action('step', 'a'*32, delta)
            with self.assertRaises(BedProbeError): await runner.action('continue', 'a'*32, .05)
            runner._send.assert_not_awaited()
            await runner.action('step', 'a'*32, -.1)

    async def test_expiry_busy_terminal_and_unconfirmed_stop_cannot_use_old_controls(self):
        for runner in self.runners():
            runner._apply_event(z_event())
            runner._action_received_at = time.monotonic()-10.
            runner._apply_event(z_event())  # Replayed status cannot extend the token's life.
            with self.assertRaises(BedProbeError): await runner.action('step', 'a'*32, .1)
            for flag in ('_stop_unconfirmed', '_aborting', '_fatal_error'):
                runner._apply_event(z_event(token=flag*3))
                setattr(runner, flag, True)
                with self.assertRaises(BedProbeError): await runner.action('step', flag*3, .1)
                setattr(runner, flag, False)
            for state in ('z_stepping', 'z_step_rejected', 'failed', 'aborted', 'starting'):
                runner._apply_event(z_event(token='c'*32))
                runner._apply_event(dict(state=state, action_token='c'*32))
                self.assertIsNone(runner.status()['action_token'])
                self.assertIsNone(runner.status()['z_controls'])
                with self.assertRaises(BedProbeError): await runner.action('step', 'c'*32, .1)
            runner._apply_event(z_event(token='d'*32))
            runner._active = False
            with self.assertRaises(BedProbeError): await runner.action('step', 'd'*32, .1)
            runner._send.assert_not_awaited()

    async def test_small_feedback_error_at_travel_limit_allows_only_inward_step(self):
        for runner in self.runners():
            for measured, inward, outward, token in ((110.05, -.05, .05, 'a'*32),
                                                      (89.95, .05, -.05, 'b'*32)):
                with self.subTest(runner=type(runner).__name__, measured=measured):
                    runner._apply_event(z_event(token=token, position_mm=[1., 2., measured]))
                    with self.assertRaisesRegex(BedProbeError, 'travel range'):
                        await runner.action('step', token, outward)
                    await runner.action('step', token, inward)
                    runner._send.assert_awaited_with(dict(command='step', token=token, delta_mm=inward))
            for measured in (110.1001, 89.8999):
                with self.assertRaisesRegex(BedProbeError, 'outside its travel'):
                    runner._apply_event(z_event(position_mm=[1., 2., measured]))

    async def test_corrupt_z_feedback_is_rejected_before_enabling_controls(self):
        invalid = [dict(phase='unknown'), dict(position_mm=[0., 1.]),
            dict(position_mm=[0., 1., float('nan')]), dict(position_mm=[0., 1., 200.]),
            dict(lower_z_mm=True), dict(delta_counts=float('inf')), dict(loaded='yes'),
            dict(step_sizes=[[]]), dict(step_sizes=[True]), dict(step_sizes=[1., 1.]),
            dict(step_sizes=[2.]), dict(failure=12), dict(can_finish=True)]
        for runner in self.runners():
            for change in invalid:
                with self.subTest(runner=type(runner).__name__, change=change):
                    with self.assertRaises(BedProbeError): runner._apply_event(z_event(**change))
            for change in (dict(upward_only=False), dict(can_continue=True)):
                with self.assertRaises(BedProbeError): runner._apply_event(z_event(phase='recovery', **change))
            for field in ('can_continue', 'can_finish'):
                with self.assertRaises(BedProbeError): runner._apply_event(z_event(loaded=True, **{field:True}))
            with self.assertRaises(BedProbeError): runner._apply_event(dict(state='positioning_z', action_token='a'*32))
            with self.assertRaises(BedProbeError): runner._apply_event(z_event(token=True))
            runner._send.assert_not_awaited()

    async def test_pipe_failure_keeps_existing_stop_supervision(self):
        for runner in self.runners():
            runner._apply_event(z_event())
            runner._send.side_effect = BedProbeError('pipe disconnected')
            runner._schedule_failure_stop = Mock()
            with self.assertRaises(BedProbeError): await runner.action('step', 'a'*32, .1)
            runner._schedule_failure_stop.assert_called_once_with('pipe disconnected')
            self.assertIsNone(runner.status()['action_token'])

    async def test_calibration_record_and_save_cannot_run_from_positioning(self):
        runner = list(self.runners())[1]
        runner._apply_event(z_event())
        for command in ('record', 'finish'):
            with self.assertRaises(BedProbeError): await runner.action(command, 'a'*32)
        with self.assertRaises(BedProbeError): await runner.action('step', 'a'*32, .1, grams=100.)
        runner._apply_event(dict(state='adjusting_load', action_token='b'*32, unloaded=True, calibration_recorded=True))
        self.assertIsNone(runner.status()['z_controls'])
        with self.assertRaises(BedProbeError): await runner.action('continue', 'b'*32)
        await runner.action('finish', 'b'*32)
        runner._send.assert_awaited_once_with(dict(command='finish', token='b'*32))
