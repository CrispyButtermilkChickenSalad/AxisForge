"""Scale calibration uses the same watchdog, stop handling and ownership as probing."""
import asyncio
import math
import time

from bridge_dashboard.bed_probe import BedProbeRunner, BedProbeError, TERMINAL, Z_STATES, Z_WAITING


class LoadCalibrationRunner(BedProbeRunner):
    executable = 'alps_load_calibration'
    routine_name = 'ALPS scale calibration'
    simulation_message = 'Calibration simulation complete. No hardware or Klipper configuration was changed.'
    waiting = {f'waiting_{s}':s for s in ('home', 'scale', 'tare')}
    states = set(waiting) | TERMINAL | Z_STATES | {'starting', 'checking_robot_limits', 'waiting_temperature', 'finding_scale', 'retracting',
                                      'adjusting_load', 'stepping', 'step_rejected', 'load_limit',
                                      'recording_load', 'accepting_calibration', 'ready_to_save'}
    terminal_states = TERMINAL | {'ready_to_save', 'saving_config', 'verifying_config', 'save_failed'}
    success_states = {'complete', 'ready_to_save'}
    action_states = Z_WAITING | {'adjusting_load'}

    def __init__(self, *args, save_calibration=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._snapshot.update(message='Tare unloaded, record the scale reading, then release the load and save to Klipper.', total_points=0)
        self._action_received_at = 0.
        self._launch_options_lock = asyncio.Lock()
        self._calibration_max_counts = 120000.
        self._calibration_hot_target = None
        self._saving_config = False
        self._clearing_session = False
        self._save_calibration = save_calibration or self._save_native_configuration

    def busy(self):
        return super().busy() or getattr(self, '_clearing_session', False)

    async def start(self, *, dry_run=False, calibration_max_counts=120000., calibration_hot_target=None):
        async with self._launch_options_lock:
            if self.busy():
                raise BedProbeError('A scale routine is already active or its stop is unconfirmed')
            if (type(calibration_max_counts) not in (int, float) or
                    not math.isfinite(calibration_max_counts) or not 90000 <= calibration_max_counts <= 250000):
                raise BedProbeError('Calibration raw limit must be 90000..250000 counts')
            if calibration_hot_target is not None and (
                    type(calibration_hot_target) not in (int, float) or
                    not math.isfinite(calibration_hot_target) or not 50 <= calibration_hot_target <= 300):
                raise BedProbeError('Hot calibration target must be 50..300 C')
            self._calibration_max_counts = float(calibration_max_counts)
            self._calibration_hot_target = None if calibration_hot_target is None else float(calibration_hot_target)
            return await super().start(dry_run=dry_run)

    async def clear_session(self):
        """Explicit operator action; never clear another helper automatically."""
        async with self._launch_options_lock:
            if self.busy():
                raise BedProbeError('Finish or stop the calibration routine first')
            self._clearing_session = True
            try:
                from print_control.klipper_load_calibration import KlipperLoadCalibration
                client = KlipperLoadCalibration(self.settings.moonraker_url,
                                               api_key=self.settings.moonraker_api_key)
                operation = asyncio.create_task(asyncio.to_thread(client.cancel_existing))
                try: await asyncio.shield(operation)
                except asyncio.CancelledError:
                    await operation
                    raise
                self._snapshot.update(state='idle', action_token=None, z_controls=None, calibration_recorded=False,
                    message='No unfinished Klipper load-cell calibration remains. Saved calibration values were kept.')
                self.invalidate_preflight('Unfinished load-cell calibration cleared; repeat preflight')
            except Exception as error:
                raise BedProbeError(str(error)) from error
            finally:
                self._clearing_session = False
            return self.status()

    def _command(self, dry_run):
        command = super()._command(dry_run)
        command += ['--calibration-max-counts', str(self._calibration_max_counts)]
        if self._calibration_hot_target is not None:
            command += ['--calibration-hot-target', str(self._calibration_hot_target)]
        return command

    def status(self):
        return dict(super().status(), calibration_max_counts=self._calibration_max_counts,
                    calibration_hot_target=self._calibration_hot_target)

    def _apply_event(self, event):
        super()._apply_event(event)
        for key in ('contact_z_mm','lower_z_mm','upper_z_mm','raw_counts','delta_counts',
                    'temperature_c', 'target_c', 'stable_for_s', 'required_stable_s',
                    'recorded_grams', 'calibration_counts_per_gram', 'calibration_tare_counts'):
            if key in event:
                value = event[key]
                if key == 'recorded_grams' and value is None:
                    self._snapshot[key] = None
                    continue
                if type(value) not in (int,float) or not math.isfinite(value):
                    raise BedProbeError('Invalid scale calibration feedback')
                self._snapshot[key] = value
        for key in ('hot_calibration', 'load_limit_reached', 'upward_only', 'unloaded', 'calibration_recorded'):
            if key in event:
                if type(event[key]) is not bool:
                    raise BedProbeError('Invalid scale calibration safety state')
                self._snapshot[key] = event[key]
        if event['state'] == 'ready_to_save':
            if event.get('unloaded') is not True or event.get('calibration_recorded') is not True:
                raise BedProbeError('Calibration save handoff requires a recorded measurement and released load')
            values = event.get('calibration_values')
            if not isinstance(values, dict) or values.get('accepted') is not True or values.get('saved') is not False:
                raise BedProbeError('Calibration handoff is missing its accepted, unsaved values')
            for key in ('counts_per_gram', 'reference_tare_counts', 'grams'):
                value = values.get(key)
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise BedProbeError('Invalid accepted calibration value: '+key)
            if values['counts_per_gram'] < 1 or not 50 <= values['grams'] <= 25000:
                raise BedProbeError('Accepted calibration is outside Klipper bounds')
            self._snapshot['calibration_values'] = dict(values)

    async def action(self, command, token, delta_mm=None, grams=None):
        async with self._lock:
            if self._snapshot['state'] in Z_STATES or command in ('continue', 'finish_recovery'):
                if grams is not None:
                    raise BedProbeError('Z positioning and recovery do not accept a scale reading')
                return await self._z_action_locked(command, token, delta_mm)
            if (not self._active or self._aborting or self._stop_unconfirmed or self._fatal_error or
                    self._snapshot['state'] != 'adjusting_load' or
                    token != self._snapshot.get('action_token') or token in self._consumed_tokens or
                    time.monotonic()-self._action_received_at >= 9.):
                raise BedProbeError('This step is stale or motion is busy; refresh calibration status')
            if command not in ('step','record','finish'):
                raise BedProbeError('Unknown calibration action')
            if command == 'step' and (type(delta_mm) not in (int,float) or abs(delta_mm) not in (.001,.01,.05,.1,.5,1.,10.)):
                raise BedProbeError('Choose a supported baby-step size')
            if command != 'step' and delta_mm is not None:
                raise BedProbeError('Only a step accepts a movement distance')
            if command == 'record' and (type(grams) not in (int, float) or not math.isfinite(grams) or not 50 <= grams <= 25000):
                raise BedProbeError('Enter the measured scale reading: 50..25000 grams')
            if command != 'record' and grams is not None:
                raise BedProbeError('Only recording accepts a scale reading')
            if command == 'record' and (self._snapshot.get('calibration_recorded') is True or
                                         self._snapshot.get('unloaded') is not False):
                raise BedProbeError('Apply a stable measured load before recording; only one load is recorded per calibration')
            if command == 'finish' and (self._snapshot.get('calibration_recorded') is not True or
                                         self._snapshot.get('unloaded') is not True):
                raise BedProbeError('Record a measured load, then release the scale before saving')
            if (self._snapshot.get('upward_only') or self._snapshot.get('load_limit_reached')) and (
                    command != 'step' or not 0 < delta_mm <= .1):
                raise BedProbeError('Calibration load limit reached; use an upward release of at most 0.1 mm')
            self._consumed_tokens.add(token)
            self._snapshot['action_token'] = None
            try:
                values = {'delta_mm': delta_mm} if command == 'step' else {'grams': grams} if command == 'record' else {}
                await self._send(dict(command=command, token=token, **values))
            except BedProbeError as error:
                self._schedule_failure_stop(str(error))
                raise
            return self.status()

    async def _save_native_configuration(self, values, report):
        # The native package is required only for the ROS-backed calibration
        # feature, not for using the bridge's standalone printer dashboard.
        from print_control.klipper_load_calibration import KlipperLoadCalibration
        loop = asyncio.get_running_loop()
        client = KlipperLoadCalibration(self.settings.moonraker_url,
            api_key=self.settings.moonraker_api_key,
            on_state=lambda state: loop.call_soon_threadsafe(report, state))
        return await asyncio.to_thread(client.verify_and_save, values)

    async def _after_successful_exit(self):
        if self._snapshot['state'] != 'ready_to_save':
            if not self._snapshot['dry_run']:
                self._snapshot.update(state='save_failed', message='Robot routine exited without a verified calibration handoff; configuration was not saved.')
            return
        if self._snapshot['dry_run']:
            self._snapshot.update(state='complete', message=self.simulation_message, progress=1.)
            return
        if self._process.returncode != 0 or self._stop_unconfirmed or self._aborting:
            raise BedProbeError('Robot termination must be confirmed before SAVE_CONFIG')
        self._saving_config = True
        self._snapshot.update(state='saving_config', action_token=None, progress=.95,
            message='Robot routine stopped with load released. Saving calibration to Klipper; heaters will switch off and Klipper will restart.')
        def report(state):
            if self._saving_config and state in ('saving_config', 'verifying_config'):
                self._snapshot.update(state=state, message=(
                    'Waiting for Klipper to restart and verifying the saved calibration.' if state == 'verifying_config'
                    else 'Saving the accepted calibration to Klipper.'))
        try:
            result = await self._save_calibration(dict(self._snapshot['calibration_values']), report)
            if not isinstance(result, dict) or result.get('saved') is not True:
                raise BedProbeError('Klipper calibration persistence was not confirmed')
            self._snapshot.update(state='complete', progress=1., result=result,
                message='Calibration saved and verified in Klipper. ALPS Logger can read counts_per_gram and reference_tare_counts from the Klipper configuration.')
        except Exception as error:
            self._snapshot.update(state='save_failed', progress=.95,
                message='Calibration save was not verified: '+str(error)+'. Do not assume the configuration was saved; check Klipper before retrying.')
        finally:
            self._saving_config = False

    async def abort(self, reason='Operator stopped scale calibration', *, _expected_process=None):
        if self._saving_config:
            # SAVE_CONFIG is not safely reversible or replayable. The robot has
            # exited; keep verification alive and never report an invented undo.
            if _expected_process is None or self._process is _expected_process:
                self._snapshot['message'] = 'The robot is stopped. Klipper save/restart is already underway; waiting for verification.'
            return self.status()
        return await super().abort(reason, _expected_process=_expected_process)

    async def close(self):
        if self._saving_config and self._monitor is not None:
            await asyncio.shield(self._monitor)
        await super().close()
