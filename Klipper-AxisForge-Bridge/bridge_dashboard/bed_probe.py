"""Supervise the native ROS homing/mesh routine and its explicit confirmations."""
import asyncio
from collections import deque
import contextlib
import json
import math
import os
import signal
import time
from bridge_dashboard.probe_options import SavedMesh, validate_thermal


EVENT_PREFIX = 'AXISFORGE_PROBE '
WAITING = {'waiting_surface': 'surface', 'waiting_front_left': 'front_left'}
TERMINAL = {'complete', 'failed', 'aborted'}
Z_WAITING = {'positioning_z', 'recovering_z'}
Z_STATES = Z_WAITING | {'z_stepping', 'z_step_rejected'}
Z_STEP_SIZES = (.001, .01, .05, .1, .5, 1., 10.)
Z_POSITION_TOLERANCE_MM = .1
STATES = set(WAITING) | TERMINAL | Z_STATES | {
    'starting', 'waiting_temperature', 'aligning_nozzle', 'checking_robot_limits', 'homing_fast', 'homing_slow', 'moving_front_left', 'meshing', 'returning', 'recovering_home',
}


class BedProbeError(RuntimeError):
    """The probe could not start, accept a confirmation, or confirm its stop."""


class BedProbeRunner:
    states, waiting = STATES, WAITING
    action_states = Z_WAITING
    terminal_states = TERMINAL
    success_states = {'complete'}
    executable = 'bridge_bed_probe'
    routine_name = 'Bed homing'
    simulation_message = 'Simulation complete. The active bed map was not changed.'
    def __init__(self, settings, *, robot_stop, invalidate_preflight,
                 spawn=asyncio.create_subprocess_exec, stop_timeout_s=8., heartbeat_s=1.):
        self.settings = settings
        self.robot_stop, self.invalidate_preflight = robot_stop, invalidate_preflight
        self.spawn, self.stop_timeout_s, self.heartbeat_s = spawn, stop_timeout_s, heartbeat_s
        self._lock, self._write_lock, self._abort_lock = asyncio.Lock(), asyncio.Lock(), asyncio.Lock()
        self._process = self._monitor = self._heartbeat_task = self._failure_stop = None
        self._failure_stop_process = None
        self._active = self._aborting = self._stop_unconfirmed = False
        self._fatal_error = None
        self._program_stop_confirmed = False
        self._consumed_tokens = set()
        self._action_received_at = 0.
        self._logs = deque(maxlen=12)
        self._last_failure = None
        self._probe_thermal = None
        self._saved_mesh = SavedMesh(settings.axisforge_config)
        self._failure_history = deque(maxlen=5)
        self._snapshot = {'state': 'idle', 'message': 'Home and probe when the build surface is placed or moved.',
                          'progress': 0., 'current_point': 0, 'total_points': 0,
                          'confirmation': None, 'action_token': None, 'z_controls': None, 'dry_run': False}

    def busy(self):
        return self._active or self._stop_unconfirmed

    def status(self):
        return dict(self._snapshot, busy=self.busy(), stop_unconfirmed=self._stop_unconfirmed,
                    last_failure=self._last_failure,
                    failure_history=list(self._failure_history),
                    configured=self.settings.axisforge_config is not None,
                    saved_mesh=self._saved_mesh.status(), thermal_options=self._probe_thermal,
                    return_code=None if self._process is None else self._process.returncode,
                    logs=list(self._logs))

    def _command(self, dry_run):
        if self.settings.axisforge_config is None:
            raise BedProbeError('AXISFORGE_CONFIG is required for bed homing and probing')
        if not self.settings.axisforge_config.is_file():
            raise BedProbeError('The configured AxisForge calibration file does not exist')
        command = [self.settings.ros2_command, 'run', 'print_control', self.executable,
                   '--config', str(self.settings.axisforge_config),
                   '--moonraker-url', self.settings.moonraker_url]
        if dry_run: command.append('--dry-run')
        if self._probe_thermal is not None:
            for key,arg in (('nozzle_target_c','--probe-nozzle-target'),('bed_target_c','--probe-bed-target'),('soak_s','--probe-soak-s')):
                command += [arg,str(self._probe_thermal[key])]
        return command

    async def start(self, *, dry_run=False, thermal=None):
        async with self._lock:
            if self.busy(): raise BedProbeError('A bed-probe routine is already active or its stop is unconfirmed')
            try:self._probe_thermal=validate_thermal(thermal)
            except ValueError as error:raise BedProbeError(str(error)) from error
            command = self._command(dry_run)
            self.invalidate_preflight(self.routine_name+' started; run job preflight again afterward')
            self._active, self._aborting, self._fatal_error = True, False, None
            self._program_stop_confirmed = False
            self._consumed_tokens.clear(); self._logs.clear()
            self._snapshot = {'state': 'starting', 'message': 'Checking the sensor and robot before confirmation.',
                              'progress': 0., 'current_point': 0, 'total_points': 0,
                              'confirmation': None, 'action_token': None, 'z_controls': None, 'dry_run': dry_run}
            self._action_received_at = 0.
            env = dict(os.environ, PYTHONUNBUFFERED='1', MOONRAKER_URL=self.settings.moonraker_url,
                       OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
            # Keep the validated transport alongside this robot's configuration.
            # Apply only to probe/calibration subprocesses, and respect an
            # explicitly supplied DDS profile on installations that have one.
            dds_profile = self.settings.axisforge_config.with_name('probe_dds.xml')
            if dds_profile.is_file():
                env.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(dds_profile))
            if self.settings.moonraker_api_key: env['MOONRAKER_API_KEY'] = self.settings.moonraker_api_key
            else: env.pop('MOONRAKER_API_KEY', None)
            launch = asyncio.create_task(self.spawn(*command, stdin=asyncio.subprocess.PIPE,
                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
                    env=env, start_new_session=True, limit=262144))
            try:
                self._process = await asyncio.shield(launch)
            except asyncio.CancelledError:
                # An HTTP disconnect must not orphan a process created during spawn.
                try: self._process = await launch
                except (OSError, ValueError):
                    self._active = False
                    self._snapshot.update(state='failed', message='Bed-probe launch was interrupted')
                else:
                    self._heartbeat_task = asyncio.create_task(self._heartbeat())
                    self._monitor = asyncio.create_task(self._watch())
                    self._schedule_failure_stop('Bed-probe launch request was interrupted')
                raise
            except (OSError, ValueError) as error:
                self._active = False
                self._snapshot.update(state='failed', message='Could not launch bed probing: '+str(error))
                raise BedProbeError(self._snapshot['message']) from error
            self._heartbeat_task = asyncio.create_task(self._heartbeat())
            self._monitor = asyncio.create_task(self._watch())
            return self.status()

    async def _send(self, command):
        async with self._write_lock:
            process = self._process
            if process is None or process.returncode is not None or process.stdin is None:
                raise BedProbeError('Bed-probe process is no longer accepting commands')
            try:
                process.stdin.write((json.dumps(command, allow_nan=False)+'\n').encode())
                await asyncio.wait_for(process.stdin.drain(), 1.)
            except (OSError, RuntimeError, asyncio.TimeoutError) as error:
                raise BedProbeError('Could not communicate with the bed-probe routine') from error

    async def confirm(self, stage, token):
        async with self._lock:
            expected = self._snapshot.get('confirmation')
            if (not self._active or self._aborting or not expected or
                    expected != {'stage': stage, 'token': token} or token in self._consumed_tokens):
                raise BedProbeError('This confirmation is stale or no longer requested; refresh the probe status')
            # Consume before awaiting I/O so double-clicks cannot queue another move.
            self._consumed_tokens.add(token)
            self._snapshot['confirmation'] = None
            try:
                await self._send({'command': 'confirm', 'stage': stage, 'token': token})
            except BedProbeError as error:
                self._schedule_failure_stop(str(error))
                raise
            self._snapshot['message'] = 'Confirmation sent; waiting for the robot routine.'
            return self.status()

    def _remember_failure(self, message):
        if isinstance(message, str) and message:
            self._last_failure = message
            if message not in self._failure_history:
                self._failure_history.append(message)

    def _apply_event(self, event):
        if not isinstance(event, dict) or event.get('state') not in self.states:
            raise BedProbeError('Invalid status from the bed-probe routine')
        state = event['state']
        if event.get('stop_unconfirmed') is True:
            self._stop_unconfirmed = True
        progress = event.get('progress', self._snapshot['progress'])
        if type(progress) not in (int, float) or not math.isfinite(progress) or not 0 <= progress <= 1:
            raise BedProbeError('Invalid bed-probe progress')
        confirmation = event.get('confirmation')
        if confirmation is not None:
            if (not isinstance(confirmation, dict) or set(confirmation) != {'stage', 'token'} or
                    self.waiting.get(state) != confirmation['stage'] or
                    not isinstance(confirmation['token'], str) or not 16 <= len(confirmation['token']) <= 128):
                raise BedProbeError('Invalid bed-probe confirmation request')
            if confirmation['token'] in self._consumed_tokens: confirmation = None
        if state in self.waiting and confirmation is None and not self._aborting:
            # A consumed token can appear in an in-flight status line; never restore it.
            confirmation = None
        controls = self._validate_z_controls(event.get('z_controls'), state)
        recovery = event.get('failure_recovery')
        if recovery is not None:
            if (not isinstance(recovery, dict) or recovery.get('state') not in
                    ('checking', 'releasing', 'returning', 'returned', 'blocked') or
                    not isinstance(recovery.get('failure'), str)):
                raise BedProbeError('Invalid Home recovery feedback')
        failure = controls.get('failure') if controls else None
        if state == 'recovering_home' and recovery:
            failure = recovery['failure']
        if state == 'failed' or state == 'z_step_rejected':
            failure = event.get('message')
        self._remember_failure(failure)
        token = event.get('action_token') if state in self.action_states else None
        if token is not None and (not isinstance(token, str) or not 16 <= len(token) <= 128):
            raise BedProbeError('Invalid setup action token')
        if token in self._consumed_tokens: token = None
        if token is not None and token != self._snapshot.get('action_token'):
            self._action_received_at = time.monotonic()
        allowed = ('state', 'message', 'current_point', 'total_points', 'position_mm',
                   'homed_z_mm', 'clearance_z_mm', 'trigger_mode', 'trigger_source', 'trigger_force_g', 'result',
                   'trigger_counts', 'max_counts', 'sensor_noise_counts',
                   'baseline_noise_fraction', 'alignment', 'failure_recovery',
                   'trigger_value', 'trigger_unit', 'max_value', 'first_point', 'max_descent_mm',
                   'bed_center_base_mm', 'first_point_offset_mm', 'bed_yaw_deg')
        if 'thermal' in event:
            thermal=event['thermal']
            if not isinstance(thermal,dict) or thermal.get('mode') not in ('cold','heated'):
                raise BedProbeError('Invalid probing temperature feedback')
            self._snapshot['thermal']=thermal
        self._snapshot.update({key: event[key] for key in allowed if key in event})
        self._snapshot.update(progress=progress, confirmation=None if self._aborting else confirmation,
                              z_controls=None if self._aborting else controls,
                              action_token=None if self._aborting else token)
        if self._aborting and state not in self.terminal_states:
            self._snapshot.update(state='stopping', message='Stopping the bed-probe routine.')

    @staticmethod
    def _validate_z_controls(controls, state):
        if state not in Z_STATES:
            return None
        if controls is None and state not in Z_WAITING:
            return None
        required = {'phase', 'position_mm', 'lower_z_mm', 'upper_z_mm', 'delta_counts',
                    'upward_only', 'loaded', 'can_continue', 'can_finish', 'step_sizes'}
        if (not isinstance(controls, dict) or not required <= set(controls) or
                set(controls)-required-{'failure', 'release_blocked'}):
            raise BedProbeError('Invalid Z-control feedback')
        phase = controls['phase']
        if phase not in ('positioning', 'recovery') or (
                state in Z_WAITING and state != ('positioning_z' if phase == 'positioning' else 'recovering_z')):
            raise BedProbeError('Invalid Z-control phase')
        position = controls['position_mm']
        if (not isinstance(position, list) or len(position) != 3 or
                any(type(value) not in (int, float) or not math.isfinite(value) for value in position)):
            raise BedProbeError('Invalid Z-control position')
        for key in ('lower_z_mm', 'upper_z_mm', 'delta_counts'):
            if type(controls[key]) not in (int, float) or not math.isfinite(controls[key]):
                raise BedProbeError('Invalid Z-control limit or load')
        # Measured feedback can sit just beyond a requested endpoint because of
        # tracking error or FK noise. Keep inward correction available; requested
        # step endpoints below still have to obey the original strict limits.
        if (controls['lower_z_mm'] > controls['upper_z_mm'] or
                not controls['lower_z_mm']-Z_POSITION_TOLERANCE_MM <= position[2] <=
                    controls['upper_z_mm']+Z_POSITION_TOLERANCE_MM):
            raise BedProbeError('Z-control position is outside its travel limits')
        for key in ('upward_only', 'loaded', 'can_continue', 'can_finish'):
            if type(controls[key]) is not bool:
                raise BedProbeError('Invalid Z-control safety state')
        if ('release_blocked' in controls and type(controls['release_blocked']) is not bool or
                controls.get('release_blocked') and (controls['can_continue'] or controls['can_finish'])):
            raise BedProbeError('Invalid Z release guard state')
        if (phase == 'recovery' and (not controls['upward_only'] or controls['can_continue']) or
                phase == 'positioning' and controls['can_finish'] or
                controls['loaded'] and (controls['can_continue'] or controls['can_finish'])):
            raise BedProbeError('Inconsistent Z-control safety state')
        sizes = controls['step_sizes']
        if (not isinstance(sizes, list) or not sizes or
                any(type(value) not in (int, float) or value not in Z_STEP_SIZES for value in sizes) or
                len(set(sizes)) != len(sizes)):
            raise BedProbeError('Invalid Z-control step sizes')
        if 'failure' in controls and not isinstance(controls['failure'], str):
            raise BedProbeError('Invalid Z recovery failure message')
        return {**controls, 'position_mm': list(position), 'step_sizes': list(sizes)}

    async def action(self, command, token, delta_mm=None):
        async with self._lock:
            return await self._z_action_locked(command, token, delta_mm)

    async def _z_action_locked(self, command, token, delta_mm=None):
        controls = self._snapshot.get('z_controls')
        if (not self._active or self._aborting or self._stop_unconfirmed or self._fatal_error or
                self._snapshot['state'] not in Z_WAITING or not controls or
                token is None or token != self._snapshot.get('action_token') or
                token in self._consumed_tokens or time.monotonic()-self._action_received_at >= 9.):
            raise BedProbeError('This Z action is stale or motion is busy; refresh setup status')
        if command not in ('step', 'continue', 'finish_recovery'):
            raise BedProbeError('Unknown Z-control action')
        if command == 'step':
            if controls.get('release_blocked'):
                raise BedProbeError('Load rose during upward release; wait for the original release guard to clear')
            if (type(delta_mm) not in (int, float) or not math.isfinite(delta_mm) or
                    abs(delta_mm) not in controls['step_sizes']):
                raise BedProbeError('Choose a supported Z step size')
            if controls['upward_only'] and delta_mm < 0:
                raise BedProbeError('Only upward Z release is available')
            if controls['loaded'] and not 0 < delta_mm <= .1:
                raise BedProbeError('Load is applied; use an upward release of at most 0.1 mm')
            end = controls['position_mm'][2]+delta_mm
            if not controls['lower_z_mm']-1e-8 <= end <= controls['upper_z_mm']+1e-8:
                raise BedProbeError('This Z step exceeds the allowed travel range')
        elif delta_mm is not None:
            raise BedProbeError('Only a step accepts a movement distance')
        elif command == 'continue' and (controls['phase'] != 'positioning' or
                                       not controls['can_continue'] or controls['loaded']):
            raise BedProbeError('Release the load before continuing the automatic search')
        elif command == 'finish_recovery' and (controls['phase'] != 'recovery' or
                                              not controls['can_finish'] or controls['loaded']):
            raise BedProbeError('Release the load before finishing recovery')
        self._consumed_tokens.add(token)
        self._snapshot['action_token'] = None
        try:
            await self._send(dict(command=command, token=token,
                                 **({'delta_mm': delta_mm} if command == 'step' else {})))
        except BedProbeError as error:
            self._schedule_failure_stop(str(error))
            raise
        return self.status()

    def _schedule_failure_stop(self, message):
        self._fatal_error = self._fatal_error or message
        process = self._process
        if (self._failure_stop is None or self._failure_stop.done()
                or self._failure_stop_process is not process):
            async def stop():
                with contextlib.suppress(BedProbeError):
                    await self.abort(message, _expected_process=process)
            self._failure_stop_process = process
            self._failure_stop = asyncio.create_task(stop())

    async def _heartbeat(self):
        try:
            while self._active:
                await self._send({'command': 'heartbeat'})
                await asyncio.sleep(self.heartbeat_s)
        except asyncio.CancelledError:
            raise
        except BedProbeError as error:
            if self._process.returncode is None and self._snapshot['state'] not in self.terminal_states:
                self._schedule_failure_stop(str(error))

    async def _after_successful_exit(self):
        """Optional post-processing while dashboard ownership is still held."""

    async def _watch(self):
        try:
            while True:
                line = await self._process.stdout.readline()
                if not line: break
                text = line.decode(errors='replace').rstrip()
                if text.startswith(EVENT_PREFIX):
                    try: self._apply_event(json.loads(text[len(EVENT_PREFIX):]))
                    except (ValueError, BedProbeError) as error:
                        self._schedule_failure_stop(str(error))
                elif text: self._logs.append(text[-2000:])
            code = await self._process.wait()
            state = self._snapshot['state']
            terminal = state in ('failed', 'aborted') or (state in self.success_states and code == 0)
            if not terminal:
                await self._confirm_unexpected_exit_stop()
            if self._stop_unconfirmed:
                self._snapshot['state'] = 'failed'
            elif self._fatal_error:
                self._snapshot.update(state='failed', message=self._fatal_error)
            elif state in ('failed', 'aborted'):
                # The structured child result carries the actual sensor/motion
                # failure; ros2run normally exits 1 afterward. Keep that reason.
                pass
            elif self._aborting:
                self._snapshot.update(state='aborted', message='Bed probing stopped; run preflight again before printing.')
            elif not terminal:
                detail = self._logs[-1] if self._logs else 'No completion result was received'
                self._snapshot.update(state='failed', message=f'Bed probing exited ({code}): {detail}')
            elif state == 'complete' and self._snapshot['dry_run']:
                self._snapshot['message'] = self.simulation_message
            if (state in self.success_states and code == 0 and not self._stop_unconfirmed
                    and not self._fatal_error and not self._aborting):
                if self._heartbeat_task is not None:
                    self._heartbeat_task.cancel()
                    with contextlib.suppress(asyncio.CancelledError): await self._heartbeat_task
                if self._stop_unconfirmed or self._fatal_error:
                    self._snapshot.update(state='failed', message=self._fatal_error or 'Robot stop was not confirmed')
                elif self._aborting:
                    self._snapshot.update(state='aborted', message='Routine stopped before completing.')
                else:
                    await self._after_successful_exit()
        except (OSError, ValueError) as error:
            self._schedule_failure_stop('Lost bed-probe status: '+str(error))
            # Keep ownership until the process actually exits, even if stdout failed.
            await self._process.wait()
            await self._confirm_unexpected_exit_stop()
            if not self._stop_unconfirmed:
                self._snapshot.update(state='failed', message=self._fatal_error)
        finally:
            if self._snapshot['state'] == 'failed':
                self._remember_failure(self._snapshot['message'])
            self._snapshot.update(confirmation=None, action_token=None, z_controls=None)
            if self._heartbeat_task is not None:
                self._heartbeat_task.cancel()
                with contextlib.suppress(asyncio.CancelledError): await self._heartbeat_task
            self.invalidate_preflight(self.routine_name+' finished; run job preflight again before printing')
            self._active = False

    async def _stop_robot_program(self):
        if not self._snapshot['dry_run'] and not self._program_stop_confirmed:
            await self.robot_stop()
            self._program_stop_confirmed = True

    async def _confirm_unexpected_exit_stop(self):
        # A dead Python/ROS process does not prove the controller's queued goal
        # stopped. Require the independent UR stop path before releasing jobs.
        try:
            await self._stop_robot_program()
        except Exception as error:
            self._stop_unconfirmed = True
            self._snapshot.update(state='failed',
                message='Bed-probe process exited without confirmed robot termination; use pendant stop. '+str(error))

    async def abort(self, reason='Operator stopped bed probing', *, _expected_process=None):
        async with self._abort_lock:
            # Start owns this lock through process creation, so abort can never
            # observe a running launch without its monitor/command pipe installed.
            async with self._lock:
                # A delayed internal stop belongs to the child that failed. A
                # retry may have started while this task waited for an earlier
                # abort to finish; never apply its stop to the new child.
                if _expected_process is not None and self._process is not _expected_process:
                    return self.status()
                if self._stop_unconfirmed: raise BedProbeError(self._snapshot['message'])
                if not self._active: return self.status()
                self._aborting = True
                self._snapshot.update(state='stopping', message=reason, confirmation=None,
                                      action_token=None, z_controls=None)
            with contextlib.suppress(BedProbeError): await self._send({'command': 'abort'})
            try:
                await asyncio.wait_for(asyncio.shield(self._monitor), self.stop_timeout_s)
            except asyncio.TimeoutError:
                try:
                    await self._stop_robot_program()
                except Exception as error:
                    self._stop_unconfirmed = True
                    self._snapshot.update(state='failed', message='Robot stop unconfirmed; use the pendant stop. '+str(error))
                    # Closing input also trips the child's independent EOF watchdog.
                    if self._process.stdin is not None: self._process.stdin.close()
                    raise BedProbeError(self._snapshot['message']) from error
                # Only terminate the process after a live UR stop has been acknowledged.
                with contextlib.suppress(ProcessLookupError): os.killpg(self._process.pid, signal.SIGTERM)
                try:
                    await asyncio.wait_for(asyncio.shield(self._monitor), 4.)
                except asyncio.TimeoutError:
                    with contextlib.suppress(ProcessLookupError): os.killpg(self._process.pid, signal.SIGKILL)
                    try: await asyncio.wait_for(asyncio.shield(self._monitor), 2.)
                    except asyncio.TimeoutError as error:
                        self._stop_unconfirmed = True
                        self._snapshot.update(state='failed', message='Bed-probe process termination is unconfirmed')
                        raise BedProbeError(self._snapshot['message']) from error
            return self.status()

    async def close(self):
        try: await self.abort('Dashboard shutting down')
        finally:
            if self._process is not None and self._process.stdin is not None:
                self._process.stdin.close()
            if self._heartbeat_task is not None: self._heartbeat_task.cancel()
