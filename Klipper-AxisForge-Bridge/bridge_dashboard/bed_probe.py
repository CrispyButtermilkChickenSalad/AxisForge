"""Supervise the native ROS homing/mesh routine and its explicit confirmations."""
import asyncio
from collections import deque
import contextlib
import json
import math
import os
import signal


EVENT_PREFIX = 'AXISFORGE_PROBE '
WAITING = {'waiting_surface': 'surface', 'waiting_front_left': 'front_left'}
TERMINAL = {'complete', 'failed', 'aborted'}
STATES = set(WAITING) | TERMINAL | {
    'starting', 'homing_fast', 'homing_slow', 'moving_front_left', 'meshing', 'returning',
}


class BedProbeError(RuntimeError):
    """The probe could not start, accept a confirmation, or confirm its stop."""


class BedProbeRunner:
    states, waiting = STATES, WAITING
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
        self._active = self._aborting = self._stop_unconfirmed = False
        self._fatal_error = None
        self._program_stop_confirmed = False
        self._consumed_tokens = set()
        self._logs = deque(maxlen=12)
        self._snapshot = {'state': 'idle', 'message': 'Home and probe when the build surface is placed or moved.',
                          'progress': 0., 'current_point': 0, 'total_points': 0,
                          'confirmation': None, 'dry_run': False}

    def busy(self):
        return self._active or self._stop_unconfirmed

    def status(self):
        return dict(self._snapshot, busy=self.busy(), stop_unconfirmed=self._stop_unconfirmed,
                    configured=self.settings.axisforge_config is not None,
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
        return command

    async def start(self, *, dry_run=False):
        async with self._lock:
            if self.busy(): raise BedProbeError('A bed-probe routine is already active or its stop is unconfirmed')
            command = self._command(dry_run)
            self.invalidate_preflight(self.routine_name+' started; run job preflight again afterward')
            self._active, self._aborting, self._fatal_error = True, False, None
            self._program_stop_confirmed = False
            self._consumed_tokens.clear(); self._logs.clear()
            self._snapshot = {'state': 'starting', 'message': 'Checking the sensor and robot before confirmation.',
                              'progress': 0., 'current_point': 0, 'total_points': 0,
                              'confirmation': None, 'dry_run': dry_run}
            env = dict(os.environ, PYTHONUNBUFFERED='1', MOONRAKER_URL=self.settings.moonraker_url,
                       OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
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
        allowed = ('state', 'message', 'current_point', 'total_points', 'position_mm',
                   'homed_z_mm', 'clearance_z_mm', 'trigger_mode', 'result',
                   'trigger_counts', 'max_counts', 'sensor_noise_counts',
                   'trigger_value', 'trigger_unit', 'max_value', 'first_point')
        self._snapshot.update({key: event[key] for key in allowed if key in event})
        self._snapshot.update(progress=progress, confirmation=None if self._aborting else confirmation)
        if self._aborting and state not in TERMINAL:
            self._snapshot.update(state='stopping', message='Stopping the bed-probe routine.')

    def _schedule_failure_stop(self, message):
        self._fatal_error = self._fatal_error or message
        if self._failure_stop is None or self._failure_stop.done():
            async def stop():
                with contextlib.suppress(BedProbeError): await self.abort(message)
            self._failure_stop = asyncio.create_task(stop())

    async def _heartbeat(self):
        try:
            while self._active:
                await self._send({'command': 'heartbeat'})
                await asyncio.sleep(self.heartbeat_s)
        except asyncio.CancelledError:
            raise
        except BedProbeError as error:
            if self._process.returncode is None and self._snapshot['state'] not in TERMINAL:
                self._schedule_failure_stop(str(error))

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
            terminal = state in ('failed', 'aborted') or (state == 'complete' and code == 0)
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
        except (OSError, ValueError) as error:
            self._schedule_failure_stop('Lost bed-probe status: '+str(error))
            # Keep ownership until the process actually exits, even if stdout failed.
            await self._process.wait()
            await self._confirm_unexpected_exit_stop()
            if not self._stop_unconfirmed:
                self._snapshot.update(state='failed', message=self._fatal_error)
        finally:
            self._snapshot['confirmation'] = None
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

    async def abort(self, reason='Operator stopped bed probing'):
        async with self._abort_lock:
            # Start owns this lock through process creation, so abort can never
            # observe a running launch without its monitor/command pipe installed.
            async with self._lock:
                if self._stop_unconfirmed: raise BedProbeError(self._snapshot['message'])
                if not self._active: return self.status()
                self._aborting = True
                self._snapshot.update(state='stopping', message=reason, confirmation=None)
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
