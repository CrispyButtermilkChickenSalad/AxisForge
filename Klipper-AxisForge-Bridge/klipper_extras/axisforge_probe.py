"""Leased, no-motion access to Klipper's native load_cell_probe MCU trigger.

The adapter deliberately requires the trigger_analog API used by Klipper
29d4ec2d6. It never changes calibration or issues a printer motion command.
An event requests an external robot stop; it does not itself stop that robot.
"""

import math
import uuid


class AxisForgeProbe:
    PROTOCOL = 1
    ENDSTOP_HIT = 1
    HOST_REQUEST = 2
    COMMS_TIMEOUT = 4
    ARM_LEAD_S = .025
    SETUP_TIMEOUT_S = 2.
    WATCHDOG_S = .02
    CONFLICT_COMMANDS = (
        'G0', 'G1', 'G2', 'G3', 'G28', 'G92', 'M24', 'M104', 'M109',
        'M140', 'M190', 'SDCARD_PRINT_FILE', 'SET_HEATER_TEMPERATURE',
        'PROBE', 'PROBE_ACCURACY', 'PROBE_CALIBRATE', 'BED_MESH_CALIBRATE',
        'Z_TILT_ADJUST', 'QUAD_GANTRY_LEVEL', 'LOAD_CELL_TEST_TAP',
        'LOAD_CELL_TARE', 'LOAD_CELL_CALIBRATE', 'SET_KINEMATIC_POSITION',
        'MANUAL_STEPPER', 'FORCE_MOVE', 'STEPPER_BUZZ', 'SET_STEPPER_ENABLE',
        'SET_GCODE_OFFSET', 'RESTORE_GCODE_STATE', 'ACTIVATE_EXTRUDER',
        'SYNC_EXTRUDER_MOTION', 'SAVE_CONFIG',
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.sample_batch_interval_s = config.getfloat(
            'sample_batch_interval_s', .025, minval=.02, maxval=.1)
        self.active_sample_batch_interval_s = None
        self.probe = self.move = self.load_cell = self.trigger = None
        self.toolhead = self.mcu = None
        self.heaters = []
        self.heater_by_name = {}
        self._thermal_targets = {}
        self.nozzle_heater = None
        self.supported = False
        self.support_error = 'Klipper is not ready'
        self.session = None
        self.state, self.error = 'disarmed', ''
        self.generation = 0
        self.lease_s = .5
        self.deadline = self.search_deadline = 0.
        self.arm_time = self.trigger_time = None
        self.stop_required = False
        self.cleanup_confirmed = True
        self._native_started = self._finalizing = self._arming_worker = False
        self._native_completion = None
        self._changed = self.reactor.completion()
        self._cleanup_done = self.reactor.completion()
        self._terminal_intent = None
        self._used_sessions = []
        self._max_nozzle_temp = 0.
        hooks = self.printer.lookup_object('webhooks')
        for name in ('capabilities', 'arm', 'wait', 'disarm'):
            hooks.register_endpoint('axisforge_probe/' + name,
                                    getattr(self, 'api_' + name))
        self.printer.register_event_handler('klippy:ready', self._ready)
        self.printer.register_event_handler('klippy:shutdown', self._shutdown)
        self.printer.register_event_handler('gcode:request_restart', self._restart)
        self.reactor.register_timer(self._watchdog, self.reactor.NOW)

    def _ready(self):
        try:
            self.probe = self.printer.lookup_object('probe')
            self.move = self.probe._tapping_move._load_cell_probing_move
            self.load_cell = self.probe._load_cell
            self.trigger = self.probe._mcu_trigger_analog
            self.toolhead = self.printer.lookup_object('toolhead')
            self.mcu = self.trigger.get_mcu()
            for obj, names in (
                (self.move, ('_pause_and_tare',)),
                (self.load_cell, ('is_calibrated', 'get_counts_per_gram',
                                 'get_reference_tare_counts', 'get_tare_counts',
                                 'get_sensor')),
                (self.trigger, ('home_start', 'get_dispatch', '_clear_home',
                                'set_raw_range', 'get_steppers')),
                (self.trigger.get_dispatch(), ('stop',)),
                (self.move._config_helper, ('get_trigger_force_grams',)),
                (self.load_cell.get_sensor(), ('get_status',)),
            ):
                if any(not callable(getattr(obj, name, None)) for name in names):
                    raise ValueError('Native load_cell_probe API is incompatible')
            if self.trigger.get_steppers():
                raise ValueError('No-motion probing requires zero Klipper probe steppers')
            if self.trigger.REASON_TRIGGER_ANALOG != self.COMMS_TIMEOUT + 1:
                raise ValueError('Native trigger reason API is incompatible')
            if not callable(getattr(self.trigger._query_state_cmd, 'send', None)):
                raise ValueError('Native MCU trigger query is unavailable')
            settings = self.printer.lookup_object('configfile').get_status(
                self.reactor.monotonic())['settings']
            if settings.get('printer', {}).get('kinematics') != 'none':
                raise ValueError('No-motion probing requires kinematics: none')
            manager = self.printer.lookup_object('heaters')
            self.heater_by_name = {name.split()[-1]:manager.lookup_heater(name.split()[-1])
                                  for name in manager.get_all_heaters()}
            self.heaters = list(self.heater_by_name.values())
            self.nozzle_heater = manager.lookup_heater('extruder')
            self._configure_sample_batching()
            self.supported, self.support_error = True, ''
            self._guard_commands()
        except Exception as exc:
            self.supported, self.support_error = False, str(exc)

    def _configure_sample_batching(self):
        """Deliver existing ADC samples more often; leave MCU acquisition intact.

        ADS131M0X uses BatchBulkHelper to drain its already-running bulk queue.
        Its ADC sample rate, MCU query clock, calibration and trigger remain
        unchanged. Sensors without this helper keep their existing behavior.
        """
        sensor = self.load_cell.get_sensor()
        batch = getattr(sensor, 'batch_bulk', None)
        if batch is None:
            return
        current = getattr(batch, 'batch_interval', None)
        if (isinstance(current, bool) or not isinstance(current, (float, int)) or
                not math.isfinite(current) or current <= 0 or
                not callable(getattr(batch, '_proc_batch', None))):
            raise ValueError('Load-cell batch delivery API is incompatible')
        # Never slow a sensor that already delivers faster batches.
        batch.batch_interval = min(current, self.sample_batch_interval_s)
        self.active_sample_batch_interval_s = batch.batch_interval

    def _guard_commands(self):
        gcode = self.printer.lookup_object('gcode')
        for name in self.CONFLICT_COMMANDS:
            original = gcode.register_command(name, None)
            if original is None:
                continue
            def guarded(gcmd, handler=original):
                if self.is_busy():
                    raise gcmd.error('AxisForge probe owns the sensor; disarm first')
                return handler(gcmd)
            gcode.register_command(name, guarded)
        for name in ('CANCEL_PRINT', 'PAUSE', 'TURN_OFF_HEATERS'):
            original = gcode.register_command(name, None)
            if original is None:
                continue
            def stopping(gcmd, handler=original, command=name):
                if self.is_busy():
                    self._finish('fault', 'External ' + command)
                return handler(gcmd)
            gcode.register_command(name, stopping)

    def is_busy(self):
        return (self._arming_worker or self._finalizing or self._native_started
                or self.state in ('arming', 'armed', 'triggered'))

    def _notify(self):
        previous, self._changed = self._changed, self.reactor.completion()
        previous.complete(True)

    def _calibration(self):
        if not self.supported:
            return False, None, None
        cpg = self.load_cell.get_counts_per_gram()
        reference = self.load_cell.get_reference_tare_counts()
        force = self.move._config_helper.get_trigger_force_grams()
        valid = (self.load_cell.is_calibrated()
                 and all(isinstance(v, (int, float)) and math.isfinite(v)
                         for v in (cpg, reference, force))
                 and cpg > 0 and force > 0)
        return valid, cpg, force

    def api_capabilities(self, request):
        calibrated, cpg, force = self._calibration()
        request.send({'protocol': self.PROTOCOL, 'supported': self.supported,
                      'heated_probing': True,
                      'error': self.support_error,
                      'calibrated': calibrated, 'trigger_force_g': force,
                      'counts_per_gram': cpg, 'busy': self.is_busy(),
                      'sample_batch_interval_s': self.active_sample_batch_interval_s,
                      'max_lease_s': .5, 'max_wait_s': .1})

    @staticmethod
    def _number(request, name, default, lower, upper):
        value = request.get(name, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not lower <= value <= upper):
            raise request.error('%s must be between %s and %s' %
                                (name, lower, upper))
        return float(value)

    def _idle_error(self, now, check_motion=False):
        sync = self.printer.lookup_object('axisforge_sync', None)
        if sync is not None and (sync.leased or sync.preparing
                                 or sync.reservation is not None):
            return 'A synchronized print owns Klipper'
        sd = self.printer.lookup_object('virtual_sdcard', None)
        if sd is not None and sd.is_active():
            return 'A Klipper print is active'
        stats = self.printer.lookup_object('print_stats', None)
        if stats is not None and stats.get_status(now).get('state') in ('printing', 'paused'):
            return 'A Klipper print is active or paused'
        gcode = self.printer.lookup_object('gcode')
        if any(name in gcode.get_status(now)['commands'] for name in ('TARE', 'CALIBRATE')):
            return 'Interactive load-cell calibration is active; complete or abort it first'
        if check_motion and gcode.get_mutex().test():
            return 'A Klipper G-code command is still executing'
        idle = self.printer.lookup_object('idle_timeout', None)
        # Native tare calls get_last_move_time(), which itself changes
        # idle_timeout to Printing even though this adapter never queues motion.
        # Check this before ownership; command/print-lease guards protect it after.
        if check_motion and idle is not None and idle.get_status(now).get('state') == 'Printing':
            return 'Klipper is executing printer motion'
        available = dict(self.heater_by_name)
        available.setdefault('extruder',self.nozzle_heater)
        if any(name not in available for name in self._thermal_targets):
            return 'Selected probing heater is not configured'
        for heater in self.heaters:
            temperature, target = heater.get_temp(now)
            if not math.isfinite(temperature) or not math.isfinite(target):
                return 'Heater temperature is unavailable'
            name = next((key for key,value in available.items() if value is heater),None)
            expected = self._thermal_targets.get(name,0.)
            if target != expected:
                return 'Heater target does not match the selected probing temperatures'
            if expected and abs(temperature-expected)>2.:
                return 'Probing heater left its stable +/-2 C band'
        if 'extruder' not in self._thermal_targets and self.nozzle_heater.get_temp(now)[0] > self._max_nozzle_temp:
            return 'Nozzle is too hot for probing'
        return None

    def _sensor_error(self, now):
        try:
            status = self.load_cell.get_sensor().get_status(now)
        except Exception:
            return 'Native load-cell sensor health is unavailable'
        for name in ('errors', 'overflows'):
            value = status.get(name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value != 0:
                return 'Native load-cell sensor reports ' + name
        return None

    def _snapshot(self):
        return {'protocol': self.PROTOCOL, 'session': self.session,
                'state': self.state, 'error': self.error,
                'stop_required': self.stop_required,
                'cleanup_confirmed': self.cleanup_confirmed,
                'arm_time': self.arm_time, 'trigger_time': self.trigger_time,
                'lease_s': self.lease_s}

    def _check_deadlines(self, now):
        if self.state in ('arming', 'armed', 'triggered') or self._finalizing:
            if now >= self.deadline:
                reason = ('Probe arming timed out' if self.state == 'arming'
                          else 'Probe heartbeat lease expired')
                self._finish('fault', reason)
            elif self.state == 'armed' and self.search_deadline and now >= self.search_deadline:
                self._finish('fault', 'Probe search timed out')

    def _assert_arming(self, generation):
        self._check_deadlines(self.reactor.monotonic())
        if self.generation != generation or self.state != 'arming':
            raise self.printer.command_error(self.error or 'Probe arming cancelled')

    def api_arm(self, request):
        if not self.supported:
            raise request.error(self.support_error)
        if self.is_busy() or not self.cleanup_confirmed:
            raise request.error('Native probe is busy or cleanup is unconfirmed')
        session = request.get('session')
        try:
            if not isinstance(session, str) or str(uuid.UUID(session)) != session:
                raise ValueError()
        except (ValueError, AttributeError):
            raise request.error('A canonical session UUID is required')
        if session in self._used_sessions:
            raise request.error('Use a new UUID for each probe arm')
        limit = self._number(request, 'max_delta_counts', 120000., 1., 120000.)
        self._max_nozzle_temp = self._number(request, 'max_nozzle_temp_c', 50., 0., 100.)
        targets=request.get('thermal_targets',{})
        if not isinstance(targets,dict) or set(targets)-{'extruder','heater_bed'}:
            raise request.error('Probing temperatures support extruder and heater_bed only')
        for name,target in targets.items():
            low,high=(50.,300.) if name=='extruder' else (30.,150.)
            if isinstance(target,bool) or not isinstance(target,(int,float)) or not math.isfinite(target) or not low<=target<=high:
                raise request.error('Invalid probing temperature target: '+name)
        self._thermal_targets=dict(targets)
        search_s = self._number(request, 'search_timeout_s', 60., .1, 7200.)
        lease_s = self._number(request, 'lease_s', .5, .1, .5)
        calibrated, cpg, force = self._calibration()
        if not calibrated:
            raise request.error('Native load_cell_probe must be calibrated first')
        if cpg * force >= limit:
            raise request.error('Configured trigger force exceeds the raw-count safety limit')
        error = self._idle_error(self.reactor.monotonic(), check_motion=True)
        error = error or self._sensor_error(self.reactor.monotonic())
        if error:
            raise request.error(error)
        self.session = session
        self._used_sessions = (self._used_sessions + [session])[-64:]
        self.generation += 1
        generation = self.generation
        self.state, self.error = 'arming', ''
        self.stop_required = False
        self.cleanup_confirmed = True
        self.trigger_time = self.arm_time = None
        self.search_deadline = 0.
        self.lease_s = lease_s
        # No external movement is permitted before the arm response. Tare may
        # legitimately consume a future toolhead time plus a bulk sample batch.
        self.deadline = self.reactor.monotonic() + self.SETUP_TIMEOUT_S
        self._terminal_intent = None
        self._arming_worker = True
        try:
            # Native tare configures the existing MCU filter and force trigger.
            self.move._pause_and_tare(None)
            self._assert_arming(generation)
            error = self._idle_error(self.reactor.monotonic())
            error = error or self._sensor_error(self.reactor.monotonic())
            if error:
                raise self.printer.command_error(error)
            tare = self.load_cell.get_tare_counts()
            if not isinstance(tare, (int, float)) or not math.isfinite(tare):
                raise self.printer.command_error('Native tare is unavailable')
            reference = self.load_cell.get_reference_tare_counts()
            if abs(tare - reference) >= cpg * force:
                raise self.printer.command_error('Load is present relative to the calibrated zero; unload the nozzle before arming')
            native_min, native_max = (self.trigger._raw_min, self.trigger._raw_max)
            low = max(native_min, math.ceil(tare - limit))
            high = min(native_max, math.floor(tare + limit))
            if not low < tare < high:
                raise self.printer.command_error('Tare is outside the native safety range')
            if min(tare - low, high - tare) <= cpg * force:
                raise self.printer.command_error('Native safety range cannot contain the configured trigger')
            self.trigger.set_raw_range(low, high)
            now = self.reactor.monotonic()
            self.arm_time = self.mcu.estimated_print_time(now) + self.ARM_LEAD_S
            self.search_deadline = now + search_s
            # Claim cleanup before home_start: partial setup must also be cleared.
            self._native_started = True
            self.cleanup_confirmed = False
            self._cleanup_done = self.reactor.completion()
            self._native_completion = self.trigger.home_start(self.arm_time, 0., 0, 0.)
            completion = self._native_completion
            self.reactor.register_callback(
                lambda eventtime: self._monitor_native(eventtime, generation, completion))
            while True:
                self._assert_arming(generation)
                remaining = self.arm_time - self.mcu.estimated_print_time(self.reactor.monotonic())
                if remaining <= 0:
                    break
                self.reactor.pause(self.reactor.monotonic() + min(remaining, .01))
            # A round-trip confirms MCU setup, not merely that commands were queued.
            status = self.trigger._query_state_cmd.send([self.trigger.get_oid()])
            self._assert_arming(generation)
            if status.get('homing') != 1 or completion.test():
                raise self.printer.command_error('Native probe triggered before robot descent')
            self.state = 'armed'
            self.deadline = self.reactor.monotonic() + lease_s
            self._notify()
        except Exception as exc:
            self._finish('fault', str(exc))
        finally:
            self._arming_worker = False
        request.send(self._snapshot())

    def _monitor_native(self, eventtime, generation, completion):
        completion.wait(self.search_deadline)
        if generation != self.generation or not self._native_started:
            return
        if not completion.test():
            self._finish('fault', 'Probe search timed out')
        elif self.state == 'arming':
            self._finish('fault', 'Native probe triggered before robot descent')
        else:
            self._finish('triggered', '')

    def _finish(self, target, error):
        # Sticky priority means a lease expiry/disarm during serial I/O always wins.
        priorities = {'triggered': 0, 'disarmed': 1, 'fault': 2}
        if (self._terminal_intent is None
                or priorities[target] > priorities[self._terminal_intent[0]]):
            self._terminal_intent = (target, error)
        self.stop_required = True
        self._notify()
        if self._finalizing:
            return
        result, trigger_time, cleanup_error = None, None, None
        if self._native_started:
            self._native_started = False
            self._finalizing = True
            # stop() unregisters the native response handler; wake its waiter first.
            if self._native_completion is not None and not self._native_completion.test():
                self._native_completion.complete(False)
            try:
                result = self.trigger.get_dispatch().stop()
            except Exception as exc:
                cleanup_error = str(exc)
            finally:
                try:
                    trigger_time = self.trigger._clear_home()
                except Exception as exc:
                    cleanup_error = cleanup_error or str(exc)
                self._finalizing = False
                self.cleanup_confirmed = cleanup_error is None
                self._cleanup_done.complete(True)
        target, error = self._terminal_intent
        if cleanup_error:
            target, error = 'fault', 'Native probe cleanup failed: ' + cleanup_error
        elif target == 'triggered':
            now = self.reactor.monotonic()
            if now >= self.deadline or now >= self.search_deadline:
                target, error = 'fault', 'Probe lease or search expired during trigger confirmation'
            elif isinstance(result, int) and result >= self.COMMS_TIMEOUT:
                label = ('communication timeout' if result == self.COMMS_TIMEOUT
                         else self.trigger._error_map.get(
                             result - self.trigger.REASON_TRIGGER_ANALOG,
                             'sensor or MCU fault'))
                target, error = 'fault', 'Native probe fault: %s (reason %s)' % (label, result)
            elif result != self.ENDSTOP_HIT:
                target, error = 'fault', 'Native probe ended without contact (reason %s)' % result
            elif (not isinstance(trigger_time, (int, float))
                    or not math.isfinite(trigger_time)
                    or not self.arm_time <= trigger_time <= self.mcu.estimated_print_time(now)):
                target, error = 'fault', 'Native trigger timestamp is invalid'
            else:
                self.trigger_time = trigger_time
        self.state, self.error = target, error
        if target != 'triggered':
            self.trigger_time = None
        self._notify()

    def _require_session(self, request):
        if self.session is None or request.get('session') != self.session:
            raise request.error('Unknown probe session')

    def api_wait(self, request):
        self._require_session(request)
        timeout = self._number(request, 'timeout_s', .1, 0., .1)
        now = self.reactor.monotonic()
        self._check_deadlines(now)
        if self.state == 'triggered':
            self.deadline = now + self.lease_s
        elif self.state == 'armed' and not self.stop_required:
            self.deadline = now + self.lease_s
            if timeout:
                self._changed.wait(now + timeout)
        self._check_deadlines(self.reactor.monotonic())
        request.send(self._snapshot())

    def api_disarm(self, request):
        self._require_session(request)
        self._finish('disarmed', '')
        if self._finalizing:
            self._cleanup_done.wait(self.reactor.monotonic() + 2.)
        if not self.cleanup_confirmed or self._finalizing:
            raise request.error('Native probe cleanup is not confirmed')
        request.send(self._snapshot())

    def _watchdog(self, eventtime):
        self._check_deadlines(eventtime)
        if self.state in ('arming', 'armed', 'triggered') and not self._finalizing:
            try:
                error = self._idle_error(eventtime)
                error = error or self._sensor_error(eventtime)
            except Exception as exc:
                error = 'Probe safety status unavailable: ' + str(exc)
            if error:
                self._finish('fault', error)
        return eventtime + self.WATCHDOG_S

    def _shutdown(self):
        self._finish('fault', 'Klipper shutdown')
        self.supported, self.support_error = False, 'Klipper shutdown'

    def _restart(self, *args):
        if self.is_busy():
            self._finish('fault', 'Klipper restart requested')


def load_config(config):
    return AxisForgeProbe(config)
