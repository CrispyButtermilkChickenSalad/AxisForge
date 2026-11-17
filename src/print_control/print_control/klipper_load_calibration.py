"""Verified stock Klipper load-cell calibration; no robot motion commands.

The caller owns stationary/unloaded/load/thermal checks through ``pump``. Stock
Klipper has no calibration session token: ownership is cooperative and audited
through uniquely marked Moonraker command history. Never retry mutations.
"""
import hashlib
import json
import math
import re
import threading
import time
import uuid
from urllib.parse import urlencode, urlsplit
from urllib.request import Request, urlopen


class KlipperLoadCalibrationError(RuntimeError):
    pass


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


class KlipperLoadCalibration:
    CELL = 'load_cell_probe'
    HELPER_COMMANDS = {'TARE', 'CALIBRATE', 'ACCEPT', 'ABORT'}
    FIELDS = {'counts_per_gram', 'reference_tare_counts'}

    def __init__(self, moonraker_url, api_key=None, *, opener=urlopen,
                 clock=time.monotonic, request_timeout_s=4., restart_timeout_s=45.,
                 on_state=None):
        parts = urlsplit(moonraker_url)
        if (parts.scheme not in ('http', 'https') or not parts.hostname
                or parts.username or parts.password or parts.query or parts.fragment):
            raise ValueError('Use a Moonraker HTTP(S) URL without embedded credentials')
        if (not _finite(request_timeout_s) or not .1 <= request_timeout_s <= 10.
                or not _finite(restart_timeout_s) or not 1. <= restart_timeout_s <= 120.):
            raise ValueError('Invalid calibration request/restart deadline')
        self.base_url, self.api_key = moonraker_url.rstrip('/'), api_key
        self.opener, self.clock = opener, clock
        self.request_timeout_s, self.restart_timeout_s = request_timeout_s, restart_timeout_s
        self.on_state = on_state or (lambda state: None)
        self.phase, self.owned = 'new', False
        self.tare = self.record = self.receipt = None
        self._config_fingerprint = self._other_fingerprint = None
        self._scripts = []
        self._begin_script = None
        self._last_command = None
        self._pending = None
        self._nonce = uuid.uuid4().hex
        self._sequence = 0
        self._save_sent = False

    def _request(self, path, body=None, timeout=None):
        headers = {'Accept': 'application/json'}
        if self.api_key:
            headers['X-Api-Key'] = self.api_key
        if body is not None:
            headers['Content-Type'] = 'application/json'
        request = Request(self.base_url + path,
            data=None if body is None else json.dumps(body, allow_nan=False).encode(),
            headers=headers, method='GET' if body is None else 'POST')
        with self.opener(request, timeout=timeout or self.request_timeout_s) as response:
            raw = response.read(2097153)
        if len(raw) > 2097152:
            raise KlipperLoadCalibrationError('Moonraker calibration response is too large')
        data = json.loads(raw)
        if not isinstance(data, dict) or 'error' in data or 'result' not in data:
            raise KlipperLoadCalibrationError('Moonraker rejected the calibration request')
        return data['result']

    def _io(self, operation, pump, *, timeout=None):
        if self._pending is not None:
            raise KlipperLoadCalibrationError('A previous request is unresolved; cleanup is required')
        pump()
        slot = {'done': threading.Event(), 'value': None, 'error': None}
        def worker():
            try:
                slot['value'] = operation()
            except BaseException as error:
                slot['error'] = error
            finally:
                slot['done'].set()
        slot['thread'] = threading.Thread(target=worker, daemon=True)
        self._pending = slot
        slot['thread'].start()
        deadline = self.clock() + (timeout or self.request_timeout_s + .5)
        try:
            while not slot['done'].wait(.005):
                pump()
                if self.clock() >= deadline:
                    raise KlipperLoadCalibrationError('Calibration request timed out; it was not retried')
            pump()
        except BaseException:
            # Retain the in-flight operation for bounded cleanup, never resubmit it.
            raise
        self._pending = None
        if slot['error'] is not None:
            raise KlipperLoadCalibrationError(str(slot['error'])) from slot['error']
        return slot['value']

    def _settle_pending(self, pump):
        slot = self._pending
        if slot is None:
            return
        deadline = self.clock() + self.request_timeout_s + .5
        while not slot['done'].wait(.005):
            pump()
            if self.clock() >= deadline:
                raise KlipperLoadCalibrationError('Calibration request is still unresolved; cleanup not confirmed')
        self._pending = None

    def _query(self, pump):
        names = ('webhooks', 'gcode', 'configfile', self.CELL, 'virtual_sdcard',
                 'print_stats', 'axisforge_sync', 'extruder', 'heater_bed')
        result = self._io(lambda: self._request('/printer/objects/query?' +
                          urlencode({name: '' for name in names})), pump)
        if not isinstance(result, dict) or not isinstance(result.get('status'), dict):
            raise KlipperLoadCalibrationError('Invalid Klipper object status')
        return result['status']

    def _store(self, pump):
        result = self._io(lambda: self._request('/server/gcode_store?count=1000'), pump)
        rows = result.get('gcode_store') if isinstance(result, dict) else None
        if (not isinstance(rows, list) or len(rows) > 1000
                or any(not isinstance(row, dict) or not isinstance(row.get('message'), str)
                       or row.get('type') not in ('command', 'response') for row in rows)):
            raise KlipperLoadCalibrationError('Invalid Moonraker G-code history')
        return rows

    def _audit(self, rows):
        if self._begin_script is None:
            return
        positions = [i for i, row in enumerate(rows)
                     if row['type'] == 'command' and row['message'].strip() == self._begin_script]
        if len(positions) != 1:
            self.owned = False
            raise KlipperLoadCalibrationError('Calibration ownership history is missing or truncated')
        for row in rows[positions[0]:]:
            if row['type'] != 'command' or row['message'].strip() in self._scripts:
                continue
            if row['message'].strip().upper() in ('M105', 'STATUS', 'HELP', 'GET_POSITION'):
                continue
            self.owned = False
            raise KlipperLoadCalibrationError('Another console issued G-code during calibration; ownership is uncertain')

    @staticmethod
    def _responses(rows, begin, end):
        lines = []
        for row in rows:
            if row['type'] == 'response':
                for line in row['message'].splitlines():
                    lines.append(re.sub(r'^\s*//\s?', '', line).strip())
        starts = [i for i, line in enumerate(lines) if line == 'ECHO MSG=' + begin]
        ends = [i for i, line in enumerate(lines) if line == 'ECHO MSG=' + end]
        if len(starts) != 1 or len(ends) != 1 or starts[0] >= ends[0]:
            raise KlipperLoadCalibrationError('Native command response markers are incomplete; command was not retried')
        return '\n'.join(lines[starts[0] + 1:ends[0]])

    def _command(self, command, pump, *, begin_session=False):
        self._sequence += 1
        tag = 'AXISFORGE_CAL_%s_%s' % (self._nonce, self._sequence)
        begin, end = tag + '_BEGIN', tag + '_END'
        # ECHO is an extended G-code command, so arguments must be KEY=value.
        # Its stock handler echoes the complete original command line.
        script = 'ECHO MSG=%s\n%s\nECHO MSG=%s' % (begin, command, end)
        self._scripts.append(script)
        self._last_command = (command, begin, end, script)
        if begin_session:
            self._begin_script = script
        try:
            self._io(lambda: self._request('/printer/gcode/script', {'script': script}), pump)
            rows = self._store(pump)
            self._audit(rows)
            return self._responses(rows, begin, end)
        except BaseException:
            self.phase = 'uncertain'
            raise

    @staticmethod
    def _pending_items(status):
        config = status.get('configfile', {})
        pending = config.get('save_config_pending_items')
        if type(config.get('save_config_pending')) is not bool or not isinstance(pending, dict):
            raise KlipperLoadCalibrationError('Klipper pending-configuration status is unavailable')
        return config['save_config_pending'], pending

    @classmethod
    def _fingerprints(cls, status):
        config = status.get('configfile', {}).get('config')
        if not isinstance(config, dict) or not isinstance(config.get(cls.CELL), dict):
            raise KlipperLoadCalibrationError('Loaded Klipper configuration is unavailable')
        other = json.loads(json.dumps(config))
        for field in cls.FIELDS:
            other[cls.CELL].pop(field, None)
        return _digest(config), _digest(other)

    @classmethod
    def _idle(cls, status):
        if status.get('webhooks', {}).get('state') != 'ready':
            raise KlipperLoadCalibrationError('Klipper is not ready')
        if (status.get('virtual_sdcard', {}).get('is_active')
                or status.get('print_stats', {}).get('state') in ('printing', 'paused')):
            raise KlipperLoadCalibrationError('Klipper has an active or paused print')
        sync = status.get('axisforge_sync', {})
        if sync.get('leased') or sync.get('preparing') or sync.get('reservation'):
            raise KlipperLoadCalibrationError('A synchronized print owns Klipper')
        sensor = status.get(cls.CELL, {})
        if not sensor or sensor.get('errors', 0) or sensor.get('overflows', 0):
            raise KlipperLoadCalibrationError('Native load-cell sensor is unavailable or reports errors')

    def _check_owned(self, pump):
        if not self.owned:
            raise KlipperLoadCalibrationError('This client does not own the native calibration helper')
        rows = self._store(pump)
        self._audit(rows)
        status = self._query(pump)
        self._idle(status)
        commands = status.get('gcode', {}).get('commands', {})
        if not self.HELPER_COMMANDS.issubset(commands):
            self.owned = False
            raise KlipperLoadCalibrationError('Native calibration helper disappeared or changed')
        pending, items = self._pending_items(status)
        if pending or items:
            raise KlipperLoadCalibrationError('Unrelated pending configuration changes prevent calibration')
        if self._fingerprints(status)[0] != self._config_fingerprint:
            self.owned = False
            raise KlipperLoadCalibrationError('Klipper configuration changed during calibration')
        return status

    def prepare(self, pump=lambda: None):
        if self.phase not in ('new', 'prepared'):
            raise KlipperLoadCalibrationError('Calibration has already started')
        status = self._query(pump)
        self._idle(status)
        commands = status.get('gcode', {}).get('commands', {})
        if self.HELPER_COMMANDS.intersection(commands):
            raise KlipperLoadCalibrationError('Another native calibration is open; finish or abort it manually first')
        if 'LOAD_CELL_CALIBRATE' not in commands or 'ECHO' not in commands:
            raise KlipperLoadCalibrationError('Required stock Klipper calibration commands are unavailable')
        pending, items = self._pending_items(status)
        if pending or items:
            raise KlipperLoadCalibrationError('Resolve existing pending SAVE_CONFIG changes before calibration')
        self._config_fingerprint, self._other_fingerprint = self._fingerprints(status)
        self._store(pump)  # Prove the bounded response journal is available before mutation.
        self.phase = 'prepared'
        return {'prepared': True}

    def begin_unloaded(self, pump=lambda: None):
        if self.phase == 'new':
            self.prepare(pump)
        if self.phase != 'prepared':
            raise KlipperLoadCalibrationError('Native calibration cannot be started twice')
        # Recheck immediately before creating a helper; never adopt an existing one.
        self.prepare(pump)
        response = self._command('LOAD_CELL_CALIBRATE LOAD_CELL=' + self.CELL,
                                 pump, begin_session=True)
        if 'Starting load cell calibration.' not in response:
            self.phase = 'uncertain'
            raise KlipperLoadCalibrationError('Native calibration start was not confirmed')
        self.owned = True
        self._check_owned(pump)
        response = self._command('TARE', pump)
        match = re.search(r'Load cell tare value:\s*[-+\d.]+%\s*\((-?\d+)\)', response)
        if not match:
            self.phase = 'uncertain'
            raise KlipperLoadCalibrationError('Native unloaded tare was not reported')
        self.tare = int(match.group(1))
        self.phase = 'tared'
        return {'reference_tare_counts': self.tare}

    def record_load(self, grams, pump=lambda: None):
        if not _finite(grams) or not 50. <= grams <= 25000.:
            raise ValueError('Known load must be a finite 50..25000 grams')
        if self.phase != 'tared':
            raise KlipperLoadCalibrationError('A confirmed native tare is required before recording one load')
        self._check_owned(pump)
        self.record = None
        response = self._command('CALIBRATE GRAMS=%.9g' % grams, pump)
        match = re.search(r'Counts/gram:\s*([-+\d.]+)', response)
        if not match or not _finite(float(match.group(1))) or float(match.group(1)) < 1.:
            self.phase = 'uncertain'
            raise KlipperLoadCalibrationError('Klipper did not report a valid native counts-per-gram result')
        self.record = {'reference_tare_counts': self.tare,
                       'counts_per_gram': float(match.group(1)), 'grams': float(grams)}
        self.phase = 'recorded'
        return dict(self.record)

    @classmethod
    def _verify_values(cls, status, expected, *, pending):
        sensor = status.get(cls.CELL, {})
        if sensor.get('is_calibrated') is not True:
            raise KlipperLoadCalibrationError('Klipper did not apply the native calibration')
        for field in cls.FIELDS:
            actual = sensor.get(field)
            tolerance = .0000051 if field == 'counts_per_gram' else 0.
            if (not _finite(actual) or abs(actual - expected[field]) > tolerance):
                raise KlipperLoadCalibrationError('Klipper calibration does not match the recorded ' + field)
        dirty, items = cls._pending_items(status)
        if pending:
            if not dirty or set(items) != {cls.CELL} or not isinstance(items[cls.CELL], dict) or set(items[cls.CELL]) != cls.FIELDS:
                raise KlipperLoadCalibrationError('Unexpected pending SAVE_CONFIG changes; configuration was not saved')
            values = items[cls.CELL]
        else:
            if dirty or items:
                raise KlipperLoadCalibrationError('Calibration remains pending after restart')
            values = status.get('configfile', {}).get('config', {}).get(cls.CELL, {})
        for field in cls.FIELDS:
            try:
                value = float(values[field])
            except (KeyError, TypeError, ValueError):
                raise KlipperLoadCalibrationError('Persisted/pending calibration value is missing: ' + field)
            if not math.isfinite(value) or abs(value - expected[field]) > 1e-8:
                raise KlipperLoadCalibrationError('Persisted/pending calibration differs from native recording: ' + field)

    def accept(self, pump=lambda: None):
        if self.phase != 'recorded' or not self.record:
            raise KlipperLoadCalibrationError('A native known-load result is required before acceptance')
        self._check_owned(pump)
        try:
            self._command('ACCEPT', pump)
        except BaseException as error:
            raise KlipperLoadCalibrationError('Native ACCEPT may have applied calibration, but SAVE_CONFIG was not sent: '
                                             + str(error)) from error
        self.owned = False  # Native ACCEPT removes the helper before applying its values.
        self.phase = 'accepted_unverified'
        try:
            status = self._query(pump)
            self._idle(status)
            self._verify_values(status, self.record, pending=True)
            if self.HELPER_COMMANDS.intersection(status.get('gcode', {}).get('commands', {})):
                raise KlipperLoadCalibrationError('Unexpected calibration helper remains after ACCEPT')
            if self._fingerprints(status)[0] != self._config_fingerprint:
                raise KlipperLoadCalibrationError('Loaded configuration changed before SAVE_CONFIG')
        except BaseException as error:
            raise KlipperLoadCalibrationError('Native ACCEPT completed, but pending calibration verification failed; '
                                             'configuration was not saved: ' + str(error)) from error
        self.receipt = dict(self.record, accepted=True, saved=False,
            config_fingerprint=self._config_fingerprint,
            other_config_fingerprint=self._other_fingerprint)
        self.phase = 'accepted'
        return dict(self.receipt)

    def abort(self, pump=lambda: None):
        """Clean only a proven owned helper; never undo accepted configuration."""
        self._settle_pending(pump)
        if self.phase in ('new', 'prepared', 'aborted', 'saved'):
            return {'aborted': self.phase == 'aborted', 'cleanup_confirmed': True}
        rows = self._store(pump)
        self._audit(rows)
        status = self._query(pump)
        helper_active = self.HELPER_COMMANDS.issubset(status.get('gcode', {}).get('commands', {}))
        if not helper_active:
            self.owned = False
            dirty, items = self._pending_items(status)
            return {'aborted': False, 'cleanup_confirmed': True,
                    'accepted_unsaved': bool(dirty or items)}
        if not self.owned:
            # A timed-out start may have completed. Require its exact native response.
            if not self._last_command or self._last_command[0] != 'LOAD_CELL_CALIBRATE LOAD_CELL=' + self.CELL:
                raise KlipperLoadCalibrationError('Calibration ownership is uncertain; no ABORT was sent')
            text = self._responses(rows, self._last_command[1], self._last_command[2])
            if 'Starting load cell calibration.' not in text:
                raise KlipperLoadCalibrationError('Calibration ownership is uncertain; no ABORT was sent')
            self.owned = True
        self._command('ABORT', pump)
        status = self._query(pump)
        if self.HELPER_COMMANDS.intersection(status.get('gcode', {}).get('commands', {})):
            raise KlipperLoadCalibrationError('Native calibration abort was not confirmed')
        self.owned, self.phase = False, 'aborted'
        return {'aborted': True, 'cleanup_confirmed': True}

    def save(self, pump=lambda: None):
        if self.receipt is None:
            raise KlipperLoadCalibrationError('Verified ACCEPT is required before saving')
        return self.verify_and_save(self.receipt, pump)

    def cancel_existing(self, pump=lambda: None):
        """Explicit operator action: discard an identified unfinished load helper.

        Never call automatically on startup or failure. Other interactive tools
        also register ABORT, so exact native command descriptions are required.
        """
        if self.owned or self._pending is not None:
            raise KlipperLoadCalibrationError('An automated calibration operation is still active')
        status = self._query(pump)
        self._idle(status)
        pending, items = self._pending_items(status)
        if pending or items:
            raise KlipperLoadCalibrationError('Pending configuration changes prevent clearing calibration')
        before = self._fingerprints(status)[0]
        commands = status.get('gcode', {}).get('commands', {})
        if not self.HELPER_COMMANDS.intersection(commands):
            return {'cleared': False, 'cleanup_confirmed': True}
        descriptions = {'TARE': 'Tare the load cell',
                        'CALIBRATE': 'Enter the load cell value in grams',
                        'ABORT': 'Abort load cell calibration tool'}
        if any(not isinstance(commands.get(name), dict)
               or commands[name].get('help') != description
               for name, description in descriptions.items()):
            raise KlipperLoadCalibrationError('The active interactive tool is not an identified native load-cell calibration')
        self._command('ABORT', pump)
        status = self._query(pump)
        pending, items = self._pending_items(status)
        if (self.HELPER_COMMANDS.intersection(status.get('gcode', {}).get('commands', {}))
                or pending or items or self._fingerprints(status)[0] != before):
            raise KlipperLoadCalibrationError('Clearing unfinished native calibration was not confirmed')
        self.phase = 'aborted'
        return {'cleared': True, 'cleanup_confirmed': True}

    def verify_and_save(self, expected, pump=lambda: None):
        """Run only after robot teardown: heaters off, save once, verify a restart."""
        required = self.FIELDS | {'config_fingerprint', 'other_config_fingerprint', 'accepted'}
        if (not isinstance(expected, dict) or not required.issubset(expected)
                or expected['accepted'] is not True
                or not _finite(expected['counts_per_gram']) or expected['counts_per_gram'] < 1
                or type(expected['reference_tare_counts']) is not int
                or any(not isinstance(expected[key], str) or not re.fullmatch(r'[0-9a-f]{64}', expected[key])
                       for key in ('config_fingerprint', 'other_config_fingerprint'))):
            raise ValueError('A verified native calibration acceptance receipt is required')
        if self._save_sent:
            raise KlipperLoadCalibrationError('SAVE_CONFIG was already submitted; inspect its result instead of retrying')
        status = self._query(pump)
        self._idle(status)
        self._verify_values(status, expected, pending=True)
        if self.HELPER_COMMANDS.intersection(status.get('gcode', {}).get('commands', {})):
            raise KlipperLoadCalibrationError('Another native calibration is active')
        if self._fingerprints(status)[0] != expected['config_fingerprint']:
            raise KlipperLoadCalibrationError('Configuration changed since native ACCEPT')
        # This phase runs after the calibration child exits: its hot monitoring
        # must already be closed and robot termination independently confirmed.
        self.on_state('saving_config')
        self._io(lambda: self._request('/printer/gcode/script', {'script': 'TURN_OFF_HEATERS'}), pump)
        status = self._query(pump)
        self._idle(status)
        self._verify_values(status, expected, pending=True)
        if self._fingerprints(status)[0] != expected['config_fingerprint']:
            raise KlipperLoadCalibrationError('Configuration changed before SAVE_CONFIG')
        configured = status['configfile']['config']
        if any(status.get(name, {}).get('target') != 0 for name in ('extruder', 'heater_bed') if name in configured):
            raise KlipperLoadCalibrationError('Heater shutoff was not confirmed; configuration was not saved')
        stopped = threading.Event()
        observed_restart = threading.Event()
        observer_started = threading.Event()
        def observe():
            while not stopped.is_set():
                try:
                    info = self._request('/server/info', timeout=.5)
                    if isinstance(info, dict) and info.get('klippy_state') == 'ready' and info.get('klippy_connected') is True:
                        observer_started.set()
                    if self._save_sent and isinstance(info, dict) and (info.get('klippy_connected') is False
                            or info.get('klippy_state') in ('disconnected', 'startup', 'shutdown')):
                        observed_restart.set()
                except Exception:
                    pass  # A transport error alone never proves a Klipper restart.
                stopped.wait(.05)
        observer = threading.Thread(target=observe, daemon=True)
        observer.start()
        observer_deadline = self.clock() + 1.5
        while not observer_started.wait(.01):
            try:
                pump()
                if self.clock() >= observer_deadline:
                    raise KlipperLoadCalibrationError('Cannot observe Klipper readiness before SAVE_CONFIG')
            except BaseException:
                stopped.set()
                observer.join(.6)
                raise
        save_error = None
        self.phase, self._save_sent = 'saving', True
        try:
            try:
                self._io(lambda: self._request('/printer/gcode/script', {'script': 'SAVE_CONFIG'}), pump)
            except KlipperLoadCalibrationError as error:
                save_error = str(error)  # SAVE_CONFIG can disconnect the HTTP request.
                self._settle_pending(pump)
            self.on_state('verifying_config')
            deadline = self.clock() + self.restart_timeout_s
            last_error = None
            while self.clock() < deadline:
                pump()
                try:
                    status = self._query(pump)
                    self._idle(status)
                    self._verify_values(status, expected, pending=False)
                    if self._fingerprints(status)[1] != expected['other_config_fingerprint']:
                        raise KlipperLoadCalibrationError('Unrelated configuration changed during SAVE_CONFIG')
                    if observed_restart.is_set():
                        self.phase = 'saved'
                        self.receipt = dict(expected, saved=True, restart_confirmed=True)
                        return dict(self.receipt)
                    last_error = 'Klipper restart was not independently observed'
                except KlipperLoadCalibrationError as error:
                    last_error = str(error)
                time.sleep(.05)
            self.phase = 'save_unverified'
            raise KlipperLoadCalibrationError('SAVE_CONFIG was submitted once, but persistence/restart is unverified: '
                + str(last_error or save_error or 'Klipper did not return ready'))
        finally:
            stopped.set()
            observer.join(.6)

