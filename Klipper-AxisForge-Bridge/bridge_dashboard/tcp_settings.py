"""Edit only the planner's nozzle translation, under robot ownership and revision checks."""
from contextlib import contextmanager
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import tempfile

import yaml


class TcpSettingsError(ValueError):
    pass


class TcpSettingsConflict(TcpSettingsError):
    pass


def vector(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 3 or
            not all(type(v) in (int, float) and math.isfinite(v) for v in value)):
        raise TcpSettingsError('TCP must contain three finite numbers in millimetres')
    return [float(v) for v in value]


def mapping_value(node, key):
    if not isinstance(node, yaml.MappingNode): raise TcpSettingsError('Invalid AxisForge parameter mapping')
    entries = [value for name, value in node.value if name.value == key]
    if len(entries) != 1: raise TcpSettingsError('Expected exactly one '+key+' parameter')
    return entries[0]


def parse(raw):
    try:
        text = raw.decode('utf-8')
        if any(isinstance(token, (yaml.tokens.AnchorToken, yaml.tokens.AliasToken)) for token in yaml.scan(text)):
            raise TcpSettingsError('TCP editing requires explicit YAML values without anchors or aliases')
        data = yaml.safe_load(text)
        node = yaml.compose(text)
        for key in ('gcode_interpreter', 'ros__parameters', 'tcp_offset_mm'):
            node = mapping_value(node, key)
        params = data['gcode_interpreter']['ros__parameters']
        values = vector(params['tcp_offset_mm'])
        # Editing aliases/anchors could silently change another parameter too.
        fragment = text[node.start_mark.index:node.end_mark.index]
        if not isinstance(node, yaml.SequenceNode) or '&' in fragment or '*' in fragment:
            raise TcpSettingsError('TCP must be an explicit three-number YAML list without anchors or aliases')
        return text, data, params, node, values
    except (UnicodeError, KeyError, TypeError, yaml.YAMLError) as error:
        raise TcpSettingsError('Cannot read the AxisForge TCP configuration: '+str(error)) from error


class TcpSettings:
    def __init__(self, config_path, *, lock_path=None):
        self.path = Path(config_path) if config_path else None
        self.lock_path = Path(lock_path or os.getenv('AXISFORGE_ROBOT_LOCK', '.axisforge-robot-control.lock'))

    def _read(self):
        if self.path is None: raise TcpSettingsError('AXISFORGE_CONFIG is required to edit the nozzle TCP')
        try: return self.path.read_bytes()
        except OSError as error: raise TcpSettingsError('Cannot read the AxisForge configuration file') from error

    @contextmanager
    def ownership(self):
        try: fd = os.open(self.lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        except OSError as error: raise TcpSettingsError('Cannot check robot ownership; TCP was not changed') from error
        try:
            try: fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as error:
                raise TcpSettingsConflict('The robot is owned by another operation; finish or stop it before saving TCP') from error
            yield
        finally: os.close(fd)

    def status(self):
        raw = self._read()
        _, _, params, _, values = parse(raw)
        bed = {'state':'not_configured', 'requires_probe':False,
               'message':'No bed mesh is configured; the flat-bed settings apply.'}
        filename = params.get('bed_probe_file')
        if filename:
            try:
                measured = yaml.safe_load(Path(filename).expanduser().read_text())
                matches = all(abs(a-b) <= .001 for a,b in zip(vector(measured['tcp_offset_mm']),values))
                valid = measured.get('simulated') is not True and measured.get('complete') is not False
                bed = {'state':'tcp_matches' if matches and valid else 'probe_required',
                       'requires_probe':not (matches and valid),
                       'message':'Bed mesh TCP matches the saved offset; job preflight still checks the complete mesh.'
                                 if matches and valid else 'Run Home & probe bed with the saved TCP before printing.'}
            except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
                bed = {'state':'probe_required', 'requires_probe':True,
                       'message':'The configured bed mesh is missing or invalid. Run Home & probe bed before printing.'}
        return {'tcp_offset_mm':values, 'revision':hashlib.sha256(raw).hexdigest(),
                'frame':'AxisForge kinematic end frame', 'units':'mm',
                'controller_tcp_changed':False, 'bed_mesh':bed}

    @staticmethod
    def _atomic_write(path, raw, mode):
        fd, name = tempfile.mkstemp(prefix='.'+path.name+'.', dir=path.parent)
        try:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(raw); stream.flush(); os.fsync(stream.fileno())
                os.fchmod(stream.fileno(), mode)
            os.replace(name, path)
        finally:
            if os.path.exists(name): os.unlink(name)

    def save(self, values, revision, invalidate):
        values = vector(values)
        if any(abs(v) > 1000. for v in values):
            raise TcpSettingsError('Dashboard TCP entries must be between −1000 and 1000 mm per axis')
        with self.ownership():
            raw = self._read()
            if hashlib.sha256(raw).hexdigest() != revision:
                raise TcpSettingsConflict('Configuration changed since it was loaded. Reload saved TCP and review your values.')
            text, data, _, node, current = parse(raw)
            if current == values: return dict(self.status(), changed=False, message='TCP values are unchanged.')
            expected = copy.deepcopy(data)
            expected['gcode_interpreter']['ros__parameters']['tcp_offset_mm'] = values
            replacement = json.dumps(values, allow_nan=False)
            if not node.flow_style:
                replacement += '\n'+' '*node.end_mark.column
            candidate = text[:node.start_mark.index]+replacement+text[node.end_mark.index:]
            if yaml.safe_load(candidate) != expected:
                raise TcpSettingsError('Cannot update TCP without changing other settings; configuration was not saved')
            if self._read() != raw:
                raise TcpSettingsConflict('Configuration changed while preparing the update. Reload before saving.')
            invalidate('Nozzle TCP changed; run bed homing if needed and repeat job preflight')
            mode = stat.S_IMODE(self.path.stat().st_mode)
            try:
                self._atomic_write(self.path.with_name(self.path.name+'.tcp-backup'), raw, mode)
                self._atomic_write(self.path, candidate.encode('utf-8'), mode)
            except OSError as error:
                raise TcpSettingsError('Could not save TCP; reload the saved values before retrying') from error
            return dict(self.status(), changed=True,
                        message='AxisForge nozzle TCP saved. Run job preflight again; re-probe if the mesh TCP differs.')
