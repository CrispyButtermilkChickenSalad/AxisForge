"""Persist native probing parameters without changing unrelated profile settings."""
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


class ProbingSettingsError(ValueError):
    pass


class ProbingSettingsConflict(ProbingSettingsError):
    pass


FIELDS = {
    'first_speed_mm_s': ('homing', 'first_speed_mm_s', .5, 10.),
    'second_speed_mm_s': ('homing', 'second_speed_mm_s', .02, 1.),
    'lift_speed_mm_s': ('homing', 'lift_speed_mm_s', 1., 20.),
    'mesh_columns': ('homing', 'mesh_columns', 2, 9),
    'mesh_rows': ('homing', 'mesh_rows', 2, 9),
    'mesh_speed_mm_s': ('contact', 'speed_mm_s', .02, .2),
    'mesh_samples': ('contact', 'samples', 3, 5),
    'mesh_repeatability_mm': ('contact', 'repeatability_mm', .005, .05),
}
INTEGER_FIELDS = {'mesh_samples', 'mesh_columns', 'mesh_rows'}


def validate(values):
    if not isinstance(values, dict) or set(values) != set(FIELDS):
        raise ProbingSettingsError('Supply all eight probing settings and no other parameters')
    result = {}
    for name, (_, _, minimum, maximum) in FIELDS.items():
        value = values[name]
        if (type(value) not in (int, float) or not math.isfinite(value) or
                (name in INTEGER_FIELDS and type(value) is not int) or
                not minimum <= value <= maximum):
            raise ProbingSettingsError('%s must be %sbetween %g and %g' %
                                       (name, 'an integer ' if name in INTEGER_FIELDS else '', minimum, maximum))
        result[name] = value if name in INTEGER_FIELDS else float(value)
    if result['second_speed_mm_s'] >= result['first_speed_mm_s']:
        raise ProbingSettingsError('Precision rehoming must be slower than the initial surface search')
    return result


def _mapping(node):
    if not isinstance(node, yaml.MappingNode):
        raise ProbingSettingsError('Probing profile sections must be YAML mappings')
    result = {}
    for key, value in node.value:
        if not isinstance(key, yaml.ScalarNode) or key.value in result:
            raise ProbingSettingsError('Probing profile contains an invalid or duplicate mapping key')
        result[key.value] = value
    return result


def parse(raw):
    try:
        text = raw.decode('utf-8')
        if any(isinstance(token, (yaml.tokens.AnchorToken, yaml.tokens.AliasToken)) for token in yaml.scan(text)):
            raise ProbingSettingsError('Probing editing requires explicit YAML values without anchors or aliases')
        data = yaml.safe_load(text)
        sections = _mapping(yaml.compose(text))
        if set(sections) != {'contact', 'homing'}:
            raise ProbingSettingsError('Probing profile must contain contact and homing sections')
        nodes = {section: _mapping(node) for section, node in sections.items()}
        for section, values in data.items():
            for name, value in values.items():
                if section == 'contact' and name == 'trigger_mode':
                    if value not in ('raw', 'grams'):
                        raise ProbingSettingsError('Invalid contact trigger mode')
                elif type(value) not in (int, float) or not math.isfinite(value):
                    raise ProbingSettingsError('Probing profile values must be finite numbers')
        for trigger, maximum in (('contact_delta_counts', 'max_delta_counts'),
                                 ('contact_force_g', 'max_force_g')):
            contact = data['contact']
            if trigger in contact and maximum in contact and not 0 < contact[trigger] < contact[maximum]:
                raise ProbingSettingsError('Contact trigger must be positive and lower than its stop threshold')
        values = validate({name: data[section][key] for name, (section, key, _, _) in FIELDS.items()})
        editable = {}
        for name, (section, key, _, _) in FIELDS.items():
            node = nodes[section][key]
            if not isinstance(node, yaml.ScalarNode):
                raise ProbingSettingsError('Editable probing settings must be explicit numeric scalars')
            editable[name] = node
        return text, data, editable, values
    except (UnicodeError, KeyError, TypeError, AttributeError, yaml.YAMLError) as error:
        raise ProbingSettingsError('Cannot read the native probing profile: '+str(error)) from error


class ProbingSettings:
    def __init__(self, config_path, *, lock_path=None):
        self.path = Path(config_path).with_name('bridge_bed_probe.yaml') if config_path else None
        self.lock_path = Path(lock_path or os.getenv('AXISFORGE_ROBOT_LOCK', '.axisforge-robot-control.lock'))

    def _read(self):
        if self.path is None:
            raise ProbingSettingsError('AXISFORGE_CONFIG is required to edit probing settings')
        try:
            return self.path.read_bytes()
        except OSError as error:
            raise ProbingSettingsError('Cannot read the native bridge_bed_probe.yaml profile') from error

    @contextmanager
    def ownership(self):
        try:
            fd = os.open(self.lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        except OSError as error:
            raise ProbingSettingsError('Cannot check robot ownership; probing settings were not changed') from error
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as error:
                raise ProbingSettingsConflict('The robot is owned by another operation; finish or stop it before saving') from error
            yield
        finally:
            os.close(fd)

    def status(self):
        raw = self._read()
        _, _, _, values = parse(raw)
        return {'revision': hashlib.sha256(raw).hexdigest(), 'settings': values,
                'bounds': {name: {'min': minimum, 'max': maximum}
                           for name, (_, _, minimum, maximum) in FIELDS.items()}}

    @staticmethod
    def _atomic_write(path, raw, mode):
        fd, name = tempfile.mkstemp(prefix='.'+path.name+'.', dir=path.parent)
        try:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(raw)
                stream.flush()
                os.fsync(stream.fileno())
                os.fchmod(stream.fileno(), mode)
            os.replace(name, path)
        finally:
            if os.path.exists(name):
                os.unlink(name)

    def save(self, values, revision, invalidate):
        values = validate(values)
        with self.ownership():
            raw = self._read()
            if hashlib.sha256(raw).hexdigest() != revision:
                raise ProbingSettingsConflict('Probing configuration changed since it was loaded. Reload and review your values.')
            text, data, nodes, current = parse(raw)
            if current == values:
                return dict(self.status(), changed=False, message='Probing settings are unchanged.')
            expected = copy.deepcopy(data)
            for name, (section, key, _, _) in FIELDS.items():
                expected[section][key] = values[name]
            candidate = text
            for name, node in sorted(nodes.items(), key=lambda item: item[1].start_mark.index, reverse=True):
                candidate = (candidate[:node.start_mark.index] + json.dumps(values[name], allow_nan=False) +
                             candidate[node.end_mark.index:])
            if yaml.safe_load(candidate) != expected:
                raise ProbingSettingsError('Cannot save probing values without changing other settings')
            parse(candidate.encode('utf-8'))
            if self._read() != raw:
                raise ProbingSettingsConflict('Probing configuration changed while preparing the update. Reload before saving.')
            invalidate('Probing settings changed; repeat job preflight before printing')
            try:
                mode = stat.S_IMODE(self.path.stat().st_mode)
                self._atomic_write(self.path.with_name(self.path.name+'.probing-backup'), raw, mode)
                self._atomic_write(self.path, candidate.encode('utf-8'), mode)
            except OSError as error:
                raise ProbingSettingsError('Could not save probing settings; reload before retrying') from error
            return dict(self.status(), changed=True,
                        message='Probing settings saved for the next routine. Run job preflight again before printing.')
