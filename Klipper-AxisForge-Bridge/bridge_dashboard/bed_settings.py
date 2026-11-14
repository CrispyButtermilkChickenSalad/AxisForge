"""Persist build-surface dimensions while preserving the physical front-left corner."""
from contextlib import contextmanager
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import stat

import yaml

from .tcp_settings import TcpSettings, TcpSettingsError, mapping_value


class BedSettingsError(ValueError):
    pass


class BedSettingsConflict(BedSettingsError):
    pass


def dimensions(width, depth, margin):
    if not all(type(v) in (int, float) and math.isfinite(v) for v in (width, depth, margin)):
        raise BedSettingsError('Bed dimensions and probe margin must be finite numbers')
    if not 10 <= width <= 2000 or not 10 <= depth <= 2000:
        raise BedSettingsError('Bed width and depth must be 10..2000 mm')
    if not 0 <= margin < min(width, depth)/2:
        raise BedSettingsError('Probe margin must be nonnegative and less than half the smaller bed dimension')
    return [float(width), float(depth)], float(margin)


def parse(raw):
    try:
        text = raw.decode('utf-8')
        if any(isinstance(t, (yaml.tokens.AnchorToken, yaml.tokens.AliasToken)) for t in yaml.scan(text)):
            raise BedSettingsError('Bed editing requires explicit YAML values without anchors or aliases')
        data = yaml.safe_load(text)
        node = yaml.compose(text)
        for key in ('gcode_interpreter', 'ros__parameters'):
            node = mapping_value(node, key)
        params = data['gcode_interpreter']['ros__parameters']
        keys = ('bed_size_mm', 'bed_probe_margin_mm', 'x_offset', 'y_offset', 'origin_at_center')
        nodes = {key: mapping_value(node, key) for key in keys}
        size = params['bed_size_mm']
        if not isinstance(size, list) or len(size) != 2:
            raise BedSettingsError('bed_size_mm must be an explicit two-number list')
        size, margin = dimensions(*size, params['bed_probe_margin_mm'])
        if (type(params['origin_at_center']) is not bool or
                not all(type(params[k]) in (int, float) and math.isfinite(params[k]) for k in ('x_offset','y_offset'))):
            raise BedSettingsError('Invalid bed origin configuration')
        return text, data, params, nodes, size, margin
    except (UnicodeError, KeyError, TypeError, yaml.YAMLError, TcpSettingsError) as error:
        raise BedSettingsError('Cannot read bed geometry: '+str(error)) from error


class BedSettings:
    def __init__(self, config_path, *, lock_path=None):
        self.path = Path(config_path) if config_path else None
        self.lock_path = Path(lock_path or os.getenv('AXISFORGE_ROBOT_LOCK', '.axisforge-robot-control.lock'))

    def _read(self):
        if self.path is None:
            raise BedSettingsError('AXISFORGE_CONFIG is required to edit the bed dimensions')
        try:
            return self.path.read_bytes()
        except OSError as error:
            raise BedSettingsError('Cannot read the AxisForge configuration') from error

    @contextmanager
    def ownership(self):
        try:
            fd = os.open(self.lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        except OSError as error:
            raise BedSettingsError('Cannot check robot ownership; bed settings were not changed') from error
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError as error:
                raise BedSettingsConflict('The robot is owned by another operation; finish or stop it before saving') from error
            yield
        finally:
            os.close(fd)

    def status(self):
        raw = self._read()
        _, _, params, _, size, margin = parse(raw)
        mesh = {'requires_probe': False, 'message': 'No bed map configured; the flat-bed settings apply.'}
        if params.get('bed_probe_file'):
            try:
                measured = yaml.safe_load(Path(params['bed_probe_file']).expanduser().read_text())
                recorded = measured['transform']['bed_size_mm']
                match = (isinstance(recorded, list) and len(recorded) == 2 and
                         all(type(v) in (int,float) and math.isfinite(v) and abs(v-w) <= .001
                             for v,w in zip(recorded,size)))
                valid = measured.get('simulated') is not True and measured.get('complete') is not False
                mesh = {'requires_probe': not (match and valid),
                        'message': 'Bed map dimensions match; preflight still checks coverage and TCP.' if match and valid else
                                   'Bed dimensions differ from the saved mesh. Run Home & probe bed before printing.'}
            except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
                mesh = {'requires_probe': True, 'message': 'Bed map missing or invalid. Run Home & probe bed before printing.'}
        return {'revision': hashlib.sha256(raw).hexdigest(), 'bed_size_mm': size,
                'probe_margin_mm': margin, 'probe_area': {'x': [margin,size[0]-margin], 'y': [margin,size[1]-margin]},
                'front_left_base_mm': [params[k]+(size[i]/2 if params['origin_at_center'] else 0.)
                                       for i,k in enumerate(('x_offset','y_offset'))], 'bed_mesh': mesh}

    def save(self, width, depth, margin, revision, invalidate):
        size, margin = dimensions(width, depth, margin)
        with self.ownership():
            raw = self._read()
            if hashlib.sha256(raw).hexdigest() != revision:
                raise BedSettingsConflict('AxisForge configuration changed. Reload bed settings before saving.')
            text, data, params, nodes, current, current_margin = parse(raw)
            if size == current and margin == current_margin:
                return dict(self.status(), changed=False, message='Bed settings are unchanged.')
            changes = {'bed_size_mm': size, 'bed_probe_margin_mm': margin}
            if params['origin_at_center']:
                # The legacy transform adds half the bed size. Compensate so a
                # resize changes coverage without moving the calibrated corner.
                changes.update({k: params[k]+(current[i]-size[i])/2
                                for i,k in enumerate(('x_offset','y_offset'))})
            expected = copy.deepcopy(data)
            expected['gcode_interpreter']['ros__parameters'].update(changes)
            candidate = text
            for key in sorted(changes, key=lambda k:nodes[k].start_mark.index, reverse=True):
                node = nodes[key]
                value = json.dumps(changes[key], allow_nan=False)
                if isinstance(node, yaml.SequenceNode) and not node.flow_style:
                    value += '\n'+' '*node.end_mark.column
                candidate = candidate[:node.start_mark.index]+value+candidate[node.end_mark.index:]
            if yaml.safe_load(candidate) != expected:
                raise BedSettingsError('Cannot update bed geometry without changing unrelated configuration')
            parse(candidate.encode('utf-8'))
            if self._read() != raw:
                raise BedSettingsConflict('Configuration changed while preparing the update. Reload before saving.')
            invalidate('Bed geometry changed; repeat homing/mesh and job preflight')
            try:
                mode = stat.S_IMODE(self.path.stat().st_mode)
                TcpSettings._atomic_write(self.path.with_name(self.path.name+'.bed-backup'), raw, mode)
                TcpSettings._atomic_write(self.path, candidate.encode('utf-8'), mode)
            except OSError as error:
                raise BedSettingsError('Could not save bed geometry; reload before retrying') from error
            return dict(self.status(), changed=True,
                        message='Bed dimensions and probe area saved; front-left position preserved. Run Home & probe bed, then job preflight.')
