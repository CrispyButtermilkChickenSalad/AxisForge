"""Deterministic stop-to-stop Cartesian/rotary schedule owned by Klipper.

No ROS, networking, heating or motion side effects. Progress u is dimensionless;
the same trapezoid drives XYZ, ABC and E. Chunk boundaries have zero velocity.
"""

import hashlib
import json
import math
import re

PROTOCOL = 'axisforge.sync.v1'
WORD = re.compile(r'([A-Za-z])\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))?')
MAX_OPERATIONS = 20000


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('%s must be a finite number' % name)
    return float(value)


def profile(start, end, delta_e, feed, limits):
    """Plan normalized progress using every participating axis' speed/accel limit."""
    linear = math.dist(start[:3], end[:3])
    angular = max(abs(a-b) for a, b in zip(start[3:], end[3:]))
    speed_limits, accel_limits = [], []
    for distance, speed, accel in (
        (linear, min(feed, limits['linear_velocity']), limits['linear_acceleration']),
        (angular, limits['angular_velocity'], limits['angular_acceleration']),
        (abs(delta_e), min(feed, limits['extrusion_velocity']) if not linear else
         limits['extrusion_velocity'], limits['extrusion_acceleration']),
    ):
        if distance > 1e-12:
            speed_limits.append(speed / distance)
            accel_limits.append(accel / distance)
    if not speed_limits:
        return None
    v, a = min(speed_limits), min(accel_limits)
    if v <= 0 or a <= 0:
        raise ValueError('Motion speed and acceleration must be positive')
    v = min(v, math.sqrt(a))
    ramp = v / a
    cruise = max(0.0, (1.0 - v * ramp) / v)
    duration = 2 * ramp + cruise
    if duration > limits.get('max_move_seconds', 120.0):
        raise ValueError('A move exceeds the maximum duration; split the source move')
    return {'acceleration': a, 'velocity': v, 'ramp_s': ramp,
            'cruise_s': cruise, 'duration_s': duration}


def progress(p, seconds):
    """Return u and du/dt; reject extrapolation outside the scheduled move."""
    t = finite(seconds, 'sample time')
    if t < -1e-9 or t > p['duration_s'] + 1e-9:
        raise ValueError('Sample outside move')
    t = max(0.0, min(t, p['duration_s']))
    if t <= p['ramp_s']:
        return 0.5*p['acceleration']*t*t, p['acceleration']*t
    if t <= p['ramp_s'] + p['cruise_s']:
        return p['velocity']*(t - 0.5*p['ramp_s']), p['velocity']
    remaining = p['duration_s'] - t
    return 1 - 0.5*p['acceleration']*remaining*remaining, p['acceleration']*remaining


def heater_check_options(value=None):
    """Keep legacy checks enabled unless explicitly disabled with a JSON boolean."""
    checks = {'extruder': True, 'heater_bed': True}
    if value is None:
        return checks
    if (not isinstance(value, dict) or set(value) - set(checks)
            or any(type(enabled) is not bool for enabled in value.values())):
        raise ValueError('heater_checks must contain only extruder/ heater_bed booleans')
    checks.update(value)
    return checks


def compile_job(text, limits, heaters, heater_checks=None):
    """Parse the supported printer dialect; unknown commands never execute implicitly."""
    checks = heater_check_options(heater_checks)
    for key in ('linear_velocity', 'linear_acceleration', 'angular_velocity',
                'angular_acceleration', 'extrusion_velocity', 'extrusion_acceleration'):
        if finite(limits.get(key), key) <= 0:
            raise ValueError('Invalid machine limit: ' + key)
    position, angles, offset = [0.0]*3, [0.0]*3, [0.0]*3
    extrusion, resolved_e, unit, feed, flow, speed_factor = 0., 0., 1., None, 1., 1.
    xyz_absolute = e_absolute = True
    mode, entry = None, None
    operations, pending, targets, peaks = [], [], {}, {}
    requested_peaks, ignored_heaters = {}, set()
    pending_duration, sequence = 0., 0
    warnings = []

    def flush():
        nonlocal pending, pending_duration, sequence
        if pending:
            operations.append({'kind': 'chunk', 'index': sequence,
                               'moves': pending, 'duration_s': pending_duration})
            pending, pending_duration = [], 0.
            sequence += 1

    for number, raw in enumerate(text.splitlines(), 1):
        try:
            line = re.sub(r'\([^()]*\)', '', raw.split(';', 1)[0]).strip().upper()
            if not line:
                continue
            if line.startswith('SET_PRESSURE_ADVANCE'):
                match = re.fullmatch(r'SET_PRESSURE_ADVANCE\s+ADVANCE=([\d.]+)', line)
                if not match or not math.isclose(float(match[1]), limits['pressure_advance'],
                                                rel_tol=0, abs_tol=1e-9):
                    raise ValueError('Pressure advance must match the Klipper configuration')
                continue
            if WORD.sub('', line).strip():
                raise ValueError('Unsupported macro, expression or malformed G-code')
            words = {}
            for letter, value in WORD.findall(line):
                if letter in words:
                    raise ValueError('Duplicate ' + letter)
                words[letter] = None if not value else finite(float(value), letter)
            words.pop('N', None)
            commands = [key for key in 'GMT' if key in words]
            if len(commands) > 1:
                # M104/M109 T0 is a parameter, not a tool-change command.
                if commands == ['M', 'T'] and words['M'] in (104., 109.) and words['T'] == 0:
                    words.pop('T')
                    commands = ['M']
                else:
                    raise ValueError('Only one command per line is supported')
            command = None
            if commands:
                key = commands[0]
                val = words.pop(key)
                if val is None or not val.is_integer():
                    raise ValueError('Invalid command number')
                command = key + str(int(val))
            if command == 'G28':
                if entry is not None or set(words) - set('XYZ'):
                    raise ValueError('G28 is only supported before the robot entry move')
                warnings.append('Line %d: robot entry approach replaces Cartesian homing' % number)
                continue
            if any(value is None for value in words.values()):
                raise ValueError('Missing parameter value')
            if command is None:
                if set(words) <= {'F'}:
                    if words.get('F', 0) <= 0:
                        raise ValueError('Feed must be positive')
                    feed = words['F'] * unit / 60
                    continue
                command = mode or ('G1' if set(words) & set('ABC') else '')
            allowed = {'G0':'XYZABCEF', 'G1':'XYZABCEF', 'G90':'', 'G91':'',
                       'G20':'', 'G21':'', 'G92':'XYZE', 'M82':'', 'M83':'',
                       'M220':'S', 'M221':'S', 'G4':'PS', 'M400':'', 'M84':'',
                       'M104':'S', 'M109':'SR', 'M140':'S', 'M190':'SR',
                       'M106':'SP', 'M107':'', 'T0':''}
            if command not in allowed or set(words) - set(allowed[command]):
                raise ValueError('Unsupported command or parameters: ' + command)
            if command in ('G90', 'G91'):
                xyz_absolute = e_absolute = command == 'G90'
            elif command in ('M82', 'M83'):
                e_absolute = command == 'M82'
            elif command in ('G20', 'G21'):
                unit = 25.4 if command == 'G20' else 1.
            elif command == 'G92':
                if not words: words = dict.fromkeys('XYZE', 0.)
                for i, axis in enumerate('XYZ'):
                    if axis in words:
                        offset[i] = position[i] - words[axis]*unit
                if 'E' in words:
                    extrusion = words['E']*unit
            elif command in ('M220', 'M221'):
                factor = words.get('S', -1)/100
                if factor < 0 or (factor == 0 and command == 'M220'):
                    raise ValueError('Invalid speed/flow factor')
                if command == 'M220': speed_factor = factor
                else: flow = factor
            elif command in ('M104', 'M109', 'M140', 'M190'):
                flush()
                heater = 'extruder' if command in ('M104', 'M109') else 'heater_bed'
                target = words.get('S', words.get('R'))
                if target is None or target < 0 or ('S' in words and 'R' in words):
                    raise ValueError('A heater command requires one nonnegative S/R target')
                requested_peaks[heater] = max(requested_peaks.get(heater, 0), target)
                if heater not in heaters:
                    if target == 0:
                        continue
                    if not checks[heater]:
                        ignored_heaters.add(heater)
                        warnings.append('Line %d: %s %g C ignored for this job; %s is not configured and its preflight check is disabled'
                                        % (number, command, target, heater))
                        continue
                    raise ValueError('Heater is not configured: %s; disable its preflight check to ignore its commands for this job' % heater)
                cfg = heaters[heater]
                if target and not cfg['min_temp'] <= target <= cfg['max_temp']:
                    raise ValueError('Heater target outside configured limits: ' + heater)
                targets[heater] = target
                peaks[heater] = max(peaks.get(heater, 0), target)
                operations.append({'kind':'heater', 'heater':heater, 'target':target,
                                   'wait':command in ('M109', 'M190'), 'source_line':number})
            elif command in ('M106', 'M107'):
                flush()
                value = words.get('S', 255)/255 if command == 'M106' else 0.
                if words.get('P', 0) != 0 or not 0 <= value <= 1:
                    raise ValueError('Only the primary fan and S0..255 are supported')
                operations.append({'kind':'fan', 'speed':value, 'source_line':number})
            elif command in ('G4', 'M400'):
                flush()
                seconds = words.get('S', words.get('P', 0)/1000)
                if not 0 <= seconds <= 600:
                    raise ValueError('Dwell must be between 0 and 600 seconds')
                if seconds:
                    operations.append({'kind':'dwell', 'duration_s':seconds})
            elif command in ('G0', 'G1'):
                mode = command
                if 'F' in words:
                    if words['F'] <= 0: raise ValueError('Feed must be positive')
                    feed = words['F']*unit/60
                move_feed = feed or (limits['linear_velocity'] if command == 'G0' else None)
                if move_feed is None: raise ValueError('Motion requires a feed rate')
                start = position[:] + angles[:]
                for i, axis in enumerate('XYZ'):
                    if axis in words:
                        position[i] = (words[axis]*unit+offset[i] if xyz_absolute else
                                       position[i]+words[axis]*unit)
                for i, axis in enumerate('ABC'):
                    if axis in words: angles[i] = words[axis]
                next_e = (words['E']*unit + (0 if e_absolute else extrusion)
                          if 'E' in words else extrusion)
                delta_e, extrusion = (next_e-extrusion)*flow, next_e
                end = position[:] + angles[:]
                if entry is None:
                    if delta_e: raise ValueError('Position XY before extruding')
                    if set(words) & set('XY'):
                        entry = end[:]
                    continue
                if delta_e:
                    minimum = limits['min_extrude_temp']
                    if targets.get('extruder', 0) < minimum:
                        raise ValueError('Extrusion requires an earlier M104/M109 target >= %g' % minimum)
                    linear = math.dist(start[:3], end[:3])
                    if linear and delta_e > 0 and delta_e/linear > limits['max_extrude_ratio']:
                        raise ValueError('Extrusion exceeds configured cross-section')
                    if (delta_e < 0 or start[:2] == end[:2]) and abs(delta_e) > limits['max_extrude_distance']:
                        raise ValueError('Extrusion-only move exceeds configured distance')
                p = profile(start, end, delta_e, move_feed*speed_factor, limits)
                if p:
                    pending.append({'start':start, 'end':end, 'e_start':resolved_e,
                                    'delta_e':delta_e, 'profile':p, 'source_line':number,
                                    'offset_s':pending_duration})
                    resolved_e += delta_e
                    pending_duration += p['duration_s']
                    if pending_duration >= limits.get('chunk_seconds', 2.0): flush()
            if len(operations)+len(pending) > MAX_OPERATIONS:
                raise ValueError('Job exceeds operation limit')
        except (ValueError, OverflowError) as error:
            raise ValueError('G-code line %d: %s' % (number, error)) from error
    flush()
    if entry is None or not any(op['kind']=='chunk' for op in operations):
        raise ValueError('Job needs an XY entry position and at least one subsequent move')
    plan = {'protocol':PROTOCOL, 'coordinate_frame':'gcode_bed',
            'entry':entry, 'operations':operations,
            'heater_targets':peaks, 'warnings':warnings,
            'requested_heater_targets':requested_peaks, 'heater_checks':checks,
            'ignored_heaters':sorted(ignored_heaters),
            'duration_s':sum(op.get('duration_s', 0) for op in operations),
            'limits':limits}
    plan['plan_hash'] = digest(plan)
    return plan
