"""Convert a Klipper master schedule into joint samples without changing time."""

from dataclasses import fields
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from print_control.gcode_path import GCodeConfig, GCodePlanner
from print_control.bed_surface import BedSurface
from print_control.motion_planner import MotionConfig, PlannedMotion

PROTOCOL = 'axisforge.sync.v1'


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def configuration(path):
    """Fingerprint calibration inputs as well as the ROS parameter file."""
    path = Path(path)
    values = yaml.safe_load(path.read_text())['gcode_interpreter']['ros__parameters']
    settings = GCodeConfig(**{f.name:values[f.name] for f in fields(GCodeConfig)
                             if f.name in values})
    motion = MotionConfig(**{f.name:values[f.name] for f in fields(MotionConfig)
                             if f.name in values and f.name not in ('dry_run', 'nozzle_rpy_deg')})
    material = [path.read_bytes()]
    if settings.bed_probe_file:
        material.append(Path(settings.bed_probe_file).read_bytes())
    calibration = path.parent/'ur16e_calibration.yaml'
    if calibration.exists(): material.append(calibration.read_bytes())
    signature = hashlib.sha256(b'\0'.join(material)).hexdigest()
    return settings, motion, signature


def progress(profile, seconds):
    """Evaluate the protocol's normalized trapezoid, including phase boundaries."""
    t = min(profile['duration_s'], max(0., seconds))
    a, v, ramp = profile['acceleration'], profile['velocity'], profile['ramp_s']
    if t <= ramp: return .5*a*t*t
    if t <= ramp+profile['cruise_s']: return v*(t-.5*ramp)
    return 1.-.5*a*(profile['duration_s']-t)**2


def sample_times(profile):
    """Sample each phase without duplicate times from floating-point roundoff.

    A triangular profile can report a tiny positive cruise (around 1e-17 s).
    Keeping both sides creates effectively zero intervals in joint derivatives.
    Coalesce only sub-nanosecond intervals, below the ROS timestamp resolution;
    keep the original start/end and all resolvable phase boundaries.
    """
    duration = profile['duration_s']
    boundaries = (0., profile['ramp_s'], profile['ramp_s']+profile['cruise_s'], duration)
    candidates = sorted(t for left,right in zip(boundaries, boundaries[1:])
                        for t in np.linspace(left, right, max(2, math.ceil((right-left)/.02)+1)))
    if len(set(candidates)) < 5:
        candidates = sorted(candidates+list(np.linspace(0.,duration,5)))
    times = [0.]
    for t in candidates:
        if t-times[-1] >= 1e-9: times.append(float(t))
    if duration-times[-1] < 1e-9: times[-1] = duration
    else: times.append(duration)
    if len(times) < 2 or np.min(np.diff(times)) < 1e-9:
        raise ValueError('Master move is too short for ROS trajectory timestamps')
    return np.array(times)


def validate_plan(plan):
    """Reject forged/inconsistent timing before IK or command construction."""
    if not isinstance(plan, dict) or plan.get('protocol') != PROTOCOL:
        raise ValueError('Unsupported master schedule protocol')
    if plan.get('coordinate_frame') != 'gcode_bed':
        raise ValueError('Expected uncompensated G-code bed coordinates')
    content = {k:v for k,v in plan.items() if k != 'plan_hash'}
    if canonical_hash(content) != plan.get('plan_hash'):
        raise ValueError('Master schedule hash mismatch')

    def vector(value):
        if (not isinstance(value, list) or len(value) != 6 or
                any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
            raise ValueError('Invalid XYZABC pose')
        return np.asarray(value, dtype=float)

    last = vector(plan['entry'])
    count = 0
    operations = plan.get('operations')
    if not isinstance(operations, list) or len(operations) > 20000:
        raise ValueError('Invalid operation count')
    for op in operations:
        if op['kind'] != 'chunk': continue
        if op['index'] != count or not op['moves'] or len(op['moves']) > 20000:
            raise ValueError('Chunk sequence is not contiguous')
        elapsed = 0.
        for move in op['moves']:
            start, end = vector(move['start']), vector(move['end'])
            if not np.allclose(start, last, atol=1e-9, rtol=0):
                raise ValueError('Discontinuous source pose')
            p = move['profile']
            if any(type(p.get(k)) not in (int, float) or not math.isfinite(p[k])
                   for k in ('acceleration','velocity','ramp_s','cruise_s','duration_s')):
                raise ValueError('Invalid motion profile')
            if min(p['acceleration'],p['velocity'],p['ramp_s'],p['duration_s']) <= 0 or p['cruise_s'] < 0:
                raise ValueError('Nonpositive profile duration/limit')
            if (not math.isclose(p['velocity'], p['acceleration']*p['ramp_s'], rel_tol=1e-8) or
                not math.isclose(1., p['velocity']*(p['ramp_s']+p['cruise_s']), rel_tol=1e-8) or
                not math.isclose(p['duration_s'], 2*p['ramp_s']+p['cruise_s'], rel_tol=1e-8) or
                not math.isclose(move['offset_s'], elapsed, abs_tol=1e-8)):
                raise ValueError('Inconsistent phase times or progress')
            elapsed += p['duration_s']
            last = end
        if not math.isclose(elapsed, op['duration_s'], abs_tol=1e-8):
            raise ValueError('Chunk duration mismatch')
        count += 1
    if not count: raise ValueError('No motion chunks')


class TimedPath:
    """Preflight all chunks; AxisForge never stretches a Klipper time interval."""

    def __init__(self, config_path, plan, seed, report=lambda *_:None, cancelled=lambda:False):
        validate_plan(plan)
        settings, motion, self.config_hash = configuration(config_path)
        if settings.bed_probe_file:
            orientations = {tuple(plan['entry'][3:])}
            for op in plan['operations']:
                if op['kind'] == 'chunk':
                    orientations.update(tuple(m['end'][3:]) for m in op['moves'])
            for a, b, c in orientations:
                BedSurface.load(settings.bed_probe_file, motion.tcp_offset_mm,
                    (settings.nozzle_roll_deg+a, settings.nozzle_pitch_deg+b,
                     settings.wrist_angle+c), settings.bed_probe_extrapolation_mm)
        self.planner = planner = GCodePlanner(settings, motion)
        self.plan, self.chunks = plan, []
        self.entry = np.array(plan['entry'], dtype=float)
        planner.orientation = tuple(self.entry[3:])
        planner.rotation = planner.rotation_for(self.entry[3:])
        self.entry_joints = planner.solve(np.asarray(planner.transform(self.entry[:3])), np.array(seed))
        joints = self.entry_joints.copy()
        chunks = [op for op in plan['operations'] if op['kind']=='chunk']
        total_moves = sum(len(chunk['moves']) for chunk in chunks)
        done, points = 0, 0
        for chunk in chunks:
            positions, velocities, times = [], [], []
            for record in chunk['moves']:
                if cancelled(): raise RuntimeError('Planning cancelled')
                motion = self._move(record, joints, cancelled)
                offset = record['offset_s']
                first = 0 if not positions else 1
                positions.extend(motion.positions[first:])
                velocities.extend(motion.velocities[first:])
                times.extend(motion.times[first:]+offset)
                joints = motion.positions[-1]
                points += len(motion.times)
                if points > 1000000: raise ValueError('Schedule exceeds one million robot samples')
                done += 1
                report(done, total_moves)
            self.chunks.append(PlannedMotion('synchronized chunk %d' % chunk['index'],
                                            np.array(positions), np.array(velocities), np.array(times)))
        # Validate the entry path without executing it; rebuild from live joints at start.
        self.approach(seed)

    def _pose(self, record, t):
        u = progress(record['profile'], t)
        pose = np.array(record['start']) + u*(np.array(record['end'])-record['start'])
        return np.asarray(self.planner.transform(pose[:3])), self.planner.rotation_for(pose[3:])

    def _move(self, record, seed, cancelled):
        times = sample_times(record['profile'])
        positions = [np.asarray(seed)]
        start, rotation = self._pose(record, 0.)
        self.planner.check_pose(seed, start, start, rotation)
        for index, t in enumerate(times[1:]):
            if index % 32 == 0 and cancelled(): raise RuntimeError('Planning cancelled')
            point, rotation = self._pose(record, t)
            joints = self.planner.solve(point, positions[-1], rotation)
            if np.max(abs(np.rad2deg(joints-positions[-1]))) > self.planner.config.max_joint_step_deg:
                raise ValueError('Discontinuous IK branch')
            positions.append(joints)
        positions, times = np.array(positions), np.array(times)
        velocities = np.gradient(positions, times, axis=0)
        velocities[[0,-1]] = 0.
        motion = PlannedMotion('G-code line %s' % record['source_line'], positions, velocities, times)

        def pose_at(index, fraction):
            return self._pose(record, times[index]+fraction*(times[index+1]-times[index]))

        # check_motion includes cubic-interpolation extrema, joint velocity and acceleration.
        self.planner.check_motion(motion, pose_at=pose_at,
            timing_hint='reduce max_velocity/max_accel or angular limits in Klipper '
                        '[axisforge_sync] and rerun preflight; approach_duration_s '
                        'only affects the entry approach')
        return motion

    def approach(self, current_joints):
        """Untimed entry motion is allowed only before the master releases extrusion."""
        planner = self.planner
        planner.orientation = tuple(self.entry[3:])
        planner.rotation = planner.rotation_for(self.entry[3:])
        entry = np.array(planner.transform(self.entry[:3]))
        above = entry + [0,0,planner.config.clearance_mm]
        above_q = planner.solve(above, self.entry_joints)
        first = planner.retime(PlannedMotion('entry approach',
            np.array([current_joints, above_q]), np.zeros((2,6)),
            np.array([0.,planner.config.approach_duration_s])))
        planner.check_motion(first)
        descent = planner.sampled_line('entry descent', above, entry, above_q,
                                      planner.settings.max_travel_speed_mm_s)
        if np.max(abs(descent.positions[-1]-self.entry_joints)) > .001:
            raise ValueError('Entry approach changed the preflighted IK branch')
        return PlannedMotion('entry approach',
            np.concatenate((first.positions,descent.positions[1:])),
            np.concatenate((first.velocities,descent.velocities[1:])),
            np.concatenate((first.times,descent.times[1:]+first.times[-1])))
