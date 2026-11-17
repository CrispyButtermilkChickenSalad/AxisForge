"""Checked finishing moves after a successful print; never used after abort."""
from dataclasses import replace
from pathlib import Path

import numpy as np
from spatialmath import SE3
import yaml

from print_control.gcode_path import GCodePlanner
from print_control.probe_alignment import rotation_rpy, rotation_error_deg


def lift_motion(settings, config, current_joints, pump=lambda: None):
    """Lift from the last measured nozzle position while holding XY and rotation."""
    settings.validate()
    if not settings.post_print_lift_mm:
        return []
    pump()
    planner = GCodePlanner(settings, config)
    joints = np.asarray(current_joints).copy()
    start = planner.nozzle_position(joints)
    rotation = planner.kinematics.eval(joints)[:3, :3]
    rpy = rotation_rpy(rotation)
    planner.config = replace(config, nozzle_rpy_deg=tuple(rpy))
    planner.settings = replace(settings, nozzle_roll_deg=rpy[0], nozzle_pitch_deg=rpy[1], wrist_angle=rpy[2])
    planner.rotation, planner.orientation = rotation, (0., 0., 0.)
    end = start + np.array([0., 0., settings.post_print_lift_mm])
    motion = planner.sampled_line('post-print vertical lift', start, end, joints,
        min(10., settings.max_travel_speed_mm_s), pump=pump)
    if (np.linalg.norm(planner.nozzle_position(motion.positions[-1])-end) > .02 or
            rotation_error_deg(planner.kinematics.eval(motion.positions[-1])[:3, :3], rotation) > .05):
        raise ValueError('Post-print trajectory does not reach the vertical lift target')
    return [motion]


def saved_home(settings):
    if not settings.return_to_homing_pose:
        return None
    if not settings.bed_probe_file:
        raise ValueError('Return home requires a completed bed-homing mesh')
    data = yaml.safe_load(Path(settings.bed_probe_file).read_text())
    homing = data.get('homing') or {}
    if data.get('complete') is not True or not homing.get('returned_to_start'):
        raise ValueError('Return home requires a successfully completed bed-homing routine')
    xyz = np.asarray(homing.get('starting_nozzle_base_mm'), dtype=float)
    rpy = np.asarray(homing.get('starting_nozzle_rpy_deg', data.get('nozzle_rpy_deg')), dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.all(np.isfinite([xyz, rpy])):
        raise ValueError('Saved bed-homing pose is missing or invalid; home and probe again')
    return {'position_mm': xyz.tolist(), 'rpy_deg': rpy.tolist(), 'mesh_stamp': data.get('stamp')}


def return_motions(settings, config, home, current_joints, print_top_mm, pump=lambda: None):
    """Lift, travel above Home, restore its orientation, then lower to Home.

    Mesh/model/TCP validity is checked by GCodePlanner. Controller installation
    safety queries are performed by the worker before execution. Never used on
    an abort, where the path and printed-part state are uncertain.
    """
    if home is None:
        return []
    target = np.array(home['position_mm'])
    clearance = config.clearance_mm
    if clearance <= 0 or target[2] < print_top_mm + clearance:
        raise ValueError('Saved homing pose is too low to clear this print; home above the part or disable return_to_homing_pose')
    planner = GCodePlanner(settings, config)
    joints = np.array(current_joints).copy()
    xyz = planner.nozzle_position(joints)
    high = max(xyz[2], target[2], print_top_mm + clearance) + clearance
    motions = []

    def move(name, end, rpy=None):
        nonlocal joints, xyz
        pump()
        actual = planner.kinematics.eval(joints)[:3, :3]
        actual_rpy = rotation_rpy(actual)
        planner.config = replace(planner.config, nozzle_rpy_deg=tuple(actual_rpy))
        planner.settings = replace(settings, nozzle_roll_deg=actual_rpy[0],
                                   nozzle_pitch_deg=actual_rpy[1], wrist_angle=actual_rpy[2])
        planner.rotation, planner.orientation = actual, (0., 0., 0.)
        delta = None if rpy is None else (np.array(rpy)-actual_rpy+180.) % 360.-180.
        if np.linalg.norm(end-xyz) < 1e-5 and (delta is None or np.max(abs(delta)) < 1e-5):
            return
        motion = planner.sampled_line(name, xyz, end, joints,
            min(20., settings.max_travel_speed_mm_s), end_orientation_deg=delta,
            minimum_duration_s=3. if rpy is not None else 0., pump=pump)
        motions.append(motion)
        joints = motion.positions[-1].copy()
        xyz = planner.nozzle_position(joints)

    move('print return clearance lift', np.array([*xyz[:2], high]))
    move('print return above saved Home', np.array([*target[:2], high]))
    move('print return Home orientation', xyz.copy(), home['rpy_deg'])
    move('print return saved Home Z', target)
    if (np.linalg.norm(xyz-target) > .02 or
            rotation_error_deg(planner.kinematics.eval(joints)[:3, :3],
                               SE3.RPY(home['rpy_deg'], unit='deg', order='zyx').R) > .05):
        raise ValueError('Return trajectory does not reach the saved homing pose')
    return motions
