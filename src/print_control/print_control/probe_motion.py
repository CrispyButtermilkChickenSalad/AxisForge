"""Fixed-speed vertical probing trajectories; planning never commands hardware.

Each descent has smooth acceleration/deceleration ramps and a constant-speed
middle. Joint or geometry limits reject the motion instead of silently retiming
it. Contact during the initial acceleration ramp can occur before cruise speed.
"""

import math

import numpy as np

from print_control.motion_planner import PlannedMotion


def _profile(distance, speed, acceleration):
    # Cubic velocity ramps have max acceleration 1.5*v/T, zero endpoint
    # acceleration, and distance v*T/2 per ramp.
    ramp = max(.1, 1.5 * speed / acceleration)
    cruise = (distance - speed * ramp) / speed
    if cruise <= 1e-9:
        raise ValueError(
            'Continuous homing path is too short to reach %g mm/s with a cruise '
            'interval; allow more than %.4f mm or reduce the requested speed' %
            (speed, speed * ramp))
    end = 2 * ramp + cruise
    if not math.isfinite(end) or ramp <= 0:
        raise ValueError('Continuous homing duration is outside numerical bounds')

    def state(t):
        t = min(end, max(0., float(t)))
        if t < ramp:
            u = t / ramp
            return speed * ramp * (u**3 - .5*u**4), speed * u*u*(3-2*u)
        if t <= ramp + cruise:
            return speed*ramp/2 + speed*(t-ramp), speed
        u = (t-ramp-cruise) / ramp
        position = speed*ramp/2 + speed*cruise + speed*ramp*(u-u**3+.5*u**4)
        return position, speed*(1-3*u*u+2*u**3)

    # Include ramp boundaries exactly; no individual sample crosses a change
    # between the ramp and constant-speed expressions.
    phases = ((0., ramp), (ramp, ramp+cruise), (ramp+cruise, end))
    # Slow precision probes need not allocate one waypoint every 10 ms over
    # minutes of travel. Bound spacing to 0.1 mm and sample time to 50 ms;
    # execution still checks sensor/robot health independently every ROS cycle.
    period = min(.05, .1/speed)
    counts = [max(2, math.ceil((finish-begin)/(period if index == 1 else .01)))
              for index, (begin, finish) in enumerate(phases)]
    if sum(counts) > 10000:
        raise ValueError('Continuous homing path exceeds 10000 time intervals')
    pieces = []
    for (begin, finish), intervals in zip(phases, counts):
        pieces.extend(np.linspace(begin, finish, intervals+1)[:-1])
    times = np.array([*pieces, end], dtype=float)
    return state, times


def plan_probe_descent(planner, start, end, seed, speed_mm_s,
                       acceleration_mm_s2=50., *, pump=lambda: None):
    """Return one checked trajectory with the requested Z cruise speed.

    Cartesian units are mm, mm/s and mm/s². The supplied planner uses the actual
    nozzle TCP and a fixed orientation. A path too short for acceleration,
    constant-speed cruise, and deceleration fails before an action is sent.
    ``pump`` services supervision during IK and validation; it must not move hardware.
    """
    for name, value, maximum in (('speed', speed_mm_s, 10.),
                                 ('acceleration', acceleration_mm_s2, 100.)):
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= maximum:
            raise ValueError('Continuous homing %s must be finite, positive and at most %g' %
                             (name, maximum))
    start, end, seed = (np.asarray(value, dtype=float) for value in (start, end, seed))
    if (start.shape != (3,) or end.shape != (3,) or seed.shape != (6,) or
            not all(np.all(np.isfinite(value)) for value in (start, end, seed))):
        raise ValueError('Continuous homing requires finite XYZ endpoints and six joints')
    if np.max(np.abs(start[:2]-end[:2])) > 1e-9 or end[2] >= start[2]:
        raise ValueError('Continuous homing must descend along Z only')
    distance = float(start[2]-end[2])
    state, times = _profile(distance, speed_mm_s, acceleration_mm_s2)
    rotation = planner.rotation.copy()
    planner.check_joints(seed)
    planner.check_pose(seed, start, start, rotation)
    positions, velocities = [], []
    current = seed.copy()
    for index, t in enumerate(times):
        pump()
        travelled, speed = state(t)
        point = start + np.array([0., 0., -travelled])
        if index:
            previous = current
            current = planner.solve(point, previous, rotation)
            if np.max(np.abs(np.rad2deg(current-previous))) > planner.config.max_joint_step_deg:
                raise ValueError('Continuous homing has a discontinuous IK solution')
        jacobian = planner.nozzle_jacobian(current)
        # A singular wrist can have an apparently solvable vertical translation
        # but no robust fixed-orientation differential solution. Fail closed.
        if (np.shape(jacobian) != (6, 6) or not np.all(np.isfinite(jacobian)) or
                np.linalg.cond(jacobian) > 1e8):
            raise ValueError('Continuous homing encounters a singular nozzle Jacobian')
        try:
            # Jacobian translation is metres/radian; joint velocity is rad/s.
            velocity = np.linalg.solve(jacobian, [0., 0., -speed/1000., 0., 0., 0.])
        except np.linalg.LinAlgError as error:
            raise ValueError('Continuous homing encounters a singular nozzle Jacobian') from error
        positions.append(current.copy())
        velocities.append(velocity)
    positions, velocities = np.asarray(positions), np.asarray(velocities)
    velocities[[0, -1]] = 0.
    motion = PlannedMotion('continuous Z homing', positions, velocities, times)

    def pose_at(index, fraction):
        t = times[index] + fraction * (times[index+1]-times[index])
        travelled, _ = state(t)
        return start + np.array([0., 0., -travelled]), rotation

    # Validate every interval, in bounded groups so ROS/sensor callbacks do not
    # become stale during an otherwise non-actuating planning operation.
    for first in range(0, len(times)-1, 8):
        pump()
        last = min(first+9, len(times))
        section = PlannedMotion(motion.name, positions[first:last], velocities[first:last], times[first:last])
        planner.check_motion(section,
                             pose_at=lambda index, fraction, offset=first: pose_at(offset+index, fraction),
                             timing_hint='continuous homing cannot maintain the requested speed; '
                                         'reduce homing speed or acceleration',
                             path_tolerance_mm=.005)
    pump()
    return motion
