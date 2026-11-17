"""Validate the bridge/worker clock-options wire contract without a Bridge dependency."""

import math

DEFAULTS = {'max_uncertainty_ms': 10., 'max_alignment_error_ms': 10.}
MAX_CLOCK_RATE = 1.001


def clock_rate(value):
    if type(value) not in (float,int) or not math.isfinite(value) or not .999 <= value <= MAX_CLOCK_RATE:
        raise ValueError('Print clock rate must be finite and within 1000 ppm of host time')
    return float(value)


def motion_in_host_time(motion, rate):
    """Convert units without changing the master's path or progress function."""
    from print_control.motion_planner import PlannedMotion
    rate=clock_rate(rate)
    if rate == 1.: return motion
    converted=PlannedMotion(motion.name, motion.positions, motion.velocities*rate, motion.times/rate)
    converted.master_motion=motion  # Reuse the prepared ROS geometry/message.
    return converted


def clock_options(value=None):
    if value is not None and (not isinstance(value, dict) or set(value)-set(DEFAULTS)):
        raise ValueError('Unknown clock timing options')
    selected = dict(DEFAULTS, **(value or {}))
    for key, number in selected.items():
        if type(number) not in (int, float) or not math.isfinite(number) or not 2. <= number <= 20.:
            raise ValueError('%s must be a finite number from 2 to 20 ms' % key)
        selected[key] = float(number)
    if selected['max_uncertainty_ms'] > selected['max_alignment_error_ms']:
        raise ValueError('Clock uncertainty limit must not exceed the alignment limit')
    return selected
