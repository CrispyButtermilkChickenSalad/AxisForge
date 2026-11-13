"""Validate the bridge/worker clock-options wire contract without a Bridge dependency."""

import math

DEFAULTS = {'max_uncertainty_ms': 10., 'max_alignment_error_ms': 10.}


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
