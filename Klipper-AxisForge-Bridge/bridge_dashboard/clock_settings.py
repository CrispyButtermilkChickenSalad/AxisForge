"""Clock acceptance limits and illustrative commissioning guidance, in milliseconds."""

import math

DEFAULTS = {'max_uncertainty_ms': 10., 'max_alignment_error_ms': 10.}
MIN_MS, MAX_MS = 2., 20.
GUIDANCE_DISTANCE_MM = .10


def clock_options(value=None):
    if value is not None and (not isinstance(value, dict) or set(value)-set(DEFAULTS)):
        raise ValueError('Unknown clock timing options')
    selected = dict(DEFAULTS, **(value or {}))
    for key, number in selected.items():
        if type(number) not in (int, float) or not math.isfinite(number) or not MIN_MS <= number <= MAX_MS:
            raise ValueError('%s must be a finite number from 2 to 20 ms' % key)
        selected[key] = float(number)
    if selected['max_uncertainty_ms'] > selected['max_alignment_error_ms']:
        raise ValueError('Clock uncertainty limit must not exceed the alignment limit')
    return selected


def profiles():
    """Engineering starting points; not firmware ratings or validated speed limits."""
    return [dict(id=key, label=label, network=network,
                 options=clock_options({'max_uncertainty_ms':uncertainty,
                                        'max_alignment_error_ms':alignment}),
                 suggested_max_speed_mm_s=GUIDANCE_DISTANCE_MM*1000/alignment)
            for key, label, network, uncertainty, alignment in (
                ('wired_precision', 'Wired precision — 5 ms',
                 'Both hosts wired to the same switch or a direct Ethernet link; verify low jitter.', 5., 5.),
                ('standard', 'Standard — 10 ms (default)',
                 'Both hosts wired to the same switch. This preserves the previous limits.', 10., 10.),
                ('wifi_trial', 'Wi-Fi commissioning — 15 ms',
                 'Strong local Wi-Fi, power saving off, no hotspot/repeater hops if possible. Wired is preferred.', 10., 15.),
                ('slow_trial', 'Slow commissioning — 20 ms',
                 'Controlled slow trials on a measured local connection; investigate persistent jitter.', 10., 20.))]


def speed_guidance(plan, selected):
    """XYZ peak is normalized profile velocity times XYZ segment length."""
    alignment_s = selected['max_alignment_error_ms']/1000
    peak = max((math.dist(move['start'][:3], move['end'][:3])*move['profile']['velocity']
                for op in plan['operations'] if op['kind']=='chunk' for move in op['moves']), default=0.)
    displacement = peak*alignment_s
    return {'peak_xyz_speed_mm_s':peak, 'timing_displacement_mm':displacement,
            'guidance_distance_mm':GUIDANCE_DISTANCE_MM,
            'suggested_max_speed_mm_s':GUIDANCE_DISTANCE_MM/alignment_s,
            'exceeds_guidance':displacement > GUIDANCE_DISTANCE_MM+1e-9}
