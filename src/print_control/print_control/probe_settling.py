"""Bounded stationary joint estimates for nozzle contact, without hardware I/O.

The position bounds apply to feedback stability, not absolute robot accuracy.
Block intervals assume approximately independent block medians; velocity,
timestamp, raw containment and drift checks are separate prerequisites.
"""
from dataclasses import dataclass
import math

import numpy as np


# Twenty-eight independent 50 ms blocks improve the estimate of noisy encoder
# feedback without widening any motion/position bound.
WINDOW_S = 1.4
BLOCK_COUNT = 28
LATEST_AGE_S = .02
MAX_GAP_S = .02
MAX_VELOCITY_RAD_S = 1e-4
POSITION_BOUND_MM = .01
JOINT_BOUND_RAD = .0001
RAW_POSITION_BOUND_MM = .1


class StationaryError(RuntimeError):
    """Fresh feedback cannot yet establish a bounded stationary pose."""
    def __init__(self, message, *, waitable=False):
        super().__init__(message)
        self.waitable = waitable


class StationaryMotionError(StationaryError):
    """Motion or a nonstationary pattern; never accept this observation window.

    Before the first settled anchor, the caller may observe another window
    within its original deadline. After an anchor this is a measurement fault.
    """


@dataclass(frozen=True)
class JointFeedback:
    received_at: float
    source_time: float
    joints: object
    velocity: object
    source_age_s: float


def _median_interval(values):
    """Conservative order-statistic interval for independent block medians."""
    n, k = len(values), 0
    for candidate in range(1, n // 2 + 1):
        tail = 2. * sum(math.comb(n, j) for j in range(candidate)) / 2. ** n
        if tail <= .01 / 9.:
            k = candidate
        else:
            break
    if not k:
        raise StationaryError('Too few independent feedback blocks')
    ordered = np.sort(values, axis=0)
    return ordered[k-1], ordered[-k]


def estimate_stationary(samples, now, nozzle_fk, *, stopped_at, terminal_confirmed):
    """Return one joint estimate and its FK pose from fresh post-stop feedback.

    ``received_at``/``now``/``stopped_at`` use the same monotonic clock. Source
    times must be strictly increasing. ``source_age_s`` is measured against the
    ROS clock when the callback receives the sample, so old source data cannot
    appear fresh merely by arriving after cancellation.
    """
    if terminal_confirmed is not True or not math.isfinite(stopped_at):
        raise StationaryError('Terminal robot cancellation must be confirmed first')
    if not math.isfinite(now):
        raise StationaryError('Invalid feedback observation time')
    samples = list(samples)
    if not samples:
        raise StationaryError('No fresh joint feedback after cancellation', waitable=True)
    try:
        t = np.asarray([s.received_at for s in samples], dtype=float)
        source = np.asarray([s.source_time for s in samples], dtype=float)
        q = np.asarray([s.joints for s in samples], dtype=float)
        v = np.asarray([s.velocity for s in samples], dtype=float)
        age = np.asarray([s.source_age_s for s in samples], dtype=float)
    except (TypeError, ValueError, AttributeError) as error:
        raise StationaryError('Invalid joint feedback sample') from error
    if (q.shape != (len(samples), 6) or v.shape != q.shape or
            not all(np.all(np.isfinite(a)) for a in (t, source, q, v, age))):
        raise StationaryError('Six finite joint positions and velocities are required')
    if np.any(np.diff(t) <= 0) or np.any(np.diff(source) <= 0):
        raise StationaryError('Duplicate or backward joint feedback timestamps')
    if not 0 <= now-t[-1] <= LATEST_AGE_S or not 0 <= now-(t[-1]-age[-1]) <= LATEST_AGE_S:
        raise StationaryError('Latest stationary joint feedback is stale')
    start = t[-1]-WINDOW_S
    chosen = (t >= start) & (t-age > stopped_at)
    t, source, q, v, age = (a[chosen] for a in (t, source, q, v, age))
    if len(t) < math.ceil(200*WINDOW_S) or t[-1]-t[0] < WINDOW_S-.01:
        raise StationaryError('Waiting for %g seconds of fresh post-stop joint feedback' % WINDOW_S, waitable=True)
    if np.max(np.diff(t)) > MAX_GAP_S or np.max(np.diff(source)) > MAX_GAP_S:
        raise StationaryError('Stationary joint feedback contains a delivery gap', waitable=True)
    if np.any(age < 0):
        raise StationaryError('Stationary joint source feedback is in the future')
    if np.any(age > LATEST_AGE_S):
        # Reject the entire window, including this delayed observation. Before
        # pose acceptance the caller may collect a wholly fresh replacement
        # within its original deadline; no old sample is accepted or omitted.
        raise StationaryError('Stationary joint source feedback is stale', waitable=True)
    if np.ptp(t-source) > MAX_GAP_S:
        raise StationaryError('Joint feedback delivery stalled or its clock changed')
    maximum_velocity = float(np.max(np.abs(v)))
    if maximum_velocity > MAX_VELOCITY_RAD_S:
        raise StationaryMotionError(
            'Joint motion in the observation window (peak %.6g rad/s; latest %.6g rad/s; limit %.6g rad/s)' %
            (maximum_velocity, float(np.max(np.abs(v[-1]))), MAX_VELOCITY_RAD_S))
    xyz = np.asarray([nozzle_fk(joints) for joints in q], dtype=float)
    if xyz.shape != (len(q), 3) or not np.all(np.isfinite(xyz)):
        raise StationaryError('Invalid nozzle forward-kinematics feedback')
    data = np.column_stack((xyz, q))
    centre = np.median(data, axis=0)
    position_excursion = float(np.max(np.linalg.norm(xyz-centre[:3], axis=1)))
    joint_excursion = float(np.max(np.abs(q-centre[3:])))
    if position_excursion > RAW_POSITION_BOUND_MM or joint_excursion > JOINT_BOUND_RAD:
        raise StationaryMotionError('Raw stationary pose excursion exceeds 0.1 mm or 100 microradians')
    # A fixed cap above prevents broad vibration from inflating this adaptive
    # outlier screen. No outliers are removed or silently replaced.
    mad_sigma = 1.4826*np.median(np.abs(data-centre), axis=0)
    spike_limit = np.maximum(6.*mad_sigma, np.array([POSITION_BOUND_MM]*3+[JOINT_BOUND_RAD]*6))
    if np.any(np.abs(data-centre) > spike_limit):
        raise StationaryError('Stationary feedback contains a raw position spike')

    quarters, blocks = [], []
    for count, destination in ((4, quarters), (BLOCK_COUNT, blocks)):
        for index in range(count):
            left, right = start+WINDOW_S*index/count, start+WINDOW_S*(index+1)/count
            take = (t >= left) & (t <= right if index == count-1 else t < right)
            if np.count_nonzero(take) < (25 if count == 4 else 8):
                raise StationaryError('Too few distinct observations in each stationary window', waitable=True)
            destination.append(np.median(data[take], axis=0))
    quarters, blocks = np.asarray(quarters), np.asarray(blocks)
    differences = quarters[:, None, :] - quarters[None, :, :]
    position_drift = float(np.max(np.linalg.norm(differences[:, :, :3], axis=2)))
    joint_drift = float(np.max(np.abs(differences[:, :, 3:])))
    if position_drift > POSITION_BOUND_MM or joint_drift > JOINT_BOUND_RAD:
        raise StationaryMotionError(
            'Stationary pose drift exceeds 0.01 mm or 100 microradians '
            '(measured %.6f mm; %.3f microradians)' % (position_drift, joint_drift*1e6))

    # Screen resolvable coherent oscillation using uniform source-time samples.
    # This is an additional diagnostic, not proof that physical vibration is absent.
    uniform_t = np.linspace(source[0], source[-1], len(source))
    uniform = np.column_stack([np.interp(uniform_t, source, data[:, i]) for i in range(data.shape[1])])
    trend = np.column_stack((uniform_t-uniform_t.mean(), np.ones(len(uniform_t))))
    residual = uniform-trend@np.linalg.lstsq(trend, uniform, rcond=None)[0]
    power = np.abs(np.fft.rfft(residual, axis=0))**2
    power[0] = 0.
    total, peak = power.sum(axis=0), np.zeros(data.shape[1])
    for index in range(1, len(power)):
        peak = np.maximum(peak, power[max(1,index-1):index+2].sum(axis=0)/np.maximum(total, 1e-30))
    # Ignore numerical roundoff for exactly constant poses.
    meaningful = np.ptp(uniform, axis=0) > np.array([1e-6]*3+[1e-9]*6)
    if np.any(meaningful & (peak > .16)):
        raise StationaryMotionError('Coherent oscillation or correlated joint feedback prevents settling')

    low, high = _median_interval(blocks)
    joints = np.median(blocks[:, 3:], axis=0)
    nozzle = np.asarray(nozzle_fk(joints), dtype=float)
    if nozzle.shape != (3,) or not np.all(np.isfinite(nozzle)):
        raise StationaryError('Invalid nozzle forward-kinematics estimate')
    radius = np.maximum(np.abs(np.r_[nozzle, joints]-low), np.abs(high-np.r_[nozzle, joints]))
    position_radius, joint_radius = float(np.linalg.norm(radius[:3])), float(np.max(radius[3:]))
    if position_radius > POSITION_BOUND_MM or joint_radius > JOINT_BOUND_RAD:
        raise StationaryError('Stationary pose estimate has not converged within 0.01 mm / 100 microradians '
                             '(radius %.6f mm; %.3f microradians)' %
                             (position_radius, joint_radius*1e6), waitable=True)
    return dict(nozzle=nozzle, joints=joints, sample_count=len(t),
                observed_start=float(t[0]), observed_end=float(t[-1]),
                duration_s=float(t[-1]-t[0]), position_radius_mm=position_radius,
                joint_radius_rad=joint_radius, raw_position_excursion_mm=position_excursion)
