"""Measure the Klipper-host clock rate against this PC, without wall-clock time."""

from collections import deque
from dataclasses import dataclass
import math
from statistics import median


@dataclass(frozen=True)
class HostRate:
    rate: float
    span_s: float
    error_ppm: float
    residual_ms: float
    rejected_samples: int = 0


class HostRateEstimator:
    """Fit low-RTT observations over a recent, bounded window.

    The MCU/RAW rate alone cannot describe seconds on a different computer.
    Measure RAW/PC here, then multiply by the MCU/RAW rate in each clock reply.
    Only rate is fitted: current phase is always measured afresh, so fitting
    cannot conceal a phase jump or retime an already queued trajectory.
    """
    MIN_SPAN = 60.
    WINDOW = 180.
    BIN = 2.

    def __init__(self):
        self.points = deque()
        self.epoch = None
        self.last = None

    def observe(self, sent, received, response):
        raw = response.get('eventtime')
        epoch = response.get('clock_epoch')
        mcu_rate = response.get('mcu_raw_clock_rate')
        if (not isinstance(epoch, str) or not epoch or
                any(type(v) not in (int, float) or not math.isfinite(v)
                    for v in (sent, received, raw, mcu_rate)) or
                received < sent or not .999 <= mcu_rate <= 1.001):
            raise ValueError('Update the Klipper module for inter-host clock measurement')
        midpoint = (sent+received)/2
        if (epoch != self.epoch or
                self.last is not None and (midpoint-self.last[0] > 10. or
                                           midpoint <= self.last[0] or raw <= self.last[1])):
            self.points.clear()
        self.epoch, self.last = epoch, (midpoint, raw)
        while self.points and midpoint-self.points[0][0] > self.WINDOW:
            self.points.popleft()
        rtt = received-sent
        if rtt > .020:
            return
        point = (midpoint, raw, rtt)
        if self.points and int(midpoint/self.BIN) == int(self.points[-1][0]/self.BIN):
            if rtt < self.points[-1][2]:
                self.points[-1] = point
        else:
            self.points.append(point)

    def estimate(self, now):
        if not self.points or now-self.points[-1][0] > 5.:
            raise ValueError('Inter-host clock measurements are unavailable or stale')
        first, last = self.points[0], self.points[-1]
        span = last[0]-first[0]
        if span < self.MIN_SPAN or len(self.points) < 25:
            raise ValueError('Inter-host clock calibration warming up (%d/%d s)' %
                             (max(0., span), self.MIN_SPAN))
        # Remove unit slope before regression to retain microsecond precision.
        xy = [(p[0]-first[0], (p[1]-first[1])-(p[0]-first[0])) for p in self.points]
        # A single asymmetric RPC can have a short RTT yet a biased midpoint.
        # Use long-baseline pair slopes to identify isolated rate-fit outliers.
        # Phase validation still uses ALL exchanges in the current burst.
        seed = median((yb-ya)/(xb-xa) for i,(xa,ya) in enumerate(xy)
                      for xb,yb in xy[i+1:] if xb-xa >= span*.4)
        centre = median(y-seed*x for x,y in xy)
        deviations = [abs(y-seed*x-centre) for x,y in xy]
        cutoff = max(.0003, 6.*median(deviations))
        selected = [p for p,d in zip(xy,deviations) if d <= cutoff]
        rejected = len(xy)-len(selected)
        if len(selected) < .8*len(xy) or selected[-1][0]-selected[0][0] < self.MIN_SPAN:
            raise ValueError('Inter-host clock rate needs more consistent network measurements')
        xy = selected
        count = len(xy)
        mx = sum(x for x, _ in xy)/count
        my = sum(y for _, y in xy)/count
        sxx = sum((x-mx)**2 for x, _ in xy)
        slope = sum((x-mx)*(y-my) for x, y in xy)/sxx
        residuals = [y-my-slope*(x-mx) for x, y in xy]
        # Fit quality is not a guarantee about future drift. The independent
        # running phase/RTT bound remains authoritative.
        error_ppm = 3.*math.sqrt(sum(r*r for r in residuals)/(count-2)/sxx)*1e6
        residual_ms = max(abs(r) for r in residuals)*1000.
        if (not .999 <= 1.+slope <= 1.001 or error_ppm > 5. or residual_ms > 1.):
            raise ValueError('Inter-host clock rate is unstable (fit error %.2f ppm; '
                             'residual %.2f ms); check the host clocks/network' %
                             (error_ppm, residual_ms))
        return HostRate(1.+slope, span, error_ppm, residual_ms, rejected)
