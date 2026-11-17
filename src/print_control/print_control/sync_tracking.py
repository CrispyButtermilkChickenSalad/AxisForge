"""Measure physical nozzle tracking separately from network clock uncertainty."""
import math
import numpy as np
from print_control.motion_planner import cubic_joint_state


class PrintTracking:
    def __init__(self):
        self.samples = 0
        self.max_position_mm = self.max_z_mm = self.max_lag_ms = 0.
        self.lag_sum_ms = 0.
        self.lag_samples = 0

    def observe(self, motion, elapsed, actual_joints, nozzle_fk):
        if not 0 <= elapsed <= motion.times[-1]:
            return
        index = min(len(motion.times)-2, max(0, np.searchsorted(motion.times, elapsed)-1))
        dt = motion.times[index+1]-motion.times[index]
        expected, velocity, _ = cubic_joint_state(*motion.positions[index:index+2],
            *motion.velocities[index:index+2], dt, (elapsed-motion.times[index])/dt)
        actual, commanded = nozzle_fk(actual_joints), nozzle_fk(expected)
        error = commanded-actual
        if not np.all(np.isfinite(error)):
            raise RuntimeError('Nonfinite physical nozzle tracking observation')
        self.samples += 1
        self.max_position_mm = max(self.max_position_mm, float(np.linalg.norm(error)))
        self.max_z_mm = max(self.max_z_mm, abs(float(error[2])))
        # Tangential displacement / commanded speed is a lag estimate, not a
        # clock offset. It must never silently shift the extrusion schedule.
        speed = (nozzle_fk(expected+velocity*.001)-nozzle_fk(expected-velocity*.001))/.002
        squared = float(np.dot(speed, speed))
        if squared >= 4.:
            lag = float(np.dot(error, speed)/squared*1000.)
            if math.isfinite(lag):
                self.lag_samples += 1
                self.lag_sum_ms += lag
                self.max_lag_ms = max(self.max_lag_ms, abs(lag))

    def status(self):
        return {'samples': self.samples, 'max_position_error_mm': self.max_position_mm,
                'max_z_error_mm': self.max_z_mm, 'max_abs_estimated_lag_ms': self.max_lag_ms,
                'mean_estimated_lag_ms': self.lag_sum_ms/self.lag_samples if self.lag_samples else None,
                'lag_samples': self.lag_samples}
