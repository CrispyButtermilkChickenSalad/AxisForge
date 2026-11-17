"""Calibration-only thermal and load policies; never send heater or robot commands."""
import threading
import time

from print_control.load_cell_contact import ContactDetector, LoadCellOverload, finite, validate_printer


class CalibrationPolicy:
    """Keep hot calibration explicit and separate from cold bed probing."""
    def __init__(self, max_counts=120000., hot_target=None, *, clock=time.monotonic):
        if not finite(max_counts) or not 90000 <= max_counts <= 250000:
            raise ValueError('Calibration load limit must be 90000..250000 raw counts')
        if hot_target is not None and (not finite(hot_target) or not 50 <= hot_target <= 300):
            raise ValueError('Hot calibration target must be 50..300 C, or omitted for cold calibration')
        self.max_counts, self.hot_target, self.clock = float(max_counts), hot_target, clock
        self.lock = threading.RLock()
        self.temperature = self.target = None
        self.last_status = self.stable_since = None
        self.locked = False
        self.fault = None

    def validate_status(self, status, cell, options, require_calibration=True):
        # Keep readiness, ADC, ownership, and print interlocks. Apply thermal
        # rules here only for this explicitly selected calibration context.
        sensor = validate_printer(status, cell, options, require_calibration=False)
        now = self.clock()
        try:
            if self.hot_target is None:
                validate_printer(status, cell, options, require_calibration=True)
            else:
                heater = status.get('extruder', {})
                temperature, target = heater.get('temperature'), heater.get('target')
                if not finite(temperature) or not finite(target):
                    raise RuntimeError('Hot calibration requires valid nozzle temperature and target readings')
                if abs(target-self.hot_target) > 1e-6:
                    raise RuntimeError('Hot calibration target changed; set the selected %.1f C target before restarting' % self.hot_target)
                if status.get('heater_bed', {}).get('target', 0) != 0:
                    raise RuntimeError('Turn the bed heater off before load calibration')
                with self.lock:
                    in_band = abs(temperature-self.hot_target) <= 2.
                    gap = self.last_status is not None and now-self.last_status > 1.
                    if self.locked and (not in_band or gap):
                        raise RuntimeError('Hot calibration temperature left its stable +/-2 C band or feedback became stale')
                    if not in_band or gap:
                        self.stable_since = None
                    elif self.stable_since is None:
                        self.stable_since = now
            with self.lock:
                self.temperature = status.get('extruder', {}).get('temperature')
                self.target = status.get('extruder', {}).get('target')
                self.last_status = now
        except Exception as error:
            with self.lock:
                self.fault = self.fault or str(error)
            raise
        return sensor

    def status(self):
        with self.lock:
            stable = 0. if self.stable_since is None or self.last_status is None else max(0., self.last_status-self.stable_since)
            result = dict(hot_calibration=self.hot_target is not None, stable_for_s=stable,
                          required_stable_s=30., calibration_max_counts=self.max_counts)
            if finite(self.temperature):
                result['temperature_c'] = self.temperature
            if finite(self.target):
                result['target_c'] = self.target
            return result

    def ready(self):
        with self.lock:
            if self.fault:
                raise RuntimeError(self.fault)
            if self.last_status is None or self.clock()-self.last_status > 1.:
                if self.locked:
                    raise RuntimeError('Calibration temperature feedback is stale')
                return False
            return self.hot_target is None or (self.stable_since is not None and self.last_status-self.stable_since >= 30.)

    def require_ready(self):
        with self.lock:
            if not self.ready():
                raise RuntimeError('Wait for the selected calibration temperature to remain within +/-2 C for 30 seconds')
            self.locked = True

    def wait_ready(self, source, control, *, sleep=time.sleep):
        deadline = self.clock()+600.
        next_message = 0.
        while True:
            control.check()
            source.snapshot()  # Keep sample, sensor health and delivery checks active.
            if self.ready():
                self.require_ready()
                return
            if self.clock() >= deadline:
                raise RuntimeError('Calibration temperature did not stabilize within ten minutes')
            if self.clock() >= next_message:
                control.emit('waiting_temperature',
                    'Waiting for the manually selected %.1f C nozzle target to stay within +/-2 C for 30 seconds. '
                    'No heater command or robot motion is issued.' % self.hot_target, **self.status())
                next_message = self.clock()+1.
            sleep(.05)


class CalibrationDetector(ContactDetector):
    """Delegate only typed load-limit events to the supervised release handler."""
    def __init__(self, baseline, counts_per_gram, options, on_limit, on_sample=lambda load: None):
        super().__init__(baseline, counts_per_gram, options)
        self.on_limit = on_limit
        self.on_sample = on_sample

    def update(self, samples):
        # Scan all new samples even when a recoverable limit is held. Never tare
        # or skip a remaining batch simply because its first sample overloaded.
        for sample in samples:
            if sample.sequence <= self.last_sequence:
                continue
            self.on_sample(abs(self.load(sample.counts)))
            try:
                super().update([sample])
            except LoadCellOverload as error:
                self.on_limit(error)
        return self.triggered
