"""Single-use, expiring operator actions for the scale-assisted ALPS routine."""
import math
import secrets

from print_control.probe_control import ProbeControl


STEP_SIZES = (.001, .01, .05, .1, .5, 1., 10.)


class LoadCalibrationControl(ProbeControl):
    confirmation_stages = ('home', 'scale', 'tare')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.action_token = None
        self.action_deadline = 0.
        self.action = None

    def ingest(self, command):
        if not isinstance(command, dict) or command.get('command') not in ('step', 'finish'):
            return super().ingest(command)
        with self.lock:
            self.check()
            if (self.reason or self.action_token is None or self.action is not None or
                    self.clock() >= self.action_deadline or command.get('token') != self.action_token):
                return False
            if command['command'] == 'step':
                delta = command.get('delta_mm')
                if type(delta) not in (int, float) or not math.isfinite(delta) or abs(delta) not in STEP_SIZES:
                    raise ValueError('Invalid baby-step distance')
            self.action = dict(command)
            self.action_token = None
            return True

    def wait_action(self, pump, values):
        idle_deadline = self.clock()+self.confirmation_timeout_s
        try:
            while True:
                self.check()
                if self.clock() >= idle_deadline:
                    raise RuntimeError('Scale calibration inactive for five minutes; stopped without moving')
                with self.lock:
                    if self.action is not None:
                        result, self.action = self.action, None
                        return result
                    if self.action_token is None or self.clock() >= self.action_deadline:
                        self.action_token = secrets.token_hex(16)
                        self.action_deadline = self.clock()+10.
                        self.emit('adjusting_load', 'Use one Z step at a time and read the weighing scale. '
                                  'Record two distinct stable loads in ALPS Logger → Calibration.',
                                  action_token=self.action_token, **values())
                pump()
        finally:
            with self.lock:
                self.action_token, self.action = None, None
