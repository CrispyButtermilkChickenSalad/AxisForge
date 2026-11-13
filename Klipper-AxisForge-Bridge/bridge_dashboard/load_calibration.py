"""Scale calibration uses the same watchdog, stop handling and ownership as probing."""
import math
import time

from bridge_dashboard.bed_probe import BedProbeRunner, BedProbeError, TERMINAL


class LoadCalibrationRunner(BedProbeRunner):
    executable = 'alps_load_calibration'
    routine_name = 'ALPS scale calibration'
    simulation_message = 'Scale calibration simulation complete. No hardware or calibration was changed.'
    waiting = {f'waiting_{s}':s for s in ('home', 'scale', 'tare')}
    states = set(waiting) | TERMINAL | {'starting', 'finding_scale', 'retracting',
                                      'adjusting_load', 'stepping', 'step_rejected'}

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._snapshot.update(message='Use a weighing scale to obtain two ALPS calibration loads.', total_points=0)
        self._action_received_at = 0.

    def _apply_event(self, event):
        super()._apply_event(event)
        token = event.get('action_token') if event['state'] == 'adjusting_load' else None
        if token is not None and (not isinstance(token, str) or not 16 <= len(token) <= 128):
            raise BedProbeError('Invalid scale calibration action token')
        if token in self._consumed_tokens: token = None
        self._snapshot['action_token'] = None if self._aborting else token
        self._action_received_at = time.monotonic()
        for key in ('contact_z_mm','lower_z_mm','upper_z_mm','raw_counts','delta_counts'):
            if key in event:
                value = event[key]
                if type(value) not in (int,float) or not math.isfinite(value):
                    raise BedProbeError('Invalid scale calibration feedback')
                self._snapshot[key] = value

    async def action(self, command, token, delta_mm=None):
        async with self._lock:
            if (not self._active or self._aborting or self._snapshot['state'] != 'adjusting_load' or
                    token != self._snapshot.get('action_token') or token in self._consumed_tokens or
                    time.monotonic()-self._action_received_at >= 9.):
                raise BedProbeError('This step is stale or motion is busy; refresh calibration status')
            if command not in ('step','finish'):
                raise BedProbeError('Unknown calibration action')
            if command == 'step' and (type(delta_mm) not in (int,float) or abs(delta_mm) not in (.001,.01,.05,.5)):
                raise BedProbeError('Choose a supported baby-step size')
            self._consumed_tokens.add(token)
            self._snapshot['action_token'] = None
            try:
                await self._send(dict(command=command, token=token, **({'delta_mm':delta_mm} if command=='step' else {})))
            except BedProbeError as error:
                self._schedule_failure_stop(str(error))
                raise
            return self.status()
