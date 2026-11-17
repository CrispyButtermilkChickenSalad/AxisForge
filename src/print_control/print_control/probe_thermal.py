"""Explicit temperature policy for bed probing; never sends heater commands."""
import threading
import time
from print_control.load_cell_contact import finite, validate_printer


def thermal_options(value=None):
    if value is None:
        return {'nozzle_target_c': 0., 'bed_target_c': 0., 'soak_s': 60.}
    if not isinstance(value, dict) or set(value) != {'nozzle_target_c','bed_target_c','soak_s'}:
        raise ValueError('Supply nozzle target, bed target and soak duration')
    for key, upper in (('nozzle_target_c',300.),('bed_target_c',150.)):
        target=value[key]
        minimum=50. if key=='nozzle_target_c' else 30.
        if not finite(target) or not (target==0 or minimum<=target<=upper):
            raise ValueError('%s must be 0 (off) or %g..%g C' % (key,minimum,upper))
    if not finite(value['soak_s']) or not 5<=value['soak_s']<=1800:
        raise ValueError('Temperature soak must be 5..1800 seconds')
    return {key:float(v) for key,v in value.items()}


class ProbeThermalPolicy:
    def __init__(self, options=None, *, clock=time.monotonic):
        self.options=thermal_options(options)
        self.targets={name:self.options[key] for name,key in
            (('extruder','nozzle_target_c'),('heater_bed','bed_target_c')) if self.options[key]>0}
        self.clock=clock
        self.lock=threading.RLock()
        self.stable_since=self.last_status=None
        self.locked=False
        self.fault=None
        self.readings={}

    def validate_status(self,status,cell,options,require_calibration=True):
        sensor=validate_printer(status,cell,options,require_calibration,check_temperature=False)
        now=self.clock()
        try:
            if not self.targets:
                validate_printer(status,cell,options,require_calibration)
            readings={}
            in_band=True
            for name in ('extruder','heater_bed'):
                heater=status.get(name)
                expected=self.targets.get(name,0.)
                # Moonraker represents an unknown queried object as {}, not
                # necessarily an omitted key. Losing an existing heater is a
                # fault even when its selected target is zero.
                if heater is None or heater=={}:
                    if name in self.readings:raise RuntimeError('Probing heater feedback disappeared: '+name)
                    if expected or name=='extruder':raise RuntimeError('Selected heater is not configured: '+name)
                    continue
                temperature,target=heater.get('temperature'),heater.get('target')
                if not finite(temperature) or not finite(target):
                    raise RuntimeError('Invalid probing temperature feedback: '+name)
                if abs(target-expected)>1e-6:
                    raise RuntimeError('Probing target changed or does not match: set %s to %g C in Klipper and restart the routine' % (name,expected))
                if expected:
                    in_band=in_band and abs(temperature-expected)<=2.
                elif name=='extruder' and temperature>options.max_nozzle_temp_c:
                    raise RuntimeError('Select a hot nozzle target, or wait for the nozzle to cool below %g C' % options.max_nozzle_temp_c)
                readings[name]={'temperature_c':temperature,'target_c':target}
            with self.lock:
                gap=self.last_status is not None and now-self.last_status>1.
                if self.locked and (gap or not in_band):
                    raise RuntimeError('Probing temperature left its stable +/-2 C band or feedback became stale')
                if not in_band or gap:self.stable_since=None
                elif self.stable_since is None:self.stable_since=now
                self.readings=readings
                self.last_status=now
        except Exception as error:
            with self.lock:self.fault=self.fault or str(error)
            raise
        return sensor

    def status(self):
        with self.lock:
            stable=0. if self.stable_since is None or self.last_status is None else max(0.,self.last_status-self.stable_since)
            return {'mode':'heated' if self.targets else 'cold', 'options':dict(self.options),
                    'temperatures':{k:dict(v) for k,v in self.readings.items()},
                    'stable_for_s':stable,'required_stable_s':self.options['soak_s'] if self.targets else 0.,
                    'locked':self.locked}

    def ready(self):
        with self.lock:
            if self.fault:raise RuntimeError(self.fault)
            if self.last_status is None or self.clock()-self.last_status>1.:return False
            return not self.targets or (self.stable_since is not None and
                self.last_status-self.stable_since>=self.options['soak_s'])

    def require_ready(self):
        with self.lock:
            if not self.ready():raise RuntimeError('Wait for stable probing temperatures and fresh heater feedback')
            self.locked=True

    def wait_ready(self,source,control,*,sleep=time.sleep):
        deadline=self.clock()+self.options['soak_s']+1200.
        next_message=0.
        while True:
            control.check();source.snapshot()
            if self.ready():self.require_ready();return
            if self.clock()>=deadline:raise RuntimeError('Probing temperatures did not stabilize before the heating/soak deadline')
            if self.clock()>=next_message:
                control.emit('waiting_temperature','Waiting for the selected Klipper heater targets to stabilize within +/-2 C and finish the soak. The robot has not moved.',thermal=self.status())
                next_message=self.clock()+1.
            sleep(.05)
