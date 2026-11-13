"""ALPS nozzle-contact bed probing using measured robot joint feedback."""
from dataclasses import asdict, replace
from datetime import datetime
import os
from pathlib import Path
import shutil
import statistics
import time

from action_msgs.msg import GoalStatus
import numpy as np
import rclpy
import yaml

from print_control.bed_probe import BedProbe, rounded
from print_control.load_cell_contact import (
    AlpsStream, ContactDetected, ContactDetector, ContactOptions, ForceSample,
    repeat_contact, stable_baseline,
)

HELP = """
ALPS bed probe: clean, bare, cool nozzle; use a configured contact threshold.
  space / t    probe this point (repeated contacts, then lift)
  a            probe the complete 3x3 grid, then finish
  n / p        next / previous point at the hover height
  h            lift       q  lift and quit       Ctrl-C  cancel and quit
Partial results go to *.pending.yaml; only a complete grid replaces the bed map.
"""


def sensor_setup(args, options=None):
    if options is None:
        path = Path(args.sensor_config) if args.sensor_config else Path(args.config).with_name('load_cell_bed_probe.yaml')
        data = yaml.safe_load(path.read_text())
        if not isinstance(data, dict) or set(data)-set(ContactOptions.__dataclass_fields__):
            raise ValueError('Invalid load-cell settings in '+str(path))
        options = ContactOptions(**data)
    if args.dry_run: return options, None
    # Read only these settings, without executing shell syntax or exposing secrets.
    env = {}
    if Path('.env').is_file():
        for line in Path('.env').read_text().splitlines():
            name, sep, value = line.partition('=')
            if sep and name.strip() in ('MOONRAKER_URL', 'MOONRAKER_API_KEY', 'ALPS_BRIDGE_URL'):
                env[name.strip()] = value.strip().strip('\"\'')
    env.update({key: os.environ[key] for key in
                ('MOONRAKER_URL', 'MOONRAKER_API_KEY', 'ALPS_BRIDGE_URL') if key in os.environ})
    url = args.moonraker_url or env.get('MOONRAKER_URL')
    if not url: raise ValueError('Set MOONRAKER_URL in .env or supply --moonraker-url')
    source = AlpsStream(url, args.alps_url or env.get('ALPS_BRIDGE_URL'), args.load_cell,
                        env.get('MOONRAKER_API_KEY'), options, not args.sensor_check)
    try:
        source.start()
        samples = source.wait_window(source.clock())
        if not args.sensor_check: stable_baseline(samples, source.sensor, options)
    except Exception:
        source.close()
        raise
    return options, source


def sensor_check(source):
    samples = source.wait_window(source.clock())
    counts = [s.counts for s in samples]
    rate = (len(samples)-1)/(samples[-1].source_time-samples[0].source_time)
    print('ALPS stream OK: %s, %.1f samples/s, raw median %.1f, range %.1f counts' %
          (source.cell, rate, statistics.median(counts), max(counts)-min(counts)))
    print('Calibrated: %s; robot was not connected or moved.' % source.sensor.get('is_calibrated', False))
    if not source.sensor.get('is_calibrated'):
        print('Grams mode needs known-load calibration. Guided bridge homing can use raw ADC counts.')


class SimulatedAlps:
    """Synthetic elastic bed for dry runs only; no network or hardware access."""
    sensor = {'is_calibrated': True, 'counts_per_gram': 100., 'reference_tare_counts': 0.}
    clock = staticmethod(time.monotonic)

    def __init__(self, probe):
        self.probe, self.sequence = probe, 0
        self.bed_z = probe.settings.z_offset
        self.sensor = dict(type(self).sensor)
        if probe.contact_options.trigger_mode == 'raw':
            self.sensor.update(is_calibrated=False, counts_per_gram=None, reference_tare_counts=None)

    def snapshot(self):
        point = None if self.probe.target is None else self.probe.nozzle_now()[0]
        force = 0. if point is None else float(max(0., self.bed_z-point[2])*1000.)
        now = self.clock()
        result = [ForceSample(self.sequence+i+1, now-.12+i*.002, now-.12+i*.002, force*100.)
                  for i in range(61)]
        self.sequence += len(result)
        return result

    def wait_window(self, *args, **kwargs): return self.snapshot()


class LoadCellBedProbe(BedProbe):
    help_text = HELP

    def __init__(self, settings, motion_config, args, source, options):
        self.source, self.contact_options = source, options
        self.detector = None
        self.seeking = False
        self.promoted = False
        probe_motion = replace(motion_config,
                               feedback_timeout_s=min(.25, motion_config.feedback_timeout_s),
                               orientation_tolerance_deg=min(.2, motion_config.orientation_tolerance_deg),
                               path_tolerance_mm=min(options.step_mm/4., motion_config.path_tolerance_mm))
        super().__init__(settings, probe_motion, args)
        if self.config.dry_run:
            self.source = SimulatedAlps(self)
        else:
            try:
                # Pair the robot lock with a fresh Klipper idle check. The bridge
                # likewise checks our lock after acquiring Klipper, before heating.
                self.source.refresh_status()
            except Exception:
                self.destroy_node()
                raise
        sensor = self.source.sensor
        self.reference_counts = sensor.get('reference_tare_counts') if options.trigger_mode == 'grams' else None
        # Guard approach/travel too; only an unloaded baseline can replace this reference.
        if self.reference_counts is not None:
            self.detector = ContactDetector(self.reference_counts, sensor.get('counts_per_gram'), options)
            self.detector.last_sequence = self.source.snapshot()[-1].sequence

    def check_interrupted(self):
        super().check_interrupted()
        if self.source is not None:
            samples = self.source.snapshot()
            if self.detector is not None and self.detector.update(samples) and self.seeking:
                raise ContactDetected('Nozzle contact detected')

    def pump(self):
        self.check_interrupted()
        if not self.config.dry_run: rclpy.spin_once(self, timeout_sec=.01)

    def window(self):
        return self.source.wait_window(self.source.clock(), pump=self.pump)

    def set_baseline(self):
        samples = self.window()
        sensor = self.source.sensor
        baseline = stable_baseline(samples, sensor, self.contact_options, getattr(self, 'reference_counts', None))
        if self.contact_options.trigger_mode == 'raw' and getattr(self, 'reference_counts', None) is None:
            self.reference_counts = baseline
        self.detector = ContactDetector(baseline, sensor.get('counts_per_gram'), self.contact_options)
        self.detector.last_sequence = samples[-1].sequence
        return baseline

    def cancel_contact(self):
        """Require a terminal action result before any measurement or retraction."""
        self.seeking = False
        if not self.config.dry_run:
            self.stop()
            if self.goal_future is not None and not self.goal_future.done():
                raise RuntimeError('Contact stop unconfirmed: trajectory acceptance is unresolved; use pendant stop')
            if self.goal_handle is not None and self.goal_handle.accepted:
                if self.result_future is None or not self.result_future.done():
                    raise RuntimeError('Contact stop unconfirmed; use pendant stop')
                result = self.result_future.result()
                if (result.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED) or
                        (result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code != 0)):
                    raise RuntimeError('Controller fault during contact stop; point rejected')
            self.aborting = False
            self.goal_future = self.goal_handle = self.result_future = self.motion_token = None
        nozzle, joints = self.nozzle_now()
        self.target, self.seed = np.asarray(nozzle), joints.copy()

    def touch_once(self, *, floor_z=None, speed_mm_s=None, step_mm=None):
        options = self.contact_options
        floor = self.bed_point(self.points[self.index])[2]-self.args.floor_mm if floor_z is None else floor_z
        speed = options.speed_mm_s if speed_mm_s is None else speed_mm_s
        step = options.step_mm if step_mm is None else step_mm
        self.seeking = True
        try:
            while True:
                self.check_interrupted()
                if self.target[2] <= floor+1e-8:
                    raise RuntimeError('No contact before the configured probe floor; check bed placement/z_offset')
                end = self.target.copy()
                end[2] = max(floor, end[2]-step)
                self.line_to(end, 'contact step', speed)
                # Wait for samples collected after this step, never reuse a previous plateau.
                samples = self.window()
                if self.detector.update(samples): raise ContactDetected('Nozzle contact detected')
        except ContactDetected:
            self.cancel_contact()
        finally:
            self.seeking = False
        samples = self.window()
        self.detector.update(samples)  # The overload guard remains active while stopped.
        load = statistics.median(abs(self.detector.load(s.counts)) for s in samples)
        if load < options.threshold*.7:
            raise RuntimeError('Contact did not persist after stopping; point rejected')
        nozzle, joints = self.nozzle_now()
        return {'nozzle': nozzle, 'joints': joints, 'load': load,
                'peak_load': self.detector.peak_load, 'tare_counts': self.detector.baseline}

    def retract_contact(self, *, distance_mm=None, speed_mm_s=None):
        end = self.target.copy()
        end[2] += self.contact_options.retract_mm if distance_mm is None else distance_mm
        self.line_to(end, 'contact retract', self.contact_options.speed_mm_s if speed_mm_s is None else speed_mm_s)
        samples = self.window()
        self.detector.update(samples)
        if max(abs(self.detector.load(s.counts)) for s in samples) >= self.contact_options.threshold*.25:
            raise RuntimeError('Nozzle load did not clear after retracting; point rejected')

    def probe_current(self):
        """Repeat, validate and lift before recording a point."""
        self.lift()
        readings = []
        try:
            for _ in range(self.contact_options.samples):
                self.set_baseline()
                readings.append(self.touch_once())
                self.retract_contact()
            selected, spread = repeat_contact(readings, self.contact_options.repeatability_mm)
            self.lift()
        except ValueError as error:
            # Failure may follow physical contact: terminate instead of permitting jog keys.
            raise RuntimeError(str(error)) from error
        nozzle, joints = selected['nozzle'], selected['joints']
        load_key = 'delta_counts' if self.contact_options.trigger_mode == 'raw' else 'force_g'
        self.records[self.index] = {
            'point': self.index+1, 'grid_xy': rounded(self.points[self.index], 3),
            'gcode_xy': rounded(self.gcode_xy(nozzle)), 'nozzle_base_mm': rounded(nozzle),
            'bed_z_mm': round(float(nozzle[2])+self.contact_options.contact_offset_mm, 4),
            'joints_rad': rounded(joints, 6), 'contact_'+load_key: round(selected['load'], 2),
            'repeatability_mm': round(spread, 5),
            'contacts': [{'nozzle_base_mm': rounded(r['nozzle']), load_key: round(r['load'], 2),
                          'peak_'+load_key: round(r['peak_load'], 2), 'tare_counts': r['tare_counts']}
                         for r in readings],
        }
        self.save()
        print('Point %d: bed Z %.4f mm; repeat spread %.4f mm (%d/%d recorded)' %
              (self.index+1, self.records[self.index]['bed_z_mm'], spread, len(self.records), len(self.points)))

    def save(self, *, promote=True):
        data = self.measurement_data()
        complete = len(self.records) == len(self.points) and promote
        data.update(description='ALPS nozzle contact; robot Base frame, mm', method='alps_load_cell',
                    complete=complete, contact_settings=asdict(self.contact_options),
                    load_cell={'name': self.args.load_cell, 'trigger_mode': self.contact_options.trigger_mode,
                               'session_reference_counts': getattr(self, 'reference_counts', None),
                               **{k: self.source.sensor.get(k) for k in
                                  ('is_calibrated', 'counts_per_gram', 'reference_tare_counts')}})
        pending = self.output.with_suffix('.pending.yaml')
        destination = self.output if complete else pending
        destination.parent.mkdir(parents=True, exist_ok=True)
        if complete and not self.promoted and self.output.exists():
            backup = self.output.with_name(self.output.name+'.'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.bak')
            shutil.copy2(self.output, backup)
            print('Previous bed map backed up to '+str(backup))
        temporary = destination.with_name(destination.name+'.tmp')
        temporary.write_text(yaml.safe_dump(data, sort_keys=False))
        temporary.replace(destination)
        if complete:
            self.promoted = True
            pending.unlink(missing_ok=True)
            print('Complete bed map saved to '+str(self.output))

    def probe_all(self):
        for index in range(len(self.points)):
            if index != self.index: self.go_to(index)
            self.probe_current()

    def on_key(self, key):
        if key in (' ', 't'): self.probe_current()
        elif key == 'a':
            self.probe_all()
            return False
        elif key == '?': print(HELP)
        elif key in ('n', 'p', 'h', 'q'): return super().on_key(key)
        return True

    def run(self, stream=None):
        if self.detector is None: self.set_baseline()
        print('SIMULATION: synthetic force, no robot or printer connection.' if self.config.dry_run
              else 'ALPS contact threshold: %g %s; maximum: %g %s' %
                   (self.contact_options.threshold, self.contact_options.unit,
                    self.contact_options.maximum, self.contact_options.unit))
        # The manual loop uses the subclass help/keys too.
        if not self.args.automatic:
            import sys
            return super().run(sys.stdin if stream is None else stream)
        print(HELP)
        if not self.config.dry_run:
            self.wait_until(self.action.server_is_ready, self.config.startup_timeout_s, 'trajectory action server')
        self.check_interrupted()
        self.approach(self.hover(self.points[0]))
        self.probe_all()
