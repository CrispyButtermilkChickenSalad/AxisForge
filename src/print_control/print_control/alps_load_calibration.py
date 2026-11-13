"""Supervised, Z-only scale contact and manual load adjustment; never writes calibration."""
from dataclasses import replace
import math
from pathlib import Path
import signal
import statistics
import sys

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

from print_control.bed_probe import load_configs, rounded
from print_control.bridge_bed_probe import BridgeBedProbe, load_profile, parse_args
from print_control.load_cell_bed_probe import sensor_setup
from print_control.load_cell_contact import stable_baseline
from print_control.load_calibration_control import LoadCalibrationControl, STEP_SIZES


def sensitive_options(options, samples):
    """Use the lowest threshold with a 4:1 measured noise margin, bounded at 20k."""
    noise = max(s.counts for s in samples)-min(s.counts for s in samples)
    threshold = max(8000., math.ceil(noise*4.1))
    if threshold > 20000.:
        raise ValueError('ALPS noise is too high for scale contact; stabilize the sensor before moving')
    return replace(options, trigger_mode='raw', contact_delta_counts=threshold), noise


class LoadCalibration(BridgeBedProbe):
    def run_routine(self):
        # The operator may move to home here; capture the pose only AFTER confirmation.
        self.control.wait_confirmation('home',
            'Manually move the arm to its home/start pose with the nozzle upright and cool. '
            'Stop manual motion, then confirm. This routine will not move the arm home.', self.pump)
        self.capture_start()
        self.confirm_position('scale',
            'Place a weighing scale on the buildplate directly below the clean nozzle. '
            'Leave a small unloaded gap (at most 20 mm). Confirm that the Z-only descent is clear.')
        samples = self.window()
        self.contact_options, noise = sensitive_options(self.contact_options, samples)
        stable_baseline(samples, self.source.sensor, self.contact_options)
        self.set_baseline()
        self.emit_stage('finding_scale', 'Finding the scale at 0.1 mm/s using 0.01 mm Z steps.',
                        sensor_noise_counts=noise)
        contact = self.touch_once(floor_z=self.start_xyz[2]-20., speed_mm_s=.1, step_mm=.01)
        self.contact_z = float(contact['nozzle'][2])
        self.emit_stage('retracting', 'Lifting 5 mm to release the weighing scale.')
        self.retract_contact(distance_mm=5., speed_mm_s=.5)
        self.upper_z = float(self.target[2])
        self.lower_z = self.contact_z-.25
        self.confirm_position('tare',
            'The nozzle has lifted 5 mm and the load has cleared. Tare the weighing scale to zero, '
            'then confirm to enable manual Z steps. Do not tare ALPS while it is loaded.')
        while True:
            expected, joints = self.nozzle_now()
            def guarded_pump():
                self.pump()
                actual, current = self.nozzle_now()
                if np.linalg.norm(actual-expected) > .1 or np.max(np.abs(current-joints)) > math.radians(.1):
                    raise RuntimeError('Arm moved outside calibration controls; restart from the new pose')
            action = self.control.wait_action(guarded_pump, self.load_status)
            guarded_pump()
            if action['command'] == 'finish':
                # Completing at a loaded pose would leave the scale compressed.
                samples = self.window()
                if max(abs(self.detector.load(s.counts)) for s in samples) >= self.contact_options.threshold*.25:
                    self.control.emit('step_rejected', 'Load is still applied. Use Up to release the scale before finishing.')
                    continue
                return
            delta = action['delta_mm']
            error = self.step_error(delta)
            if error:
                self.control.emit('step_rejected', error)
                continue
            self.control.emit('stepping', 'Moving Z by %+.3f mm' % delta)
            self.move_z(float(self.target[2])+delta, 'scale baby step', .1)
            self.detector.update(self.window())

    def step_error(self, delta):
        if type(delta) not in (int, float) or abs(delta) not in STEP_SIZES:
            return 'Invalid baby-step distance'
        end = float(self.target[2])+delta
        if not self.lower_z-1e-8 <= end <= self.upper_z+1e-8:
            return 'Step exceeds the calibration travel limit (5 mm above contact to 0.25 mm below).'
        if delta < -.05 and end+1e-8 < self.contact_z+.5:
            return 'Use 0.05 mm or smaller steps within 0.5 mm of contact.'
        return ''

    def load_status(self):
        samples = self.source.snapshot()
        latest = samples[-1].source_time
        samples = [s for s in samples if s.source_time >= latest-self.contact_options.window_s]
        return {'position_mm':rounded(self.target), 'contact_z_mm':self.contact_z,
                'lower_z_mm':self.lower_z, 'upper_z_mm':self.upper_z,
                'raw_counts':statistics.median(s.counts for s in samples),
                'delta_counts':statistics.median(self.detector.load(s.counts) for s in samples),
                'trigger_value':self.contact_options.threshold, 'trigger_unit':'counts',
                'max_value':self.contact_options.maximum}


def main(argv=None):
    control = LoadCalibrationControl().start()
    node = source = None
    handlers, failure = {}, None
    try:
        args = parse_args(sys.argv[1:] if argv is None else argv)
        args.profile = str(Path(args.config).with_name('alps_load_calibration.yaml'))
        options, homing = load_profile(args.profile)
        if options.trigger_mode != 'raw': raise ValueError('Scale calibration requires raw ADC mode')
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, lambda *_: control.cancel('Scale calibration stopped'))
        settings, motion = load_configs(args.config, args.dry_run)
        options, source = sensor_setup(args, options)
        control.check()
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = LoadCalibration(settings, motion, args, source, options, homing, control)
        node.run_routine()
    except Exception as error:
        failure = str(error)
    finally:
        if node is not None:
            try: node.stop()
            except Exception as error: failure = (failure+'; ' if failure else '')+str(error)
            finally: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if source is not None: source.close()
        for signum, handler in handlers.items(): signal.signal(signum, handler)
    if failure:
        control.emit('aborted' if control.reason else 'failed', failure,
                     stop_unconfirmed=bool(node is not None and node.stop_unconfirmed))
        raise SystemExit(1)
    control.emit('complete', 'Scale routine finished with the load released. '
                 'Save the two measured weights in ALPS Logger → Calibration. '
                 'Klipper calibration and the bed map were not changed.', progress=1.)


if __name__ == '__main__': main()
