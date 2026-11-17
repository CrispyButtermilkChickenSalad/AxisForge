"""Supervised Z-only load calibration; hand saving/restart to the dashboard after stopping."""
from dataclasses import replace
import math
from pathlib import Path
import signal
import statistics
import sys

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.signals import SignalHandlerOptions

from print_control.bed_probe import load_configs, rounded
from print_control.bridge_bed_probe import BridgeBedProbe, load_profile, parse_args
from print_control.load_cell_bed_probe import sensor_setup
from print_control.load_cell_contact import (stable_baseline, thermal_contact_options, ContactDetected, ContactDetector,
                                            LoadCellOverload, ContactMeasurementRejected)
from print_control.load_calibration_control import LoadCalibrationControl, STEP_SIZES
from print_control.load_calibration_policy import CalibrationPolicy, CalibrationDetector
from print_control.motion_planner import JOINT_NAMES


def sensitive_options(options, samples):
    """Adapt within the session's bounded unloaded-noise margin, capped at 20k."""
    noise = max(s.counts for s in samples)-min(s.counts for s in samples)
    threshold = max(8000., math.ceil(noise*1.025/options.baseline_noise_fraction))
    if threshold > 20000.:
        raise ValueError('ALPS noise is too high for scale contact; stabilize the sensor before moving')
    return replace(options, trigger_mode='raw', contact_delta_counts=threshold), noise


class LoadCalibration(BridgeBedProbe):
    def on_joints(self, message):
        previous = self.joints
        super().on_joints(message)
        if self.joints is previous:
            return  # Never accept velocities from a rejected/stale position sample.
        mapping = dict(zip(message.name, message.velocity))
        velocity = np.array([mapping.get(name, float('nan')) for name in JOINT_NAMES])
        self._calibration_velocity = velocity if np.all(np.isfinite(velocity)) else None
        self._calibration_velocity_received = self.joint_received

    def require_calibration_stationary(self):
        """Check terminal commands and fresh reported speed, separately from bed-Z accuracy."""
        if self.config.dry_run:
            return
        if not self._load_limit_stop_confirmed:
            raise RuntimeError('Calibration measurement requires confirmed robot termination')
        goal_future = getattr(self, 'goal_future', None)
        goal_handle = getattr(self, 'goal_handle', None)
        if goal_future is not None and (not goal_future.done() or goal_handle is None):
            raise RuntimeError('Calibration measurement has unresolved robot trajectory acceptance')
        if goal_handle is not None and not goal_handle.accepted:
            raise RuntimeError('Robot rejected the calibration trajectory')
        if goal_handle is not None and goal_handle.accepted:
            result_future = getattr(self, 'result_future', None)
            if result_future is None or not result_future.done():
                raise RuntimeError('Calibration measurement requires a terminal robot trajectory')
            result = result_future.result()
            if (result.status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED) or
                    (result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code != 0)):
                raise RuntimeError('Robot controller fault prevents calibration measurement')
        if not self.joints_fresh():
            raise RuntimeError('Calibration joint feedback is stale')
        velocity = getattr(self, '_calibration_velocity', None)
        if (velocity is None or np.shape(velocity) != (len(JOINT_NAMES),) or not np.all(np.isfinite(velocity)) or
                getattr(self, '_calibration_velocity_received', None) != self.joint_received):
            raise RuntimeError('Calibration requires fresh finite joint velocity feedback')
        if np.max(np.abs(velocity)) > 1e-4:
            raise RuntimeError('Robot reports motion during calibration; wait for it to stop')

    def set_baseline(self):
        policy = getattr(self, 'calibration_policy', None)
        if policy is not None and not self.config.dry_run:
            policy.require_ready()
        return super().set_baseline()

    def _run_routine(self):
        self.calibration_policy = getattr(self, 'calibration_policy', CalibrationPolicy(
            getattr(self.args, 'calibration_max_counts', 120000.),
            getattr(self.args, 'calibration_hot_target', None)))
        self._manual_loading = self._calibration_step_active = self._release_step_active = False
        self.load_limit_reached = False
        self._release_ceiling = None
        self._load_limit_stop_confirmed = True
        self.calibration_measurement = None
        self.calibration_tare_counts = None
        hot = self.calibration_policy.hot_target
        temperature_text = 'cool' if hot is None else 'at the selected %.1f C temperature (hot nozzle)' % hot
        # Initial upright setup remains manual; Z positioning follows confirmation.
        self.control.wait_confirmation('home',
            'Confirm the arm is stationary at an upright start pose with the nozzle '+temperature_text+'. '
            'Dashboard Z controls follow scale placement; this routine does not change XY or rotate the tool.', self.pump,
            **self.calibration_policy.status())
        self.capture_start()
        self.confirm_position('scale',
            'Place a weighing scale on the buildplate directly below the clean nozzle. '
            'Keep the nozzle unloaded and confirm that the Z-only travel is clear. Dashboard Z controls '
            'will let you leave a gap of at most 20 mm before continuing the automatic search.'+
            ('' if hot is None else ' The nozzle is hot at %.1f C: confirm the contact surface can tolerate '
             'this temperature and keep hands clear.' % hot))
        samples = self.window()
        self.contact_options, noise = sensitive_options(self.contact_options, samples)
        stable_baseline(samples, self.source.sensor, self.contact_options)
        self.set_baseline()
        self.manual_z_session()
        self.emit_stage('finding_scale', 'Finding the scale at 0.1 mm/s using 0.01 mm Z steps.',
                        sensor_noise_counts=noise)
        contact = self.touch_once(floor_z=self.start_xyz[2]-20., speed_mm_s=.1, step_mm=.01)
        self.contact_z = float(contact['nozzle'][2])
        self.emit_stage('retracting', 'Lifting 5 mm to release the weighing scale.')
        self.retract_contact(distance_mm=5., speed_mm_s=.5)
        self.upper_z = self.contact_z+25.
        self.lower_z = self.contact_z-.25
        self.confirm_position('tare',
            'The nozzle has lifted 5 mm and the load has cleared. Tare the weighing scale to zero, '
            'then confirm. The bridge will record the unloaded ALPS reference and enable manual Z steps. '
            'Do not touch or load the nozzle while confirming.')
        self.enable_manual_loading()
        self.begin_calibration()
        while True:
            expected, joints = self.nozzle_now()
            def guarded_pump():
                self.pump()
                actual, current = self.nozzle_now()
                if np.linalg.norm(actual-expected) > .1 or np.max(np.abs(current-joints)) > math.radians(.1):
                    raise RuntimeError('Arm moved outside calibration controls; restart from the new pose')
            action = self.control.wait_action(guarded_pump, self.load_status)
            guarded_pump()
            if action['command'] == 'record':
                self.record_calibration(action['grams'])
                continue
            if action['command'] == 'finish':
                # Completing at a loaded pose would leave the scale compressed.
                samples = self.window()
                self._clear_released_limit(samples)
                if self.load_limit_reached:
                    self.control.emit('step_rejected', 'Load limit reached. Use small Up steps to release before finishing.')
                    continue
                if max(abs(self.detector.load(s.counts)) for s in samples) >= self.contact_options.threshold*.25:
                    self.control.emit('step_rejected', 'Load is still applied. Use Up to release the scale before finishing.')
                    continue
                if self.calibration_measurement is None:
                    self.control.emit('step_rejected', 'Record a stable loaded scale reading before saving calibration.')
                    continue
                return self.accept_calibration()
            delta = action['delta_mm']
            error = self.step_error(delta)
            if error:
                self.control.emit('step_rejected', error)
                continue
            self.control.emit('stepping', 'Moving Z by %+.3f mm' % delta)
            self.calibration_step(delta)
            samples = self.window()
            self.detector.update(samples)
            self._clear_released_limit(samples)

    def calibration_window(self):
        """A fresh stationary window; keep the same guards during native collection."""
        if not self._load_limit_stop_confirmed:
            raise RuntimeError('Calibration measurement requires confirmed robot termination')
        if not self.config.dry_run:
            self.calibration_policy.require_ready()
        expected, joints = self.nozzle_now()
        self.require_calibration_stationary()
        def pump():
            self.pump()
            actual, current = self.nozzle_now()
            self.require_calibration_stationary()
            if (np.linalg.norm(actual-expected) > .1 or
                    np.max(np.abs(current-joints)) > math.radians(.1)):
                raise RuntimeError('Robot moved during the calibration measurement')
            if not self.config.dry_run:
                self.calibration_policy.require_ready()
        samples = self.source.wait_window(self.source.clock(), pump=pump)
        pump()
        self.detector.update(samples)
        return samples, pump

    def measurement_pump(self, pump, samples, *, unloaded, minimum_counts=0.):
        expected = statistics.median(self.detector.load(s.counts) for s in samples)
        def checked():
            pump()
            recent = self.source.snapshot()
            latest = recent[-1].source_time
            recent = [s for s in recent if s.source_time >= latest-self.contact_options.window_s]
            self.detector.update(recent)
            stable_baseline(recent, self.source.sensor, self.contact_options)
            loads = [self.detector.load(s.counts) for s in recent]
            if self.load_limit_reached:
                raise RuntimeError('Release the calibration load limit before recording or saving')
            if abs(statistics.median(loads)-expected) > self.contact_options.threshold*.25:
                raise RuntimeError('Load changed during the calibration measurement')
            if unloaded and max(abs(value) for value in loads) >= self.contact_options.threshold*.25:
                raise RuntimeError('The nozzle must remain unloaded while zeroing or saving calibration')
            if minimum_counts and min(abs(s.counts-self.calibration_tare_counts) for s in recent) < minimum_counts:
                raise RuntimeError('Load fell below the native calibration minimum during recording')
        return checked

    def begin_calibration(self):
        samples, pump = self.calibration_window()
        if not self.is_unloaded(samples):
            raise RuntimeError('Release all load before recording the unloaded ALPS reference')
        pump = self.measurement_pump(pump, samples, unloaded=True)
        if not self.config.dry_run:
            result = self.calibration_client.begin_unloaded(pump)
            self.calibration_tare_counts = result['reference_tare_counts']
        pump()

    def record_calibration(self, grams):
        if type(grams) not in (int, float) or not math.isfinite(grams) or not 50 <= grams <= 25000:
            raise ValueError('Enter the actual scale reading in grams (50..25000)')
        if self.calibration_measurement is not None:
            self.control.emit('step_rejected', 'A measurement is already recorded. Release the load and save, or restart calibration.')
            return
        if self.load_limit_reached:
            self.control.emit('step_rejected', 'Release the load limit before recording a scale reading.')
            return
        samples, pump = self.calibration_window()
        if self.load_limit_reached:
            self.control.emit('step_rejected', 'The latest reading reached the load limit. Use small Up steps to release it.')
            return
        try:
            stable_baseline(samples, self.source.sensor, self.contact_options)
        except ValueError as error:
            self.control.emit('step_rejected', 'Wait for a stable load before recording: '+str(error))
            return
        if abs(statistics.median(self.detector.load(s.counts) for s in samples)) < self.contact_options.threshold:
            self.control.emit('step_rejected', 'Apply a stable load before recording the scale reading.')
            return
        # Stock Klipper requires >=1% of the ADS131M02 positive ADC range:
        # 83886.07 counts. Reject insufficient load before issuing CALIBRATE,
        # so the operator can keep using the small release/adjustment buttons.
        minimum = 0. if self.config.dry_run else 83887.
        if minimum and min(abs(s.counts-self.calibration_tare_counts) for s in samples) < minimum:
            self.control.emit('step_rejected', 'Klipper needs a stable ADC change of at least 83887 counts. '
                              'Adjust the load cautiously within the configured limit before recording.')
            return
        pump = self.measurement_pump(pump, samples, unloaded=False, minimum_counts=minimum)
        self.control.emit('recording_load', 'Recording the measured %.3f g scale reading in Klipper.' % grams)
        if self.config.dry_run:
            measurement = {'grams': float(grams), 'simulated': True}
        else:
            measurement = self.calibration_client.record_load(grams, pump)
        pump()
        self.calibration_measurement = measurement

    def accept_calibration(self):
        if self.calibration_measurement is None:
            raise RuntimeError('Record a valid scale measurement before saving calibration')
        self._load_limit_stop_confirmed = False
        self.cancel_contact()  # Prove terminal robot state before staging persistent values.
        self._load_limit_stop_confirmed = True
        samples, pump = self.calibration_window()
        if not self.is_unloaded(samples) or self.load_limit_reached:
            raise RuntimeError('Release all load and wait for stable readings before saving calibration')
        pump = self.measurement_pump(pump, samples, unloaded=True)
        self.control.emit('accepting_calibration', 'Validating the recorded Klipper calibration before stopping robot monitoring.')
        if self.config.dry_run:
            pump()
            return None
        result = self.calibration_client.accept(pump)
        pump()
        return result

    def is_unloaded(self, samples):
        if (len(samples) < 10 or samples[-1].source_time-samples[0].source_time < self.contact_options.window_s-1e-8 or
                max(abs(self.detector.load(s.counts)) for s in samples) >= self.contact_options.threshold*.25):
            return False
        try:
            stable_baseline(samples, self.source.sensor, self.contact_options, self.reference_counts)
        except ValueError:
            return False
        return True

    def enable_manual_loading(self):
        """Raise only the manual weight-building guard, after unloaded tare confirmation."""
        samples = self.window()
        if max(abs(self.detector.load(s.counts)) for s in samples) >= self.contact_options.threshold*.25:
            raise RuntimeError('Release all load before enabling manual calibration weights')
        options = replace(self.contact_options, max_delta_counts=self.calibration_policy.max_counts)
        stable_baseline(samples, self.source.sensor, options, self.reference_counts)
        previous = self.detector
        self.contact_options = options
        self.detector = self._manual_detector(previous.baseline)
        self.detector.last_sequence = previous.last_sequence
        self.detector.peak_load, self.detector.triggered = previous.peak_load, previous.triggered
        self._manual_loading = True

    def _manual_detector(self, baseline):
        return CalibrationDetector(baseline, self.source.sensor.get('counts_per_gram'),
                                   self.contact_options, self._on_load_limit, self._check_release_load)

    def _check_release_load(self, load):
        if load >= 250000.:
            raise RuntimeError('Calibration hard load ceiling reached (250000 counts); use manual recovery')
        ceiling = getattr(self, '_release_ceiling', None)
        if getattr(self, 'load_limit_reached', False) and ceiling is not None and load > ceiling:
            raise RuntimeError('Load increased after the calibration stop; use manual recovery')

    def _freeze_release_ceiling(self, peak):
        self._check_release_load(peak)
        if getattr(self, '_release_ceiling', None) is None:
            self._release_ceiling = min(250000., peak+self.contact_options.threshold*.25)

    def _on_load_limit(self, error):
        self.load_limit_reached = True
        self._limit_seen_at = self.source.clock()
        if getattr(self, '_release_step_active', False):
            return  # Only bounded +Z release; every other safety check still runs.
        if getattr(self, '_calibration_step_active', False):
            raise error
        if not getattr(self, '_load_limit_stop_confirmed', False):
            raise RuntimeError('Calibration load limit reached with unconfirmed robot termination') from error
        self._freeze_release_ceiling(error.load)

    def _clear_released_limit(self, samples):
        if not getattr(self, 'load_limit_reached', False):
            return
        fresh = [s for s in samples if s.observed_at >= self._limit_seen_at+self.contact_options.settle_s]
        if (len(fresh) < 10 or fresh[-1].source_time-fresh[0].source_time < self.contact_options.window_s or
                max(abs(self.detector.load(s.counts)) for s in fresh) >= self.contact_options.threshold*.25):
            return
        try:
            stable_baseline(fresh, self.source.sensor, self.contact_options, self.reference_counts)
        except ValueError:
            return
        self.load_limit_reached = False
        self._release_ceiling = None

    def calibration_step(self, delta):
        error = self.step_error(delta)
        if error:
            raise ValueError(error)
        # Coarse downward clicks are clearance motion only; stop on any
        # unexpected contact rather than permitting them to build load.
        clearance = float(self.target[2]) >= self.contact_z+.5
        speed = 1. if abs(delta) > .1 and clearance else .1
        releasing = getattr(self, 'load_limit_reached', False)
        if releasing:
            if not self._load_limit_stop_confirmed:
                raise RuntimeError('Confirm robot termination before releasing the load')
            samples = self.source.snapshot()
            latest = samples[-1].source_time
            recent = [s for s in samples if s.source_time >= latest-self.contact_options.window_s]
            self._check_release_load(max(abs(self.detector.load(s.counts)) for s in recent))
            if self._release_ceiling is None:
                raise RuntimeError('Calibration release ceiling was not established after stopping')
            speed = .1
        if delta < -.1:
            samples = self.window()
            if max(abs(self.detector.load(s.counts)) for s in samples) >= self.contact_options.threshold*.25:
                raise RuntimeError('Unload the scale before a coarse downward Z step')
            # The initial contact is latched. Rearm detection at the SAME
            # unloaded reference; never tare away an existing load.
            baseline = self.detector.baseline
            self.detector = (self._manual_detector(baseline) if getattr(self, '_manual_loading', False) else
                             ContactDetector(baseline, self.source.sensor.get('counts_per_gram'), self.contact_options))
            self.detector.last_sequence = samples[-1].sequence
        self.seeking = delta < -.1
        self._calibration_step_active = True
        self._release_step_active = releasing
        self._load_limit_stop_confirmed = False
        try:
            self.move_z(float(self.target[2])+delta, 'scale baby step', speed)
            self._load_limit_stop_confirmed = True
        except LoadCellOverload as error:
            if not getattr(self, '_manual_loading', False):
                raise
            self._calibration_step_active = self._release_step_active = False
            self.load_limit_reached = True
            self._limit_seen_at = self.source.clock()
            self.cancel_contact()  # Requires a terminal action result and refreshes actual pose.
            self._load_limit_stop_confirmed = True
            samples = self.source.snapshot()
            latest = samples[-1].source_time
            recent = [s for s in samples if s.source_time >= latest-self.contact_options.window_s]
            self._freeze_release_ceiling(max(abs(self.detector.load(s.counts)) for s in recent))
            self.check_interrupted()  # Includes robot, configuration, thermal and stream guards.
            self.control.emit('load_limit', str(error)+'. Motion stopped. Only small Up steps are allowed until released.',
                              **self.load_status())
        except ContactDetected as error:
            self.cancel_contact()
            raise ContactMeasurementRejected('Unexpected contact during a coarse Z step; release before restarting scale calibration') from error
        finally:
            self.seeking = False
            self._calibration_step_active = self._release_step_active = False

    def step_error(self, delta):
        if type(delta) not in (int, float) or abs(delta) not in STEP_SIZES:
            return 'Invalid baby-step distance'
        if getattr(self, 'load_limit_reached', False) and delta not in (.001, .01, .05, .1):
            return 'Load limit reached: only Up 0.001, 0.01, 0.05 or 0.1 mm is allowed until released.'
        end = float(self.target[2])+delta
        if not self.lower_z-1e-8 <= end <= self.upper_z+1e-8:
            return 'Step exceeds the calibration travel limit (25 mm above contact to 0.25 mm below).'
        if delta < -.1 and end+1e-8 < self.contact_z+.5:
            return 'Use 0.1 mm or smaller steps within 0.5 mm of contact.'
        return ''

    def load_status(self):
        samples = self.source.snapshot()
        latest = samples[-1].source_time
        samples = [s for s in samples if s.source_time >= latest-self.contact_options.window_s]
        self._clear_released_limit(samples)
        measurement = getattr(self, 'calibration_measurement', None)
        result = {'position_mm':rounded(self.target), 'contact_z_mm':self.contact_z,
                'lower_z_mm':self.lower_z, 'upper_z_mm':self.upper_z,
                'raw_counts':statistics.median(s.counts for s in samples),
                'delta_counts':statistics.median(self.detector.load(s.counts) for s in samples),
                'trigger_value':self.contact_options.threshold, 'trigger_unit':'counts',
                'max_value':self.contact_options.maximum,
                'load_limit_reached':getattr(self, 'load_limit_reached', False),
                'upward_only':getattr(self, 'load_limit_reached', False),
                'unloaded':self.is_unloaded(samples),
                'calibration_recorded':measurement is not None,
                **getattr(self, 'calibration_policy', CalibrationPolicy()).status()}
        if measurement is not None:
            result['recorded_grams'] = measurement['grams']
            for field, source in (('calibration_counts_per_gram', 'counts_per_gram'),
                                  ('calibration_tare_counts', 'reference_tare_counts')):
                if source in measurement:
                    result[field] = measurement[source]
        return result


def main(argv=None):
    control = LoadCalibrationControl().start()
    node = source = client = None
    calibration_values = None
    handlers, failure = {}, None
    try:
        args = parse_args(sys.argv[1:] if argv is None else argv)
        args.profile = str(Path(args.config).with_name('alps_load_calibration.yaml'))
        options, homing = load_profile(args.profile)
        if options.trigger_mode != 'raw': raise ValueError('Scale calibration requires raw ADC mode')
        if options.trigger_source != 'stream': raise ValueError('Scale calibration requires the raw stream trigger source')
        if args.load_cell != 'load_cell_probe':
            raise ValueError('Native scale calibration requires the load_cell_probe sensor')
        policy = CalibrationPolicy(args.calibration_max_counts, args.calibration_hot_target)
        options = thermal_contact_options(options,hot_nozzle=policy.hot_target is not None)
        # The initial search has its own guard, before manual calibration loading.
        options = replace(options, max_delta_counts=min(options.max_delta_counts, 120000.))
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, lambda *_: control.cancel('Scale calibration stopped'))
        settings, motion = load_configs(args.config, args.dry_run)
        options, source = sensor_setup(args, options, status_validator=policy.validate_status, check_baseline=False)
        control.check()
        if not args.dry_run:
            policy.wait_ready(source, control)
            from print_control.klipper_load_calibration import KlipperLoadCalibration
            client = KlipperLoadCalibration(source.moonraker_url, source.api_key)
            def prepare_pump():
                control.check()
                source.snapshot()
                policy.require_ready()
            client.prepare(prepare_pump)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = LoadCalibration(settings, motion, args, source, options, homing, control)
        node.calibration_policy = policy
        node.calibration_client = client
        calibration_values = node.run_routine()
        if not args.dry_run and (not isinstance(calibration_values, dict) or
                calibration_values.get('accepted') is not True or calibration_values.get('saved') is not False):
            raise RuntimeError('Klipper did not confirm the calibration values for saving')
    except Exception as error:
        failure = str(error)
    finally:
        if node is not None:
            try: node.stop()
            except Exception as error: failure = (failure+'; ' if failure else '')+str(error)
            try: node.destroy_node()
            except Exception as error: failure = (failure+'; ' if failure else '')+str(error)
        if client is not None and failure:
            try: client.abort(lambda: None)
            except Exception as error: failure += '; calibration cleanup failed: '+str(error)
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception as error: failure = (failure+'; ' if failure else '')+str(error)
        if source is not None:
            try: source.close()
            except Exception as error: failure = (failure+'; ' if failure else '')+str(error)
        for signum, handler in handlers.items(): signal.signal(signum, handler)
    if not failure:
        try: control.check()
        except Exception as error: failure = str(error)
    if failure:
        control.emit('aborted' if control.reason else 'failed', failure,
                     stop_unconfirmed=bool(node is not None and node.stop_unconfirmed))
        raise SystemExit(1)
    if calibration_values is not None:
        control.emit('ready_to_save', 'Robot stopped and sensor monitoring closed. Saving the validated calibration and restarting Klipper.',
                     calibration_values=calibration_values, unloaded=True, calibration_recorded=True)
    else:
        control.emit('complete', 'Dry-run scale calibration finished with the load released. '
                     'No Klipper calibration or bed map was changed.', progress=1.)


if __name__ == '__main__': main()
