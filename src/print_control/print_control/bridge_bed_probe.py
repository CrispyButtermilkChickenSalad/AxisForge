"""Bridge-controlled Z homing, confirmed mesh placement, and ALPS bed probing."""
import argparse
from dataclasses import dataclass, fields, replace
import math
from pathlib import Path
import signal
import sys
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from spatialmath import SE3
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import SafetyMode
import yaml

from print_control.bed_probe import grid_points, load_configs, rounded
from print_control.load_cell_bed_probe import LoadCellBedProbe, sensor_setup
from print_control.load_cell_contact import ContactOptions, finite, repeat_contact
from print_control.probe_control import ProbeControl
from print_control.continuous_probe import ContinuousHoming
from print_control.load_cell_contact import ContactDetected


@dataclass(frozen=True)
class HomingOptions:
    first_speed_mm_s: float = 10.
    second_speed_mm_s: float = .2
    # Accepted for compatibility with old profiles; continuous passes do not step.
    first_step_mm: float = .05
    second_step_mm: float = .01
    acceleration_mm_s2: float = 50.
    sample_age_s: float = .15
    cancel_timeout_s: float = .2
    max_stop_travel_mm: float = .5
    retract_mm: float = 3.
    max_descent_mm: float = 20.
    max_retest_difference_mm: float = .5
    clearance_above_start_mm: float = 10.
    lift_speed_mm_s: float = 20.
    # Optional legacy override; otherwise inherit AxisForge's XY travel speed.
    travel_speed_mm_s: float = None
    mesh_columns: int = 3
    mesh_rows: int = 3
    confirmation_timeout_s: float = 300.
    heartbeat_timeout_s: float = 5.
    confirmation_pose_tolerance_mm: float = .1

    def __post_init__(self):
        if not all(finite(v) and v > 0 for k, v in vars(self).items()
                   if k != 'travel_speed_mm_s'):
            raise ValueError('Homing settings must be finite positive numbers')
        if self.travel_speed_mm_s is not None and (not finite(self.travel_speed_mm_s) or self.travel_speed_mm_s <= 0):
            raise ValueError('XY travel override must be a finite positive speed')
        if not 1 <= self.lift_speed_mm_s <= 20:
            raise ValueError('Upward speed must be 1..20 mm/s')
        if any(type(v) is not int or not 2 <= v <= 9 for v in (self.mesh_columns, self.mesh_rows)):
            raise ValueError('Mesh columns and rows must be integers from 2 to 9')
        if not 0 < self.second_speed_mm_s < self.first_speed_mm_s <= 10.:
            raise ValueError('Homing speeds require second < first <= 10 mm/s')
        if not .001 <= self.second_step_mm < self.first_step_mm <= .05:
            raise ValueError('Homing steps require 0.001 <= second < first <= 0.05 mm')
        if not 1 <= self.acceleration_mm_s2 <= 100:
            raise ValueError('Continuous homing acceleration must be 1..100 mm/s^2')
        if not .02 <= self.sample_age_s <= .2 or not .05 <= self.cancel_timeout_s <= .25:
            raise ValueError('Homing sample age must be 20..200 ms; cancellation 50..250 ms')
        if not .05 <= self.max_stop_travel_mm <= 1.:
            raise ValueError('Observed homing stop travel must be 0.05..1 mm')
        if not 1 <= self.retract_mm <= 5 or not self.retract_mm < self.max_descent_mm <= 50:
            raise ValueError('Homing retract must be 1..5 mm, and descent > retract and <= 50 mm')
        if self.max_retest_difference_mm >= self.retract_mm:
            raise ValueError('Homing repeat difference must be less than the retract')
        if not 5 <= self.clearance_above_start_mm <= 50:
            raise ValueError('Travel clearance must be 5..50 mm')
        if self.heartbeat_timeout_s > 5 or self.confirmation_pose_tolerance_mm > .2:
            raise ValueError('Heartbeat timeout must be <= 5 s; confirmation pose tolerance <= 0.2 mm')


def load_profile(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or set(data)-{'contact', 'homing'}:
        raise ValueError('Bridge probe profile must contain contact and homing settings')
    for name, cls in (('contact', ContactOptions), ('homing', HomingOptions)):
        values = data.get(name, {})
        if not isinstance(values, dict) or set(values)-{f.name for f in fields(cls)}:
            raise ValueError('Unknown '+name+' probe settings')
    return ContactOptions(**data.get('contact', {})), HomingOptions(**data.get('homing', {}))


class BridgeBedProbe(ContinuousHoming, LoadCellBedProbe):
    def probe_grid(self):
        return grid_points(self.settings.bed_size_mm, self.args.margin_mm,
                           self.homing.mesh_columns, self.homing.mesh_rows)

    @property
    def xy_travel_speed(self):
        return min(self.settings.max_travel_speed_mm_s,
                   self.homing.travel_speed_mm_s or self.settings.max_travel_speed_mm_s)

    def retract_contact(self, *, distance_mm=None, speed_mm_s=None):
        # Includes the small retraction between every repeated mesh contact.
        return super().retract_contact(distance_mm=distance_mm,
            speed_mm_s=self.homing.lift_speed_mm_s if speed_mm_s is None else speed_mm_s)

    def __init__(self, settings, motion, args, source, options, homing, control):
        self.control, self.homing = control, homing
        self.monitor_robot = False
        self.robot_signals = {}
        self.returned = False
        self.start_xyz = self.start_joints = None
        self.homed_z = None
        self.homing_readings = []
        self.stop_client = None
        self.stop_unconfirmed = False
        self.files = {Path(p).resolve(): (Path(p).stat().st_mtime_ns, Path(p).stat().st_size)
                      for p in (args.config, args.profile)}
        # Preserve a fixed absolute simulation plane when homing changes the Z estimate.
        super().__init__(settings, motion, args, source, options)
        if not self.config.dry_run:
            def received(name, value): self.robot_signals[name] = (value, time.monotonic())
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
                                     lambda m: received('speed', m.data), qos_profile_sensor_data)
            self.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
                                     lambda m: received('program', m.data), latched)
            self.create_subscription(SafetyMode, '/io_and_status_controller/safety_mode',
                                     lambda m: received('safety', m.mode), latched)
            self.stop_client = self.create_client(Trigger, '/dashboard_client/stop')

    def robot_reason(self):
        if self.config.dry_run: return ''
        if not self.joints_fresh(): return 'Robot joint feedback is stale'
        if not self.action.server_is_ready(): return 'UR trajectory controller is unavailable'
        if not self.robot_signals.get('program', (False, 0))[0]: return 'Start the UR External Control program'
        if self.robot_signals.get('safety', (None, 0))[0] != 1: return 'UR safety mode must be NORMAL'
        speed, stamp = self.robot_signals.get('speed', (None, 0))
        if time.monotonic()-stamp > .25: return 'UR speed-scaling feedback is stale'
        if not finite(speed) or not 99.9 <= speed <= 100.1: return 'Set UR speed scaling to 100% for probing'
        return ''

    def check_interrupted(self):
        self.control.check()
        self.check_continuous_homing()
        try:
            super().check_interrupted()
        except ContactDetected:
            self.capture_continuous_trigger()
            raise
        if self.monitor_robot:
            reason = self.robot_reason()
            if reason: raise RuntimeError(reason)
        for path, original in self.files.items():
            info = path.stat()
            if (info.st_mtime_ns, info.st_size) != original:
                raise RuntimeError('Probe configuration changed during the routine: '+path.name)

    def pump(self):
        self.check_interrupted()
        if self.config.dry_run: time.sleep(.01)
        else: rclpy.spin_once(self, timeout_sec=.01)

    def capture_start(self):
        if self.config.dry_run:
            centre = tuple(size/2 for size in self.settings.bed_size_mm)
            self.joints = self.planner.solve(self.bed_point(centre, 5.), self.joints)
        else:
            deadline = time.monotonic()+self.config.startup_timeout_s
            while self.robot_reason():
                self.pump()
                if time.monotonic() >= deadline: raise RuntimeError(self.robot_reason())
        self.monitor_robot = True
        nozzle, joints = self.nozzle_now()
        self.planner.check_joints(joints)
        # Reject a tilted starting nozzle; do not rotate or approach before confirmation.
        self.planner.check_pose(joints, nozzle, nozzle)
        rotation = self.planner.kinematics.eval(joints)[:3, :3]
        actual_rpy = tuple(float(v) for v in SE3.Rt(rotation, np.zeros(3)).rpy(unit='deg', order='zyx'))
        self.config = replace(self.config, nozzle_rpy_deg=actual_rpy)
        self.planner.config, self.planner.rotation = self.config, rotation.copy()
        # sampled_line derives rotations from its settings, not only .rotation.
        # Preserve the measured tool orientation on subsequent Z/XY moves too.
        self.settings = replace(self.settings, nozzle_roll_deg=actual_rpy[0],
                                nozzle_pitch_deg=actual_rpy[1], wrist_angle=actual_rpy[2])
        self.planner.settings, self.planner.orientation = self.settings, (0., 0., 0.)
        self.target, self.seed = nozzle.copy(), joints.copy()
        self.start_xyz, self.start_joints = nozzle.copy(), joints.copy()

    def emit_stage(self, state, message, **values):
        options = self.contact_options
        self.control.emit(state, message, total_points=len(self.points), current_point=len(self.records),
                          trigger_mode=options.trigger_mode, trigger_value=options.threshold,
                          trigger_unit=options.unit, max_value=options.maximum,
                          position_mm=rounded(self.target) if self.target is not None else None,
                          **values)

    def confirm_position(self, stage, message, **values):
        expected, joints = self.nozzle_now()
        self.control.wait_confirmation(stage, message, self.pump,
            position_mm=rounded(expected), total_points=len(self.points), current_point=len(self.records),
            trigger_mode=self.contact_options.trigger_mode, trigger_value=self.contact_options.threshold,
            trigger_unit=self.contact_options.unit, max_value=self.contact_options.maximum, **values)
        nozzle, current = self.nozzle_now()
        if (np.linalg.norm(nozzle-expected) > self.homing.confirmation_pose_tolerance_mm or
                np.max(np.abs(current-joints)) > math.radians(.1)):
            raise RuntimeError('Robot moved while awaiting confirmation; restart homing from the new pose')
        self.planner.check_pose(current, nozzle, nozzle)
        self.target, self.seed = nozzle.copy(), current.copy()

    def move_z(self, height, name, speed):
        self.line_to(np.array([*self.target[:2], height]), name, speed)

    def run_routine(self):
        self.capture_start()
        self.confirm_position('surface',
            'Confirm a flat build surface is under the upright nozzle, with the nozzle clean, cool, '
            'and clear of the surface (unloaded). Confirmation starts continuous vertical homing '
            'at %g mm/s, followed by two precision contacts at %g mm/s, with acceleration ramps.' %
            (self.homing.first_speed_mm_s, self.homing.second_speed_mm_s), progress=0.)
        # In raw mode this is the first tare, made only after explicit unloaded confirmation.
        self.set_baseline()
        floor = self.start_xyz[2]-self.homing.max_descent_mm
        self.emit_stage('homing_fast', 'Finding the surface with continuous Z motion at %g mm/s' %
                        self.homing.first_speed_mm_s, progress=.03)
        first = self.homing_touch(floor_z=floor, speed_mm_s=self.homing.first_speed_mm_s)
        self.retract_contact(distance_mm=self.homing.retract_mm)
        fine = []
        for index in range(2):
            self.set_baseline()
            self.emit_stage('homing_slow', 'Precision contact %d of 2 at %g mm/s' %
                            (index+1, self.homing.second_speed_mm_s), progress=.06+.025*index)
            fine.append(self.homing_touch(
                floor_z=max(floor, first['nozzle'][2]-self.homing.max_retest_difference_mm),
                speed_mm_s=self.homing.second_speed_mm_s))
            self.retract_contact(distance_mm=self.homing.retract_mm)
        # The fast touch locates the surface approximately. Never average its
        # speed-dependent stopping deflection into the accurate bed height.
        selected, spread = repeat_contact(fine, self.contact_options.repeatability_mm)
        self.homing_readings = [first, *fine]
        self.homing_repeatability_mm = spread
        self.homed_z = float(selected['nozzle'][2])+self.contact_options.contact_offset_mm
        self.settings = replace(self.settings, z_offset=self.homed_z)
        clearance = float(self.start_xyz[2])+self.homing.clearance_above_start_mm
        self.emit_stage('moving_front_left', 'Lifting before moving to the first mesh point',
                        progress=.12, homed_z_mm=self.homed_z, clearance_z_mm=clearance)
        self.move_z(clearance, 'homing clearance lift', self.homing.lift_speed_mm_s)
        first_xy = self.bed_point(self.points[0])[:2]
        self.line_to(np.array([*first_xy, clearance]), 'front-left travel', self.xy_travel_speed)
        self.confirm_position('front_left',
            'Confirm the nozzle is above the first mesh point and this is the front-leftmost point of the mesh. '
            'Confirmation starts the automatic grid scan.', progress=.15,
            first_point=rounded(self.points[0]), homed_z_mm=self.homed_z, clearance_z_mm=clearance)
        # Lower only at the confirmed XY, into known clearance above the homed plane.
        self.move_z(float(self.hover(self.points[0])[2]), 'mesh hover', self.homing.first_speed_mm_s)
        for index in range(len(self.points)):
            if index != self.index: self.go_to(index)
            self.emit_stage('meshing', 'Probing mesh point %d of %d' % (index+1, len(self.points)),
                            progress=.15+.75*index/len(self.points))
            self.probe_current()
        self.emit_stage('returning', 'Returning to the saved starting pose', progress=.93)
        self.return_to_start()
        self.returned = True
        self.save()

    def mesh_touch(self):
        # One slow, monitored approach instead of hundreds of stop/settle steps.
        floor = self.bed_point(self.points[self.index])[2]-self.args.floor_mm
        return self.homing_touch(floor_z=floor, speed_mm_s=self.contact_options.speed_mm_s)

    def go_to(self, index):
        self.lift()
        self.index = index
        target = self.hover(self.points[index])
        target[2] = max(target[2], self.target[2])
        self.line_to(target, 'mesh travel', self.xy_travel_speed)

    def lift(self):
        height = float(self.hover(self.points[self.index])[2])
        if self.target[2] < height:
            self.move_z(height, 'mesh lift', self.homing.lift_speed_mm_s)

    def return_to_start(self):
        clearance = max(float(self.start_xyz[2])+self.homing.clearance_above_start_mm,
                        max(r['bed_z_mm'] for r in self.records.values())+self.config.clearance_mm,
                        float(self.target[2]))
        self.move_z(clearance, 'return clearance lift', self.homing.lift_speed_mm_s)
        self.line_to(np.array([*self.start_xyz[:2], clearance]), 'return XY', self.xy_travel_speed)
        self.move_z(float(self.start_xyz[2]), 'return starting Z', self.homing.first_speed_mm_s)
        actual, joints = self.nozzle_now()
        if np.linalg.norm(actual-self.start_xyz) > .2:
            raise RuntimeError('Robot did not reach the saved starting pose; mesh remains pending')
        self.planner.check_pose(joints, actual, actual)

    def measurement_data(self):
        data = super().measurement_data()
        data['mesh_shape'] = {'columns': self.homing.mesh_columns, 'rows': self.homing.mesh_rows}
        data['upright_tolerance_deg'] = min(.2, self.config.orientation_tolerance_deg)
        data['homing'] = {'starting_nozzle_base_mm': rounded(self.start_xyz), 'homed_z_mm': self.homed_z,
                          'returned_to_start': self.returned,
                          'fine_repeatability_mm': getattr(self, 'homing_repeatability_mm', None),
                          'contacts': [{'nozzle_base_mm': rounded(r['nozzle']), 'load': float(r['load']),
                                        'unit': self.contact_options.unit,
                                        **{k: float(r[k]) for k in ('speed_mm_s', 'stop_ack_s', 'stop_travel_mm')
                                           if k in r}} for r in self.homing_readings]}
        return data

    def save(self, *, promote=True):
        super().save(promote=self.returned and promote)

    def stop(self):
        error = None
        try: super().stop()
        except Exception as caught: error = caught
        if self.config.dry_run: return
        unresolved = (self.goal_future is not None and not self.goal_future.done()) or (
            self.goal_handle is not None and self.goal_handle.accepted and
            (self.result_future is None or not self.result_future.done()))
        if error is None and not unresolved: return
        # The action did not confirm termination: stop the UR program, then fail.
        try:
            if self.stop_client is None or not self.stop_client.service_is_ready():
                raise RuntimeError('UR dashboard Stop service unavailable')
            future = self.stop_client.call_async(Trigger.Request())
            deadline = time.monotonic()+2.
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.02)
            if not future.done() or not future.result().success:
                raise RuntimeError('UR dashboard Stop was not confirmed')
        except Exception as stop_error:
            self.stop_unconfirmed = True
            raise RuntimeError('Robot stop unconfirmed; use pendant Stop: '+str(stop_error)) from error
        raise RuntimeError('Trajectory cancellation was unconfirmed; UR program stopped. Restart the routine.') from error


def parse_args(argv):
    default = Path(get_package_share_directory('axisforge_bringup'))/'config'/'ur_master.yaml'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(default))
    parser.add_argument('--profile')
    parser.add_argument('--output')
    parser.add_argument('--moonraker-url')
    parser.add_argument('--alps-url')
    parser.add_argument('--load-cell', default='load_cell_probe')
    parser.add_argument('--margin-mm', type=float, default=None,
                        help='Override bed_probe_margin_mm from the AxisForge configuration')
    parser.add_argument('--floor-mm', type=float, default=3.)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    args.profile = args.profile or str(Path(args.config).with_name('bridge_bed_probe.yaml'))
    args.sensor_config, args.sensor_check = None, False
    args.paper_mm, args.jog_speed, args.xy_step_mm = 0., 5., 10.
    return args


def main(argv=None):
    control = ProbeControl().start()
    node = source = None
    handlers = {}
    failure = None
    try:
        args = parse_args(sys.argv[1:] if argv is None else argv)
        options, homing = load_profile(args.profile)
        control.heartbeat_timeout_s, control.confirmation_timeout_s = homing.heartbeat_timeout_s, homing.confirmation_timeout_s
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, lambda *_: control.cancel('Probe process was stopped'))
        settings, motion = load_configs(args.config, args.dry_run)
        options, source = sensor_setup(args, options)
        control.check()
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = BridgeBedProbe(settings, motion, args, source, options, homing, control)
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
    control.emit('complete', 'Homing and mesh complete; arm returned to its starting pose', progress=1.,
                 current_point=len(node.points), total_points=len(node.points),
                 result={'output': str(node.output), 'simulated': node.config.dry_run, 'homed_z_mm': node.homed_z})


if __name__ == '__main__': main()
