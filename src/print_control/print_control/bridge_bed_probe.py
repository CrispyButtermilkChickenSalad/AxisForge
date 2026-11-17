"""Bridge-controlled Z homing, confirmed mesh placement, and ALPS bed probing."""
import argparse
import json
from dataclasses import dataclass, fields, replace
import math
from pathlib import Path
import signal
import sys
import threading
import time
import traceback

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
from print_control.load_cell_contact import ContactOptions, finite, thermal_contact_options
from print_control.probe_control import ProbeControl
from print_control.continuous_probe import ContinuousHoming
from print_control.load_cell_contact import ContactDetected
from print_control.probe_z_moves import ProbeZMoves
from print_control.ur_safety_limits import RobotSafetyChecker
from print_control.bed_frame import BedFrame
from print_control.probe_thermal import ProbeThermalPolicy
from print_control.probe_alignment import ProbeAlignment
from print_control.mesh_recovery import MeshFailureRecovery


@dataclass(frozen=True)
class HomingOptions:
    first_speed_mm_s: float = 5.
    second_speed_mm_s: float = .2
    # Accepted for compatibility with old profiles; continuous passes do not step.
    first_step_mm: float = .05
    second_step_mm: float = .01
    acceleration_mm_s2: float = 50.
    sample_age_s: float = .15
    cancel_timeout_s: float = .2
    max_stop_travel_mm: float = .5
    retract_mm: float = 3.
    max_descent_mm: float = 350.
    max_retest_difference_mm: float = .5
    clearance_above_start_mm: float = 10.
    lift_speed_mm_s: float = 10.
    # Optional legacy override; otherwise inherit AxisForge's XY travel speed.
    travel_speed_mm_s: float = None
    mesh_columns: int = 3
    mesh_rows: int = 3
    mesh_yaw_deg: float = 0.
    mesh_clearance_mm: float = 3.
    mesh_search_step_mm: float = .1
    confirmation_timeout_s: float = 300.
    heartbeat_timeout_s: float = 5.
    confirmation_pose_tolerance_mm: float = .1

    def __post_init__(self):
        if not all(finite(v) and v > 0 for k, v in vars(self).items()
                   if k not in ('travel_speed_mm_s','mesh_yaw_deg')):
            raise ValueError('Homing settings must be finite positive numbers')
        if not finite(self.mesh_yaw_deg) or not -180 <= self.mesh_yaw_deg <= 180:
            raise ValueError('Bed direction must be -180..180 degrees')
        if not 1 <= self.mesh_clearance_mm <= 10:
            raise ValueError('Mesh travel clearance must be 1..10 mm')
        if not .01 <= self.mesh_search_step_mm <= .1:
            raise ValueError('Mesh near-contact search steps must be 0.01..0.1 mm')
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
        if (not 1 <= self.retract_mm <= 5 or not 5 <= self.max_descent_mm <= 350 or
                self.max_descent_mm <= self.retract_mm):
            raise ValueError('Homing retract must be 1..5 mm, and descent must be 5..350 mm and exceed retract')
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


class BridgeBedProbe(ProbeAlignment, MeshFailureRecovery, ProbeZMoves, ContinuousHoming, LoadCellBedProbe):
    def probe_grid(self):
        return grid_points(self.settings.bed_size_mm, self.args.margin_mm,
                           self.homing.mesh_columns, self.homing.mesh_rows)

    def bed_point(self, xy, height=0.):
        frame=getattr(self,'bed_frame',None)
        if frame is None:return super().bed_point(xy,height)
        return np.r_[frame.to_base(xy),self.settings.z_offset+height]

    def gcode_xy(self, base):
        frame=getattr(self,'bed_frame',None)
        return super().gcode_xy(base) if frame is None else frame.to_gcode(np.asarray(base)[:2])

    def hover(self, xy):
        # Mesh-only clearance, independent of the printing entry/final lift and
        # the coarse-to-fine homing retract. Keep it above every measured high
        # spot before XY travel, including the just-measured, unsaved point.
        target = self.bed_point(xy, self.homing.mesh_clearance_mm)
        measured = [float(r['bed_z_mm']) for r in self.records.values()]
        measured.extend(float(r['nozzle'][2]) for r in getattr(self, '_point_readings', ()))
        if measured:
            target[2] = max(target[2], max(measured)+self.homing.mesh_clearance_mm)
        return target

    @property
    def xy_travel_speed(self):
        return min(self.settings.max_travel_speed_mm_s,
                   self.homing.travel_speed_mm_s or self.settings.max_travel_speed_mm_s)

    def retract_contact(self, *, distance_mm=None, speed_mm_s=None):
        # Includes the small retraction between every repeated mesh contact.
        return super().retract_contact(distance_mm=distance_mm,
            speed_mm_s=self.homing.lift_speed_mm_s if speed_mm_s is None else speed_mm_s)

    def __init__(self, settings, motion, args, source, options, homing, control):
        self._feedback_executor = None
        self.control, self.homing = control, homing
        self.monitor_robot = False
        self.robot_signals = {}
        self.returned = False
        self._home_established = self._mesh_phase_active = self._mesh_recovery_attempted = False
        self.failure_recovery = None
        self.start_xyz = self.start_joints = None
        self.homed_z = None
        self.homing_readings = []
        self.stop_client = None
        self.robot_safety_checker = None
        self._diagnostic_thread = None
        self.stop_unconfirmed = False
        self.files = {Path(p).resolve(): (Path(p).stat().st_mtime_ns, Path(p).stat().st_size)
                      for p in (args.config, args.profile)}
        if motion.kinematics_file:
            path = Path(motion.kinematics_file).expanduser().resolve()
            self.files[path] = (path.stat().st_mtime_ns, path.stat().st_size)
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
            self.attach_feedback_executor()
            self.robot_safety_checker = RobotSafetyChecker()

    def run_motion(self, motion):
        if not self.config.dry_run:
            # Applies to both continuous homing passes, mesh moves, retracts,
            # return travel, calibration searches and dashboard Z positioning.
            # Failure/timeout must occur before the action executor sees a goal.
            previous = self.control.state
            self.control.emit('checking_robot_limits',
                              'Checking samples along the move against the UR controller safety limits')
            self.robot_safety_checker.validate(motion, pump=self.pump,
                nozzle_fk=self.planner.nozzle_position,
                progress=lambda checked,total,elapsed:self.control.emit('checking_robot_limits',
                    'Robot safety checks: %d/%d positions (%.1f s)' % (checked,total,elapsed)))
            self.control.emit(previous, 'Robot safety check passed; executing '+motion.name)
        return super().run_motion(motion)

    def record_contact_failure(self, error):
        """Retain bounded observations after a stop attempt, including its failure."""
        if self.config.dry_run:
            return
        try:
            feedback=list(getattr(self, '_homing_feedback', ()))
            source=self.source
            with source.lock:
                samples=list(source.samples)[-2000:]
                health={'received_at':source.received_at,'health_at':source.health_at,'error':source.error}
            goal=getattr(self,'goal_handle',None)
            result=getattr(self,'result_future',None)
            acceptance=getattr(self,'goal_future',None)
            data={'error':str(error),'observed_at':time.monotonic(),
                  'stop_unconfirmed':bool(getattr(self,'stop_unconfirmed',False)),
                  'action':{'acceptance_done':None if acceptance is None else acceptance.done(),
                            'accepted':None if goal is None else goal.accepted,
                            'result_done':None if result is None else result.done(),
                            'feedback_received':getattr(self,'feedback_received',None)},
                  'context':getattr(self,'_continuous_homing',None),
                  'point_readings':list(getattr(self,'_point_readings',())),
                  'baseline_counts':None if self.detector is None else self.detector.baseline,
                  'sensor_health':health,
                  'joints':[vars(s) for s in feedback], 'alps':[vars(s) for s in samples]}
            path=self.output.with_suffix('.contact-failure.json')
            def write():
                try:
                    temporary=path.with_suffix('.json.tmp')
                    temporary.write_text(json.dumps(data,default=lambda x:x.tolist(),allow_nan=False))
                    temporary.replace(path)
                    print('Contact diagnostic saved to '+str(path),flush=True)
                except Exception as diagnostic_error:
                    print('Could not save contact diagnostic: '+str(diagnostic_error),flush=True)
            self._diagnostic_thread=threading.Thread(target=write,daemon=True)
            self._diagnostic_thread.start()
        except Exception as diagnostic_error:
            print('Could not save contact diagnostic: '+str(diagnostic_error),flush=True)

    def attach_feedback_executor(self):
        # All inherited wait/cancel paths use rclpy.spin_once's global executor.
        # Keep the node registered across those calls: repeatedly adding and
        # removing it can interrupt delivery while waiting for operator input.
        # No background spinner is used; callbacks still run on this thread.
        if self.config.dry_run or self._feedback_executor is not None:
            return
        executor = rclpy.get_global_executor()
        if executor.add_node(self):
            self._feedback_executor = executor

    def spin_feedback(self, timeout_sec):
        # Humble rebuilds its ready-callback iterator whenever timeout_sec
        # changes. Alternating the pump's 10 ms and wait_until's 20 ms can keep
        # selecting the first 500 Hz subscription and starve speed/action
        # feedback. Every probe wait uses one bounded 2 ms poll, preserving the
        # iterator and serving all ready entities without a background spinner.
        # Planning calls pump once per IK interval. One callback per interval
        # cannot keep up with separate 500 Hz joint and speed subscriptions.
        # Drain a bounded group while retaining the same iterator/timeout;
        # return to force and interruption checks within this short budget.
        deadline = time.monotonic()+.004
        for _ in range(8):
            rclpy.spin_once(self, timeout_sec=.002)
            if time.monotonic() >= deadline:
                break

    def destroy_node(self):
        writer=getattr(self,'_diagnostic_thread',None)
        if writer is not None:
            writer.join(timeout=1.)  # Shutdown only, after motion has been stopped.
        executor = getattr(self, '_feedback_executor', None)
        try:
            if executor is not None:
                executor.remove_node(self)
        finally:
            self._feedback_executor = None
            destroyed = super().destroy_node()
        return destroyed

    def robot_reason(self):
        if self.config.dry_run: return ''
        if not self.joints_fresh():
            age = 'missing' if self.joint_received is None else '%.3f s old' % (time.monotonic()-self.joint_received)
            return 'Robot joint feedback is stale ('+age+')'
        if not self.action.server_is_ready(): return 'UR trajectory controller is unavailable'
        if not self.robot_signals.get('program', (False, 0))[0]: return 'Start the UR External Control program'
        if self.robot_signals.get('safety', (None, 0))[0] != 1: return 'UR safety mode must be NORMAL'
        speed, stamp = self.robot_signals.get('speed', (None, 0))
        if time.monotonic()-stamp > .25: return 'UR speed-scaling feedback is stale'
        if not finite(speed) or not 99.9 <= speed <= 100.1: return 'Set UR speed scaling to 100% for probing'
        return ''

    def check_interrupted(self):
        self.control.check()
        policy=getattr(self,'thermal_policy',None)
        if policy is not None and not self.config.dry_run:policy.require_ready()
        self.check_continuous_homing()
        self.check_z_stop_travel()
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
        else: self.spin_feedback(.01)

    def capture_start(self, *, allow_alignment=False):
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
        # Only inspect a potential correction here; never move before confirmation.
        self._nominal_probe_rotation = self.planner.rotation.copy()
        self.prepare_start_alignment(joints,nozzle,enabled=allow_alignment)
        self.freeze_probe_orientation(joints)
        self.target, self.seed = nozzle.copy(), joints.copy()
        self.start_xyz, self.start_joints = nozzle.copy(), joints.copy()

    def check_start_orientation(self, joints, nozzle):
        """Explain an invalid starting pose without widening the motion limit."""
        try:
            self.planner.check_pose(joints, nozzle, nozzle, rotation=self._nominal_probe_rotation)
        except ValueError as error:
            if not str(error).startswith('Orientation error '):
                raise
            actual=self.planner.kinematics.eval(joints)[:3,:3]
            def angles(rotation):
                return ', '.join('%.3f' % value for value in
                    SE3.Rt(rotation,np.zeros(3)).rpy(unit='deg',order='zyx'))
            raise ValueError(str(error)+' (%.3f deg limit). Nozzle RPY in robot Base: [%s] deg; '
                'required: [%s] deg. Clear the nozzle and restore the upright orientation before retrying; '
                'The homing cycle can align errors up to 0.5 deg after its unloaded surface confirmation. '
                'Z descent itself does not rotate the tool.' %
                (self.config.orientation_tolerance_deg,angles(actual),angles(self._nominal_probe_rotation))) from error

    def freeze_probe_orientation(self, joints):
        """Keep position and fixed orientation anchored to the same measured pose."""
        rotation = self.planner.kinematics.eval(joints)[:3, :3]
        actual_rpy = tuple(float(v) for v in SE3.Rt(rotation, np.zeros(3)).rpy(unit='deg', order='zyx'))
        self.config = replace(self.config, nozzle_rpy_deg=actual_rpy)
        self.planner.config, self.planner.rotation = self.config, rotation.copy()
        # sampled_line derives rotations from its settings, not only .rotation.
        # Preserve the measured tool orientation on subsequent Z/XY moves too.
        self.settings = replace(self.settings, nozzle_roll_deg=actual_rpy[0],
                                nozzle_pitch_deg=actual_rpy[1], wrist_angle=actual_rpy[2])
        self.planner.settings, self.planner.orientation = self.settings, (0., 0., 0.)

    def emit_stage(self, state, message, **values):
        options = self.contact_options
        self.control.emit(state, message, total_points=len(self.points), current_point=len(self.records),
                          max_descent_mm=self.homing.max_descent_mm,
                          trigger_source=options.trigger_source,
                          trigger_force_g=getattr(self.source, 'native_trigger_force_g', None),
                          trigger_mode=options.trigger_mode, trigger_value=options.threshold,
                          baseline_noise_fraction=options.baseline_noise_fraction,
                          trigger_unit=options.unit, max_value=options.maximum,
                          position_mm=rounded(self.target) if self.target is not None else None,
                          **values)

    def confirm_position(self, stage, message, **values):
        expected, joints = self.z_waiting_anchor()
        def guarded_pump():
            self.pump()
            nozzle, current = self.nozzle_now()
            if (np.linalg.norm(nozzle-expected) > self.homing.confirmation_pose_tolerance_mm or
                    np.max(np.abs(current-joints)) > math.radians(.1)):
                raise RuntimeError('Robot moved while awaiting confirmation; restart homing from the new pose')
        self.control.wait_confirmation(stage, message, guarded_pump,
            alignment=getattr(self,'start_alignment',None),
            trigger_source=self.contact_options.trigger_source,
            trigger_force_g=getattr(self.source, 'native_trigger_force_g', None),
            position_mm=rounded(expected), total_points=len(self.points), current_point=len(self.records),
            max_descent_mm=self.homing.max_descent_mm,
            trigger_mode=self.contact_options.trigger_mode, trigger_value=self.contact_options.threshold,
            baseline_noise_fraction=self.contact_options.baseline_noise_fraction,
            trigger_unit=self.contact_options.unit, max_value=self.contact_options.maximum, **values)
        nozzle, current = self.z_waiting_anchor()
        if (np.linalg.norm(nozzle-expected) > self.homing.confirmation_pose_tolerance_mm or
                np.max(np.abs(current-joints)) > math.radians(.1)):
            raise RuntimeError('Robot moved while awaiting confirmation; restart homing from the new pose')
        self.planner.check_pose(current, nozzle, nozzle)
        if stage in ('surface', 'scale'):
            # Confirmation refreshes the measured joint seed. Freeze its matching
            # orientation too: correcting even a few thousandths of a degree
            # from an older sample in the first 10 ms can exceed joint acceleration
            # limits. Keep the upright/movement checks and never re-anchor mid-mesh.
            if stage=='surface' and self.alignment_pending():
                self.check_alignment_candidate(current,nozzle)
            else:
                self.check_start_orientation(current, nozzle)
            self.freeze_probe_orientation(current)
        self.target, self.seed = nozzle.copy(), current.copy()
        if stage=='surface':
            self._surface_confirmed=True
            self.start_xyz,self.start_joints=nozzle.copy(),current.copy()

    def move_z(self, height, name, speed):
        self.line_to(np.array([*self.target[:2], height]), name, speed)

    def _run_routine(self):
        policy=getattr(self,'thermal_policy',None)
        if policy is not None and not self.config.dry_run:policy.require_ready()
        self.capture_start(allow_alignment=True)
        self.confirm_position('surface',
            'Position the nozzle above the CENTRE of the bed. This position defines the mesh centre. '
            'Confirm a flat build surface suitable for the selected probing temperature is below it, with the nozzle clean '
            'and clear of the surface (unloaded), with room around the toolhead for a small rotation. '
            'If needed, confirmation first aligns the nozzle upright by up to 0.5 deg around its tip, '
            'then enables dashboard Z positioning; '
            'Continue then starts continuous vertical homing '
            'at %g mm/s, stopping at contact or a maximum descent of %g mm, followed by one '
            'precision contact at %g mm/s, with acceleration ramps.' %
            (self.homing.first_speed_mm_s, self.homing.max_descent_mm,
             self.homing.second_speed_mm_s), progress=0.)
        # In raw mode this is the first tare, made only after explicit unloaded confirmation.
        self.set_baseline()
        self.align_homing_start()
        self.manual_z_session()
        self.bed_frame=BedFrame(tuple(self.start_xyz[:2]),tuple(self.settings.bed_size_mm),self.homing.mesh_yaw_deg)
        floor = self.start_xyz[2]-self.homing.max_descent_mm
        self.emit_stage('homing_fast', 'Finding the surface with continuous Z motion at %g mm/s' %
                        self.homing.first_speed_mm_s, progress=.03)
        first = self.homing_touch(floor_z=floor, speed_mm_s=self.homing.first_speed_mm_s,
                                  locating_only=True)
        self.retract_contact(distance_mm=self.homing.retract_mm)
        self.set_baseline()
        self.emit_stage('homing_slow', 'Precision contact 1 of 1 at %g mm/s' %
                        self.homing.second_speed_mm_s, progress=.08)
        selected = self.homing_touch(
            floor_z=max(floor, first['nozzle'][2]-self.homing.max_retest_difference_mm),
            speed_mm_s=self.homing.second_speed_mm_s,
            precision_reference_z=float(first['nozzle'][2]))
        self.retract_contact(distance_mm=self.homing.retract_mm)
        # The fast touch locates the surface approximately. Never average its
        # speed-dependent stopping deflection into the accurate bed height.
        self.homing_readings = [first, selected]
        self.homing_repeatability_mm = None  # One fine contact has no repeat spread.
        self.homed_z = float(selected['nozzle'][2])+self.contact_options.contact_offset_mm
        self.settings = replace(self.settings, z_offset=self.homed_z)
        # Both homing passes and their unload checks succeeded. The confirmed
        # starting pose is now the recovery destination for subsequent faults.
        self._home_established = self._mesh_phase_active = True
        clearance = float(self.start_xyz[2])+self.homing.clearance_above_start_mm
        self.emit_stage('moving_front_left', 'Lifting before moving to the first mesh point',
                        progress=.12, homed_z_mm=self.homed_z, clearance_z_mm=clearance)
        self.move_z(clearance, 'homing clearance lift', self.homing.lift_speed_mm_s)
        first_xy = self.bed_point(self.points[0])[:2]
        self.line_to(np.array([*first_xy, clearance]), 'front-left travel', self.xy_travel_speed)
        self.confirm_position('front_left',
            'Confirm the nozzle is above the first mesh point and this is the front-leftmost point of the mesh. '
            'Confirmation starts the automatic grid scan.', progress=.15,
            first_point=rounded(self.points[0]), homed_z_mm=self.homed_z, clearance_z_mm=clearance,
            bed_center_base_mm=rounded(self.start_xyz[:2]),
            first_point_offset_mm=rounded(first_xy-self.start_xyz[:2]),bed_yaw_deg=self.homing.mesh_yaw_deg)
        # Lower only at the confirmed XY, into known clearance above the homed plane.
        self.move_z(float(self.hover(self.points[0])[2]), 'mesh hover', self.homing.first_speed_mm_s)
        for index in range(len(self.points)):
            if index != self.index: self.go_to(index)
            self.emit_stage('meshing', 'Probing mesh point %d of %d' % (index+1, len(self.points)),
                            progress=.15+.75*index/len(self.points))
            self.probe_current()
        self.emit_stage('returning', 'Returning to the saved starting pose', progress=.93)
        self._mesh_phase_active = False  # Never retry a failed return automatically.
        self.return_to_start()
        self.returned = True
        self.save()

    def mesh_touch(self):
        # Continuous travel followed by bounded precision landings near contact.
        floor = self.bed_point(self.points[self.index])[2]-self.args.floor_mm
        reference=getattr(self,'_point_reference_z',None)
        if reference is not None:
            # A repeat at the same XY need not search several millimetres below
            # its already measured surface. This shortens planning/controller
            # checks while retaining every safety sample and the original floor.
            floor=max(floor,reference-self.homing.max_retest_difference_mm)
        reading = self.homing_touch(floor_z=floor, speed_mm_s=self.contact_options.speed_mm_s,
            precision_reference_z=(float(reference) if reference is not None else self.homed_z),
            allow_approach_locator=reference is None,
            near_contact_step_mm=self.homing.mesh_search_step_mm if reference is None else None)
        if self.config.dry_run or reading.get('contact_method') not in ('continuous_cancel','stepped_locator_cancel'):
            return reading
        # An unknown mesh point can be higher than the centre's precision band.
        # A cancelled approach is a locator, not a comparable precision sample.
        # Permit one refinement at a new point; never silently discard a failed
        # precision reading or retry a changing surface indefinitely.
        if reference is not None:
            raise RuntimeError('Surface reached before the precision band on a repeat; point rejected')
        locator = reading
        print('Point %d: locating contact at Z %.5f mm; locating only, '
              'then taking the configured precision contacts' %
              (self.index+1, locator['nozzle'][2]), flush=True)
        self.retract_contact()
        self.set_baseline()
        reading = self.homing_touch(
            floor_z=max(floor,float(locator['nozzle'][2])-self.homing.max_retest_difference_mm),
            speed_mm_s=self.contact_options.speed_mm_s,
            precision_reference_z=float(locator['nozzle'][2]))
        if reading.get('contact_method') != 'bounded_precision_steps':
            raise RuntimeError('Surface changed during precision refinement; point rejected')
        reading['locator_contact'] = {'nozzle_base_mm': rounded(locator['nozzle']),
            'load':float(locator['load']), 'tare_counts':float(locator['tare_counts']),
            'contact_method':locator['contact_method']}
        return reading

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
                        self.return_surface_height()+self.config.clearance_mm,
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
        if getattr(self,'bed_frame',None) is not None:data['bed_frame']=self.bed_frame.as_dict()
        data['mesh_shape'] = {'columns': self.homing.mesh_columns, 'rows': self.homing.mesh_rows}
        if getattr(self,'start_alignment',None) is not None:
            data['start_alignment']=dict(self.start_alignment)
        data['upright_tolerance_deg'] = min(.2, self.config.orientation_tolerance_deg)
        data['homing'] = {'starting_nozzle_base_mm': rounded(self.start_xyz), 'homed_z_mm': self.homed_z,
                          'starting_nozzle_rpy_deg': list(self.config.nozzle_rpy_deg),
                          'returned_to_start': self.returned,
                          'mesh_clearance_mm': self.homing.mesh_clearance_mm,
                          'mesh_search_step_mm': self.homing.mesh_search_step_mm,
                          'fine_repeatability_mm': getattr(self, 'homing_repeatability_mm', None),
                          'contacts': [{'nozzle_base_mm': rounded(r['nozzle']), 'load': float(r['load']),
                                        'unit': self.contact_options.unit,
                                        'contact_method': r.get('contact_method', 'continuous_cancel'),
                                        'contact_persistence_checked': r.get('contact_persistence_checked', True),
                                        **{k: float(r[k]) for k in ('speed_mm_s', 'stop_ack_s', 'stop_travel_mm')
                                           if k in r}} for r in self.homing_readings]}
        policy=getattr(self,'thermal_policy',None)
        if policy is not None:data['probing_temperature']=policy.status()
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
                self.spin_feedback(.02)
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
    parser.add_argument('--probe-nozzle-target', type=float, default=0.)
    parser.add_argument('--probe-bed-target', type=float, default=0.)
    parser.add_argument('--probe-soak-s', type=float, default=60.)
    parser.add_argument('--calibration-max-counts', type=float, default=120000.,
                        help='Scale calibration only: maximum raw-count excursion')
    parser.add_argument('--calibration-hot-target', type=float,
                        help='Scale calibration only: explicitly selected stable hot-nozzle target in C')
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
        policy=ProbeThermalPolicy({'nozzle_target_c':args.probe_nozzle_target,
            'bed_target_c':args.probe_bed_target,'soak_s':args.probe_soak_s})
        options=thermal_contact_options(options,hot_nozzle='extruder' in policy.targets)
        options, source = sensor_setup(args, options, status_validator=policy.validate_status,
            check_baseline=False, native_thermal_targets=policy.targets)
        control.check()
        if not args.dry_run:policy.wait_ready(source,control)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = BridgeBedProbe(settings, motion, args, source, options, homing, control)
        node.thermal_policy=policy
        control.emit('starting','Temperature checks passed; checking robot before surface confirmation',thermal=policy.status())
        node.run_routine()
    except Exception as error:
        failure = str(error)
        # Preserve the failing planning/feedback call site in process logs.
        # The dashboard message stays concise; no additional motion is sent.
        traceback.print_exc()
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
                     failure_recovery=getattr(node, 'failure_recovery', None),
                     stop_unconfirmed=bool(node is not None and node.stop_unconfirmed))
        raise SystemExit(1)
    control.emit('complete', 'Homing and mesh complete; arm returned to its starting pose', progress=1.,
                 current_point=len(node.points), total_points=len(node.points),
                 result={'output': str(node.output), 'simulated': node.config.dry_run, 'homed_z_mm': node.homed_z})


if __name__ == '__main__': main()
