"""Small, explicitly confirmed nozzle-centred alignment before Z homing."""
from dataclasses import replace
import math

import numpy as np
from spatialmath import SE3

MAX_ALIGNMENT_DEG = .5
ALIGNMENT_DURATION_S = 3.
ALIGNMENT_PATH_MM = .0025
ALIGNMENT_RESULT_DEG = .05


def rotation_error_deg(actual, target):
    return math.degrees(math.acos(float(np.clip((np.trace(target.T @ actual)-1)/2,-1,1))))


def rotation_rpy(rotation):
    return [float(value) for value in SE3.Rt(rotation,np.zeros(3)).rpy(unit='deg',order='zyx')]


class ProbeAlignment:
    def prepare_start_alignment(self, joints, nozzle, *, enabled):
        """Inspect only. The first surface confirmation must precede any motion."""
        self._surface_confirmed = False
        self._alignment_attempted = False
        self.start_alignment = None
        actual = self.planner.kinematics.eval(joints)[:3,:3]
        angle = rotation_error_deg(actual,self._nominal_probe_rotation)
        if not enabled:
            self.check_start_orientation(joints,nozzle)
            return
        if angle > MAX_ALIGNMENT_DEG:
            raise ValueError('Starting orientation error %.4f deg exceeds the %.1f deg automatic alignment limit; '
                             'reposition the nozzle closer to upright before homing' % (angle,MAX_ALIGNMENT_DEG))
        # Check position against the measured rotation. This only permits
        # preparation/confirmation; descending still requires upright checks.
        self.planner.check_pose(joints,nozzle,nozzle,rotation=actual)
        required = angle > min(ALIGNMENT_RESULT_DEG,self.config.orientation_tolerance_deg)
        self.start_alignment = dict(required=required,state='pending' if required else 'not_needed',
            angle_deg=angle,maximum_deg=MAX_ALIGNMENT_DEG,duration_s=ALIGNMENT_DURATION_S,
            target_rpy_deg=rotation_rpy(self._nominal_probe_rotation),
            pivot_nozzle_mm=[float(v) for v in nozzle],path_tolerance_mm=ALIGNMENT_PATH_MM)

    def alignment_pending(self):
        return bool(getattr(self,'start_alignment',None) and self.start_alignment['required']
                    and self.start_alignment['state']=='pending')

    def check_alignment_candidate(self, joints, nozzle):
        self.planner.check_joints(joints)
        actual=self.planner.kinematics.eval(joints)[:3,:3]
        angle=rotation_error_deg(actual,self._nominal_probe_rotation)
        if angle>MAX_ALIGNMENT_DEG:
            raise RuntimeError('Nozzle moved outside the 0.5 deg alignment range; restart from a closer upright pose')
        self.planner.check_pose(joints,nozzle,nozzle,rotation=actual)
        return angle

    def align_homing_start(self):
        alignment=getattr(self,'start_alignment',None)
        if not alignment or not alignment['required']:
            return
        if (not getattr(self,'_surface_confirmed',False) or
                getattr(self,'_alignment_attempted',False) or not self.alignment_pending()):
            raise RuntimeError('Upright alignment requires a fresh unloaded surface confirmation')
        self._alignment_attempted=True
        try:
            self._z_require_stationary()
            xyz,joints=self.z_waiting_anchor()
            if (np.linalg.norm(xyz-self.start_xyz)>self.homing.confirmation_pose_tolerance_mm or
                    np.max(np.abs(joints-self.start_joints))>math.radians(.1)):
                raise RuntimeError('Robot moved from the confirmed alignment position')
            angle=self.check_alignment_candidate(joints,xyz)
            if self.detector is None or not self._z_unloaded(self._z_samples()):
                raise RuntimeError('Upright alignment requires an unloaded nozzle and a valid ALPS baseline')
            # Use one fresh measured pose for the start rotation, joint seed and
            # pivot; do not command a jump back to an older feedback sample.
            self.freeze_probe_orientation(joints)
            self.target,self.seed=xyz.copy(),joints.copy()
            delta=(np.asarray(rotation_rpy(self._nominal_probe_rotation))-
                   np.asarray(self.config.nozzle_rpy_deg)+180.)%360.-180.
            alignment.update(state='aligning',angle_deg=angle,
                             pivot_nozzle_mm=[float(v) for v in xyz])
            self.emit_stage('aligning_nozzle','Aligning upright around the unloaded nozzle tip before Z homing',
                            alignment=dict(alignment),progress=0.)
            # A non-probing rotation uses the raw contact guard, including when
            # native MCU probing is configured. It never arms the native probe.
            self.seeking=True
            self._z_manual_active=True
            try:
                original=self.planner.config
                try:
                    self.planner.config=replace(original,path_tolerance_mm=min(ALIGNMENT_PATH_MM,original.path_tolerance_mm))
                    motion=self.planner.sampled_line('pre-homing upright alignment',xyz,xyz,joints,1.,
                        end_orientation_deg=delta,minimum_duration_s=ALIGNMENT_DURATION_S,pump=self.pump)
                    self.planner.check_pose(motion.positions[-1],xyz,xyz,rotation=self._nominal_probe_rotation)
                finally:
                    self.planner.config=original
                if not self._z_unloaded(self._z_samples()):
                    raise RuntimeError('Nozzle is no longer unloaded after alignment planning')
                self.run_motion(motion)  # Full UR safety queries and guarded action execution.
            finally:
                self.seeking=False
                self._z_manual_active=False
            self._z_require_stationary()
            after,current=self.z_waiting_anchor()
            residual=rotation_error_deg(self.planner.kinematics.eval(current)[:3,:3],self._nominal_probe_rotation)
            shift=float(np.linalg.norm(after-xyz))
            if shift>min(.1,self.homing.confirmation_pose_tolerance_mm):
                raise RuntimeError('Nozzle shifted beyond the stationary alignment bound')
            if residual>min(ALIGNMENT_RESULT_DEG,self.config.orientation_tolerance_deg):
                raise RuntimeError('Nozzle did not reach the upright alignment target')
            if not self._z_unloaded(self._z_samples()):
                raise RuntimeError('Nozzle load did not remain clear after upright alignment')
            self.check_start_orientation(current,after)
            self.freeze_probe_orientation(current)
            self.target,self.seed=after.copy(),current.copy()
            # Completion/return uses the corrected pose, never the old tilted Home.
            self.start_xyz,self.start_joints=after.copy(),current.copy()
            alignment.update(required=False,state='aligned',residual_deg=residual,
                             measured_shift_mm=shift,duration_s=float(motion.times[-1]))
            self.emit_stage('aligning_nozzle','Upright alignment verified; enabling Z positioning',
                            alignment=dict(alignment),progress=0.)
        except Exception as error:
            alignment['state']='failed'
            # Do not enter the general contact-recovery path from a failed
            # rotation. Main cleanup cancels/Stops before the routine can exit.
            raise RuntimeError('Pre-homing alignment failed; Z homing was not started: '+str(error)) from error
