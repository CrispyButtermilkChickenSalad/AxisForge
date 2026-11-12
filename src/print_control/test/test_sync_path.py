"""Timed robot conversion must preserve the master's times and nozzle path."""

import copy
import math
import unittest
from pathlib import Path

import numpy as np

from print_control.sync_path import TimedPath, canonical_hash, sample_times, validate_plan
from print_control.motion_planner import cubic_joint_state, duration_parts


def plan():
    value = {'protocol':'axisforge.sync.v1','coordinate_frame':'gcode_bed',
             'entry':[110.,110.,5.,0.,0.,0.],
             'operations':[{'kind':'chunk','index':0,'duration_s':2.,'moves':[
                 {'start':[110.,110.,5.,0.,0.,0.], 'end':[111.,110.,5.,2.,-2.,2.],
                  'e_start':0.,'delta_e':0.,'offset_s':0.,'source_line':2,
                  'profile':{'acceleration':1.,'velocity':1.,'ramp_s':1.,'cruise_s':0.,'duration_s':2.}}
             ]}]}
    value['plan_hash']=canonical_hash(value)
    return value


class TimedPathTests(unittest.TestCase):
    def test_exact_master_times_and_rotated_tip(self):
        config = Path(__file__).resolve().parents[2]/'axisforge_bringup/config/ur_master.yaml'
        compiled = TimedPath(config,plan(),np.deg2rad([-3.73,-90,-120,-60,90,0]))
        motion = compiled.chunks[0]
        self.assertEqual(motion.times[0],0.)
        self.assertEqual(motion.times[-1],2.)
        np.testing.assert_allclose(motion.velocities[[0,-1]],0.)
        expected = compiled.planner.transform([111,110,5])
        np.testing.assert_allclose(compiled.planner.nozzle_position(motion.positions[-1]),expected,atol=.01)

    def test_hash_and_profile_integrity(self):
        modified=plan()
        modified['operations'][0]['duration_s']=10.
        with self.assertRaisesRegex(ValueError,'hash'): validate_plan(modified)
        modified['plan_hash']=canonical_hash({k:v for k,v in modified.items() if k!='plan_hash'})
        with self.assertRaisesRegex(ValueError,'duration'): validate_plan(modified)

    def test_too_fast_is_rejected_instead_of_retimed(self):
        value=plan()
        p=value['operations'][0]['moves'][0]['profile']
        p.update(acceleration=10000.,velocity=100.,ramp_s=.01,duration_s=.02)
        value['operations'][0]['duration_s']=.02
        value['plan_hash']=canonical_hash({k:v for k,v in value.items() if k!='plan_hash'})
        config=Path(__file__).resolve().parents[2]/'axisforge_bringup/config/ur_master.yaml'
        with self.assertRaisesRegex(ValueError,r'(speed|acceleration).*Klipper.*axisforge_sync'):
            TimedPath(config,value,np.deg2rad([-3.73,-90,-120,-60,90,0]))

    def test_short_triangular_move_with_roundoff_cruise_keeps_master_timing(self):
        # The line-57 connector: 0.543939 mm at 20 mm/s^2. Float subtraction
        # can leave a positive cruise lasting only one floating-point ULP.
        value=plan()
        start=[136.65,130.55,.24,0.,0.,0.]
        end=[136.65,130.006061,.24,0.,0.,0.]
        accel=20./abs(end[1]-start[1])
        velocity=math.sqrt(accel)
        ramp=velocity/accel
        cruise=float(np.spacing(ramp))
        duration=2*ramp+cruise
        value['entry']=start
        chunk=value['operations'][0]
        chunk['duration_s']=duration
        move=chunk['moves'][0]
        move.update(start=start,end=end,source_line=57,
                    profile={'acceleration':accel,'velocity':velocity,'ramp_s':ramp,
                             'cruise_s':cruise,'duration_s':duration})
        value['plan_hash']=canonical_hash({k:v for k,v in value.items() if k!='plan_hash'})
        config=Path(__file__).resolve().parents[2]/'axisforge_bringup/config/ur_master.yaml'
        compiled=TimedPath(config,value,np.deg2rad([-3.73,-90,-120,-60,90,0]))
        motion=compiled.chunks[0]
        self.assertEqual(motion.times[0],0.)
        self.assertEqual(motion.times[-1],duration)
        stamps=[duration_parts(float(t)) for t in motion.times]
        self.assertTrue(all(a<b for a,b in zip(stamps,stamps[1:])))
        for index,dt in enumerate(np.diff(motion.times)):
            for fraction in (0.,1.):
                _,_,acceleration=cubic_joint_state(*motion.positions[index:index+2],
                    *motion.velocities[index:index+2],dt,fraction)
                self.assertLessEqual(np.max(abs(np.rad2deg(acceleration))),
                                     compiled.planner.config.max_joint_acceleration_deg_s2)
        np.testing.assert_allclose(compiled.planner.nozzle_position(motion.positions[-1]),
                                   compiled.planner.transform(end[:3]),atol=.01)

    def test_sampling_preserves_resolvable_phase_boundaries(self):
        profile={'ramp_s':.12,'cruise_s':.03,'duration_s':.27}
        times=sample_times(profile)
        self.assertEqual(times[0],0.)
        self.assertEqual(times[-1],.27)
        self.assertIn(.12,times)
        self.assertIn(.15,times)
        self.assertGreaterEqual(np.min(np.diff(times)),1e-9)
        self.assertLessEqual(np.max(np.diff(times)),.02+1e-12)

    def test_motion_shorter_than_timestamp_resolution_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'too short'):
            sample_times({'ramp_s':1e-10,'cruise_s':0.,'duration_s':2e-10})

    def test_no_double_bed_transform(self):
        value=plan()
        value['coordinate_frame']='klipper_compensated'
        value['plan_hash']=canonical_hash({k:v for k,v in value.items() if k!='plan_hash'})
        with self.assertRaisesRegex(ValueError,'coordinates'): validate_plan(value)


if __name__=='__main__': unittest.main()
