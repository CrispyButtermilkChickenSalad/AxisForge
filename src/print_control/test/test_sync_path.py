"""Timed robot conversion must preserve the master's times and nozzle path."""

import copy
import unittest
from pathlib import Path

import numpy as np

from print_control.sync_path import TimedPath, canonical_hash, validate_plan


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
        with self.assertRaisesRegex(ValueError,'speed|acceleration'):
            TimedPath(config,value,np.deg2rad([-3.73,-90,-120,-60,90,0]))

    def test_no_double_bed_transform(self):
        value=plan()
        value['coordinate_frame']='klipper_compensated'
        value['plan_hash']=canonical_hash({k:v for k,v in value.items() if k!='plan_hash'})
        with self.assertRaisesRegex(ValueError,'coordinates'): validate_plan(value)


if __name__=='__main__': unittest.main()
