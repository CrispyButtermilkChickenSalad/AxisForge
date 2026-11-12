"""Master schedule contracts, rotary timing and heater policy."""

import math
import unittest

from klipper_extras.axisforge_schedule import compile_job, digest, progress

LIMITS = {'linear_velocity':20.,'linear_acceleration':20.,'angular_velocity':10.,
          'angular_acceleration':20.,'extrusion_velocity':20.,'extrusion_acceleration':100.,
          'min_extrude_temp':170.,'pressure_advance':.04,'max_extrude_ratio':5.,
          'max_extrude_distance':50.,'chunk_seconds':2.}
HEATERS = {'extruder':{'min_temp':0.,'max_temp':300.},'heater_bed':{'min_temp':0.,'max_temp':120.}}
ENTRY = 'G21\nG90\nG0 X110 Y110 Z5 F600\n'


class ScheduleTests(unittest.TestCase):
    def test_shared_progress_for_xyz_abc_and_extrusion(self):
        plan = compile_job('M109 S200\n'+ENTRY+'G1 X120 A5 B-3 C2 E1\n',LIMITS,HEATERS)
        move = plan['operations'][-1]['moves'][0]
        p = move['profile']
        u,v = progress(p,p['duration_s']/2)
        self.assertAlmostEqual(u,.5)
        self.assertGreater(v,0)
        self.assertEqual(progress(p,p['duration_s']), (1.,0.))
        self.assertEqual(move['end'],[120.,110.,5.,5.,-3.,2.])
        self.assertEqual(move['delta_e'],1.)
        self.assertAlmostEqual(p['velocity']*(p['ramp_s']+p['cruise_s']),1.)
        self.assertEqual(plan['plan_hash'],digest({k:v for k,v in plan.items() if k!='plan_hash'}))

    def test_rotation_only_has_nonzero_time(self):
        plan = compile_job(ENTRY+'G1 A5\nG1 A0\n',LIMITS,HEATERS)
        moves = [m for op in plan['operations'] if op['kind']=='chunk' for m in op['moves']]
        self.assertEqual(len(moves),2)
        self.assertGreater(plan['duration_s'],0.)
        self.assertEqual(moves[0]['start'][:3],moves[0]['end'][:3])

    def test_linear_modes_do_not_scale_or_accumulate_angles(self):
        plan = compile_job(ENTRY+'G1 A5\nG91\nG20\nG1 X.1 A2\n',LIMITS,HEATERS)
        move = [m for o in plan['operations'] if o['kind']=='chunk' for m in o['moves']][-1]
        self.assertAlmostEqual(move['end'][0],112.54)
        self.assertEqual(move['end'][3],2.)

    def test_heater_peaks_and_ordered_shutdown(self):
        plan = compile_job('M140 S60\nM104 S200\n'+ENTRY+'G1 X111 E.1\nM109 S210\nG1 X112 E.2\nM104 S0\n',LIMITS,HEATERS)
        self.assertEqual(plan['heater_targets'],{'heater_bed':60.,'extruder':210.})
        self.assertEqual(plan['operations'][-1]['target'],0.)
        self.assertEqual([op['kind'] for op in plan['operations']],
                         ['heater','heater','chunk','heater','chunk','heater'])

    def test_dangerous_or_unknown_input_is_rejected_before_heating(self):
        for text in ('M104 S301\n', 'M109 S-1\n', 'START_PRINT TEMP=200\n',
                     'M104 S200\nG2 X10\n', 'M104 S200 T1\n', 'M104 SNaN\n'):
            with self.subTest(text=text),self.assertRaises(ValueError):
                compile_job(text+ENTRY+'G1 X111\n',LIMITS,HEATERS)

    def test_extrusion_requires_an_explicit_prior_target(self):
        for text in ('', 'M104 S150\n', 'M104 S200\nM104 S0\n'):
            with self.subTest(text=text),self.assertRaisesRegex(ValueError,'Extrusion requires'):
                compile_job(text+ENTRY+'G1 X111 E1\nM109 S200',LIMITS,HEATERS)

    def test_relative_extrusion_flow_and_reset(self):
        plan = compile_job('M104 S200\n'+ENTRY+'M83\nM221 S50\nG1 X111 E2\nG92 E0\nG1 X112 E2\n',LIMITS,HEATERS)
        moves = [m for o in plan['operations'] if o['kind']=='chunk' for m in o['moves']]
        self.assertEqual([m['delta_e'] for m in moves],[1.,1.])
        self.assertEqual([m['e_start'] for m in moves],[0.,1.])

    def test_pressure_advance_must_match_host(self):
        with self.assertRaisesRegex(ValueError,'Pressure advance'):
            compile_job('SET_PRESSURE_ADVANCE ADVANCE=1\n'+ENTRY+'G1 X111',LIMITS,HEATERS)

    def test_missing_bed_is_allowed_only_for_off_command(self):
        heaters = {'extruder':HEATERS['extruder']}
        compile_job('M140 S0\n'+ENTRY+'G1 X111',LIMITS,heaters)
        with self.assertRaisesRegex(ValueError,'not configured'):
            compile_job('M140 S60\n'+ENTRY+'G1 X111',LIMITS,heaters)

    def test_disabled_missing_bed_commands_are_omitted_from_execution(self):
        plan = compile_job('M140 S60\nM190 S60\nM104 S200\n'+ENTRY+'G1 X111 E.1\nM140 S0',
                           LIMITS, {'extruder':HEATERS['extruder']}, {'heater_bed':False})
        self.assertEqual(plan['ignored_heaters'], ['heater_bed'])
        self.assertEqual(plan['requested_heater_targets']['heater_bed'], 60.)
        self.assertNotIn('heater_bed',plan['heater_targets'])
        self.assertFalse(any(o.get('heater')=='heater_bed' for o in plan['operations']))
        self.assertTrue(any('ignored for this job' in w for w in plan['warnings']))

    def test_disabled_check_keeps_configured_heater_commands_and_changes_hash(self):
        text = 'M140 S60\nM109 S200\n'+ENTRY+'G1 X111 E.1'
        checked = compile_job(text,LIMITS,HEATERS)
        skipped = compile_job(text,LIMITS,HEATERS,{'extruder':False,'heater_bed':False})
        self.assertEqual(checked['operations'],skipped['operations'])
        self.assertNotEqual(checked['plan_hash'],skipped['plan_hash'])

    def test_disabled_check_does_not_bypass_target_or_cold_extrusion_checks(self):
        for text in ('M104 S301\n','M104 S-1\n','M104 S150\n','M140 S121\nM104 S200\n'):
            with self.subTest(text=text),self.assertRaises(ValueError):
                compile_job(text+ENTRY+'G1 X111 E.1',LIMITS,HEATERS,
                            {'extruder':False,'heater_bed':False})

    def test_heater_options_reject_unknown_names_and_non_booleans(self):
        for options in ({'bed':False},{'heater_bed':'false'},{'extruder':0},[],False):
            with self.subTest(options=options),self.assertRaisesRegex(ValueError,'heater_checks'):
                compile_job(ENTRY+'G1 X111',LIMITS,HEATERS,options)

    def test_profile_limits_and_phase_integral(self):
        plan = compile_job('M109 S200\n'+ENTRY+'G1 X115 A30 E5 F1200',LIMITS,HEATERS)
        move = plan['operations'][-1]['moves'][0]
        p = move['profile']
        for d,v,a in ((5,20,20),(30,10,20),(5,20,100)):
            self.assertLessEqual(p['velocity']*d,v+1e-8)
            self.assertLessEqual(p['acceleration']*d,a+1e-8)
        with self.assertRaises(ValueError): progress(p,-1.)
        with self.assertRaises(ValueError): progress(p,math.inf)


if __name__=='__main__': unittest.main()
