"""Junction policy, geometry, material conservation and option contracts."""
import math
import unittest
from klipper_extras.axisforge_corners import options, junction_limit
from klipper_extras.axisforge_schedule import compile_job
from test_schedule import LIMITS, HEATERS, ENTRY


class CornerTests(unittest.TestCase):
    def plan(self,text,policy,**values):
        return compile_job('M109 S200\n'+ENTRY+text,
            dict(LIMITS,continuous_motion=True,extrusion_corner_velocity=1.),HEATERS,
            motion_options=dict(policy=policy,**values))

    def moves(self,plan):
        return [m for op in plan['operations'] if op['kind']=='chunk' for m in op['moves']]

    def test_square_corner_speed_and_equivalent_jd(self):
        klipper=options({'policy':'klipper','square_corner_velocity':2.})
        marlin=options({'policy':'marlin','junction_deviation_mm':4.*(math.sqrt(2)-1)/20})
        self.assertAlmostEqual(junction_limit(0,100,100,20,20,klipper,20),2.)
        for angle in (15,45,90,135,170):
            cosine=math.cos(math.radians(angle))
            self.assertAlmostEqual(junction_limit(cosine,100,100,20,20,klipper,20),
                                   junction_limit(cosine,100,100,20,20,marlin,20))
        self.assertEqual(junction_limit(-1,1,1,20,20,klipper,20),0.)

    def test_short_segment_policies_apply_their_distinct_limits(self):
        k=options({'policy':'klipper','square_corner_velocity':20.})
        m=options({'policy':'marlin','junction_deviation_mm':1.})
        self.assertAlmostEqual(junction_limit(0,1,.01,20,20,k,20),math.sqrt(.1))
        cosine=math.cos(math.radians(30))
        self.assertAlmostEqual(junction_limit(cosine,100,.01,20,20,m,20),math.sqrt(.2/math.radians(30)))

    def test_firmware_policies_keep_geometry_and_nonzero_corner_speed(self):
        source='G1 X120 E1\nG1 Y120 E2\n'
        exact=self.moves(self.plan(source,'exact'))
        for policy in ('klipper','marlin'):
            moves=self.moves(self.plan(source,policy))
            for before,after in zip(exact,moves):
                for key in ('start','end','e_start','delta_e','source_line'):
                    self.assertEqual(before[key],after[key])
            self.assertGreater(moves[0]['profile']['end_velocity'],0.)

    def test_blend_preserves_source_filament_and_bounds_corresponding_path(self):
        source='G1 X120 E1\nG1 Y120 E3\nG1 X130 E4\n'
        exact=self.moves(self.plan(source,'exact'))
        rounded=self.moves(self.plan(source,'blend'))
        self.assertEqual(rounded[0]['start'],exact[0]['start'])
        self.assertEqual(rounded[-1]['end'],exact[-1]['end'])
        self.assertTrue(any(m.get('blend_segment') for m in rounded))
        for original in exact:
            parts=[m for m in rounded if m['source_line']==original['source_line']]
            self.assertAlmostEqual(sum(m['delta_e'] for m in parts),original['delta_e'])
        for left,right in zip(rounded,rounded[1:]):
            for a,b in zip(left['end'],right['start']): self.assertAlmostEqual(a,b)
            self.assertAlmostEqual(left['e_start']+left['delta_e'],right['e_start'])
            self.assertGreater(left['profile']['end_velocity'],0.)
        for move in rounded:
            if not move.get('blend_segment'): continue
            for which in ('start','end'):
                self.assertLessEqual(math.dist(move[which][:3],move['reference_'+which][:3]),.025+1e-9)
                self.assertEqual(move[which][3:],move['reference_'+which][3:])

    def test_reversals_rotary_changes_and_extrusion_sign_changes_stop(self):
        for source in ('G1 X120 E1\nG1 X110 E2\n','G1 X120 A2 E1\nG1 Y120 A3 E2\n',
                       'G1 X120 E1\nG1 Y120 E.5\n','G1 X120 E1\nG1 Y120\n'):
            for policy in ('klipper','marlin','blend'):
                with self.subTest(source=source,policy=policy):
                    moves=self.moves(self.plan(source,policy))
                    self.assertEqual(len(moves),2)
                    self.assertEqual(moves[0]['profile']['end_velocity'],0.)

    def test_zero_blend_and_barriers_do_not_round(self):
        for source,params in (('G1 X120\nM400\nG1 Y120\n',{}),
                              ('G1 X120\nG1 Y120\n',{'blend_tolerance_mm':0.})):
            moves=self.moves(self.plan(source,'blend',**params))
            self.assertEqual(len(moves),2)
            self.assertEqual(moves[0]['profile']['end_velocity'],0.)

    def test_invalid_options_are_rejected_and_choices_change_hash(self):
        for value in ({'policy':'unknown'},{'policy':None},{'blend_tolerance_mm':True},
                      {'square_corner_velocity':float('nan')},{'junction_deviation_mm':-.1},
                      {'blend_tolerance_mm':.3},{'typo':1}):
            with self.subTest(value=value),self.assertRaises(ValueError): options(value)
        hashes={self.plan('G1 X120\nG1 Y120\n',p)['plan_hash'] for p in ('exact','klipper','marlin','blend')}
        self.assertEqual(len(hashes),4)
        with self.assertRaisesRegex(ValueError,'continuous_motion'):
            compile_job(ENTRY+'G1 X120\n',LIMITS,HEATERS,motion_options={'policy':'klipper'})

    def test_blend_checks_extrusion_ratio_on_the_shortened_path(self):
        limits=dict(LIMITS,continuous_motion=True,max_extrude_ratio=.1)
        with self.assertRaisesRegex(ValueError,'rounded extrusion.*cross-section'):
            compile_job('M109 S200\n'+ENTRY+'G1 X120 E1\nG1 Y120 E2\n',limits,HEATERS,
                        motion_options={'policy':'blend'})


if __name__=='__main__': unittest.main()
