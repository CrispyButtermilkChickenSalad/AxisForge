"""Clock acceptance boundaries and speed guidance without hardware."""

import unittest

from bridge_dashboard.clock_settings import clock_options, profiles, speed_guidance
from bridge_dashboard.sync_clients import ParticipantError, sample_clock


class ClockSettingsTests(unittest.TestCase):
    def test_defaults_presets_and_recommended_speed_budget(self):
        self.assertEqual(clock_options(),{'max_uncertainty_ms':10.,'max_alignment_error_ms':10.})
        for profile in profiles():
            settings=clock_options(profile['options'])
            self.assertAlmostEqual(profile['suggested_max_speed_mm_s']*settings['max_alignment_error_ms']/1000,.1)
        self.assertEqual(profiles()[2]['options'],{'max_uncertainty_ms':10.,'max_alignment_error_ms':15.})

    def test_rejects_invalid_limits_and_inverted_bounds(self):
        for value in (True,[],{'unknown':10}, {'max_alignment_error_ms':9},
                      *({'max_uncertainty_ms':v} for v in (True,'5',float('nan'),float('inf'),0,1.9,20.1))):
            with self.subTest(value=value), self.assertRaises(ValueError):clock_options(value)
        self.assertEqual(clock_options({'max_uncertainty_ms':2})['max_uncertainty_ms'],2.)

    def test_guidance_uses_planned_xyz_peak_and_does_not_retime(self):
        import copy
        plan={'operations':[{'kind':'heater'},{'kind':'chunk','moves':[
            {'start':[0,0,0,0,0,0], 'end':[3,4,0,0,0,0], 'profile':{'velocity':4}},
            {'start':[3,4,0,0,0,0], 'end':[3,4,0,90,0,0], 'profile':{'velocity':100}}]}]}
        original=copy.deepcopy(plan)
        guidance=speed_guidance(plan,clock_options({'max_alignment_error_ms':15}))
        self.assertEqual(guidance['peak_xyz_speed_mm_s'],20.)
        self.assertAlmostEqual(guidance['timing_displacement_mm'],.3)
        self.assertTrue(guidance['exceeds_guidance'])
        self.assertEqual(plan,original)


class ClockSamplingLimitTests(unittest.IsolatedAsyncioTestCase):
    async def test_selected_limit_controls_acceptance_and_message(self):
        class Master:
            async def call(self,action):return {'print_time':100.}
        for limit,accepted in ((.005,False),(.01,False),(.015,True),(.02,True)):
            # 24 ms RTT => 13 ms uncertainty, including the 1 ms MCU allowance.
            times=iter([0.,.024])
            call=sample_clock(Master(),clock=lambda:next(times),samples=1,max_samples=1,
                              interval=0,max_uncertainty_s=limit)
            if accepted:self.assertAlmostEqual((await call).uncertainty,.013)
            else:
                with self.assertRaisesRegex(ParticipantError,'exceeds %g ms' % (limit*1000)):await call

    async def test_invalid_limit_fails_before_network_request(self):
        for limit in (True,float('nan'),float('inf'),.001,.021):
            with self.subTest(limit=limit),self.assertRaises(ValueError):
                await sample_clock(None,max_uncertainty_s=limit)
