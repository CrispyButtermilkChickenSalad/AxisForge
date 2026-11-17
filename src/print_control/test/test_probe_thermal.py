"""Heated mesh interlocks using simulated temperatures and time only."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from print_control.load_cell_contact import ContactOptions
from print_control.probe_thermal import ProbeThermalPolicy, thermal_options


def printer():
    return {'webhooks':{'state':'ready'},'extruder':{'temperature':210.,'target':210.},
            'heater_bed':{'temperature':60.,'target':60.},'axisforge_sync':{},
            'load_cell_probe':{'is_calibrated':True,'counts_per_gram':100.,
                'reference_tare_counts':1000.,'errors':0,'overflows':0}}


class ThermalTests(unittest.TestCase):
    def setUp(self):
        self.now=0.
        self.policy=ProbeThermalPolicy({'nozzle_target_c':210.,'bed_target_c':60.,'soak_s':30.},clock=lambda:self.now)
        self.data=printer()
        self.options=ContactOptions(trigger_mode='raw')

    def sample(self):
        self.policy.validate_status(self.data,'load_cell_probe',self.options)

    def soak(self,seconds=30.):
        self.sample()
        for _ in range(int(seconds*4)):
            self.now+=.25
            self.sample()

    def test_both_heaters_must_complete_uninterrupted_soak_before_motion(self):
        self.data['heater_bed']['temperature']=50.
        self.soak(40)
        self.assertFalse(self.policy.ready())
        self.data['heater_bed']['temperature']=60.
        self.soak(15)
        self.assertFalse(self.policy.ready())
        self.data['extruder']['temperature']=207.
        self.sample()
        self.data['extruder']['temperature']=210.
        self.soak(29.75)
        self.assertFalse(self.policy.ready())
        self.now+=.25;self.sample();self.policy.require_ready()
        self.assertTrue(self.policy.status()['locked'])
        self.assertEqual(self.policy.status()['temperatures']['heater_bed']['target_c'],60.)

    def test_five_second_soak_requires_full_stable_interval(self):
        self.policy=ProbeThermalPolicy({'nozzle_target_c':210.,'bed_target_c':60.,'soak_s':5.},clock=lambda:self.now)
        self.soak(4.75)
        self.assertFalse(self.policy.ready())
        self.now+=.25;self.sample();self.policy.require_ready()
        self.assertEqual(self.policy.status()['required_stable_s'],5.)

    def test_drift_target_change_or_status_gap_latches_failure_after_soak(self):
        for change in ('drift','target','gap'):
            with self.subTest(change=change):
                self.setUp();self.soak();self.policy.require_ready()
                if change=='drift':self.data['heater_bed']['temperature']=63.
                elif change=='target':self.data['extruder']['target']=0.
                else:self.now+=1.01
                with self.assertRaises(RuntimeError):self.sample()
                self.data=printer()
                with self.assertRaises(RuntimeError):self.policy.require_ready()

    def test_stale_feedback_and_pre_soak_gap_cannot_authorize_motion(self):
        self.soak(20)
        self.now+=1.01
        with self.assertRaises(RuntimeError):self.policy.require_ready()
        self.sample();self.soak(29.75)
        self.assertFalse(self.policy.ready())
        self.now+=.25;self.sample();self.policy.require_ready()
        self.now+=1.01
        with self.assertRaises(RuntimeError):self.policy.require_ready()

    def test_nozzle_only_and_bed_only_are_supported_and_missing_bed_is_explicit(self):
        self.policy=ProbeThermalPolicy({'nozzle_target_c':210.,'bed_target_c':0.,'soak_s':30.},clock=lambda:self.now)
        del self.data['heater_bed']
        self.soak();self.policy.require_ready()
        self.setUp();del self.data['heater_bed']
        with self.assertRaisesRegex(RuntimeError,'not configured'):self.sample()
        self.setUp()
        self.policy=ProbeThermalPolicy({'nozzle_target_c':0.,'bed_target_c':60.,'soak_s':30.},clock=lambda:self.now)
        self.data['extruder']={'temperature':25.,'target':0.}
        self.soak();self.policy.require_ready()

    def test_heat_does_not_bypass_native_calibration_or_print_ownership(self):
        self.options=ContactOptions(trigger_mode='raw',trigger_source='klipper_probe')
        self.data['load_cell_probe']['is_calibrated']=False
        with self.assertRaisesRegex(RuntimeError,'not calibrated'):self.sample()
        self.data=printer();self.data['axisforge_sync']['leased']=True
        with self.assertRaises(RuntimeError):self.sample()

    def test_moonraker_empty_object_means_absent_bed_but_existing_feedback_cannot_disappear(self):
        self.policy=ProbeThermalPolicy({'nozzle_target_c':210.,'bed_target_c':0.,'soak_s':30.},clock=lambda:self.now)
        self.data['heater_bed']={}
        self.soak();self.policy.require_ready()
        self.setUp();self.data['heater_bed']={}
        with self.assertRaisesRegex(RuntimeError,'not configured'):self.sample()
        self.setUp();self.soak();self.policy.require_ready()
        self.data['heater_bed']={}
        with self.assertRaisesRegex(RuntimeError,'disappeared'):self.sample()
        self.policy=ProbeThermalPolicy(clock=lambda:self.now)
        self.data['extruder']={'temperature':25.,'target':0.}
        self.sample();self.policy.require_ready()

    def test_cold_default_rejects_heat_and_requires_no_soak(self):
        self.policy=ProbeThermalPolicy(clock=lambda:self.now)
        with self.assertRaises(RuntimeError):self.sample()
        self.policy=ProbeThermalPolicy(clock=lambda:self.now)
        self.data['extruder']={'temperature':25.,'target':0.}
        self.data['heater_bed']={'temperature':25.,'target':0.}
        self.sample();self.policy.require_ready()
        self.assertEqual(self.policy.status()['required_stable_s'],0.)

    def test_wait_reports_progress_allows_cancellation_and_never_commands_heat(self):
        source=Mock();control=Mock()
        self.sample()
        def tick(_):
            self.now+=.25;self.sample()
        self.policy.wait_ready(source,control,sleep=tick)
        self.assertTrue(self.policy.locked)
        self.assertTrue(all(call.args[0]=='waiting_temperature' for call in control.emit.call_args_list))
        self.assertEqual(set(call[0] for call in source.mock_calls),{'snapshot'})
        control.check.side_effect=RuntimeError('Operator cancelled')
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.policy.wait_ready(source,control)

    def test_invalid_options_are_rejected(self):
        base={'nozzle_target_c':210.,'bed_target_c':60.,'soak_s':30.}
        for change in ({'nozzle_target_c':True},{'nozzle_target_c':301.},
                       {'bed_target_c':20.},{'bed_target_c':float('nan')},{'soak_s':0.},{'soak_s':4.99}):
            with self.subTest(change=change),self.assertRaises(ValueError):thermal_options({**base,**change})

    def test_failed_temperature_wait_never_creates_robot_node(self):
        from print_control import bridge_bed_probe as routine
        source=Mock();control=Mock();control.reason=None
        args=SimpleNamespace(profile='profile',config='config',dry_run=False,
            probe_nozzle_target=210.,probe_bed_target=60.,probe_soak_s=30.)
        homing=SimpleNamespace(heartbeat_timeout_s=2.,confirmation_timeout_s=60.)
        with patch.object(routine,'ProbeControl',return_value=Mock(start=lambda:control)), \
             patch.object(routine,'parse_args',return_value=args), \
             patch.object(routine,'load_profile',return_value=(self.options,homing)), \
             patch.object(routine,'load_configs',return_value=({},{})), \
             patch.object(routine,'sensor_setup',return_value=(self.options,source)) as setup, \
             patch.object(routine.ProbeThermalPolicy,'wait_ready',side_effect=RuntimeError('temperature changed')), \
             patch.object(routine,'BridgeBedProbe') as robot, \
             patch.object(routine.rclpy,'init') as ros_init,patch.object(routine.rclpy,'ok',return_value=False), \
             patch.object(routine.traceback,'print_exc'):
            with self.assertRaises(SystemExit):routine.main([])
        robot.assert_not_called();ros_init.assert_not_called();source.close.assert_called_once()
        self.assertFalse(setup.call_args.kwargs['check_baseline'])
        self.assertEqual(setup.call_args.args[1].baseline_noise_fraction,.4)
        self.assertEqual(setup.call_args.kwargs['native_thermal_targets'],{'extruder':210.,'heater_bed':60.})
