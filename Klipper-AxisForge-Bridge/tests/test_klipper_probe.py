"""Native probe ownership, MCU confirmation and cancellation regression tests."""

import math
import unittest
import uuid
from types import SimpleNamespace

from klipper_extras.axisforge_probe import AxisForgeProbe


class Completion:
    def __init__(self, reactor):
        self.reactor = reactor
        self.done = False
        self.result = None

    def test(self):
        return self.done

    def complete(self, result):
        self.done, self.result = True, result

    def wait(self, waketime=math.inf):
        if not self.done:
            self.reactor.advance(waketime)
        return self.result


class Reactor:
    NOW = 0.

    def __init__(self):
        self.now = 10.
        self.callbacks = []
        self.on_advance = None

    def monotonic(self):
        return self.now

    def completion(self):
        return Completion(self)

    def register_timer(self, callback, when):
        return callback

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def advance(self, until):
        callback, self.on_advance = self.on_advance, None
        if callback:
            callback()
        if math.isfinite(until):
            self.now = max(self.now, until)
        return self.now

    pause = advance


class Request:
    error = RuntimeError

    def __init__(self, **values):
        self.values = values
        self.response = None

    def get(self, name, default=None):
        return self.values.get(name, default)

    def send(self, response):
        self.response = response


class Trigger:
    REASON_TRIGGER_ANALOG = 5

    def __init__(self, reactor):
        self.reactor = reactor
        self.mcu = SimpleNamespace(estimated_print_time=lambda now: now + 100.)
        self._raw_min, self._raw_max = -300000, 300000
        self._error_map = {0: 'RAW_RANGE'}
        self.started = self.stopped = self.cleared = 0
        self.reason = 2
        self.timestamp = None
        self.homing = True
        self.on_stop = self.on_query = None
        self.stop_error = self.clear_error = None
        self.steppers = []
        self._query_state_cmd = SimpleNamespace(send=self.query)

    def query(self, args):
        if self.on_query:
            self.on_query()
        return {'homing': self.homing, 'homing_clock': 0}

    def get_mcu(self):
        return self.mcu

    def get_steppers(self):
        return self.steppers

    def get_dispatch(self):
        return self

    def get_oid(self):
        return 17

    def set_raw_range(self, low, high):
        self._raw_min, self._raw_max = low, high

    def home_start(self, print_time, sample_time, sample_count, rest_time):
        self.started += 1
        self.arm_time = print_time
        self.completion = self.reactor.completion()
        return self.completion

    def stop(self):
        self.stopped += 1
        callback, self.on_stop = self.on_stop, None
        if callback:
            callback()
        if self.stop_error:
            raise RuntimeError(self.stop_error)
        return self.reason

    def _clear_home(self):
        self.cleared += 1
        if self.clear_error:
            raise RuntimeError(self.clear_error)
        return self.timestamp

    def contact(self, reason=1, timestamp=None):
        self.reason = reason
        self.timestamp = self.reactor.now + 100. if timestamp is None else timestamp
        self.homing = False
        self.completion.complete(reason >= 4)


class Fixture:
    def __init__(self):
        self.reactor = Reactor()
        self.trigger = Trigger(self.reactor)
        self.calibrated = True
        self.cpg, self.force, self.tare = 138., 200., 1000
        self.target, self.temperature = 0., 25.
        self.printing = False
        self.print_state = 'standby'
        self.idle_state = 'Ready'
        self.sensor_errors = self.overflows = 0
        self.gcode_busy = False
        self.tare_calls = 0
        self.on_tare = None
        self.sync = SimpleNamespace(leased=False, preparing=False, reservation=None)
        self.load_cell = SimpleNamespace(
            is_calibrated=lambda: self.calibrated,
            get_counts_per_gram=lambda: self.cpg,
            get_reference_tare_counts=lambda: 0 if self.calibrated else None,
            get_tare_counts=lambda: self.tare,
            get_sensor=lambda: SimpleNamespace(get_status=lambda now: {
                'errors': self.sensor_errors, 'overflows': self.overflows}),
        )
        self.move = SimpleNamespace(
            _pause_and_tare=self.pause_and_tare,
            _config_helper=SimpleNamespace(get_trigger_force_grams=lambda: self.force),
        )
        self.probe = SimpleNamespace(
            _tapping_move=SimpleNamespace(_load_cell_probing_move=self.move),
            _load_cell=self.load_cell, _mcu_trigger_analog=self.trigger,
        )
        self.heater = SimpleNamespace(get_temp=lambda now: (self.temperature, self.target))
        self.commands = {'G1': lambda gcmd: 'original',
                         'LOAD_CELL_TEST_TAP': lambda gcmd: 'original',
                         'TURN_OFF_HEATERS': lambda gcmd: 'off'}
        self.objects = {
            'probe': self.probe, 'toolhead': SimpleNamespace(),
            'webhooks': SimpleNamespace(register_endpoint=lambda name, handler: None),
            'configfile': SimpleNamespace(get_status=lambda now: {
                'settings': {'printer': {'kinematics': 'none'}}}),
            'heaters': SimpleNamespace(get_all_heaters=lambda: ['extruder'],
                                       lookup_heater=lambda name: self.heater),
            'gcode': SimpleNamespace(register_command=self.register_command,
                get_status=lambda now: {'commands': self.commands},
                get_mutex=lambda: SimpleNamespace(test=lambda: self.gcode_busy)),
            'virtual_sdcard': SimpleNamespace(is_active=lambda: self.printing),
            'print_stats': SimpleNamespace(get_status=lambda now: {'state': self.print_state}),
            'idle_timeout': SimpleNamespace(get_status=lambda now: {'state': self.idle_state}),
            'axisforge_sync': self.sync,
        }
        self.printer = SimpleNamespace(
            get_reactor=lambda: self.reactor,
            lookup_object=lambda name, default=None: self.objects.get(name, default),
            register_event_handler=lambda name, handler: None,
            command_error=RuntimeError,
        )
        self.adapter = AxisForgeProbe(SimpleNamespace(get_printer=lambda: self.printer,
            getfloat=lambda name,default,**kwargs: default))
        self.adapter._ready()

    def test_faster_raw_batch_delivery_preserves_acquisition_and_calibration(self):
        batch=SimpleNamespace(batch_interval=.1,_proc_batch=lambda now: None)
        sensor=SimpleNamespace(batch_bulk=batch,sps=500,query_clock=12345)
        self.load_cell.get_sensor=lambda:sensor
        before=(self.cpg,self.tare,self.force)
        self.adapter._configure_sample_batching()
        self.assertEqual(batch.batch_interval,.025)
        self.assertEqual(self.adapter.active_sample_batch_interval_s,.025)
        self.assertEqual((sensor.sps,sensor.query_clock),(500,12345))
        self.assertEqual((self.cpg,self.tare,self.force),before)

    def test_existing_faster_batch_interval_is_not_slowed(self):
        batch=SimpleNamespace(batch_interval=.01,_proc_batch=lambda now:None)
        self.load_cell.get_sensor=lambda:SimpleNamespace(batch_bulk=batch)
        self.adapter._configure_sample_batching()
        self.assertEqual(batch.batch_interval,.01)

    def test_invalid_batch_api_is_rejected_without_modification(self):
        for value in (0.,float('nan'),True):
            batch=SimpleNamespace(batch_interval=value,_proc_batch=lambda now:None)
            self.load_cell.get_sensor=lambda:SimpleNamespace(batch_bulk=batch)
            with self.assertRaisesRegex(ValueError,'incompatible'):
                self.adapter._configure_sample_batching()
            self.assertIs(batch.batch_interval,value)

    def register_command(self, name, handler):
        previous = self.commands.pop(name, None)
        if handler is not None:
            self.commands[name] = handler
        return previous

    def pause_and_tare(self, gcmd):
        self.tare_calls += 1
        if self.on_tare:
            self.on_tare()

    def arm(self, **changes):
        values = dict(session=str(uuid.uuid4()), lease_s=.5,
                      max_delta_counts=120000, max_nozzle_temp_c=50,
                      search_timeout_s=10.)
        values.update(changes)
        request = Request(**values)
        self.adapter.api_arm(request)
        return request.response

    def request(self, **values):
        return Request(session=self.adapter.session, **values)

    def contact(self, **values):
        self.trigger.contact(**values)
        self.adapter._monitor_native(self.reactor.now, self.adapter.generation,
                                     self.trigger.completion)


class NativeProbeTests(unittest.TestCase):
    def setUp(self):
        self.f = Fixture()
        self.a = self.f.adapter

    def test_both_heaters_are_supported_but_target_changes_and_drift_stop_native_probe(self):
        for fault in ('target','drift'):
            with self.subTest(fault=fault):
                self.setUp()
                bed={'temperature':60.,'target':60.}
                heater=SimpleNamespace(get_temp=lambda now:(bed['temperature'],bed['target']))
                self.a.heater_by_name['heater_bed']=heater
                self.a.heaters.append(heater)
                self.f.target=self.f.temperature=210.
                self.f.arm(thermal_targets={'extruder':210.,'heater_bed':60.})
                self.assertEqual(self.a.state,'armed')
                if fault=='target':bed['target']=0.
                else:bed['temperature']=63.
                self.a._watchdog(self.f.reactor.now)
                self.assertEqual(self.a.state,'fault')
                self.assertTrue(self.a.stop_required)

    def test_heated_mode_cannot_hide_missing_heater_or_bypass_cold_default(self):
        self.f.target=self.f.temperature=210.
        with self.assertRaisesRegex(RuntimeError,'not configured'):
            self.f.arm(thermal_targets={'extruder':210.,'heater_bed':60.})
        with self.assertRaisesRegex(RuntimeError,'target'):
            self.f.arm()
        self.assertEqual(self.f.tare_calls,0)
        self.assertEqual(self.f.trigger.started,0)

    def test_invalid_heated_options_do_not_tare_or_arm(self):
        for value in ({'extruder':True},{'extruder':301.},{'heater_bed':20.},
                      {'extruder':float('nan')},{'chamber':60.},[] ):
            with self.subTest(value=value),self.assertRaises(RuntimeError):
                self.f.arm(thermal_targets=value)
        self.assertEqual(self.f.tare_calls,0)
        self.assertEqual(self.f.trigger.started,0)

    def test_read_only_capabilities_and_uncalibrated_refusal(self):
        self.f.calibrated = False
        request = Request()
        self.a.api_capabilities(request)
        self.assertTrue(request.response['supported'])
        self.assertFalse(request.response['calibrated'])
        self.assertEqual(request.response['trigger_force_g'], 200.)
        with self.assertRaisesRegex(RuntimeError, 'calibrated'):
            self.f.arm()
        self.assertEqual((self.f.tare_calls, self.f.trigger.started), (0, 0))

    def test_invalid_calibration_values_fail_before_tare(self):
        for value in (None, float('nan'), float('inf'), 0., -1.):
            self.f.cpg = value
            with self.assertRaisesRegex(RuntimeError, 'calibrated'):
                self.f.arm()
        self.assertEqual(self.f.tare_calls, 0)

    def test_arming_confirms_future_mcu_setup_and_intersects_raw_limits(self):
        self.f.trigger._raw_min, self.f.trigger._raw_max = -40000, 50000
        result = self.f.arm()
        self.assertEqual(result['state'], 'armed')
        self.assertGreaterEqual(self.f.reactor.now + 100., result['arm_time'])
        self.assertEqual((self.f.trigger._raw_min, self.f.trigger._raw_max), (-40000, 50000))
        self.assertFalse(result['stop_required'])
        self.assertFalse(result['cleanup_confirmed'])
        self.assertEqual(self.f.tare_calls, 1)

    def test_wide_native_safety_is_tightened_to_tared_one_hundred_twenty_thousand(self):
        self.f.arm(max_delta_counts=120000.)
        self.assertEqual((self.f.trigger._raw_min, self.f.trigger._raw_max), (-119000, 121000))

    def test_omitted_limit_defaults_to_one_hundred_twenty_thousand(self):
        self.a.api_arm(Request(session=str(uuid.uuid4())))
        self.assertEqual((self.f.trigger._raw_min, self.f.trigger._raw_max), (-119000, 121000))

    def test_limit_above_one_hundred_twenty_thousand_is_rejected_before_tare(self):
        for limit in (120000.1, 120001., 250000.):
            with self.subTest(limit=limit), self.assertRaisesRegex(RuntimeError, 'max_delta_counts'):
                self.f.arm(max_delta_counts=limit)
        self.assertEqual((self.f.tare_calls, self.f.trigger.started), (0, 0))

    def test_trigger_at_overload_boundary_is_rejected_before_tare(self):
        self.f.cpg = 1.
        self.f.force = 120000.
        with self.assertRaisesRegex(RuntimeError, 'trigger force exceeds'):
            self.f.arm()
        self.assertEqual((self.f.tare_calls, self.f.trigger.started), (0, 0))
        self.f.force = 119999.
        self.assertEqual(self.f.arm()['state'], 'armed')

    def test_narrow_native_safety_cannot_be_loosened(self):
        self.f.trigger._raw_min = 0
        result = self.f.arm()
        self.assertEqual(result['state'], 'fault')
        self.assertEqual(self.f.trigger.started, 0)

    def test_initial_printer_or_heater_activity_refuses_arm(self):
        for attribute, value in (('printing', True), ('target', 100.),
                                 ('temperature', 60.), ('idle_state', 'Printing')):
            old = getattr(self.f, attribute)
            setattr(self.f, attribute, value)
            with self.assertRaises(RuntimeError):
                self.f.arm()
            setattr(self.f, attribute, old)
        self.assertEqual(self.f.tare_calls, 0)

    def test_print_lease_conflicts_and_native_commands_are_guarded(self):
        self.f.sync.leased = True
        with self.assertRaisesRegex(RuntimeError, 'synchronized'):
            self.f.arm()
        self.f.sync.leased = False
        self.f.arm()
        for name in ('G1', 'LOAD_CELL_TEST_TAP'):
            with self.assertRaisesRegex(RuntimeError, 'disarm'):
                self.f.commands[name](Request())

    def test_native_tare_printing_state_does_not_reject_own_no_motion_arm(self):
        self.f.on_tare = lambda: setattr(self.f, 'idle_state', 'Printing')
        self.assertEqual(self.f.arm()['state'], 'armed')
        self.a._watchdog(self.f.reactor.now)
        self.assertEqual(self.a.state, 'armed')

    def test_tare_expiry_never_starts_mcu(self):
        def expire():
            self.f.reactor.now += 2.01
            self.a._watchdog(self.f.reactor.now)
        self.f.on_tare = expire
        result = self.f.arm()
        self.assertEqual(result['state'], 'fault')
        self.assertIn('arming', result['error'])
        self.assertEqual(self.f.trigger.started, 0)

    def test_tare_has_bounded_two_second_setup_window_before_active_lease(self):
        self.f.on_tare = lambda: setattr(self.f.reactor, 'now', self.f.reactor.now + 1.)
        result = self.f.arm(lease_s=.4)
        self.assertEqual(result['state'], 'armed')
        self.assertAlmostEqual(self.a.deadline, self.f.reactor.now + .4)

    def test_paused_print_and_existing_calibration_refuse_arm(self):
        self.f.print_state = 'paused'
        with self.assertRaisesRegex(RuntimeError, 'paused'):
            self.f.arm()
        self.f.print_state = 'standby'
        self.f.commands['TARE'] = lambda gcmd: None
        with self.assertRaisesRegex(RuntimeError, 'calibration'):
            self.f.arm()
        self.assertEqual(self.f.tare_calls, 0)

    def test_in_progress_gcode_command_refuses_arm(self):
        self.f.gcode_busy = True
        with self.assertRaisesRegex(RuntimeError, 'still executing'):
            self.f.arm()

    def test_tare_cannot_hide_preexisting_contact(self):
        self.f.tare = 30000
        result = self.f.arm()
        self.assertEqual(result['state'], 'fault')
        self.assertIn('Load is present', result['error'])
        self.assertEqual(self.f.trigger.started, 0)

    def test_sensor_errors_before_and_during_tare_refuse_arm(self):
        self.f.sensor_errors = 1
        with self.assertRaisesRegex(RuntimeError, 'sensor'):
            self.f.arm()
        self.f.sensor_errors = 0
        self.f.on_tare = lambda: setattr(self.f, 'overflows', 1)
        result = self.f.arm()
        self.assertEqual(result['state'], 'fault')
        self.assertEqual(self.f.trigger.started, 0)

    def test_sensor_failure_while_armed_disarms(self):
        self.f.arm()
        self.f.sensor_errors = 1
        self.a._watchdog(self.f.reactor.now)
        self.assertEqual(self.a.state, 'fault')
        self.assertEqual(self.f.trigger.cleared, 1)

    def test_cancel_while_tare_yields_never_rearms(self):
        self.f.on_tare = lambda: self.a.api_disarm(self.f.request())
        result = self.f.arm()
        self.assertNotEqual(result['state'], 'armed')
        self.assertTrue(result['cleanup_confirmed'])
        self.assertEqual(self.f.trigger.started, 0)

    def test_no_armed_result_on_early_contact_or_missing_mcu_ack(self):
        self.f.trigger.on_query = lambda: self.f.trigger.contact()
        result = self.f.arm()
        self.assertEqual(result['state'], 'fault')
        self.assertTrue(result['stop_required'])
        self.assertEqual((self.f.trigger.stopped, self.f.trigger.cleared), (1, 1))

    def test_valid_contact_latches_timestamp_and_cleanup_once(self):
        self.f.arm()
        self.f.contact()
        self.assertEqual(self.a.state, 'triggered')
        self.assertTrue(self.a.stop_required)
        self.assertTrue(self.a.cleanup_confirmed)
        self.assertEqual(self.a.trigger_time, self.f.reactor.now + 100.)
        for unused in range(2):
            request = self.f.request(timeout_s=0.)
            self.a.api_wait(request)
            self.assertEqual(request.response['state'], 'triggered')
        self.assertEqual((self.f.trigger.stopped, self.f.trigger.cleared), (1, 1))

    def test_triggered_ownership_and_heartbeat_continue_until_disarm(self):
        self.f.arm(search_timeout_s=.1)
        self.f.contact()
        self.assertTrue(self.a.is_busy())
        with self.assertRaisesRegex(RuntimeError, 'disarm'):
            self.f.commands['G1'](Request())
        self.f.reactor.now += .2
        self.a.api_wait(self.f.request(timeout_s=0.))
        self.assertEqual(self.a.state, 'triggered')
        self.assertAlmostEqual(self.a.deadline, self.f.reactor.now + .5)
        self.a.api_disarm(self.f.request())
        self.assertFalse(self.a.is_busy())

    def test_triggered_disconnect_expires_and_invalidates_contact(self):
        self.f.arm()
        self.f.contact()
        self.f.reactor.now += .51
        self.a._watchdog(self.f.reactor.now)
        self.assertEqual(self.a.state, 'fault')
        self.assertIsNone(self.a.trigger_time)
        self.assertFalse(self.a.is_busy())
        self.assertEqual(self.f.trigger.cleared, 1)

    def test_stop_required_visible_before_serial_classification_completes(self):
        self.f.arm()
        observations = []
        def inspect():
            request = self.f.request(timeout_s=0.)
            self.a.api_wait(request)
            observations.append(request.response)
        self.f.trigger.on_stop = inspect
        self.f.contact()
        self.assertEqual(observations[0]['state'], 'armed')
        self.assertTrue(observations[0]['stop_required'])
        self.assertIsNone(observations[0]['trigger_time'])
        self.assertEqual(self.a.state, 'triggered')

    def test_fault_reasons_never_become_contact(self):
        for reason in (2, 3, 4, 5, 99):
            fixture = Fixture()
            fixture.arm()
            fixture.contact(reason=reason)
            self.assertEqual(fixture.adapter.state, 'fault')
            self.assertIsNone(fixture.adapter.trigger_time)

    def test_invalid_latched_clock_never_becomes_contact(self):
        for timestamp in (float('nan'), float('inf'), 1., 999.):
            fixture = Fixture()
            fixture.arm()
            fixture.contact(timestamp=timestamp)
            self.assertEqual(fixture.adapter.state, 'fault')
            self.assertIsNone(fixture.adapter.trigger_time)

    def test_lease_expiry_clears_mcu_and_wakes_native_waiter(self):
        self.f.arm()
        self.f.reactor.now += .51
        self.a._watchdog(self.f.reactor.now)
        self.assertEqual(self.a.state, 'fault')
        self.assertTrue(self.f.trigger.completion.test())
        self.assertTrue(self.a.cleanup_confirmed)
        self.assertEqual((self.f.trigger.stopped, self.f.trigger.cleared), (1, 1))
        self.f.contact()
        self.assertEqual(self.a.state, 'fault')

    def test_wait_renews_lease_and_search_timeout_is_independent(self):
        self.f.arm(search_timeout_s=.2)
        self.f.reactor.now += .1
        request = self.f.request(timeout_s=0.)
        self.a.api_wait(request)
        self.assertAlmostEqual(self.a.deadline, self.f.reactor.now + .5)
        self.f.reactor.now += .11
        self.a._watchdog(self.f.reactor.now)
        self.assertEqual(self.a.state, 'fault')
        self.assertIn('search', self.a.error)

    def test_wait_cannot_resurrect_expired_lease(self):
        self.f.arm()
        self.f.reactor.now += .51
        request = self.f.request(timeout_s=0.)
        self.a.api_wait(request)
        self.assertEqual(request.response['state'], 'fault')

    def test_disarm_is_idempotent_and_invalidates_late_trigger(self):
        self.f.arm()
        for unused in range(2):
            request = self.f.request()
            self.a.api_disarm(request)
            self.assertEqual(request.response['state'], 'disarmed')
        self.f.contact()
        self.assertEqual(self.a.state, 'disarmed')
        self.assertEqual((self.f.trigger.stopped, self.f.trigger.cleared), (1, 1))

    def test_lease_expiry_during_cleanup_wins_over_real_contact(self):
        self.f.arm()
        def expire_during_query():
            self.f.reactor.now += .51
            self.a._watchdog(self.f.reactor.now)
        self.f.trigger.on_stop = expire_during_query
        self.f.contact()
        self.assertEqual(self.a.state, 'fault')
        self.assertIsNone(self.a.trigger_time)
        self.assertEqual((self.f.trigger.stopped, self.f.trigger.cleared), (1, 1))

    def test_shutdown_during_cleanup_wins_over_real_contact(self):
        self.f.arm()
        self.f.trigger.on_stop = self.a._shutdown
        self.f.contact()
        self.assertEqual(self.a.state, 'fault')
        self.assertIn('shutdown', self.a.error)
        self.assertFalse(self.a.supported)

    def test_cancel_during_cleanup_wins_over_real_contact(self):
        self.f.arm()
        self.f.trigger.on_stop = lambda: self.a._finish('disarmed', '')
        self.f.contact()
        self.assertEqual(self.a.state, 'disarmed')
        self.assertIsNone(self.a.trigger_time)
        self.assertEqual((self.f.trigger.stopped, self.f.trigger.cleared), (1, 1))

    def test_unknown_or_motion_capable_probe_api_is_unsupported(self):
        self.f.trigger.steppers = [object()]
        self.a._ready()
        self.assertFalse(self.a.supported)
        with self.assertRaisesRegex(RuntimeError, 'zero Klipper'):
            self.f.arm()
        self.f.trigger.steppers = []
        self.f.trigger.REASON_TRIGGER_ANALOG = 9
        self.a._ready()
        self.assertFalse(self.a.supported)
        self.assertIn('incompatible', self.a.support_error)

    def test_dispatch_failure_still_clears_analog_and_blocks_rearm(self):
        self.f.arm()
        self.f.trigger.stop_error = 'USB unavailable'
        with self.assertRaisesRegex(RuntimeError, 'not confirmed'):
            self.a.api_disarm(self.f.request())
        self.assertEqual(self.f.trigger.cleared, 1)
        self.assertEqual(self.a.state, 'fault')
        self.assertFalse(self.a.cleanup_confirmed)
        with self.assertRaisesRegex(RuntimeError, 'cleanup'):
            self.f.arm()

    def test_analog_cleanup_failure_is_not_reported_as_disarmed(self):
        self.f.arm()
        self.f.trigger.clear_error = 'ADC unreachable'
        with self.assertRaisesRegex(RuntimeError, 'not confirmed'):
            self.a.api_disarm(self.f.request())
        self.assertFalse(self.a.cleanup_confirmed)

    def test_input_limits_and_session_ownership(self):
        for values in ({'lease_s': .51}, {'timeout_s': .2}, {'search_timeout_s': 7201},
                       {'max_delta_counts': 120001}, {'max_delta_counts': float('nan')},
                       {'session': 'not-a-uuid'}):
            if 'timeout_s' in values:
                continue
            with self.assertRaises(RuntimeError):
                self.f.arm(**values)
        result = self.f.arm()
        with self.assertRaisesRegex(RuntimeError, 'Unknown'):
            self.a.api_disarm(Request(session=str(uuid.uuid4())))
        with self.assertRaises(RuntimeError):
            self.a.api_wait(self.f.request(timeout_s=.2))
        self.a.api_disarm(self.f.request())
        with self.assertRaisesRegex(RuntimeError, 'new UUID'):
            self.f.arm(session=result['session'])


if __name__ == '__main__':
    unittest.main()
