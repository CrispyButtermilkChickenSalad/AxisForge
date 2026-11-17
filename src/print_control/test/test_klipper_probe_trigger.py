"""Native trigger protocol and cancellation tests; all HTTP requests are simulated."""
from io import BytesIO
import json
import threading
import time
import unittest
import uuid

from print_control.klipper_probe_trigger import KlipperProbeTrigger, KlipperProbeTriggerError


class Server:
    def __init__(self):
        self.capabilities = dict(protocol=1, supported=True, calibrated=True,
                                 trigger_force_g=15., counts_per_gram=138.)
        self.arm_reply = {}
        self.wait_reply = None
        self.disarm_reply = {}
        self.state = dict(state='armed', stop_required=False)
        self.requests = []
        self.arm_entered = threading.Event()
        self.wait_entered = threading.Event()
        self.arm_release = self.wait_release = None
        self.fail_disarm = False
        self.session = None

    def __call__(self, request, timeout):
        endpoint = request.full_url.rsplit('/', 1)[1]
        body = None if request.data is None else json.loads(request.data)
        self.requests.append((endpoint, body, request, timeout, threading.get_ident()))
        if endpoint == 'capabilities':
            result = dict(self.capabilities)
        elif endpoint == 'arm':
            self.session = body['session']
            self.arm_entered.set()
            if self.arm_release is not None:
                self.arm_release.wait(2.)
            self.state = {'state': 'armed', 'stop_required': False, **self.arm_reply}
            result = dict(protocol=1, session=self.session, **self.state)
        elif endpoint == 'wait':
            self.wait_entered.set()
            if self.wait_release is not None:
                self.wait_release.wait(2.)
            else:
                time.sleep(.005)
            result = dict(protocol=1, session=self.session, **self.state)
            if self.wait_reply is not None:
                result.update(self.wait_reply)
        else:
            assert endpoint == 'disarm'
            if self.fail_disarm:
                raise TimeoutError('disarm timeout')
            result = dict(protocol=1, state='disarmed', session=body['session'])
            result.update(self.disarm_reply)
        return BytesIO(json.dumps({'result': result}).encode())


class NativeProbeClientTests(unittest.TestCase):
    def setUp(self):
        self.server = Server()
        self.client = KlipperProbeTrigger('http://printer:7125', api_key='private-test-key', opener=self.server)
        self.addCleanup(self.cleanup)

    def cleanup(self):
        if self.server.arm_release is not None:
            self.server.arm_release.set()
        if self.server.wait_release is not None:
            self.server.wait_release.set()
        try:
            self.client.end_contact()
        except KlipperProbeTriggerError:
            pass

    def wait_for_stop(self):
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            if self.client.check():
                return
            time.sleep(.005)
        self.fail('Native stop was not observed')

    def test_capability_check_is_read_only_and_requires_calibration(self):
        result = self.client.validate_ready()
        self.assertEqual(result['trigger_force_g'], 15.)
        self.assertEqual([r[0] for r in self.server.requests], ['capabilities'])
        for change in ({'protocol': True}, {'protocol': '1'}, {'protocol': 2},
                       {'supported': False}, {'calibrated': False}, {'calibrated': 1},
                       {'trigger_force_g': 0.}, {'trigger_force_g': float('nan')},
                       {'counts_per_gram': None}, {'counts_per_gram': -138.}):
            original = self.server.capabilities.copy()
            self.server.capabilities.update(change)
            with self.subTest(change=change), self.assertRaises(KlipperProbeTriggerError):
                self.client.validate_ready()
            self.server.capabilities = original

    def test_hot_probing_requires_explicit_server_support_and_real_calibration(self):
        client=KlipperProbeTrigger('http://printer:7125',opener=self.server,
            thermal_targets={'extruder':210.,'heater_bed':60.})
        with self.assertRaisesRegex(KlipperProbeTriggerError,'heated'):client.validate_ready()
        self.server.capabilities['heated_probing']=True
        client.validate_ready()
        self.server.capabilities['calibrated']=False
        with self.assertRaises(KlipperProbeTriggerError):client.validate_ready()

    def test_arm_and_wait_run_off_thread_and_forward_limits_and_api_key(self):
        self.server.arm_release = threading.Event()
        pump_calls = []
        def pump():
            pump_calls.append(threading.get_ident())
            self.server.arm_release.set()
        self.client.begin_contact(pump, search_timeout_s=120.)
        self.assertFalse(self.client.check())
        self.assertTrue(self.server.wait_entered.wait(.5))
        arm = next(r for r in self.server.requests if r[0] == 'arm')
        self.assertNotEqual(arm[4], threading.get_ident())
        self.assertTrue(pump_calls)
        self.assertEqual(arm[1], dict(session=self.client.session, max_delta_counts=120000.,
                                    max_nozzle_temp_c=50., search_timeout_s=120., lease_s=.4))
        self.assertEqual(arm[2].get_header('X-api-key'), 'private-test-key')
        self.assertNotIn('private-test-key', arm[2].full_url)
        self.assertEqual(arm[3], 3.)
        self.assertEqual(str(uuid.UUID(self.client.session)), self.client.session)
        self.assertEqual(next(r for r in self.server.requests if r[0] == 'wait')[3], .25)
        self.client.end_contact()
        self.assertEqual(self.server.requests[-1][0], 'disarm')

    def test_each_contact_has_fresh_token_and_trigger_is_latched(self):
        self.client.begin_contact()
        first = self.client.session
        self.server.wait_reply = dict(state='triggered', stop_required=True, trigger_time=123.5)
        self.wait_for_stop()
        self.assertEqual(self.client.confirm_contact(), 123.5)
        self.assertEqual(self.client.triggered_timestamp, 123.5)
        self.client.end_contact()
        self.server.wait_reply = None
        self.client.begin_contact()
        self.assertNotEqual(self.client.session, first)
        self.assertIsNone(self.client.triggered_timestamp)
        self.assertFalse(self.client.check())

    def test_already_triggered_arm_stops_before_motion(self):
        self.server.arm_reply = dict(state='triggered', stop_required=True, trigger_time=51.)
        self.client.begin_contact()
        self.assertTrue(self.client.check())
        self.assertEqual(self.client.confirm_contact(), 51.)

    def test_early_stop_is_not_a_valid_contact_until_classified(self):
        self.client.begin_contact()
        self.server.wait_reply = dict(state='armed', stop_required=True)
        self.wait_for_stop()
        self.assertIsNone(self.client.triggered_timestamp)
        calls = []
        def pump():
            calls.append(True)
            self.server.wait_reply = dict(state='triggered', stop_required=True, trigger_time=99.)
        self.assertEqual(self.client.confirm_contact(pump), 99.)
        self.assertTrue(calls)

    def test_fault_after_early_stop_cannot_be_accepted_as_contact(self):
        self.client.begin_contact()
        self.server.wait_reply = dict(state='armed', stop_required=True)
        self.wait_for_stop()
        self.server.wait_reply = dict(state='fault', stop_required=True, reason='MCU timeout')
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'MCU timeout'):
            self.client.confirm_contact()
        self.assertIsNone(self.client.triggered_timestamp)

    def test_confirmed_cleanup_keeps_original_native_fault_without_masking_it(self):
        self.client.begin_contact()
        self.server.disarm_reply = dict(state='fault', cleanup_confirmed=True, error='ADC fault')
        self.server.wait_reply = dict(state='fault', stop_required=True, error='ADC fault')
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'ADC fault'):
            self.wait_for_stop()
        self.client.end_contact()
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'ADC fault'):
            self.client.check()
        self.assertIsNone(self.client.triggered_timestamp)

    def test_fault_discovered_at_cleanup_rejects_an_otherwise_classified_contact(self):
        self.client.begin_contact()
        self.server.wait_reply = dict(state='triggered', stop_required=True, trigger_time=123.)
        self.wait_for_stop()
        self.assertEqual(self.client.confirm_contact(), 123.)
        self.server.disarm_reply = dict(state='fault', cleanup_confirmed=True, error='overload')
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'overload'):
            self.client.end_contact()

    def test_classified_trigger_cannot_regress_to_armed(self):
        self.client.begin_contact()
        self.server.wait_reply = dict(state='triggered', stop_required=True, trigger_time=123.)
        self.wait_for_stop()
        self.client.confirm_contact()
        self.server.wait_reply = dict(state='armed', stop_required=True)
        self.client._thread.join(.5)
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'classification changed'):
            self.client.check()

    def test_arm_can_exceed_active_stall_window_while_pump_keeps_servicing(self):
        self.server.arm_release = threading.Event()
        pumped = []
        def pump():
            pumped.append(time.monotonic())
            if self.server.arm_entered.is_set() and pumped[-1]-pumped[0] > .32:
                self.server.arm_release.set()
        self.client.begin_contact(pump)
        self.assertGreater(pumped[-1]-pumped[0], .3)
        self.assertFalse(self.client.check())

    def test_arming_deadline_rejects_before_motion_and_confirms_cleanup(self):
        now = [100.]
        self.client = KlipperProbeTrigger('http://printer:7125', opener=self.server, clock=lambda: now[0])
        self.server.arm_release = threading.Event()
        def pump():
            if self.server.arm_entered.is_set():
                now[0] = 103.251
                threading.Timer(.03, self.server.arm_release.set).start()
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'arming timed out before motion'):
            self.client.begin_contact(pump)
        self.assertEqual(self.server.requests[-1][0], 'disarm')

    def test_worker_stall_latches_fault_at_three_hundred_ms(self):
        now = [100.]
        self.client = KlipperProbeTrigger('http://printer:7125', opener=self.server, clock=lambda: now[0])
        self.server.wait_release = threading.Event()
        self.client.begin_contact()
        self.assertTrue(self.server.wait_entered.wait(.5))
        now[0] += .301
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'stalled'):
            self.client.check()
        self.server.wait_release.set()
        self.client.end_contact()
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'stalled'):
            self.client.check()

    def test_bad_session_protocol_state_or_trigger_time_fail_closed(self):
        for reply in ({'session': 'wrong'}, {'protocol': '1'}, {'state': 'disarmed'},
                      {'state': 'mystery'}, {'stop_required': 1},
                      *[dict(state='triggered', stop_required=True, trigger_time=value)
                        for value in (None, 0., -1., True, float('nan'), float('inf'))]):
            with self.subTest(reply=reply):
                self.server.wait_reply = None
                self.client.begin_contact()
                self.server.wait_reply = reply
                with self.assertRaises(KlipperProbeTriggerError):
                    self.wait_for_stop()
                self.client.end_contact()

    def test_pump_failure_during_arm_disarms_late_reply_without_reviving(self):
        self.server.arm_release = threading.Event()
        self.client = KlipperProbeTrigger('http://printer:7125', opener=self.server,
                                          cleanup_timeout_s=.05)
        def pump():
            if self.server.arm_entered.is_set():
                # Longer than active cleanup: arming has a separate deadline and
                # still must be cleaned up after an operator cancels its wait.
                threading.Timer(.1, self.server.arm_release.set).start()
                raise RuntimeError('operator abort')
        with self.assertRaisesRegex(RuntimeError, 'operator abort'):
            self.client.begin_contact(pump)
        self.assertEqual(self.server.requests[-1][0], 'disarm')
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'operator abort'):
            self.client.check()

    def test_unconfirmed_cleanup_blocks_rearming_and_preserves_error(self):
        self.client.begin_contact()
        self.server.disarm_reply = {'session': 'wrong'}
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'disarm'):
            self.client.end_contact()
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'previous native contact'):
            self.client.begin_contact()

    def test_cleanup_is_bounded_even_if_transport_ignores_timeout(self):
        self.client = KlipperProbeTrigger('http://printer:7125', opener=self.server, cleanup_timeout_s=.05)
        self.server.wait_release = threading.Event()
        self.client.begin_contact()
        self.assertTrue(self.server.wait_entered.wait(.5))
        begin = time.monotonic()
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'timed out'):
            self.client.end_contact()
        self.assertLess(time.monotonic()-begin, .2)
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'previous native contact'):
            self.client.begin_contact()
        self.server.wait_release.set()
        self.client._thread.join(.5)
        self.assertEqual(self.server.requests[-1][0], 'disarm')
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'timed out'):
            self.client.check()

    def test_unclassified_stop_times_out_without_inventing_a_timestamp(self):
        self.client = KlipperProbeTrigger('http://printer:7125', opener=self.server,
                                          classification_timeout_s=.03)
        self.client.begin_contact()
        self.server.wait_reply = dict(state='armed', stop_required=True)
        self.wait_for_stop()
        with self.assertRaisesRegex(KlipperProbeTriggerError, 'classification timed out'):
            self.client.confirm_contact()
        self.assertIsNone(self.client.triggered_timestamp)

    def test_invalid_limits_do_not_send_http_requests(self):
        for change in ({'lease_s': 1.}, {'stall_timeout_s': .31}, {'request_timeout_s': .3},
                       {'max_delta_counts': 120000.1}, {'max_delta_counts': 120001.},
                       {'max_delta_counts': 250000.}, {'max_nozzle_temp_c': 51.},
                       {'cleanup_timeout_s': 1.01}, {'classification_timeout_s': 1.01},
                       {'arm_timeout_s': 3.01}, {'arm_timeout_s': True}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                KlipperProbeTrigger('http://printer:7125', opener=self.server, **change)
        for value in (0., -1., True, float('nan'), 7200.01):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.client.begin_contact(search_timeout_s=value)
        self.assertFalse(self.server.requests)

    def test_explicit_one_hundred_twenty_thousand_limit_is_forwarded(self):
        self.client = KlipperProbeTrigger('http://printer:7125', opener=self.server,
                                          max_delta_counts=120000.)
        self.client.begin_contact()
        arm = next(r for r in self.server.requests if r[0] == 'arm')
        self.assertEqual(arm[1]['max_delta_counts'], 120000.)


if __name__ == '__main__':
    unittest.main()
