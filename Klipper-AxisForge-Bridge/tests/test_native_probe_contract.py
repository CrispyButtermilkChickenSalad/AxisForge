"""Real client/server protocol agreement with a non-actuating Klipper double."""
import io
import json
from pathlib import Path
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/print_control'))
from print_control.klipper_probe_trigger import KlipperProbeTrigger
from test_klipper_probe import Fixture, Request


class NativeProtocolTests(unittest.TestCase):
    def setUp(self):
        self.fixture = Fixture()
        self.event = threading.Event()
        self.reason = 1
        self.client = KlipperProbeTrigger('http://test:7125', opener=self.transport)
        self.addCleanup(self.client.end_contact)

    def transport(self, request, timeout):
        endpoint = request.full_url.rsplit('/', 1)[-1]
        values = {} if request.data is None else json.loads(request.data)
        fixture = self.fixture
        if endpoint == 'wait':
            # The mocked MCU is stationary until the test explicitly releases
            # its event. Execute the adapter's real completion classifier.
            if self.event.wait(.01) and fixture.adapter._native_started:
                fixture.trigger.contact(reason=self.reason)
                fixture.adapter._monitor_native(fixture.reactor.now,
                    fixture.adapter.generation, fixture.trigger.completion)
            values['timeout_s'] = 0.
        query = Request(**values)
        getattr(fixture.adapter, 'api_'+endpoint)(query)
        return io.BytesIO(json.dumps({'result': query.response}).encode())

    def test_native_contact_and_confirmed_disarm_use_matching_protocol(self):
        self.assertTrue(self.client.validate_ready()['calibrated'])
        self.client.begin_contact()
        self.assertTrue(self.fixture.adapter.is_busy())
        self.event.set()
        deadline = time.monotonic()+1.
        while not self.client.check() and time.monotonic() < deadline: time.sleep(.005)
        self.assertGreater(self.client.confirm_contact(), 0.)
        self.assertTrue(self.fixture.adapter.is_busy())
        self.client.end_contact()
        self.assertFalse(self.fixture.adapter.is_busy())
        self.assertEqual(self.fixture.trigger.stopped, 1)
        self.assertEqual(self.fixture.trigger.cleared, 1)

    def test_heated_native_contact_uses_selected_targets_and_retains_overload_guard(self):
        self.fixture.target=self.fixture.temperature=210.
        self.client=KlipperProbeTrigger('http://test:7125',opener=self.transport,
            thermal_targets={'extruder':210.})
        self.addCleanup(self.client.end_contact)
        self.assertTrue(self.client.validate_ready()['heated_probing'])
        self.client.begin_contact()
        self.assertTrue(self.fixture.adapter.is_busy())
        self.assertEqual(self.fixture.adapter._thermal_targets,{'extruder':210.})
        self.assertEqual(self.fixture.trigger._raw_max-self.fixture.tare,120000.)
        self.client.end_contact()
        self.assertEqual(self.fixture.target,210.,'Probing must not issue heater commands')

    def test_mcu_fault_is_not_contact_and_cleanup_preserves_original_error(self):
        self.client.begin_contact()
        self.reason = 4
        self.event.set()
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            try:
                self.client.check()
            except RuntimeError as error:
                self.assertIn('communication timeout', str(error))
                break
            time.sleep(.005)
        else: self.fail('MCU fault was not delivered')
        self.client.end_contact()
        self.assertIsNone(self.client.triggered_timestamp)
        self.assertTrue(self.fixture.adapter.cleanup_confirmed)
