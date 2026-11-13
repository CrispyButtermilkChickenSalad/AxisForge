"""Sensor integrity and force gates without any robot or printer hardware."""
from dataclasses import replace
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import unittest

from print_control.load_cell_contact import (
    AlpsStream, ContactDetector, ContactOptions, ForceSample, repeat_contact,
    stable_baseline, validate_printer,
)


def status():
    return {'webhooks': {'state': 'ready'},
            'load_cell_probe': {'is_calibrated': True, 'counts_per_gram': 100.,
                                'reference_tare_counts': 1000., 'errors': 0, 'overflows': 0},
            'extruder': {'temperature': 25., 'target': 0.}, 'axisforge_sync': {}}


def samples(forces, start=1):
    return [ForceSample(i, i*.002, i*.002, 1000.+100.*force)
            for i, force in enumerate(forces, start)]


class ContactTests(unittest.TestCase):
    def setUp(self): self.options = ContactOptions()

    def test_uncalibrated_or_invalid_scale_never_allows_probing(self):
        for change in ({'is_calibrated': False}, {'reference_tare_counts': None},
                       {'counts_per_gram': 0}, {'counts_per_gram': float('nan')}):
            data = status(); data['load_cell_probe'].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(RuntimeError, 'not calibrated'):
                validate_printer(data, 'load_cell_probe', self.options)
        data = status(); data['load_cell_probe']['is_calibrated'] = False
        self.assertFalse(validate_printer(data, 'load_cell_probe', self.options, False)['is_calibrated'])

    def test_printer_interlocks(self):
        changes = [('webhooks', {'state': 'shutdown'}), ('load_cell_probe', {}),
                   ('load_cell_probe', {'errors': 1}), ('load_cell_probe', {'overflows': 1}),
                   ('axisforge_sync', {'leased': True}), ('axisforge_sync', {'preparing': True}),
                   ('axisforge_sync', {'reservation': 'job'}), ('virtual_sdcard', {'is_active': True}),
                   ('print_stats', {'state': 'paused'}), ('extruder', {'temperature': 70., 'target': 0.}),
                   ('extruder', {'temperature': 25., 'target': 200.}), ('heater_bed', {'target': 60.})]
        for name, value in changes:
            data = status(); data[name] = value
            with self.subTest(name=name, value=value), self.assertRaises(RuntimeError):
                validate_printer(data, 'load_cell_probe', self.options)

    def test_positive_and_negative_contact_use_distinct_consecutive_samples(self):
        for sign in (-1, 1):
            detector = ContactDetector(1000., 100., self.options)
            first = samples([60*sign]*4)
            self.assertFalse(detector.update(first))
            self.assertFalse(detector.update(first))  # Repeated polling is not another sample.
            self.assertTrue(detector.update(samples([60*sign], 5)))

    def test_spike_is_not_contact_but_single_overload_stops(self):
        detector = ContactDetector(1000., 100., self.options)
        self.assertFalse(detector.update(samples([0, 55, 0, 55, 0, 55, 0])))
        with self.assertRaisesRegex(RuntimeError, 'overload'):
            detector.update(samples([-201], 8))

    def test_baseline_rejects_noise_and_already_loaded_nozzle(self):
        sensor = status()['load_cell_probe']
        self.assertEqual(stable_baseline(samples([1]*20), sensor, self.options), 1100.)
        for forces in ([0, 20]*10, [60]*20, [0]*9):
            with self.subTest(forces=forces), self.assertRaises(ValueError):
                stable_baseline(samples(forces), sensor, self.options)

    def test_repeatability_uses_an_actual_median_touch(self):
        readings = [{'nozzle': [0, 0, z]} for z in (100.01, 100.04, 100.02)]
        chosen, spread = repeat_contact(readings, .05)
        self.assertIs(chosen, readings[2]); self.assertAlmostEqual(spread, .03)
        with self.assertRaisesRegex(ValueError, 'repeatability'): repeat_contact(readings, .02)

    def test_invalid_contact_settings_are_rejected(self):
        for change in ({'step_mm': .1}, {'speed_mm_s': .6}, {'max_force_g': 40}, {'samples': 1},
                       {'samples': True}, {'retract_mm': .02}, {'max_sample_age_s': 2},
                       {'contact_force_g': float('nan')}, {'consecutive_samples': 1}):
            with self.subTest(change=change), self.assertRaises(ValueError): ContactOptions(**change)


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.source = AlpsStream('http://printer:7125', clock=lambda: self.now)
        self.source.health_at = self.now

    def batch(self, start=100.):
        return {'now': start+.098, 'rows': [[start+i*.002, 1000+i, None] for i in range(50)], 'errors': 0}

    def test_raw_counts_are_valid_before_calibration_and_expire(self):
        self.source.ingest(self.batch())
        self.assertEqual(len(self.source.snapshot()), 50)
        self.assertEqual(self.source.snapshot()[-1].counts, 1049.)
        self.now += .31
        with self.assertRaisesRegex(RuntimeError, 'stale'): self.source.snapshot()

    def test_missing_health_and_latched_failure_block_fresh_samples(self):
        self.source.ingest(self.batch()); self.source.health_at = 8.
        with self.assertRaisesRegex(RuntimeError, 'status is stale'): self.source.snapshot()
        self.source.health_at = self.now
        self.source._fail('first error'); self.source._fail('second error')
        with self.assertRaisesRegex(RuntimeError, 'first error'): self.source.snapshot()

    def test_replayed_or_gapped_samples_are_rejected(self):
        self.source.ingest(self.batch()); self.now += .1
        with self.assertRaisesRegex(RuntimeError, 'repeated'): self.source.ingest(self.batch())
        self.now += .1
        with self.assertRaisesRegex(RuntimeError, 'gap'): self.source.ingest(self.batch(100.2))

    def test_source_clock_jump_or_transport_backlog_is_rejected(self):
        for elapsed, stamp in ((.5, 100.1), (.1, 101.)):
            self.setUp(); self.source.ingest(self.batch()); self.now += elapsed
            with self.subTest(elapsed=elapsed), self.assertRaisesRegex(RuntimeError, 'clock changed'):
                self.source.ingest(self.batch(stamp))

    def test_invalid_stream_data(self):
        for bad in ({'error': 'ADC disconnected'}, {'errors': 1}, [], {'rows': [], 'now': 100},
                    {'rows': [[100, None, None]], 'now': 100},
                    {'rows': [[99, 100, None]], 'now': 100},
                    {'rows': [[100, float('nan'), None]], 'now': 100}):
            with self.subTest(batch=bad), self.assertRaises(RuntimeError): self.source.ingest(bad)

    def test_http_stream_and_status_calibration_change(self):
        printer = status(); stopped = threading.Event()
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def do_GET(self):
                self.send_response(200); self.end_headers()
                if self.path.startswith('/printer/objects/query'):
                    self.wfile.write(json.dumps({'result': {'status': printer}}).encode()); return
                try:
                    while not stopped.is_set():
                        stamp = time.time()
                        batch = {'now': stamp, 'rows': [[stamp-.198+i*.002, 1000, None] for i in range(100)]}
                        self.wfile.write(json.dumps(batch).encode()+b'\n'); self.wfile.flush()
                        stopped.wait(.2)
                except (BrokenPipeError, ConnectionResetError): pass
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
        url = 'http://127.0.0.1:'+str(server.server_port)
        source = AlpsStream(url, url)
        try:
            source.start()
            self.assertGreater(len(source.wait_window(source.clock())), 10)
            printer['load_cell_probe']['counts_per_gram'] = 200.
            deadline = time.monotonic()+1.5
            while source.error is None and time.monotonic() < deadline: time.sleep(.01)
            with self.assertRaisesRegex(RuntimeError, 'calibration changed'): source.snapshot()
        finally:
            stopped.set(); source.close(); server.shutdown(); server.server_close()


if __name__ == '__main__': unittest.main()
