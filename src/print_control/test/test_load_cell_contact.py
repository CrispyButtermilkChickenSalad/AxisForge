"""Sensor integrity and force gates without any robot or printer hardware."""
from dataclasses import replace
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import unittest

from print_control.load_cell_contact import (
    AlpsStream, ContactDetector, ContactOptions, ForceSample, repeat_contact,
    stable_baseline, validate_printer, thermal_contact_options,
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

    def test_hot_baseline_accepts_bounded_noise_without_changing_contact_or_overload(self):
        cold=ContactOptions(trigger_mode='raw')
        hot=thermal_contact_options(cold,hot_nozzle=True)
        def raw(values):return [ForceSample(i,i*.002,i*.002,float(value)) for i,value in enumerate(values,1)]
        noisy=raw([-3500.,3500.]*20)
        with self.assertRaisesRegex(ValueError,'too noisy'):stable_baseline(noisy,{},cold)
        self.assertEqual(stable_baseline(noisy,{},hot),0.)
        self.assertEqual(hot.baseline_noise_limit,8000.)
        self.assertEqual(thermal_contact_options(hot,hot_nozzle=False),cold)
        self.assertEqual((hot.threshold,hot.maximum),(cold.threshold,cold.maximum))
        detector=ContactDetector(0.,None,hot)
        self.assertTrue(detector.update(raw([20000.]*5)))
        with self.assertRaisesRegex(RuntimeError,'overload'):
            ContactDetector(0.,None,hot).update(raw([120000.]))
        for values,reason in (([-4500.,4500.]*20,'too noisy'),
                              (list(range(-3500,3501,100)),'drifting'),
                              ([0.]*19+[6000.],'asymmetric'),
                              ([25000.-3500.,25000.+3500.]*20,'already be loaded')):
            with self.subTest(reason=reason),self.assertRaisesRegex(ValueError,reason):
                stable_baseline(raw(values),{},hot,reference_counts=0.)
        for fraction in (.24,.401):
            with self.assertRaises(ValueError):replace(cold,baseline_noise_fraction=fraction)

    def test_raw_search_guard_stops_at_one_hundred_twenty_thousand(self):
        for sign in (-1, 1):
            detector = ContactDetector(-200000., None, ContactOptions(trigger_mode='raw'))
            detector.update([ForceSample(1, .002, .002, -200000.+sign*80000.)])
            detector.update([ForceSample(2, .004, .004, -200000.+sign*119999.)])
            with self.assertRaisesRegex(RuntimeError, 'overload: 120000.0 counts exceeds 120000.0 counts'):
                detector.update([ForceSample(3, .006, .006, -200000.+sign*120000.)])

    def test_native_trigger_requires_real_calibration_even_with_raw_guards(self):
        options = ContactOptions(trigger_mode='raw', trigger_source='klipper_probe')
        data = status()
        validate_printer(data, 'load_cell_probe', options)
        for change in ({'is_calibrated': False}, {'reference_tare_counts': None}):
            data['load_cell_probe'].update(change)
            with self.assertRaisesRegex(RuntimeError, 'not calibrated'):
                validate_printer(data, 'load_cell_probe', options)

    def test_native_trigger_rejects_unknown_sources_and_ambiguous_guard_units(self):
        for change in ({'trigger_source': 'query_probe'}, {'trigger_source': 'klipper_probe'}):
            with self.assertRaises(ValueError): ContactOptions(**change)

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
        for change in ({'step_mm': .1}, {'speed_mm_s': .6}, {'max_force_g': 40}, {'samples': 0},
                       {'samples': True}, {'retract_mm': .02}, {'max_sample_age_s': 2},
                       {'contact_force_g': float('nan')}, {'consecutive_samples': 1}):
            with self.subTest(change=change), self.assertRaises(ValueError): ContactOptions(**change)

    def test_single_contact_returns_height_without_claiming_repeatability(self):
        self.assertEqual(ContactOptions(samples=1).samples, 1)
        reading = {'nozzle': [0., 0., 100.]}
        chosen, spread = repeat_contact([reading], .02)
        self.assertIs(chosen, reading)
        self.assertIsNone(spread)
        with self.assertRaises(ValueError): repeat_contact([], .02)


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

    def test_delivery_catchup_rebases_the_whole_interval_without_reversing_time(self):
        self.source.ingest(self.batch())
        before=self.source.snapshot()
        # Source advances by 100 ms; its next packet arrives 3 ms sooner.
        # Per-batch anchoring used to put the next sample before the last one.
        self.now += .097
        self.source.ingest(self.batch(100.1))
        samples=self.source.snapshot()
        self.assertTrue(all(b.observed_at>=a.observed_at for a,b in zip(samples,samples[1:])))
        self.assertTrue(all(new.observed_at<=old.observed_at for old,new in zip(before,samples)))
        self.assertEqual([s.counts for s in samples[:50]],[s.counts for s in before])
        self.assertEqual([s.source_time for s in samples[:50]],[s.source_time for s in before])
        for sample in samples:
            self.assertAlmostEqual(sample.observed_at,sample.source_time+self.source.min_offset)

    def test_transport_delay_is_kept_in_sample_age_instead_of_reanchoring_as_fresh(self):
        self.source.ingest(self.batch())
        self.now += .125
        self.source.ingest(self.batch(100.1))
        self.assertAlmostEqual(self.now-self.source.snapshot()[-1].observed_at,.025)
        self.now += .075
        self.source.ingest(self.batch(100.2))
        samples=self.source.snapshot()
        self.assertTrue(all(b.observed_at>=a.observed_at for a,b in zip(samples,samples[1:])))
        self.assertTrue(all(s.observed_at<=self.now for s in samples))

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
