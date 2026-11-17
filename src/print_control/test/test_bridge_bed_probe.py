"""Bridge homing/mesh behavior without printer or robot hardware."""
from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from spatialmath import SE3
import yaml

from print_control.load_cell_contact import (
    ADC_MIN_COUNTS, ADC_MAX_COUNTS, AlpsStream, ContactDetector, ContactOptions,
    ForceSample, stable_baseline, validate_printer,
)
from print_control.probe_control import EVENT_PREFIX, ProbeControl
from print_control.bed_probe import grid_points
from print_control.bed_surface import BedSurface
from print_control.bridge_bed_probe import BridgeBedProbe, HomingOptions, load_profile
from print_control.gcode_path import GCodeConfig, GCodePlanner
from print_control.load_cell_bed_probe import LoadCellBedProbe, SimulatedAlps
from print_control.motion_planner import MotionConfig, MotionPlanner
from print_control.probe_motion import plan_probe_descent


def raw_samples(values):
    return [ForceSample(i + 1, i * .002, i * .002, float(value))
            for i, value in enumerate(values)]


def uncalibrated_printer():
    return {
        'webhooks': {'state': 'ready'},
        'load_cell_probe': {'is_calibrated': False, 'counts_per_gram': 138.,
                            'reference_tare_counts': None, 'errors': 0, 'overflows': 0},
        'extruder': {'temperature': 25., 'target': 0.},
        'axisforge_sync': {},
    }


class RawContactModeTests(unittest.TestCase):
    def setUp(self):
        self.options = ContactOptions(trigger_mode='raw', contact_delta_counts=20000.,
                                      max_delta_counts=60000.)

    def test_uncalibrated_raw_contact_has_no_assumed_grams_conversion(self):
        sensor = validate_printer(uncalibrated_printer(), 'load_cell_probe', self.options)
        self.assertFalse(sensor['is_calibrated'])
        for scale in (None, 138., 1000., float('nan')):
            with self.subTest(scale=scale):
                detector = ContactDetector(-200000., scale, self.options)
                self.assertEqual(detector.load(-179999.), 20001.)
                self.assertTrue(detector.update(raw_samples([-179999.] * 5)))
                self.assertEqual(self.options.unit, 'counts')

    def test_raw_mode_keeps_thermal_and_job_interlocks(self):
        for key, change in (
                ('extruder', {'temperature': 80., 'target': 0.}),
                ('extruder', {'temperature': 25., 'target': 200.}),
                ('heater_bed', {'temperature': 25., 'target': 60.}),
                ('axisforge_sync', {'leased': True}),
                ('print_stats', {'state': 'printing'}),
                ('load_cell_probe', {'is_calibrated': False, 'errors': 1})):
            data = uncalibrated_printer(); data[key] = change
            with self.subTest(key=key, change=change), self.assertRaises(RuntimeError):
                validate_printer(data, 'load_cell_probe', self.options)

    def test_raw_baseline_uses_retained_unloaded_reference(self):
        sensor = uncalibrated_printer()['load_cell_probe']
        first = stable_baseline(raw_samples([-200000.] * 20), sensor, self.options)
        self.assertEqual(first, -200000.)
        # Re-zeroing a loaded nozzle must not hide the existing load.
        for shifted in (-225000., -175000.):
            with self.subTest(shifted=shifted), self.assertRaisesRegex(ValueError, 'already be loaded'):
                stable_baseline(raw_samples([shifted] * 20), sensor, self.options,
                                reference_counts=first)

    def test_raw_noise_spikes_and_both_polarities(self):
        sensor = uncalibrated_printer()['load_cell_probe']
        with self.assertRaisesRegex(ValueError, 'too noisy'):
            stable_baseline(raw_samples([-204000., -196000.] * 10), sensor, self.options)
        for polarity in (-1., 1.):
            detector = ContactDetector(-200000., None, self.options)
            transient = raw_samples([-200000., -200000. + polarity * 25000., -200000.])
            self.assertFalse(detector.update(transient))
            overload = replace(transient[-1], sequence=4, counts=-200000. + polarity * 60001.)
            with self.assertRaisesRegex(RuntimeError, 'overload.*counts'):
                detector.update([overload])

    def test_raw_baseline_requires_adc_headroom_for_the_maximum_delta(self):
        sensor = uncalibrated_printer()['load_cell_probe']
        for counts in (ADC_MIN_COUNTS, ADC_MIN_COUNTS+self.options.maximum,
                       ADC_MAX_COUNTS-self.options.maximum, ADC_MAX_COUNTS):
            with self.subTest(counts=counts), self.assertRaisesRegex(ValueError, 'headroom'):
                stable_baseline(raw_samples([counts]*20), sensor, self.options)
        for counts in (ADC_MIN_COUNTS+self.options.maximum+1.,
                       ADC_MAX_COUNTS-self.options.maximum-1.):
            self.assertEqual(stable_baseline(raw_samples([counts]*20), sensor, self.options), counts)
        with self.assertRaisesRegex(ValueError, 'headroom'):
            replace(self.options, max_delta_counts=float(ADC_MAX_COUNTS))

    def test_sample_stream_rejects_adc_saturation_and_out_of_range_counts(self):
        source = AlpsStream('http://unused:7125', options=self.options, clock=lambda: 100.)
        for counts in (ADC_MIN_COUNTS-1, ADC_MIN_COUNTS, ADC_MAX_COUNTS, ADC_MAX_COUNTS+1):
            with self.subTest(counts=counts), self.assertRaisesRegex(RuntimeError, 'saturated'):
                source.ingest({'now': 100., 'rows': [[100., counts, None]]})

    def test_advancing_timestamps_cannot_hide_a_frozen_adc(self):
        now = [100.]
        source = AlpsStream('http://unused:7125', options=self.options, clock=lambda: now[0])
        for batch in range(12):
            start = 100.+batch*.1
            now[0] = start+.098
            source.health_at = now[0]
            source.ingest({'now': now[0], 'rows': [[start+i*.002, -200000., None] for i in range(50)]})
        with self.assertRaisesRegex(RuntimeError, 'frozen despite advancing'):
            source.snapshot()


class ProbeControlTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.output = io.StringIO()
        self.control = ProbeControl(io.StringIO(), self.output, clock=lambda: self.now,
                                    heartbeat_timeout_s=5., confirmation_timeout_s=10.)

    def events(self):
        return [json.loads(line[len(EVENT_PREFIX):]) for line in self.output.getvalue().splitlines()]

    def test_confirmation_waits_for_matching_stage_and_fresh_single_use_token(self):
        seen = []
        def confirm():
            pending = dict(self.control.pending)
            seen.append(pending)
            command = {'command': 'confirm', **pending}
            self.assertFalse(self.control.ingest({**command, 'stage': 'front_left'}))
            self.assertFalse(self.control.ingest({**command, 'token': 'stale'}))
            self.assertTrue(self.control.ingest(command))
            self.assertFalse(self.control.ingest(command))
        self.control.wait_confirmation('surface', 'Place the bed', confirm, progress=0.)
        self.assertEqual(len(seen), 1)
        self.assertIsNone(self.control.pending)
        self.assertFalse(self.control.ingest({'command': 'confirm', **seen[0]}))
        first_event = self.events()[0]
        self.assertEqual(first_event['state'], 'waiting_surface')
        self.assertEqual(first_event['confirmation'], seen[0])

        def front_left():
            self.assertFalse(self.control.ingest({'command': 'confirm', **seen[0]}))
            self.assertNotEqual(self.control.pending['token'], seen[0]['token'])
            self.assertTrue(self.control.ingest({'command': 'confirm', **self.control.pending}))
        self.control.wait_confirmation('front_left', 'Check the mesh origin', front_left)
        self.assertEqual(self.events()[-1]['state'], 'waiting_front_left')

    def test_no_confirmation_is_inferred_from_heartbeats_or_elapsed_time(self):
        pumps = []
        def wait():
            pumps.append(self.now)
            self.now += 2.
            self.control.ingest({'command': 'heartbeat'})
        with self.assertRaisesRegex(RuntimeError, 'Timed out waiting for surface'):
            self.control.wait_confirmation('surface', 'Confirm', wait)
        self.assertEqual(len(pumps), 5)
        self.assertIsNone(self.control.pending)
        self.assertFalse(self.control.confirmed)

    def test_heartbeat_timeout_latches_and_cannot_be_recovered_by_late_input(self):
        self.now += 5.01
        with self.assertRaisesRegex(RuntimeError, 'heartbeat timed out'):
            self.control.check()
        self.control.ingest({'command': 'heartbeat'})
        with self.assertRaisesRegex(RuntimeError, 'heartbeat timed out'):
            self.control.check()

    def test_reader_cannot_revive_expired_heartbeat_before_main_loop_checks(self):
        self.now += 5.01
        self.assertFalse(self.control.ingest({'command': 'heartbeat'}))
        with self.assertRaisesRegex(RuntimeError, 'heartbeat timed out'):
            self.control.check()

    def test_matching_confirmation_received_after_deadline_cannot_start_motion(self):
        def late_confirm():
            self.now += 10.01
            # Keep heartbeat valid independently: this is a confirmation expiry.
            self.control.last_heartbeat = self.now
            self.assertFalse(self.control.ingest({'command': 'confirm', **self.control.pending}))
        with self.assertRaisesRegex(RuntimeError, 'Timed out waiting for surface'):
            self.control.wait_confirmation('surface', 'Confirm', late_confirm)
        self.assertIsNone(self.control.pending)

    def test_eof_and_invalid_input_cancel_instead_of_confirming(self):
        for payload, reason in (('', 'pipe closed'), ('not json\n', 'input failed'),
                                ('[]\n', 'Expected a control object'),
                                ('x' * 65537, 'exceeds limit')):
            control = ProbeControl(io.StringIO(payload), io.StringIO())
            control._read()
            with self.subTest(reason=reason), self.assertRaisesRegex(RuntimeError, reason):
                control.check()
            self.assertFalse(control.confirmed)

    def test_cancel_prevents_confirmation_and_preserves_first_failure(self):
        def abort():
            pending = dict(self.control.pending)
            self.control.ingest({'command': 'abort'})
            self.assertFalse(self.control.ingest({'command': 'confirm', **pending}))
            self.control.cancel('Later error must not replace operator cancellation')
        with self.assertRaisesRegex(RuntimeError, 'Operator cancelled'):
            self.control.wait_confirmation('surface', 'Confirm', abort)
        self.assertIsNone(self.control.pending)

    def test_invalid_stage_and_nonfinite_event_payload_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Unknown confirmation stage'):
            self.control.wait_confirmation('auto_continue', 'Confirm', lambda: None)
        with self.assertRaises(ValueError):
            self.control.emit('meshing', 'Invalid data', progress=float('nan'))


class GuidedProbeHarness(BridgeBedProbe):
    """Run real orchestration/saving with deterministic contacts; no ROS node."""
    def __init__(self, output):
        self.settings = GCodeConfig(z_offset=100., bed_probe_file='')
        self.config = MotionConfig(dry_run=True, clearance_mm=6., tcp_offset_mm=(-100., 0., 117.06))
        rotation = SE3.RPY(self.config.nozzle_rpy_deg, unit='deg', order='zyx').R
        self.planner = SimpleNamespace(config=self.config, rotation=rotation,
            kinematic_model={'kind':'nominal_ur16e'},
                                      check_pose=lambda *args, **kwargs: None,
                                      kinematics=SimpleNamespace(eval=lambda q: SE3.Rt(rotation, np.zeros(3)).A))
        self.args = SimpleNamespace(config='unused', floor_mm=3., paper_mm=0., load_cell='load_cell_probe',margin_mm=30.)
        self.output = Path(output)
        self.points = grid_points(self.settings.bed_size_mm, 30.)
        self.records = {}; self.index = 0; self.promoted = False; self.returned = False
        self.start_xyz = self.start_joints = self.target = self.seed = None
        self.initial_xyz = np.array([-580., -240., 105.])
        self.homing_readings = []; self.homed_z = None
        self.contact_options = ContactOptions(trigger_mode='raw', contact_offset_mm=.02)
        self.homing = HomingOptions()
        self.source = SimulatedAlps(self)
        self.reference_counts = None; self.detector = None; self.seeking = False
        self.control = ProbeControl(io.StringIO(), io.StringIO(), clock=lambda: 100.)
        self.moves = []; self.confirmed_stages = []; self.touches = []; self.saved = []
        self.reject_stage = None; self.fail_return = False; self.shift_on_confirm = None
        self.contacts_z = [99.8, 99.81]
        self.stop_unconfirmed = False

    def capture_start(self, *, allow_alignment=False):
        self._nominal_probe_rotation = self.planner.rotation.copy()
        self.start_xyz = self.initial_xyz.copy(); self.target = self.start_xyz.copy()
        self.start_joints = np.zeros(6); self.seed = self.start_joints.copy()

    def robot_reason(self): return ''

    def manual_z_session(self, **kwargs):
        # Routine orchestration tests skip operator steps. The shared Z-control
        # tests exercise their movement and load guards independently.
        self.start_xyz, self.start_joints = self.target.copy(), self.seed.copy()

    def nozzle_now(self): return self.target.copy(), self.seed.copy()

    def z_waiting_anchor(self): return self.nozzle_now()

    def pump(self):
        pending = self.control.pending
        if pending is not None:
            if pending['stage'] == self.reject_stage:
                self.control.cancel('Operator declined '+pending['stage'])
                raise RuntimeError('Operator declined '+pending['stage'])
            self.confirmed_stages.append(pending['stage'])
            if self.shift_on_confirm is not None: self.target += self.shift_on_confirm
            self.control.ingest({'command': 'confirm', **pending})

    def check_interrupted(self): self.control.check()

    def line_to(self, end, name, speed):
        if self.fail_return and name == 'return starting Z': raise RuntimeError('Return motion failed')
        self.moves.append({'from': self.target.copy(), 'to': np.array(end), 'name': name,
                           'speed': speed, 'confirmed': list(self.confirmed_stages),
                           'state': self.control.state})
        self.target = np.array(end)

    def touch_once(self, *, floor_z=None, speed_mm_s=None, step_mm=None):
        self.touches.append({'floor': floor_z, 'speed': speed_mm_s, 'step': step_mm,
                             'xy': self.target[:2].copy(), 'confirmed': list(self.confirmed_stages)})
        self.move_z(self.contacts_z[len(self.touches)-1], 'simulated homing descent', speed_mm_s)
        return {'nozzle': self.target.copy(), 'joints': self.seed.copy(), 'load': 20500.,
                'peak_load': 20500., 'tare_counts': 0.}

    def homing_touch(self, *, floor_z, speed_mm_s, locating_only=False, precision_reference_z=None):
        result = self.touch_once(floor_z=floor_z, speed_mm_s=speed_mm_s)
        result['contact_persistence_checked'] = not locating_only
        return result

    def retract_contact(self, *, distance_mm=None, speed_mm_s=None):
        self.move_z(self.target[2]+distance_mm, 'contact retract',
                    self.homing.lift_speed_mm_s if speed_mm_s is None else speed_mm_s)

    def probe_current(self):
        z = 99.82+self.index*.001
        nozzle = self.target.copy(); nozzle[2] = z
        self.records[self.index] = {'point': self.index+1, 'grid_xy': list(self.points[self.index]),
                                   'nozzle_base_mm': nozzle.tolist(), 'gcode_xy':self.gcode_xy(nozzle).tolist(), 'bed_z_mm': z,
                                   'joints_rad': self.seed.tolist()}
        self.save()

    def save(self, *, promote=True):
        self.saved.append((len(self.records), self.returned))
        return super().save(promote=promote)


class GuidedRoutineTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.output = Path(self.temp.name)/'bed_probe.yaml'; self.output.write_text('old mesh\n')
        self.probe = GuidedProbeHarness(self.output)

    def test_alignment_precedes_z_positioning_and_homing_and_is_saved(self):
        order=[]
        baseline=self.probe.set_baseline
        positioning=self.probe.manual_z_session
        def tare():
            order.append('tare');baseline()
        def align():
            self.assertEqual(self.probe.confirmed_stages,['surface'])
            self.assertIsNotNone(self.probe.detector)
            self.assertEqual(self.probe.moves,[])
            order.append('align')
            self.probe.start_alignment={'required':False,'state':'aligned','residual_deg':.001}
        def position(**kwargs):
            order.append('position');positioning(**kwargs)
        with patch.object(self.probe,'set_baseline',side_effect=tare), \
             patch.object(self.probe,'align_homing_start',side_effect=align), \
             patch.object(self.probe,'manual_z_session',side_effect=position):
            self.probe.run_routine()
        self.assertEqual(order[:3],['tare','align','position'])
        self.assertEqual(len(self.probe.touches),2)
        self.assertEqual(yaml.safe_load(self.output.read_text())['start_alignment']['state'],'aligned')

    def test_failed_alignment_never_descends_or_overwrites_existing_mesh(self):
        with patch.object(self.probe,'align_homing_start',side_effect=RuntimeError('Alignment failed')), \
             patch.object(self.probe,'manual_z_session') as positioning:
            with self.assertRaisesRegex(RuntimeError,'Alignment failed'):
                self.probe.run_routine()
            positioning.assert_not_called()
        self.assertEqual(self.probe.moves,[])
        self.assertEqual(self.probe.touches,[])
        self.assertEqual(self.output.read_text(),'old mesh\n')

    def test_routine_orders_confirmations_contacts_transit_mesh_and_safe_return(self):
        self.probe.run_routine()
        self.assertEqual(self.probe.confirmed_stages, ['surface', 'front_left'])
        self.assertEqual(len(self.probe.touches), 2)
        first, second = self.probe.touches
        self.assertEqual([r['contact_persistence_checked'] for r in self.probe.homing_readings],
                         [False, True])
        self.assertEqual(first['floor'], self.probe.start_xyz[2] - 350.)
        self.assertEqual(first['speed'], 5.)
        self.assertEqual(second['speed'], .2)
        self.assertIsNone(first['step'])
        self.assertIsNone(second['step'])
        self.assertAlmostEqual(second['floor'], 99.3)
        for touch in self.probe.touches:
            np.testing.assert_equal(touch['xy'], self.probe.initial_xyz[:2])
            self.assertEqual(touch['confirmed'], ['surface'])
        self.assertAlmostEqual(self.probe.homed_z, 99.83)
        self.assertAlmostEqual(self.probe.settings.z_offset, self.probe.homed_z)
        transit_index = next(i for i, m in enumerate(self.probe.moves) if m['name'] == 'front-left travel')
        lift, transit = self.probe.moves[transit_index-1:transit_index+1]
        self.assertEqual(lift['name'], 'homing clearance lift')
        np.testing.assert_equal(lift['from'][:2], lift['to'][:2])
        self.assertEqual(lift['to'][2], self.probe.initial_xyz[2]+10.)
        self.assertEqual(transit['from'][2], transit['to'][2])
        self.assertEqual(lift['speed'], 10.)
        self.assertEqual(transit['speed'], self.probe.settings.max_travel_speed_mm_s)
        np.testing.assert_equal(transit['to'][:2], self.probe.bed_point(self.probe.points[0])[:2])
        for move in self.probe.moves:
            self.assertIn('surface', move['confirmed'])
            if move['name'] in ('mesh hover', 'mesh travel'):
                self.assertIn('front_left', move['confirmed'])
            if move['name'] in ('contact retract', 'mesh lift', 'return clearance lift'):
                self.assertEqual(move['speed'], 10.)
            if move['name'] in ('mesh travel', 'return XY'):
                self.assertEqual(move['speed'], self.probe.settings.max_travel_speed_mm_s)
        final = self.probe.moves[-3:]
        self.assertEqual([m['name'] for m in final], ['return clearance lift', 'return XY', 'return starting Z'])
        np.testing.assert_equal(final[0]['from'][:2], final[0]['to'][:2])
        self.assertEqual(final[1]['from'][2], final[1]['to'][2])
        np.testing.assert_equal(final[2]['from'][:2], final[2]['to'][:2])
        np.testing.assert_equal(self.probe.target, self.probe.initial_xyz)
        self.assertEqual(self.probe.saved[-2:], [(9, False), (9, True)])
        data = yaml.safe_load(self.output.read_text())
        self.assertTrue(data['complete']); self.assertTrue(data['homing']['returned_to_start'])
        self.assertEqual(data['load_cell']['trigger_mode'], 'raw')

    def test_rectangular_and_boundary_meshes_finish_only_after_all_points_and_return(self):
        from print_control.bed_surface import BedSurface
        for columns, rows in ((2, 2), (2, 9), (9, 9)):
            with self.subTest(columns=columns, rows=rows):
                probe = GuidedProbeHarness(self.output)
                probe.homing = replace(probe.homing, mesh_columns=columns, mesh_rows=rows)
                probe.args.margin_mm = 30.
                probe.points = probe.probe_grid()
                probe.run_routine()
                self.assertEqual(probe.saved[-2:], [(columns*rows, False), (columns*rows, True)])
                data = yaml.safe_load(self.output.read_text())
                self.assertEqual(data['mesh_shape'], {'columns': columns, 'rows': rows})
                self.assertEqual(len(data['points']), columns*rows)
                # Every supported map size also fits the printing bed interpolator.
                surface = BedSurface([r['nozzle_base_mm'][:2] for r in data['points']],
                                     [r['bed_z_mm'] for r in data['points']], 0.)
                first = data['points'][0]
                self.assertAlmostEqual(surface.height(first['nozzle_base_mm'][:2]), first['bed_z_mm'])

    def test_lift_speed_applies_to_mesh_retract_but_not_explicit_scale_release(self):
        with patch.object(LoadCellBedProbe, 'retract_contact') as retract:
            BridgeBedProbe.retract_contact(self.probe)
            retract.assert_called_with(distance_mm=None, speed_mm_s=10.)
            self.probe.homing = replace(self.probe.homing, lift_speed_mm_s=15.)
            BridgeBedProbe.retract_contact(self.probe, distance_mm=3.)
            retract.assert_called_with(distance_mm=3., speed_mm_s=15.)
            BridgeBedProbe.retract_contact(self.probe, distance_mm=5., speed_mm_s=.5)
            retract.assert_called_with(distance_mm=5., speed_mm_s=.5)

    def test_xy_uses_printing_travel_speed_with_legacy_override_only_lowering_it(self):
        self.probe.settings = replace(self.probe.settings, max_travel_speed_mm_s=120.)
        self.assertEqual(self.probe.xy_travel_speed, 120.)
        self.probe.homing = replace(self.probe.homing, travel_speed_mm_s=30.)
        self.assertEqual(self.probe.xy_travel_speed, 30.)
        self.probe.homing = replace(self.probe.homing, travel_speed_mm_s=200.)
        self.assertEqual(self.probe.xy_travel_speed, 120.)

    def test_declining_surface_confirmation_leaves_robot_and_mesh_untouched(self):
        self.probe.reject_stage = 'surface'
        with self.assertRaisesRegex(RuntimeError, 'declined surface'): self.probe.run_routine()
        self.assertFalse(self.probe.moves); self.assertFalse(self.probe.touches)
        self.assertIsNone(self.probe.reference_counts)
        self.assertEqual(self.output.read_text(), 'old mesh\n')

    def test_declining_front_left_confirmation_never_starts_mesh(self):
        self.probe.reject_stage = 'front_left'
        with self.assertRaisesRegex(RuntimeError, 'declined front_left'): self.probe.run_routine()
        self.assertEqual(len(self.probe.touches), 2); self.assertFalse(self.probe.records)
        self.assertEqual(self.probe.moves[-1]['name'], 'front-left travel')
        self.assertEqual(self.output.read_text(), 'old mesh\n')

    def test_robot_jog_during_confirmation_rejects_descent(self):
        self.probe.shift_on_confirm = np.array([.2, 0., 0.])
        with self.assertRaisesRegex(RuntimeError, 'moved while awaiting confirmation'):
            self.probe.run_routine()
        self.assertFalse(self.probe.moves); self.assertFalse(self.probe.touches)

    def test_confirmation_uses_fixed_median_reference_instead_of_opposite_raw_noise_extremes(self):
        self.probe.capture_start()
        anchor = (self.probe.target.copy(), self.probe.seed.copy())
        def confirm(stage, message, pump, **kwargs):
            for noise in (-.07, .07):
                self.probe.target = anchor[0]+[noise, 0., 0.]
                pump()
        with patch.object(self.probe, 'z_waiting_anchor', return_value=anchor), \
                patch.object(self.probe.control, 'wait_confirmation', side_effect=confirm):
            self.probe.confirm_position('front_left', 'Confirm')
        np.testing.assert_array_equal(self.probe.target, anchor[0])

    def test_motion_during_confirmation_is_rejected_before_return_to_original_pose(self):
        self.probe.capture_start()
        def confirm(stage, message, pump, **kwargs):
            original = self.probe.target.copy()
            self.probe.target += [.2, 0., 0.]
            pump()
            self.probe.target = original
        with patch.object(self.probe.control, 'wait_confirmation', side_effect=confirm):
            with self.assertRaisesRegex(RuntimeError, 'moved while awaiting confirmation'):
                self.probe.confirm_position('front_left', 'Confirm')
        self.assertFalse(self.probe.moves)

    def test_failed_fine_contact_prevents_mesh_and_return(self):
        touch = self.probe.homing_touch
        def failed_fine(**kwargs):
            if self.probe.touches:
                raise RuntimeError('Fine contact measurement rejected')
            return touch(**kwargs)
        self.probe.homing_touch = failed_fine
        with self.assertRaisesRegex(RuntimeError, 'Fine contact measurement rejected'):
            self.probe.run_routine()
        self.assertFalse(self.probe.records); self.assertFalse(self.probe.returned)
        self.assertFalse(any(m['name'] == 'front-left travel' for m in self.probe.moves))

    def test_failed_return_keeps_complete_measurements_pending_and_old_map_active(self):
        self.probe.fail_return = True
        with self.assertRaisesRegex(RuntimeError, 'Return motion failed'): self.probe.run_routine()
        self.assertEqual(len(self.probe.records), 9)
        self.assertFalse(self.probe.returned)
        self.assertEqual(self.output.read_text(), 'old mesh\n')
        data = yaml.safe_load(self.output.with_suffix('.pending.yaml').read_text())
        self.assertFalse(data['complete']); self.assertFalse(data['homing']['returned_to_start'])

    def test_failed_mesh_point_returns_home_but_keeps_previous_map_active(self):
        probe_current=self.probe.probe_current
        def fail_point():
            if self.probe.index==1:raise RuntimeError('Contact repeatability 0.0209 exceeds 0.0200')
            probe_current()
        self.probe.probe_current=fail_point
        with self.assertRaisesRegex(RuntimeError,'returned to established Home'):
            self.probe.run_routine()
        self.assertEqual(len(self.probe.records),1)
        self.assertFalse(self.probe.returned)
        self.assertEqual(self.probe.failure_recovery['state'],'returned')
        np.testing.assert_allclose(self.probe.target,self.probe.start_xyz)
        self.assertEqual(self.output.read_text(),'old mesh\n')
        self.assertFalse(yaml.safe_load(self.output.with_suffix('.pending.yaml').read_text())['complete'])

    def test_saved_mesh_records_selected_temperature_and_measured_heaters(self):
        thermal={'mode':'heated','options':{'nozzle_target_c':210.,'bed_target_c':60.,'soak_s':60.},
            'temperatures':{'extruder':{'temperature_c':210.2,'target_c':210.}},'locked':True}
        self.probe.thermal_policy=SimpleNamespace(status=lambda:thermal,require_ready=lambda:None)
        self.probe.run_routine()
        data=yaml.safe_load(self.output.read_text())
        self.assertEqual(data['probing_temperature'],thermal)
        self.assertTrue(data['homing']['returned_to_start'])

    def test_capture_rejects_tilt_and_preserves_small_measured_orientation_without_moving(self):
        self.probe.capture_start()
        self.probe.config = replace(self.probe.config, dry_run=False)
        nominal = SE3.RPY((180., 0., 90.), unit='deg', order='zyx').R
        def configure(tilt):
            pose = np.eye(4)
            pose[:3, :3] = SE3.RPY((180.+tilt, 0., 90.), unit='deg', order='zyx').R
            pose[:3, 3] = self.probe.target/1000.
            planner = SimpleNamespace(config=self.probe.config, rotation=nominal, offset_m=np.zeros(3),
                                      kinematics=SimpleNamespace(eval=lambda q: pose), check_joints=lambda q: None)
            planner.check_pose = lambda q, start, end, **kwargs: MotionPlanner.check_pose(planner, q, start, end, **kwargs)
            self.probe.planner = planner
            return pose[:3, :3]
        configure(1.)
        with self.assertRaisesRegex(ValueError, 'Orientation error'): BridgeBedProbe.capture_start(self.probe)
        self.assertFalse(self.probe.moves)
        actual = configure(.1)
        BridgeBedProbe.capture_start(self.probe)
        np.testing.assert_allclose(self.probe.planner.rotation, actual)
        np.testing.assert_allclose(SE3.RPY(self.probe.config.nozzle_rpy_deg, unit='deg', order='zyx').R, actual)
        self.assertFalse(self.probe.moves)

    def test_surface_confirmation_freezes_current_orientation_before_fast_descent(self):
        self.check_confirmation_orientation('surface')

    def test_scale_confirmation_freezes_current_orientation_before_contact(self):
        self.check_confirmation_orientation('scale')

    def check_confirmation_orientation(self, stage):
        # Measured UR pose that exposed the startup mismatch: even .003 degrees
        # between the initial sample and confirmation is too much to correct in
        # the first 10 ms of a continuous trajectory.
        confirmed = np.array([.1686267852783203, -1.1174641710570832,
                              1.361110035573141, -1.8143464527525843,
                              -1.5708163420306605, .17175784707069397])
        self.probe.config = replace(self.probe.config, dry_run=False,
                                    path_tolerance_mm=.0025)
        self.probe.settings = replace(self.probe.settings, lookahead_enabled=False)
        self.probe.planner = planner = GCodePlanner(self.probe.settings, self.probe.config)
        actual = planner.kinematics.eval(confirmed)[:3, :3]
        rpy = SE3.Rt(actual, np.zeros(3)).rpy(unit='deg', order='zyx')
        old_rotation = SE3.RPY(rpy + [0., .003, 0.], unit='deg', order='zyx').R
        nozzle = planner.nozzle_position(confirmed)
        result = planner.kinematics.ikine_LM(planner.tool_pose(nozzle, old_rotation),
                                            q0=confirmed, joint_limits=True, tol=1e-14, slimit=1)
        self.assertTrue(result.success)
        self.probe.seed = result.q.copy()
        self.probe.target = planner.nozzle_position(result.q)
        BridgeBedProbe.capture_start(self.probe)
        with self.assertRaisesRegex(ValueError, 'joint acceleration too high'):
            plan_probe_descent(planner, nozzle, nozzle-[0., 0., 20.], confirmed, 10., 50.)

        def confirmed_pose(*args, **kwargs):
            self.probe.target, self.probe.seed = nozzle.copy(), confirmed.copy()
        with patch.object(self.probe.control, 'wait_confirmation', side_effect=confirmed_pose):
            self.probe.confirm_position(stage, 'Confirm initial contact surface')
        np.testing.assert_allclose(planner.rotation, actual, atol=1e-14)
        np.testing.assert_allclose(planner.rotation_for((0., 0., 0.)), actual, atol=1e-14)
        motion = plan_probe_descent(planner, self.probe.target,
                                    self.probe.target-[0., 0., 20.], self.probe.seed, 10., 50.)
        np.testing.assert_allclose(motion.positions[0], confirmed, atol=1e-14)
        np.testing.assert_allclose(planner.nozzle_position(motion.positions[-1]),
                                   nozzle-[0., 0., 20.], atol=.0025)
        self.assertFalse(self.probe.moves)

    def test_surface_confirmation_rechecks_nominal_upright_before_reanchoring(self):
        self.probe.capture_start()
        nominal = self.probe._nominal_probe_rotation.copy()
        # Each sample can be within .2 degrees of the previous one while the
        # final sample is outside the nominal upright limit.
        earlier = SE3.RPY((180.15, 0., 90.), unit='deg', order='zyx').R
        tilted = SE3.RPY((180.3, 0., 90.), unit='deg', order='zyx').R
        pose = SE3.Rt(tilted, self.probe.target/1000.).A
        planner = SimpleNamespace(config=self.probe.config, rotation=earlier,
                                  offset_m=np.zeros(3), kinematics=SimpleNamespace(eval=lambda q: pose))
        planner.check_pose = lambda *args, **kwargs: MotionPlanner.check_pose(planner, *args, **kwargs)
        self.probe.planner = planner
        with self.assertRaisesRegex(ValueError, 'Orientation error'):
            self.probe.confirm_position('surface', 'Confirm surface')
        np.testing.assert_array_equal(self.probe._nominal_probe_rotation, nominal)
        np.testing.assert_array_equal(planner.rotation, earlier)
        self.assertFalse(self.probe.moves)

    def test_front_left_confirmation_keeps_orientation_frozen(self):
        self.probe.capture_start()
        rotation = self.probe.planner.rotation.copy()
        with patch.object(self.probe, 'freeze_probe_orientation',
                          side_effect=AssertionError('must not re-anchor after homing')):
            self.probe.confirm_position('front_left', 'Confirm front-left')
        np.testing.assert_array_equal(self.probe.planner.rotation, rotation)

    def test_bounded_homing_configuration_rejects_unlimited_or_reversed_motion(self):
        for change in ({'max_descent_mm': 1000.}, {'max_descent_mm': 350.01},
                       {'max_descent_mm': 4.99}, {'max_descent_mm': True},
                       {'max_descent_mm': 5., 'retract_mm': 5.}, {'first_speed_mm_s': 10.01},
                       {'second_speed_mm_s': 10.}, {'first_speed_mm_s': .1},
                       {'first_speed_mm_s': 0.}, {'second_speed_mm_s': float('inf')},
                       {'acceleration_mm_s2': 101.}, {'sample_age_s': .3},
                       {'cancel_timeout_s': 3.}, {'max_stop_travel_mm': 2.},
                       {'second_step_mm': .1}, {'retract_mm': .1},
                       {'lift_speed_mm_s': 21.}, {'lift_speed_mm_s': 0.},
                       {'travel_speed_mm_s': float('nan')}, {'travel_speed_mm_s': 0.},
                       {'mesh_columns': 1}, {'mesh_rows': 10}, {'mesh_rows': 2.5}, {'mesh_columns': True},
                       {'clearance_above_start_mm': float('nan')}, {'heartbeat_timeout_s': 60.}):
            with self.subTest(change=change), self.assertRaises(ValueError): HomingOptions(**change)

    def test_native_profile_applies_homing_speeds_without_speeding_mesh_or_scale(self):
        config = Path(__file__).resolve().parents[2]/'axisforge_bringup'/'config'
        # Exercise independent values without binding a regression test to the
        # operator's mutable dashboard speed setting.
        profile = yaml.safe_load((config/'bridge_bed_probe.yaml').read_text())
        profile['contact']['speed_mm_s'] = .07
        profile['contact']['repeatability_mm'] = .02
        profile['homing']['second_speed_mm_s'] = .13
        profile['homing']['lift_speed_mm_s'] = 8.
        fixture = Path(self.temp.name)/'profile.yaml'
        fixture.write_text(yaml.safe_dump(profile))
        contact, homing = load_profile(fixture)
        # The saved initial search speed is editable independently of its default.
        self.assertEqual(HomingOptions().first_speed_mm_s, 5.)
        self.assertEqual(HomingOptions(first_speed_mm_s=10.).first_speed_mm_s, 10.)
        # The saved search distance is editable for the current nozzle-to-bed gap.
        self.assertEqual(HomingOptions().max_descent_mm, 350.)
        self.assertEqual(HomingOptions(max_descent_mm=5.).max_descent_mm, 5.)
        self.assertEqual(HomingOptions(max_descent_mm=20.).max_descent_mm, 20.)
        self.assertEqual(homing.second_speed_mm_s, .13)
        self.assertEqual(homing.lift_speed_mm_s, 8.)
        self.assertEqual(HomingOptions().lift_speed_mm_s, 10.)
        self.assertIsNone(homing.travel_speed_mm_s)
        self.assertEqual((homing.mesh_columns, homing.mesh_rows), (3, 3))
        self.assertEqual((homing.first_step_mm, homing.second_step_mm), (.05, .01))
        self.assertEqual(contact.speed_mm_s, .07)
        self.assertIn(contact.samples, range(1, 6))  # Operator-editable saved setting.
        self.assertEqual(contact.repeatability_mm, .02)
        self.assertEqual(contact.max_delta_counts, 120000.)
        scale, _ = load_profile(config/'alps_load_calibration.yaml')
        self.assertEqual(scale.speed_mm_s, .1)
        self.assertEqual(scale.max_delta_counts, 120000.)
        # Existing customized slower profiles remain valid.
        self.assertEqual(HomingOptions(first_speed_mm_s=.5, second_speed_mm_s=.1).first_speed_mm_s, .5)

    def test_only_precise_contacts_determine_homed_height(self):
        self.probe.contacts_z = [98.8, 99.81]
        self.probe.run_routine()
        self.assertAlmostEqual(self.probe.homed_z, 99.83)
        self.assertIsNone(self.probe.homing_repeatability_mm)

    def test_mesh_uses_selected_slow_continuous_contact(self):
        self.probe.homing_touch = lambda **kwargs: kwargs
        result = self.probe.mesh_touch()
        self.assertEqual(result['speed_mm_s'], self.probe.contact_options.speed_mm_s)
        self.assertEqual(result['floor_z'], 97.)

    def test_mesh_clearance_is_separate_and_clears_measured_high_spots(self):
        original_retract = self.probe.homing.retract_mm
        original_print_clearance = self.probe.config.clearance_mm
        xy = self.probe.points[0]
        bed = self.probe.bed_point(xy)[2]
        self.assertAlmostEqual(self.probe.hover(xy)[2], bed+3.)
        self.probe.records = {0: {'bed_z_mm': bed+.8}}
        self.assertAlmostEqual(self.probe.hover(xy)[2], bed+3.8)
        self.probe._point_readings = [{'nozzle': np.array([0.,0.,bed+1.2])}]
        self.assertAlmostEqual(self.probe.hover(xy)[2], bed+4.2)
        self.assertEqual(self.probe.homing.retract_mm, original_retract)
        self.assertEqual(self.probe.config.clearance_mm, original_print_clearance)

    def test_homing_start_defines_mesh_centre_despite_old_saved_origin(self):
        self.probe.run_routine()
        first=next(m for m in self.probe.moves if m['name']=='front-left travel')
        expected=self.probe.start_xyz[:2]-np.asarray(self.probe.settings.bed_size_mm)/2+self.probe.args.margin_mm
        np.testing.assert_allclose(first['to'][:2],expected)
        data=yaml.safe_load(self.output.read_text())
        np.testing.assert_allclose(data['bed_frame']['center_base_mm'],self.probe.start_xyz[:2])
        self.assertEqual(data['bed_frame']['source'],'homing_center')

    def test_repeat_mesh_contact_shortens_only_the_already_known_search_range(self):
        self.probe.homing_touch=lambda **kwargs:kwargs
        first=self.probe.mesh_touch()
        self.probe._point_reference_z=100.2
        repeated=self.probe.mesh_touch()
        self.assertEqual(first['floor_z'],97.)
        self.assertEqual(repeated['floor_z'],99.7)
        self.assertEqual(first['speed_mm_s'],repeated['speed_mm_s'])
        self.probe._point_reference_z=96.
        self.assertEqual(self.probe.mesh_touch()['floor_z'],97.)

    def test_unresolved_trajectory_stop_uses_dashboard_stop_and_does_not_resume(self):
        self.probe.config = replace(self.probe.config, dry_run=False)
        self.probe.goal_future = SimpleNamespace(done=lambda: False)
        self.probe.goal_handle = self.probe.result_future = None
        calls = []
        def stop_program(request):
            calls.append(request)
            return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True))
        self.probe.stop_client = SimpleNamespace(service_is_ready=lambda: True, call_async=stop_program)
        with patch.object(LoadCellBedProbe, 'stop', return_value=None):
            with self.assertRaisesRegex(RuntimeError, 'UR program stopped'): self.probe.stop()
        self.assertEqual(len(calls), 1)
        self.assertFalse(self.probe.stop_unconfirmed)
        self.assertFalse(self.probe.moves)

    def test_unconfirmed_action_and_dashboard_stop_latches_hardware_stop_failure(self):
        self.probe.config = replace(self.probe.config, dry_run=False)
        self.probe.goal_future = SimpleNamespace(done=lambda: False)
        self.probe.goal_handle = self.probe.result_future = None
        self.probe.stop_client = SimpleNamespace(service_is_ready=lambda: False)
        with patch.object(LoadCellBedProbe, 'stop', return_value=None):
            with self.assertRaisesRegex(RuntimeError, 'use pendant Stop'): self.probe.stop()
        self.assertTrue(self.probe.stop_unconfirmed)
        self.assertFalse(self.probe.moves)


class GuidedMapCompatibilityTests(unittest.TestCase):
    def test_only_declared_bounded_near_upright_maps_are_reusable_at_other_orientation(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'bed.yaml'
            data = {'tcp_offset_mm': [-100., 0., 117.06], 'nozzle_rpy_deg': [179.9, 0., 90.],
                    'upright_tolerance_deg': .2,
                    'points': [{'nozzle_base_mm': [x, y, 100.], 'bed_z_mm': 100.}
                               for x, y in ((0., 0.), (20., 0.), (0., 20.), (20., 20.))]}
            def load():
                path.write_text(yaml.safe_dump(data))
                return BedSurface.load(path, (-100., 0., 117.06), (170., 10., 90.), 1.)
            self.assertIn('upright probe', load().orientation_note)
            del data['upright_tolerance_deg']
            with self.assertRaisesRegex(ValueError, 'orientation changes'): load()
            data['upright_tolerance_deg'] = .2; data['nozzle_rpy_deg'][0] = 179.
            with self.assertRaisesRegex(ValueError, 'orientation changes'): load()
            data['upright_tolerance_deg'] = 2.
            with self.assertRaisesRegex(ValueError, 'upright tolerance'): load()


if __name__ == '__main__': unittest.main()
