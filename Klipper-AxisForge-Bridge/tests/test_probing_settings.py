"""Probing edits retain force guards, obey robot ownership, and avoid lost updates."""
import fcntl
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import yaml

from bridge_dashboard.probing_settings import ProbingSettings, ProbingSettingsError, ProbingSettingsConflict


PROFILE = '''# retain this commissioning note
contact:
  trigger_mode: raw
  contact_delta_counts: 20000.0
  max_delta_counts: 60000.0 # independent force signal guard
  step_mm: 0.01
  speed_mm_s: 0.1 # mesh speed
  samples: 5
  repeatability_mm: 0.02
  contact_offset_mm: -0.01
  settle_s: 0.15
homing:
  first_speed_mm_s: 10.0
  second_speed_mm_s: 0.2
  lift_speed_mm_s: 20.0
  mesh_columns: 3
  mesh_rows: 3
  acceleration_mm_s2: 50.0
  sample_age_s: 0.15
  cancel_timeout_s: 0.2
  max_stop_travel_mm: 0.5
  max_descent_mm: 20.0
'''


class ProbingSettingsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.profile = self.root/'bridge_bed_probe.yaml'
        self.profile.write_text(PROFILE)
        self.store = ProbingSettings(self.root/'ur_master.yaml', lock_path=self.root/'robot.lock')
        self.invalidate = Mock()

    def save(self, values=None, revision=None):
        current = self.store.status()
        return self.store.save(values or {**current['settings'], 'first_speed_mm_s': 8.},
                               revision or current['revision'], self.invalidate)

    def test_atomic_update_changes_only_requested_scalars_and_preserves_comments_backup_mode(self):
        self.profile.chmod(0o640)
        before = self.store.status()
        values = dict(first_speed_mm_s=8., second_speed_mm_s=.1, mesh_speed_mm_s=.05,
                      mesh_samples=3, mesh_repeatability_mm=.01, lift_speed_mm_s=10.,
                      mesh_columns=2, mesh_rows=9, max_descent_mm=350., mesh_yaw_deg=0.)
        result = self.save(values)
        expected = PROFILE.replace('first_speed_mm_s: 10.0', 'first_speed_mm_s: 8.0')
        expected = expected.replace('second_speed_mm_s: 0.2', 'second_speed_mm_s: 0.1')
        expected = expected.replace('speed_mm_s: 0.1 # mesh speed', 'speed_mm_s: 0.05 # mesh speed')
        expected = expected.replace('samples: 5', 'samples: 3').replace('repeatability_mm: 0.02', 'repeatability_mm: 0.01')
        expected = expected.replace('lift_speed_mm_s: 20.0', 'lift_speed_mm_s: 10.0')
        expected = expected.replace('mesh_columns: 3', 'mesh_columns: 2').replace('mesh_rows: 3', 'mesh_rows: 9')
        expected = expected.replace('max_descent_mm: 20.0', 'max_descent_mm: 350.0')
        self.assertEqual(self.profile.read_text(), expected)
        self.assertEqual(result['settings'], values)
        self.assertTrue(result['changed'])
        self.assertNotEqual(before['revision'], result['revision'])
        self.assertEqual(self.profile.stat().st_mode & 0o777, 0o640)
        self.assertEqual(self.profile.with_name(self.profile.name+'.probing-backup').read_text(), PROFILE)
        self.invalidate.assert_called_once()
        self.assertFalse(list(self.root.glob('.bridge_bed_probe.yaml.*')))

    def test_noop_preserves_profile_revision_and_saved_preflight(self):
        before = self.store.status()
        result = self.save(before['settings'])
        self.assertFalse(result['changed'])
        self.assertEqual(result['revision'], before['revision'])
        self.assertEqual(self.profile.read_text(), PROFILE)
        self.invalidate.assert_not_called()

    def test_half_mm_contact_speed_round_trips_without_changing_load_guards(self):
        before = self.store.status()
        values = {**before['settings'], 'first_speed_mm_s': 5.,
                  'second_speed_mm_s': .5, 'mesh_speed_mm_s': .5}
        self.assertEqual(self.save(values)['settings'], values)
        data = yaml.safe_load(self.profile.read_text())
        self.assertEqual(data['contact']['max_delta_counts'], 60000.)
        self.assertEqual(data['homing']['cancel_timeout_s'], .2)
        with self.assertRaises(ProbingSettingsError):
            self.save({**values, 'mesh_speed_mm_s': .5001})

    def test_legacy_profile_defaults_direction_and_can_save_a_rotated_bed(self):
        before = self.store.status()
        self.assertEqual(before['settings']['mesh_yaw_deg'], 0.)
        values = {**before['settings'], 'mesh_yaw_deg': -90.}
        result = self.save(values)
        self.assertEqual(result['settings']['mesh_yaw_deg'], -90.)
        expected = yaml.safe_load(PROFILE)
        expected['homing']['mesh_yaw_deg'] = -90.
        self.assertEqual(yaml.safe_load(self.profile.read_text()), expected)
        self.assertIn('# independent force signal guard', self.profile.read_text())
        self.save({**values, 'mesh_yaw_deg': 0.})
        self.assertEqual(self.store.status()['settings']['mesh_yaw_deg'], 0.)

    def test_numeric_settings_edit_preserves_native_trigger_selection(self):
        self.profile.write_text(PROFILE.replace('trigger_mode: raw',
            'trigger_mode: raw\n  trigger_source: klipper_probe'))
        self.save()
        self.assertIn('trigger_source: klipper_probe', self.profile.read_text())
        self.assertIn('max_delta_counts: 60000.0', self.profile.read_text())

    def test_unknown_trigger_source_is_rejected(self):
        self.profile.write_text(PROFILE.replace('trigger_mode: raw',
            'trigger_mode: raw\n  trigger_source: query_probe'))
        with self.assertRaisesRegex(ProbingSettingsError, 'trigger source'):
            self.store.status()

    def test_whole_profile_revision_detects_external_force_guard_changes(self):
        before = self.store.status()
        self.profile.write_text(PROFILE.replace('60000.0', '50000.0'))
        with self.assertRaisesRegex(ProbingSettingsConflict, 'changed since'):
            self.save(revision=before['revision'])
        self.assertIn('50000.0', self.profile.read_text())
        self.invalidate.assert_not_called()

    def test_external_robot_owner_blocks_save(self):
        with self.store.lock_path.open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(ProbingSettingsConflict, 'owned'):
                self.save()
        self.assertEqual(self.profile.read_text(), PROFILE)
        self.invalidate.assert_not_called()

    def test_one_and_two_contacts_roundtrip_without_changing_spread_limit(self):
        for count in (1, 2):
            before = self.store.status()
            values = {**before['settings'], 'mesh_samples': count}
            result = self.save(values, before['revision'])
            self.assertEqual(result['settings']['mesh_samples'], count)
            self.assertEqual(result['settings']['mesh_repeatability_mm'], .02)
            self.assertEqual(result['bounds']['mesh_samples']['min'], 1)

    def test_nonfinite_boolean_out_of_range_and_reversed_speeds_are_rejected(self):
        values = self.store.status()['settings']
        for change in ({'first_speed_mm_s': True}, {'first_speed_mm_s': float('nan')},
                       {'first_speed_mm_s': 10.01}, {'second_speed_mm_s': 1.01},
                       {'mesh_speed_mm_s': float('inf')}, {'mesh_speed_mm_s': .01},
                       {'mesh_samples': 3.0}, {'mesh_samples': True}, {'mesh_samples': 6}, {'mesh_samples': 0},
                       {'mesh_repeatability_mm': .1},
                       {'lift_speed_mm_s': 20.01}, {'lift_speed_mm_s': 0},
                       {'max_descent_mm': 4.99}, {'max_descent_mm': 350.01},
                       {'max_descent_mm': True}, {'max_descent_mm': float('nan')},
                       {'mesh_columns': 1}, {'mesh_rows': 10}, {'mesh_columns': 2.5},
                       {'mesh_rows': True}, {'mesh_columns': '9'},
                       {'first_speed_mm_s': .5, 'second_speed_mm_s': .5}):
            with self.subTest(change=change), self.assertRaises(ProbingSettingsError):
                self.save({**values, **change})
        for bad in ({}, {**values, 'max_delta_counts': 100000.}):
            with self.assertRaises(ProbingSettingsError):
                self.store.save(bad, self.store.status()['revision'], self.invalidate)
        self.assertEqual(self.profile.read_text(), PROFILE)

    def test_search_range_bounds_and_retract_relationship_are_enforced_before_saving(self):
        status = self.store.status()
        self.assertEqual(status['bounds']['max_descent_mm'], {'min': 5., 'max': 350.})
        # A legacy explicit short range is preserved until the operator edits it.
        self.assertEqual(status['settings']['max_descent_mm'], 20.)
        self.save({**status['settings'], 'max_descent_mm': 5.})
        self.assertEqual(self.store.status()['settings']['max_descent_mm'], 5.)
        self.profile.write_text(PROFILE.replace('homing:\n', 'homing:\n  retract_mm: 5.0\n'))
        original = self.profile.read_bytes()
        self.invalidate.reset_mock()
        with self.assertRaisesRegex(ProbingSettingsError, 'exceed.*retract'):
            self.save({**self.store.status()['settings'], 'max_descent_mm': 5.})
        self.assertEqual(self.profile.read_bytes(), original)
        self.invalidate.assert_not_called()

    def test_malformed_profile_aliases_duplicates_and_invalid_force_guards_fail_closed(self):
        for profile in (PROFILE.replace('contact:', 'contact: &shared'),
                        PROFILE.replace('  samples: 5', '  samples: 5\n  samples: 3'),
                        PROFILE.replace('homing:', 'homing: {}\nhoming:'),
                        PROFILE.replace('max_delta_counts: 60000.0', 'max_delta_counts: 10000.0'),
                        PROFILE.replace('acceleration_mm_s2: 50.0', 'acceleration_mm_s2: .nan'),
                        PROFILE.replace('step_mm: 0.01', 'step_mm: [0.01]'),
                        'contact: null\nhoming: {}\n'):
            self.profile.write_text(profile)
            with self.subTest(profile=profile), self.assertRaises(ProbingSettingsError):
                self.store.status()
        self.invalidate.assert_not_called()

    def test_failed_replacement_preserves_original_but_invalidates_preflight(self):
        original = self.store._atomic_write
        def write(path, *args):
            if path == self.profile:
                raise OSError('read-only filesystem')
            return original(path, *args)
        with patch.object(self.store, '_atomic_write', side_effect=write):
            with self.assertRaisesRegex(ProbingSettingsError, 'Could not save'):
                self.save()
        self.assertEqual(self.profile.read_text(), PROFILE)
        self.invalidate.assert_called_once()

    def test_unconfigured_and_missing_profiles_are_explicit_errors(self):
        with self.assertRaises(ProbingSettingsError):
            ProbingSettings(None).status()
        self.profile.unlink()
        with self.assertRaises(ProbingSettingsError):
            self.store.status()


if __name__ == '__main__':
    unittest.main()
