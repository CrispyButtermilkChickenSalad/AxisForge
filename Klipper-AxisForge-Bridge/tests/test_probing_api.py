"""Offline probing configuration and API-level movement validation."""
from pathlib import Path
import unittest
from unittest.mock import AsyncMock, patch

from bridge_dashboard.probing_settings import ProbingSettings
import test_app
from test_probing_settings import PROFILE


class ProbingApiTests(unittest.TestCase):
    def setUp(self):
        test_app.DashboardApiTests.setUp(self)
        root = Path(self.temp_dir.name)
        self.profile = root/'bridge_bed_probe.yaml'
        self.profile.write_text(PROFILE)
        self.client.app.state.probing_settings = ProbingSettings(root/'ur_master.yaml', lock_path=root/'lock')
        snapshot = self.client.get('/api/setup/probing').json()
        self.body = {'revision': snapshot['revision'], 'settings': {**snapshot['settings'], 'first_speed_mm_s': 8.}}

    tearDown = test_app.DashboardApiTests.tearDown

    def test_offline_save_persists_profile_and_invalidates_preflight(self):
        self.moonraker.state = 'offline'
        workflow = self.client.app.state.workflow
        workflow.record = {'old': 'preflight'}
        response = self.client.post('/api/setup/probing', json=self.body)
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()['settings']['first_speed_mm_s'], 8.)
        self.assertIsNone(workflow.record)
        self.assertEqual(self.client.post('/api/setup/probing', json=self.body).status_code, 409)
        self.assertFalse(self.axisforge.started)

    def test_every_robot_or_job_owner_blocks_configuration_writes(self):
        with patch('bridge_dashboard.app.Workflow.busy', return_value=True):
            self.assertEqual(self.client.post('/api/setup/probing', json=self.body).status_code, 409)
        self.bed_probe.active = True
        self.assertEqual(self.client.post('/api/setup/probing', json=self.body).status_code, 409)
        self.bed_probe.active = False
        self.client.app.state.load_calibration._active = True
        self.assertEqual(self.client.post('/api/setup/probing', json=self.body).status_code, 409)
        self.client.app.state.load_calibration._active = False
        for status in ({'planning_check_running': True}, {'running': True}):
            with patch.object(self.axisforge, 'status', return_value=status):
                self.assertEqual(self.client.post('/api/setup/probing', json=self.body).status_code, 409)
        with patch.object(self.client.app.state.robot_connection, 'status', new_callable=AsyncMock, return_value={'busy': True}):
            self.assertEqual(self.client.post('/api/setup/probing', json=self.body).status_code, 409)
        self.assertEqual(self.profile.read_text(), PROFILE)

    def test_initial_search_below_five_saves_without_starting_motion(self):
        for speed in (4., 1., .5):
            before = self.client.get('/api/setup/probing').json()
            body = {'revision': before['revision'],
                    'settings': {**before['settings'], 'first_speed_mm_s': speed}}
            response = self.client.post('/api/setup/probing', json=body)
            self.assertEqual(response.status_code, 200, response.text)
            saved = self.client.get('/api/setup/probing').json()
            self.assertEqual(saved['settings']['first_speed_mm_s'], speed)
            self.assertEqual(saved['bounds']['first_speed_mm_s'], {'min': .5, 'max': 10.})
            self.assertEqual(saved['settings']['second_speed_mm_s'], .2)
        self.assertFalse(self.axisforge.started)
        self.assertFalse(self.bed_probe.started)

    def test_single_contact_count_saves_through_api_without_starting_hardware(self):
        self.body['settings']['mesh_samples'] = 1
        response = self.client.post('/api/setup/probing', json=self.body)
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()['settings']['mesh_samples'], 1)
        self.assertEqual(response.json()['bounds']['mesh_samples']['min'], 1)
        self.assertFalse(self.axisforge.started)
        self.assertFalse(self.bed_probe.started)

    def test_strict_shape_and_speed_bounds_reject_invalid_requests(self):
        for change in ({'first_speed_mm_s': '8'}, {'first_speed_mm_s': True},
                       {'mesh_samples': 3.0}, {'mesh_samples': 6}, {'mesh_speed_mm_s': .5001},
                       {'second_speed_mm_s': 5.}, {'mesh_repeatability_mm': .001},
                       {'lift_speed_mm_s': 21.}, {'lift_speed_mm_s': False},
                       {'max_descent_mm': 4.99}, {'max_descent_mm': 350.01},
                       {'max_descent_mm': True}, {'max_descent_mm': '350'},
                       {'mesh_columns': 1}, {'mesh_rows': 10}, {'mesh_rows': 3.0},
                       {'mesh_columns': True}, {'mesh_columns': '9'},
                       {'max_delta_counts': 100000.},
                       {'first_speed_mm_s': .5, 'second_speed_mm_s': .5}):
            body = {**self.body, 'settings': {**self.body['settings'], **change}}
            with self.subTest(change=change):
                self.assertEqual(self.client.post('/api/setup/probing', json=body).status_code, 422)
        for body in ({**self.body, 'revision': 'old'}, {**self.body, 'rx': 0},
                     {'revision': self.body['revision'], **self.body['settings']}):
            self.assertEqual(self.client.post('/api/setup/probing', json=body).status_code, 422)
        self.assertEqual(self.profile.read_text(), PROFILE)

    def test_search_and_spread_save_with_existing_half_mm_mesh_speed(self):
        # GET already permits 0.5 mm/s. Saving unrelated fields must not reject
        # that saved speed using a different, obsolete API limit.
        self.profile.write_text(PROFILE.replace('speed_mm_s: 0.1 # mesh speed',
                                               'speed_mm_s: 0.5 # mesh speed'))
        before = self.client.get('/api/setup/probing').json()
        settings = {**before['settings'], 'max_descent_mm': 350., 'mesh_repeatability_mm': .03}
        response = self.client.post('/api/setup/probing', json={
            'revision': before['revision'], 'settings': settings})
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(self.client.get('/api/setup/probing').json()['settings'], settings)
        self.assertEqual(response.json()['settings']['mesh_speed_mm_s'], .5)
        self.assertIn('max_descent_mm: 350.0', self.profile.read_text())
        self.assertIn('repeatability_mm: 0.03', self.profile.read_text())
        self.assertIn('max_delta_counts: 60000.0 # independent force signal guard', self.profile.read_text())
        self.assertFalse(self.axisforge.started)
        self.assertFalse(self.bed_probe.started)

    def test_search_range_saves_offline_without_launching_motion(self):
        self.moonraker.state = 'offline'
        self.body['settings']['max_descent_mm'] = 350.
        response = self.client.post('/api/setup/probing', json=self.body)
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(response.json()['settings']['max_descent_mm'], 350.)
        self.assertEqual(response.json()['bounds']['max_descent_mm'], {'min': 5., 'max': 350.})
        self.assertIn('max_descent_mm: 350.0', self.profile.read_text())
        self.assertFalse(self.axisforge.started)
        self.assertFalse(self.bed_probe.started)

    def test_missing_profile_cannot_report_saved(self):
        self.profile.unlink()
        self.assertEqual(self.client.get('/api/setup/probing').status_code, 503)
        self.assertEqual(self.client.post('/api/setup/probing', json=self.body).status_code, 400)

    def test_requested_step_sizes_are_forwarded_but_still_require_runner_authorization(self):
        routine = self.client.app.state.load_calibration
        with patch.object(routine, 'action', new_callable=AsyncMock, return_value={'state': 'stepping'}) as action:
            for size in (.001, .01, .05, .1, .5, 1., 10.):
                for direction in (-1, 1):
                    body = {'command': 'step', 'token': 'a'*24, 'delta_mm': direction*size}
                    response = self.client.post('/api/load-calibration/action', json=body)
                    self.assertEqual(response.status_code, 200, response.text)
                    action.assert_awaited_with('step', 'a'*24, direction*size, None)
            for size in (0., .2, 2., 11., True, '1'):
                body = {'command': 'step', 'token': 'a'*24, 'delta_mm': size}
                self.assertEqual(self.client.post('/api/load-calibration/action', json=body).status_code, 422)
            self.assertEqual(action.await_count, 14)
        response = self.client.post('/api/load-calibration/action', json={
            'command': 'step', 'token': 'a'*24, 'delta_mm': 10.})
        self.assertEqual(response.status_code, 409)


if __name__ == '__main__':
    unittest.main()
