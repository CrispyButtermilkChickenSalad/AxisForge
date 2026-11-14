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

    def test_strict_shape_and_speed_bounds_reject_invalid_requests(self):
        for change in ({'first_speed_mm_s': '8'}, {'first_speed_mm_s': True},
                       {'mesh_samples': 3.0}, {'mesh_samples': 6}, {'mesh_speed_mm_s': .3},
                       {'second_speed_mm_s': 5.}, {'mesh_repeatability_mm': .001},
                       {'lift_speed_mm_s': 21.}, {'lift_speed_mm_s': False},
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
                    action.assert_awaited_with('step', 'a'*24, direction*size)
            for size in (0., .2, 2., 11., True, '1'):
                body = {'command': 'step', 'token': 'a'*24, 'delta_mm': size}
                self.assertEqual(self.client.post('/api/load-calibration/action', json=body).status_code, 422)
            self.assertEqual(action.await_count, 14)
        response = self.client.post('/api/load-calibration/action', json={
            'command': 'step', 'token': 'a'*24, 'delta_mm': 10.})
        self.assertEqual(response.status_code, 409)


if __name__ == '__main__':
    unittest.main()
