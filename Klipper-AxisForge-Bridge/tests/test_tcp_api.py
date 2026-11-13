import unittest
from pathlib import Path
from unittest.mock import AsyncMock,patch
from bridge_dashboard.tcp_settings import TcpSettings
import test_app
from test_tcp_settings import CONFIG


class TcpApiTests(unittest.TestCase):
    def setUp(self):
        test_app.DashboardApiTests.setUp(self)
        root=Path(self.temp_dir.name)
        self.path=root/'tcp.yaml';self.path.write_text(CONFIG)
        self.client.app.state.tcp_settings=TcpSettings(self.path,lock_path=root/'lock')
        self.body=dict(x_mm=-100.,y_mm=1.,z_mm=117.06,revision=self.client.get('/api/setup/tcp').json()['revision'])
    tearDown=test_app.DashboardApiTests.tearDown

    def test_offline_configuration_can_be_saved_and_saved_preflight_is_cleared(self):
        self.moonraker.state='offline'
        workflow=self.client.app.state.workflow;workflow.record={'old':'preflight'}
        response=self.client.post('/api/setup/tcp',json=self.body)
        self.assertEqual(response.status_code,200,response.text)
        self.assertIsNone(workflow.record)
        self.assertEqual(response.json()['tcp_offset_mm'],[-100.,1.,117.06])
        self.assertEqual(self.client.post('/api/setup/tcp',json=self.body).status_code,409)

    def test_jobs_probing_calibration_and_driver_start_block_tcp_save(self):
        with patch('bridge_dashboard.app.Workflow.busy',return_value=True):
            self.assertEqual(self.client.post('/api/setup/tcp',json=self.body).status_code,409)
        self.bed_probe.active=True
        self.assertEqual(self.client.post('/api/setup/tcp',json=self.body).status_code,409)
        self.bed_probe.active=False
        self.client.app.state.load_calibration._active=True
        self.assertEqual(self.client.post('/api/setup/tcp',json=self.body).status_code,409)
        self.client.app.state.load_calibration._active=False
        with patch.object(self.axisforge,'status',return_value={'planning_check_running':True}):
            self.assertEqual(self.client.post('/api/setup/tcp',json=self.body).status_code,409)
        with patch.object(self.client.app.state.robot_connection,'status',new_callable=AsyncMock,return_value={'busy':True}):
            self.assertEqual(self.client.post('/api/setup/tcp',json=self.body).status_code,409)
        self.assertEqual(self.path.read_text(),CONFIG)

    def test_bad_values_and_unrequested_controller_settings_are_rejected(self):
        for changes in ({'x_mm':'1.0'},{'x_mm':True},{'z_mm':1001.},{'rx':0.},{'revision':'old'}):
            with self.subTest(changes=changes):
                self.assertEqual(self.client.post('/api/setup/tcp',json={**self.body,**changes}).status_code,422)
        self.assertEqual(self.path.read_text(),CONFIG)

    def test_configuration_unavailable_does_not_return_success(self):
        self.path.unlink()
        self.assertEqual(self.client.get('/api/setup/tcp').status_code,503)
        self.assertEqual(self.client.post('/api/setup/tcp',json=self.body).status_code,400)
