"""Offline editing and operation interlocks for build-surface geometry."""
from pathlib import Path
import unittest
from unittest.mock import AsyncMock, patch

from bridge_dashboard.bed_settings import BedSettings
import test_app
from test_bed_settings import CONFIG


class BedApiTests(unittest.TestCase):
    def setUp(self):
        test_app.DashboardApiTests.setUp(self)
        root = Path(self.temp_dir.name)
        self.config = root/'bed-config.yaml'; self.config.write_text(CONFIG)
        self.client.app.state.bed_settings = BedSettings(self.config,lock_path=root/'lock')
        current = self.client.get('/api/setup/bed').json()
        self.body = dict(width_mm=150.,depth_mm=100.,probe_margin_mm=10.,revision=current['revision'])
    tearDown = test_app.DashboardApiTests.tearDown

    def test_offline_edit_updates_coverage_clears_preflight_and_requires_revision(self):
        self.moonraker.state = 'offline'
        self.client.app.state.workflow.record = {'old':'preflight'}
        result = self.client.post('/api/setup/bed',json=self.body)
        self.assertEqual(result.status_code,200,result.text)
        self.assertEqual(result.json()['probe_area'],{'x':[10.,140.],'y':[10.,90.]})
        self.assertIsNone(self.client.app.state.workflow.record)
        self.assertEqual(self.client.post('/api/setup/bed',json=self.body).status_code,409)
        self.assertFalse(self.axisforge.started)

    def test_robot_and_job_operations_prevent_geometry_edits(self):
        with patch('bridge_dashboard.app.Workflow.busy',return_value=True):
            self.assertEqual(self.client.post('/api/setup/bed',json=self.body).status_code,409)
        self.bed_probe.active = True
        self.assertEqual(self.client.post('/api/setup/bed',json=self.body).status_code,409)
        self.bed_probe.active = False
        with patch.object(self.axisforge,'status',return_value={'planning_check_running':True}):
            self.assertEqual(self.client.post('/api/setup/bed',json=self.body).status_code,409)
        with patch.object(self.client.app.state.robot_connection,'status',new_callable=AsyncMock,return_value={'busy':True}):
            self.assertEqual(self.client.post('/api/setup/bed',json=self.body).status_code,409)
        self.assertEqual(self.config.read_text(),CONFIG)

    def test_invalid_sizes_margins_or_unrequested_offsets_are_rejected(self):
        for change in ({'width_mm':True},{'width_mm':'100'},{'depth_mm':0.},
                       {'probe_margin_mm':50.},{'probe_margin_mm':-1.},{'x_offset':0.}):
            with self.subTest(change=change):
                self.assertEqual(self.client.post('/api/setup/bed',json={**self.body,**change}).status_code,422)
        self.assertEqual(self.config.read_text(),CONFIG)
