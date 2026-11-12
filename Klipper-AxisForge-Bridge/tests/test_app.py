"""API tests for G-code staging and synchronized-job interlocks."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch, AsyncMock

from fastapi.testclient import TestClient

from bridge_dashboard.app import create_app
from bridge_dashboard.axisforge import AxisForgeError
from bridge_dashboard.config import Settings
from bridge_dashboard.moonraker import MoonrakerError


class FakeMoonraker:
    def __init__(self, state="ready"):
        self.state = state
        self.uploads = []
        self.fail_upload = False
        self.fail_telemetry = False
        self.stopped = False

    async def status(self):
        return {"connected": True, "state": self.state}

    async def telemetry(self):
        if self.fail_telemetry:
            raise MoonrakerError("telemetry unavailable")
        return {"print": {"state": "printing"}}

    async def upload_gcode(self, filename, local_path):
        if self.fail_upload:
            raise MoonrakerError("upload unavailable")
        self.uploads.append((filename, local_path.read_bytes()))

    async def emergency_stop(self):
        self.stopped = True

    async def close(self):
        pass


class FakeRosBridge:
    async def request_robot_program_stop(self): return {'stopped':True}
    async def start(self):
        pass

    def status(self):
        return {"connected": True, "joint_state_fresh": True}

    async def close(self):
        pass


class FakeAxisForge:
    def __init__(self, root):
        self.root = root
        self.started = []
        self.stop_requested = False

    def list_jobs(self):
        return {"configured": True, "jobs": ["part.gcode"], "error": None}

    def job_path(self,name):
        path=self.root/name
        if not path.exists(): raise AxisForgeError('Missing G-code')
        return path

    def stage_upload(self, name, source):
        if not name.endswith(".gcode"):
            raise AxisForgeError("Only .gcode files are accepted")
        destination = self.root / name
        if destination.exists():
            raise AxisForgeError("A G-code file with that name already exists")
        destination.write_bytes(source.read())
        if not destination.stat().st_size:
            destination.unlink()
            raise AxisForgeError("The uploaded G-code file is empty")
        return destination

    async def start(self, job, dry_run):
        self.started.append((job, dry_run))
        return {"running": True, "job": job, "dry_run": dry_run}

    async def request_stop(self, rosbridge):
        self.stop_requested = True
        return {"running": False, "message": "No AxisForge job is running"}

    def status(self):
        return {"running": False, "job": None, "dry_run": None, "return_code": None}


class DashboardApiTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.moonraker = FakeMoonraker()
        self.rosbridge = FakeRosBridge()
        self.axisforge = FakeAxisForge(Path(self.temp_dir.name))
        settings = Settings(
            moonraker_url="http://127.0.0.1:7125",
            moonraker_api_key=None,
            rosbridge_url="ws://127.0.0.1:9090",
            fluidd_url="http://printer.local",
            mainsail_url=None,
            printer_ui_url=None,
            alps_logger_url="https://crispybuttermilkchickensalad.github.io/ALPS-logger/",
            gcode_root=Path(self.temp_dir.name),
            axisforge_config=None,
            launch_package="axisforge_bringup",
            launch_file="ur_master.launch.py",
            ros2_command="ros2",
        )
        self.settings = settings
        self.patches = [
            patch("bridge_dashboard.app.MoonrakerClient", return_value=self.moonraker),
            patch("bridge_dashboard.app.RosBridge", return_value=self.rosbridge),
            patch("bridge_dashboard.app.AxisForgeRunner", return_value=self.axisforge),
        ]
        for item in self.patches:
            item.start()
        self.client_context = TestClient(create_app(settings))
        self.client = self.client_context.__enter__()

    def tearDown(self):
        self.client_context.__exit__(None, None, None)
        for item in reversed(self.patches):
            item.stop()
        self.temp_dir.cleanup()

    def test_live_job_is_always_blocked(self):
        response = self.client.post(
            "/api/axisforge/start",
            json={"job": "part.gcode", "dry_run": False},
        )
        self.assertEqual(response.status_code, 409)
        self.assertEqual(self.axisforge.started, [])

    def test_synchronized_start_requires_preflight_and_legacy_arm_is_rejected(self):
        for action in ("arm", "start"):
            response = self.client.post(f"/api/sync/{action}", json={"job": "part.gcode"})
            self.assertEqual(response.status_code, 409)
        self.assertEqual(self.axisforge.started, [])

    def test_planning_check_starts_without_live_motion(self):
        response = self.client.post(
            "/api/axisforge/start",
            json={"job": "part.gcode", "dry_run": True},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(self.axisforge.started, [("part.gcode", True)])

    def test_upload_stages_and_forwards_same_gcode(self):
        response = self.client.post(
            "/api/jobs/upload",
            files={"file": ("part.gcode", b"G1 X1\n", "application/octet-stream")},
        )
        self.assertEqual(response.status_code, 200)
        self.assertFalse(response.json()["ready"])
        self.assertEqual(self.moonraker.uploads, [("part.gcode", b"G1 X1\n")])
        self.assertEqual(
            (Path(self.temp_dir.name) / "part.gcode").read_bytes(),
            b"G1 X1\n",
        )

    def test_upload_rejects_non_gcode_files(self):
        response = self.client.post(
            "/api/jobs/upload",
            files={"file": ("part.txt", b"not gcode", "text/plain")},
        )
        self.assertEqual(response.status_code, 400)
        self.assertEqual(self.moonraker.uploads, [])

    def test_temperature_control_endpoint_is_removed(self):
        response = self.client.post(
            "/api/klipper/temperature",
            json={"heater": "extruder", "target": 200},
        )
        self.assertEqual(response.status_code, 404)

    def test_read_only_telemetry_and_failure(self):
        response = self.client.get("/api/klipper/telemetry")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["print"]["state"], "printing")
        self.moonraker.fail_telemetry = True
        response = self.client.get("/api/klipper/telemetry")
        self.assertEqual(response.status_code, 503)
        self.assertIn("telemetry unavailable", response.json()["detail"])

    def test_failed_moonraker_upload_removes_staged_file(self):
        self.moonraker.fail_upload = True
        response = self.client.post(
            "/api/jobs/upload",
            files={"file": ("part.gcode", b"G1 X1\n", "application/octet-stream")},
        )
        self.assertEqual(response.status_code, 503)
        self.assertFalse((Path(self.temp_dir.name) / "part.gcode").exists())

    def test_links_include_configured_printer_ui_and_alps_logger(self):
        response = self.client.get("/api/links")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["fluidd"], "http://printer.local")
        self.assertIsNone(response.json()["mainsail"])
        self.assertIn("ALPS-logger", response.json()["alps_logger"])

    def test_emergency_stop_requests_both_systems(self):
        response = self.client.post("/api/klipper/emergency-stop")
        self.assertEqual(response.status_code, 200)
        self.assertTrue(self.moonraker.stopped)
        self.assertTrue(self.axisforge.stop_requested)

    def test_ready_jobs_lists_only_jobs_that_passed_both_preflights(self):
        with patch("bridge_dashboard.app.Workflow.ready_jobs",
                   return_value=[{"job": "part.gcode", "sha256": "abc"}]) as ready:
            response = self.client.get("/api/jobs/ready")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["jobs"], [{"job": "part.gcode", "sha256": "abc"}])
        ready.assert_called_once_with(["part.gcode"])

    def test_preflight_passes_explicit_heater_options(self):
        choices={'extruder':False,'heater_bed':True}
        with patch('bridge_dashboard.app.Workflow.preflight',new_callable=AsyncMock,
                   return_value={'state':'preflighting'}) as preflight:
            response=self.client.post('/api/jobs/part.gcode/preflight/both',json={'heater_checks':choices})
        self.assertEqual(response.status_code,200)
        preflight.assert_awaited_once_with('part.gcode',choices)

    def test_legacy_preflight_defaults_to_both_checks(self):
        with patch('bridge_dashboard.app.Workflow.preflight',new_callable=AsyncMock,
                   return_value={'state':'preflighting'}) as preflight:
            response=self.client.post('/api/jobs/part.gcode/preflight/both')
        self.assertEqual(response.status_code,200)
        preflight.assert_awaited_once_with('part.gcode',{'extruder':True,'heater_bed':True})

    def test_invalid_heater_options_are_rejected(self):
        for choices in ({'heater_bed':'false'},{'bed':False},{'extruder':0}):
            with self.subTest(choices=choices):
                response=self.client.post('/api/jobs/part.gcode/preflight/both',json={'heater_checks':choices})
                self.assertEqual(response.status_code,422)

    def test_start_passes_options_for_comparison_with_saved_preflight(self):
        choices={'extruder':True,'heater_bed':False}
        with patch('bridge_dashboard.app.Workflow.start',new_callable=AsyncMock,
                   return_value={'state':'starting'}) as start:
            response=self.client.post('/api/sync/start',json={'job':'part.gcode','heater_checks':choices})
        self.assertEqual(response.status_code,200)
        start.assert_awaited_once_with('part.gcode',choices)


if __name__ == "__main__":
    unittest.main()
