"""Tests for G-code analysis and the Klipper/robot preflight state machine."""

import hashlib
import tempfile
import unittest
from pathlib import Path

from bridge_dashboard.axisforge import AxisForgeError
from bridge_dashboard.gcode_check import analyze_gcode
from bridge_dashboard.moonraker import MoonrakerError
from bridge_dashboard.preflight import PreflightManager

GCODE = b"M83\nG1 X10 Y5 Z0.2 A15 E0.5 ; comment\nG1 X20 E0.5\n"


class FakeMoonraker:
    def __init__(self, remote=GCODE):
        self.remote = remote
        self.snapshot = {
            "state": "ready", "state_message": "", "print_state": "standby",
            "extruder": {"can_extrude": True, "temperature": 210.0, "target": 210.0},
            "exporter_loaded": True,
        }
        self.metadata = {"size": 1}
        self.fail_snapshot = False

    async def printer_snapshot(self):
        if self.fail_snapshot:
            raise MoonrakerError("unreachable")
        return self.snapshot

    async def file_sha256(self, filename):
        return None if self.remote is None else hashlib.sha256(self.remote).hexdigest()

    async def file_metadata(self, filename):
        return self.metadata


class FakeRos:
    def __init__(self):
        self.state = {"connected": True, "joint_state_fresh": True}

    def status(self):
        return self.state


class FakeAxisForge:
    def __init__(self, path):
        self.path = path
        self.running = False
        self.result = {
            "passed": True, "return_code": 0, "output": [],
            "summary": {"source_segments": 2, "actions": 1, "duration_s": 3.0, "extrusion_mm": 1.0},
        }

    def job_path(self, name):
        if name != self.path.name:
            raise AxisForgeError("missing")
        return self.path

    def status(self):
        return {"running": self.running}

    async def preflight(self, name):
        return self.result


class AnalyzeGcodeTests(unittest.TestCase):
    def test_reports_axis_ranges_rotary_axes_and_extrusion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "a.gcode"
            path.write_bytes(b"G28 X Y\nM82\nG92 E0\nG1 X1 B5 E1\nG1 X3 C-2 E3\nG1 E2\n")
            result = analyze_gcode(path)
        self.assertEqual(result["axes"]["X"], [1.0, 3.0])
        self.assertEqual(result["rotary_axes_used"], ["B", "C"])
        self.assertEqual(result["extrusion_mm"], 3.0)


class PreflightTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "part.gcode"
        self.path.write_bytes(GCODE)
        self.moonraker = FakeMoonraker()
        self.ros = FakeRos()
        self.axisforge = FakeAxisForge(self.path)
        self.manager = PreflightManager(self.moonraker, self.ros, self.axisforge)

    def tearDown(self):
        self.directory.cleanup()

    async def test_both_preflights_pass(self):
        self.assertEqual((await self.manager.run_klipper("part.gcode"))["state"], "klipper_ok")
        result = await self.manager.run_robot("part.gcode")
        self.assertEqual(result["state"], "preflight_passed")
        self.assertEqual(result["sha256"], hashlib.sha256(GCODE).hexdigest())

    async def test_klipper_copy_mismatch_fails(self):
        self.moonraker.remote = b"different"
        result = await self.manager.run_klipper("part.gcode")
        self.assertEqual(result["state"], "failed")
        failed = [c for c in result["klipper"]["checks"] if c["status"] == "fail"]
        self.assertEqual([c["name"] for c in failed], ["G-code on Klipper"])

    async def test_cold_extruder_and_printing_fail(self):
        self.moonraker.snapshot["extruder"]["can_extrude"] = False
        self.moonraker.snapshot["print_state"] = "printing"
        result = await self.manager.run_klipper("part.gcode")
        names = {c["name"] for c in result["klipper"]["checks"] if c["status"] == "fail"}
        self.assertEqual(names, {"Printer idle", "Extruder ready"})

    async def test_missing_exporter_only_warns(self):
        self.moonraker.snapshot["exporter_loaded"] = False
        result = await self.manager.run_klipper("part.gcode")
        self.assertTrue(result["klipper"]["passed"])
        self.assertIn("warn", [c["status"] for c in result["klipper"]["checks"]])

    async def test_unreachable_klipper_fails(self):
        self.moonraker.fail_snapshot = True
        self.assertFalse((await self.manager.run_klipper("part.gcode"))["klipper"]["passed"])

    async def test_stale_joint_state_and_failed_dry_run_fail_robot(self):
        self.ros.state = {"connected": True, "joint_state_fresh": False}
        self.axisforge.result = {"passed": False, "return_code": 1, "summary": None, "output": ["IK failed"]}
        result = await self.manager.run_robot("part.gcode")
        names = {c["name"] for c in result["robot"]["checks"] if c["status"] == "fail"}
        self.assertEqual(names, {"Joint feedback", "AxisForge planning check"})

    async def test_abc_rotation_axes_pass_robot(self):
        self.path.write_bytes(b"G1 X1 A5 B10 C20\n")
        result = await self.manager.run_robot("part.gcode")
        check = next(c for c in result["robot"]["checks"] if c["name"] == "Rotation axes")
        self.assertEqual(check["status"], "pass")

    async def test_file_change_invalidates_results(self):
        await self.manager.run_klipper("part.gcode")
        self.path.write_bytes(GCODE + b"G1 X30\n")
        self.assertEqual(self.manager.get("part.gcode")["state"], "staged")

    async def test_ready_jobs_requires_both_passes_on_current_file(self):
        self.assertEqual(self.manager.ready_jobs(["part.gcode", "missing.gcode"]), [])
        await self.manager.run_klipper("part.gcode")
        self.assertEqual(self.manager.ready_jobs(["part.gcode"]), [])
        await self.manager.run_robot("part.gcode")
        ready = self.manager.ready_jobs(["part.gcode", "missing.gcode"])
        self.assertEqual([item["job"] for item in ready], ["part.gcode"])
        self.path.write_bytes(GCODE + b"G1 X30\n")
        self.assertEqual(self.manager.ready_jobs(["part.gcode"]), [])

    async def test_klipper_copy_mismatch_never_becomes_ready(self):
        self.moonraker.remote = b"different"
        await self.manager.run_klipper("part.gcode")
        await self.manager.run_robot("part.gcode")
        self.assertEqual(self.manager.ready_jobs(["part.gcode"]), [])


if __name__ == "__main__":
    unittest.main()
