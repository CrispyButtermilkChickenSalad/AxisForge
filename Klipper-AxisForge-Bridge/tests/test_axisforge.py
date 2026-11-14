"""Focused tests for AxisForge job allow-listing and command safety."""

import tempfile
import unittest
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace

from bridge_dashboard.axisforge import AxisForgeError, AxisForgeRunner


class AxisForgeRunnerTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name).resolve()
        (self.root / "sample.gcode").write_text("G1 X1\n", encoding="utf-8")
        settings = SimpleNamespace(
            gcode_root=self.root,
            axisforge_config=None,
            ros2_command="ros2",
            launch_package="axisforge_bringup",
            launch_file="ur_master.launch.py",
        )
        self.runner = AxisForgeRunner(settings)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_lists_only_gcode_files(self):
        (self.root / "notes.txt").write_text("not a job", encoding="utf-8")
        self.assertEqual(self.runner.list_jobs()["jobs"], ["sample.gcode"])

    def test_rejects_traversal_and_non_gcode(self):
        for name in ("../outside.gcode", "sample.txt", "folder\\sample.gcode"):
            with self.subTest(name=name), self.assertRaises(AxisForgeError):
                self.runner._resolve_job(name)

    def test_stages_uploaded_gcode(self):
        destination = self.runner.stage_upload("uploaded.gcode", BytesIO(b"G1 X2\n"))
        self.assertEqual(destination.read_bytes(), b"G1 X2\n")
        self.assertRegex(destination.name, r'^uploaded--[0-9a-f]{12}\.gcode$')
        self.assertIn(destination.name, self.runner.list_jobs()["jobs"])

    def test_rejects_empty_upload_and_versions_repeated_names_without_overwriting(self):
        with self.assertRaises(AxisForgeError):
            self.runner.stage_upload("empty.gcode", BytesIO(b""))
        first = self.runner.stage_upload("sample.gcode", BytesIO(b"G1 X2\n"))
        second = self.runner.stage_upload("sample.gcode", BytesIO(b"G1 X3\n"))
        self.assertNotEqual(first.name,second.name)
        self.assertEqual((self.root/'sample.gcode').read_bytes(),b'G1 X1\n')
        self.assertEqual(first.read_bytes(),b'G1 X2\n')
        self.assertEqual(second.read_bytes(),b'G1 X3\n')
        self.assertFalse((self.root / "empty.gcode").exists())

    def test_rejects_symlink_outside_job_directory(self):
        outside = self.root.parent / "outside.gcode"
        outside.write_text("G1 X100\n", encoding="utf-8")
        link = self.root / "escape.gcode"
        try:
            link.symlink_to(outside)
        except (OSError, NotImplementedError):
            self.skipTest("Symlinks are unavailable")
        with self.assertRaises(AxisForgeError):
            self.runner._resolve_job("escape.gcode")


if __name__ == "__main__":
    unittest.main()
