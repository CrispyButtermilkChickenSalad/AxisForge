import unittest

from bridge_dashboard.axisforge import AxisForgeError
from bridge_dashboard.sync_start import SyncController, SyncError


class FakePreflight:
    def __init__(self):
        self.state = "preflight_passed"
        self.sha = "abc"

    def get(self, job):
        return {"state": self.state, "sha256": self.sha}


class FakeRos:
    def status(self):
        return {"connected": True, "joint_state_fresh": True}


class FakeAxisForge:
    def __init__(self):
        self.running = False
        self.code = None
        self.stops = 0
        self.fail_start = False

    def status(self):
        return {"running": self.running, "planning_check_running": False, "return_code": self.code}

    async def start(self, job, dry_run=False):
        if self.fail_start:
            raise AxisForgeError("boom")
        assert dry_run is False
        self.running = True

    async def request_stop(self, ros):
        self.stops += 1
        self.running = False


class FakeMoonraker:
    def __init__(self):
        self.snapshot = {"state": "ready", "print_state": "standby"}
        self.started = []
        self.cancels = 0

    async def printer_snapshot(self):
        return dict(self.snapshot)

    async def start_print(self, name):
        self.started.append(name)
        self.snapshot["print_state"] = "printing"

    async def cancel_print(self):
        self.cancels += 1


class SyncTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.pre, self.af, self.mr = FakePreflight(), FakeAxisForge(), FakeMoonraker()
        self.sync = SyncController(self.mr, FakeRos(), self.af, self.pre,
                                   klipper_delay_s=0.05, poll_s=0.01)

    async def test_passing_preflight_cannot_arm_or_start(self):
        self.assertFalse(self.sync.status()["control_ready"])
        for operation in (self.sync.arm, self.sync.start):
            with self.assertRaisesRegex(SyncError, "Klipper-timed robot following"):
                await operation("a.gcode")
        self.assertFalse(self.af.running)
        self.assertEqual(self.mr.started, [])

    async def test_existing_arm_cannot_bypass_disabled_start(self):
        self.sync._armed = {
            "job": "a.gcode", "sha256": "abc", "expires": self.sync.clock() + 60,
        }
        with self.assertRaisesRegex(SyncError, "Klipper-timed robot following"):
            await self.sync.start("a.gcode")
        self.assertFalse(self.af.running)
        self.assertEqual(self.mr.started, [])

    async def test_abort_remains_available(self):
        self.af.running = True
        result = await self.sync.abort()
        self.assertEqual(result["state"], "aborted")
        self.assertEqual(self.af.stops, 1)
        self.assertEqual(self.mr.cancels, 1)


if __name__ == "__main__":
    unittest.main()
