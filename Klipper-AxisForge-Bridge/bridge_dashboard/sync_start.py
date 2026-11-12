"""Synchronized start is disabled until Klipper-timed robot following exists."""

import asyncio
import logging
import time

from bridge_dashboard.axisforge import AxisForgeError
from bridge_dashboard.moonraker import MoonrakerError

logger = logging.getLogger(__name__)
ARM_TTL_S = 60.0
CONTROL_READY = False
CONTROL_UNAVAILABLE = (
    "Synchronized start is unavailable: Klipper-timed robot following is not implemented"
)


class SyncError(RuntimeError):
    pass


class SyncController:
    def __init__(self, moonraker, rosbridge, axisforge, preflight, klipper_delay_s=10.0,
                 poll_s=0.5, clock=time.monotonic):
        self.moonraker, self.rosbridge = moonraker, rosbridge
        self.axisforge, self.preflight = axisforge, preflight
        self.klipper_delay_s, self.poll_s, self.clock = klipper_delay_s, poll_s, clock
        self._armed = None
        self._task = None
        self.state, self.message = "idle", ""

    def status(self):
        armed = self._armed
        if armed and self.clock() > armed["expires"]:
            self._armed = armed = None
        return {
            "state": self.state, "message": self.message, "job": self._job(),
            "armed": armed is not None,
            "armed_expires_in_s": None if armed is None else max(0.0, armed["expires"] - self.clock()),
            "control_ready": CONTROL_READY,
        }

    def _job(self):
        return self._armed["job"] if self._armed else getattr(self, "_running_job", None)

    async def _verify(self, job):
        record = self.preflight.get(job)
        if record["state"] != "preflight_passed":
            raise SyncError(f"Preflight has not passed for {job} (state: {record['state']})")
        ros = self.rosbridge.status()
        if not ros.get("connected") or not ros.get("joint_state_fresh"):
            raise SyncError("ROS bridge is disconnected or joint states are stale")
        if self.axisforge.status()["running"] or self.axisforge.status()["planning_check_running"]:
            raise SyncError("AxisForge is already busy")
        try:
            snapshot = await self.moonraker.printer_snapshot()
        except MoonrakerError as error:
            raise SyncError(str(error)) from error
        if snapshot["state"] != "ready" or snapshot["print_state"] in ("printing", "paused"):
            raise SyncError("Klipper is not ready and idle")
        return record["sha256"]

    async def arm(self, job):
        if not CONTROL_READY:
            raise SyncError(CONTROL_UNAVAILABLE)
        if self._task and not self._task.done():
            raise SyncError("A synchronized job is already running")
        sha = await self._verify(job)
        self._armed = {"job": job, "sha256": sha, "expires": self.clock() + ARM_TTL_S}
        self.state, self.message = "armed", f"Armed for {ARM_TTL_S:.0f} s"
        return self.status()

    def disarm(self):
        self._armed = None
        if self.state == "armed":
            self.state, self.message = "idle", "Disarmed"

    async def start(self, job):
        if not CONTROL_READY:
            raise SyncError(CONTROL_UNAVAILABLE)
        armed = self._armed
        if not armed or armed["job"] != job or self.clock() > armed["expires"]:
            self._armed = None
            raise SyncError("Arm the job first; arming expires after 60 s")
        if await self._verify(job) != armed["sha256"]:
            self._armed = None
            raise SyncError("The job changed after arming")
        self._armed = None
        self._running_job = job
        self.state, self.message = "starting_robot", "Starting AxisForge"
        try:
            await self.axisforge.start(job, dry_run=False)
        except AxisForgeError as error:
            self.state, self.message = "failed", str(error)
            raise SyncError(str(error)) from error
        self._task = asyncio.create_task(self._run(job))
        return self.status()

    async def _run(self, job):
        try:
            deadline = self.clock() + self.klipper_delay_s
            self.state, self.message = "robot_moving", "Robot approaching; Klipper starts shortly"
            while self.clock() < deadline:
                self._check_robot()
                await asyncio.sleep(self.poll_s)
            self._check_robot()
            self.state, self.message = "starting_klipper", "Starting Klipper print"
            await self.moonraker.start_print(job)
            self.state, self.message = "running", "Running"
            while True:
                await asyncio.sleep(self.poll_s)
                robot_running = self.axisforge.status()["running"]
                snapshot = await self.moonraker.printer_snapshot()
                if snapshot["state"] != "ready" or snapshot["print_state"] == "error":
                    raise SyncError(f"Klipper fault: {snapshot['state']} / {snapshot['print_state']}")
                if not robot_running:
                    code = self.axisforge.status()["return_code"]
                    if code not in (0, None):
                        raise SyncError(f"AxisForge exited with code {code}")
                    if snapshot["print_state"] == "printing":
                        raise SyncError("Robot finished while Klipper is still printing")
                    self.state, self.message = "complete", "Job complete"
                    return
        except asyncio.CancelledError:
            raise
        except Exception as error:
            logger.exception("Synchronized job fault; stopping both sides")
            await self.abort(str(error))

    def _check_robot(self):
        status = self.axisforge.status()
        if not status["running"]:
            raise SyncError(f"AxisForge stopped before Klipper started (code {status['return_code']})")

    async def abort(self, reason="Operator abort"):
        self._armed = None
        results = await asyncio.gather(
            self.axisforge.request_stop(self.rosbridge),
            self.moonraker.cancel_print(),
            return_exceptions=True,
        )
        failed = [str(r) for r in results if isinstance(r, Exception)]
        self.state = "aborted"
        self.message = reason + (f" (stop errors: {'; '.join(failed)})" if failed else "")
        return self.status()
