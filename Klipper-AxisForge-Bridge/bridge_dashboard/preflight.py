"""Per-job Klipper and robot preflight state.

Klipper is responsible for extrusion only; the robot performs all six motion
axes. Preflight results are tied to the file's content hash and invalidated when
the staged file changes.
"""

import asyncio
import time

from bridge_dashboard.axisforge import AxisForgeError
from bridge_dashboard.gcode_check import analyze_gcode, sha256_file
from bridge_dashboard.moonraker import MoonrakerError

# AxisForge's rotary-axes-abc branch reads A/B/C (degrees about X/Y/Z) from G-code.
AXISFORGE_ROTARY_AXES = ("A", "B", "C")
BUSY_PRINT_STATES = ("printing", "paused")


def _check(name, status, detail):
    return {"name": name, "status": status, "detail": detail}


def _passed(checks):
    return all(check["status"] != "fail" for check in checks)


class PreflightManager:
    def __init__(self, moonraker, rosbridge, axisforge):
        self.moonraker = moonraker
        self.rosbridge = rosbridge
        self.axisforge = axisforge
        self._records = {}
        self._locks = {"klipper": asyncio.Lock(), "robot": asyncio.Lock()}

    def _fingerprint(self, path):
        stat = path.stat()
        return (stat.st_size, stat.st_mtime_ns)

    def _record_for(self, job, fingerprint, file_hash=None):
        """Return the record for the current file, discarding results for an older one."""
        record = self._records.get(job)
        if record is None or record["fingerprint"] != fingerprint or (
            file_hash is not None and record["sha256"] != file_hash
        ):
            record = {
                "job": job, "sha256": file_hash, "fingerprint": fingerprint,
                "klipper": None, "robot": None,
            }
            self._records[job] = record
        return record

    def get(self, job):
        path = self.axisforge.job_path(job)
        return self._view(self._record_for(job, self._fingerprint(path)))

    def ready_jobs(self, names):
        """Jobs whose Klipper and robot preflights both passed for the current file."""
        ready = []
        for name in names:
            try:
                view = self.get(name)
            except (AxisForgeError, OSError):
                continue
            if view["state"] == "preflight_passed":
                ready.append({"job": name, "sha256": view["sha256"]})
        return ready

    def _view(self, record):
        results = [record["klipper"], record["robot"]]
        if all(result is not None and result["passed"] for result in results):
            state = "preflight_passed"
        elif any(result is not None and not result["passed"] for result in results):
            state = "failed"
        elif record["klipper"]:
            state = "klipper_ok"
        elif record["robot"]:
            state = "robot_ok"
        else:
            state = "staged"
        return {
            "job": record["job"], "sha256": record["sha256"], "state": state,
            "klipper": record["klipper"], "robot": record["robot"],
        }

    async def _prepare(self, job):
        path = self.axisforge.job_path(job)
        fingerprint = self._fingerprint(path)
        file_hash = await asyncio.to_thread(sha256_file, path)
        analysis = await asyncio.to_thread(analyze_gcode, path)
        return path, fingerprint, file_hash, analysis

    def _store(self, job, path, fingerprint, file_hash, kind, checks):
        try:
            changed = self._fingerprint(path) != fingerprint
        except OSError:
            changed = True
        if changed:
            raise AxisForgeError("The G-code file changed during preflight; run it again")
        record = self._record_for(job, fingerprint, file_hash)
        record[kind] = {"passed": _passed(checks), "checked_at": time.time(), "checks": checks}
        return self._view(record)

    async def run_klipper(self, job):
        async with self._locks["klipper"]:
            path, fingerprint, file_hash, analysis = await self._prepare(job)
            checks = []
            snapshot = None
            try:
                snapshot = await self.moonraker.printer_snapshot()
            except MoonrakerError as error:
                checks.append(_check("Klipper reachable", "fail", str(error)))
            if snapshot is not None:
                ready = snapshot["state"] == "ready"
                checks.append(_check(
                    "Klippy ready", "pass" if ready else "fail",
                    "Klippy is ready" if ready else
                    f"Klippy state is {snapshot['state']}: {snapshot['state_message']}".strip(),
                ))
                busy = snapshot["print_state"] in BUSY_PRINT_STATES
                checks.append(_check(
                    "Printer idle", "fail" if busy else "pass",
                    f"A print is {snapshot['print_state']}" if busy else "No print is running",
                ))
                checks.append(self._extruder_check(snapshot["extruder"], analysis["extrusion_mm"]))
                checks.append(_check(
                    "Motion exporter",
                    "pass" if snapshot["exporter_loaded"] else "warn",
                    "robot_motion_bridge is loaded" if snapshot["exporter_loaded"] else
                    "robot_motion_bridge is not loaded; real-time robot following will be unavailable",
                ))
            checks.extend(await self._file_checks(job, file_hash))
            return self._store(job, path, fingerprint, file_hash, "klipper", checks)

    def _extruder_check(self, extruder, extrusion_mm):
        if not extrusion_mm:
            return _check("Extruder ready", "pass", "The job contains no extrusion")
        if extruder is None:
            return _check("Extruder ready", "fail", "Klipper has no extruder but the job extrudes")
        if extruder["can_extrude"]:
            return _check("Extruder ready", "pass",
                          f"Hot enough to extrude ({extruder['temperature']} °C)")
        return _check("Extruder ready", "fail",
                      f"Extruder is too cold to extrude ({extruder['temperature']} °C); heat it first")

    async def _file_checks(self, job, file_hash):
        checks = []
        try:
            remote_hash = await self.moonraker.file_sha256(job)
            if remote_hash is None:
                checks.append(_check("G-code on Klipper", "fail", f"{job} is not on the Klipper host"))
            elif remote_hash != file_hash:
                checks.append(_check("G-code on Klipper", "fail",
                                     "The Klipper copy differs from the AxisForge copy"))
            else:
                checks.append(_check("G-code on Klipper", "pass",
                                     "Klipper and AxisForge hold identical files"))
            metadata = await self.moonraker.file_metadata(job)
            checks.append(_check(
                "G-code metadata", "pass" if metadata else "fail",
                "Moonraker parsed the file" if metadata else "Moonraker has no metadata for the file",
            ))
        except MoonrakerError as error:
            checks.append(_check("G-code on Klipper", "fail", str(error)))
        return checks

    async def run_robot(self, job):
        async with self._locks["robot"]:
            path, fingerprint, file_hash, analysis = await self._prepare(job)
            checks = []
            ros = self.rosbridge.status()
            checks.append(_check("ROS bridge", "pass" if ros["connected"] else "fail",
                                 "rosbridge connected" if ros["connected"] else "rosbridge is not connected"))
            fresh = ros.get("joint_state_fresh")
            checks.append(_check("Joint feedback", "pass" if fresh else "fail",
                                 "Joint states are fresh" if fresh else "Joint states are missing or stale"))
            busy = self.axisforge.status()["running"]
            checks.append(_check("AxisForge idle", "fail" if busy else "pass",
                                 "An AxisForge job is running" if busy else "No AxisForge job is running"))
            unsupported = [a for a in analysis["rotary_axes_used"] if a not in AXISFORGE_ROTARY_AXES]
            used = ", ".join(analysis["rotary_axes_used"]) or "none"
            checks.append(_check(
                "Rotation axes", "fail" if unsupported else "pass",
                f"G-code uses {', '.join(unsupported)}, which AxisForge cannot execute yet "
                "(it reads only A, rotation about X)" if unsupported else f"Rotation axes used: {used}",
            ))
            if busy:
                checks.append(_check("AxisForge planning check", "fail", "Skipped: AxisForge is busy"))
            else:
                try:
                    result = await self.axisforge.preflight(job)
                except AxisForgeError as error:
                    checks.append(_check("AxisForge planning check", "fail", str(error)))
                else:
                    summary = result["summary"]
                    if result["passed"]:
                        detail = (f"{summary['source_segments']} segments, {summary['actions']} actions, "
                                  f"{summary['duration_s']:.1f} s")
                    else:
                        detail = "; ".join(result["output"][-3:]) or f"Exited with {result['return_code']}"
                    checks.append(_check("AxisForge planning check",
                                         "pass" if result["passed"] else "fail", detail))
            return self._store(job, path, fingerprint, file_hash, "robot", checks)
