"""Small Moonraker HTTP client for status and deliberate supervisory commands."""

import hashlib
from urllib.parse import quote

import httpx


class MoonrakerError(RuntimeError):
    """Moonraker could not complete a requested operation."""


class MoonrakerClient:
    """Query printer state and send low-rate G-code or emergency-stop requests."""

    def __init__(self, base_url, api_key=None):
        headers = {"X-Api-Key": api_key} if api_key else {}
        self._client = httpx.AsyncClient(
            base_url=base_url,
            headers=headers,
            timeout=httpx.Timeout(5.0, connect=2.0),
        )

    async def close(self):
        await self._client.aclose()

    async def upload_gcode(self, filename, local_path):
        """Upload a staged G-code file to Moonraker's gcodes root."""
        if await self.file_metadata(filename) is not None:
            raise MoonrakerError('A Klipper file already uses this version name; retry to create a new version')
        try:
            with local_path.open("rb") as gcode_file:
                response = await self._client.post(
                    "/server/files/upload",
                    data={"root": "gcodes"},
                    files={"file": (filename, gcode_file, "application/octet-stream")},
                    timeout=60.,
                )
                response.raise_for_status()
        except (httpx.HTTPError, OSError) as error:
            raise MoonrakerError(f"Moonraker G-code upload failed: {error}") from error

    async def status(self):
        """Return normalized status; connection errors remain visible to the UI."""
        try:
            response = await self._client.get(
                "/printer/objects/query?webhooks&toolhead&extruder&heater_bed&fan"
            )
            response.raise_for_status()
            payload = response.json()
            objects = payload.get("result", {}).get("status", {})
            webhooks = objects.get("webhooks", {})
            toolhead = objects.get("toolhead", {})
            extruder = objects.get("extruder", {})
            bed = objects.get("heater_bed", {})
            return {
                "connected": True,
                "state": webhooks.get("state", "unknown"),
                "state_message": webhooks.get("state_message", ""),
                "position": toolhead.get("position"),
                "extruder": {
                    "temperature": extruder.get("temperature"),
                    "target": extruder.get("target"),
                },
                "bed": {
                    "temperature": bed.get("temperature"),
                    "target": bed.get("target"),
                },
                "fan": objects.get("fan", {}).get("speed"),
                "error": None,
            }
        except (httpx.HTTPError, ValueError, TypeError) as error:
            return {
                "connected": False,
                "state": "unavailable",
                "state_message": "",
                "position": None,
                "extruder": {"temperature": None, "target": None},
                "bed": {"temperature": None, "target": None},
                "fan": None,
                "error": str(error),
            }

    async def _get_result(self, path, params=None):
        try:
            response = await self._client.get(path, params=params)
            response.raise_for_status()
            payload = response.json()
            if not isinstance(payload, dict) or "result" not in payload:
                raise ValueError("Missing Moonraker result")
            return payload["result"]
        except (httpx.HTTPError, ValueError) as error:
            raise MoonrakerError(f"Moonraker telemetry request failed: {error}") from error

    async def telemetry(self):
        """Read configured toolhead telemetry without sending printer commands."""
        listing = await self._get_result("/printer/objects/list")
        if not isinstance(listing, dict) or not isinstance(listing.get("objects"), list):
            raise MoonrakerError("Moonraker returned an invalid object list")
        names = [name for name in listing["objects"] if isinstance(name, str)]

        def is_heater(name):
            if name in ("extruder", "heater_bed") or name.startswith("heater_generic "):
                return True
            return name.startswith("extruder") and name[len("extruder"):].isdigit()

        def is_fan(name):
            return name == "fan" or name.startswith(
                ("fan_generic ", "heater_fan ", "controller_fan ", "temperature_fan ")
            )

        temperature_names = [
            name for name in names
            if is_heater(name) or name.startswith(("temperature_sensor ", "temperature_fan "))
        ]
        fan_names = [name for name in names if is_fan(name)]
        print_objects = ["print_stats", "virtual_sdcard", "motion_report", "toolhead", "configfile"]
        query_names = list(dict.fromkeys(
            temperature_names + fan_names + [name for name in print_objects if name in names]
        ))
        objects = {}
        if query_names:
            result = await self._get_result(
                "/printer/objects/query",
                params=[(name, "settings" if name == "configfile" else "") for name in query_names],
            )
            if not isinstance(result, dict) or not isinstance(result.get("status"), dict):
                raise MoonrakerError("Moonraker returned invalid object status")
            objects = result["status"]
            if any(not isinstance(objects.get(name), dict) for name in query_names):
                raise MoonrakerError("A requested Klipper object is unavailable")

        history = {}
        history_error = None
        try:
            store = await self._get_result(
                "/server/temperature_store", params={"include_monitors": "true"}
            )
            if not isinstance(store, dict):
                raise MoonrakerError("Moonraker returned invalid temperature history")
            for name, sensor in store.items():
                if not isinstance(sensor, dict) or not isinstance(sensor.get("temperatures"), list):
                    raise MoonrakerError(f"Invalid temperature history for {name}")
                history[name] = sensor["temperatures"][-180:]
        except MoonrakerError as error:
            history = {}
            history_error = str(error)

        stats = objects.get("print_stats", {})
        info = stats.get("info") if isinstance(stats.get("info"), dict) else {}
        toolhead = objects.get("toolhead", {})
        config = objects.get("configfile", {}).get("settings", {})
        if not isinstance(config, dict):
            raise MoonrakerError("Moonraker returned invalid Klipper settings")
        extruder = config.get(toolhead.get("extruder"), {})
        if not isinstance(extruder, dict):
            extruder = {}
        metadata = {}
        metadata_error = None
        filename = stats.get("filename")
        if isinstance(filename, str) and filename:
            try:
                metadata = await self._get_result(
                    "/server/files/metadata", params={"filename": filename}
                )
                if not isinstance(metadata, dict):
                    raise MoonrakerError("Moonraker returned invalid G-code metadata")
            except MoonrakerError as error:
                metadata = {}
                metadata_error = str(error)

        return {
            "temperatures": [
                {
                    "name": name,
                    "temperature": objects[name].get("temperature"),
                    "target": objects[name].get("target"),
                }
                for name in temperature_names
            ],
            "fans": [
                {"name": name, "speed": objects[name].get("speed")}
                for name in fan_names
            ],
            "print": {
                "state": stats.get("state"),
                "filename": filename,
                "total_duration": stats.get("total_duration"),
                "print_duration": stats.get("print_duration"),
                "filament_used": stats.get("filament_used"),
                "current_layer": info.get("current_layer"),
                "total_layer": info.get("total_layer"),
                "progress": objects.get("virtual_sdcard", {}).get("progress"),
                "velocity": objects.get("motion_report", {}).get("live_velocity"),
                "extruder_velocity": objects.get("motion_report", {}).get("live_extruder_velocity"),
                "filament_diameter": extruder.get("filament_diameter"),
                "slicer_estimate": metadata.get("estimated_time"),
            },
            "metadata_error": metadata_error,
            "history": history,
            "history_error": history_error,
        }

    async def printer_snapshot(self):
        """Read the Klipper state needed by preflight; never sends commands."""
        try:
            listing = (await self._get("/printer/objects/list")).get("objects")
            if not isinstance(listing, list):
                raise ValueError("invalid object list")
            wanted = [name for name in ("webhooks", "print_stats", "extruder") if name in listing]
            status = (await self._get("/printer/objects/query", [(n, "") for n in wanted]))["status"]
        except (httpx.HTTPError, ValueError, KeyError, AttributeError) as error:
            raise MoonrakerError(f"Moonraker preflight query failed: {error}") from error
        extruder = status.get("extruder")
        return {
            "state": status.get("webhooks", {}).get("state", "unknown"),
            "state_message": status.get("webhooks", {}).get("state_message", ""),
            "print_state": status.get("print_stats", {}).get("state"),
            "extruder": None if not isinstance(extruder, dict) else {
                "can_extrude": extruder.get("can_extrude"),
                "temperature": extruder.get("temperature"),
                "target": extruder.get("target"),
            },
            "exporter_loaded": "robot_motion_bridge" in listing,
        }

    async def _get(self, path, params=None):
        response = await self._client.get(path, params=params)
        response.raise_for_status()
        return response.json()["result"]

    async def file_metadata(self, filename):
        """Return Moonraker's parsed G-code metadata, or None if the file is absent."""
        try:
            response = await self._client.get(
                "/server/files/metadata", params={"filename": filename}
            )
            if response.status_code == 404:
                return None
            response.raise_for_status()
            result = response.json()["result"]
            if not isinstance(result, dict):
                raise ValueError("invalid metadata")
            return result
        except (httpx.HTTPError, ValueError, KeyError) as error:
            raise MoonrakerError(f"Moonraker G-code metadata request failed: {error}") from error

    async def file_sha256(self, filename):
        """Hash the copy of a G-code file stored on Klipper, or None if absent."""
        digest = hashlib.sha256()
        try:
            async with self._client.stream(
                "GET", f"/server/files/gcodes/{quote(filename)}"
            ) as response:
                if response.status_code == 404:
                    return None
                response.raise_for_status()
                async for chunk in response.aiter_bytes():
                    digest.update(chunk)
        except httpx.HTTPError as error:
            raise MoonrakerError(f"Moonraker G-code download failed: {error}") from error
        return digest.hexdigest()

    async def start_print(self, filename):
        await self._post("/printer/print/start", {"filename": filename}, "start print")

    async def cancel_print(self):
        await self._post("/printer/print/cancel", None, "cancel print")

    async def _post(self, path, params, action):
        try:
            response = await self._client.post(path, params=params)
            response.raise_for_status()
        except httpx.HTTPError as error:
            raise MoonrakerError(f"Klipper {action} failed: {error}") from error

    async def emergency_stop(self):
        try:
            response = await self._client.post("/printer/emergency_stop")
            response.raise_for_status()
        except httpx.HTTPError as error:
            raise MoonrakerError(f"Klipper emergency stop failed: {error}") from error
