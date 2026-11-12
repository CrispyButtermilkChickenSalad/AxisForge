"""Tests for read-only Klipper toolhead telemetry."""

import unittest

import httpx

from bridge_dashboard.moonraker import MoonrakerClient, MoonrakerError


class MoonrakerTelemetryTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.requests = []
        self.history_status = 200
        self.query_status = 200
        self.metadata_status = 200

        def respond(request):
            self.requests.append(request)
            if request.url.path == "/printer/objects/list":
                return httpx.Response(200, json={"result": {"objects": [
                    "extruder", "temperature_sensor enclosure", "fan",
                    "heater_fan hotend", "print_stats", "motion_report",
                    "virtual_sdcard", "toolhead", "configfile", "gcode_macro PAUSE",
                ]}})
            if request.url.path == "/printer/objects/query":
                if self.query_status != 200:
                    return httpx.Response(self.query_status)
                return httpx.Response(200, json={"result": {"status": {
                    "extruder": {"temperature": 200.0, "target": 205.0},
                    "temperature_sensor enclosure": {"temperature": 35.0},
                    "fan": {"speed": 0.5},
                    "heater_fan hotend": {"speed": 1.0},
                    "print_stats": {
                        "filename": "folder/example.gcode", "state": "printing",
                        "total_duration": 200.0, "print_duration": 100.0,
                        "filament_used": 20.0,
                        "info": {"current_layer": 2, "total_layer": 10},
                    },
                    "motion_report": {"live_velocity": 25.0, "live_extruder_velocity": 2.0},
                    "virtual_sdcard": {"progress": 0.25},
                    "toolhead": {"extruder": "extruder"},
                    "configfile": {"settings": {"extruder": {"filament_diameter": 1.75}}},
                }}})
            if request.url.path == "/server/temperature_store":
                if self.history_status != 200:
                    return httpx.Response(self.history_status)
                return httpx.Response(200, json={"result": {
                    "extruder": {"temperatures": list(range(200))},
                    "temperature_sensor enclosure": {"temperatures": [35.0, None, 36.0]},
                }})
            if request.url.path == "/server/files/metadata":
                if self.metadata_status != 200:
                    return httpx.Response(self.metadata_status)
                return httpx.Response(200, json={"result": {"estimated_time": 400.0}})
            return httpx.Response(404)

        self.client = MoonrakerClient("http://printer")
        await self.client._client.aclose()
        self.client._client = httpx.AsyncClient(
            base_url="http://printer", transport=httpx.MockTransport(respond)
        )

    async def asyncTearDown(self):
        await self.client.close()

    async def test_discovers_telemetry_and_limits_history_without_commands(self):
        result = await self.client.telemetry()
        self.assertEqual(result["temperatures"][0], {
            "name": "extruder", "temperature": 200.0, "target": 205.0,
        })
        self.assertEqual(result["fans"][1], {"name": "heater_fan hotend", "speed": 1.0})
        self.assertEqual(result["print"]["velocity"], 25.0)
        self.assertEqual(result["print"]["extruder_velocity"], 2.0)
        self.assertEqual(result["print"]["filament_diameter"], 1.75)
        self.assertEqual(result["print"]["current_layer"], 2)
        self.assertEqual(result["print"]["slicer_estimate"], 400.0)
        self.assertNotIn("macros", result)
        self.assertEqual(result["history"]["extruder"], list(range(20, 200)))
        self.assertEqual(result["history"]["temperature_sensor enclosure"], [35.0, None, 36.0])
        self.assertTrue(all(request.method == "GET" for request in self.requests))
        self.assertIn("heater_fan hotend", self.requests[1].url.params)
        metadata_request = next(request for request in self.requests if request.url.path == "/server/files/metadata")
        self.assertEqual(metadata_request.url.params["filename"], "folder/example.gcode")

    async def test_history_failure_keeps_live_readings_and_reports_error(self):
        self.history_status = 404
        result = await self.client.telemetry()
        self.assertEqual(result["print"]["velocity"], 25.0)
        self.assertEqual(result["history"], {})
        self.assertIn("404", result["history_error"])

    async def test_missing_metadata_is_reported_without_hiding_print_stats(self):
        self.metadata_status = 404
        result = await self.client.telemetry()
        self.assertEqual(result["print"]["velocity"], 25.0)
        self.assertIsNone(result["print"]["slicer_estimate"])
        self.assertIn("404", result["metadata_error"])

    async def test_status_failure_is_not_reported_as_empty_telemetry(self):
        self.query_status = 503
        with self.assertRaises(MoonrakerError):
            await self.client.telemetry()
