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
from bridge_dashboard.bed_probe import BedProbeError


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


class FakeBedProbe:
    def __init__(self):
        self.active=False;self.started=[];self.confirmed=[];self.pending=None
    def busy(self):return self.active
    def status(self):
        return {'state':'waiting_surface' if self.active else 'idle','busy':self.active,
                'confirmation':self.pending,'configured':True}
    async def start(self,*,dry_run=False):
        if self.active:raise BedProbeError('already active')
        self.started.append(dry_run);self.active=True
        self.pending={'stage':'surface','token':'a'*24}
        return self.status()
    async def confirm(self,stage,token):
        if self.pending!={'stage':stage,'token':token}:raise BedProbeError('stale confirmation')
        self.confirmed.append((stage,token));self.pending=None
        return self.status()
    async def abort(self,reason='operator'):
        self.active=False;self.pending=None
        return self.status()
    async def close(self):await self.abort()


class DashboardApiTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.moonraker = FakeMoonraker()
        self.rosbridge = FakeRosBridge()
        self.axisforge = FakeAxisForge(Path(self.temp_dir.name))
        self.bed_probe = FakeBedProbe()
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
            patch("bridge_dashboard.app.BedProbeRunner", return_value=self.bed_probe),
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

    def test_calibration_blocks_every_motion_entry_and_connection(self):
        routine=self.client.app.state.load_calibration
        routine._active=True
        for path,body in (('/api/bed-probe/start',{}),('/api/robot/connect',{}),
                          ('/api/jobs/part.gcode/preflight/both',{}),
                          ('/api/sync/start',{'job':'part.gcode'}),
                          ('/api/axisforge/start',{'job':'part.gcode','dry_run':True})):
            with self.subTest(path=path):
                self.assertEqual(self.client.post(path,json=body).status_code,409)
        self.assertEqual(self.client.post('/api/jobs/upload',files={'file':('part.gcode',b'G1 X1')}).status_code,409)
        routine._active=False

    def test_connection_launch_is_allowed_with_klipper_offline_but_not_during_jobs(self):
        self.moonraker.state='disconnected'
        connection=self.client.app.state.robot_connection
        with patch.object(connection,'connect',new_callable=AsyncMock,return_value={'busy':True}) as connect:
            self.assertEqual(self.client.post('/api/robot/connect').status_code,200)
            connect.assert_awaited_once()
            with patch('bridge_dashboard.app.Workflow.busy',return_value=True):
                self.assertEqual(self.client.post('/api/robot/connect').status_code,409)
            self.assertEqual(connect.await_count,1)

    def test_scale_routes_reject_cross_routine_confirmation_and_invalid_moves(self):
        routine=self.client.app.state.load_calibration
        routine._active=True
        routine._snapshot.update(state='waiting_tare',confirmation={'stage':'tare','token':'a'*24})
        with patch.object(routine,'_send',new_callable=AsyncMock) as send:
            self.assertEqual(self.client.post('/api/load-calibration/confirm',json={'stage':'surface','token':'a'*24}).status_code,422)
            self.assertEqual(self.client.post('/api/load-calibration/confirm',json={'stage':'tare','token':'a'*24}).status_code,200)
            self.assertEqual(self.client.post('/api/load-calibration/confirm',json={'stage':'tare','token':'a'*24}).status_code,409)
            self.assertEqual(send.await_count,1)
        for body in ({'command':'step','token':'a'*24,'delta_mm':1.},
                     {'command':'power_on','token':'a'*24}):
            self.assertEqual(self.client.post('/api/load-calibration/action',json=body).status_code,422)
        routine._active=False

    def test_live_job_is_always_blocked(self):
        response = self.client.post(
            "/api/axisforge/start",
            json={"job": "part.gcode", "dry_run": False},
        )
        self.assertEqual(response.status_code, 409)
        self.assertEqual(self.axisforge.started, [])

    def test_bed_probe_api_requires_explicit_matching_confirmation(self):
        result=self.client.post('/api/bed-probe/start',json={'dry_run':False})
        self.assertEqual(result.status_code,200)
        self.assertEqual(result.json()['state'],'waiting_surface')
        self.assertFalse(self.bed_probe.confirmed)
        result=self.client.post('/api/bed-probe/confirm',json={'stage':'front_left','token':'a'*24})
        self.assertEqual(result.status_code,409)
        result=self.client.post('/api/bed-probe/confirm',json={'stage':'surface','token':'a'*24})
        self.assertEqual(result.status_code,200)
        self.assertEqual(self.bed_probe.confirmed,[('surface','a'*24)])
        result=self.client.post('/api/bed-probe/confirm',json={'stage':'surface','token':'a'*24})
        self.assertEqual(result.status_code,409)

    def test_bed_probe_blocks_every_pipeline_entry(self):
        self.bed_probe.active=True
        requests=(('/api/jobs/part.gcode/preflight/both',{}),
                  ('/api/sync/start',{'job':'part.gcode'}),
                  ('/api/axisforge/start',{'job':'part.gcode','dry_run':True}))
        for path,body in requests:
            with self.subTest(path=path):
                response=self.client.post(path,json=body)
                self.assertEqual(response.status_code,409,response.text)
                self.assertIn('Bed homing',response.json()['detail'])
        result=self.client.post('/api/jobs/upload',files={'file':('part.gcode',b'G1 X1\n')})
        self.assertEqual(result.status_code,409);self.assertFalse(self.moonraker.uploads)

    def test_job_and_planning_ownership_block_probe_start(self):
        with patch('bridge_dashboard.app.Workflow.busy',return_value=True):
            result=self.client.post('/api/bed-probe/start',json={})
        self.assertEqual(result.status_code,409)
        for state in ({'running':True},{'planning_check_running':True}):
            with patch.object(self.axisforge,'status',return_value=state):
                result=self.client.post('/api/bed-probe/start',json={})
            self.assertEqual(result.status_code,409)
        self.assertFalse(self.bed_probe.started)

    def test_probe_start_and_preflight_are_serialized_before_child_is_running(self):
        import asyncio
        from concurrent.futures import ThreadPoolExecutor
        import threading
        entered,release=threading.Event(),threading.Event()
        original=self.bed_probe.start
        async def starting(**kwargs):
            entered.set();await asyncio.to_thread(release.wait,2.)
            return await original(**kwargs)
        with patch.object(self.bed_probe,'start',side_effect=starting), \
                patch('bridge_dashboard.app.Workflow.preflight',new_callable=AsyncMock) as preflight, \
                ThreadPoolExecutor(max_workers=2) as executor:
            start=executor.submit(self.client.post,'/api/bed-probe/start',json={})
            self.assertTrue(entered.wait(1.))
            planning=executor.submit(self.client.post,'/api/jobs/part.gcode/preflight/both',json={})
            release.set()
            self.assertEqual(start.result().status_code,200)
            self.assertEqual(planning.result().status_code,409)
            preflight.assert_not_awaited()

    def test_stop_and_emergency_routes_cancel_bed_probe(self):
        for path in ('/api/axisforge/stop','/api/sync/abort','/api/bed-probe/abort'):
            self.bed_probe.active=True
            result=self.client.post(path)
            self.assertEqual(result.status_code,200,result.text)
            self.assertFalse(self.bed_probe.active)
        self.bed_probe.active=True
        result=self.client.post('/api/klipper/emergency-stop')
        self.assertEqual(result.status_code,200,result.text)
        self.assertFalse(self.bed_probe.active);self.assertTrue(self.moonraker.stopped)

    def test_probe_options_are_strict_and_do_not_accept_arbitrary_commands(self):
        for value in ({'dry_run':'false'},{'dry_run':0},{'command':'G28'}):
            with self.subTest(value=value):
                self.assertEqual(self.client.post('/api/bed-probe/start',json=value).status_code,422)
        for value in ({'stage':'surface','token':'short'}, {'stage':'home','token':'a'*24}):
            self.assertEqual(self.client.post('/api/bed-probe/confirm',json=value).status_code,422)

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
        preflight.assert_awaited_once_with('part.gcode',choices,None,None)

    def test_legacy_preflight_defaults_to_both_checks(self):
        with patch('bridge_dashboard.app.Workflow.preflight',new_callable=AsyncMock,
                   return_value={'state':'preflighting'}) as preflight:
            response=self.client.post('/api/jobs/part.gcode/preflight/both')
        self.assertEqual(response.status_code,200)
        preflight.assert_awaited_once_with('part.gcode',{'extruder':True,'heater_bed':True},None,None)

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
        start.assert_awaited_once_with('part.gcode',choices,None,None)

    def test_corner_options_pass_to_preflight_and_start(self):
        from klipper_extras.axisforge_corners import options
        chosen=options({'policy':'blend','blend_tolerance_mm':.04})
        for method,path,body in (
            ('preflight','/api/jobs/part.gcode/preflight/both',{'motion_options':chosen}),
            ('start','/api/sync/start',{'job':'part.gcode','motion_options':chosen})):
            with patch('bridge_dashboard.app.Workflow.'+method,new_callable=AsyncMock,
                       return_value={'state':'ready'}) as call:
                response=self.client.post(path,json=body)
            self.assertEqual(response.status_code,200,response.text)
            self.assertEqual(call.await_args.args[-2],chosen)

    def test_invalid_corner_options_fail_before_workflow(self):
        for value in ({'policy':'classic'},{'blend_tolerance_mm':True},
                      {'blend_speed_mm_s':'5'},{'square_corner_velocity':21},
                      {'junction_deviation_mm':-1},{'unknown':1}):
            with self.subTest(value=value):
                response=self.client.post('/api/jobs/part.gcode/preflight/both',json={'motion_options':value})
                self.assertEqual(response.status_code,422,response.text)

    def test_clock_limits_forwarded_to_preflight_and_start(self):
        chosen={'max_uncertainty_ms':10.,'max_alignment_error_ms':15.}
        for method,path,body in (
            ('preflight','/api/jobs/part.gcode/preflight/both',{'clock_options':chosen}),
            ('start','/api/sync/start',{'job':'part.gcode','clock_options':chosen})):
            with patch('bridge_dashboard.app.Workflow.'+method,new_callable=AsyncMock,
                       return_value={'state':'ready'}) as call:
                response=self.client.post(path,json=body)
            self.assertEqual(response.status_code,200,response.text)
            self.assertEqual(call.await_args.args[-1],chosen)

    def test_invalid_clock_limits_rejected_before_workflow(self):
        for value in ({'max_uncertainty_ms':True},{'max_alignment_error_ms':'15'},
                      {'max_alignment_error_ms':21},{'max_alignment_error_ms':5},
                      {'max_uncertainty_ms':1},{'max_alignment_error_ms':None},{'unknown':1}):
            for path in ('/api/jobs/part.gcode/preflight/both','/api/sync/start'):
                with self.subTest(value=value,path=path):
                    response=self.client.post(path,json={'clock_options':value,**({'job':'part.gcode'} if path.endswith('start') else {})})
                    self.assertEqual(response.status_code,422,response.text)

    def test_start_rejects_misspelled_clock_field(self):
        response=self.client.post('/api/sync/start',json={'job':'part.gcode','clock_settings':{'max_alignment_error_ms':15}})
        self.assertEqual(response.status_code,422)


if __name__ == "__main__":
    unittest.main()
