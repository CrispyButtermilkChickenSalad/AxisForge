"""FastAPI application for the local AxisForge/Klipper dashboard."""

import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field, StrictBool, model_validator

from bridge_dashboard.axisforge import AxisForgeError, AxisForgeRunner
from bridge_dashboard.config import Settings
from bridge_dashboard.moonraker import MoonrakerClient, MoonrakerError
from bridge_dashboard.rosbridge import RosBridge
from bridge_dashboard.sync_clients import MasterClient, RobotClient, ParticipantError
from bridge_dashboard.workflow import Workflow
from bridge_dashboard.bed_probe import BedProbeRunner, BedProbeError

from bridge_dashboard.load_calibration import LoadCalibrationRunner
from bridge_dashboard.robot_connection import RobotConnectionClient, RobotConnectionError
from bridge_dashboard.tcp_settings import TcpSettings, TcpSettingsError, TcpSettingsConflict
from bridge_dashboard.probing_settings import ProbingSettings, ProbingSettingsError, ProbingSettingsConflict
from bridge_dashboard.bed_settings import BedSettings, BedSettingsError, BedSettingsConflict

logger = logging.getLogger(__name__)
STATIC_DIR = Path(__file__).parent / "static"


class HeaterChecks(BaseModel):
    model_config = ConfigDict(extra='forbid')
    extruder: StrictBool = True
    heater_bed: StrictBool = True


class MotionOptions(BaseModel):
    model_config = ConfigDict(extra='forbid',strict=True)
    policy: Literal['exact','klipper','marlin','blend'] = 'exact'
    square_corner_velocity: float = Field(default=1.,ge=0,le=20,allow_inf_nan=False)
    junction_deviation_mm: float = Field(default=.01,ge=0,le=1,allow_inf_nan=False)
    blend_tolerance_mm: float = Field(default=.05,ge=0,le=.2,allow_inf_nan=False)
    blend_speed_mm_s: float = Field(default=5.,ge=0,le=20,allow_inf_nan=False)


class ClockOptions(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    max_uncertainty_ms: float = Field(default=10., ge=2, le=20, allow_inf_nan=False)
    max_alignment_error_ms: float = Field(default=10., ge=2, le=20, allow_inf_nan=False)

    @model_validator(mode='after')
    def ordered_limits(self):
        if self.max_uncertainty_ms > self.max_alignment_error_ms:
            raise ValueError('Clock uncertainty limit must not exceed the alignment limit')
        return self


class PreflightRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    heater_checks: HeaterChecks = Field(default_factory=HeaterChecks)
    motion_options: MotionOptions | None = None
    clock_options: ClockOptions | None = None


class StartJobRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    job: str
    dry_run: bool = False
    heater_checks: HeaterChecks | None = None
    motion_options: MotionOptions | None = None
    clock_options: ClockOptions | None = None


class BedProbeStartRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    dry_run: StrictBool = False


class BedProbeConfirmation(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    stage: Literal['surface', 'front_left']
    token: str = Field(min_length=16, max_length=128)


class LoadCalibrationConfirmation(BedProbeConfirmation):
    stage: Literal['home', 'scale', 'tare']


class LoadCalibrationAction(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    command: Literal['step', 'finish']
    token: str = Field(min_length=16, max_length=128)
    delta_mm: float | None = Field(default=None, ge=-10., le=10., allow_inf_nan=False)

    @model_validator(mode='after')
    def supported_step(self):
        if self.command == 'step' and (self.delta_mm is None or abs(self.delta_mm) not in (.001, .01, .05, .1, .5, 1., 10.)):
            raise ValueError('Choose a supported Z step size')
        if self.command == 'finish' and self.delta_mm is not None:
            raise ValueError('Finish does not accept a movement distance')
        return self


class TcpSettingsRequest(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    x_mm: float = Field(ge=-1000, le=1000, allow_inf_nan=False)
    y_mm: float = Field(ge=-1000, le=1000, allow_inf_nan=False)
    z_mm: float = Field(ge=-1000, le=1000, allow_inf_nan=False)
    revision: str = Field(pattern=r'^[0-9a-f]{64}$')


class ProbingValues(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    first_speed_mm_s: float = Field(ge=.5, le=10., allow_inf_nan=False)
    second_speed_mm_s: float = Field(ge=.02, le=1., allow_inf_nan=False)
    lift_speed_mm_s: float = Field(ge=1., le=20., allow_inf_nan=False)
    mesh_columns: int = Field(ge=2, le=9)
    mesh_rows: int = Field(ge=2, le=9)
    mesh_speed_mm_s: float = Field(ge=.02, le=.2, allow_inf_nan=False)
    mesh_samples: int = Field(ge=3, le=5)
    mesh_repeatability_mm: float = Field(ge=.005, le=.05, allow_inf_nan=False)

    @model_validator(mode='after')
    def slower_precision_pass(self):
        if self.second_speed_mm_s >= self.first_speed_mm_s:
            raise ValueError('Precision rehoming must be slower than the initial surface search')
        return self


class ProbingSettingsRequest(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    settings: ProbingValues
    revision: str = Field(pattern=r'^[0-9a-f]{64}$')


class BedSettingsRequest(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True)
    width_mm: float = Field(ge=10., le=2000., allow_inf_nan=False)
    depth_mm: float = Field(ge=10., le=2000., allow_inf_nan=False)
    probe_margin_mm: float = Field(ge=0., allow_inf_nan=False)
    revision: str = Field(pattern=r'^[0-9a-f]{64}$')

    @model_validator(mode='after')
    def positive_probe_area(self):
        if 2*self.probe_margin_mm >= min(self.width_mm, self.depth_mm):
            raise ValueError('Probe margin must be less than half the smaller bed dimension')
        return self


class CopyJobRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    job: str


def create_app(settings=None):
    settings = settings or Settings.from_env()

    @asynccontextmanager
    async def lifespan(app):
        app.state.moonraker = MoonrakerClient(
            settings.moonraker_url, settings.moonraker_api_key
        )
        app.state.rosbridge = RosBridge(settings.rosbridge_url)
        app.state.axisforge = AxisForgeRunner(settings)
        app.state.workflow = Workflow(MasterClient(app.state.moonraker),
            RobotClient(settings.robot_sync_url), app.state.axisforge,
            heat_timeout=settings.heater_check_timeout_s,
            robot_stop=app.state.rosbridge.request_robot_program_stop)
        app.state.bed_probe = BedProbeRunner(settings,
            robot_stop=app.state.rosbridge.request_robot_program_stop,
            invalidate_preflight=app.state.workflow.invalidate_preflight)
        app.state.load_calibration = LoadCalibrationRunner(settings,
            robot_stop=app.state.rosbridge.request_robot_program_stop,
            invalidate_preflight=app.state.workflow.invalidate_preflight)
        app.state.robot_connection = RobotConnectionClient(settings.robot_connection_url)
        app.state.tcp_settings = TcpSettings(settings.axisforge_config)
        app.state.probing_settings = ProbingSettings(settings.axisforge_config)
        app.state.bed_settings = BedSettings(settings.axisforge_config)
        try:
            await app.state.rosbridge.start()
            yield
        finally:
            try:
                await app.state.bed_probe.close()
            except BedProbeError:
                logger.exception('Could not confirm bed-probe stop during dashboard shutdown')
            try: await app.state.load_calibration.close()
            except BedProbeError: logger.exception('Could not confirm scale calibration stop')
            await app.state.robot_connection.close()
            await app.state.workflow.close()
            try:
                await app.state.axisforge.request_stop(app.state.rosbridge)
            except AxisForgeError:
                logger.exception("Could not request AxisForge stop during dashboard shutdown")
            await app.state.rosbridge.close()
            await app.state.moonraker.close()

    app = FastAPI(title="Klipper-ROS 2 Bridge", lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
    upload_lock = asyncio.Lock()
    operation_lock = asyncio.Lock()

    async def require_probe_idle():
        if app.state.load_calibration.busy():
            raise HTTPException(409, "ALPS calibration owns the robot; finish or stop it first")
        await require_connection_idle()
        if app.state.bed_probe.busy():
            raise HTTPException(status_code=409, detail='Bed homing/probing owns the robot; finish or stop it first')

    async def require_connection_idle():
        connection = await app.state.robot_connection.status()
        if connection.get('busy'):
            raise HTTPException(409, 'Wait for the PC robot driver to finish starting')

    def require_jobs_idle():
        if app.state.workflow.busy():
            raise HTTPException(409, 'Wait for the synchronized job or preflight to finish')
        standalone = app.state.axisforge.status()
        if standalone.get('running') or standalone.get('planning_check_running'):
            raise HTTPException(409, 'Wait for the standalone AxisForge operation to finish')

    @app.get('/api/setup/tcp')
    async def tcp_settings():
        try: return app.state.tcp_settings.status()
        except TcpSettingsError as error: raise HTTPException(503,str(error)) from error

    @app.post('/api/setup/tcp')
    async def save_tcp_settings(request: TcpSettingsRequest):
        async with operation_lock:
            require_jobs_idle()
            await require_probe_idle()
            try:
                return app.state.tcp_settings.save([request.x_mm,request.y_mm,request.z_mm],
                    request.revision,app.state.workflow.invalidate_preflight)
            except TcpSettingsConflict as error: raise HTTPException(409,str(error)) from error
            except TcpSettingsError as error: raise HTTPException(400,str(error)) from error

    @app.get('/api/setup/probing')
    async def probing_settings():
        try:
            return app.state.probing_settings.status()
        except ProbingSettingsError as error:
            raise HTTPException(503, str(error)) from error

    @app.get('/api/setup/bed')
    async def bed_settings():
        try:
            return app.state.bed_settings.status()
        except BedSettingsError as error:
            raise HTTPException(503, str(error)) from error

    @app.post('/api/setup/bed')
    async def save_bed_settings(request: BedSettingsRequest):
        async with operation_lock:
            require_jobs_idle()
            await require_probe_idle()
            try:
                return app.state.bed_settings.save(request.width_mm, request.depth_mm,
                    request.probe_margin_mm, request.revision, app.state.workflow.invalidate_preflight)
            except BedSettingsConflict as error:
                raise HTTPException(409, str(error)) from error
            except BedSettingsError as error:
                raise HTTPException(400, str(error)) from error

    @app.post('/api/setup/probing')
    async def save_probing_settings(request: ProbingSettingsRequest):
        async with operation_lock:
            require_jobs_idle()
            await require_probe_idle()
            try:
                return app.state.probing_settings.save(request.settings.model_dump(), request.revision,
                                                       app.state.workflow.invalidate_preflight)
            except ProbingSettingsConflict as error:
                raise HTTPException(409, str(error)) from error
            except ProbingSettingsError as error:
                raise HTTPException(400, str(error)) from error

    @app.get('/api/robot/connection')
    async def robot_connection_status():
        return await app.state.robot_connection.status()

    @app.post('/api/robot/connect')
    async def connect_robot():
        async with operation_lock:
            require_jobs_idle()
            if app.state.bed_probe.busy() or app.state.load_calibration.busy():
                raise HTTPException(409, 'Finish or stop the setup routine before connecting')
            app.state.workflow.invalidate_preflight('Robot connection requested; run preflight again')
            try: return await app.state.robot_connection.connect()
            except RobotConnectionError as error: raise HTTPException(409,str(error)) from error

    @app.get('/api/load-calibration/status')
    async def load_calibration_status(): return app.state.load_calibration.status()

    @app.post('/api/load-calibration/start')
    async def start_load_calibration(request: BedProbeStartRequest | None = None):
        async with operation_lock:
            await require_probe_idle()
            require_jobs_idle()
            try: return await app.state.load_calibration.start(dry_run=(request or BedProbeStartRequest()).dry_run)
            except BedProbeError as error: raise HTTPException(409,str(error)) from error

    @app.post('/api/load-calibration/confirm')
    async def confirm_load_calibration(request: LoadCalibrationConfirmation):
        try: return await app.state.load_calibration.confirm(request.stage,request.token)
        except BedProbeError as error: raise HTTPException(409,str(error)) from error

    @app.post('/api/load-calibration/action')
    async def load_calibration_action(request: LoadCalibrationAction):
        try: return await app.state.load_calibration.action(request.command,request.token,request.delta_mm)
        except BedProbeError as error: raise HTTPException(409,str(error)) from error

    @app.post('/api/load-calibration/abort')
    async def abort_load_calibration():
        try: return await app.state.load_calibration.abort('Operator stopped scale calibration')
        except BedProbeError as error: raise HTTPException(503,str(error)) from error

    @app.get('/api/bed-probe/status')
    async def bed_probe_status():
        return app.state.bed_probe.status()

    @app.post('/api/bed-probe/start')
    async def start_bed_probe(request: BedProbeStartRequest | None = None):
        async with operation_lock:
            await require_probe_idle()
            if app.state.workflow.busy():
                raise HTTPException(status_code=409, detail='Wait for the synchronized job or preflight to finish')
            standalone = app.state.axisforge.status()
            if standalone.get('running') or standalone.get('planning_check_running'):
                raise HTTPException(status_code=409, detail='Wait for the standalone AxisForge operation to finish')
            try:
                return await app.state.bed_probe.start(dry_run=(request or BedProbeStartRequest()).dry_run)
            except BedProbeError as error:
                raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post('/api/bed-probe/confirm')
    async def confirm_bed_probe(request: BedProbeConfirmation):
        try:
            return await app.state.bed_probe.confirm(request.stage, request.token)
        except BedProbeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post('/api/bed-probe/abort')
    async def abort_bed_probe():
        try:
            return await app.state.bed_probe.abort()
        except BedProbeError as error:
            raise HTTPException(status_code=503, detail=str(error)) from error

    @app.get("/")
    async def index():
        return FileResponse(STATIC_DIR / "index.html")

    @app.get("/api/status")
    async def status():
        return {
            "klipper": await app.state.moonraker.status(),
            "ros": app.state.rosbridge.status(),
            "axisforge": app.state.axisforge.status(),
        }

    @app.get("/api/klipper/telemetry")
    async def klipper_telemetry():
        try:
            return await app.state.moonraker.telemetry()
        except MoonrakerError as error:
            raise HTTPException(status_code=503, detail=str(error)) from error

    @app.get("/api/jobs")
    async def jobs():
        return app.state.axisforge.list_jobs()

    @app.get("/api/jobs/ready")
    async def ready_jobs():
        return {"jobs": app.state.workflow.ready_jobs(app.state.axisforge.list_jobs()["jobs"])}

    @app.get("/api/jobs/{job}/preflight")
    async def job_preflight(job: str):
        try:
            return app.state.workflow.view(job)
        except (AxisForgeError, OSError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.post("/api/jobs/{job}/preflight/{target}")
    async def run_job_preflight(job: str, target: Literal["both", "klipper", "robot"],
                               request: PreflightRequest | None = None):
        async with operation_lock:
            await require_probe_idle()
            try:
                # All entry points use one leased sequence; heater-off cannot be bypassed.
                return await app.state.workflow.preflight(job,
                    (request or PreflightRequest()).heater_checks.model_dump(),
                    request.motion_options.model_dump() if request and request.motion_options else None,
                    request.clock_options.model_dump() if request and request.clock_options else None)
            except (ParticipantError, AxisForgeError, OSError) as error:
                raise HTTPException(status_code=400, detail=str(error)) from error

    @app.post("/api/jobs/upload")
    async def upload_job(file: UploadFile = File(...)):
        if not file.filename:
            raise HTTPException(status_code=400, detail="Choose a .gcode file")
        async with upload_lock, operation_lock:
            await require_probe_idle()
            require_jobs_idle()
            try:
                staged_path = app.state.axisforge.stage_upload(file.filename, file.file)
            except AxisForgeError as error:
                raise HTTPException(status_code=400, detail=str(error)) from error
            return await forward_staged_job(staged_path, file.filename)

    @app.post('/api/jobs/copy-to-klipper')
    async def copy_job_to_klipper(request: CopyJobRequest):
        async with upload_lock, operation_lock:
            require_jobs_idle()
            await require_probe_idle()
            try:
                original = app.state.axisforge.job_path(request.job)
                with original.open('rb') as source:
                    staged = app.state.axisforge.stage_upload(original.name, source)
            except (AxisForgeError, OSError) as error:
                raise HTTPException(400, str(error)) from error
            return await forward_staged_job(staged, original.name)

    async def forward_staged_job(staged_path, original_name):
        try:
            await app.state.moonraker.upload_gcode(staged_path.name, staged_path)
        except MoonrakerError as error:
            try:
                staged_path.unlink()
            except OSError as cleanup_error:
                logger.exception('Could not remove locally staged file after failed Moonraker upload')
                raise HTTPException(503, f'{error}; local staged file remains at {staged_path.name}') from cleanup_error
            raise HTTPException(503, str(error)) from error
        return {'job': staged_path.name, 'original_name': original_name,
                'uploaded': True, 'ready': False,
                'message': 'Saved as a new version on both hosts; file contents are unchanged. '
                           'Run both preflights; copying does not certify Klipper command compatibility.'}

    @app.get("/api/links")
    async def links():
        return {
            "fluidd": settings.fluidd_url,
            "mainsail": settings.mainsail_url,
            "printer_ui": settings.printer_ui_url,
            "alps_logger": settings.alps_logger_url,
        }

    @app.post("/api/axisforge/start")
    async def start_axisforge(request: StartJobRequest):
        async with operation_lock:
            await require_probe_idle()
            return await start_standalone(request)

    async def start_standalone(request):
        if app.state.workflow.busy():
            raise HTTPException(status_code=409,detail='The synchronized workflow owns the robot')
        if not request.dry_run:
            raise HTTPException(
                status_code=409,
                detail=(
                    "Standalone live execution is disabled; use synchronized Start after preflight"
                ),
            )
        ros_status = app.state.rosbridge.status()
        if not ros_status["connected"]:
            raise HTTPException(status_code=503, detail="ROS bridge is not connected")
        try:
            return await app.state.axisforge.start(request.job, request.dry_run)
        except AxisForgeError as error:
            raise HTTPException(status_code=400, detail=str(error)) from error

    @app.get("/api/sync/status")
    async def sync_status():
        await app.state.workflow.refresh_capabilities()
        return app.state.workflow.status()

    @app.post("/api/sync/{action}")
    async def sync_action(action: Literal["arm", "start", "disarm", "abort"],
                          request: StartJobRequest | None = None):
        sync = app.state.workflow
        try:
            if action == "disarm":
                raise HTTPException(status_code=409,detail='Arming is replaced by the preflight/start workflow')
            if action == "abort":
                if app.state.load_calibration.busy(): return await app.state.load_calibration.abort()
                if app.state.bed_probe.busy(): return await app.state.bed_probe.abort()
                return await sync.abort()
            if request is None:
                raise HTTPException(status_code=400, detail="job is required")
            if action == 'arm':
                raise HTTPException(status_code=409,detail='Choose Start after both preflights pass')
            async with operation_lock:
                await require_probe_idle()
                return await sync.start(request.job,
                    request.heater_checks.model_dump() if request.heater_checks is not None else None,
                    request.motion_options.model_dump() if request.motion_options is not None else None,
                    request.clock_options.model_dump() if request.clock_options is not None else None)
        except (ParticipantError, AxisForgeError, BedProbeError, OSError) as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/axisforge/stop")
    async def stop_axisforge():
        try:
            if app.state.load_calibration.busy(): return await app.state.load_calibration.abort()
            if app.state.bed_probe.busy(): return await app.state.bed_probe.abort()
            if app.state.workflow.busy():
                return await app.state.workflow.abort()
            return await app.state.axisforge.request_stop(app.state.rosbridge)
        except (AxisForgeError, BedProbeError) as error:
            raise HTTPException(status_code=503, detail=str(error)) from error

    @app.post("/api/klipper/emergency-stop")
    async def klipper_emergency_stop():
        if app.state.workflow.busy():
            await app.state.workflow.abort('Emergency stop')
        async def robot_stop():
            routine = app.state.load_calibration if app.state.load_calibration.busy() else app.state.bed_probe
            if routine.busy():
                # Stop the UR program independently while the probe cancels its action.
                outcomes = await asyncio.gather(app.state.rosbridge.request_robot_program_stop(),
                                               routine.abort('Emergency stop'),
                                               return_exceptions=True)
                failures = [str(result) for result in outcomes if isinstance(result, Exception)]
                if failures: raise BedProbeError('; '.join(failures))
                return {'message': 'Bed probing and the UR program stopped'}
            return await app.state.axisforge.request_stop(app.state.rosbridge)
        results = await asyncio.gather(
            app.state.moonraker.emergency_stop(),
            robot_stop(),
            return_exceptions=True,
        )
        klipper, robot = results
        failures = {}
        if isinstance(klipper, Exception):
            failures["klipper"] = str(klipper)
        if isinstance(robot, Exception):
            failures["robot"] = str(robot)
        if failures:
            raise HTTPException(
                status_code=503,
                detail={
                    "message": "One or more stop requests failed",
                    "results": {
                        "klipper": "accepted" if not isinstance(klipper, Exception) else failures["klipper"],
                        "robot": robot.get("message", "accepted") if not isinstance(robot, Exception) else failures["robot"],
                    },
                },
            )
        return {
            "klipper": "Emergency stop accepted",
            "robot": robot["message"],
        }

    return app


app = create_app()
