"""Independent, non-actuating UR driver supervision and read-only startup checks."""
import asyncio
from collections import deque
from contextlib import asynccontextmanager, suppress
import fcntl
import ipaddress
import math
import os
from pathlib import Path
import signal
import time

from fastapi import FastAPI, HTTPException, Request
import httpx
import yaml

from bridge_dashboard.config import Settings
from bridge_dashboard.rosbridge import RosBridge
from bridge_dashboard.sync_clients import RobotClient


class RobotConnectionError(RuntimeError): pass


class RobotConnectionClient:
    """Dashboard proxy; the driver survives dashboard refreshes/restarts."""
    def __init__(self, url):
        self.url = url
        self.client = httpx.AsyncClient(timeout=4., trust_env=False)

    async def status(self):
        if not self.url:
            return {'configured':False, 'available':False, 'busy':False, 'state':'unconfigured',
                    'message':'Configure the robot connection service to enable Connect robot.'}
        try:
            response = await self.client.get(self.url.rstrip('/')+'/status')
            response.raise_for_status()
            return dict(response.json(), available=True)
        except (httpx.HTTPError, ValueError):
            return {'configured':True, 'available':False, 'busy':False, 'state':'offline',
                    'message':'Robot connection service is unavailable; check the local dashboard stack.'}

    async def connect(self):
        if not self.url: raise RobotConnectionError('Robot connection service is not configured')
        try:
            response = await self.client.post(self.url.rstrip('/')+'/connect', json={}, timeout=15.)
            if response.is_error:
                raise RobotConnectionError(response.json().get('detail', 'Robot connection rejected'))
            return dict(response.json(), available=True)
        except (httpx.HTTPError, ValueError) as error:
            raise RobotConnectionError('Cannot contact the robot connection service: '+str(error)) from error

    async def close(self): await self.client.aclose()


class RobotConnection:
    """Launch only the configured driver script; never power, unlock, load or play."""
    def __init__(self, settings, ros, worker, *, spawn=asyncio.create_subprocess_exec,
                 clock=time.monotonic, lock_path=None):
        self.settings, self.ros, self.worker = settings, ros, worker
        self.spawn, self.clock = spawn, clock
        self.lock_path = Path(lock_path or os.getenv('AXISFORGE_ROBOT_LOCK', '.axisforge-robot-control.lock'))
        self._lock = asyncio.Lock()
        self.process = self.poll_task = self.log_task = self.connect_task = None
        self.logs = deque(maxlen=12)
        self._busy = False
        self._error = None
        self._robot_lock = None
        self._graph_known = False
        self._driver_present = False
        self._checked_at = None
        self._checks = []
        self._feedback = {}
        self.state, self.message = 'idle', 'Connect the robot driver to begin the arm checks.'

    def configuration(self):
        data = {'robot_ip': self.settings.robot_ip, 'robot_type': 'UR16e',
                'calibration_file': str(self.settings.robot_calibration_file or ''),
                'axisforge_tcp_mm': None, 'probing_rpy_deg': None, 'configuration_error': None}
        try:
            ipaddress.ip_address(self.settings.robot_ip)
            for path in (self.settings.robot_driver_script, self.settings.robot_calibration_file,
                         self.settings.axisforge_config):
                if path is None or not path.is_file(): raise ValueError('Driver, calibration and AxisForge config files are required')
            params = yaml.safe_load(self.settings.axisforge_config.read_text())['gcode_interpreter']['ros__parameters']
            tcp = params['tcp_offset_mm']
            rpy = [params.get('nozzle_roll_deg',180.), params.get('nozzle_pitch_deg',0.), params.get('wrist_angle',90.)]
            if len(tcp) != 3 or not all(type(v) in (int,float) and math.isfinite(v) for v in [*tcp,*rpy]):
                raise ValueError('Configured nozzle TCP/orientation must contain finite numbers')
            data.update(axisforge_tcp_mm=tcp, probing_rpy_deg=rpy)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            data['configuration_error'] = str(error)
        return data

    def status(self):
        cfg = self.configuration()
        return {'configured':not cfg['configuration_error'], 'busy':self._busy,
                'state':self.state, 'message':self.message,
                'driver_running':self._driver_present,
                'driver_owned':self.process is not None and self.process.returncode is None,
                'checks':self._checks, 'configuration':cfg, 'logs':list(self.logs),
                'checked_age_s':None if self._checked_at is None else self.clock()-self._checked_at,
                'ready':self.state=='ready' and self._checked_at is not None and self.clock()-self._checked_at < 10., 'arm_startup_controls':False}

    async def start(self):
        await self.ros.start()
        self.poll_task = asyncio.create_task(self._poll())

    async def _poll(self):
        while True:
            try: await self.refresh()
            except asyncio.CancelledError: raise
            except Exception as error:
                self.state, self.message = 'offline', 'Could not check the robot: '+str(error)
            await asyncio.sleep(2.)

    async def refresh(self):
        if self._lock.locked(): return
        async with self._lock:
            await self._refresh_locked()

    async def _refresh_locked(self):
        checks = []
        def check(name, passed, detail):
            checks.append({'name':name, 'status':'pass' if passed is True else 'fail' if passed is False else 'unknown',
                           'detail':detail})
        ros_ready = self.ros.status()['connected']
        check('ROS bridge', ros_ready, 'Connected' if ros_ready else 'Waiting for the local ROS bridge')
        self._graph_known, self._driver_present = False, False
        graph_error = ''
        if ros_ready:
            try:
                graph = await self.ros.call_service('/rosapi/nodes', 'rosapi_msgs/srv/Nodes')
                nodes = graph.get('nodes')
                if not isinstance(nodes, list) or not all(isinstance(n,str) for n in nodes):
                    raise RuntimeError('Invalid ROS node list')
                self._graph_known = True
                self._driver_present = any(n in nodes for n in ('/dashboard_client','/controller_manager','/ur_ros2_control_node'))
            except Exception as error: graph_error = type(error).__name__+': '+str(error)
        check('UR driver', self._driver_present if self._graph_known else None,
              ('Running in the connection service' if self.process is not None and self.process.returncode is None
               else 'Existing driver detected; no duplicate will be started') if self._driver_present else
              'Driver is not running' if self._graph_known else 'Cannot inspect the ROS graph: '+graph_error)
        try:
            worker = await self.worker.call('status')
            self._feedback = worker.get('feedback', {})
        except Exception:
            self._feedback = {}
        fresh = self._feedback.get('joints_fresh') is True
        check('Robot Ethernet / joint feedback', fresh,
              'Fresh robot joint positions' if fresh else 'Waiting for the robot at '+self.settings.robot_ip+'; check Ethernet and controller power')
        details = {}
        if self._driver_present and fresh:
            queries = {'remote':('is_in_remote_control','IsInRemoteControl'),
                       'mode':('get_robot_mode','GetRobotMode'),
                       'loaded':('get_loaded_program','GetLoadedProgram')}
            async def query(key, endpoint, name):
                try: details[key] = await self.ros.call_service('/dashboard_client/'+endpoint, 'ur_dashboard_msgs/srv/'+name)
                except Exception: details[key] = {}
            # UR dashboard commands share one driver-side TCP connection.
            for key, values in queries.items(): await query(key, *values)
        remote = details.get('remote',{}).get('remote_control')
        mode = details.get('mode',{}).get('robot_mode',{}).get('mode')
        check('Remote Control', remote, 'Remote selected' if remote is True else 'Select Remote in the pendant’s top-right menu')
        check('Arm power / brakes', mode==7 if mode is not None else None,
              {3:'Power on the arm using the pendant',4:'Arm is powering on',5:'Release brakes using the pendant',
               7:'Arm powered; brakes released'}.get(mode,'Awaiting the arm’s power state'))
        safety = self._feedback.get('safety_mode') if fresh else None
        check('Safety mode', safety==1 if safety is not None else None,
              'NORMAL' if safety==1 else 'Check the pendant for safety messages')
        program = self._feedback.get('robot_program_running') if fresh else None
        loaded = details.get('loaded',{}).get('program_name')
        check('External Control program', program,
              ('External Control running' if program else 'Start the saved External Control program using the pendant')+
              ('; loaded: '+str(loaded) if loaded else ''))
        speed = self._feedback.get('speed_scaling_percent') if fresh else None
        check('Speed slider', abs(speed-100.)<=.1 if type(speed) in (int,float) else None,
              ('%g%%; use 100%% for synchronized jobs' % speed) if type(speed) in (int,float) else 'Awaiting speed-scaling feedback')
        cfg = self.configuration()
        check('Saved calibration / nozzle TCP', not cfg['configuration_error'],
              cfg['configuration_error'] or 'Existing calibration and AxisForge nozzle offset loaded; arm settings are unchanged')
        self._checks, self._checked_at = checks, self.clock()
        if self._busy:
            self.state, self.message = 'connecting', 'Starting the UR driver; checking for feedback.'
        elif self.process is not None and self.process.returncode is not None and not self._driver_present:
            self.state, self.message = 'failed', 'UR driver exited (%s). Check the connection and press Connect robot to retry.' % self.process.returncode
        elif all(c['status']=='pass' for c in checks) and self._feedback.get('ready'):
            self.state, self.message = 'ready', 'Robot checks passed. Continue to Home & probe bed.'
        elif self._error:
            self.state, self.message = 'failed', self._error
        elif self._driver_present or (self.process is not None and self.process.returncode is None):
            self.state, self.message = 'waiting_robot', next((c['detail'] for c in checks if c['status']!='pass'),
                                                          self._feedback.get('reason','Checking robot readiness'))
        elif not self._graph_known:
            self.state, self.message = 'offline', 'Cannot inspect the local ROS graph; driver launch is blocked until discovery recovers.'
        else:
            self.state, self.message = 'idle', 'Connect robot to start the PC driver. Klipper can be offline during this step.'

    def _claim_robot(self):
        fd = os.open(self.lock_path, os.O_CREAT|os.O_RDWR, 0o600)
        try: fcntl.flock(fd, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            raise RobotConnectionError('The robot is owned by a job or probing routine')
        self._robot_lock = fd

    def _release_robot(self):
        if self._robot_lock is not None:
            os.close(self._robot_lock); self._robot_lock = None

    async def connect(self):
        async with self._lock:
            if self._busy: return self.status()
            cfg = self.configuration()
            if cfg['configuration_error']: raise RobotConnectionError(cfg['configuration_error'])
            self._claim_robot()
            try:
                await self._refresh_locked()
                if not self._graph_known:
                    raise RobotConnectionError('Cannot inspect ROS for an existing driver; restore ROS bridge before connecting')
                if self._driver_present or (self.process is not None and self.process.returncode is None):
                    return self.status()
                self._busy = True
                self.state, self.message = 'connecting', 'Starting the configured UR16e driver.'
                self.logs.clear()
                self._error = None
                self.connect_task = asyncio.create_task(self._launch())
                return self.status()
            finally:
                if not self._busy: self._release_robot()

    async def _launch(self):
        try:
            env = dict(os.environ, ROBOT_IP=self.settings.robot_ip,
                       UR_CALIBRATION_FILE=str(self.settings.robot_calibration_file))
            launch = asyncio.create_task(self.spawn('bash', str(self.settings.robot_driver_script),
                'headless_mode:=false', stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
                env=env, start_new_session=True))
            try: self.process = await asyncio.shield(launch)
            except asyncio.CancelledError:
                self.process = await launch
                raise
            async def logs():
                while True:
                    line = await self.process.stdout.readline()
                    if not line: return
                    self.logs.append(line.decode(errors='replace').rstrip()[-2000:])
            self.log_task = asyncio.create_task(logs())
            # Bound startup ownership; a disconnected robot must not freeze the UI.
            for _ in range(30):
                await asyncio.sleep(1.)
                await self.refresh()
                if self.process.returncode is not None or self._feedback.get('joints_fresh'): break
        except Exception as error:
            self._error = 'Could not start the UR driver: '+str(error)
            self.state, self.message = 'failed', self._error
        finally:
            self._busy = False
            self._release_robot()

    async def close(self):
        # This belongs to the separate connection service, not dashboard shutdown.
        if self.poll_task:
            self.poll_task.cancel()
            with suppress(asyncio.CancelledError): await self.poll_task
        if self.connect_task and not self.connect_task.done():
            self.connect_task.cancel()
            with suppress(asyncio.CancelledError): await self.connect_task
        if self.process is not None and self.process.returncode is None:
            with suppress(ProcessLookupError): os.killpg(self.process.pid, signal.SIGINT)
            try: await asyncio.wait_for(self.process.wait(), 8.)
            except asyncio.TimeoutError:
                with suppress(ProcessLookupError): os.killpg(self.process.pid, signal.SIGTERM)
                try: await asyncio.wait_for(self.process.wait(), 4.)
                except asyncio.TimeoutError:
                    with suppress(ProcessLookupError): os.killpg(self.process.pid, signal.SIGKILL)
                    await asyncio.wait_for(self.process.wait(), 2.)
        if self.log_task:
            self.log_task.cancel()
            with suppress(asyncio.CancelledError): await self.log_task
        self._release_robot()
        await self.worker.close()
        await self.ros.close()


def create_connection_app(settings=None):
    settings = settings or Settings.from_env()
    @asynccontextmanager
    async def lifespan(app):
        app.state.connection = RobotConnection(settings, RosBridge(settings.rosbridge_url), RobotClient(settings.robot_sync_url))
        await app.state.connection.start()
        try: yield
        finally: await app.state.connection.close()
    app = FastAPI(lifespan=lifespan)
    @app.get('/status')
    async def status(): return app.state.connection.status()
    @app.get('/health')
    async def health(): return {'available':True}
    @app.post('/connect')
    async def connect(request: Request):
        if request.headers.get('origin'): raise HTTPException(403,'Use the bridge dashboard to connect')
        if await request.json() != {}: raise HTTPException(400,'Connection settings are read from the configured files')
        try: return await app.state.connection.connect()
        except RobotConnectionError as error: raise HTTPException(409,str(error)) from error
    return app


app = create_connection_app()
