"""ROS 2 status and stop-topic client using rosbridge_suite's websocket protocol."""

import asyncio
import json
import logging
import time

import websockets

logger = logging.getLogger(__name__)


class RosBridge:
    """Maintain a reconnecting rosbridge connection and expose joint-state health."""

    def __init__(self, url):
        self.url = url
        self._socket = None
        self._task = None
        self._connected = False
        self._last_joint_state = None
        self._joint_names = []
        self._send_lock = asyncio.Lock()
        self._service_requests = {}

    async def start(self):
        self._task = asyncio.create_task(self._run())

    async def close(self):
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        self._socket = None
        self._connected = False

    async def _run(self):
        while True:
            try:
                async with websockets.connect(
                    self.url, open_timeout=3, ping_interval=20, ping_timeout=20
                ) as socket:
                    self._socket = socket
                    self._connected = True
                    await socket.send(json.dumps({
                        "op": "subscribe",
                        "topic": "/joint_states",
                        "type": "sensor_msgs/JointState",
                        "throttle_rate": 250,
                        "queue_length": 1,
                    }))
                    async for raw_message in socket:
                        self._handle_message(json.loads(raw_message))
            except asyncio.CancelledError:
                raise
            except Exception as error:
                logger.warning("ROS bridge disconnected: %s", error)
            finally:
                self._socket = None
                self._connected = False
            await asyncio.sleep(2)

    def _handle_message(self, message):
        if message.get('op') == 'service_response':
            future = self._service_requests.get(message.get('id'))
            if future is not None and not future.done(): future.set_result(message)
            return
        if message.get("topic") != "/joint_states":
            return
        msg = message.get("msg", {})
        names = msg.get("name", [])
        positions = msg.get("position", [])
        if len(names) != len(positions):
            return
        self._joint_names = list(names)
        self._last_joint_state = time.monotonic()

    def status(self):
        age = None
        if self._last_joint_state is not None:
            age = max(0.0, time.monotonic() - self._last_joint_state)
        return {
            "connected": self._connected,
            "joint_state_fresh": age is not None and age < 1.5,
            "joint_state_age_s": round(age, 2) if age is not None else None,
            "joint_names": self._joint_names,
        }

    async def request_axisforge_stop(self):
        if not self._connected or self._socket is None:
            raise RuntimeError("ROS bridge is not connected; robot stop was not sent")
        message = {
            "op": "publish",
            "topic": "/kb/shutdown",
            "type": "std_msgs/Bool",
            "msg": {"data": True},
        }
        async with self._send_lock:
            await self._socket.send(json.dumps(message))

    async def request_robot_program_stop(self):
        """Independent stop path, including when the robot worker process is lost."""
        if not self._connected or self._socket is None:
            raise RuntimeError('ROS disconnected; UR program stop could not be sent')
        import uuid
        request_id = 'sync-stop-'+uuid.uuid4().hex
        future = asyncio.get_running_loop().create_future()
        self._service_requests[request_id] = future
        try:
            async with self._send_lock:
                await self._socket.send(json.dumps({'op':'call_service','id':request_id,
                    'service':'/dashboard_client/stop','type':'std_srvs/srv/Trigger','args':{}}))
            response = await asyncio.wait_for(future,3.)
            if not response.get('result') or not response.get('values',{}).get('success'):
                raise RuntimeError('UR dashboard did not acknowledge program stop')
            return {'stopped':True}
        finally:
            self._service_requests.pop(request_id,None)
