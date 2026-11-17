"""Driver startup and dashboard reconnection never actuate the arm."""
import asyncio
import json
import fcntl
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock

from bridge_dashboard.robot_connection import RobotConnection, RobotConnectionError
from bridge_dashboard.rosbridge import RosBridge


class ConnectionTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        root=Path(self.temp.name)
        cfg=root/'config.yaml';cfg.write_text('gcode_interpreter:\n  ros__parameters:\n    tcp_offset_mm: [-100, 0, 117.06]\n')
        script=root/'driver.sh';script.write_text('# test only\n')
        calibration=root/'calibration.yaml';calibration.write_text('test: true\n')
        self.settings=SimpleNamespace(robot_ip='192.168.0.100',axisforge_config=cfg,
            robot_driver_script=script,robot_calibration_file=calibration)
        self.nodes=[];self.fresh=False;self.calls=[]
        async def service(path,kind):
            self.calls.append(path)
            return {'/rosapi/nodes':{'nodes':self.nodes},
                    '/dashboard_client/is_in_remote_control':{'remote_control':True},
                    '/dashboard_client/get_robot_mode':{'robot_mode':{'mode':7}},
                    '/dashboard_client/get_loaded_program':{'program_name':'external.urp'}}[path]
        self.ros=SimpleNamespace(status=lambda:{'connected':True},call_service=service,close=AsyncMock())
        async def status(*args):
            return {'feedback':{'joints_fresh':self.fresh,'ready':self.fresh,'safety_mode':1,
                'robot_program_running':True,'speed_scaling_percent':100.}}
        self.worker=SimpleNamespace(call=status,close=AsyncMock())
        self.spawn=AsyncMock(side_effect=OSError('fake launch failure'))
        self.manager=RobotConnection(self.settings,self.ros,self.worker,spawn=self.spawn,lock_path=root/'lock')

    async def asyncTearDown(self):await self.manager.close()

    async def test_existing_driver_is_adopted_and_no_actuating_service_is_called(self):
        self.nodes=['/dashboard_client'];self.fresh=True
        result=await self.manager.connect()
        self.assertTrue(result['ready']);self.spawn.assert_not_awaited()
        self.assertFalse(result['arm_startup_controls'])
        self.assertEqual(set(self.calls),{'/rosapi/nodes','/dashboard_client/is_in_remote_control',
                         '/dashboard_client/get_robot_mode','/dashboard_client/get_loaded_program'})

    async def test_offline_hardware_still_allows_fixed_driver_launch_and_failure_persists(self):
        result=await self.manager.connect();self.assertTrue(result['busy'])
        await self.manager.connect_task
        args,kwargs=self.spawn.call_args
        self.assertEqual(args,('bash',str(self.settings.robot_driver_script),'headless_mode:=false'))
        self.assertEqual(kwargs['env']['ROBOT_IP'],'192.168.0.100')
        await self.manager.refresh()
        self.assertEqual(self.manager.status()['state'],'failed')
        self.assertIn('fake launch failure',self.manager.status()['message'])
        self.assertFalse(self.manager.status()['busy'])

    async def test_unknown_graph_and_robot_ownership_prevent_duplicate_start(self):
        with open(self.manager.lock_path,'w') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaisesRegex(RobotConnectionError,'owned'):await self.manager.connect()
        self.ros.call_service=AsyncMock(side_effect=TimeoutError())
        with self.assertRaisesRegex(RobotConnectionError,'Cannot inspect ROS'):await self.manager.connect()
        self.spawn.assert_not_awaited()

    async def test_missing_configuration_and_invalid_ip_do_not_launch(self):
        self.settings.robot_ip='$(unsafe)'
        with self.assertRaises(RobotConnectionError):await self.manager.connect()
        self.spawn.assert_not_awaited()

    async def test_disconnected_dashboard_reconnects_without_starting_arm(self):
        self.nodes=['/dashboard_client'];self.fresh=True
        original=self.ros.call_service;connected=False
        async def service(path,kind):
            nonlocal connected
            if path=='/dashboard_client/connect':
                self.calls.append(path);connected=True;return {'success':True}
            if path=='/dashboard_client/is_in_remote_control' and not connected:
                return {'success':False,'remote_control':False,
                        'answer':'Failed to send request to dashboard server. Are you connected to the Dashboard Server?'}
            return await original(path,kind)
        self.ros.call_service=service
        await self.manager.refresh()
        self.assertTrue(self.manager.status()['ready'])
        self.assertEqual(self.calls.count('/dashboard_client/connect'),1)
        self.assertFalse(any(path.endswith(('/play','/power_on','/brake_release')) for path in self.calls))

    async def test_failed_reconnection_is_rate_limited_and_shown_as_connection_fault(self):
        self.nodes=['/dashboard_client'];self.fresh=True
        original=self.ros.call_service
        async def service(path,kind):
            if path=='/dashboard_client/connect':
                self.calls.append(path);return {'success':False}
            if path=='/dashboard_client/is_in_remote_control':
                return {'success':False,'remote_control':False,'answer':'Not connected'}
            return await original(path,kind)
        self.ros.call_service=service;self.manager.clock=lambda:100.
        await self.manager.refresh();await self.manager.refresh()
        self.assertEqual(self.calls.count('/dashboard_client/connect'),1)
        self.assertFalse(self.manager.status()['ready'])
        self.assertIn('dashboard connection',self.manager.status()['message'])
        self.manager.clock=lambda:111.
        await self.manager.refresh()
        self.assertEqual(self.calls.count('/dashboard_client/connect'),2)

    async def test_local_mode_does_not_trigger_reconnect_or_play(self):
        self.nodes=['/dashboard_client'];self.fresh=True
        original=self.ros.call_service
        async def service(path,kind):
            if path=='/dashboard_client/is_in_remote_control':
                return {'success':True,'remote_control':False}
            return await original(path,kind)
        self.ros.call_service=service
        await self.manager.refresh()
        self.assertNotIn('/dashboard_client/connect',self.calls)
        self.assertIn('Select Remote',self.manager.status()['message'])

    async def test_real_ros_rpc_recovers_local_to_remote_disconnect_without_actuation(self):
        self.nodes=['/dashboard_client'];self.fresh=True
        ros=RosBridge('ws://simulated');ros._connected=True
        self.manager.ros=ros
        connected=True;remote=False
        async def send(payload):
            nonlocal connected
            request=json.loads(payload);path=request['service'];self.calls.append(path)
            if path=='/rosapi/nodes':values={'nodes':self.nodes}
            elif path=='/dashboard_client/connect':connected=True;values={'success':True}
            elif not connected:values={'success':False,'remote_control':False,'answer':'Failed to send request to dashboard server. Are you connected to the Dashboard Server?'}
            elif path=='/dashboard_client/is_in_remote_control':values={'success':True,'remote_control':remote}
            elif path=='/dashboard_client/get_robot_mode':values={'success':True,'robot_mode':{'mode':7}}
            elif path=='/dashboard_client/get_loaded_program':values={'success':True,'program_name':'external.urp'}
            else:raise AssertionError('Unexpected service '+path)
            ros._handle_message(dict(op='service_response',id=request['id'],result=True,values=values))
        ros._socket=SimpleNamespace(send=send)
        await self.manager.refresh()
        self.assertFalse(self.manager.status()['ready'])
        self.assertIn('Select Remote',self.manager.status()['message'])
        self.assertNotIn('/dashboard_client/connect',self.calls)
        connected=False;remote=True
        await self.manager.refresh()
        self.assertTrue(self.manager.status()['ready'])
        self.assertEqual(self.calls.count('/dashboard_client/connect'),1)
        self.assertFalse(any(path.endswith(('/play','/power_on','/brake_release','/stop')) for path in self.calls))
