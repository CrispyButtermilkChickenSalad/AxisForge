import asyncio,json
import unittest
from types import SimpleNamespace
from bridge_dashboard.rosbridge import RosBridge, RosServiceError


class ReadOnlyRpcTests(unittest.IsolatedAsyncioTestCase):
    async def test_stale_local_dashboard_permissions_reconnect_once_then_retry_stop(self):
        for remote, retry_ok in ((True, True), (False, False), (True, False)):
            with self.subTest(remote=remote,retry_ok=retry_ok):
                ros=RosBridge('ws://unused');ros._connected=True;calls=[]
                async def send(payload):
                    request=json.loads(payload);path=request['service'];calls.append(path)
                    if path.endswith('/stop'):
                        ok=retry_ok and calls.count(path)==2
                        values={'success':ok,'message':'Stopped' if ok else
                            'Command is not allowed due to safety reasons, please switch robot to Remote Control mode and reconnect to port 29999'}
                    elif path.endswith('/is_in_remote_control'):
                        values={'success':True,'remote_control':remote}
                    elif path.endswith(('/connect','/quit')):values={'success':True}
                    else:raise AssertionError(path)
                    ros._handle_message(dict(op='service_response',id=request['id'],result=True,values=values))
                ros._socket=SimpleNamespace(send=send)
                if remote and retry_ok:
                    self.assertEqual(await ros.request_robot_program_stop(),{'stopped':True})
                else:
                    with self.assertRaisesRegex(RuntimeError,'did not acknowledge'):
                        await ros.request_robot_program_stop()
                self.assertEqual(calls.count('/dashboard_client/connect'),int(remote))
                self.assertEqual(calls.count('/dashboard_client/quit'),int(remote))
                if remote:
                    self.assertLess(calls.index('/dashboard_client/quit'),calls.index('/dashboard_client/connect'))
                self.assertEqual(calls.count('/dashboard_client/stop'),1+int(remote))
                self.assertFalse(ros._service_requests)

    async def test_program_stop_preserves_controller_rejection_detail(self):
        ros=RosBridge('ws://unused');ros._connected=True
        async def send(payload):
            request=json.loads(payload)
            ros._handle_message(dict(op='service_response',id=request['id'],result=True,
                values={'success':False,'message':'Not connected to dashboard server'}))
        ros._socket=SimpleNamespace(send=send)
        with self.assertRaisesRegex(RuntimeError,'Not connected to dashboard server'):
            await ros.request_robot_program_stop()
        self.assertFalse(ros._service_requests)

    async def test_failed_reply_remains_an_error_and_preserves_disconnect_reason(self):
        ros=RosBridge('ws://unused');ros._connected=True
        values={'success':False,'remote_control':False,'answer':'Not connected'}
        async def send(payload):
            request=json.loads(payload)
            ros._handle_message(dict(op='service_response',id=request['id'],result=True,values=values))
        ros._socket=SimpleNamespace(send=send)
        with self.assertRaises(RosServiceError) as caught:
            await ros.call_service('/dashboard_client/is_in_remote_control','ur_dashboard_msgs/srv/IsInRemoteControl')
        self.assertEqual(caught.exception.values,values)
        self.assertIn('Not connected',str(caught.exception))
        self.assertFalse(ros._service_requests)

    async def test_server_deadline_and_response_correlation(self):
        ros=RosBridge('ws://unused');ros._connected=True;sent=[]
        async def send(payload):
            data=json.loads(payload);sent.append(data)
            ros._handle_message(dict(op='service_response',id='unrelated',result=True,values={}))
            ros._handle_message(dict(op='service_response',id=data['id'],result=True,values={'nodes':['/dashboard_client']}))
        ros._socket=SimpleNamespace(send=send)
        self.assertEqual(await ros.call_service('/rosapi/nodes','rosapi_msgs/srv/Nodes'),{'nodes':['/dashboard_client']})
        self.assertEqual(sent[0]['timeout'],1.5)
        self.assertFalse(ros._service_requests)

    async def test_timeout_cleans_pending_future_and_a_late_reply_is_ignored(self):
        ros=RosBridge('ws://unused');ros._connected=True;sent=[]
        async def send(payload):sent.append(json.loads(payload))
        ros._socket=SimpleNamespace(send=send)
        with self.assertRaises(asyncio.TimeoutError):await ros.call_service('/rosapi/nodes','rosapi_msgs/srv/Nodes',timeout=.01)
        self.assertFalse(ros._service_requests)
        ros._handle_message(dict(op='service_response',id=sent[0]['id'],result=True,values={}))
