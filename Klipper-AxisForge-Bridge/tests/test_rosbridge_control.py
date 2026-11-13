import asyncio,json
import unittest
from types import SimpleNamespace
from bridge_dashboard.rosbridge import RosBridge


class ReadOnlyRpcTests(unittest.IsolatedAsyncioTestCase):
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
