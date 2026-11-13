"""Clock transport checks against a local Moonraker-like WebSocket server."""

import asyncio
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import unittest
from types import SimpleNamespace
import httpx

from websockets.legacy.server import serve

from bridge_dashboard.moonraker import MoonrakerClient
from bridge_dashboard.sync_clients import MasterClient, RobotClient, ParticipantError, ParticipantTransportError, sample_clock


class ClockTransportTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.connections = []
        self.requests = []
        self.mode = 'normal'
        self.received = asyncio.Event()
        self.server = await serve(self.handle,'127.0.0.1',0)
        port = self.server.sockets[0].getsockname()[1]
        self.moonraker = MoonrakerClient('http://127.0.0.1:%d/proxy/' % port,'test-key')
        self.master = MasterClient(self.moonraker)

    async def asyncTearDown(self):
        await self.master.close()
        await self.moonraker.close()
        self.server.close()
        await self.server.wait_closed()

    async def handle(self, socket, path):
        self.connections.append((socket,path,socket.request_headers.get('X-Api-Key')))
        async for raw in socket:
            request = json.loads(raw)
            self.requests.append(request)
            self.received.set()
            if self.mode == 'stall' or (self.mode=='stall-clock' and request['method'].endswith('.clock')):
                await socket.wait_closed()
                return
            if self.mode == 'disconnect':
                self.mode = 'normal'
                await socket.close()
                return
            if self.mode == 'error':
                await socket.send(json.dumps({'id':request['id'],'error':{'message':'not ready'}}))
                continue
            await socket.send(json.dumps({'method':'notify_status_update','params':[]}))
            await socket.send(json.dumps({'id':request['id']-1,'result':{'print_time':-1.}}))
            await socket.send(json.dumps({'id':request['id'],'result':{'print_time':123.}}))

    async def test_reuses_authenticated_connection_and_matches_reply_ids(self):
        for _ in range(3):
            self.assertEqual(await self.master.call('clock'),{'print_time':123.})
        self.assertEqual(len(self.connections),1)
        _,path,key = self.connections[0]
        self.assertEqual((path,key),('/proxy/websocket','test-key'))
        self.assertEqual([r['id'] for r in self.requests],[1,2,3])
        self.assertTrue(all(r['method']=='printer.axisforge_sync.clock' for r in self.requests))

    async def test_sampling_reconnects_after_connection_loss(self):
        self.mode = 'disconnect'
        await sample_clock(self.master,samples=1,interval=0)
        self.assertEqual(len(self.connections),2)

    async def test_idle_notifications_do_not_block_pongs_or_the_next_rpc(self):
        await self.master.call('clock')
        await self.master.call('heartbeat', session='s')
        for socket, _, _ in self.connections:
            # More than websockets' default 32-message receive queue. An idle
            # RPC-only reader blocks here and stops answering server pings.
            for _ in range(80):
                await socket.send(json.dumps({'method':'notify_proc_stat_update','params':[{}]}))
            pong = await socket.ping()
            await asyncio.wait_for(pong, .5)
        self.assertEqual(await self.master.call('clock'), {'print_time':123.})
        self.assertEqual(await self.master.call('heartbeat', session='s'), {'print_time':123.})
        self.assertEqual(len(self.connections), 2)

    async def test_reconnects_after_idle_socket_closes(self):
        await self.master.call('clock')
        await self.connections[0][0].close()
        self.assertEqual(await self.master.call('clock'), {'print_time':123.})
        self.assertEqual(len(self.connections), 2)

    async def test_rpc_error_is_not_a_clock_sample(self):
        self.mode = 'error'
        with self.assertRaisesRegex(ParticipantError,'not ready'):
            await self.master.call('clock')
        self.assertIsNone(self.master._clock_socket)

    async def test_cancelled_exchange_closes_socket_and_next_call_reconnects(self):
        self.mode = 'stall'
        task = asyncio.create_task(self.master.call('clock'))
        await asyncio.wait_for(self.received.wait(),1.)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError): await task
        self.assertIsNone(self.master._clock_socket)
        self.mode = 'normal'
        self.assertEqual(await self.master.call('clock'),{'print_time':123.})
        self.assertEqual(len(self.connections),2)

    async def test_timeout_is_bounded_and_closes_socket(self):
        self.mode = 'stall'
        with self.assertRaisesRegex(ParticipantError,'timed out'):
            await asyncio.wait_for(self.master.call('clock'),2.)
        self.assertIsNone(self.master._clock_socket)

    async def test_heartbeat_has_independent_authenticated_connection_and_lock(self):
        self.mode='stall-clock'
        clock=asyncio.create_task(self.master.call('clock'))
        await self.received.wait()
        await asyncio.wait_for(self.master.call('heartbeat',session='session'),.5)
        self.assertEqual(len(self.connections),2)
        self.assertTrue(all(path=='/proxy/websocket' and key=='test-key' for _,path,key in self.connections))
        self.assertFalse(clock.done())
        clock.cancel()
        with self.assertRaises(asyncio.CancelledError):await clock
        heartbeat_socket=self.master._heartbeat_socket
        await self.master.call('heartbeat',session='session')
        self.assertIs(self.master._heartbeat_socket,heartbeat_socket)
        self.assertEqual(len(self.connections),2)

    async def test_heartbeat_timeout_reconnects_without_closing_clock_socket(self):
        await self.master.call('clock')
        clock_socket=self.master._clock_socket
        self.mode='stall'
        with self.assertRaisesRegex(ParticipantTransportError,'heartbeat response timed out'):
            await self.master.call('heartbeat',session='s')
        self.assertIsNone(self.master._heartbeat_socket)
        self.assertIs(self.master._clock_socket,clock_socket)
        self.mode='normal'
        await self.master.call('heartbeat',session='s')
        self.assertEqual(len(self.connections),3)

    async def test_heartbeat_rpc_rejection_is_not_retryable_transport_failure(self):
        self.mode='error'
        with self.assertRaises(ParticipantError) as caught:
            await self.master.call('heartbeat',session='s')
        self.assertNotIsInstance(caught.exception,ParticipantTransportError)
        self.assertIn('not ready',str(caught.exception))


class MasterRpcTests(unittest.IsolatedAsyncioTestCase):
    async def test_large_preparation_has_its_own_timeout_without_relaxing_control(self):
        requests=[]
        async def handle(request):
            requests.append((request.url.path,request.extensions['timeout']['read']))
            return httpx.Response(200,json={'result':{'state':'prepared'}})
        async with httpx.AsyncClient(base_url='http://test',transport=httpx.MockTransport(handle)) as http:
            master=MasterClient(SimpleNamespace(_client=http))
            await master.call('prepare',job='test.gcode')
            await master.call('status')
            await master.call('commit',session='s',token='t')
        self.assertEqual([t for _,t in requests],[60.,10.,10.])

    async def test_read_timeout_names_action_and_deadline(self):
        async def handle(request): raise httpx.ReadTimeout('')
        async with httpx.AsyncClient(base_url='http://test',transport=httpx.MockTransport(handle)) as http:
            master=MasterClient(SimpleNamespace(_client=http))
            with self.assertRaisesRegex(ParticipantError,'prepare timed out after 60 s'):
                await master.call('prepare',job='test.gcode')


class RobotTransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_heartbeats_keep_their_connection_while_control_pool_is_busy(self):
        ports=[];entered=threading.Event();release=threading.Event()
        class Handler(BaseHTTPRequestHandler):
            protocol_version='HTTP/1.1'
            def log_message(self,*args):pass
            def do_POST(self):
                self.rfile.read(int(self.headers['Content-Length']))
                if self.path=='/schedule':entered.set();release.wait(2.)
                else:ports.append(self.client_address[1])
                body=b'{"state":"ready","leased":true}'
                self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers()
                self.wfile.write(body)
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        url='http://127.0.0.1:'+str(server.server_port)
        robot=RobotClient(url)
        await robot.client.aclose()
        robot.client=httpx.AsyncClient(base_url=url,limits=httpx.Limits(max_connections=1),trust_env=False)
        control=asyncio.create_task(robot.call('schedule',session='s'))
        try:
            while not entered.is_set():await asyncio.sleep(.005)
            for _ in range(3):
                await asyncio.wait_for(robot.call('heartbeat',session='s'),.3)
            self.assertEqual(len(set(ports)),1)
            self.assertFalse(control.done())
        finally:
            release.set();await control;await robot.close();server.shutdown();server.server_close()


if __name__ == '__main__': unittest.main()
