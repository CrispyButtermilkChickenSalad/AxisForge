"""Clock transport checks against a local Moonraker-like WebSocket server."""

import asyncio
import json
import unittest

from websockets.legacy.server import serve

from bridge_dashboard.moonraker import MoonrakerClient
from bridge_dashboard.sync_clients import MasterClient, ParticipantError, sample_clock


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
            if self.mode == 'stall':
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


if __name__ == '__main__': unittest.main()
