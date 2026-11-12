"""Bounded RPC and clock mapping for the two synchronized-job participants."""

import asyncio
from dataclasses import dataclass
import json
import math
import time
from urllib.parse import urlsplit, urlunsplit

import httpx
from websockets.exceptions import WebSocketException
from websockets.legacy.client import connect

PROTOCOL = 'axisforge.sync.v1'


class ParticipantError(RuntimeError):
    pass


class MasterClient:
    def __init__(self, moonraker):
        self.moonraker = moonraker
        self._clock_socket = None
        self._clock_lock = asyncio.Lock()
        self._clock_id = 0

    async def close(self):
        socket, self._clock_socket = self._clock_socket, None
        if socket is not None:
            await socket.close()

    async def _clock(self):
        # A dedicated persistent connection avoids HTTP setup/dispatch jitter.
        # Heartbeats and other control RPCs continue independently over HTTP.
        async with self._clock_lock:
            try:
                if self._clock_socket is None or self._clock_socket.closed:
                    parts = urlsplit(str(self.moonraker._client.base_url))
                    url = urlunsplit(('wss' if parts.scheme == 'https' else 'ws',
                                     parts.netloc, parts.path.rstrip('/')+'/websocket', '', ''))
                    key = self.moonraker._client.headers.get('X-Api-Key')
                    self._clock_socket = await connect(
                        url, extra_headers={'X-Api-Key':key} if key else {},
                        open_timeout=1., close_timeout=.1)
                self._clock_id += 1
                request_id = self._clock_id
                await self._clock_socket.send(json.dumps({
                    'jsonrpc':'2.0', 'id':request_id,
                    'method':'printer.axisforge_sync.clock', 'params':{}}))
                while True:
                    payload = json.loads(await self._clock_socket.recv())
                    if not isinstance(payload,dict):
                        raise ValueError('Invalid clock response')
                    # Moonraker notifications and late replies are not samples.
                    if payload.get('id') != request_id: continue
                    if 'error' in payload or not isinstance(payload.get('result'),dict):
                        raise ValueError(str(payload.get('error','Missing clock result')))
                    return payload['result']
            except asyncio.CancelledError:
                # A cancelled exchange must not leave its reply for the next sample.
                await self.close()
                raise
            except (OSError, WebSocketException, ValueError) as error:
                await self.close()
                raise ParticipantError('Klipper clock connection failed: '+str(error)) from error

    async def call(self, action, **data):
        if action == 'clock':
            try:
                return await asyncio.wait_for(self._clock(),1.)
            except asyncio.TimeoutError as error:
                raise ParticipantError('Klipper clock request timed out') from error
        try:
            response = await self.moonraker._client.post('/printer/axisforge_sync/'+action,
                                                        json=data, timeout=10.)
            response.raise_for_status()
            payload = response.json()
            if 'error' in payload or 'result' not in payload:
                raise ValueError(str(payload.get('error','Missing result')))
            return payload['result']
        except (httpx.HTTPError, ValueError) as error:
            detail = ''
            if isinstance(error,httpx.HTTPStatusError): detail = error.response.text[:500]
            raise ParticipantError('Klipper sync '+action+' failed: '+str(error)+' '+detail) from error


class RobotClient:
    def __init__(self, url):
        self.client = httpx.AsyncClient(base_url=url,timeout=5.,trust_env=False)

    async def call(self, action, **data):
        try:
            if action in ('status','capabilities'):
                response = await self.client.get('/'+action)
            else:
                response = await self.client.post('/'+action,json=data)
            payload = response.json()
            if response.is_error: raise ValueError(payload.get('error',response.text))
            return payload
        except (httpx.HTTPError,ValueError) as error:
            raise ParticipantError('Robot sync '+action+' failed: '+str(error)) from error

    async def close(self): await self.client.aclose()


@dataclass(frozen=True)
class ClockMap:
    """Klipper print_time = local monotonic + offset, bounded by measured RTT."""
    offset: float
    uncertainty: float
    sampled_at: float

    def local_time(self, print_time, now=None):
        now = time.monotonic() if now is None else now
        if not math.isfinite(print_time) or now-self.sampled_at > 5.:
            raise ParticipantError('Clock mapping is stale or invalid')
        return print_time-self.offset


async def sample_clock(master, clock=time.monotonic, samples=5, *,
                       max_samples=20, timeout=1.5, interval=.02):
    """Find a bounded low-RTT sample, retrying transient jitter for at most 1.5 s.

    The uncertainty limit stays 10 ms, including a 1 ms MCU estimation allowance.
    Retries happen at stopped motion boundaries, before reserving a new chunk.
    """
    if samples < 1 or max_samples < samples or timeout <= 0 or interval < 0:
        raise ValueError('Invalid clock sampling limits')
    candidates = []
    deadline = None
    last_error = None
    for index in range(max_samples):
        sent = clock()
        if deadline is None: deadline = sent+timeout
        remaining = deadline-sent
        if remaining <= 0: break
        try:
            response = await asyncio.wait_for(master.call('clock'),remaining)
        except (ParticipantError, asyncio.TimeoutError) as error:
            last_error = str(error) or 'Clock request timed out'
            if index+1 < max_samples: await asyncio.sleep(interval)
            continue
        received = clock()
        remote = response.get('print_time') if isinstance(response,dict) else None
        if (type(remote) not in (int,float) or not math.isfinite(remote)
                or not math.isfinite(sent) or not math.isfinite(received) or received < sent):
            raise ParticipantError('Invalid Klipper clock sample')
        uncertainty = (received-sent)/2+.001  # Include the MCU clock-estimation allowance.
        candidates.append(ClockMap(remote-(sent+received)/2,uncertainty,received))
        if len(candidates) >= samples and min(c.uncertainty for c in candidates) <= .01:
            break
        if index+1 < max_samples:
            await asyncio.sleep(min(interval,max(0.,deadline-received)))
    if not candidates:
        raise ParticipantError('Unable to measure Klipper clock: '+(last_error or 'sampling timed out'))
    best = min(candidates,key=lambda item:item.uncertainty)
    if best.uncertainty > .01:
        raise ParticipantError('Clock uncertainty %.2f ms exceeds 10 ms '
                               '(best round trip %.2f ms across %d samples); '
                               'check the robot/Klipper network and retry preflight' %
                               (best.uncertainty*1000,(best.uncertainty-.001)*2000,len(candidates)))
    return best
