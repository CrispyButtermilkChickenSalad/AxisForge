"""Bounded RPC and clock mapping for the two synchronized-job participants."""

import asyncio
from dataclasses import dataclass
import json
import math
import time
from urllib.parse import urlsplit, urlunsplit

from bridge_dashboard.clock_sync import HostRate, HostRateEstimator

import httpx
from websockets.exceptions import WebSocketException
from websockets.legacy.client import connect

PROTOCOL = 'axisforge.sync.v1'


class ParticipantError(RuntimeError):
    pass


class ParticipantTransportError(ParticipantError):
    """A missing/broken transport reply, distinct from a participant rejecting an RPC."""


class MasterClient:
    def __init__(self, moonraker):
        self.moonraker = moonraker
        self._clock_socket = None
        self._clock_lock = asyncio.Lock()
        self._clock_id = 0
        self._heartbeat_socket = None
        self._heartbeat_lock = asyncio.Lock()
        self._heartbeat_id = 0
        self._readers = {}
        self._pending = {}
        self.clock_estimator = None
        self._clock_monitor = None

    def start_clock_monitor(self):
        if self._clock_monitor is None:
            self.clock_estimator = HostRateEstimator()
            self._clock_monitor = asyncio.create_task(self._monitor_clock())

    async def _monitor_clock(self):
        # Read-only, including while idle/preflighting. Calibration overlaps
        # setup rather than holding the nozzle at the print entry point.
        while True:
            try:
                await sample_clock(self, samples=3, max_samples=3, interval=.02)
            except ParticipantError:
                pass
            await asyncio.sleep(1.)

    async def wait_clock_ready(self, timeout=90.):
        self.start_clock_monitor()
        deadline = time.monotonic()+timeout
        while True:
            try:
                return self.clock_estimator.estimate(time.monotonic())
            except ValueError as error:
                if time.monotonic() >= deadline:
                    raise ParticipantError(str(error)) from error
            await asyncio.sleep(.25)

    async def _close_channel(self, channel):
        socket = getattr(self, '_'+channel+'_socket')
        setattr(self, '_'+channel+'_socket', None)
        reader = self._readers.pop(channel, None)
        if reader is not None:
            reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)
        pending = self._pending.pop(channel, None)
        if pending is not None: pending[1].cancel()
        if socket is not None:
            try: await socket.close()
            except (OSError, WebSocketException): pass

    async def close(self):
        if self._clock_monitor is not None:
            self._clock_monitor.cancel()
            await asyncio.gather(self._clock_monitor, return_exceptions=True)
            self._clock_monitor = None
        await asyncio.gather(self._close_channel('clock'), self._close_channel('heartbeat'))

    async def _clock(self):
        return await self._socket_rpc('clock', {})

    async def _read_channel(self, channel, socket):
        # Moonraker broadcasts notifications even without subscriptions. Drain
        # them between RPCs too: a full receive queue blocks WebSocket ping/pong
        # processing and can kill an otherwise healthy idle clock connection.
        try:
            while True:
                payload = json.loads(await socket.recv())
                if not isinstance(payload, dict):
                    raise ValueError('Invalid '+channel+' response')
                pending = self._pending.get(channel)
                if pending is None or payload.get('id') != pending[0]: continue
                if 'error' in payload or not isinstance(payload.get('result'), dict):
                    raise ValueError(str(payload.get('error','Missing '+channel+' result')))
                if not pending[1].done(): pending[1].set_result(payload['result'])
        except (OSError, WebSocketException, ValueError) as error:
            pending = self._pending.get(channel)
            if pending is not None and not pending[1].done():
                pending[1].set_exception(error)

    async def _socket_rpc(self, channel, data):
        # Clock and heartbeat have separate authenticated sockets and locks.
        # Large plan/status HTTP traffic cannot occupy the heartbeat connection.
        async with getattr(self, '_'+channel+'_lock'):
            try:
                socket = getattr(self, '_'+channel+'_socket')
                reader = self._readers.get(channel)
                if reader is not None and reader.done():
                    await self._close_channel(channel)
                    socket = None
                if socket is None or socket.closed:
                    await self._close_channel(channel)
                    parts = urlsplit(str(self.moonraker._client.base_url))
                    url = urlunsplit(('wss' if parts.scheme == 'https' else 'ws',
                                     parts.netloc, parts.path.rstrip('/')+'/websocket', '', ''))
                    key = self.moonraker._client.headers.get('X-Api-Key')
                    socket = await connect(
                        url, extra_headers={'X-Api-Key':key} if key else {},
                        open_timeout=1., close_timeout=.05)
                    setattr(self, '_'+channel+'_socket', socket)
                    self._readers[channel] = asyncio.create_task(self._read_channel(channel, socket))
                request_id = getattr(self, '_'+channel+'_id')+1
                setattr(self, '_'+channel+'_id', request_id)
                reply = asyncio.get_running_loop().create_future()
                self._pending[channel] = (request_id, reply)
                await socket.send(json.dumps({
                    'jsonrpc':'2.0', 'id':request_id,
                    'method':'printer.axisforge_sync.'+channel, 'params':data}))
                return await reply
            except asyncio.CancelledError:
                # A cancelled exchange must not leave its reply for the next sample.
                await self._close_channel(channel)
                raise
            except (OSError, WebSocketException) as error:
                await self._close_channel(channel)
                raise ParticipantTransportError('Klipper '+channel+' connection failed: '+str(error)) from error
            except ValueError as error:
                await self._close_channel(channel)
                raise ParticipantError('Klipper '+channel+' rejected: '+str(error)) from error
            finally:
                self._pending.pop(channel, None)

    async def call(self, action, **data):
        if action == 'clock':
            try:
                return await asyncio.wait_for(self._clock(),1.)
            except asyncio.TimeoutError as error:
                raise ParticipantError('Klipper clock request timed out') from error
        if action == 'heartbeat':
            # Recovery, where permitted, belongs to the workflow's bounded cold
            # phase. A single exchange retains the strict 500 ms reply deadline.
            try:
                return await asyncio.wait_for(self._socket_rpc('heartbeat', data), .5)
            except asyncio.TimeoutError as error:
                raise ParticipantTransportError('Klipper heartbeat response timed out') from error
        try:
            # Preparation is non-actuating and can return a large rounded plan.
            # Leases are acquired only after it completes. Execution RPCs retain
            # their original timeout; heartbeat callers enforce a separate 0.5 s bound.
            timeout=60. if action=='prepare' else 10.
            response = await self.moonraker._client.post('/printer/axisforge_sync/'+action,
                                                        json=data, timeout=timeout)
            response.raise_for_status()
            payload = response.json()
            if 'error' in payload or 'result' not in payload:
                raise ValueError(str(payload.get('error','Missing result')))
            return payload['result']
        except httpx.TimeoutException as error:
            raise ParticipantError('Klipper sync %s timed out after %g s' % (action,timeout)) from error
        except (httpx.HTTPError, ValueError) as error:
            detail = ''
            if isinstance(error,httpx.HTTPStatusError): detail = error.response.text[:500]
            raise ParticipantError('Klipper sync '+action+' failed: '+str(error)+' '+detail) from error


class RobotClient:
    def __init__(self, url):
        self.client = httpx.AsyncClient(base_url=url,timeout=5.,trust_env=False)
        self.heartbeat_client = httpx.AsyncClient(base_url=url,timeout=.5,trust_env=False,
            limits=httpx.Limits(max_connections=1,max_keepalive_connections=1,keepalive_expiry=60.))

    async def call(self, action, **data):
        try:
            if action in ('status','capabilities'):
                response = await self.client.get('/'+action)
            else:
                client = self.heartbeat_client if action == 'heartbeat' else self.client
                response = await client.post('/'+action,json=data)
            payload = response.json()
            if response.is_error: raise ValueError(payload.get('error',response.text))
            return payload
        except httpx.TimeoutException as error:
            detail = 'Robot heartbeat response timed out' if action=='heartbeat' else 'Robot sync '+action+' timed out'
            raise ParticipantError(detail) from error
        except (httpx.HTTPError,ValueError) as error:
            raise ParticipantError('Robot sync '+action+' failed: '+str(error)) from error

    async def close(self):
        await asyncio.gather(self.client.aclose(),self.heartbeat_client.aclose())


@dataclass(frozen=True)
class ClockMap:
    """Affine print-time conversion, anchored at sampled_at with an RTT bound."""
    offset: float
    uncertainty: float
    sampled_at: float
    rate: float = 1.
    host_rate: HostRate | None = None
    epoch: str | None = None
    rate_diagnostic: str | None = None

    def offset_at(self, local_time):
        return self.offset+(local_time-self.sampled_at)*(self.rate-1.)

    def local_time(self, print_time, now=None):
        now = time.monotonic() if now is None else now
        if not math.isfinite(print_time) or now-self.sampled_at > 5.:
            raise ParticipantError('Clock mapping is stale or invalid')
        return self.sampled_at+(print_time-self.sampled_at-self.offset)/self.rate


async def sample_clock(master, clock=time.monotonic, samples=5, *,
                       max_samples=20, timeout=1.5, interval=.02, max_uncertainty_s=.01,
                       reference=None):
    """Intersect bounded exchanges, retrying transient jitter for at most 1.5 s.

    The selected limit includes a 1 ms MCU estimation allowance. The same sampler
    is used before scheduling and during the periodic running alignment check.
    """
    if samples < 1 or max_samples < samples or timeout <= 0 or interval < 0:
        raise ValueError('Invalid clock sampling limits')
    if (type(max_uncertainty_s) not in (int,float) or
            not math.isfinite(max_uncertainty_s) or not .002 <= max_uncertainty_s <= .02):
        raise ValueError('Clock uncertainty limit must be from 2 to 20 ms')
    candidates = []
    deadline = None
    last_error = None
    estimator = getattr(master, 'clock_estimator', None)
    host_rate = None
    rate_diagnostic = None
    epoch = None
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
        if estimator is not None:
            try:
                estimator.observe(sent, received, response)
                epoch = response['clock_epoch']
            except (ValueError, TypeError, AttributeError) as error:
                raise ParticipantError(str(error)) from error
        remote = response.get('print_time') if isinstance(response,dict) else None
        rate = response.get('print_time_rate',1.) if isinstance(response,dict) else None
        if (type(remote) not in (int,float) or not math.isfinite(remote)
                or type(rate) not in (int,float) or not math.isfinite(rate) or not .999 <= rate <= 1.001
                or not math.isfinite(sent) or not math.isfinite(received) or received < sent):
            raise ParticipantError('Invalid Klipper clock sample')
        candidates.append((sent, received, remote, rate, response))
        if len(candidates) >= samples and min((c[1]-c[0])/2+.001 for c in candidates) <= max_uncertainty_s:
            break
        if index+1 < max_samples:
            await asyncio.sleep(min(interval,max(0.,deadline-received)))
    if not candidates:
        raise ParticipantError('Unable to measure Klipper clock: '+(last_error or 'sampling timed out'))
    sent, received, remote, rate, response = min(candidates,key=lambda item:item[1]-item[0])
    if estimator is not None:
        if any(c[4]['clock_epoch'] != epoch for c in candidates):
            raise ParticipantError('Klipper clock restarted during measurement')
        if reference is not None and reference.epoch != epoch:
            raise ParticipantError('Klipper clock restarted during synchronized motion')
        try:
            host_rate = estimator.estimate(candidates[-1][1])
        except ValueError as error:
            if reference is None:
                raise ParticipantError(str(error)) from error
            rate_diagnostic = str(error)
        # A moving trajectory already has a fixed rate. Its watchdog needs a
        # fresh phase interval, not permission to estimate a new trajectory rate.
        # Bad fit quality blocks NEW scheduling, never masks current phase drift.
        rate = reference.rate if reference is not None else response['mcu_raw_clock_rate']*host_rate.rate
        if not .999 <= rate <= 1.001:
            raise ParticipantError('Combined MCU/PC clock rate exceeds the checked trajectory margin')
    # Each exchange bounds the timestamp between send and receive. Intersect
    # them at one instant instead of trusting one RPC midpoint. Preserve the
    # full MCU allowance and reject contradictory observations.
    anchor = candidates[-1][1]
    lower = max(p+(anchor-r)*rate-anchor-.001 for s,r,p,_,_ in candidates)
    upper = min(p+(anchor-s)*rate-anchor+.001 for s,r,p,_,_ in candidates)
    if lower > upper:
        raise ParticipantError('Inconsistent Klipper clock samples; clock phase changed during measurement')
    best = ClockMap((lower+upper)/2, (upper-lower)/(2*rate), anchor, rate, host_rate, epoch, rate_diagnostic)
    if best.uncertainty > max_uncertainty_s:
        raise ParticipantError('Clock uncertainty %.2f ms exceeds %g ms '
                               '(best round trip %.2f ms across %d samples); '
                               'check the robot/Klipper network and retry preflight' %
                               (best.uncertainty*1000,max_uncertainty_s*1000,
                                min(c[1]-c[0] for c in candidates)*1000,len(candidates)))
    return best
