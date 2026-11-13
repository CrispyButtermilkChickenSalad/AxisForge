"""Strict, single-use confirmations and a bridge heartbeat for guided probing."""
import json
import secrets
import sys
import threading
import time


EVENT_PREFIX = 'AXISFORGE_PROBE '


class ProbeControl:
    confirmation_stages = ('surface', 'front_left')
    def __init__(self, input_stream=None, output_stream=None, *, clock=time.monotonic,
                 heartbeat_timeout_s=5., confirmation_timeout_s=300.):
        self.input = input_stream if input_stream is not None else sys.stdin
        self.output = output_stream if output_stream is not None else sys.stdout
        self.clock = clock
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.confirmation_timeout_s = confirmation_timeout_s
        self.last_heartbeat = clock()
        self.lock = threading.RLock()
        self.pending = None
        self.pending_deadline = None
        self.confirmed = False
        self.reason = None
        self.state = 'idle'

    def start(self):
        threading.Thread(target=self._read, daemon=True).start()
        return self

    def _read(self):
        try:
            while True:
                line = self.input.readline(65537)
                if not line:
                    self.cancel('Bridge control pipe closed')
                    return
                if len(line) > 65536: raise ValueError('Control message exceeds limit')
                if line.strip(): self.ingest(json.loads(line))
        except Exception as error:
            self.cancel('Bridge control input failed: '+str(error))

    def ingest(self, command):
        if not isinstance(command, dict): raise ValueError('Expected a control object')
        with self.lock:
            kind = command.get('command')
            if kind == 'heartbeat':
                if self.clock()-self.last_heartbeat > self.heartbeat_timeout_s:
                    self.cancel('Bridge heartbeat timed out during bed probing')
                    return False
                self.last_heartbeat = self.clock()
                return True
            if kind == 'abort':
                self.cancel('Operator cancelled bed probing')
                return True
            if kind != 'confirm': raise ValueError('Unknown probe control command')
            if (self.reason or self.confirmed or self.pending is None or
                    self.pending_deadline is None or self.clock() >= self.pending_deadline or
                    command.get('stage') != self.pending['stage'] or
                    command.get('token') != self.pending['token']):
                return False
            self.confirmed = True
            return True

    def cancel(self, reason):
        with self.lock:
            if self.reason is None: self.reason = reason

    def check(self):
        with self.lock:
            if self.reason: raise RuntimeError(self.reason)
            if self.clock()-self.last_heartbeat > self.heartbeat_timeout_s:
                self.cancel('Bridge heartbeat timed out during bed probing')
                raise RuntimeError(self.reason)

    def emit(self, state, message, **values):
        with self.lock:
            self.state = state
            data = {'state': state, 'message': message, 'confirmation': None, **values}
            self.output.write(EVENT_PREFIX+json.dumps(data, allow_nan=False)+'\n')
            self.output.flush()

    def wait_confirmation(self, stage, message, pump, **values):
        if stage not in self.confirmation_stages: raise ValueError('Unknown confirmation stage')
        with self.lock:
            self.pending = {'stage': stage, 'token': secrets.token_hex(16)}
            self.pending_deadline = self.clock()+self.confirmation_timeout_s
            self.confirmed = False
            self.emit('waiting_'+stage, message, confirmation=dict(self.pending), **values)
        deadline = self.pending_deadline
        try:
            while True:
                self.check()
                if self.clock() >= deadline: raise RuntimeError('Timed out waiting for '+stage+' confirmation')
                with self.lock:
                    if self.confirmed: return
                pump()
        finally:
            with self.lock:
                self.pending = None
                self.pending_deadline = None
                self.confirmed = False
