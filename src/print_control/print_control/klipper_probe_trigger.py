"""Lease-bound native Klipper contact events; this client never commands a robot."""
import json
import math
import threading
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import uuid


class KlipperProbeTriggerError(RuntimeError):
    pass


def _positive(value):
    return type(value) in (int, float) and math.isfinite(value) and value > 0


class KlipperProbeTrigger:
    """One fresh server session per contact, with no fallback to streamed counts.

    ``check`` requests immediate robot cancellation for an early MCU completion.
    Only ``confirm_contact`` classifies that completion as a valid contact.
    """
    def __init__(self, moonraker_url, api_key=None, *, max_delta_counts=120000.,
                 max_nozzle_temp_c=50., lease_s=.4, stall_timeout_s=.3,
                 request_timeout_s=.25, cleanup_timeout_s=.8,
                 classification_timeout_s=1., arm_timeout_s=3., clock=time.monotonic, opener=urlopen,
                 thermal_targets=None):
        parts = urlsplit(moonraker_url)
        if (parts.scheme not in ('http', 'https') or not parts.hostname or
                parts.username or parts.password or parts.query or parts.fragment):
            raise ValueError('Use a Moonraker HTTP(S) URL without credentials, query, or fragment')
        for value in (max_delta_counts, max_nozzle_temp_c, lease_s, stall_timeout_s,
                      request_timeout_s, cleanup_timeout_s, classification_timeout_s):
            if not _positive(value):
                raise ValueError('Native probe limits and deadlines must be finite and positive')
        if (lease_s > .4 or stall_timeout_s > .3 or request_timeout_s >= stall_timeout_s or
                stall_timeout_s >= lease_s):
            raise ValueError('Native probe requires HTTP timeout < stall deadline <= 0.3 s < lease <= 0.4 s')
        if max_delta_counts > 120000 or max_nozzle_temp_c > 50:
            raise ValueError('Native probing retains the 120000-count overload and 50 C cold-nozzle ceilings')
        if cleanup_timeout_s > 1 or classification_timeout_s > 1:
            raise ValueError('Native probe cleanup and classification deadlines must not exceed one second')
        if not _positive(arm_timeout_s) or not request_timeout_s < arm_timeout_s <= 3:
            raise ValueError('Native probe arming deadline must exceed HTTP polling timeout and be at most 3 seconds')
        self.base_url, self.api_key = moonraker_url.rstrip('/'), api_key
        self.max_delta_counts, self.max_nozzle_temp_c = max_delta_counts, max_nozzle_temp_c
        targets={} if thermal_targets is None else thermal_targets
        if not isinstance(targets,dict) or set(targets)-{'extruder','heater_bed'}:
            raise ValueError('Native heated probing supports nozzle and bed targets only')
        for name,target in targets.items():
            low,high=(50.,300.) if name=='extruder' else (30.,150.)
            if not _positive(target) or not low<=target<=high:
                raise ValueError('Invalid native probing temperature target: '+name)
        self.thermal_targets=dict(targets)
        self.lease_s, self.stall_timeout_s = lease_s, stall_timeout_s
        self.request_timeout_s, self.cleanup_timeout_s = request_timeout_s, cleanup_timeout_s
        self.classification_timeout_s = classification_timeout_s
        self.arm_timeout_s = arm_timeout_s
        self.clock, self.opener = clock, opener
        self._lock = threading.RLock()
        self._thread = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._session = None
        self._phase = 'idle'
        self._error = self._cleanup_error = None
        self._disarmed = True
        self._last_success = None
        self._arming_deadline = None
        self._arm_inflight = False
        self._stop_required = False
        self._triggered_timestamp = None
        self._capabilities = None

    @property
    def session(self):
        with self._lock:
            return self._session

    @property
    def triggered_timestamp(self):
        with self._lock:
            return self._triggered_timestamp

    def _request(self, endpoint, body=None, *, timeout_s=None):
        headers = {'Accept': 'application/json'}
        if self.api_key:
            headers['X-Api-Key'] = self.api_key
        if body is not None:
            headers['Content-Type'] = 'application/json'
        request = Request(self.base_url+'/printer/axisforge_probe/'+endpoint,
                          data=None if body is None else json.dumps(body, allow_nan=False).encode(),
                          headers=headers, method='GET' if body is None else 'POST')
        try:
            with self.opener(request, timeout=self.request_timeout_s if timeout_s is None else timeout_s) as response:
                raw = response.read(262145)
            if len(raw) > 262144:
                raise ValueError('Response exceeds the native probe size limit')
            data = json.loads(raw)
            if (not isinstance(data, dict) or
                    'error' in data and 'result' not in data and 'protocol' not in data):
                raise ValueError('Native probe HTTP request returned an error')
            data = data.get('result', data)
            if not isinstance(data, dict):
                raise ValueError('Native probe response is not an object')
            if type(data.get('protocol')) is not int or data['protocol'] != 1:
                raise ValueError('Unsupported native probe protocol; expected protocol 1')
            return data
        except Exception as error:
            raise KlipperProbeTriggerError('Klipper native probe '+endpoint+' failed: '+str(error)) from error

    def validate_ready(self):
        """Read native calibration/capabilities without arming anything."""
        with self._lock:
            self._capabilities = None
        data = self._request('capabilities')
        if data.get('supported') is not True:
            raise KlipperProbeTriggerError('Klipper native probing is not supported by this host')
        if self.thermal_targets and data.get('heated_probing') is not True:
            raise KlipperProbeTriggerError('Install the updated axisforge_probe Klipper module for heated native probing')
        if (data.get('calibrated') is not True or not _positive(data.get('trigger_force_g')) or
                not _positive(data.get('counts_per_gram'))):
            raise KlipperProbeTriggerError('Native probing requires calibrated positive force and counts-per-gram settings')
        with self._lock:
            self._capabilities = dict(data)
        return dict(data)

    def _fail(self, message):
        with self._lock:
            self._error = self._error or str(message)
            self._phase = 'fault'
            self._stop.set()
            self._ready.set()

    def _accept(self, data, session):
        if data.get('session') != session:
            raise KlipperProbeTriggerError('Native probe returned the wrong contact session')
        state = data.get('state')
        if state == 'fault':
            detail = data.get('reason') or data.get('message') or data.get('error') or 'unspecified native fault'
            raise KlipperProbeTriggerError('Klipper native probe fault: '+str(detail)[:1000])
        if state not in ('armed', 'triggered'):
            raise KlipperProbeTriggerError('Unexpected native probe state: '+str(state))
        stop_required = data.get('stop_required', state == 'triggered')
        if type(stop_required) is not bool:
            raise KlipperProbeTriggerError('Invalid native probe stop indication')
        stamp = data.get('trigger_time')
        if state == 'triggered' and not _positive(stamp):
            raise KlipperProbeTriggerError('Native probe trigger timestamp is invalid')
        with self._lock:
            if session != self._session or self._stop.is_set():
                return False
            if self._stop_required and not stop_required:
                raise KlipperProbeTriggerError('Native probe stop indication regressed')
            if self._triggered_timestamp is not None and (
                    state != 'triggered' or stamp != self._triggered_timestamp):
                raise KlipperProbeTriggerError('Native probe contact classification changed')
            self._stop_required = stop_required or state == 'triggered'
            if state == 'triggered':
                self._triggered_timestamp = float(stamp)
            self._phase, self._last_success = state, self.clock()
            self._ready.set()
        return True

    def _run(self, session, search_timeout_s):
        arm_sent = False
        try:
            if self._capabilities is None:
                self.validate_ready()
                with self._lock:
                    self._last_success = self.clock()
            with self._lock:
                if self._stop.is_set():
                    return
                arm_sent = self._arm_inflight = True
            try:
                data = self._request('arm', dict(session=session, max_delta_counts=self.max_delta_counts,
                    max_nozzle_temp_c=self.max_nozzle_temp_c, search_timeout_s=search_timeout_s,
                    lease_s=self.lease_s, **({'thermal_targets':dict(self.thermal_targets)} if self.thermal_targets else {})), timeout_s=self.arm_timeout_s)
            finally:
                with self._lock:
                    self._arm_inflight = False
            if not self._accept(data, session):
                return
            while not self._stop.is_set():
                data = self._request('wait', dict(session=session, timeout_s=.1))
                if not self._accept(data, session):
                    return
                # Triggered responses can be immediate. Continue lease renewal
                # without spinning while the robot confirms cancellation.
                if self._stop_required:
                    self._stop.wait(.02)
        except Exception as error:
            self._fail(error)
        finally:
            try:
                if arm_sent:
                    data = self._request('disarm', {'session': session})
                    cleaned_fault = data.get('state') == 'fault' and data.get('cleanup_confirmed') is True
                    if data.get('session') != session or not (data.get('state') == 'disarmed' or cleaned_fault):
                        raise KlipperProbeTriggerError('Native probe disarm was not confirmed for this session')
                    if cleaned_fault:
                        detail = data.get('reason') or data.get('message') or data.get('error') or 'native fault'
                        self._fail('Klipper native probe fault: '+str(detail)[:1000])
                with self._lock:
                    self._disarmed = True
                    if self._error is None:
                        self._phase = 'disarmed'
            except Exception as error:
                with self._lock:
                    self._cleanup_error = str(error)
                self._fail('Native probe cleanup failed: '+str(error))
            self._ready.set()

    def begin_contact(self, pump=lambda: None, *, search_timeout_s=7200.):
        """Arm on a worker thread while the caller services robot/sensor guards."""
        if not _positive(search_timeout_s) or search_timeout_s > 7200:
            raise ValueError('Native probe search timeout must be positive and at most 7200 seconds')
        with self._lock:
            if self._thread is not None and self._thread.is_alive() or not self._disarmed:
                raise KlipperProbeTriggerError('The previous native contact is not confirmed disarmed')
            self._session = session = str(uuid.uuid4())
            self._stop, self._ready = threading.Event(), threading.Event()
            self._phase = 'starting'
            self._error = self._cleanup_error = None
            self._disarmed, self._stop_required = False, False
            self._triggered_timestamp = None
            self._last_success = self.clock()
            # Tare + live MCU setup may take longer than an active event poll.
            # No ROS motion is allowed until the arm response is accepted.
            self._arming_deadline = self.clock()+self.arm_timeout_s+self.request_timeout_s
            self._thread = threading.Thread(target=self._run, args=(session, search_timeout_s), daemon=True)
            self._thread.start()
        try:
            while not self._ready.is_set():
                pump()
                self.check()
                self._ready.wait(.005)
            pump()
            self.check()
        except BaseException as error:
            self._fail(error)
            try:
                self.end_contact()
            except Exception as cleanup_error:
                raise KlipperProbeTriggerError(str(error)+'; '+str(cleanup_error)) from error
            raise

    def check(self):
        """Return whether to stop motion; raise on faults or stale renewal."""
        with self._lock:
            if self._phase == 'starting' and self.clock() >= self._arming_deadline:
                self._fail('Native probe arming timed out before motion')
            if self._phase in ('armed', 'triggered') and (
                    self._last_success is None or self.clock()-self._last_success > self.stall_timeout_s):
                self._fail('Native probe event worker stalled; stop motion')
            if self._error:
                raise KlipperProbeTriggerError(self._error)
            if self._phase not in ('starting', 'armed', 'triggered'):
                raise KlipperProbeTriggerError('No active native probe contact')
            return self._stop_required

    def confirm_contact(self, pump=lambda: None):
        """After robot cancellation, require a classified native contact."""
        if not self.check():
            raise KlipperProbeTriggerError('Native probe has not requested a contact stop')
        deadline = self.clock()+self.classification_timeout_s
        while True:
            pump()
            self.check()
            with self._lock:
                if self._triggered_timestamp is not None:
                    return self._triggered_timestamp
            if self.clock() >= deadline:
                self._fail('Native contact classification timed out; contact rejected')
                self.check()
            self._stop.wait(.005)

    def end_contact(self):
        """Bound cleanup and require a matching confirmed disarm response."""
        with self._lock:
            thread = self._thread
            if thread is None:
                return
            original_error = self._error
            self._stop.set()
            # During pre-motion arming, allow its separately bounded request to
            # finish before confirming disarm. Active contact cleanup stays short.
            timeout = max(self.cleanup_timeout_s, self.arm_timeout_s+self.request_timeout_s+.1) \
                if self._arm_inflight else self.cleanup_timeout_s
        thread.join(timeout)
        with self._lock:
            if thread.is_alive() or not self._disarmed or self._cleanup_error:
                reason = self._cleanup_error or 'Native probe disarm acknowledgement timed out'
                self._fail(reason)
                raise KlipperProbeTriggerError(reason)
            if original_error is None and self._error is not None:
                raise KlipperProbeTriggerError(self._error)

    close = end_contact
