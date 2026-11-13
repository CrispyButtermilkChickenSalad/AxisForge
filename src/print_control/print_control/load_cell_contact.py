"""Read ALPS bulk samples and detect bounded nozzle contact; no robot commands."""
from collections import deque
from dataclasses import dataclass
import json
import math
import statistics
import threading
import time
from urllib.parse import urlencode, urlsplit, urlunsplit
from urllib.request import Request, urlopen

# Installed Klipper ADS131M0X.get_range(): signed, unscaled 24-bit samples.
ADC_MIN_COUNTS, ADC_MAX_COUNTS = -0x800000, 0x7fffff


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class ContactOptions:
    trigger_mode: str = 'grams'
    contact_delta_counts: float = 20000.
    max_delta_counts: float = 60000.
    contact_force_g: float = 50.
    max_force_g: float = 200.
    step_mm: float = .02
    speed_mm_s: float = .2
    retract_mm: float = .5
    samples: int = 3
    repeatability_mm: float = .05
    contact_offset_mm: float = 0.
    settle_s: float = .15
    window_s: float = .1
    max_sample_age_s: float = .3
    max_nozzle_temp_c: float = 50.
    consecutive_samples: int = 5

    def __post_init__(self):
        if self.trigger_mode not in ('grams', 'raw'):
            raise ValueError('trigger_mode must be grams or raw')
        for name, value in vars(self).items():
            if name == 'trigger_mode': continue
            if not finite(value): raise ValueError(name+' must be finite')
            if name != 'contact_offset_mm' and value <= 0:
                raise ValueError(name+' must be positive')
        if self.max_force_g <= self.contact_force_g:
            raise ValueError('max_force_g must exceed contact_force_g')
        if self.max_delta_counts <= self.contact_delta_counts:
            raise ValueError('max_delta_counts must exceed contact_delta_counts')
        if self.max_delta_counts >= ADC_MAX_COUNTS:
            raise ValueError('max_delta_counts must leave headroom within the ADS131M02 range')
        if not .001 <= self.step_mm <= .05 or self.speed_mm_s > .5:
            raise ValueError('Contact steps must be 0.001..0.05 mm and speed at most 0.5 mm/s')
        if self.retract_mm < 5*self.step_mm:
            raise ValueError('retract_mm must be at least five contact steps')
        if type(self.samples) is not int or not 2 <= self.samples <= 5:
            raise ValueError('samples must be an integer from 2 to 5')
        if type(self.consecutive_samples) is not int or not 2 <= self.consecutive_samples <= 50:
            raise ValueError('consecutive_samples must be an integer from 2 to 50')
        if not .1 <= self.max_sample_age_s <= .5:
            raise ValueError('max_sample_age_s must be 0.1..0.5 s')

    @property
    def threshold(self):
        return self.contact_delta_counts if self.trigger_mode == 'raw' else self.contact_force_g

    @property
    def maximum(self):
        return self.max_delta_counts if self.trigger_mode == 'raw' else self.max_force_g

    @property
    def unit(self): return 'counts' if self.trigger_mode == 'raw' else 'g'


@dataclass(frozen=True)
class ForceSample:
    sequence: int
    observed_at: float
    source_time: float
    counts: float


class ContactDetected(RuntimeError):
    """A normal contact event: cancel the current tiny step and measure at rest."""


class ContactDetector:
    def __init__(self, baseline, counts_per_gram, options):
        if not finite(baseline): raise ValueError('A finite unloaded baseline is required')
        if options.trigger_mode == 'grams' and (not finite(counts_per_gram) or counts_per_gram <= 0):
            raise ValueError('A calibrated positive counts_per_gram is required')
        self.baseline, self.options = baseline, options
        self.scale = counts_per_gram if options.trigger_mode == 'grams' else 1.
        self.last_sequence = -1
        self.consecutive = 0
        self.triggered = False
        self.peak_load = 0.

    def load(self, counts): return (counts-self.baseline)/self.scale

    def update(self, samples):
        for sample in samples:
            if sample.sequence <= self.last_sequence: continue
            self.last_sequence = sample.sequence
            load = abs(self.load(sample.counts))
            self.peak_load = max(self.peak_load, load)
            if load >= self.options.maximum:
                raise RuntimeError('Load-cell overload: %.1f %s exceeds %.1f %s' %
                                   (load,self.options.unit,self.options.maximum,self.options.unit))
            self.consecutive = self.consecutive+1 if load >= self.options.threshold else 0
            if self.consecutive >= self.options.consecutive_samples: self.triggered = True
        return self.triggered


def stable_baseline(samples, sensor, options, reference_counts=None):
    counts = [s.counts for s in samples]
    if len(counts) < 10: raise ValueError('Too few load-cell samples for a baseline')
    scale = sensor.get('counts_per_gram') if options.trigger_mode == 'grams' else 1.
    if not finite(scale) or scale <= 0: raise ValueError('Invalid load-cell calibration scale')
    if options.trigger_mode == 'grams': reference_counts = sensor.get('reference_tare_counts')
    baseline = statistics.median(counts)
    if options.trigger_mode == 'raw' and (
            min(counts)-ADC_MIN_COUNTS <= options.maximum or ADC_MAX_COUNTS-max(counts) <= options.maximum):
        raise ValueError('ALPS ADC is saturated or has insufficient raw-count headroom')
    if (max(counts)-min(counts))/scale > options.threshold*.25:
        raise ValueError('Load cell is too noisy or still moving; baseline rejected')
    if reference_counts is not None and abs(baseline-reference_counts)/scale >= options.threshold:
        raise ValueError('Nozzle may already be loaded at hover; unload/check calibration before probing')
    return baseline


def validate_printer(status, cell, options, require_calibration=True):
    """False is read-only sensor diagnosis; raw motion still passes True here."""
    sensor = status.get(cell,{})
    if not sensor: raise RuntimeError('Klipper load-cell object not found: '+cell)
    if status.get('webhooks',{}).get('state') != 'ready':
        raise RuntimeError('Klipper is not ready')
    if require_calibration and options.trigger_mode == 'grams' and (sensor.get('is_calibrated') is not True or
            not finite(sensor.get('counts_per_gram')) or sensor['counts_per_gram'] <= 0 or
            not finite(sensor.get('reference_tare_counts'))):
        raise RuntimeError('ALPS is not calibrated: run LOAD_CELL_CALIBRATE LOAD_CELL='+cell+
                           ' with a known load, then ACCEPT and SAVE_CONFIG before probing')
    if sensor.get('errors',0) or sensor.get('overflows',0):
        raise RuntimeError('Klipper reports load-cell ADC errors or overflows')
    sync = status.get('axisforge_sync',{})
    if sync.get('leased') or sync.get('preparing') or sync.get('reservation'):
        raise RuntimeError('A synchronized job owns Klipper')
    if status.get('virtual_sdcard',{}).get('is_active') or status.get('print_stats',{}).get('state') in ('printing','paused'):
        raise RuntimeError('Klipper has an active or paused print')
    if require_calibration:
        for name in ('extruder','heater_bed'):
            heater = status.get(name,{})
            if heater.get('target',0) != 0: raise RuntimeError('Turn heater targets off before bed probing')
        temperature = status.get('extruder',{}).get('temperature')
        if not finite(temperature) or temperature > options.max_nozzle_temp_c:
            raise RuntimeError('Wait for the nozzle to cool below %g C' % options.max_nozzle_temp_c)
    return sensor


class AlpsStream:
    """Fail-closed reader for the existing ALPS bridge's newline JSON stream.

    Local receipt time, source progression and batch age are all checked. Once
    started, errors are latched; reconnection requires a new probing session.
    """
    def __init__(self, moonraker_url, alps_url=None, cell='load_cell_probe', api_key=None,
                 options=None, require_calibration=True, clock=time.monotonic):
        self.options = options or ContactOptions()
        parts = urlsplit(moonraker_url)
        if parts.scheme not in ('http','https') or not parts.hostname or parts.username or parts.password:
            raise ValueError('Invalid Moonraker URL')
        if alps_url is None:
            host = '['+parts.hostname+']' if ':' in parts.hostname else parts.hostname
            alps_url = urlunsplit((parts.scheme,host+':7126','','',''))
        alps_parts = urlsplit(alps_url)
        if (alps_parts.scheme not in ('http','https') or not alps_parts.hostname or
                alps_parts.username or alps_parts.password):
            raise ValueError('Invalid ALPS URL')
        self.moonraker_url, self.alps_url = moonraker_url.rstrip('/'), alps_url.rstrip('/')
        self.cell, self.api_key, self.clock = cell, api_key, clock
        self.require_calibration = require_calibration
        self.lock = threading.Lock()
        self.samples = deque(maxlen=6000)
        self.error = None
        self.sensor = None
        self.health_at = self.received_at = None
        self.sequence = 0
        self.last_source = None
        self.min_offset = None
        self.stop_event = threading.Event()
        self.response = None
        self.threads = []

    def _read_status(self):
        names = [self.cell,'webhooks','axisforge_sync','extruder','heater_bed','virtual_sdcard','print_stats']
        query = urlencode({name:'' for name in names})
        headers = {'X-Api-Key':self.api_key} if self.api_key else {}
        with urlopen(Request(self.moonraker_url+'/printer/objects/query?'+query,headers=headers),timeout=.75) as response:
            status = json.load(response)['result']['status']
        sensor = validate_printer(status,self.cell,self.options,self.require_calibration)
        with self.lock:
            if self.sensor is not None and self.require_calibration and self.options.trigger_mode == 'grams':
                if any(sensor.get(k)!=self.sensor.get(k) for k in ('counts_per_gram','reference_tare_counts')):
                    raise RuntimeError('Load-cell calibration changed during probing')
            self.sensor, self.health_at = dict(sensor), self.clock()

    def start(self):
        self._read_status()  # Calibration/interlocks are checked before robot creation.
        for target in (self._stream,self._health):
            thread = threading.Thread(target=target,daemon=True)
            self.threads.append(thread);thread.start()
        return self

    def refresh_status(self):
        self._read_status()
        self.snapshot()

    def _fail(self, error):
        with self.lock:
            if self.error is None: self.error = str(error)

    def _health(self):
        while not self.stop_event.wait(.25):
            try:self._read_status()
            except Exception as error:self._fail(error);return

    def _stream(self):
        try:
            with urlopen(self.alps_url+'/stream?'+urlencode({'cell':self.cell}),timeout=1.) as response:
                self.response = response
                while not self.stop_event.is_set():
                    line = response.readline(1024*1024)
                    if not line: raise RuntimeError('ALPS sample stream disconnected')
                    if len(line) >= 1024*1024: raise RuntimeError('ALPS sample batch exceeds limit')
                    if line.strip():self.ingest(json.loads(line))
        except Exception as error:
            if not self.stop_event.is_set():self._fail(error)

    def ingest(self, batch):
        now = self.clock()
        if not isinstance(batch,dict) or batch.get('error'):
            raise RuntimeError('ALPS stream error: '+str(batch.get('error') if isinstance(batch,dict) else batch))
        if batch.get('errors') not in (None,0): raise RuntimeError('ALPS stream reports ADC errors')
        rows, source_now = batch.get('rows'), batch.get('now')
        if not isinstance(rows,list) or not rows or not finite(source_now):
            raise RuntimeError('Invalid ALPS sample batch')
        if not all(isinstance(r,list) and len(r)>=3 and finite(r[0]) and finite(r[1]) for r in rows):
            raise RuntimeError('Invalid ALPS sample values')
        if any(not ADC_MIN_COUNTS < r[1] < ADC_MAX_COUNTS for r in rows):
            raise RuntimeError('ALPS ADC sample is saturated or outside its range')
        if not -.05 <= source_now-rows[-1][0] <= self.options.max_sample_age_s:
            raise RuntimeError('ALPS is delivering old or invalid samples')
        offset = now-source_now
        with self.lock:
            if self.min_offset is not None and (offset-self.min_offset > self.options.max_sample_age_s or offset<self.min_offset-.1):
                raise RuntimeError('ALPS delivery stalled or source clock changed')
            self.min_offset = offset if self.min_offset is None else min(self.min_offset,offset)
            for stamp,counts,*_ in rows:
                if self.last_source is not None and stamp <= self.last_source:
                    raise RuntimeError('ALPS sample timestamps repeated or moved backwards')
                if self.last_source is not None and stamp-self.last_source > .05:
                    raise RuntimeError('ALPS samples have a gap greater than 50 ms')
                self.sequence += 1
                self.samples.append(ForceSample(self.sequence,min(now,now+stamp-source_now),stamp,float(counts)))
                self.last_source = stamp
            self.received_at = now

    def snapshot(self):
        with self.lock:
            now = self.clock()
            if self.error: raise RuntimeError(self.error)
            if self.health_at is None or now-self.health_at > 1.:
                raise RuntimeError('Klipper sensor/idle status is stale')
            if self.received_at is None or now-self.received_at > self.options.max_sample_age_s:
                raise RuntimeError('ALPS load-cell samples are stale or missing')
            recent = [s for s in self.samples if s.observed_at >= now-1.1]
            if len(recent) >= 20 and recent[-1].observed_at-recent[0].observed_at >= 1.:
                if all(s.counts == recent[0].counts for s in recent):
                    raise RuntimeError('ALPS ADC values are frozen despite advancing timestamps')
            return list(self.samples)

    def wait_window(self, after, pump=lambda:time.sleep(.01), timeout=3.):
        deadline = self.clock()+timeout
        while self.clock() < deadline:
            with self.lock:
                if self.error: raise RuntimeError(self.error)
                ready = self.received_at is not None
            if ready:
                samples = [s for s in self.snapshot() if s.observed_at >= after+self.options.settle_s]
                if len(samples)>=10 and samples[-1].source_time-samples[0].source_time >= self.options.window_s:
                    return samples
            pump()
        raise RuntimeError('Timed out waiting for settled, fresh ALPS samples')

    def close(self):
        self.stop_event.set()
        # The bounded socket timeout lets the read thread exit without waiting
        # on BufferedReader.close() while another thread owns its read lock.
        for thread in self.threads:thread.join(1.1)


def repeat_contact(readings, tolerance):
    if len(readings)<2: raise ValueError('Repeated contact measurements are required')
    heights = [float(r['nozzle'][2]) for r in readings]
    spread = max(heights)-min(heights)
    if spread > tolerance:
        raise ValueError('Contact repeatability %.4f mm exceeds %.4f mm; point not saved' % (spread,tolerance))
    return sorted(readings,key=lambda r:r['nozzle'][2])[len(readings)//2], spread
