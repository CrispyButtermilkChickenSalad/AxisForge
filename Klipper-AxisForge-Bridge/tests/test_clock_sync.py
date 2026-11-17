"""Independent host oscillators, transport asymmetry and clock discontinuities."""
import asyncio
import math
from types import SimpleNamespace
import unittest

from bridge_dashboard.clock_sync import HostRateEstimator
from bridge_dashboard.sync_clients import sample_clock, ParticipantError, ClockMap


def reply(t, host_rate=.999996, mcu_rate=1.000036, epoch='boot-a'):
    raw = 500.+t*host_rate
    return dict(eventtime=raw, print_time=1000.+raw*mcu_rate,
                mcu_raw_clock_rate=mcu_rate, clock_epoch=epoch,
                print_time_rate=1.000022)  # MCU/local host; deliberately NOT MCU/PC.


def warm(estimator, end=120.):
    for i in range(int(end)+1):
        t = float(i)
        jitter = .00008*math.sin(i*1.7)
        estimator.observe(t-.003, t+.003, reply(t+jitter))


class HostRateTests(unittest.TestCase):
    def test_independent_host_rate_and_low_rtt_binning(self):
        estimator = HostRateEstimator()
        for i in range(181):
            t = float(i)
            estimator.observe(t-.003, t+.003, reply(t+.00006*math.sin(i)))
            # Large, asymmetric queue stalls must not skew the oscillator fit.
            estimator.observe(t+.1, t+.17, reply(t+.103))
        fit = estimator.estimate(180.2)
        self.assertAlmostEqual(fit.rate, .999996, delta=.000001)
        self.assertLess(fit.error_ppm, 1.)
        self.assertLessEqual(len(estimator.points), 91)

    def test_not_ready_during_warmup_or_after_gap_restart_and_backward_clock(self):
        for reset in ('gap', 'epoch', 'backward'):
            with self.subTest(reset=reset):
                estimator = HostRateEstimator()
                warm(estimator)
                estimator.estimate(120.1)
                t = 140. if reset == 'gap' else 121.
                value = reply(t, epoch='boot-b' if reset == 'epoch' else 'boot-a')
                if reset == 'backward': value['eventtime'] = 1.
                estimator.observe(t-.003, t+.003, value)
                with self.assertRaisesRegex(ValueError, 'warming up'):
                    estimator.estimate(t+.01)
        with self.assertRaisesRegex(ValueError, 'stale'):
            estimator.estimate(t+6.)

    def test_unstable_host_or_network_is_rejected(self):
        for noise, rate in ((.004, .999996), (0., 1.002)):
            estimator = HostRateEstimator()
            for i in range(121):
                t = float(i)
                estimator.observe(t-.005, t+.005, reply(t+noise*math.sin(i),host_rate=rate))
            with self.assertRaisesRegex(ValueError, 'unstable'):
                estimator.estimate(120.1)

    def test_lost_low_latency_samples_expire_the_fit(self):
        estimator = HostRateEstimator();warm(estimator)
        for i in range(121, 130):
            estimator.observe(i, i+.1, reply(i+.05))
        with self.assertRaisesRegex(ValueError, 'stale'):
            estimator.estimate(129.2)

    def test_isolated_midpoint_outlier_does_not_corrupt_rate_fit(self):
        estimator=HostRateEstimator();warm(estimator)
        estimator.observe(122.-.004,122.+.004,reply(122.+.003))
        estimator.observe(124.-.003,124.+.003,reply(124.))
        fit=estimator.estimate(124.1)
        self.assertGreaterEqual(fit.rejected_samples,1)
        self.assertAlmostEqual(fit.rate,.999996,delta=.000001)


class ClockMappingTests(unittest.IsolatedAsyncioTestCase):
    async def test_bad_rate_fit_blocks_scheduling_but_keeps_running_phase_guard(self):
        estimator=HostRateEstimator()  # Deliberately not warmed up.
        original=ClockMap(1000.+500.*1.000036,.004,0.,.999996*1.000036,
                          epoch='boot-a')
        async def measure(reference,phase=0.):
            async def call(action):
                value=reply(121.);value['print_time']+=phase;return value
            times=iter([120.997,121.003])
            return await sample_clock(SimpleNamespace(call=call,clock_estimator=estimator),
                clock=lambda:next(times),samples=1,reference=reference)
        with self.assertRaisesRegex(ParticipantError,'warming up'):
            await measure(None)
        current=await measure(original)
        self.assertIn('warming up',current.rate_diagnostic)
        self.assertAlmostEqual(current.offset,original.offset_at(current.sampled_at),delta=.000001)
        shifted=await measure(original,.02)
        self.assertGreater(abs(shifted.offset-original.offset_at(shifted.sampled_at)),.0199)

    async def test_clock_restart_is_not_hidden_by_an_existing_rate(self):
        estimator=HostRateEstimator();warm(estimator)
        async def call(action):return reply(121.,epoch='new-boot')
        times=iter([120.997,121.003])
        with self.assertRaisesRegex(ParticipantError,'restarted'):
            await sample_clock(SimpleNamespace(call=call,clock_estimator=estimator),
                clock=lambda:next(times),samples=1,reference=ClockMap(1.,.004,120.,epoch='boot-a'))

    async def test_combined_rate_preserves_full_print_timeline(self):
        estimator = HostRateEstimator();warm(estimator)
        pairs = [(121.+i*.1,121.006+i*.1) for i in range(5)]
        responses = iter(reply((s+r)/2) for s,r in pairs)
        async def call(action): return next(responses)
        clock = iter(v for pair in pairs for v in pair)
        mapping = await sample_clock(SimpleNamespace(call=call,clock_estimator=estimator),
            clock=lambda:next(clock),interval=0)
        rate = .999996*1.000036
        self.assertAlmostEqual(mapping.rate, rate, delta=.000001)
        self.assertEqual(mapping.epoch, 'boot-a')
        for t in (123., 903.):
            self.assertAlmostEqual(mapping.local_time(reply(t)['print_time'],now=122.),t,delta=.001)
        self.assertGreater(abs(mapping.rate-1.000022)*780.,.007)

    async def test_opposite_transport_asymmetries_tighten_a_valid_interval(self):
        times = iter([10.,10.006,10.1,10.106])
        responses = iter([{'print_time':110.001}, {'print_time':110.105}])
        async def call(action): return next(responses)
        mapping = await sample_clock(SimpleNamespace(call=call),clock=lambda:next(times),
                                    samples=2,interval=0)
        self.assertAlmostEqual(mapping.offset,100.)
        self.assertAlmostEqual(mapping.uncertainty,.002)

    async def test_inconsistent_phase_samples_never_become_a_good_clock(self):
        times = iter([10.,10.006,10.1,10.106])
        responses = iter([{'print_time':110.003}, {'print_time':110.123}])
        async def call(action): return next(responses)
        with self.assertRaisesRegex(ParticipantError,'Inconsistent'):
            await sample_clock(SimpleNamespace(call=call),clock=lambda:next(times),samples=2,interval=0)

    async def test_rate_fit_does_not_smooth_away_a_phase_jump(self):
        estimator=HostRateEstimator();warm(estimator)
        async def mapping_at(t,jump):
            async def call(action):
                value=reply(t);value['print_time']+=jump;return value
            times=iter([t-.003,t+.003])
            return await sample_clock(SimpleNamespace(call=call,clock_estimator=estimator),
                clock=lambda:next(times),samples=1)
        baseline=await mapping_at(121.,0.)
        current=await mapping_at(123.,.02)
        self.assertGreater(abs(current.offset-baseline.offset_at(current.sampled_at)),.0199)

    async def test_combined_rate_must_fit_the_worker_checked_margin(self):
        estimator=HostRateEstimator()
        for i in range(121):
            estimator.observe(i-.003,i+.003,reply(i,host_rate=1.0008,mcu_rate=1.0008))
        async def call(action):return reply(121.,host_rate=1.0008,mcu_rate=1.0008)
        times=iter([120.997,121.003])
        with self.assertRaisesRegex(ParticipantError,'trajectory margin'):
            await sample_clock(SimpleNamespace(call=call,clock_estimator=estimator),
                               clock=lambda:next(times),samples=1)


if __name__ == '__main__': unittest.main()
