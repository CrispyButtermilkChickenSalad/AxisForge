"""Dashboard Z positioning and recovery; every HTTP request is intercepted."""
import json
from pathlib import Path
import shutil
import sys
from urllib.parse import urlsplit

from playwright.sync_api import sync_playwright, expect

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bridge_dashboard.clock_settings import clock_options, profiles


def main():
    motion = dict(policy='exact', square_corner_velocity=1., junction_deviation_mm=.01,
                  blend_tolerance_mm=.05, blend_speed_mm_s=5.)
    sync = dict(state='idle', busy=False, control_ready=True, robot_ready=True,
                configured_heaters=['extruder'], corner_policies=['exact'], motion_defaults=motion,
                clock_options_supported=True, clock_profiles=profiles(), clock_defaults=clock_options(),
                clock_options=clock_options(), clock_guidance_distance_mm=.1, checks=[])
    routines = {name: dict(state='idle', busy=False, configured=True, confirmation=None,
                           action_token=None, z_controls=None, message='Ready')
                for name in ('bed-probe', 'load-calibration')}
    requests, errors, cases = [], [], []
    unavailable, drop_action = set(), set()
    counter = [0]

    def token():
        counter[0] += 1
        return '%032x' % counter[0]

    def position(name, recovery=False, loaded=False):
        routines[name].update(state='recovering_z' if recovery else 'positioning_z',
            busy=True, action_token=token(), confirmation=None, z_controls=dict(
                phase='recovery' if recovery else 'positioning', position_mm=[1., 2., 110.],
                lower_z_mm=60., upper_z_mm=160., delta_counts=25000. if loaded else 0.,
                loaded=loaded, upward_only=recovery or loaded, can_continue=not recovery and not loaded,
                can_finish=recovery and not loaded, step_sizes=[.001, .01, .05, .1, .5, 1., 10.],
                failure='Contact height was rejected.' if recovery else ''))

    def route(handler):
        req = handler.request
        parsed = urlsplit(req.url)
        path = parsed.path
        if parsed.netloc != 'z-controls.test':
            handler.abort()
            return
        if path == '/' or path.startswith('/static/'):
            file = ROOT / 'bridge_dashboard/static' / ('index.html' if path == '/' else path.split('/')[-1])
            handler.fulfill(body=file.read_text(), content_type={'.html': 'text/html', '.js': 'application/javascript', '.css': 'text/css'}[file.suffix])
            return
        name = next((name for name in routines if path.startswith('/api/'+name+'/')), None)
        if req.method == 'POST':
            data = req.post_data_json
            requests.append((path, data))
            assert name and path.endswith('/action'), 'Unexpected POST: '+path
            if name in drop_action:
                drop_action.remove(name)
                handler.abort('connectionfailed')
                return
            body = routines[name]
            assert data['token'] == body['action_token'], 'Stale token sent'
            if data['command'] == 'step':
                body['z_controls']['position_mm'][2] += data['delta_mm']
                body['action_token'] = token()
            elif data['command'] == 'continue':
                body.update(state='homing_fast' if name == 'bed-probe' else 'finding_scale',
                            action_token=None, z_controls=None)
            elif data['command'] == 'finish_recovery':
                body.update(state='failed', busy=False, action_token=None, z_controls=None,
                            message='Contact failed; recovery finished. Start a fresh attempt.')
            else:
                raise AssertionError('Unexpected action '+data['command'])
            result = body
        elif name and path.endswith('/status'):
            if name in unavailable:
                handler.fulfill(status=503, body='{}')
                return
            result = routines[name]
        elif path == '/api/robot/connection':
            result = dict(available=True, configured=True, busy=False, ready=True, state='ready', checks=[])
        elif path == '/api/status':
            result = dict(klipper=dict(connected=True, state='ready'), ros=dict(connected=True, joint_state_fresh=True), axisforge=dict(running=False))
        elif path == '/api/sync/status':
            result = sync
        elif path == '/api/jobs':
            result = dict(jobs=['part.gcode'])
        elif path.endswith('/preflight'):
            result = dict(state='staged')
        elif path == '/api/links':
            result = dict(alps_logger='http://logger.invalid/')
        elif path == '/api/klipper/telemetry':
            result = {}
        else:
            handler.fulfill(status=404, body='{}')
            return
        handler.fulfill(body=json.dumps(result), content_type='application/json')

    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(executable_path=shutil.which('google-chrome'),
            headless=False, args=['--headless=new'], chromium_sandbox=True)
        context = browser.new_context(viewport=dict(width=1280, height=1000))
        context.route('**/*', route)
        page = context.new_page()
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto('http://z-controls.test/')
        for name, prefix, attribute, refresh in (
                ('bed-probe', 'bed-z', 'data-bed-z-step', 'refreshBedProbe()'),
                ('load-calibration', 'load-z', 'data-calibration-step', 'refreshSetup()')):
            def button(delta):
                return page.locator('[%s="%g"]' % (attribute, delta))
            for item in page.locator('['+attribute+']').all():
                expect(item).to_be_disabled()
            position(name)
            page.evaluate(refresh)
            expect(button(-10)).to_be_enabled()
            expect(page.locator('#'+prefix+'-continue')).to_be_enabled()
            expect(page.locator('#'+prefix+'-finish')).to_be_hidden()
            expect(page.locator('#preflight-both')).to_be_disabled()
            for delta in (-10, 10, -1, 1, -.1, .1, -.05, .05):
                before = len(requests)
                button(delta).evaluate('(button) => { button.click(); button.click(); }')
                expect(button(delta)).to_be_enabled()
                assert len(requests) == before+1
                assert requests[-1][0] == '/api/'+name+'/action'
                assert requests[-1][1]['command'] == 'step' and requests[-1][1]['delta_mm'] == delta
            cases.append(name+': eight Z buttons issue one bounded request per double click')
            drop_action.add(name)
            before = len(requests)
            button(.05).click()
            expect(button(.05)).to_be_enabled()
            assert len(requests) == before+1
            button(.05).click()
            expect(button(.05)).to_be_enabled()
            assert len(requests) == before+2
            cases.append(name+': dropped request requires fresh status and another explicit click')
            if name == 'load-calibration':
                expect(page.locator('#load-calibration-record')).to_be_disabled()
                expect(page.locator('#load-calibration-finish')).to_be_disabled()
            page.locator('#'+prefix+'-continue').click()
            expect(button(.05)).to_be_disabled()
            assert requests[-1][1]['command'] == 'continue'
            assert 'delta_mm' not in requests[-1][1]
            cases.append(name+': Continue starts the search and closes manual controls')
            position(name, recovery=True, loaded=True)
            page.evaluate(refresh)
            expect(button(.1)).to_be_enabled()
            for delta in (10, 1, -10, -1, -.1, -.05):
                expect(button(delta)).to_be_disabled()
            expect(page.locator('#'+prefix+'-continue')).to_be_hidden()
            expect(page.locator('#'+prefix+'-finish')).to_be_disabled()
            before = len(requests)
            button(-.05).dispatch_event('click')
            assert len(requests) == before
            button(.05).click()
            expect(button(.05)).to_be_enabled()
            assert requests[-1][1]['delta_mm'] == .05
            cases.append(name+': loaded recovery permits only small upward moves')
            routines[name]['z_controls']['release_blocked'] = True
            page.evaluate(refresh)
            expect(button(.05)).to_be_disabled()
            expect(button(.1)).to_be_disabled()
            expect(page.locator('#'+prefix+'-finish')).to_be_disabled()
            before = len(requests)
            button(.05).dispatch_event('click')
            assert len(requests) == before
            routines[name]['z_controls']['release_blocked'] = False
            page.evaluate(refresh)
            expect(button(.05)).to_be_enabled()
            assert len(requests) == before
            cases.append(name+': load rise blocks steps without closing recovery or automatically retrying')

            body = routines[name]
            body.update(state='z_stepping', action_token=None, z_controls=None)
            page.evaluate(refresh)
            expect(button(.05)).to_be_disabled()
            position(name, recovery=True, loaded=False)
            page.evaluate(refresh)
            expect(button(10)).to_be_enabled()
            expect(button(-.05)).to_be_disabled()
            expect(page.locator('#'+prefix+'-finish')).to_be_enabled()
            unavailable.add(name)
            page.evaluate(refresh)
            expect(button(.05)).to_be_disabled()
            unavailable.remove(name)
            page.evaluate(refresh)
            expect(button(.05)).to_be_enabled()
            body['z_controls']['position_mm'][2] = body['z_controls']['upper_z_mm']-.1
            page.evaluate(refresh)
            expect(button(1)).to_be_disabled()
            expect(button(.05)).to_be_enabled()
            page.locator('#'+prefix+'-finish').click()
            expect(button(.05)).to_be_disabled()
            assert requests[-1][1]['command'] == 'finish_recovery'
            assert 'delta_mm' not in requests[-1][1]
            cases.append(name+': release enables larger upward steps; finish, bounds and unavailable status close controls')
            if name == 'bed-probe':
                expect(page.locator('#bed-probe-retry')).to_be_enabled()
            # Stale action data must not re-enable a terminated child.
            position(name, recovery=True, loaded=False)
            body.update(state='failed', busy=False)
            page.evaluate(refresh)
            expect(button(.05)).to_be_disabled()
            body.update(action_token=None, z_controls=None)
            page.evaluate(refresh)
            cases.append(name+': stale recovery payload cannot move a terminated session')
        page.set_viewport_size(dict(width=390, height=844))
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth+1')
        assert not errors, errors
        print(json.dumps(dict(passed=len(cases), cases=cases, page_errors=errors, hardware_requests=0), indent=2))
        browser.close()


if __name__ == '__main__':
    main()
