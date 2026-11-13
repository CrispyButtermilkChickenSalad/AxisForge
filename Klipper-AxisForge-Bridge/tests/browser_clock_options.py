"""Optional Playwright UI checks. Every HTTP request is intercepted; no hardware I/O.

Run with Playwright and Chrome installed: python tests/browser_clock_options.py
"""

import json
import os
from pathlib import Path
import shutil
import sys
from urllib.parse import urlsplit

from playwright.sync_api import sync_playwright, expect

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bridge_dashboard.clock_settings import clock_options, profiles


def main():
    motion = {'policy':'exact','square_corner_velocity':1.,'junction_deviation_mm':.01,
              'blend_tolerance_mm':.05,'blend_speed_mm_s':5.}
    state = {'state':'idle','busy':False,'control_ready':True,'robot_ready':True,
             'configured_heaters':['extruder'], 'corner_policies':['exact','klipper','marlin','blend'],
             'motion_defaults':motion, 'clock_options_supported':True,'clock_profiles':profiles(),
             'clock_defaults':clock_options(),'clock_options':clock_options(),
             'clock_guidance_distance_mm':.1, 'job':None,'checks':[]}
    saved = {}
    requests = []
    errors = []
    cases = []

    def route(handler):
        request=handler.request
        path=urlsplit(request.url).path
        if urlsplit(request.url).netloc != 'clock-ui.test':
            handler.abort();return
        if path=='/' or path.startswith('/static/'):
            file=ROOT/'bridge_dashboard'/'static'/('index.html' if path=='/' else path.split('/')[-1])
            if not file.is_file():handler.fulfill(status=404);return
            mime={'.html':'text/html','.js':'application/javascript','.css':'text/css'}[file.suffix]
            handler.fulfill(body=file.read_text(),content_type=mime);return
        if request.method=='POST':
            payload=request.post_data_json
            requests.append((path,payload))
            if path.endswith('/preflight/both'):
                saved.update(payload)
                state.update(state='ready',busy=False,job='part.gcode',**payload)
            elif path=='/api/sync/start':state.update(state='running',busy=True,stage='motion')
            else:raise AssertionError('Unexpected POST: '+path)
            data=state
        elif path=='/api/status':
            data={'klipper':{'connected':True,'state':'ready'},
                  'ros':{'connected':True,'joint_state_fresh':True},'axisforge':{'running':False}}
        elif path=='/api/sync/status':data=state
        elif path=='/api/jobs':data={'jobs':['part.gcode']}
        elif path.endswith('/preflight'):
            data=dict(saved,state='preflight_passed' if saved else 'staged')
        elif path=='/api/links':data={'alps_logger':'http://clock-ui.test/'}
        elif path=='/api/klipper/telemetry':data={}
        else:handler.fulfill(status=404,body='{}');return
        handler.fulfill(content_type='application/json',body=json.dumps(data))

    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(executable_path=shutil.which('google-chrome'),
            headless=False,args=['--headless=new'],chromium_sandbox=True)
        context=browser.new_context(viewport={'width':1280,'height':1000})
        context.route('**/*',route)
        page=context.new_page();page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto('http://clock-ui.test/')
        expect(page.locator('#clock-profile option')).to_have_count(5)
        page.click('#dashboard-tab');page.select_option('#job-select','part.gcode');page.click('#setup-tab')
        expect(page.locator('#preflight-both')).to_be_enabled()
        expect(page.locator('#clock-profile')).to_have_value('standard');cases.append('default stays 10/10')

        for profile in profiles():
            page.select_option('#clock-profile',profile['id'])
            expect(page.locator('#clock-alignment')).to_have_value(str(int(profile['options']['max_alignment_error_ms'])))
            expect(page.locator('#clock-speed-note')).to_contain_text('%.2f mm/s' % profile['suggested_max_speed_mm_s'])
            assert page.locator('#clock-alignment').evaluate('(el) => el.readOnly')
            cases.append(profile['id']+' guidance and limits')

        page.select_option('#clock-profile','custom')
        for uncertainty,alignment in (('10','5'),('1','15'),('10','21'),('','15')):
            page.fill('#clock-uncertainty',uncertainty);page.fill('#clock-alignment',alignment)
            page.locator('#clock-alignment').dispatch_event('change')
            expect(page.locator('#preflight-both')).to_be_disabled()
            cases.append('invalid custom '+uncertainty+'/'+alignment)

        page.select_option('#clock-profile','wifi_trial')
        page.click('#dashboard-tab');page.click('#preflight-both');page.click('#setup-tab')
        expect(page.locator('#sync-start')).to_be_enabled()
        assert saved['clock_options']=={'max_uncertainty_ms':10,'max_alignment_error_ms':15}
        cases.append('preflight forwards limits and enables start')
        page.select_option('#clock-profile','slow_trial')
        expect(page.locator('#sync-start')).to_be_disabled()
        expect(page.locator('#sync-start-status')).to_contain_text('Clock timing differs')
        cases.append('changed limit blocks old preflight')
        page.select_option('#clock-profile','wifi_trial')
        expect(page.locator('#sync-start')).to_be_enabled();cases.append('restoring validated limits enables start')
        page.reload();page.click('#dashboard-tab');page.select_option('#job-select','part.gcode');page.click('#setup-tab')
        expect(page.locator('#clock-profile')).to_have_value('wifi_trial')
        expect(page.locator('#sync-start')).to_be_enabled();cases.append('selection persists across reload')

        state.update(clock_speed_guidance={'peak_xyz_speed_mm_s':20.,'timing_displacement_mm':.3,
                                         'exceeds_guidance':True},
                     clock_measurement={'uncertainty_ms':6.,'alignment_error_ms':13.,'age_s':3.})
        page.evaluate('refreshSync()')
        expect(page.locator('#clock-speed-note')).to_contain_text('Above the illustrative allowance')
        expect(page.locator('#clock-measurement')).to_contain_text('running alignment 13.00 ms')
        cases.append('planned speed warning and measured alignment visible')
        state['heartbeat']={'active':True,'lease_mode':'cold_preflight','last_reply_ms':501.,
                            'cold_retries':2,'max_event_loop_delay_ms':3.,'retrying':True}
        page.evaluate('refreshSync()')
        expect(page.locator('#heartbeat-measurement')).to_contain_text('cold retries 2')
        expect(page.locator('#heartbeat-measurement')).to_contain_text('Reconnecting within')
        state['heartbeat']['active']=False
        page.evaluate('refreshSync()')
        expect(page.locator('#heartbeat-measurement')).not_to_contain_text('Reconnecting within')
        cases.append('heartbeat retries visible without stale reconnect status')
        state['robot_heartbeat']={'active':True,'last_reply_ms':12.,'max_reply_ms':38.}
        page.evaluate('refreshSync()')
        expect(page.locator('#heartbeat-measurement')).to_contain_text('Robot heartbeat: strict control')
        expect(page.locator('#heartbeat-measurement')).to_contain_text('maximum 38.0 ms')
        cases.append('robot heartbeat latency is distinguished from Klipper')
        if os.getenv('AXISFORGE_UI_SCREENSHOT'):
            page.locator('#clock-options').scroll_into_view_if_needed()
            page.screenshot(path=os.environ['AXISFORGE_UI_SCREENSHOT'])
        page.click('#dashboard-tab');page.click('#sync-start');page.click('#setup-tab')
        expect(page.locator('#clock-profile')).to_be_disabled()
        expect(page.locator('#clock-alignment')).to_be_disabled()
        assert requests[-1][1]['clock_options']==saved['clock_options']
        cases.append('start forwards limits and locks controls')
        state.update(state='idle',busy=False,clock_options_supported=False,control_ready=False,
                     reason='Update the AxisForge worker to support clock timing options')
        page.evaluate('refreshSync()')
        expect(page.locator('#preflight-both')).to_be_disabled()
        expect(page.locator('#sync-start')).to_be_disabled();cases.append('old worker blocks execution')
        assert not errors, errors
        print(json.dumps({'cases':len(cases),'passed':cases,'javascript_errors':errors,
                          'real_hardware_requests':0},indent=2))
        browser.close()


if __name__=='__main__':main()
