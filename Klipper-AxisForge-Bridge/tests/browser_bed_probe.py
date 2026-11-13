"""Dashboard homing checks with every request intercepted; no hardware I/O."""
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
    motion = {'policy':'exact','square_corner_velocity':1.,'junction_deviation_mm':.01,
              'blend_tolerance_mm':.05,'blend_speed_mm_s':5.}
    sync = {'state':'idle','busy':False,'control_ready':True,'robot_ready':True,
            'configured_heaters':['extruder'],'corner_policies':['exact'],
            'motion_defaults':motion,'clock_options_supported':True,'clock_profiles':profiles(),
            'clock_defaults':clock_options(),'clock_options':clock_options(),
            'clock_guidance_distance_mm':.1,'job':None,'checks':[]}
    probe = {'state':'idle','busy':False,'configured':True,'confirmation':None,'dry_run':False,
             'progress':0.,'current_point':0,'total_points':9,'message':'Place the bed before homing.'}
    requests, errors, cases = [], [], []
    unavailable = False
    drop_confirmation = False

    def route(handler):
        nonlocal unavailable, drop_confirmation
        request = handler.request
        parsed = urlsplit(request.url); path = parsed.path
        if parsed.netloc != 'probe-ui.test': handler.abort(); return
        if path == '/' or path.startswith('/static/'):
            file = ROOT/'bridge_dashboard'/'static'/('index.html' if path == '/' else path.split('/')[-1])
            mime = {'.html':'text/html','.js':'application/javascript','.css':'text/css'}[file.suffix]
            handler.fulfill(body=file.read_text(),content_type=mime); return
        if request.method == 'POST':
            data = request.post_data_json
            requests.append((path, data))
            if path == '/api/bed-probe/start':
                probe.update(state='waiting_surface',busy=True,dry_run=data['dry_run'],
                    confirmation={'stage':'surface','token':'a'*24},position_mm=[-700.,-300.,110.],
                    trigger_mode='raw',trigger_value=20000,trigger_unit='counts',max_value=60000,
                    message='Confirm an unloaded nozzle with a flat surface underneath.')
            elif path == '/api/bed-probe/confirm':
                if drop_confirmation:
                    drop_confirmation=False;handler.abort('connectionfailed');return
                assert data == probe['confirmation'], 'Wrong or stale confirmation sent'
                if data['stage'] == 'surface':
                    probe.update(state='waiting_front_left',progress=.2,
                        confirmation={'stage':'front_left','token':'b'*24},
                        first_point=[30.,30.],
                        position_mm=[-737.,-380.,120.],message='Confirm the front-left mesh point.')
                else: probe.update(state='meshing',confirmation=None,progress=.3,current_point=1)
            elif path == '/api/bed-probe/abort':
                probe.update(state='aborted',busy=False,confirmation=None,message='Probe stopped.')
            else: raise AssertionError('Unexpected POST '+path)
            result = probe
        elif path == '/api/bed-probe/status':
            if unavailable: handler.fulfill(status=503,body='{"detail":"offline"}',content_type='application/json');return
            result = probe
        elif path == '/api/status':
            result = {'klipper':{'connected':True,'state':'ready'},
                      'ros':{'connected':True,'joint_state_fresh':True},'axisforge':{'running':False}}
        elif path == '/api/sync/status': result = sync
        elif path == '/api/jobs': result = {'jobs':['part.gcode']}
        elif path.endswith('/preflight'): result = {'state':'staged'}
        elif path == '/api/links': result = {'alps_logger':'http://probe-ui.test/'}
        elif path == '/api/klipper/telemetry': result = {}
        else: handler.fulfill(status=404,body='{}'); return
        handler.fulfill(body=json.dumps(result),content_type='application/json')

    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(executable_path=shutil.which('google-chrome'),
            headless=False,args=['--headless=new'],chromium_sandbox=True)
        context = browser.new_context(viewport={'width':1280,'height':1050})
        context.route('**/*',route)
        page = context.new_page(); page.on('pageerror',lambda error:errors.append(str(error)))
        page.goto('http://probe-ui.test/')
        expect(page.locator('.workflow-steps li').first).to_contain_text('Setup & calibration')
        page.click('#dashboard-tab'); page.select_option('#job-select','part.gcode'); page.click('#setup-tab')
        expect(page.locator('#bed-probe-start')).to_be_enabled()
        expect(page.locator('#preflight-both')).to_be_enabled()
        cases.append('first checklist action; existing bed map does not force a new probe')
        page.click('#bed-probe-start')
        expect(page.locator('#bed-probe-confirmation')).to_be_visible()
        expect(page.locator('#bed-probe-confirm-detail')).to_contain_text('clear of the surface and unloaded')
        expect(page.locator('#bed-probe-sensor')).to_contain_text('20,000 counts')
        expect(page.locator('#bed-probe-sensor')).to_contain_text('no conversion to grams')
        assert [r[0] for r in requests] == ['/api/bed-probe/start']
        expect(page.locator('#preflight-both')).to_be_disabled()
        expect(page.locator('#upload-job')).to_be_disabled()
        expect(page.locator('#sync-start')).to_be_disabled()
        cases.append('start waits for explicit surface confirmation and blocks job controls')
        expect(page.locator('#bed-probe-confirm')).to_be_enabled()
        page.evaluate('document.getElementById("bed-probe-confirm").click(); document.getElementById("bed-probe-confirm").click()')
        expect(page.locator('#bed-probe-confirm-title')).to_contain_text('front-left')
        assert len([r for r in requests if r[0].endswith('/confirm')]) == 1
        expect(page.locator('#bed-probe-confirm-detail')).to_contain_text('front-leftmost position')
        expect(page.locator('#bed-probe-position')).to_contain_text('G-code: X 30.00, Y 30.00')
        cases.append('double click sends one stage-bound confirmation; front-left has a separate prompt')
        expect(page.locator('#bed-probe-confirm')).to_be_enabled()
        page.click('#bed-probe-confirm')
        expect(page.locator('#bed-probe-confirmation')).to_be_hidden()
        expect(page.locator('#bed-probe-badge')).to_have_text('Measuring mesh')
        cases.append('mesh begins only after second explicit confirmation')
        probe.update(state='complete',progress=1.,current_point=9,message='Mesh saved; returning cleanup.')
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#preflight-both')).to_be_disabled()
        cases.append('completion event keeps controls locked while child is still busy')
        probe.update(busy=False)
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#preflight-both')).to_be_enabled()
        expect(page.locator('#sync-start')).to_be_disabled()
        cases.append('successful mesh requires fresh job preflight')
        # A fresh run must use fresh tokens, including after page reload.
        page.reload(); page.click('#dashboard-tab'); page.select_option('#job-select','part.gcode'); page.click('#setup-tab')
        page.click('#bed-probe-simulate')
        expect(page.locator('#bed-probe-confirm-detail')).to_contain_text('simulation')
        expect(page.locator('#bed-probe-message')).to_contain_text('Simulation:')
        drop_confirmation=True
        expect(page.locator('#bed-probe-confirm')).to_be_enabled()
        page.click('#bed-probe-confirm')
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-confirm')).to_be_enabled()
        expect(page.locator('#bed-probe-confirm-title')).to_contain_text('surface')
        cases.append('dropped confirmation permits explicit retry only after fresh pending server status')
        page.click('#bed-probe-abort')
        expect(page.locator('#bed-probe-badge')).to_have_text('Stopped')
        cases.append('simulation is clearly labeled and can be aborted at confirmation')
        unavailable = True
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-start')).to_be_disabled()
        expect(page.locator('#bed-probe-message')).to_contain_text('offline')
        cases.append('unavailable probe service cannot launch motion')
        unavailable = False
        page.evaluate('refreshBedProbe()')
        page.set_viewport_size({'width':390,'height':844})
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth+2')
        cases.append('mobile layout fits viewport')
        assert not errors, errors
        print(json.dumps({'passed':len(cases),'cases':cases,'page_errors':errors},indent=2))
        browser.close()


if __name__ == '__main__': main()
