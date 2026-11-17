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
    starts = 0

    def route(handler):
        nonlocal unavailable, drop_confirmation, starts
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
                starts += 1
                probe.update(state='waiting_surface',busy=True,dry_run=data['dry_run'],
                    confirmation={'stage':'surface','token':f'{starts:024x}'},position_mm=[-700.,-300.,110.],
                    trigger_mode='raw',trigger_value=20000,trigger_unit='counts',max_value=120000,
                    baseline_noise_fraction=.4 if data.get('thermal',{}).get('nozzle_target_c',0)>0 else .25,
                    message='Confirm an unloaded nozzle with a flat surface underneath.')
                probe['thermal_options']=data.get('thermal')
                probe.pop('thermal',None)
                if data.get('thermal'):
                    probe.update(state='waiting_temperature',confirmation=None,
                        thermal={'mode':'heated','stable_for_s':2,'required_stable_s':data['thermal']['soak_s']})
            elif path == '/api/bed-probe/confirm':
                if drop_confirmation:
                    drop_confirmation=False;handler.abort('connectionfailed');return
                assert data == probe['confirmation'], 'Wrong or stale confirmation sent'
                if data['stage'] == 'surface':
                    if probe.get('alignment',{}).get('required'):
                        probe['alignment']['state']='aligning'
                        probe.update(state='aligning_nozzle',confirmation=None,message='Aligning upright')
                    else:
                        probe.update(state='waiting_front_left',progress=.2,
                            confirmation={'stage':'front_left','token':f'{starts:023x}f'},
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
            result = {'klipper':{'connected':True,'state':'ready',
                                'extruder':{'temperature':210.,'target':210.},
                                'bed':{'temperature':60.,'target':60.} if 'heater_bed' in sync['configured_heaters'] else None},
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
        expect(page.locator('#bed-probe-recovery')).to_be_hidden()
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
        expect(page.locator('#bed-probe-start')).to_have_text('Redo homing & mesh')
        expect(page.locator('#bed-probe-start')).to_be_enabled()
        expect(page.locator('#preflight-both')).to_be_enabled()
        expect(page.locator('#sync-start')).to_be_disabled()
        cases.append('successful mesh requires fresh job preflight')
        probe['alignment']={'required':True,'state':'pending','angle_deg':.255,'maximum_deg':.5,'duration_s':3}
        page.click('#bed-probe-start')
        expect(page.locator('#bed-probe-confirm-title')).to_have_text('Confirm surface and upright alignment')
        expect(page.locator('#bed-probe-confirm-detail')).to_contain_text('0.255° upright correction')
        expect(page.locator('#bed-probe-confirm-detail')).to_contain_text('at least 3 seconds')
        expect(page.locator('#bed-probe-confirm')).to_have_text('Surface clear — align nozzle')
        expect(page.locator('#bed-probe-confirm')).to_be_enabled()
        previous_requests=len(requests)
        page.evaluate('document.getElementById("bed-probe-confirm").click(); document.getElementById("bed-probe-confirm").click()')
        expect(page.locator('#bed-probe-badge')).to_have_text('Aligning nozzle')
        assert len(requests)==previous_requests+1
        expect(page.locator('#bed-probe-confirmation')).to_be_hidden()
        for button in page.locator('[data-bed-z-step]').all():expect(button).to_be_disabled()
        expect(page.locator('#bed-z-status')).to_contain_text('Z controls enable after')
        expect(page.locator('#bed-probe-abort')).to_be_enabled()
        page.reload();page.click('#setup-tab')
        expect(page.locator('#bed-probe-badge')).to_have_text('Aligning nozzle')
        expect(page.locator('#bed-probe-confirmation')).to_be_hidden()
        assert len(requests)==previous_requests+1
        probe['alignment'].update(required=False,state='aligned',residual_deg=.001)
        probe.update(state='positioning_z',action_token='c'*24,
            z_controls={'phase':'positioning','position_mm':[-700,-300,110],
                'lower_z_mm':0,'upper_z_mm':150,'delta_counts':0,'upward_only':False,
                'loaded':False,'can_continue':True,'can_finish':False,'step_sizes':[10,1,.1,.05]})
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-alignment')).to_contain_text('remaining error 0.001°')
        expect(page.locator('#bed-z-continue')).to_be_enabled()
        page.click('#bed-probe-abort')
        probe.pop('alignment');probe.pop('z_controls');probe.pop('action_token')
        cases.append('alignment requires explicit confirmation; reload and double clicks do not repeat it; Z waits for verified correction')
        probe.update(state='recovering_home',busy=True,confirmation=None,message='Returning Home after failure',
            failure_recovery={'state':'returning','failure':'Repeatability failed'},last_failure='Repeatability failed')
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-badge')).to_have_text('Returning Home after failure')
        expect(page.locator('#bed-probe-home-recovery')).to_contain_text('Stop probing cancels recovery too')
        expect(page.locator('#bed-probe-abort')).to_be_enabled()
        expect(page.locator('#bed-probe-start')).to_be_disabled()
        for button in page.locator('[data-bed-z-step]').all():expect(button).to_be_disabled()
        probe.update(state='failed',busy=False)
        probe['failure_recovery']['state']='returned'
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-badge')).to_have_text('Failed')
        expect(page.locator('#bed-probe-home-recovery')).to_contain_text('arm returned to its established Home')
        expect(page.locator('#bed-probe-retry')).to_be_enabled()
        probe['failure_recovery'].update(state='blocked',reason='Robot feedback stale')
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-home-recovery')).to_contain_text('Robot feedback stale')
        probe.pop('failure_recovery')
        cases.append('automatic failure return blocks manual moves, retains Stop, and reports returned/blocked without marking mesh complete')
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
        expect(page.locator('#bed-probe-retry')).to_have_text('Retry simulation')
        expect(page.locator('#bed-probe-retry')).to_be_enabled()
        previous_requests = len(requests)
        page.click('#bed-probe-retry')
        expect(page.locator('#bed-probe-confirm-detail')).to_contain_text('simulation')
        assert requests[previous_requests:] == [('/api/bed-probe/start', {'dry_run':True})]
        expect(page.locator('#bed-probe-recovery')).to_be_hidden()
        cases.append('stopped simulation retries as simulation with fresh surface confirmation only')
        page.click('#bed-probe-abort')
        probe.update(state='failed',dry_run=False,busy=True,message='Load-cell overload',stop_unconfirmed=False)
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-retry')).to_have_text('Retry homing')
        expect(page.locator('#bed-probe-retry')).to_be_disabled()
        probe.update(busy=False)
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-retry')).to_be_enabled()
        expect(page.locator('#bed-probe-retry-guidance')).to_contain_text('clear of the surface and unloaded')
        expect(page.locator('#bed-probe-retry-guidance')).to_contain_text('current position')
        probe.update(last_failure='Load increased during upward release')
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-last-error')).to_contain_text('Load increased during upward release')
        page.evaluate("robotConnectionSnapshot = {available:true, ready:false, message:'Select Remote Control'}; updateBedProbeControls()")
        expect(page.locator('#bed-probe-retry')).to_be_disabled()
        expect(page.locator('#bed-probe-retry-status')).to_contain_text('Select Remote Control')
        page.evaluate("robotConnectionSnapshot = {available:true, ready:true}; updateBedProbeControls()")
        expect(page.locator('#bed-probe-retry')).to_be_enabled()
        cases.append('last failure stays visible and retry names the missing Remote Control prerequisite')

        previous_requests = len(requests)
        page.evaluate('document.getElementById("bed-probe-retry").click(); document.getElementById("bed-probe-retry").click()')
        expect(page.locator('#bed-probe-confirm-title')).to_contain_text('surface')
        expect(page.locator('#bed-probe-confirm')).to_be_enabled()
        assert requests[previous_requests:] == [('/api/bed-probe/start', {'dry_run':False})]
        cases.append('failed homing waits for cleanup then double-click sends one fresh start with no automatic confirmation')
        page.click('#bed-probe-abort')
        probe.update(state='failed',busy=False,stop_unconfirmed=True,message='Robot stop unconfirmed')
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-retry')).to_be_disabled()
        expect(page.locator('#bed-probe-retry-status')).to_contain_text('Robot stop is unconfirmed')
        expect(page.locator('#bed-probe-start')).to_be_disabled()
        previous_requests = len(requests)
        page.evaluate('document.getElementById("bed-probe-retry").dispatchEvent(new MouseEvent("click"))')
        assert len(requests) == previous_requests
        cases.append('unconfirmed stop keeps retry locked even if busy is false or a click event is dispatched')
        probe.update(stop_unconfirmed=False)
        probe['saved_mesh']={'available':True,'complete':True,'points':9,'stamp':'saved cold mesh'}
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-start')).to_have_text('Redo homing & mesh')
        expect(page.locator('#bed-probe-saved-mesh')).to_contain_text('failed runs leave it unchanged')
        page.click('#probe-copy-temperatures')
        expect(page.locator('#probe-heated')).to_be_checked()
        expect(page.locator('#probe-nozzle-target')).to_have_value('210')
        expect(page.locator('#probe-bed-target')).to_have_value('0')
        page.fill('#probe-bed-target','60')
        expect(page.locator('#bed-probe-start')).to_be_disabled()
        expect(page.locator('#bed-probe-readiness')).to_contain_text('no configured bed heater')
        page.fill('#probe-bed-target','0')
        page.fill('#probe-soak','4')
        expect(page.locator('#bed-probe-start')).to_be_disabled()
        page.fill('#probe-soak','5')
        page.click('#bed-probe-start')
        assert requests[-1]==('/api/bed-probe/start',{'dry_run':False,'thermal':{'nozzle_target_c':210,'bed_target_c':0,'soak_s':5}})
        expect(page.locator('#bed-probe-badge')).to_have_text('Temperature soak')
        expect(page.locator('#bed-probe-confirmation')).to_be_hidden()
        expect(page.locator('#probe-heated')).to_be_disabled()
        expect(page.locator('#bed-probe-start')).to_be_disabled()
        expect(page.locator('#probe-thermal-status')).to_contain_text('Stable 2/5 s')
        expect(page.locator('#bed-probe-sensor')).to_contain_text('Unloaded sample-range limit: 8,000 counts')
        page.reload()
        expect(page.locator('#probe-heated')).to_be_checked()
        expect(page.locator('#probe-bed-target')).to_have_value('0')
        probe.update(state='waiting_surface',confirmation={'stage':'surface','token':f'{starts:024x}'})
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-confirm-detail')).to_contain_text('at the selected temperature')
        expect(page.locator('#bed-probe-confirm-detail')).not_to_contain_text('cool,')
        page.click('#bed-probe-abort')
        cases.append('heated redo requires valid targets, waits without motion controls, survives reload and preserves saved mesh')
        sync['configured_heaters'].append('heater_bed')
        page.evaluate('refreshSync(); refreshStatus()')
        page.click('#probe-copy-temperatures')
        expect(page.locator('#probe-bed-target')).to_have_value('60')
        page.click('#bed-probe-start')
        assert requests[-1][1]['thermal']['bed_target_c']==60
        page.click('#bed-probe-abort')
        assert not any('gcode' in path or 'heater' in path for path,_ in requests)
        cases.append('both configured heaters supported; no heater or G-code commands sent by probing UI')
        unavailable = True
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-start')).to_be_disabled()
        expect(page.locator('#bed-probe-retry')).to_be_disabled()
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
