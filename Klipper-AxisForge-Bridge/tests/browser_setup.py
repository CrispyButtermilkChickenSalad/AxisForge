"""Intercept every request: startup, scale confirmations, baby steps, tabs, and interlocks."""
import json
from pathlib import Path
import shutil
import sys
from urllib.parse import urlsplit
from playwright.sync_api import sync_playwright, expect
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from bridge_dashboard.clock_settings import profiles,clock_options


def main():
    motion=dict(policy='exact',square_corner_velocity=1.,junction_deviation_mm=.01,blend_tolerance_mm=.05,blend_speed_mm_s=5.)
    sync=dict(state='idle',busy=False,control_ready=True,robot_ready=True,configured_heaters=['extruder'],corner_policies=['exact'],motion_defaults=motion,clock_options_supported=True,clock_profiles=profiles(),clock_defaults=clock_options(),clock_options=clock_options(),clock_guidance_distance_mm=.1,checks=[])
    robot=dict(available=True,configured=True,busy=False,state='idle',message='Ready to connect',checks=[])
    load=dict(state='idle',busy=False,configured=True,message='Ready',confirmation=None)
    probe=dict(state='idle',busy=False,configured=True,confirmation=None)
    requests,errors,cases=[],[],[]
    online=False;unavailable=False;generation=0;drop_confirmation=False
    def route(handler):
        nonlocal generation,drop_confirmation
        req=handler.request;url=urlsplit(req.url);path=url.path
        if url.netloc!='setup.test':handler.abort();return
        if path=='/' or path.startswith('/static/'):
            file=ROOT/'bridge_dashboard/static'/('index.html' if path=='/' else path.split('/')[-1])
            handler.fulfill(body=file.read_text(),content_type={'.html':'text/html','.js':'application/javascript','.css':'text/css'}[file.suffix]);return
        if req.method=='POST':
            data=req.post_data_json;requests.append((path,data))
            if path=='/api/robot/connect':result=robot
            elif path=='/api/load-calibration/clear-session':
                load.update(message='Unfinished Klipper calibration cleared; saved values retained.');result=load
            elif path=='/api/load-calibration/start':
                load.update(state='waiting_temperature',busy=True,dry_run=data['dry_run'],confirmation=None,
                            calibration_max_counts=data['calibration_max_counts'],calibration_hot_target=data['calibration_hot_target'],
                            hot_calibration=True,temperature_c=209.,target_c=210.,stable_for_s=12.,required_stable_s=30.,
                            message='Waiting for the manually selected nozzle temperature')
                result=load
            elif path=='/api/load-calibration/confirm':
                if drop_confirmation:
                    drop_confirmation=False;handler.abort();return
                stage=data['stage'];next_stage={'home':'scale','scale':'tare'}.get(stage)
                if next_stage:load.update(state='waiting_'+next_stage,confirmation=dict(stage=next_stage,token=next_stage[0]*32),message='Confirm '+next_stage)
                else:load.update(state='adjusting_load',confirmation=None,action_token='x'*32,position_mm=[1,2,105.],contact_z_mm=100.,lower_z_mm=99.75,upper_z_mm=125.,delta_counts=0.,trigger_value=8000.,max_value=load['calibration_max_counts'],unloaded=True,calibration_recorded=False)
                result=load
            elif path=='/api/load-calibration/action':
                generation+=1;load.update(action_token=str(generation)*32)
                if data['command']=='step':load['position_mm'][2]+=data['delta_mm']
                elif data['command']=='record':load.update(calibration_recorded=True,recorded_grams=data['grams'],calibration_counts_per_gram=180.,calibration_tare_counts=240000.)
                else:load.update(state='saving_config',busy=True,action_token=None)
                result=load
            elif path=='/api/load-calibration/abort':load.update(state='aborted',busy=False,action_token=None);result=load
            else:raise AssertionError('Unexpected POST '+path)
        elif path=='/api/robot/connection':result=robot
        elif path=='/api/load-calibration/status':
            if unavailable:handler.fulfill(status=503,body='{}');return
            result=load
        elif path=='/api/bed-probe/status':result=probe
        elif path=='/api/status':result=dict(klipper=dict(connected=online,state='ready' if online else 'offline'),ros=dict(connected=True,joint_state_fresh=True),axisforge=dict(running=False))
        elif path=='/api/sync/status':result=sync
        elif path=='/api/jobs':result=dict(jobs=['part.gcode'])
        elif path.endswith('/preflight'):result=dict(state='staged')
        elif path=='/api/links':result=dict(alps_logger='http://logger.invalid/')
        elif path=='/api/klipper/telemetry':result={}
        else:handler.fulfill(status=404,body='{}');return
        handler.fulfill(body=json.dumps(result),content_type='application/json')
    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(executable_path=shutil.which('google-chrome'),headless=False,args=['--headless=new'],chromium_sandbox=True)
        context=browser.new_context(viewport=dict(width=1280,height=1000));context.route('**/*',route)
        page=context.new_page();page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto('http://setup.test/')
        expect(page.locator('#setup-view')).to_be_visible();expect(page.locator('#dashboard-view')).to_be_hidden()
        headings = page.locator('#setup-view > section > .card-heading > h2').all_text_contents()
        assert headings.index('2. Home & probe bed') < headings.index('3. ALPS load calibration'), headings
        expect(page.locator('#robot-connect')).to_be_enabled();expect(page.locator('#load-calibration-start')).to_be_disabled()
        expect(page.locator('#load-calibration-clear-session')).to_be_disabled()
        expect(page.locator('[data-calibration-step]')).to_have_count(8)
        expect(page.locator('#load-calibration-overview')).to_contain_text('at most 20 mm')
        expect(page.locator('#load-calibration-overview')).to_contain_text('5 mm at 0.5 mm/s')
        expect(page.locator('#load-calibration-z-status')).to_be_visible()
        expect(page.locator('#load-calibration-z-status')).to_contain_text('Start the scale routine')
        for button in page.locator('[data-calibration-step]').all():
            expect(button).to_be_visible();expect(button).to_be_disabled()
        page.click('#robot-connect');assert requests==[('/api/robot/connect',{})]
        assert not page.get_by_role('button',name='Power on',exact=True).count()
        cases.append('PC driver connect works with Klipper offline; arm startup stays manual')
        online=True;page.evaluate('refreshStatus()')
        expect(page.locator('#load-calibration-start')).to_be_enabled()
        expect(page.locator('#load-calibration-clear-session')).to_be_enabled()
        page.evaluate('document.getElementById("load-calibration-clear-session").click();document.getElementById("load-calibration-clear-session").click()')
        expect(page.locator('#load-calibration-message')).to_contain_text('saved values retained')
        assert requests==[('/api/robot/connect',{}),('/api/load-calibration/clear-session',{})]
        cases.append('unfinished Klipper calibration clears only by explicit idle connected request; duplicate clicks send once')
        expect(page.locator('#load-calibration-max-counts')).to_have_value('120000')
        expect(page.locator('#load-calibration-hot')).not_to_be_checked()
        expect(page.locator('#load-calibration-hot-target-label')).to_be_hidden()
        expect(page.locator('#load-calibration-limit-guidance')).to_contain_text('not rated mechanical forces')
        expect(page.locator('#load-calibration-limit-guidance')).to_contain_text('Bed homing, mesh probing and the initial scale search use a separate 120,000-count overload limit')
        for value in ('60000','250001',''):
            page.fill('#load-calibration-max-counts',value)
            expect(page.locator('#load-calibration-start')).to_be_disabled()
            expect(page.locator('#load-calibration-simulate')).to_be_disabled()
        page.fill('#load-calibration-max-counts','140000')
        page.check('#load-calibration-hot')
        expect(page.locator('#load-calibration-hot-target')).to_have_value('210')
        expect(page.locator('#load-calibration-hot-guidance')).to_contain_text('does not turn heaters on')
        for value in ('49','301',''):
            page.fill('#load-calibration-hot-target',value)
            expect(page.locator('#load-calibration-start')).to_be_disabled()
        page.fill('#load-calibration-hot-target','210')
        expect(page.locator('#load-calibration-start')).to_be_enabled()
        assert requests==[('/api/robot/connect',{}),('/api/load-calibration/clear-session',{})]
        cases.append('calibration-only limit and optional hot target validate locally without heating or motion')
        page.click('#load-calibration-start')
        assert requests[-1]==('/api/load-calibration/start',dict(dry_run=False,calibration_max_counts=140000,calibration_hot_target=210))
        expect(page.locator('#load-calibration-temperature')).to_contain_text('Stable: 12/30 s')
        expect(page.locator('#load-calibration-temperature')).to_contain_text('Target: 210.0°C')
        expect(page.locator('#load-calibration-max-counts')).to_be_disabled()
        expect(page.locator('#load-calibration-hot')).to_be_disabled()
        expect(page.locator('#load-calibration-hot-target')).to_be_disabled()
        expect(page.locator('#load-calibration-clear-session')).to_be_disabled()
        expect(page.locator('#load-calibration-confirmation')).to_be_hidden()
        expect(page.locator('#load-step-up')).to_be_disabled()
        cases.append('selected calibration conditions are sent once; temperature wait exposes no motion action')
        load.update(state='waiting_home',confirmation=dict(stage='home',token='a'*32),message='Move arm home manually')
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-calibration-confirm-title')).to_contain_text('home')
        expect(page.locator('#load-calibration-z-status')).to_contain_text('Confirm the home/start pose')
        expect(page.locator('#bed-probe-start')).to_be_disabled();expect(page.locator('#robot-connect')).to_be_disabled()
        drop_confirmation=True
        page.click('#load-calibration-confirm')
        expect(page.locator('#load-calibration-confirm')).to_be_enabled()
        assert load['confirmation']['stage']=='home'
        cases.append('dropped confirmation needs an explicit retry after fresh server status')
        for stage in ('home','scale','tare'):
            expect(page.locator('#load-calibration-confirm')).to_be_enabled()
            assert load['confirmation']['stage']==stage
            page.evaluate('document.getElementById("load-calibration-confirm").click();document.getElementById("load-calibration-confirm").click()')
            expect(page.locator('#load-calibration-confirm-title' if stage!='tare' else '#load-calibration-steps')).to_be_visible()
        assert len([r for r in requests if r[0].endswith('/confirm')])==4
        expect(page.locator('#load-step-down')).to_be_enabled()
        expect(page.locator('#load-calibration-z-status')).to_contain_text('Z steps are available')
        cases.append('three distinct confirmations; duplicate clicks cannot skip a stage')
        page.evaluate('document.getElementById("load-step-down").click();document.getElementById("load-step-down").click()')
        expect(page.locator('#load-calibration-position')).to_contain_text('104.990')
        actions=[r for r in requests if r[0].endswith('/action')]
        assert len(actions)==1 and actions[0][1]['delta_mm']==-.01
        cases.append('one click issues one bounded step with the current token')
        load.update(state='load_limit',action_token=None,upward_only=True,load_limit_reached=True)
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-step-up')).to_be_disabled()
        load.update(state='adjusting_load',action_token='r'*32)
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-step-up')).to_be_enabled()
        expect(page.locator('#load-step-down')).to_be_disabled()
        expect(page.locator('#load-calibration-finish')).to_be_disabled()
        page.select_option('#load-step-size','1')
        expect(page.locator('#load-step-up')).to_be_disabled()
        page.select_option('#load-step-size','0.1')
        expect(page.locator('#load-step-up')).to_be_enabled()
        page.click('#load-step-up')
        expect(page.locator('#load-calibration-position')).to_contain_text('105.090')
        actions=[r for r in requests if r[0].endswith('/action')]
        assert len(actions)==2 and actions[-1][1]['delta_mm']==.1
        expect(page.locator('#load-step-down')).to_be_disabled()
        cases.append('load limit requires fresh recovery token and permits only a small upward release')
        load.update(state='failed',busy=False,action_token=None,message='Temperature fault')
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-step-up')).to_be_disabled()
        expect(page.locator('#load-step-down')).to_be_disabled()
        expect(page.locator('#load-calibration-z-status')).to_contain_text('Temperature fault')
        cases.append('load-limit recovery does not bypass a temperature fault')
        load.update(message='Another native calibration is open; finish or abort it manually first')
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-calibration-z-status')).to_contain_text('Clear unfinished Klipper calibration')
        for button in page.locator('[data-calibration-step]').all():
            expect(button).to_be_disabled()
            expect(button).to_have_attribute('title',page.locator('#load-calibration-z-status').inner_text())
        cases.append('disabled Z controls explain the failed stage and unfinished native-session recovery')
        load.update(state='adjusting_load',busy=True,action_token='z'*32,upward_only=False,load_limit_reached=False)
        load['position_mm'][2]=104.99
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-step-size')).to_be_enabled()
        page.select_option('#load-step-size','0.01')
        expect(page.locator('#load-step-down')).to_be_enabled()
        load['position_mm'][2]=115.
        page.evaluate('refreshSetup()')
        for delta in (-10.,10.,-1.,1.,-.1,.1,-.05,.05):
            button=page.locator('[data-calibration-step="%s"]' % ('%g' % delta))
            expect(button).to_be_enabled()
            before=len([r for r in requests if r[0].endswith('/action')])
            button.evaluate('(button) => { button.click(); button.click(); }')
            expect(page.locator('#load-step-up')).to_be_enabled()
            actions=[r for r in requests if r[0].endswith('/action')]
            assert len(actions)==before+1 and actions[-1][1]['delta_mm']==delta
        cases.append('eight direct Z buttons each send one fresh-token move in either direction')
        load['position_mm'][2]=104.99
        page.evaluate('refreshSetup()')
        page.click('#calibration-open-logger');expect(page.locator('#embedded-view')).to_be_visible()
        expect(page.locator('#klipper-estop')).to_be_visible()
        page.click('#dashboard-tab');expect(page.locator('#upload-job')).to_be_disabled();expect(page.locator('#sync-start')).to_be_disabled()
        page.click('#setup-tab');expect(page.locator('#load-step-up')).to_be_enabled()
        cases.append('logger navigation preserves calibration; job controls stay locked; stop remains visible')
        unavailable=True;page.evaluate('refreshSetup()');expect(page.locator('#load-step-up')).to_be_disabled()
        unavailable=False;page.evaluate('refreshSetup()');expect(page.locator('#load-step-up')).to_be_enabled()
        page.click('#load-step-up');expect(page.locator('#load-calibration-position')).to_contain_text('105.000')
        expect(page.locator('#load-calibration-finish')).to_be_disabled()
        for value in ('49','25001',''):
            page.fill('#load-calibration-grams',value)
            expect(page.locator('#load-calibration-record')).to_be_disabled()
        page.fill('#load-calibration-grams','567.8')
        expect(page.locator('#load-calibration-record')).to_be_disabled()
        load.update(unloaded=False,delta_counts=102204.)
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-calibration-record')).to_be_enabled()
        page.evaluate('document.getElementById("load-calibration-record").click();document.getElementById("load-calibration-record").click()')
        expect(page.locator('#load-calibration-result')).to_contain_text('567.8 g measured load')
        records=[r for r in requests if r[0].endswith('/action') and r[1]['command']=='record']
        assert len(records)==1 and records[0][1]['grams']==567.8
        expect(page.locator('#load-calibration-record')).to_be_disabled()
        expect(page.locator('#load-calibration-finish')).to_be_disabled()
        load.update(unloaded=True,delta_counts=0.)
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-calibration-finish')).to_be_enabled()
        expect(page.locator('#load-calibration-save-guidance')).to_contain_text('SAVE_CONFIG, which restarts Klipper')
        cases.append('record uses one actual scale reading; saving requires recorded and freshly unloaded status')
        page.click('#load-calibration-finish')
        expect(page.locator('#load-calibration-steps')).to_be_hidden()
        expect(page.locator('#load-calibration-readiness')).to_contain_text('heaters off')
        expect(page.locator('#load-calibration-start')).to_be_disabled()
        expect(page.locator('#load-calibration-abort')).to_be_disabled()
        for button in page.locator('[data-calibration-step]').all():expect(button).to_be_disabled()
        expect(page.locator('#load-calibration-result')).not_to_contain_text('saved and verified')
        load.update(state='verifying_config');page.evaluate('refreshSetup()')
        expect(page.locator('#load-calibration-badge')).to_contain_text('verifying config')
        expect(page.locator('#load-calibration-abort')).to_be_disabled()
        load.update(state='save_failed',busy=False,message='Saved values could not be verified')
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-calibration-result')).to_contain_text('save was not verified')
        load.update(state='complete',message='Saved calibration matches Klipper')
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-calibration-result')).to_contain_text('saved and verified in Klipper')
        expect(page.locator('#load-calibration-result')).to_contain_text('180.000 counts/g')
        cases.append('saving and verification keep controls locked; only verified completion reports success')
        page.set_viewport_size(dict(width=390,height=844));page.screenshot(path='/tmp/axisforge-setup-mobile.png',full_page=True)
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth+1')
        page.set_viewport_size(dict(width=1280,height=1000));page.screenshot(path='/tmp/axisforge-setup.png',full_page=True)
        assert not errors,errors
        print(json.dumps(dict(passed=len(cases),cases=cases,page_errors=errors,hardware_requests=0),indent=2))
        browser.close()


if __name__=='__main__':main()
