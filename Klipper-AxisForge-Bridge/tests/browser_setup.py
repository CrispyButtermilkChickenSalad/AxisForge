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
            elif path=='/api/load-calibration/start':
                load.update(state='waiting_home',busy=True,dry_run=data['dry_run'],confirmation=dict(stage='home',token='a'*32),message='Move arm home manually')
                result=load
            elif path=='/api/load-calibration/confirm':
                if drop_confirmation:
                    drop_confirmation=False;handler.abort();return
                stage=data['stage'];next_stage={'home':'scale','scale':'tare'}.get(stage)
                if next_stage:load.update(state='waiting_'+next_stage,confirmation=dict(stage=next_stage,token=next_stage[0]*32),message='Confirm '+next_stage)
                else:load.update(state='adjusting_load',confirmation=None,action_token='x'*32,position_mm=[1,2,105.],contact_z_mm=100.,lower_z_mm=99.75,upper_z_mm=125.,delta_counts=0.,trigger_value=8000.,max_value=60000.)
                result=load
            elif path=='/api/load-calibration/action':
                generation+=1;load.update(action_token=str(generation)*32)
                if data['command']=='step':load['position_mm'][2]+=data['delta_mm']
                else:load.update(state='complete',busy=False,action_token=None)
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
        expect(page.locator('#robot-connect')).to_be_enabled();expect(page.locator('#load-calibration-start')).to_be_disabled()
        page.click('#robot-connect');assert requests==[('/api/robot/connect',{})]
        assert not page.get_by_role('button',name='Power on',exact=True).count()
        cases.append('PC driver connect works with Klipper offline; arm startup stays manual')
        online=True;page.evaluate('refreshStatus()')
        expect(page.locator('#load-calibration-start')).to_be_enabled()
        page.click('#load-calibration-start')
        expect(page.locator('#load-calibration-confirm-title')).to_contain_text('home')
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
        cases.append('three distinct confirmations; duplicate clicks cannot skip a stage')
        page.evaluate('document.getElementById("load-step-down").click();document.getElementById("load-step-down").click()')
        expect(page.locator('#load-calibration-position')).to_contain_text('104.990')
        actions=[r for r in requests if r[0].endswith('/action')]
        assert len(actions)==1 and actions[0][1]['delta_mm']==-.01
        cases.append('one click issues one bounded step with the current token')
        page.click('#calibration-open-logger');expect(page.locator('#embedded-view')).to_be_visible()
        expect(page.locator('#klipper-estop')).to_be_visible()
        page.click('#dashboard-tab');expect(page.locator('#upload-job')).to_be_disabled();expect(page.locator('#sync-start')).to_be_disabled()
        page.click('#setup-tab');expect(page.locator('#load-step-up')).to_be_enabled()
        cases.append('logger navigation preserves calibration; job controls stay locked; stop remains visible')
        unavailable=True;page.evaluate('refreshSetup()');expect(page.locator('#load-step-up')).to_be_disabled()
        unavailable=False;page.evaluate('refreshSetup()');expect(page.locator('#load-step-up')).to_be_enabled()
        page.click('#load-step-up');expect(page.locator('#load-calibration-position')).to_contain_text('105.000')
        page.click('#load-calibration-finish');expect(page.locator('#load-calibration-steps')).to_be_hidden()
        cases.append('lost status disables steps; finish releases setup ownership')
        page.set_viewport_size(dict(width=390,height=844));page.screenshot(path='/tmp/axisforge-setup-mobile.png',full_page=True)
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth+1')
        page.set_viewport_size(dict(width=1280,height=1000));page.screenshot(path='/tmp/axisforge-setup.png',full_page=True)
        assert not errors,errors
        print(json.dumps(dict(passed=len(cases),cases=cases,page_errors=errors,hardware_requests=0),indent=2))
        browser.close()


if __name__=='__main__':main()
