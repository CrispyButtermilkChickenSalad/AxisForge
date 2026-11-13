"""TCP editor UI with every request intercepted; never changes a real configuration."""
import json
from pathlib import Path
import shutil
import sys
from urllib.parse import urlsplit
from playwright.sync_api import sync_playwright,expect
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from bridge_dashboard.clock_settings import clock_options,profiles


def main():
    motion=dict(policy='exact',square_corner_velocity=1.,junction_deviation_mm=.01,blend_tolerance_mm=.05,blend_speed_mm_s=5.)
    sync=dict(state='idle',busy=False,control_ready=True,robot_ready=True,configured_heaters=['extruder'],corner_policies=['exact'],motion_defaults=motion,clock_options_supported=True,clock_profiles=profiles(),clock_defaults=clock_options(),clock_options=clock_options(),clock_guidance_distance_mm=.1,checks=[])
    tcp=dict(tcp_offset_mm=[-100.,0.,117.06],revision='a'*64,bed_mesh=dict(requires_probe=False,message='Bed mesh TCP matches the saved offset.'))
    routine=dict(state='idle',busy=False,configured=True,confirmation=None)
    saved=[];errors=[];cases=[];unavailable=False;drop_save=False
    def route(handler):
        nonlocal drop_save
        req=handler.request;url=urlsplit(req.url);path=url.path
        if url.netloc!='tcp.test':handler.abort();return
        if path=='/' or path.startswith('/static/'):
            file=ROOT/'bridge_dashboard/static'/('index.html' if path=='/' else path.split('/')[-1])
            handler.fulfill(body=file.read_text(),content_type={'.html':'text/html','.js':'application/javascript','.css':'text/css'}[file.suffix]);return
        if path=='/api/setup/tcp':
            if unavailable:handler.fulfill(status=503,body=json.dumps({'detail':'Config unavailable'}));return
            if req.method=='POST':
                data=req.post_data_json;saved.append(data)
                assert data['revision']==tcp['revision']
                tcp.update(tcp_offset_mm=[data[k] for k in ('x_mm','y_mm','z_mm')],revision=str(len(saved))*64,changed=True,message='Nozzle TCP saved.',bed_mesh=dict(requires_probe=True,message='Run Home & probe bed with the saved TCP before printing.'))
                if drop_save:drop_save=False;handler.abort();return
            result=tcp
        elif req.method=='POST':raise AssertionError('Unexpected write '+path)
        elif path=='/api/robot/connection':result=dict(state='idle',available=True,configured=True,busy=False,checks=[])
        elif path in ('/api/bed-probe/status','/api/load-calibration/status'):result=routine
        elif path=='/api/status':result=dict(klipper=dict(connected=False,state='offline'),ros=dict(connected=True,joint_state_fresh=False),axisforge=dict(running=False))
        elif path=='/api/sync/status':result=sync
        elif path=='/api/jobs':result=dict(jobs=[])
        elif path=='/api/links':result={}
        elif path=='/api/klipper/telemetry':result={}
        else:handler.fulfill(status=404,body='{}');return
        handler.fulfill(body=json.dumps(result),content_type='application/json')
    with sync_playwright() as playwright:
        browser=playwright.chromium.launch(executable_path=shutil.which('google-chrome'),headless=False,args=['--headless=new'],chromium_sandbox=True)
        context=browser.new_context(viewport=dict(width=1280,height=1000));context.route('**/*',route)
        page=context.new_page();page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto('http://tcp.test/')
        expect(page.locator('#tcp-x')).to_have_value('-100');expect(page.locator('#tcp-z')).to_have_value('117.06')
        expect(page.locator('#tcp-save')).to_be_disabled();expect(page.locator('#tcp-x')).to_be_enabled()
        cases.append('saved offsets load with hardware offline; no automatic writes')
        page.fill('#tcp-z','118.25');expect(page.locator('#tcp-save')).to_be_enabled()
        page.evaluate('refreshTcp()');expect(page.locator('#tcp-z')).to_have_value('118.25')
        for bad in ('','1001'):
            page.fill('#tcp-x',bad);expect(page.locator('#tcp-save')).to_be_disabled()
        page.fill('#tcp-x','-101')
        page.evaluate('document.getElementById("tcp-save").click();document.getElementById("tcp-save").click()')
        expect(page.locator('#tcp-badge')).to_have_text('Saved')
        assert len(saved)==1 and saved[0]['x_mm']==-101 and saved[0]['z_mm']==118.25
        expect(page.locator('#tcp-bed-mesh')).to_contain_text('Home & probe bed')
        cases.append('valid save sends explicit XYZ and revision once; dirty edits survive polling; changed mesh needs probing')
        page.reload();expect(page.locator('#tcp-z')).to_have_value('118.25')
        page.fill('#tcp-y','2')
        sync['busy']=True;sync['stage']='preflight';page.evaluate('refreshSync()')
        expect(page.locator('#tcp-x')).to_be_disabled();expect(page.locator('#tcp-save')).to_be_disabled()
        sync['busy']=False;page.evaluate('refreshSync()');expect(page.locator('#tcp-save')).to_be_enabled()
        routine['busy']=True;page.evaluate('refreshSetup()')
        expect(page.locator('#tcp-save')).to_be_disabled()
        routine['busy']=False;page.evaluate('refreshSetup()');expect(page.locator('#tcp-save')).to_be_enabled()
        cases.append('saved settings survive reload; preflight and calibration block editing')
        tcp.update(revision='e'*64,tcp_offset_mm=[-102.,0.,118.25]);page.evaluate('refreshTcp()')
        expect(page.locator('#tcp-badge')).to_have_text('Reload required');expect(page.locator('#tcp-y')).to_have_value('2')
        expect(page.locator('#tcp-save')).to_be_disabled();page.click('#tcp-reload')
        expect(page.locator('#tcp-x')).to_have_value('-102');expect(page.locator('#tcp-y')).to_have_value('0')
        cases.append('another editor cannot be silently overwritten; explicit reload discards pending edits')
        page.fill('#tcp-y','3');drop_save=True;page.click('#tcp-save')
        expect(page.locator('#tcp-badge')).to_have_text('Reload required');expect(page.locator('#tcp-save')).to_be_disabled()
        page.click('#tcp-reload');expect(page.locator('#tcp-y')).to_have_value('3');expect(page.locator('#tcp-badge')).to_have_text('Saved')
        cases.append('lost save response requires reload; no blind retry')
        unavailable=True;page.evaluate('refreshTcp()');expect(page.locator('#tcp-x')).to_be_disabled()
        unavailable=False;page.click('#tcp-reload');expect(page.locator('#tcp-x')).to_be_enabled()
        page.set_viewport_size(dict(width=390,height=844));page.locator('#tcp-form').scroll_into_view_if_needed()
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth+1')
        page.screenshot(path='/tmp/axisforge-tcp-mobile.png')
        page.set_viewport_size(dict(width=1280,height=1000));page.locator('#tcp-form').scroll_into_view_if_needed();page.screenshot(path='/tmp/axisforge-tcp-editor.png')
        assert not errors,errors
        print(json.dumps(dict(passed=len(cases),cases=cases,page_errors=errors,real_config_writes=0),indent=2))
        browser.close()


if __name__=='__main__':main()
