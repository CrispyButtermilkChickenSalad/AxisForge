"""Saved probing controls and guarded Z increments; every network request mocked."""
import json
from pathlib import Path
import shutil
import sys
from urllib.parse import urlsplit

from playwright.sync_api import expect, sync_playwright

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bridge_dashboard.clock_settings import clock_options, profiles


def main():
    motion = dict(policy='exact', square_corner_velocity=1., junction_deviation_mm=.01,
                  blend_tolerance_mm=.05, blend_speed_mm_s=5.)
    sync = dict(state='idle', busy=False, control_ready=False, robot_ready=False,
                configured_heaters=['extruder'], corner_policies=['exact'], motion_defaults=motion,
                clock_options_supported=True, clock_profiles=profiles(), clock_defaults=clock_options(),
                clock_options=clock_options(), clock_guidance_distance_mm=.1, checks=[])
    settings = dict(first_speed_mm_s=10., second_speed_mm_s=.2, mesh_speed_mm_s=.1,
                    mesh_samples=5, mesh_repeatability_mm=.02, lift_speed_mm_s=20.,
                    mesh_columns=3, mesh_rows=3)
    probing = dict(revision='a'*64, settings=settings)
    bed = dict(revision='d'*64, bed_size_mm=[235.,235.], probe_margin_mm=30., bed_mesh={})
    bed_saved, copies, jobs = [], [], ['preloaded.gcode']
    robot = dict(state='waiting_robot', available=True, configured=True, busy=False, checks=[],
                 configuration=dict(robot_type='UR16e', robot_ip='192.168.0.100',
                                    axisforge_tcp_mm=[-100., 0., 117.06], probing_rpy_deg=[180., 0., 90.]))
    probe = dict(state='idle', busy=False, configured=True, confirmation=None)
    load = dict(state='idle', busy=False, configured=True, confirmation=None)
    saved, actions, errors, cases = [], [], [], []
    drop_save = unavailable = False

    def route(handler):
        nonlocal drop_save
        req = handler.request
        url = urlsplit(req.url)
        path = url.path
        if url.netloc != 'probing.test':
            handler.abort()
            return
        if path == '/' or path.startswith('/static/'):
            file = ROOT/'bridge_dashboard/static'/('index.html' if path == '/' else path.split('/')[-1])
            handler.fulfill(body=file.read_text(), content_type={'.html':'text/html', '.js':'application/javascript', '.css':'text/css'}[file.suffix])
            return
        if path == '/api/setup/probing':
            if unavailable:
                handler.fulfill(status=503, body=json.dumps(dict(detail='Profile unavailable')))
                return
            if req.method == 'POST':
                data = req.post_data_json
                saved.append(data)
                assert data['revision'] == probing['revision']
                probing.update(revision=str(len(saved))*64, settings=data['settings'],
                               changed=True, message='Homing and mesh settings saved.')
                if drop_save:
                    drop_save = False
                    handler.abort()
                    return
            result = probing
        elif path == '/api/setup/bed':
            if req.method == 'POST':
                data = req.post_data_json
                assert data['revision'] == bed['revision']
                bed_saved.append(data)
                bed.update(revision='f'*64, bed_size_mm=[data['width_mm'],data['depth_mm']],
                           probe_margin_mm=data['probe_margin_mm'], changed=True,
                           message='Bed saved; re-probe before printing.',bed_mesh={'requires_probe':True})
            result = bed
        elif path == '/api/jobs/copy-to-klipper' and req.method == 'POST':
            data = req.post_data_json
            assert data == {'job':'preloaded.gcode'}
            copies.append(data)
            name = 'preloaded--123456abcdef.gcode'
            jobs.append(name)
            result = dict(job=name,original_name='preloaded.gcode',ready=False,message='New version; run both preflights.')
        elif path == '/api/load-calibration/action' and req.method == 'POST':
            data = req.post_data_json
            assert data['command'] == 'step'
            assert data['token'] == load['action_token']
            actions.append(data)
            load['position_mm'][2] += data['delta_mm']
            load['action_token'] = str(len(actions))*32
            result = load
        elif req.method == 'POST':
            raise AssertionError('Unexpected write '+path)
        elif path == '/api/setup/tcp':
            result = dict(tcp_offset_mm=[-100., 0., 117.06], revision='b'*64, bed_mesh={})
        elif path == '/api/robot/connection':
            result = robot
        elif path == '/api/bed-probe/status':
            result = probe
        elif path == '/api/load-calibration/status':
            result = load
        elif path == '/api/status':
            result = dict(klipper=dict(connected=False, state='offline'),
                          ros=dict(connected=False, joint_state_fresh=False), axisforge=dict(running=False))
        elif path == '/api/sync/status':
            result = sync
        elif path == '/api/jobs':
            result = dict(jobs=jobs)
        elif path.endswith('/preflight'):
            result = dict(state='staged',checks=[])
        elif path in ('/api/links', '/api/klipper/telemetry'):
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
        page.goto('http://probing.test/')
        expect(page.locator('#probing-search-speed')).to_have_value('10')
        expect(page.locator('#probing-fine-speed')).to_have_value('0.2')
        expect(page.locator('#probing-lift-speed')).to_have_value('20')
        expect(page.locator('#probing-mesh-columns')).to_have_value('3')
        expect(page.locator('#probing-mesh-rows')).to_have_value('3')
        expect(page.locator('#probing-mesh-speed')).to_have_value('0.1')
        expect(page.locator('#probing-mesh-samples')).to_have_value('5')
        expect(page.locator('#probing-repeatability')).to_have_value('0.02')
        expect(page.locator('#probing-search-speed')).to_be_enabled()
        expect(page.locator('#probing-save')).to_be_disabled()
        expect(page.locator('#bed-probe-start')).to_be_disabled()
        expect(page.locator('#robot-orientation')).to_contain_text('roll 180°')
        expect(page.locator('#robot-orientation')).to_contain_text('yaw 90°')
        assert not saved and not actions
        cases.append('saved speeds and actual configured orientation load offline; no automatic writes or motion')

        expect(page.locator('#bed-width')).to_have_value('235')
        page.fill('#bed-width','150')
        page.fill('#bed-depth','100')
        page.fill('#bed-margin','50')
        expect(page.locator('#bed-settings-save')).to_be_disabled()
        page.fill('#bed-margin','10')
        expect(page.locator('#bed-settings-area')).to_contain_text('X 10–140 mm, Y 10–90 mm')
        page.evaluate('refreshBedGeometry()')
        expect(page.locator('#bed-width')).to_have_value('150')
        expect(page.locator('#bed-probe-simulate')).to_be_disabled()
        sync['busy'] = True
        page.evaluate('refreshSync()')
        expect(page.locator('#bed-settings-save')).to_be_disabled()
        sync['busy'] = False
        page.evaluate('refreshSync()')
        page.evaluate('document.getElementById("bed-settings-save").click();document.getElementById("bed-settings-save").click()')
        expect(page.locator('#bed-settings-badge')).to_have_text('Saved')
        assert len(bed_saved) == 1
        assert bed_saved[0]['width_mm'] == 150 and bed_saved[0]['depth_mm'] == 100
        page.fill('#bed-width','140')
        bed['revision'] = '9'*64
        page.evaluate('refreshBedGeometry()')
        expect(page.locator('#bed-settings-badge')).to_have_text('Reload required')
        expect(page.locator('#bed-width')).to_have_value('140')
        page.click('#bed-settings-reload')
        expect(page.locator('#bed-width')).to_have_value('150')
        cases.append('offline bed resize previews coverage, validates margin, blocks active operations and preserves revision conflicts')

        page.fill('#probing-search-speed', '8')
        expect(page.locator('#probing-save')).to_be_enabled()
        page.evaluate('refreshProbing()')
        expect(page.locator('#probing-search-speed')).to_have_value('8')
        expect(page.locator('#bed-probe-simulate')).to_be_disabled()
        for field, bad, restored in (('probing-search-speed', '11', '8'),
                                     ('probing-fine-speed', '', '0.2'),
                                     ('probing-mesh-speed', '.3', '.1'),
                                     ('probing-mesh-samples', '3.5', '5'),
                                     ('probing-lift-speed', '21', '20'),
                                     ('probing-mesh-columns', '1', '3'),
                                     ('probing-mesh-columns', '2.5', '3'),
                                     ('probing-mesh-rows', '10', '3'),
                                     ('probing-repeatability', '.001', '.02')):
            page.fill('#'+field, bad)
            expect(page.locator('#probing-save')).to_be_disabled()
            page.fill('#'+field, restored)
        page.fill('#probing-search-speed', '.5')
        page.fill('#probing-fine-speed', '.5')
        expect(page.locator('#probing-save')).to_be_disabled()
        page.fill('#probing-search-speed', '8')
        page.fill('#probing-fine-speed', '.2')
        page.fill('#probing-lift-speed', '10')
        page.fill('#probing-mesh-columns', '2')
        page.fill('#probing-mesh-rows', '9')
        page.evaluate('document.getElementById("probing-save").click();document.getElementById("probing-save").click()')
        expect(page.locator('#probing-settings-badge')).to_have_text('Saved')
        assert len(saved) == 1 and saved[0]['settings']['first_speed_mm_s'] == 8
        assert saved[0]['settings']['lift_speed_mm_s'] == 10
        assert saved[0]['settings']['mesh_columns'] == 2 and saved[0]['settings']['mesh_rows'] == 9
        expect(page.locator('#probing-saved')).to_contain_text('2×9 mesh (18 points)')
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-progress-label')).to_contain_text('0/18 mesh points')
        probe.update(state='meshing', total_points=81, current_point=40)
        page.evaluate('refreshBedProbe()')
        expect(page.locator('#bed-probe-progress-label')).to_contain_text('40/81 mesh points')
        probe.update(state='idle', total_points=0, current_point=0)
        cases.append('input bounds and slower fine speed enforced; edits survive polling; save uses one revisioned request')

        page.fill('#probing-fine-speed', '0.15')
        for active, refresh in ((sync, 'refreshSync()'), (load, 'refreshSetup()'),
                                (probe, 'refreshBedProbe()'), (robot, 'refreshSetup()')):
            active['busy'] = True
            page.evaluate(refresh)
            expect(page.locator('#probing-save')).to_be_disabled()
            expect(page.locator('#probing-search-speed')).to_be_disabled()
            active['busy'] = False
            page.evaluate(refresh)
            expect(page.locator('#probing-save')).to_be_enabled()
        cases.append('preflight, homing, calibration and driver startup block settings changes')

        probing.update(revision='e'*64, settings=dict(probing['settings'], first_speed_mm_s=7.))
        page.evaluate('refreshProbing()')
        expect(page.locator('#probing-settings-badge')).to_have_text('Reload required')
        expect(page.locator('#probing-fine-speed')).to_have_value('0.15')
        expect(page.locator('#probing-save')).to_be_disabled()
        page.click('#probing-reload')
        expect(page.locator('#probing-search-speed')).to_have_value('7')
        expect(page.locator('#probing-fine-speed')).to_have_value('0.2')
        page.fill('#probing-fine-speed', '0.15')
        drop_save = True
        page.click('#probing-save')
        expect(page.locator('#probing-settings-badge')).to_have_text('Reload required')
        page.evaluate('refreshProbing()')
        expect(page.locator('#probing-settings-badge')).to_have_text('Reload required')
        assert len(saved) == 2
        page.click('#probing-reload')
        expect(page.locator('#probing-settings-badge')).to_have_text('Saved')
        expect(page.locator('#probing-fine-speed')).to_have_value('0.15')
        cases.append('external edits and lost save replies require explicit reload with no blind retry')

        unavailable = True
        page.evaluate('refreshProbing()')
        expect(page.locator('#probing-search-speed')).to_be_disabled()
        unavailable = False
        page.click('#probing-reload')
        expect(page.locator('#probing-search-speed')).to_be_enabled()
        load.update(state='adjusting_load', busy=True, action_token='x'*32, position_mm=[1., 2., 105.],
                    contact_z_mm=100., lower_z_mm=99.75, upper_z_mm=125., delta_counts=0.)
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-calibration-steps')).to_be_visible()
        for value in ('10', '1', '0.1', '0.05', '0.01', '0.001'):
            assert page.locator(f'#load-step-size option[value="{value}"]').count() == 1
        page.select_option('#load-step-size', '10')
        expect(page.locator('#load-step-up')).to_be_enabled()
        expect(page.locator('#load-step-down')).to_be_disabled()
        page.evaluate('document.getElementById("load-step-up").click();document.getElementById("load-step-up").click()')
        expect(page.locator('#load-calibration-position')).to_contain_text('115.000')
        assert len(actions) == 1 and actions[0]['delta_mm'] == 10
        expect(page.locator('#load-step-down')).to_be_enabled()
        page.click('#load-step-down')
        expect(page.locator('#load-calibration-position')).to_contain_text('105.000')
        assert actions[-1]['delta_mm'] == -10
        cases.append('requested coarse/fine Z sizes are available; each click consumes one action token in either direction')

        load['position_mm'][2] = 101.
        page.evaluate('refreshSetup()')
        page.select_option('#load-step-size', '1')
        expect(page.locator('#load-step-down')).to_be_disabled()
        expect(page.locator('#load-step-guidance')).to_contain_text('0.05 mm or finer')
        load['position_mm'][2] = 100.6
        page.evaluate('refreshSetup()')
        page.select_option('#load-step-size', '0.1')
        expect(page.locator('#load-step-down')).to_be_enabled()
        page.click('#load-step-down')
        expect(page.locator('#load-calibration-position')).to_contain_text('100.500')
        expect(page.locator('#load-step-down')).to_be_disabled()
        page.select_option('#load-step-size', '0.05')
        expect(page.locator('#load-step-down')).to_be_enabled()
        load['position_mm'][2] = 125.
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-step-up')).to_be_disabled()
        load['position_mm'][2] = 99.75
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-step-down')).to_be_disabled()
        load['contact_z_mm'] = None
        page.evaluate('refreshSetup()')
        expect(page.locator('#load-step-up')).to_be_disabled()
        expect(page.locator('#load-step-down')).to_be_disabled()
        cases.append('large downward moves cannot enter contact zone; travel limits and missing position data disable motion')

        load.update(state='idle',busy=False,action_token=None)
        page.evaluate('refreshSetup()')
        page.click('#dashboard-tab')
        page.select_option('#job-select','preloaded.gcode')
        expect(page.locator('#copy-job-klipper')).to_be_enabled()
        page.evaluate('document.getElementById("copy-job-klipper").click();document.getElementById("copy-job-klipper").click()')
        expect(page.locator('#job-select')).to_have_value('preloaded--123456abcdef.gcode')
        assert len(copies) == 1
        expect(page.locator('#sync-start')).to_be_disabled()
        expect(page.locator('#copy-job-message')).to_contain_text('run both preflights')
        page.click('#setup-tab')
        cases.append('preloaded file copy is single-request, selects its new version and still requires preflight')

        page.set_viewport_size(dict(width=390, height=844))
        page.locator('#probing-form').scroll_into_view_if_needed()
        assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth+1')
        page.screenshot(path='/tmp/axisforge-probing-settings-mobile.png')
        assert not errors, errors
        print(json.dumps(dict(passed=len(cases), cases=cases, page_errors=errors, real_hardware_requests=0), indent=2))
        browser.close()


if __name__ == '__main__':
    main()
