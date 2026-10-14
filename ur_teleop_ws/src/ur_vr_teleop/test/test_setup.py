"""Synthetic setup telemetry/files only. Never a driver or physical robot."""
from copy import deepcopy
import json
from pathlib import Path
import pytest
from ur_vr_teleop import engineering as e
from ur_vr_teleop.preferences import DEFAULTS
from ur_vr_teleop.setup_checks import SetupChecks
from ur_vr_teleop.startup import Startup


@pytest.fixture
def installation(tmp_path,monkeypatch):
    monkeypatch.setattr(e, "motion_packages", lambda: {"servo": "synthetic-version"})
    for name in e.CONTROL_MODULES:
        f=tmp_path/'ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop'/name
        f.parent.mkdir(parents=True,exist_ok=True);f.write_text('synthetic control')
    for name in ('dependencies.repos','ur_teleop/config/commissioning_controllers.yaml',
                 'ur_teleop_ws/src/ur_vr_sim_teleop/ur_vr_sim_teleop/core.py','robot.yaml','tracking.json'):
        f=tmp_path/name;f.parent.mkdir(parents=True,exist_ok=True);f.write_text('synthetic')
    settings={'robot':{'ip':'127.0.0.1','calibration_file':str(tmp_path/'robot.yaml')},'teleop':deepcopy(DEFAULTS)}
    encoded=e.context(settings,tmp_path,tmp_path/'engineering_record.json',tmp_path/'tracking.json')
    baseline=e.identity(settings,tmp_path,tmp_path/'tracking.json')
    results={k:{'observed':True,'physical_pass':True,'details':{'synthetic_fixture':True}} for k in e.CHECKS}
    return tmp_path,settings,encoded,baseline,results


def test_first_launch_and_restart(installation):
    root,settings,encoded,baseline,results=installation
    assert not e.Verification(encoded).refresh({})
    e.save_setup(encoded,{},baseline,results,'Synthetic test assessment only')
    v=e.Verification(encoded)
    assert v.refresh({}) and e.Verification(encoded).refresh({})
    data=json.loads((root/'verified_setup.json').read_text())
    assert not {'enable_requested','valid_until','evidence','engineer'} & data.keys()
    assert (root/'verified_setup.json').stat().st_mode & 0o777 == 0o600


@pytest.mark.parametrize('kind',['mapping','calibration','tracking','control','corrupt','schema','limit','mode','missing'])
def test_reverification(installation,kind):
    root,s,c,b,r=installation;e.save_setup(c,{},b,r,'Synthetic test assessment')
    params={}
    if kind=='mapping':params={'tracking_to_robot_rpy_deg':[0.,0.,30.]}
    elif kind=='limit':params={'max_linear_speed_m_s':10.}
    elif kind=='mode':params={'teleop_control_mode':'position'}
    elif kind in ('corrupt','schema'):(root/'verified_setup.json').write_text('broken' if kind=='corrupt' else '{"version":42}')
    elif kind=='missing':(root/'verified_setup.json').unlink()
    else:
        name={'calibration':'robot.yaml','tracking':'tracking.json','control':'ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/tracking.py'}[kind]
        (root/name).write_text('changed')
    v=e.Verification(c);assert not v.refresh(params)
    assert 'Setup Wizard' in v.reason and str(root) not in v.reason


def test_ui_and_documentation_do_not_invalidate(installation):
    root,s,c,b,r=installation;e.save_setup(c,{},b,r,'Synthetic test assessment')
    for name in ('gui.py','setup_wizard.py'):
        (root/'ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop'/name).write_text('new styling')
    (root/'README.md').write_text('documentation change')
    assert e.Verification(c).refresh({})
    assert e.Verification(c).refresh({'max_linear_speed_m_s':.01})


def test_no_checkboxes_only_and_atomic_failure(installation,monkeypatch):
    root,s,c,b,r=installation
    with pytest.raises(ValueError):e.save_setup(c,{},b,{},'checked boxes')
    e.save_setup(c,{},b,r,'Synthetic test assessment')
    previous=(root/'verified_setup.json').read_bytes()
    monkeypatch.setattr(e.os,'replace',lambda *a: (_ for _ in ()).throw(OSError('disk error')))
    with pytest.raises(OSError):e.save_setup(c,{},b,r,'Another assessment')
    assert (root/'verified_setup.json').read_bytes()==previous
    assert not list(root.glob('tmp*'))


def status(**changes):
    d=dict(session_id='fixture',effective_parameters=deepcopy(DEFAULTS),mode='LIVE OUTPUT',
        engineering_required=False,setup_feedback_fresh=True,tracking_valid=True,observations_healthy=True,armed=True,
        enable_requested=True,a_held=True,clutch='ACTIVE',motion_ready=True,stationary=False,
        accepted_targets=1,controller_state_fresh=True,setup_controllers_inactive=False,
        velocity_diagnostics={'published_twist':[.01,0,0,0,0,0], 'measured_tcp_velocity':[.01,0,0,0,0,0],
                              'raw_pose':[0,0,0,0,0,0,1]})
    d.update(changes);return d


def preview():
    return status(mode='DRY RUN',armed=False,enable_requested=False,a_held=False,
                  setup_controllers_inactive=True,optical=True)


def stopped(d):
    d=deepcopy(d);d.update(stationary=True,clutch='BLOCKED')
    d['velocity_diagnostics']['published_twist']=[0.]*6
    d['velocity_diagnostics']['measured_tcp_velocity']=[0.]*6
    return d


def observe_stop(name):
    c=SetupChecks();d=status();c.update(d,0);c.begin(name,d)
    c.update(d,.1);d['accepted_targets']=2;c.update(d,.2)
    assert c.phase=='trigger'
    if name=='a_stop':d['a_held']=False
    if name=='tracking_stop':d['tracking_valid']=False
    if name=='software_stop':d['enable_requested']=False
    d=stopped(d)
    for t in (.3,.5,.7,.9):c.update(d,t)
    assert c.results[name]['observed'] and not c.results[name]['physical_pass']
    c.attest(name)
    return c,d


@pytest.mark.parametrize('name',['a_stop','tracking_stop','software_stop'])
def test_measured_stop_and_physical_review(name):
    observe_stop(name)


def test_loss_requires_held_restore_release_fresh_press():
    c,d=observe_stop('tracking_stop')
    d['tracking_valid']=True;c.update(d,1.)
    assert 'fresh_clutch' not in c.results
    d['a_held']=False;c.update(d,1.1)
    d=status(accepted_targets=3);c.update(d,1.2)
    assert c.results['fresh_clutch']['observed']


@pytest.mark.parametrize('bad',['no_pose','no_measured_motion','no_targets','stale','session'])
def test_no_fake_physical_pass(bad):
    c=SetupChecks();d=status();c.update(d,0);c.begin('a_stop',d)
    if bad=='no_pose':d['tracking_valid']=False
    if bad=='no_measured_motion':d['velocity_diagnostics']['measured_tcp_velocity']=None
    c.update(d,.1)
    if bad!='no_targets':d['accepted_targets']=2
    if bad=='session':d['session_id']='new'
    c.update(d,2. if bad=='stale' else .2)
    d=stopped(d);d['a_held']=False
    for t in (2.1,2.3,2.5,2.7):c.update(d,t)
    assert 'a_stop' not in c.results
    with pytest.raises(ValueError):c.attest('a_stop')


def test_preview_pose_capture_and_mapping_change():
    c=SetupChecks();d=preview();d['effective_parameters']['tracking_to_robot_rpy_deg']=[0.,0.,0.]
    c.update(d,0);c.begin('mapping_x',d)
    d['velocity_diagnostics']['raw_pose'][0]=.03;c.update(d,.1);c.capture(d)
    assert c.results['mapping_x']['observed']
    c.attest('mapping_x')
    d['effective_parameters']['tracking_to_robot_rpy_deg']=[0.,0.,90.];c.update(d,.2)
    assert not c.results


def test_preview_loss_not_station_count_only():
    c=SetupChecks();d=preview();c.update(d,0);c.begin('preview_loss',d)
    d['active_station_count']=1;c.update(d,.1)
    assert not c.results
    d.update(tracking_valid=False,optical=False);c.update(d,.2)
    d.update(tracking_valid=True,optical=True);c.update(d,.3)
    assert c.results['preview_loss']['observed']


def test_start_uses_configuration_without_supervised_bypass():
    from test_startup import ready
    d=ready(engineering_verified=False,engineering_required=False,
            alignment_verified=False,optical_loss_test_verified=False)
    s=Startup();s.start(d['session_id']);assert s.step(d,True,0)=='prepare'
    s.stop();assert s.step(d,True,1) is None
    d['configuration_ready']=False
    s.start(d['session_id']);assert s.step(d,True,2) is None


def test_legacy_migration_preserves_limits_and_rejects_changed_control(installation):
    from ur_vr_teleop import legacy_engineering as legacy
    root,s,c,b,r=installation
    report=root/'report.txt';report.write_text('Synthetic fixture only')
    record=dict(version=4,workcell_validated=True,engineer='fixture',workcell='mock',
        evidence=str(report),evidence_sha256=e.digest(report),valid_until='2099-01-01T00:00:00+00:00',
        identity=legacy.identity(s,root,root/'tracking.json'),approved_modes=['velocity'],
        limits={k:s['teleop'][k] for k in e.LIMITS})
    (root/'engineering_record.json').write_text(json.dumps(record))
    migrated=e.migrate_legacy(c,{})
    assert migrated['limits']==record['limits'] and e.Verification(c).refresh({})
    (root/'ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/tracking.py').write_text('new controller behavior')
    with pytest.raises(ValueError,match='Control software changed'):e.migrate_legacy(c,{})


def test_stale_measured_feedback_cannot_verify_stop():
    c=SetupChecks();d=status();c.update(d,0);c.begin('a_stop',d)
    c.update(d,.1);d['accepted_targets']=2;c.update(d,.2)
    d=stopped(d);d.update(a_held=False,setup_feedback_fresh=False)
    for t in (.3,.5,.7,.9):c.update(d,t)
    assert not c.results


def test_beginning_recalibration_retires_old_setup_without_deleting_evidence(installation):
    root,s,c,b,r=installation;e.save_setup(c,{},b,r,'Synthetic test assessment')
    old=(root/'verified_setup.json').read_bytes()
    e.invalidate_setup(c)
    assert not e.Verification(c).refresh({})
    assert next(root.glob('verified_setup.previous-*.json')).read_bytes()==old


def test_mapping_capture_cannot_span_tracking_loss_or_reuse_previous_pass():
    c=SetupChecks();d=preview();d['effective_parameters']['tracking_to_robot_rpy_deg']=[0.,0.,0.]
    c.update(d,0);c.begin('mapping_x',d)
    d['velocity_diagnostics']['raw_pose'][0]=.03;c.capture(d);c.attest('mapping_x')
    c.begin('mapping_x',d)
    assert 'mapping_x' not in c.results
    d['tracking_valid']=False;c.update(d,.1)
    d['tracking_valid']=True;c.update(d,.2)
    with pytest.raises(ValueError):c.capture(d)
    assert 'mapping_x' not in c.results


def test_motion_dependency_upgrade_requires_verification(installation,monkeypatch):
    root,s,c,b,r=installation;e.save_setup(c,{},b,r,'Synthetic test assessment')
    monkeypatch.setattr(e,'motion_packages',lambda:{'servo':'new-version'})
    assert not e.Verification(c).refresh({})
