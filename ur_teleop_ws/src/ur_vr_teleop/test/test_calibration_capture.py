"""Deterministic controller-only measurements. No robot, ROS or motion publisher."""
import json
from dataclasses import replace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from ur_vr_teleop.calibration_capture import Capture, CaptureConfig, STEPS, analyze
from ur_vr_teleop.calibration_observer import capture_isolation


class Stream:
    def __init__(self, cfg=None):
        self.c=Capture(cfg);self.t=1.;self.p=np.zeros(3);self.q=Rotation.identity();self.held=False
        self.session='one';self.params='unchanged';self.isolated=True
        self.run(.4)
    def sample(self, pose=True, joy=True, flags=True):
        self.t+=.01
        self.c.context_update(self.session,self.params,self.isolated,self.t)
        if flags:
            self.c.flag('pose',True,self.t);self.c.flag('optical',True,self.t)
        if pose:
            noise=np.sin(self.t*83)*.00001
            self.c.pose(int(self.t*1e9),self.p+[noise,0,0],self.q.as_quat(),self.t)
        if joy:self.c.joy(self.held,self.t)
        self.c.tick(self.t)
    def run(self,seconds,**kw):
        for _ in range(round(seconds/.01)):self.sample(**kw)
    def move(self,delta=None,angle=None):
        p=self.p.copy();q=self.q
        for factor in np.linspace(0,1,61):
            if delta is not None:self.p=p+np.asarray(delta)*factor
            if angle is not None:self.q=Rotation.from_rotvec(np.asarray(angle)*factor)*q
            self.sample()
    def capture(self,step=0,delta=None,angle=None):
        self.c.retry(step);self.run(.1)
        self.held=True;self.run(.4)
        assert self.c.state=='RECORDING',self.c.snapshot()
        self.move(delta,angle);self.run(.35)
        assert self.c.end is not None
        self.held=False;self.run(.22)
        return self.c.last_report


@pytest.mark.parametrize('axis',range(3))
def test_translation_all_axes_and_robust_endpoint(axis):
    s=Stream();delta=np.eye(3)[axis]*.05
    r=s.capture(axis,delta=delta)
    assert r['passed'] and r['requested_axis']==pytest.approx(.05,abs=.0001)
    assert r['accepted_samples']>90 and r['source_hz']==pytest.approx(100)
    assert r['tracking_dropouts']==0
    assert max(r['position_jitter_m'])<.0001
    assert s.c.state=='PASSED'
    s.run(.7);assert s.c.step==next(i for i in range(6) if i!=axis)


@pytest.mark.parametrize('axis',range(3))
def test_rotation_all_axes_and_sign_ambiguity(axis):
    s=Stream();r=s.capture(axis+3,angle=np.eye(3)[axis]*np.deg2rad(25))
    assert r['passed'] and r['requested_axis']==pytest.approx(np.deg2rad(25),abs=1e-8)


@pytest.mark.parametrize('delta,reason',[( [0,.05,0],'Wrong dominant'),([-.05,0,0],'Wrong direction'),
    ([.01,0,0],'too small'),([.04,.03,0],'perpendicular')])
def test_bad_translations_retry(delta,reason):
    s=Stream();r=s.capture(delta=delta)
    assert not r['passed'] and any(reason in x for x in r['failure_reasons'])
    assert s.c.state=='FAILED_RETRY' and not s.c.results
    # No mouse needed to retry: new genuine A edge after a healthy release.
    s.held=True;s.run(.4);assert s.c.capturing


def test_wrong_rotation_axis_and_quaternion_sign():
    s=Stream();r=s.capture(3,angle=[0,.4,0]);assert not r['passed']
    s=Stream();s.held=True;s.run(.4)
    for _ in range(40):
        s.t+=.01;s.c.context_update('one','unchanged',True,s.t)
        for name in ('pose','optical'):s.c.flag(name,True,s.t)
        s.c.pose(int(s.t*1e9),[1e-5*np.sin(s.t*83),0,0],[0,0,0,-1],s.t);s.c.joy(True,s.t)
    s.held=False;s.run(.22)
    assert not s.c.last_report['passed'] and s.c.last_report['magnitude']<.001


def test_nonidentity_mapping_and_179_degree_orientation_wrap():
    s=Stream();s.c.mapping=[0,0,90]
    r=s.capture(0,delta=[0,-.04,0]);assert r['passed']
    s=Stream();s.q=Rotation.from_euler('x',179,degrees=True);s.c.last_pose=None;s.run(.4)
    r=s.capture(3,angle=[.3,0,0]);assert r['passed']


def test_duplicate_press_bounce_and_early_release():
    s=Stream();s.held=True;s.run(.02);s.held=False;s.run(.08)
    assert not s.c.capturing
    s.held=True;s.run(.1);start=s.c.started
    s.run(.1);assert s.c.started==start
    s.held=False;s.run(.1)
    assert not s.c.results and 'early' in s.c.reason


def test_held_before_readiness_requires_release():
    s=Stream();s.isolated=False;s.held=True;s.run(.3)
    s.isolated=True;s.run(.4);assert not s.c.capturing
    s.held=False;s.run(.1);s.held=True;s.run(.1);assert s.c.capturing


@pytest.mark.parametrize('fault',['pose','optical','joy','session','parameters','isolation','stop'])
def test_interruptions_never_complete_on_synthetic_release(fault):
    s=Stream();s.held=True;s.run(.4);s.move([.05,0,0]);s.run(.3)
    if fault=='pose':s.run(.3,pose=False)
    elif fault=='joy':s.run(.3,joy=False)
    elif fault=='optical':s.c.flag('optical',False,s.t)
    elif fault=='session':s.session='new';s.sample()
    elif fault=='parameters':s.params='changed';s.sample()
    elif fault=='isolation':s.isolated=False;s.sample()
    else:s.c.cancel('STOP')
    s.held=False;s.run(.3)
    assert not s.c.results


@pytest.mark.parametrize('bad',['nan','quaternion','jump','rotation_jump','reorder'])
def test_bad_samples_abort(bad):
    s=Stream();s.held=True;s.run(.4)
    p=[0,0,0];q=[0,0,0,1];stamp=int((s.t+.01)*1e9)
    if bad=='nan':p[0]=float('nan')
    if bad=='quaternion':q=[0,0,0,0]
    if bad=='jump':p[0]=.1
    if bad=='rotation_jump':q=Rotation.from_euler('x',90,degrees=True).as_quat()
    if bad=='reorder':stamp-=100000000
    s.c.pose(stamp,p,q,s.t+.01)
    assert not s.c.capturing and not s.c.results


def test_duplicate_timestamp_does_not_refresh_receipt():
    s=Stream();p=s.c.last_pose.copy()
    s.c.pose(p['source_ns'],p['position'],p['quaternion'],s.t+.1)
    assert s.c.duplicates==1 and s.c.last_pose['receipt']==p['receipt']


def test_bounded_time_buffer_and_stationary_noise():
    s=Stream(replace(CaptureConfig(),maximum_capture_s=.5));s.held=True;s.run(.7)
    assert not s.c.results and not s.c.capturing
    s=Stream(replace(CaptureConfig(),maximum_samples=30));s.held=True;s.run(.7)
    assert len(s.c.samples)<=30 and not s.c.results
    s=Stream();r=s.capture(delta=[0,0,0]);assert not r['passed']


def isolated_status():
    return dict(managed_session_id='token',session_id='session',motion_output_enabled=False,
        command_publisher_present=False,armed=False,enable_requested=False,controller_state_fresh=True,
        setup_controllers_inactive=True,observations_healthy=True,setup_feedback_fresh=True,tracking_valid=True)


@pytest.mark.parametrize('field',list(isolated_status()))
def test_every_isolation_fact_required(field):
    d=isolated_status();assert capture_isolation(d,'token',True,True,True,0)
    d[field]=None
    assert not capture_isolation(d,'token',True,True,True,0)


@pytest.mark.parametrize('args',[('other',True,True,True,0),('token',False,True,True,0),
    ('token',True,False,True,0),('token',True,True,False,0),('token',True,True,True,1)])
def test_independent_readback_and_graph_requirements(args):
    assert not capture_isolation(isolated_status(),*args)


def test_all_six_advance_without_step_selection():
    s=Stream()
    for i in range(6):
        assert s.c.step==i
        s.held=True;s.run(.4)
        axis=np.eye(3)[i%3]
        s.move(delta=axis*.04 if i<3 else None,angle=axis*.3 if i>=3 else None)
        s.run(.35);s.held=False;s.run(.9)
    assert s.c.state=='REVIEW' and len(s.c.results)==6
    assert all(r['passed'] for r in s.c.results.values())


def test_idle_preview_predicate_is_not_relaxed():
    from ur_vr_teleop.setup_checks import SetupChecks
    from test_setup import preview
    d=preview();assert SetupChecks.preview(d)
    d['a_held']=True;assert not SetupChecks.preview(d)


def test_new_readiness_change_is_not_hidden_by_old_observational_transition( tmp_path, monkeypatch):
    from ur_vr_teleop import engineering as e
    from test_setup import installation
    root,settings,encoded,baseline,results=installation.__wrapped__(tmp_path,monkeypatch)
    project=__import__('pathlib').Path(__file__).resolve().parents[4]
    path='ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/commissioning.py'
    data=json.loads((project/'ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/setup_source_compatibility.json').read_text())
    current,original=next(iter(data[path].items()))
    (root/path).write_bytes((project/path).read_bytes())
    bound=e.identity(settings,root,root/'tracking.json')
    assert bound['software'][path]==e.digest(project/path)
    assert bound['software'][path]!=original  # Do not broaden the old compatibility exception.
    e.save_setup(encoded,{},bound,results,'Synthetic preexisting V5 evidence')
    assert e.Verification(encoded).refresh({})
    (root/path).write_text((root/path).read_text()+'\n# Simulated motion-code change\n')
    assert not e.Verification(encoded).refresh({})


def test_mouse_fallback_uses_same_samples_and_isolation():
    s=Stream();s.c.mouse_start(s.t);s.run(.4);s.move([.05,0,0]);s.run(.3)
    s.c.mouse_finish(s.t);s.run(.2)
    assert s.c.results['mapping_x']['passed']
    s.run(.7);s.held=True;s.run(.4)
    assert s.c.capturing and s.c.step==1
    s=Stream();s.isolated=False;s.sample();s.c.mouse_start(s.t)
    assert not s.c.capturing


def test_source_order_watermark_survives_rejected_samples():
    s=Stream();stamp=s.c.last_pose['source_ns']
    s.c.pose(stamp-1,[0,0,0],[0,0,0,1],s.t)
    assert s.c.last_pose is None
    accepted=s.c.accepted
    s.c.pose(stamp-2,[0,0,0],[0,0,0,1],s.t)
    assert s.c.accepted==accepted and s.c.last_pose is None
    s.sample();assert s.c.last_pose is not None


def test_mouse_fallback_cannot_overwrite_completed_review():
    s=Stream();s.capture(delta=[.05,0,0])
    report=s.c.results['mapping_x'].copy()
    s.c.mouse_start(s.t)
    assert not s.c.capturing and s.c.results['mapping_x']==report
