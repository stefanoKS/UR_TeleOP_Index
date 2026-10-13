"""Synthetic engineering evidence tests, never proof of physical validation."""
import json
from pathlib import Path
from copy import deepcopy
import pytest
from ur_vr_teleop.legacy_engineering import Verification, context, identity, digest, LIMITS, VERSION, main
from ur_vr_teleop.preferences import DEFAULTS


@pytest.fixture
def record(tmp_path):
    root=tmp_path
    (root/'dependencies.repos').write_text('pinned')
    (root/'ur_teleop/config').mkdir(parents=True)
    (root/'ur_teleop/config/commissioning_controllers.yaml').write_text('controller')
    calibration=root/'robot.yaml';calibration.write_text('kinematics')
    tracking=root/'tracking.json';tracking.write_text('tracking calibration')
    report=root/'report.txt';report.write_text('SYNTHETIC FIXTURE — NO HARDWARE VALIDATION')
    settings={'robot':{'ip':'127.0.0.1','calibration_file':str(calibration)},'teleop':dict(DEFAULTS)}
    path=root/'engineering_record.json'
    data=dict(version=VERSION,workcell_validated=True,engineer='fixture',workcell='mock',
              evidence=str(report),evidence_sha256=digest(report),valid_until='2099-01-01T00:00:00+00:00',
              identity=identity(settings,root,tracking),approved_modes=['velocity'],limits={k:settings['teleop'][k] for k in LIMITS})
    path.write_text(json.dumps(data))
    return Verification(context(settings,root,path,tracking)),settings,data,path,tracking


def test_valid_record_survives_restart_without_motion_permission(record):
    v,s,d,p,t=record
    assert v.refresh({})
    assert Verification(context(s,p.parent,p,t)).refresh({})
    assert 'enable_requested' not in d


@pytest.mark.parametrize('change', ['missing','malformed','expired','version','unvalidated','mapping','calibration','tracking','software','evidence','limit','filter'])
def test_changed_or_unverified_installation_fails_closed(record,change):
    v,s,d,p,t=record
    params={}
    if change=='missing':p.unlink()
    elif change=='malformed':p.write_text('bad')
    elif change in ('expired','version','unvalidated'):
        d[{'expired':'valid_until','version':'version','unvalidated':'workcell_validated'}[change]]={'expired':'2000-01-01T00:00:00+00:00','version':3,'unvalidated':False}[change]
        p.write_text(json.dumps(d))
    elif change=='mapping':params={'tracking_to_robot_rpy_deg':[0.,0.,42.]}
    elif change=='calibration':Path(s['robot']['calibration_file']).write_text('changed')
    elif change=='tracking':t.write_text('changed')
    elif change=='software':(p.parent/'dependencies.repos').write_text('changed')
    elif change=='evidence':Path(d['evidence']).write_text('changed')
    elif change=='limit':params={'max_linear_speed_m_s':100.}
    else:params={'linear_velocity_filter_s':.12}
    assert not v.refresh(params) and 'ENGINEERING SETUP REQUIRED' in v.reason


def test_lower_limits_allowed_but_no_record_cannot_be_replaced_by_flags(record):
    v,s,d,p,t=record
    assert v.refresh({'max_linear_speed_m_s':.01,'alignment_verified':True,'optical_loss_test_verified':True})
    p.unlink()
    assert not v.refresh({'alignment_verified':True,'optical_loss_test_verified':True})


def test_only_reviewed_modes_and_mapping_ratios_are_reusable(record):
    v,s,d,p,t=record
    assert v.refresh({'translation_scale':.1})
    assert not v.refresh({'rotation_scale':10.})
    assert not v.refresh({'teleop_control_mode':'position'})
    d['approved_modes'].append('position');p.write_text(json.dumps(d))
    assert v.refresh({'teleop_control_mode':'position'})
