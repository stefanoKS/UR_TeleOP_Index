import json
from copy import deepcopy
import pytest
from ur_vr_teleop.configuration import Configuration, CALIBRATION_HASH
from ur_vr_teleop.preferences import DEFAULTS

@pytest.fixture
def configured(tmp_path):
    calibration=tmp_path/'robot.yaml';calibration.write_text('kinematics:\n  hash: '+CALIBRATION_HASH+'\n')
    tracking=tmp_path/'tracking.config';tracking.write_text('synthetic tracking configuration; not hardware evidence')
    settings=dict(robot=dict(ip='127.0.0.1',calibration_file=str(calibration)),
                  teleop=dict(deepcopy(DEFAULTS),base_frame='base_link',tool_frame='tool0'))
    return dict(settings=settings,tracking=str(tracking))

def test_no_evidence_record_needed_and_no_flags_fabricated(configured,tmp_path):
    c=Configuration(json.dumps(configured))
    assert c.refresh({}),c.reason
    assert not (tmp_path/'verified_setup.json').exists()
    assert 'physical alignment is an operator responsibility' in c.reason

@pytest.mark.parametrize('fault', ['mapping_missing','default_mapping','invalid_mapping','frames','robot','calibration','tracking'])
def test_unknown_or_invalid_configuration_blocks(configured,fault):
    d=configured;v=d['settings']
    if fault=='mapping_missing':del v['teleop']['tracking_to_robot_rpy_deg']
    if fault=='default_mapping':v['effective_sources']={'tracking_to_robot_rpy_deg':'built-in default'}
    if fault=='invalid_mapping':v['teleop']['tracking_to_robot_rpy_deg']=[0,False,3]
    if fault=='frames':v['teleop']['tool_frame']='unknown'
    if fault=='robot':v['robot']['ip']='not an address'
    if fault=='calibration':v['robot']['calibration_file']='/nonexistent/robot.yaml'
    if fault=='tracking':d['tracking']='/nonexistent/tracking'
    c=Configuration(json.dumps(d));assert not c.refresh({})

def test_changed_calibration_is_not_silently_accepted(configured):
    from pathlib import Path
    c=Configuration(json.dumps(configured));assert c.refresh({})
    path=Path(configured['settings']['robot']['calibration_file'])
    path.write_text(path.read_text()+'# changed robot calibration file\n')
    assert not c.refresh({}) and 'changed during this session' in c.reason

def test_runtime_mapping_is_revalidated_and_not_physically_certified(configured):
    c=Configuration(json.dumps(configured))
    assert c.refresh({'tracking_to_robot_rpy_deg':[0.,0.,40.]})
    assert not c.refresh({'tracking_to_robot_rpy_deg':[float('nan'),0.,40.]})


def test_runtime_frames_must_match_servo(configured):
    c=Configuration(json.dumps(configured))
    assert not c.refresh({'tool_frame':'other_tool'})


@pytest.mark.parametrize('key,value', [('ip',123), ('servoj_gain',True),
    ('servoj_lookahead_time',.001), ('joint_command_acceleration_rad_s2',float('nan')),
    ('joint_command_jerk_rad_s3',-1.)])
def test_invalid_driver_tuning_is_not_configuration_ready(configured,key,value):
    configured['settings']['robot'][key]=value
    assert not Configuration(json.dumps(configured)).refresh({})


@pytest.mark.parametrize('update', ['metadata', 'station_estimate', 'truncate', 'replace'])
def test_libsurvive_persistence_changes_do_not_revoke_configuration(configured, update):
    from pathlib import Path
    path=Path(configured['tracking'])
    c=Configuration(json.dumps(configured));assert c.refresh({})
    if update=='metadata':path.write_text('"variance":["0.0001"], "unlock_count":"1"')
    elif update=='station_estimate':path.write_text('"lighthouse0":{"pose":["1","2","3","1","0","0","0"]}')
    elif update=='truncate':path.write_text('')  # config_save uses fopen(path, "w").
    else:path.unlink()  # replacement window; solver already owns loaded state
    assert c.refresh({}),c.reason
    assert not c.refresh({'tracking_to_robot_rpy_deg':[float('nan'),0.,0.]})


def test_initial_tracking_write_or_missing_file_recovers_without_restart(configured):
    from pathlib import Path
    path=Path(configured['tracking']);path.write_text('')
    c=Configuration(json.dumps(configured));assert not c.refresh({})
    path.unlink();assert not c.refresh({})
    path.write_text('synthetic loaded tracking configuration')
    assert c.refresh({}),c.reason
    path.write_text('normal runtime rewrite')
    assert c.refresh({}),c.reason
    # A new backend still checks initial availability; this is not persisted approval.
    path.write_text('')
    assert not Configuration(json.dumps(configured)).refresh({})
