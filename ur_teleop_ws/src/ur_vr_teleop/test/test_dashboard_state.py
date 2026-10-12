from copy import deepcopy
import pytest
from ur_vr_teleop.dashboard_state import (ParameterState, Notifications, equivalent,
    boolean, vector, vector_text, station_text, motion_text)
from ur_vr_teleop.preferences import DEFAULTS, PARAMETERS_BY_KEY, display_value, stored_value


def state():
    s=ParameterState()
    s.receive(deepcopy(DEFAULTS))
    return s


def test_initial_and_normalized_dirty_state():
    s=ParameterState()
    assert s.label()=='Unable to verify running values'
    s.receive(deepcopy(DEFAULTS))
    assert s.label()=='No pending edits'
    s.draft['max_linear_speed_m_s']='0.200'
    assert set(s.changed())=={'max_linear_speed_m_s'}
    s.draft['max_linear_speed_m_s']='0.10000000000001'
    assert not s.changed()
    s.draft['tracking_to_robot_rpy_deg']='0.000, 0, -90.0'
    assert not s.changed()
    s.draft['tracking_to_robot_rpy_deg']='0, 0, -89'
    assert set(s.changed())=={'tracking_to_robot_rpy_deg'}


@pytest.mark.parametrize('text',['bad','nan','inf','-2',''])
def test_invalid_text_preserved(text):
    s=state();s.draft['max_linear_speed_m_s']=text
    assert s.label().startswith('Invalid input')
    with pytest.raises(ValueError):s.values()
    s.receive(DEFAULTS)
    assert s.draft['max_linear_speed_m_s']==text


def test_round_trip_every_registry_unit_and_precision():
    for key,spec in PARAMETERS_BY_KEY.items():
        value=DEFAULTS[key]
        assert equivalent(stored_value(spec,display_value(spec,value)),value)
    value=[1.123456789,2.987654321,-39.999998]
    spec=PARAMETERS_BY_KEY['tracking_to_robot_rpy_deg']
    assert equivalent(value,stored_value(spec,display_value(spec,value)))


def test_hidden_values_modes_defaults_revert():
    s=state();values=dict(DEFAULTS,single_station_motion_scale=.4)
    s.receive(values)
    s.draft['tracking_gain_s_inv']='3'
    s.draft['teleop_control_mode']='position'
    s.draft['teleop_control_mode']='velocity'
    assert s.values()['tracking_gain_s_inv']==3
    s.defaults()
    assert s.values()['single_station_motion_scale']==.4
    s.revert()
    assert not s.changed()


def test_authoritative_readback_and_draft_during_pending():
    s=state();s.draft['max_linear_speed_m_s']='.2'
    s.begin(s.changed(),s.values(),False)
    assert s.effective['max_linear_speed_m_s']==.1
    s.phase='readback'
    assert not s.receive(DEFAULTS)
    assert s.label()=='Waiting for backend confirmation'
    s.draft['max_linear_speed_m_s']='.3'
    assert s.receive(dict(DEFAULTS,max_linear_speed_m_s=.2))
    assert s.effective['max_linear_speed_m_s']==.2
    assert s.values()['max_linear_speed_m_s']==.3
    assert not s.conflicts


def test_external_change_preserves_draft_and_requires_reconciliation():
    s=state();s.draft['max_linear_speed_m_s']='.2'
    s.receive(dict(DEFAULTS,max_linear_speed_m_s=.3,tracking_gain_s_inv=4.))
    assert s.values()['max_linear_speed_m_s']==.2
    assert s.values()['tracking_gain_s_inv']==4.
    assert s.conflicts=={'max_linear_speed_m_s'}
    s.revert();assert not s.conflicts and not s.changed()


def test_incomplete_snapshot_never_confirms_and_restart_needs_native_limit():
    s=state();s.restart={'max_joint_speed_rad_s':.7}
    s.receive(dict(DEFAULTS,max_joint_speed_rad_s=.7),None)
    assert s.restart
    s.revert();assert s.restart
    s.receive(dict(DEFAULTS,max_joint_speed_rad_s=.7),.7)
    assert not s.restart
    s.receive({'max_linear_speed_m_s':.2})
    assert s.label()=='Unable to verify running values'


def test_notification_expiry_error_persistence_and_bounded_history():
    n=Notifications();n.post('Apply confirmed',10)
    assert n.text(14.99)=='Apply confirmed' and not n.text(15)
    n.post('Save failed',20,True)
    assert n.text(1000)=='Save failed'
    n.post('Save: waiting for backend',1000)
    assert n.text(1000)=='Save failed'
    n.post('Servo started',1000)
    assert n.text(1000)=='Save failed'
    n.post('Save confirmed',1000)
    for i in range(100):n.post(str(i),1000)
    assert len(n.history)==40 and n.text(1000)=='99'


@pytest.mark.parametrize('value',[None,[],{},'yes',1])
def test_unknown_boolean_and_malformed_vectors(value):
    assert boolean({'ready':value},'ready') is None
    assert vector({'v':value},'v') is None
    assert vector_text(None)=='Unavailable'


def test_station_and_motion_data_are_not_invented():
    assert station_text({},0)=='Unavailable'
    assert 'signal' in station_text({'optical_stations':{'0':1,'1':0}},0)
    assert 'no recent' in station_text({'optical_stations':{'0':1,'1':0}},1)
    data={'motion_metrics_current':True,'a_held':True,'teleop_control_mode':'position',
          'motion_metrics':{'hand_m':.1,'hand_deg':10.,'target_m':.05,'target_deg':5.}}
    assert '0.1 m' in motion_text(data) and '0.05 m' in motion_text(data)
    data['a_held']=False
    assert 'not current' in motion_text(data)
    data.update(a_held=True,teleop_control_mode='velocity',velocity_diagnostics={'requested_twist':[.1,0,0,0,0,.2]})
    assert 'rad/s' in motion_text(data)
