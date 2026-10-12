import json
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from ur_vr_teleop.preferences import (DEFAULTS, PARAMETER_REGISTRY, PARAMETERS_BY_KEY,
                                      load_settings, save_settings, stored_value, validate)
from ur_vr_teleop.gui import OperatorGUI


def test_saved_tuning_overrides_config_without_restoring_run_permissions(tmp_path):
    config = tmp_path/'real.yaml'
    config.write_text('robot: {ip: 127.0.0.1}\nteleop: {translation_scale: 1.0, alignment_verified: true}\n')
    values = dict(DEFAULTS, max_linear_speed_m_s=.2)
    path = save_settings(config, values)
    result = load_settings(config)
    assert result['teleop']['translation_scale'] == pytest.approx(1/1.2)
    assert result['teleop']['max_linear_speed_m_s'] == .2
    assert result['robot']['ip'] == '127.0.0.1'
    assert not result['teleop']['alignment_verified']
    assert not result['teleop']['optical_loss_test_verified']
    save_settings(config, DEFAULTS)
    assert load_settings(config)['teleop']['max_linear_speed_m_s'] == .1
    assert json.loads(path.read_text())['version'] == 1


@pytest.mark.parametrize('values', [{'enabled': True}, {'max_joint_speed_rad_s': float('nan')},
    {'translation_scale': 0.}, {'rotation_scale': 100.}, {'vr_rotation_filter_alpha': 1.1},
    {'vr_translation_deadband_m': -.1}, {'vr_rotation_deadband_deg': 11.},
    {'tracking_to_robot_rpy_deg': [0., 0.]}, {'max_joint_speed_rad_s': True}])
def test_invalid_preferences_do_not_replace_previous_save(tmp_path, values):
    path = save_settings(tmp_path/'config.yaml', DEFAULTS)
    original = path.read_text()
    with pytest.raises(ValueError):
        save_settings(tmp_path/'config.yaml', values)
    assert path.read_text() == original


def test_parameter_registry_covers_every_saved_key_and_named_gui_values():
    assert set(PARAMETERS_BY_KEY) == set(DEFAULTS)
    assert len(PARAMETER_REGISTRY) == len(DEFAULTS)
    fields = {spec.key: Mock(get=Mock(return_value=str(
        1 / DEFAULTS[spec.key] if spec.display_inverse else
        DEFAULTS[spec.key] * spec.display_multiplier if isinstance(DEFAULTS[spec.key], (int, float)) else
        ','.join(map(str, DEFAULTS[spec.key])) if isinstance(DEFAULTS[spec.key], list) else DEFAULTS[spec.key])))
        for spec in PARAMETER_REGISTRY if spec.editable}
    panel = SimpleNamespace(fields=fields, running_values={})
    assert OperatorGUI.field_values(panel) == DEFAULTS
    fields['translation_scale'].get.return_value = '0'
    with pytest.raises(ValueError): OperatorGUI.field_values(panel)


def test_hidden_compatibility_parameters_are_not_editable_or_submitted():
    hidden = [spec for spec in PARAMETER_REGISTRY if not spec.editable]
    assert {spec.key for spec in hidden} == {'tracking_station_policy', 'single_station_motion_scale'}
    assert all(spec.key not in {item.key for item in PARAMETER_REGISTRY if item.editable} for spec in hidden)


def test_save_waits_for_backend_acceptance_and_records_requested_restart_limit(tmp_path):
    from rclpy.parameter import Parameter
    values = dict(DEFAULTS, max_joint_speed_rad_s=.7)
    future = object()
    panel = SimpleNamespace(field_values=lambda: values.copy(), settings=Mock(), pending=[],
        pending_settings={}, last_status={'hardware_smoothing': {'velocity': .5}},
        alignment=Mock(get=lambda: False), optical=Mock(get=lambda: False), message=Mock(),
        config_path=tmp_path/'config.yaml', stale=lambda: False,
        fields={spec.key: Mock() for spec in PARAMETER_REGISTRY if spec.editable})
    panel.settings.call_async.return_value = future
    OperatorGUI.apply(panel, save=True)
    assert not (tmp_path/'operator_settings.json').exists()
    request = panel.settings.call_async.call_args.args[0]
    assert 'max_joint_speed_rad_s' not in [p.name for p in request.parameters]
    assert Parameter.from_parameter_msg(next(p for p in request.parameters if p.name == 'translation_scale')).value == pytest.approx(1/1.2)
    OperatorGUI.complete_settings(panel, future, SimpleNamespace(successful=False, reason='A is held'))
    assert not (tmp_path/'operator_settings.json').exists()
    panel.pending.clear()
    OperatorGUI.apply(panel, save=True)
    OperatorGUI.complete_settings(panel, future, SimpleNamespace(successful=True))
    document = json.loads((tmp_path/'operator_settings.json').read_text())
    assert document['parameters']['max_joint_speed_rad_s'] == .7
    assert 'alignment_verified' not in document['parameters']
    assert 'Restart ROS' in panel.message.set.call_args.args[0]


def test_save_failure_is_distinguished_from_runtime_apply_failure(tmp_path, monkeypatch):
    future = object()
    panel = SimpleNamespace(pending_settings={future: DEFAULTS}, config_path=tmp_path/'config.yaml',
                            last_status={}, message=Mock())
    def fail(*args): raise OSError('Disk full')
    monkeypatch.setattr('ur_vr_teleop.gui.save_settings', fail)
    OperatorGUI.complete_settings(panel, future, SimpleNamespace(successful=True))
    assert 'applied but not saved' in panel.message.set.call_args.args[0]


def test_damping_saves_and_old_profiles_remain_loadable(tmp_path):
    config = tmp_path/'config.yaml'
    config.write_text('robot: {}\nteleop: {translation_damping_gain: 1.0, rotation_damping_gain: 1.0, damping_velocity_filter_s: 0.05}\n')
    save_settings(config, {'max_linear_speed_m_s': .2})
    assert load_settings(config)['teleop']['translation_damping_gain'] == 1.
    save_settings(config, dict(DEFAULTS, translation_damping_gain=.7, rotation_damping_gain=0., damping_velocity_filter_s=.08))
    result = load_settings(config)['teleop']
    assert result['translation_damping_gain'] == .7 and result['rotation_damping_gain'] == 0.
    assert result['damping_velocity_filter_s'] == .08


def test_effective_snapshot_is_immutable_and_reports_sources(tmp_path):
    from ur_vr_teleop.preferences import write_effective_snapshot, load_settings_with_sources
    config = tmp_path/'config.yaml'
    config.write_text('robot: {servoj_lookahead_time: 0.03}\nteleop: {tracking_gain_s_inv: 2.0}\n')
    save_settings(config, {'max_joint_speed_rad_s': .5, 'translation_damping_gain': 0.})
    settings, snapshot = write_effective_snapshot(config, tmp_path/'run')
    assert settings['teleop']['max_joint_speed_rad_s'] == .5
    save_settings(config, {'max_joint_speed_rad_s': 1.})
    restored, sources = load_settings_with_sources(snapshot)
    assert restored['teleop']['max_joint_speed_rad_s'] == .5
    assert sources['max_joint_speed_rad_s'].endswith('operator_settings.json')
    assert sources['tracking_gain_s_inv'] == str(config)
    assert restored['robot']['servoj_lookahead_time'] == .03
    assert 'joint_command' in (tmp_path/'run/effective_settings.log').read_text()


def test_servo_halts_are_distinguished_from_invalid_status():
    from ur_vr_teleop.commissioning import servo_status_detail
    assert 'singularity' in servo_status_detail(2)
    assert 'collision' in servo_status_detail(4)
    assert 'joint position or velocity' in servo_status_detail(5)
    assert 'Invalid' in servo_status_detail(-1)
    assert 'Unknown' in servo_status_detail(99)
