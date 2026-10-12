"""Validated operator tuning, separate from robot identity and run permissions."""
import json
import math
import os
from pathlib import Path
import tempfile
import yaml


DEFAULTS = {
    'max_linear_speed_m_s': .1, 'max_angular_speed_deg_s': 30.,
    'tracking_to_robot_rpy_deg': [0., 0., -90.], 'max_joint_speed_rad_s': .5,
    'tracking_gain_s_inv': 2., 'max_linear_acceleration_m_s2': .25,
    'max_angular_acceleration_rad_s2': math.pi/2,
    'vr_translation_filter_alpha': .8, 'vr_rotation_filter_alpha': .8,
    'joint_acceleration_warning_rad_s2': 5.,
    'translation_scale': 1./1.2, 'rotation_scale': 1.,
    'vr_translation_deadband_m': .001, 'vr_rotation_deadband_deg': .3,
    'translation_damping_gain': 0., 'rotation_damping_gain': 0.,
    'damping_velocity_filter_s': .05,
    'teleop_control_mode': 'velocity',
    'tracking_station_policy': 'any_station',
    'linear_velocity_filter_s': .04, 'angular_velocity_filter_s': .04,
    'single_station_motion_scale': 1.  # Legacy saved field; ignored by the motion gate,
}
FIELD_PARAMETERS = list(DEFAULTS)


def validate(values):
    if not isinstance(values, dict) or set(values)-set(DEFAULTS):
        raise ValueError('Saved settings contain unsupported fields')
    for key, value in values.items():
        choices = {'teleop_control_mode': ('position', 'velocity'),
                   'tracking_station_policy': ('any_station', 'two_station_strict', 'single_station_experimental')}
        if key in choices:
            if value not in choices[key]:
                raise ValueError(key + ' must be one of ' + str(choices[key]))
            continue
        if key in ('linear_velocity_filter_s', 'angular_velocity_filter_s'):
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= .2:
                raise ValueError(key + ' must be 0–0.2 seconds')
            continue
        if key == 'single_station_motion_scale' and (type(value) not in (int, float) or not .01 <= value <= 1.):
            raise ValueError('Single station motion scale must be 0.01–1')
        if key == 'tracking_to_robot_rpy_deg':
            if not isinstance(value, list) or len(value) != 3:
                raise ValueError('Alignment requires three angles')
            numbers = value
        else:
            numbers = [value]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in numbers):
            raise ValueError(key + ' requires finite numbers')
        if key == 'tracking_to_robot_rpy_deg':
            continue
        if value < 0 or (value == 0 and 'deadband' not in key and not key.endswith('damping_gain')):
            raise ValueError(key + ' must be positive (deadbands may be zero)')
        if 'filter_alpha' in key and value > 1:
            raise ValueError('Filter alpha must be at most 1')
        if key in ('translation_scale', 'rotation_scale') and not .01 <= value <= 10.:
            raise ValueError('Robot/VR mapping must be between 0.01 and 10')
        if key == 'vr_translation_deadband_m' and value > .02:
            raise ValueError('Translation deadband must be 0–20 mm')
        if key == 'vr_rotation_deadband_deg' and value > 10.:
            raise ValueError('Rotation deadband must be 0–10 degrees')
        if key.endswith('damping_gain') and value > 1.:
            raise ValueError('Damping gain must be 0–1 (0 disables)')
        if key == 'damping_velocity_filter_s' and not .005 <= value <= .2:
            raise ValueError('Damping velocity filter must be 5–200 ms')
    return values


def preference_path(config):
    return Path(config).with_name('operator_settings.json')


def load_settings(config):
    return load_settings_with_sources(config)[0]


def load_settings_with_sources(config):
    settings = yaml.safe_load(Path(config).read_text())
    # Managed launches use one immutable effective snapshot for driver and gate.
    if 'effective_sources' in settings:
        settings['teleop']['alignment_verified'] = False
        settings['teleop']['optical_loss_test_verified'] = False
        return settings, settings['effective_sources']
    configured = settings.get('teleop', {})
    sources = {key: str(config) if key in configured else 'built-in default' for key in DEFAULTS}
    settings['teleop'] = dict(DEFAULTS, **configured)
    path = preference_path(config)
    if path.exists():
        document = json.loads(path.read_text())
        if document.get('version') != 1:
            raise ValueError('Unsupported saved settings version: ' + str(path))
        saved = validate(document['parameters'])
        settings['teleop'].update(saved)
        sources.update({key: str(path) for key in saved})
    for key in settings.get('robot', {}):
        sources['robot.' + key] = str(config)
    settings['teleop']['alignment_verified'] = False
    settings['teleop']['optical_loss_test_verified'] = False
    return settings, sources


def write_effective_snapshot(config, directory):
    settings, sources = load_settings_with_sources(config)
    settings['effective_sources'] = sources
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    snapshot = directory/'effective_config.yaml'
    snapshot.write_text(yaml.safe_dump(settings, sort_keys=False))
    rows = []
    for key, source in sources.items():
        value = settings['robot'][key[6:]] if key.startswith('robot.') else settings['teleop'][key]
        rows.append(f'{key} = {value}  [source: {source}]')
    for name, value, source in [
        ('smoothing.max_velocity', settings['teleop']['max_joint_speed_rad_s'], sources['max_joint_speed_rad_s']),
        ('smoothing.max_acceleration', settings['robot'].get('joint_command_acceleration_rad_s2', 1.), sources.get('robot.joint_command_acceleration_rad_s2', 'built-in default')),
        ('smoothing.max_jerk', settings['robot'].get('joint_command_jerk_rad_s3', 10.), sources.get('robot.joint_command_jerk_rad_s3', 'built-in default'))]:
        rows.append(f'{name} = {value}  [source: {source}; robot.joint_command_* for acceleration/jerk]')
    (directory/'effective_settings.log').write_text('\n'.join(rows)+'\n')
    return settings, snapshot


def save_settings(config, values):
    values = validate(values)
    path = preference_path(config)
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            name = stream.name
            json.dump({'version': 1, 'parameters': values}, stream, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name and Path(name).exists():
            Path(name).unlink()
    return path
