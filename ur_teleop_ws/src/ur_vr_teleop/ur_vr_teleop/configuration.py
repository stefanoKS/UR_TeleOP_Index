"""Configuration integrity, not commissioning evidence or physical validation.

Called by the backend's file worker, never by the command executor. A configured
mapping is explicit input; syntactic validity does not prove physical alignment.
"""
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import yaml
from .preferences import DEFAULTS, validate

CALIBRATION_HASH = 'calib_2528998354424493788'


class Configuration:
    def __init__(self, encoded):
        self.context = json.loads(encoded)
        self.valid = False
        self.reason = 'Waiting for configuration check'
        self.technical = ''
        self.file_identity = None

    def refresh(self, parameters):
        self.valid = False
        try:
            settings = self.context['settings']
            robot, original = settings['robot'], settings['teleop']
            values = dict(original, **parameters)
            # Defaults alone must never establish an installation's mapping.
            source = settings.get('effective_sources', {}).get('tracking_to_robot_rpy_deg')
            if 'tracking_to_robot_rpy_deg' not in original or source == 'built-in default':
                raise ValueError('Tracking-to-robot mapping is missing; configure physical alignment')
            validate({key: values[key] for key in DEFAULTS})
            if not isinstance(robot['ip'], str):
                raise ValueError('Robot IP must be an explicit address string')
            ipaddress.ip_address(robot['ip'])
            gain = robot.get('servoj_gain', 500)
            if type(gain) is not int or not 100 <= gain <= 2000:
                raise ValueError('servoj_gain must be an integer from 100 to 2000')
            for key, default in (('servoj_lookahead_time', .08),
                                 ('joint_command_acceleration_rad_s2', 1.), ('joint_command_jerk_rad_s3', 10.)):
                value = robot.get(key, default)
                if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                    raise ValueError(key+' must be finite and positive')
                if key == 'servoj_lookahead_time' and not .03 <= value <= .2:
                    raise ValueError('servoj_lookahead_time must be between .03 and .2 s')
            if any(original.get(k) != expected or values.get(k) != expected
                   for k, expected in (('base_frame', 'base_link'), ('tool_frame', 'tool0'))):
                raise ValueError('Configured frames must match Servo: base_link and tool0')
            calibration = Path(robot['calibration_file']).expanduser().read_bytes()
            data = yaml.safe_load(calibration)
            if data.get('kinematics', {}).get('hash') != CALIBRATION_HASH:
                raise ValueError('Unexpected robot calibration hash; review robot configuration')
            tracking = Path(self.context['tracking']).expanduser().read_bytes()
            if not tracking.strip():
                raise ValueError('Tracking calibration file is empty')
            identity = tuple(hashlib.sha256(v).hexdigest() for v in (calibration, tracking))
            if self.file_identity is not None and identity != self.file_identity:
                raise ValueError('Calibration files changed during this session; review mapping and restart')
            self.file_identity = identity
            self.valid = True
            self.reason = 'Configuration present and consistent; physical alignment is an operator responsibility'
            self.technical = ''
        except (OSError, ValueError, TypeError, KeyError, AttributeError, yaml.YAMLError) as error:
            self.reason = 'Configuration unavailable: ' + str(error)
            self.technical = repr(error)
        return self.valid
