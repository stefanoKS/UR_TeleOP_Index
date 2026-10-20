"""Separate, versioned gripper calibration; never changes arm preferences."""
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import tempfile


@dataclass(frozen=True)
class Config:
    version: int = 1
    enabled: bool = False
    port: str = ''
    servo_id: int = -1  # discovery, never an assumed ID
    open_ticks: int = 0
    closed_ticks: int = 0
    min_ticks: int = 0
    max_ticks: int = 0
    # SDK fields are raw vendor units; the firmware determines physical speed.
    # Acceleration is a one-byte field (0..255, 255 reserved), so 254 is the
    # highest verified nonzero application value. Speed is a 16-bit field, but
    # 1000 remains the reviewed ceiling for ST3215 firmware.
    speed: int = 100
    acceleration: int = 5
    # Software target advancement per monotonic second. This bounds setpoint
    # generation only; it is not a claim that the servo can reach this rate.
    rate_ticks_s: float = 100.0
    # Maximum software target distance ahead of fresh measured feedback.
    max_target_lead_ticks: int = 20
    trigger_min: float = 0.0
    trigger_max: float = 1.0
    deadzone: float = 0.02
    invert: bool = False
    threshold_ticks: int = 2
    jog_ticks: int = 5
    temperature_limit: int = 60
    stall_error_ticks: int = 40
    stall_seconds: float = 2.0
    # Follow continuity across a short verified input/serial interruption.
    follow_resume_window_s: float = 2.0
    follow_verify_s: float = 0.15
    # Bounded read-only feedback retries; a write is never retried.
    comm_retry_count: int = 2
    comm_retry_window_s: float = 0.5

    def validate(self, calibrated=False):
        if type(self.version) is not int or self.version != 1:
            raise ValueError('Unsupported gripper configuration version')
        for key in ('enabled', 'invert'):
            if type(getattr(self, key)) is not bool:
                raise ValueError(key + ' must be boolean')
        if not isinstance(self.port, str) or (self.port and not self.port.startswith('/dev/')):
            raise ValueError('Select a /dev/serial/by-id path or manual /dev port')
        ranges = {'servo_id': (-1, 253), 'open_ticks': (0, 4095), 'closed_ticks': (0, 4095),
                  'min_ticks': (0, 4095), 'max_ticks': (0, 4095), 'speed': (1, 1000),
                  'acceleration': (1, 254), 'threshold_ticks': (1, 20), 'jog_ticks': (1, 20),
                  'max_target_lead_ticks': (5, 60),
                  'temperature_limit': (20, 70), 'stall_error_ticks': (5, 200),
                  'comm_retry_count': (0, 5)}
        for key, (low, high) in ranges.items():
            value = getattr(self, key)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f'{key} must be an integer in {low}..{high}')
        for key in ('rate_ticks_s', 'trigger_min', 'trigger_max', 'deadzone',
                    'stall_seconds', 'follow_resume_window_s', 'follow_verify_s',
                    'comm_retry_window_s'):
            value = getattr(self, key)
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError(key + ' must be finite')
        if not 1 <= self.rate_ticks_s <= 2000 or not 0 <= self.deadzone < .25:
            raise ValueError('Rate must be 1..2000 ticks/s; deadzone must be 0..<0.25')
        if not .5 <= self.follow_resume_window_s <= 10:
            raise ValueError('Follow resume window must be 0.5..10 s')
        if not .05 <= self.follow_verify_s <= 1:
            raise ValueError('Follow verification interval must be 0.05..1 s')
        if not .1 <= self.comm_retry_window_s <= 2:
            raise ValueError('Communication retry window must be 0.1..2 s')
        if self.trigger_max - self.trigger_min < .01 or not .2 <= self.stall_seconds <= 5:
            raise ValueError('Invalid trigger range or stall observation interval')
        if calibrated or self.enabled:
            if self.servo_id < 0 or not self.min_ticks <= min(self.open_ticks, self.closed_ticks) < max(self.open_ticks, self.closed_ticks) <= self.max_ticks:
                raise ValueError('Review safe travel and distinct OPEN/CLOSED endpoints; verify servo ID')
        return self


def load(path):
    path = Path(path)
    return Config(**json.loads(path.read_text())).validate() if path.exists() else Config()


def save(path, config):
    config.validate()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            name = stream.name
            json.dump(asdict(config), stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)
