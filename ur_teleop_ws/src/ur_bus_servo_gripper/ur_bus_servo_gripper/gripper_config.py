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
    speed: int = 100  # SDK raw units; firmware phase determines physical units
    acceleration: int = 5  # SDK units, nonzero
    rate_ticks_s: float = 100.0
    trigger_min: float = 0.0
    trigger_max: float = 1.0
    deadzone: float = 0.02
    invert: bool = False
    threshold_ticks: int = 2
    jog_ticks: int = 5
    temperature_limit: int = 60
    stall_error_ticks: int = 40
    stall_seconds: float = 2.0

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
                  'temperature_limit': (20, 70), 'stall_error_ticks': (5, 200)}
        for key, (low, high) in ranges.items():
            value = getattr(self, key)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f'{key} must be an integer in {low}..{high}')
        for key in ('rate_ticks_s', 'trigger_min', 'trigger_max', 'deadzone', 'stall_seconds'):
            value = getattr(self, key)
            if type(value) not in (float, int) or not math.isfinite(value):
                raise ValueError(key + ' must be finite')
        if not 1 <= self.rate_ticks_s <= 500 or not 0 <= self.deadzone < .25:
            raise ValueError('Rate must be 1..500 ticks/s; deadzone must be 0..<0.25')
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
