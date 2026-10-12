"""Pure dashboard state; no ROS, Tk, process, or motion-control side effects."""
from collections import deque
from copy import deepcopy
import math
from .preferences import DEFAULTS, PARAMETERS_BY_KEY, display_value, stored_value, validate

EDITABLE = tuple(k for k, s in PARAMETERS_BY_KEY.items() if s.editable)


def display_text(spec,value):
    shown=display_value(spec,value)
    return format(shown,'.10g') if type(shown) in (int,float) else str(shown)


def equivalent(a, b):
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(equivalent(x, y) for x, y in zip(a, b))
    if type(a) in (float, int) and type(b) in (float, int):
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-10)
    return type(a) is type(b) and a == b


class ParameterState:
    def __init__(self):
        self.effective = {}
        self.draft = {k: display_text(PARAMETERS_BY_KEY[k], DEFAULTS[k]) for k in EDITABLE}
        self.saved = {}
        self.restart = {}
        self.pending = None
        self.conflicts = set()
        self.confirmed = False
        self.phase = ''

    def values(self):
        values = deepcopy(self.effective or DEFAULTS)
        for key, text in self.draft.items():
            try:
                values[key] = stored_value(PARAMETERS_BY_KEY[key], text)
                validate({key: values[key]})
            except (ValueError, TypeError, OverflowError) as error:
                raise ValueError(f'{key}: {error}') from error
        return validate(values)

    def changed(self):
        values = self.values()
        return {k: values[k] for k in EDITABLE if not equivalent(values[k], self.effective.get(k))}

    def receive(self, values, active_joint=None):
        if not isinstance(values, dict) or set(values) != set(DEFAULTS):
            self.confirmed = False
            return False
        try:
            validate(values)
        except (ValueError, TypeError):
            self.confirmed = False
            return False
        old = self.effective
        for key in EDITABLE:
            try:
                edited = not equivalent(stored_value(PARAMETERS_BY_KEY[key], self.draft[key]), old.get(key, DEFAULTS[key]))
            except (ValueError, TypeError):
                edited = True
            if old and not equivalent(values[key], old[key]) and edited:
                submitted = self.pending and equivalent(values[key], self.pending['applied'].get(key))
                if not submitted:
                    self.conflicts.add(key)
            if not edited:
                self.draft[key] = display_text(PARAMETERS_BY_KEY[key], values[key])
        self.effective = deepcopy(values)
        self.confirmed = True
        if active_joint is not None and 'max_joint_speed_rad_s' in self.restart:
            wanted = self.restart['max_joint_speed_rad_s']
            if equivalent(wanted, active_joint) and equivalent(wanted, values['max_joint_speed_rad_s']):
                self.restart.clear()
        if self.pending and self.phase == 'readback':
            if all(equivalent(values[k], v) for k, v in self.pending['applied'].items() if k in DEFAULTS):
                self.pending = None
                self.phase = ''
                return True
        return False

    def begin(self, applied, values, save):
        self.pending = deepcopy(dict(applied=applied, values=values, save=save))
        self.phase = 'applying'

    def revert(self):
        if not self.confirmed:
            raise ValueError('Unable to verify running values')
        self.draft = {k: display_text(PARAMETERS_BY_KEY[k], self.effective[k]) for k in EDITABLE}
        self.conflicts.clear()

    def defaults(self):
        self.draft = {k: display_text(PARAMETERS_BY_KEY[k], DEFAULTS[k]) for k in EDITABLE}

    def label(self, fresh=True):
        try:
            changed = self.changed()
        except ValueError:
            return 'Invalid input — review highlighted field'
        if self.phase == 'applying':
            return 'Applying changes...'
        if self.phase == 'readback':
            return 'Waiting for backend confirmation'
        if not fresh or not self.confirmed:
            return 'Unable to verify running values'
        if self.conflicts:
            return 'External changes conflict — Revert edits to refresh'
        if self.restart:
            return 'Restart required' + (' — unsaved parameter changes' if any(
                k not in self.restart or not equivalent(v, self.restart[k]) for k, v in changed.items()) else '')
        return 'Unsaved parameter changes' if changed else 'No pending edits'


class Notifications:
    def __init__(self):
        self.history = deque(maxlen=40)
        self.current = ''
        self.expires = math.inf
        self.error = False
        self.operation = ''

    def post(self, text, now, error=False):
        operation = text.split(':',1)[0].split(' ',1)[0].lower()
        if operation in ('apply', 'applied', 'save', 'settings', 'edits', 'defaults'):
            operation = 'settings'
        self.history.append(text)
        pending_notice = not error and any(word in text.lower() for word in ('request', 'waiting', 'pending'))
        if self.error and now < self.expires and (operation != self.operation or pending_notice):
            return
        self.error, self.operation = error, operation
        self.current = text
        self.expires = math.inf if error else now + 5.

    def text(self, now):
        return self.current if now < self.expires else ''


def boolean(data, key):
    value = data.get(key)
    return value if type(value) is bool else None


def vector(data, key):
    value = data.get(key) if isinstance(data, dict) else None
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        return None
    return value if all(type(x) in (float, int) and math.isfinite(x) for x in value) else None


def vector_text(value):
    if value is None:
        return 'Unavailable'
    return ', '.join(f'{x:.3g}' for x in value[:3]) + ' m/s; ' + ', '.join(f'{x:.3g}' for x in value[3:]) + ' rad/s'


def station_text(data, station):
    counts = data.get('optical_stations')
    count = counts.get(str(station)) if isinstance(counts, dict) else None
    if type(count) is not int or count < 0:
        return 'Unavailable'
    return f'{count} paired sensors — ' + ('signal' if count else 'no recent signal')


def motion_text(data):
    if boolean(data, 'motion_metrics_current') is not True or boolean(data, 'a_held') is not True:
        return 'Clutch displacement: not current (A released or tracking unavailable)'
    metrics = data.get('motion_metrics')
    if not isinstance(metrics, dict):
        return 'Clutch displacement: Unavailable'
    def number(key, unit):
        value = metrics.get(key)
        return f'{value:.3g} {unit}' if type(value) in (float, int) and math.isfinite(value) else 'Unavailable'
    hand = f"Hand displacement: {number('hand_m', 'm')}; {number('hand_deg', 'deg')}"
    if data.get('teleop_control_mode') == 'position':
        return hand + f"\nRequested displacement: {number('target_m', 'm')}; {number('target_deg', 'deg')} (current)"
    return hand + '\nRequested velocity: ' + vector_text(vector(data.get('velocity_diagnostics'), 'requested_twist'))
