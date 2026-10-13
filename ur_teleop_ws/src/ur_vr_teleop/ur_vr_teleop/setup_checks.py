"""Observational setup checks. Consumes telemetry, never publishes robot commands."""
from copy import deepcopy
import math
import numpy as np
from scipy.spatial.transform import Rotation


def finite_vector(value, size):
    return (isinstance(value, (list, tuple)) and len(value) == size
            and all(type(x) in (int, float) and math.isfinite(x) for x in value))


class SetupChecks:
    def __init__(self):
        self.results = {}
        self.parameters = None
        self.session = None
        self.last_at = None
        self.before = None
        self.test = None
        self.phase = ''
        self.since = None
        self.start_count = None
        self.transition = None

    def update(self, d, now):
        params = d.get('effective_parameters')
        if not isinstance(params, dict):
            self.cancel()
            return
        if self.parameters is not None and params != self.parameters:
            self.results.clear()
            self.cancel()
        self.parameters = deepcopy(params)
        if self.session != d.get('session_id') or self.last_at is not None and now-self.last_at > .5:
            self.cancel()
        self.session, self.last_at = d.get('session_id'), now
        if self.test is None:
            return
        if self.phase == 'capture':
            if not self.preview(d) or d.get('tracking_valid') is not True or d.get('observations_healthy') is not True:
                self.cancel()
            return
        if self.test == 'preview_loss':
            if not self.preview(d):
                self.cancel()
                return
            if self.phase == 'loss' and d.get('tracking_valid') is False and d.get('optical') is False:
                self.phase = 'restore'
            elif self.phase == 'restore' and d.get('tracking_valid') is True:
                self.complete({'loss_and_recovery': True})
            return
        if not str(d.get('mode', '')).startswith('LIVE') or d.get('engineering_required') is not False:
            self.cancel()
            return
        v = d.get('velocity_diagnostics', {})
        command, measured = v.get('published_twist'), v.get('measured_tcp_velocity')
        active = (d.get('clutch') == 'ACTIVE' and d.get('a_held') is True
                  and d.get('armed') is True and d.get('motion_ready') is True
                  and d.get('tracking_valid') is True and d.get('observations_healthy') is True
                  and finite_vector(command, 6) and np.linalg.norm(command) > 1e-5
                  and finite_vector(measured, 6) and np.linalg.norm(measured) > 1e-5)
        count = d.get('accepted_targets')
        if self.phase == 'move':
            if active and type(count) is int:
                if self.start_count is None:
                    self.start_count = count
                elif count > self.start_count:
                    self.phase = 'trigger'
                    self.before = deepcopy(d)
            else:
                self.start_count = None
        if self.phase == 'trigger':
            triggered = ((self.test == 'a_stop' and d.get('a_held') is False)
                or (self.test == 'tracking_stop' and d.get('a_held') is True and d.get('tracking_valid') is False)
                or (self.test == 'software_stop' and d.get('enable_requested') is False))
            if triggered:
                self.phase = 'stop'
                self.transition = deepcopy(d)
        if self.phase == 'stop':
            stopped = (d.get('setup_feedback_fresh') is True and d.get('stationary') is True and d.get('clutch') != 'ACTIVE'
                       and finite_vector(command, 6) and np.linalg.norm(command) < 1e-9
                       and finite_vector(measured, 6) and np.linalg.norm(measured[:3]) < .005
                       and np.linalg.norm(measured[3:]) < .02)
            if stopped:
                if self.since is None:
                    self.since = now
                if now-self.since >= .5:
                    name = self.test
                    self.complete({'before': self.before, 'trigger': self.transition,
                                   'after': deepcopy(d), 'standstill_observed_s': now-self.since})
                    if name == 'tracking_stop':
                        self.test, self.phase = 'fresh_clutch', 'held_recovery'
            else:
                self.since = None
        if self.test == 'fresh_clutch':
            if self.phase == 'held_recovery' and d.get('tracking_valid') is True:
                if d.get('a_held') is not True or d.get('clutch') == 'ACTIVE':
                    self.cancel()
                else:
                    self.phase = 'release'
            elif self.phase == 'release' and d.get('a_held') is False:
                self.phase = 'press'
            elif self.phase == 'press' and active:
                self.complete({'recovered_held_without_motion_then_released_and_pressed': True,
                               'after': deepcopy(d)})

    @staticmethod
    def preview(d):
        return ('DRY RUN' in str(d.get('mode', '')) and d.get('armed') is False
                and d.get('enable_requested') is False and d.get('a_held') is False
                and d.get('controller_state_fresh') is True and d.get('setup_controllers_inactive') is True)

    def cancel(self):
        self.test = self.before = self.since = self.start_count = self.transition = None
        self.phase = ''

    def begin(self, name, d):
        self.cancel()
        self.results.pop(name, None)
        if name == 'preview_loss':
            if not self.preview(d) or d.get('tracking_valid') is not True:
                raise ValueError('Use Preview with controllers inactive, valid tracking and A released.')
            self.test, self.phase = name, 'loss'
        elif name in ('a_stop', 'tracking_stop', 'software_stop'):
            if not str(d.get('mode', '')).startswith('LIVE') or d.get('engineering_required') is not False:
                raise ValueError('Begin an authorized supervised Live session first.')
            self.results.pop(name, None)
            if name == 'tracking_stop':
                self.results.pop('fresh_clutch', None)
            self.test, self.phase = name, 'move'
        else:
            if not self.preview(d) or d.get('tracking_valid') is not True or d.get('observations_healthy') is not True:
                raise ValueError('Mapping needs fresh tracking and measured tool TF in inactive Preview.')
            raw = d.get('velocity_diagnostics', {}).get('raw_pose')
            if not finite_vector(raw, 7):
                raise ValueError('Controller pose unavailable.')
            self.test, self.before = name, deepcopy(d)
            self.phase = 'capture'

    def capture(self, d):
        if self.phase != 'capture' or not self.preview(d) or d.get('tracking_valid') is not True or d.get('observations_healthy') is not True:
            raise ValueError('Capture needs fresh inactive Preview. Start this check again.')
        name, before = self.test, self.before
        raw = d.get('velocity_diagnostics', {}).get('raw_pose')
        old = before['velocity_diagnostics']['raw_pose']
        if not finite_vector(raw, 7):
            raise ValueError('Controller pose unavailable.')
        mapping = Rotation.from_euler('xyz', d['effective_parameters']['tracking_to_robot_rpy_deg'], degrees=True)
        if name.startswith('mapping_'):
            delta = mapping.apply(np.array(raw[:3])-old[:3])
            threshold = .02
        else:
            delta = mapping.apply((Rotation.from_quat(raw[3:])*Rotation.from_quat(old[3:]).inv()).as_rotvec())
            threshold = math.radians(10)
        axis = 'xyz'.index(name[-1])
        if delta[axis] < threshold or np.linalg.norm(np.delete(delta, axis)) > abs(delta[axis])*.6:
            raise ValueError('Observed direction does not match +'+name[-1].upper()+'. Review mapping and repeat.')
        self.complete({'mapped_delta': delta.tolist(), 'before': before, 'after': deepcopy(d)})

    def complete(self, details):
        self.results[self.test] = dict(observed=True, physical_pass=False, details=deepcopy(details))
        self.cancel()

    def attest(self, name):
        if name not in self.results:
            raise ValueError('Complete the observed check before accepting its physical result.')
        self.results[name]['physical_pass'] = True
