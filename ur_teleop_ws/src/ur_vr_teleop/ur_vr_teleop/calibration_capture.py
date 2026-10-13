"""Pure, bounded Preview measurement engine. No ROS, Tk or command interfaces."""
from collections import deque
from dataclasses import dataclass, asdict
import math
import numpy as np
from scipy.spatial.transform import Rotation

STEPS = ('mapping_x', 'mapping_y', 'mapping_z', 'orientation_x', 'orientation_y', 'orientation_z')
VERSION = '5.1'


@dataclass(frozen=True)
class CaptureConfig:
    stable_s: float = .25
    minimum_samples: int = 8
    position_jitter_m: float = .003
    orientation_jitter_rad: float = math.radians(2.)
    minimum_translation_m: float = .020
    minimum_rotation_rad: float = math.radians(10.)
    perpendicular_ratio: float = .6
    pose_timeout_s: float = .15
    joy_timeout_s: float = .20
    flag_timeout_s: float = .20
    isolation_timeout_s: float = .50
    debounce_s: float = .04
    release_guard_s: float = .12
    maximum_capture_s: float = 20.
    maximum_samples: int = 3000
    maximum_jump_m: float = .05
    maximum_jump_rad: float = math.radians(45.)
    freeze_s: float = .5
    recovery_s: float = .1
    advance_s: float = .6
    quaternion_norm_tolerance: float = .2
    future_tolerance_s: float = .03


def estimate(samples):
    positions = np.array([s['position'] for s in samples])
    rotations = Rotation.from_quat([s['quaternion'] for s in samples])
    position = np.median(positions, axis=0)
    rotation = rotations.mean()  # invariant to q versus -q
    return position, rotation, float(np.max(np.linalg.norm(positions-position, axis=1))), float(np.max((rotations*rotation.inv()).magnitude()))


def analyze(start, end, samples, step, rpy, cfg):
    p0, q0, pj0, qj0 = estimate(start)
    p1, q1, pj1, qj1 = estimate(end)
    mapping = Rotation.from_euler('xyz', rpy, degrees=True)
    translation = mapping.apply(p1-p0)
    rotation = mapping.apply((q1*q0.inv()).as_rotvec())
    angular = step.startswith('orientation')
    delta = rotation if angular else translation
    axis = 'xyz'.index(step[-1])
    perpendicular = float(np.linalg.norm(np.delete(delta, axis)))
    magnitude = float(np.linalg.norm(delta))
    required = cfg.minimum_rotation_rad if angular else cfg.minimum_translation_m
    dominant = int(np.argmax(np.abs(delta)))
    reasons = []
    if magnitude < required: reasons.append('Movement too small: at least '+('10°' if angular else '20 mm')+' required')
    if delta[axis] <= 0: reasons.append('Wrong direction sign: move along robot +'+step[-1].upper())
    if dominant != axis: reasons.append('Wrong dominant axis: observed '+ 'XYZ'[dominant])
    if delta[axis] < required and magnitude >= required: reasons.append('Requested-axis movement is below the minimum')
    if perpendicular > abs(delta[axis])*cfg.perpendicular_ratio: reasons.append('Too much perpendicular movement')
    if max(pj0, pj1) > cfg.position_jitter_m or max(qj0, qj1) > cfg.orientation_jitter_rad:
        reasons.append('Start or endpoint was not stable')
    duration = (samples[-1]['source_ns']-samples[0]['source_ns'])/1e9
    rate = (len(samples)-1)/duration if duration > 0 else 0.
    return dict(observer_version=VERSION, step=step, passed=not reasons, failure_reasons=reasons,
        warnings=['Low sample rate; check tracking throughput'] if rate < 30 else [],
        start_position=p0.tolist(), end_position=p1.tolist(), start_quaternion=q0.as_quat().tolist(),
        end_quaternion=q1.as_quat().tolist(), mapped_translation_m=translation.tolist(),
        mapped_rotation_rad=rotation.tolist(), mapped_vector=delta.tolist(), magnitude=magnitude,
        requested_axis=float(delta[axis]), perpendicular=perpendicular,
        dominant_axis='XYZ'[dominant], direction_sign='+' if delta[dominant] >= 0 else '-',
        position_jitter_m=[pj0, pj1], orientation_jitter_rad=[qj0, qj1], accepted_samples=len(samples),
        duration_s=duration, receipt_duration_s=samples[-1]['receipt']-samples[0]['receipt'],
        source_hz=rate, tracking_dropouts=0, continuity=True,
        source_interval_ns=[samples[0]['source_ns'], samples[-1]['source_ns']], thresholds=asdict(cfg))


class Capture:
    def __init__(self, config=None):
        self.cfg = config or CaptureConfig()
        self.samples = deque(maxlen=self.cfg.maximum_samples)
        self.window = deque(maxlen=self.cfg.maximum_samples)
        self.results = {}
        self.step = 0
        self.state, self.reason = 'WAITING_FOR_PREVIEW', 'Begin managed Preview'
        self.context = None
        self.context_at = -math.inf
        self.isolated = False
        self.mapping = [0., 0., 0.]
        self.flags = {}
        self.last_pose = None
        self.source_watermark = 0
        self.continuous_since = self.changed_at = None
        self.joy_at = -math.inf
        self.raw_button = self.button = None
        self.raw_since = 0.
        self.released = False
        self.manual = False
        self.start = self.end = None
        self.started = self.release_at = None
        self.release_candidate = None
        self.passed_at = 0.
        self.received = self.accepted = self.duplicates = self.rejected = self.dropouts = 0
        self.last_report = None
        self.last_failure = ''
        self.generation = 0
        self.manual = False

    @property
    def capturing(self):
        return self.state in ('CAPTURING_START', 'RECORDING', 'WAITING_FOR_END_STABILITY', 'PROCESSING')

    def cancel(self, reason, state='CANCELLED'):
        self.state, self.reason = state, reason
        if state == 'FAILED_RETRY': self.last_failure = reason
        self.released = False
        self.manual = False
        self.start = self.end = self.started = self.release_at = self.release_candidate = None
        self.samples.clear(); self.window.clear()

    def context_update(self, session, parameters, isolated, now, reason='Preview isolation unavailable'):
        key = (session, parameters)
        if self.context is not None and self.context != key:
            self.results.clear(); self.step = 0; self.generation += 1
            self.cancel('Session or configuration changed; release A')
            self.last_pose = None; self.continuous_since = None
            self.source_watermark = 0
        self.context = key
        self.context_at, self.isolated = now, bool(isolated and session)
        if not self.isolated:
            self.cancel(reason, 'WAITING_FOR_PREVIEW')

    def flag(self, name, value, now):
        self.flags[name] = (bool(value), now)
        if not value:
            self.dropouts += 1
            self.continuous_since = None
            self.cancel('Tracking lost; restore tracking, release A, then press again', 'WAITING_FOR_TRACKING')

    def health(self, now):
        if not self.isolated or now-self.context_at > self.cfg.isolation_timeout_s:
            return 'Preview isolation lost or stale', 'WAITING_FOR_PREVIEW'
        if any(k not in self.flags or not self.flags[k][0] or now-self.flags[k][1] > self.cfg.flag_timeout_s for k in ('pose', 'optical')):
            return 'Waiting for fresh pose and optical validity', 'WAITING_FOR_TRACKING'
        if self.last_pose is None or now-self.last_pose['receipt'] > self.cfg.pose_timeout_s:
            return 'Pose updates missing; release A and retry', 'WAITING_FOR_TRACKING'
        if self.continuous_since is None or (self.last_pose['source_ns']-self.continuous_since)/1e9 < self.cfg.recovery_s:
            return 'Waiting for continuous tracking', 'WAITING_FOR_TRACKING'
        if now-self.joy_at > self.cfg.joy_timeout_s:
            return 'Controller buttons stale; release A and retry', 'WAITING_FOR_TRACKING'
        return None

    def pose(self, stamp, position, quaternion, now, age=0.):
        self.received += 1
        try:
            p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
            if p.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite(np.r_[p,q])):
                raise ValueError('Nonfinite or malformed pose')
            norm = np.linalg.norm(q)
            if not math.isfinite(norm) or abs(norm-1) > self.cfg.quaternion_norm_tolerance:
                raise ValueError('Invalid quaternion norm')
            if not isinstance(stamp, int) or stamp <= 0 or not -self.cfg.future_tolerance_s <= age <= self.cfg.pose_timeout_s:
                raise ValueError('Pose source timestamp stale or invalid')
            if stamp == self.source_watermark:
                self.duplicates += 1
                return
            if stamp < self.source_watermark:
                raise ValueError('Out-of-order pose timestamp')
            self.source_watermark = stamp
            q = q/norm
            rotation = Rotation.from_quat(q)
            old = self.last_pose
            if old is not None:
                if stamp == old['source_ns']:
                    self.duplicates += 1
                    return  # never refresh receipt/continuity on duplicates
                if stamp < old['source_ns']: raise ValueError('Out-of-order pose timestamp')
                distance = np.linalg.norm(p-old['position'])
                angle = (rotation*Rotation.from_quat(old['quaternion']).inv()).magnitude()
                if (stamp-old['source_ns'])/1e9 > self.cfg.pose_timeout_s:
                    raise ValueError('Gap in pose samples; release A and retry')
                if distance > self.cfg.maximum_jump_m or angle > self.cfg.maximum_jump_rad:
                    raise ValueError('Pose jump / outlier; release A and retry')
                if distance > 1e-10 or angle > 1e-10: self.changed_at = stamp
                elif self.changed_at is not None and (stamp-self.changed_at)/1e9 > self.cfg.freeze_s:
                    raise ValueError('Exactly frozen pose; release A and retry')
            else:
                self.changed_at = stamp
            sample = dict(source_ns=stamp, receipt=float(now), position=p.tolist(), quaternion=q.tolist())
            self.last_pose = sample
            self.accepted += 1
            if self.continuous_since is None: self.continuous_since = stamp
            if self.capturing and self.release_at is None:
                if len(self.samples) >= self.cfg.maximum_samples: raise ValueError('Capture buffer limit reached')
                self.samples.append(sample); self.window.append(sample)
                while len(self.window) > 1 and (stamp-self.window[1]['source_ns'])/1e9 >= self.cfg.stable_s:
                    self.window.popleft()
                stable = self.stable_window()
                if self.state == 'CAPTURING_START' and stable:
                    self.start = list(self.window); self.window.clear()
                    self.state, self.reason = 'RECORDING', 'Start stable. Move along the requested robot axis'
                elif self.start is not None:
                    self.end = list(self.window) if stable else None
                    self.state = 'WAITING_FOR_END_STABILITY' if stable else 'RECORDING'
                    self.reason = 'Endpoint stable — release A to evaluate' if stable else 'Move, then hold still at the destination'
        except (ValueError, TypeError) as error:
            self.rejected += 1; self.dropouts += 1
            self.last_pose = None; self.continuous_since = None
            self.cancel(str(error), 'FAILED_RETRY')

    def stable_window(self):
        if len(self.window) < self.cfg.minimum_samples or (self.window[-1]['source_ns']-self.window[0]['source_ns'])/1e9 < self.cfg.stable_s:
            return False
        _, _, pj, qj = estimate(list(self.window))
        return pj <= self.cfg.position_jitter_m and qj <= self.cfg.orientation_jitter_rad

    def joy(self, held, now):
        self.joy_at = now
        if held != self.raw_button:
            self.raw_button, self.raw_since = bool(held), now
            self.release_candidate = (self.start, self.end, list(self.samples)) if not held and self.capturing and self.start and self.end else None
        self.tick(now)

    def mouse_start(self, now):
        if self.state in ('PASSED', 'REVIEW'):
            return  # Select Retry first so completed evidence is explicitly invalidated.
        if self.health(now) or self.button is not False or self.state not in ('READY_FOR_CAPTURE', 'FAILED_RETRY'):
            self.cancel('Mouse fallback needs isolated Preview, tracking and A released', 'FAILED_RETRY')
            return
        self.samples.clear(); self.window.clear()
        self.start = self.end = self.release_at = None
        self.results.pop(STEPS[self.step], None)
        self.last_failure = ''
        self.started = now; self.manual = True; self.released = False
        self.state, self.reason = 'CAPTURING_START', 'Mouse fallback: hold still for the starting reference'

    def mouse_finish(self, now):
        if not self.manual or not self.capturing or self.start is None or self.end is None or self.health(now):
            self.cancel('Mouse finish needs stable endpoints and continuing valid tracking', 'FAILED_RETRY')
            return
        self.release_at = now
        self.state, self.reason = 'PROCESSING', 'Checking endpoint and continuing tracking…'

    def retry(self, step=None):
        if step is not None:
            if not 0 <= step < 6: raise ValueError('Unknown calibration step')
            self.step = step
        self.results.pop(STEPS[self.step], None)
        self.generation += 1
        self.cancel('Release A, then press and hold to retry', 'WAITING_FOR_BUTTON_RELEASE')

    def tick(self, now):
        bad = self.health(now)
        if bad:
            self.cancel(*bad)
            return
        if self.capturing and now-self.started > self.cfg.maximum_capture_s:
            self.cancel('Capture too long; release A and retry', 'FAILED_RETRY')
            return
        if self.raw_button != self.button and now-self.raw_since >= self.cfg.debounce_s:
            previous, self.button = self.button, self.raw_button
            if self.manual and self.button:
                self.cancel('A pressed during mouse fallback; release and retry', 'FAILED_RETRY')
                return
            if not self.button:
                if self.capturing:
                    if self.release_candidate is None:
                        self.cancel('Released too early: hold still for a stable start and endpoint', 'FAILED_RETRY')
                    else:
                        self.start, self.end, data = self.release_candidate
                        self.samples = deque(data, maxlen=self.cfg.maximum_samples)
                        self.release_at = now
                        self.state, self.reason = 'PROCESSING', 'Checking release and continuing tracking…'
                else:
                    self.released = True
            elif previous is False and self.released and self.state not in ('PASSED', 'REVIEW'):
                self.released = False
                self.samples.clear(); self.window.clear()
                self.start = self.end = None
                self.last_failure = ''
                self.started = now; self.release_at = None
                self.state, self.reason = 'CAPTURING_START', 'Hold A and keep still for the starting reference'
            else:
                self.released = False
        # A release is provisional until newer pose + validity messages corroborate it.
        if self.state == 'PROCESSING' and self.release_at is not None:
            if self.raw_button:
                self.cancel('Button changed during release verification; release and retry', 'FAILED_RETRY')
            elif now-self.release_at >= self.cfg.release_guard_s and self.last_pose['receipt'] > self.release_at and all(self.flags[k][1] > self.release_at for k in ('pose', 'optical')):
                report = analyze(self.start, self.end, list(self.samples), STEPS[self.step], self.mapping, self.cfg)
                self.last_report = report
                self.manual = False
                if report['passed']:
                    self.results[STEPS[self.step]] = report
                    self.state, self.reason, self.passed_at = 'PASSED', 'Pass — '+STEPS[self.step], now
                else:
                    self.state, self.reason = 'FAILED_RETRY', '; '.join(report['failure_reasons'])
                self.released = True
                self.samples.clear(); self.window.clear(); self.started = self.release_at = None
        if self.state == 'PASSED' and now-self.passed_at >= self.cfg.advance_s:
            missing = [i for i, name in enumerate(STEPS) if name not in self.results]
            if not missing:
                self.state, self.reason = 'REVIEW', 'Six measurements passed. Review against the physical robot frame'
            else:
                self.step = missing[0]
                self.state = 'READY_FOR_CAPTURE' if self.button is False else 'WAITING_FOR_BUTTON_RELEASE'
                self.released = self.button is False
        if not self.capturing and self.button is False and self.raw_button is False:
            self.released = True
        if not self.capturing and self.state not in ('PASSED', 'REVIEW', 'FAILED_RETRY'):
            if self.button is False:
                self.released = True
                self.state, self.reason = 'READY_FOR_CAPTURE', 'Hold still; press and hold A'
            else:
                self.state, self.reason = 'WAITING_FOR_BUTTON_RELEASE', 'Release A before capture'

    def snapshot(self):
        progress = None
        if self.start and self.last_pose:
            p0,q0,_,_ = estimate(self.start)
            p1 = np.array(self.last_pose['position']); q1 = Rotation.from_quat(self.last_pose['quaternion'])
            r = Rotation.from_euler('xyz',self.mapping,degrees=True)
            delta = r.apply(p1-p0) if self.step < 3 else r.apply((q1*q0.inv()).as_rotvec())
            progress = dict(vector=delta.tolist(), magnitude=float(np.linalg.norm(delta)), dominant_axis='XYZ'[int(np.argmax(np.abs(delta)))])
        reason = self.reason
        if self.last_failure and self.last_failure != reason and not self.capturing:
            reason += '; last attempt: '+self.last_failure
        return dict(state=self.state, reason=reason, step=self.step, button=self.raw_button,
                    results=dict(self.results), report=self.last_report, progress=progress,
                    samples=len(self.samples), received=self.received, accepted=self.accepted,
                    duplicates=self.duplicates, rejected=self.rejected, dropouts=self.dropouts,
                    generation=self.generation)
