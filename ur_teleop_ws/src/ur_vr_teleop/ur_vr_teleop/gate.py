"""ROS-independent commissioning gate. Not a safety-rated controller."""
import math
import numpy as np
from .validation import JOINTS


class Gate:
    def __init__(self):
        self.max_joint_speed = .10
        self.native_smoothing = False
        self.armed = False
        self.reason = 'Disarmed at startup'
        self.last_command = None
        self.last_command_at = None
        self.accept_after_ns = 0
        self.last_source_ns = 0

    def disarm(self, reason):
        self.armed = False
        self.reason = reason
        self.last_command = self.last_command_at = None
        self.last_source_ns = 0

    def arm(self, reasons, pressed, stationary):
        if reasons or pressed or not stationary:
            self.disarm('; '.join(reasons) or ('Release A' if pressed else 'Robot must be stationary'))
            return False
        self.armed = True
        self.reason = 'Armed; release then hold A'
        return True

    def validate(self, names, positions, velocities, accelerations, effort,
                 horizon, source_ns, now_ns, measured, now):
        """Accept only one short position target from the staged Servo output.

        Speed follows the operator setting; timing checks remain fixed. No wrapped-angle
        normalization: a 2*pi discontinuity must be rejected, not hidden.
        """
        if not self.armed:
            raise ValueError('Gate is disarmed')
        if source_ns <= self.accept_after_ns or not 0 <= (now_ns - source_ns)/1e9 < .10:
            raise ValueError('Old, future, zero or pre-clutch Servo timestamp')
        if source_ns <= self.last_source_ns:
            raise ValueError('Duplicate or out-of-order Servo timestamp')
        if len(names) != 6 or set(names) != set(JOINTS) or len(positions) != 6:
            raise ValueError('Expected exactly six unique named joints and positions')
        if not math.isfinite(horizon) or not .005 <= horizon <= .10:
            raise ValueError('Servo trajectory horizon must be 5–100 ms')
        if effort:
            raise ValueError('Effort commands are not allowed')
        # Native smoothing enforces the applied actuator speed. Servo's desired
        # velocity may exceed that cap before the ramp; retain a gross input
        # sanity bound rather than tripping on normal requested speed changes.
        for values, bound in ((velocities, 10. if self.native_smoothing else self.max_joint_speed), (accelerations, .5)):
            if values and (len(values) != 6 or not all(math.isfinite(x) and abs(x) <= bound for x in values)):
                raise ValueError('Invalid/excessive Servo derivatives')
        q = np.array([positions[names.index(j)] for j in JOINTS], dtype=float)
        measured = np.asarray(measured, dtype=float)
        if measured.shape != (6,) or not np.all(np.isfinite(measured)) or not np.all(np.isfinite(q)):
            raise ValueError('Nonfinite or incomplete joints')
        # Bound speed from current feedback over the actual outgoing horizon,
        # independently of the target-to-target stream-rate check below.
        # With the verified native controller, these are requested positions,
        # not actuator steps. Its 500 Hz stage bounds final v/a/jerk. A one-period
        # lead test otherwise trips on the lag deliberately created by that ramp.
        # Keep an independent 0.05 rad input envelope and reject large jumps.
        lead_limit = .05 if self.native_smoothing else self.max_joint_speed * horizon
        if np.max(np.abs(q - measured)) > lead_limit:
            raise ValueError(f'Target exceeds {lead_limit:g} rad measured-position lead at configured joint speed')
        # Constrain the first target too; subsequent samples use elapsed host time.
        reference = measured if self.last_command is None else self.last_command
        dt = horizon if self.last_command_at is None else now - self.last_command_at
        step_limit = .05 if self.native_smoothing else self.max_joint_speed * dt + 1e-6
        if not 0 < dt <= .10 or np.max(np.abs(q-reference)) > step_limit:
            raise ValueError(f'Target rate exceeds configured {self.max_joint_speed:g} rad/s or stream stalled')
        self.last_command, self.last_command_at = q.copy(), now
        self.last_source_ns = source_ns
        return q
