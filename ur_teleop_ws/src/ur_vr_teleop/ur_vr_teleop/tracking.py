"""Real teleop target tracking, reusing the original simulation clutch guards."""
import numpy as np
from ur_vr_sim_teleop.core import Clutch, orientation

CONTROL_PERIOD_S = .01
DEFAULT_RESPONSE_GAIN = 2.


def smooth_velocity(error, previous, limit, acceleration, deadband, gain, dt,
                    measured_velocity=None, damping=0.):
    """Vector-norm slew bound, including ordinary deceleration/reversals.

    Do not apply a second distance/deadband clamp after the slew limiter: that
    would itself introduce unbounded deceleration. Fault/release resets bypass it.
    """
    desired = tracking_velocity(error, limit, deadband, gain)
    if measured_velocity is not None and damping:
        # Velocity-feedback damping in the outer Cartesian loop. This is a
        # dimensionless velocity gain, not a torque/impedance damping constant.
        # Differentiate measured motion, not VR error (avoids target-step kick).
        desired -= damping * measured_velocity
        desired *= min(1., limit / max(np.linalg.norm(desired), 1e-12))
    change = desired - previous
    change_norm = np.linalg.norm(change)
    return previous + change * min(1., acceleration * dt / max(change_norm, 1e-12))


def tracking_velocity(error, limit, deadband, gain):
    distance = np.linalg.norm(error)
    if distance < deadband:
        return np.zeros(3)
    # Normalize before scaling; large finite GUI ceilings must not cause NaNs.
    # A continuous zero-speed boundary avoids a small feedback-noise-driven
    # velocity jump every time the tool crosses the settling tolerance.
    return (error / distance) * min(gain * max(0., distance-deadband), limit)


class TrackingClutch(Clutch):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.control_mode = "position"  # Standalone API compatibility; commissioning selects velocity.
        self.linear_velocity_filter_s = .04
        self.angular_velocity_filter_s = .04
        self.reset_velocity_history()
        self.response_gain = DEFAULT_RESPONSE_GAIN
        self.linear_acceleration = .25
        self.angular_acceleration = np.deg2rad(90.)
        self.translation_filter_alpha = .8
        self.rotation_filter_alpha = .8
        self.rotation_scale = 1.
        self.translation_deadband = .001
        self.rotation_deadband = np.deg2rad(.3)
        self.translation_damping_gain = 0.
        self.rotation_damping_gain = 0.
        self.damping_velocity_filter_s = .05
        self.reset_feedback()
        self.stable_hand = self.stable_rotation = None
        self.velocity = np.zeros(6)
        self.filtered_hand = self.filtered_rotation = self.filtered_target_rotation = None
        self.filter_sample_id = None

    def block(self, reason):
        super().block(reason)
        self.reset_velocity_history()
        self.velocity = np.zeros(6)
        self.filtered_hand = self.filtered_rotation = self.filtered_target_rotation = None
        self.filter_sample_id = None
        self.stable_hand = self.stable_rotation = None
        self.reset_feedback()

    def reset_feedback(self):
        self.feedback_pose = None
        self.feedback_stamp = None
        self.measured_velocity = np.zeros(6)

    def update_feedback(self, tcp, tcp_q, dt, stamp):
        pose = (np.asarray(tcp, dtype=float).copy(), orientation(tcp_q))
        if self.feedback_pose is not None:
            if stamp is not None and stamp == self.feedback_stamp:
                return True  # Repeated TF is not a new zero-velocity sample.
            sample_dt = ((stamp-self.feedback_stamp)/1e9
                         if stamp is not None and self.feedback_stamp is not None else dt)
            if not np.isfinite(sample_dt) or not 0 < sample_dt <= .35:
                self.block('Invalid tool feedback interval for damping; release A')
                return False
            raw = np.concatenate(((pose[0]-self.feedback_pose[0])/sample_dt,
                (pose[1]*self.feedback_pose[1].inv()).as_rotvec()/sample_dt))
            if not np.all(np.isfinite(raw)):
                self.block('Invalid tool velocity for damping; release A')
                return False
            alpha = -np.expm1(-sample_dt/self.damping_velocity_filter_s)
            self.measured_velocity += alpha * (raw-self.measured_velocity)
        self.feedback_pose, self.feedback_stamp = pose, stamp
        return True

    def reset_command(self):
        """Clear actuator command history whenever the output gate is inactive."""
        self.velocity = np.zeros(6)
        self.reset_velocity_history()
        self.reset_feedback()

    def step(self, hand, tcp, pressed, healthy, hand_q=(0, 0, 0, 1), tcp_q=(0, 0, 0, 1), dt=CONTROL_PERIOD_S, sample_id=None, feedback_sample_id=None):
        if self.control_mode == 'velocity':
            return self.velocity_step(hand, tcp, pressed, healthy, hand_q, tcp_q, dt, sample_id, feedback_sample_id)
        if not np.isfinite(dt) or dt <= 0 or dt > .35:
            self.block('Invalid tracking interval; release A')
            return self.velocity.copy()
        feedback_dt = dt
        dt = min(dt, .05)  # A delayed callback must not create a large speed step.
        was_active = self.state == 'ACTIVE'
        result = super().step(hand, tcp, pressed, healthy, hand_q, tcp_q)
        if not healthy or not pressed or self.state != 'ACTIVE':
            self.velocity = np.zeros(6)
            self.filtered_hand = self.filtered_rotation = self.filtered_target_rotation = None
            self.filter_sample_id = None
            self.stable_hand = self.stable_rotation = None
            self.reset_feedback()
            return result
        if not was_active:
            self.filtered_hand = np.asarray(hand, dtype=float).copy()
            self.filtered_rotation = orientation(hand_q)
            self.filtered_target_rotation = orientation(tcp_q)
            self.stable_hand = self.filtered_hand.copy()
            self.stable_rotation = self.filtered_rotation
            self.filter_sample_id = sample_id
            self.velocity = np.zeros(6)
            self.update_feedback(tcp, tcp_q, feedback_dt, feedback_sample_id)
            return result
        if not self.update_feedback(tcp, tcp_q, feedback_dt, feedback_sample_id):
            return self.velocity.copy()
        # Raw pose validation/jump detection above runs BEFORE filtering. Only
        # filter a new VR sample, not repeated ROS timer reads of the same sample.
        if sample_id is None or sample_id != self.filter_sample_id:
            self.filtered_hand += self.translation_filter_alpha * (np.asarray(hand)-self.filtered_hand)
            increment = (orientation(hand_q) * self.filtered_rotation.inv()).as_rotvec()
            from scipy.spatial.transform import Rotation
            delta = Rotation.from_rotvec(self.rotation_filter_alpha * increment)
            self.filtered_rotation = delta * self.filtered_rotation
            # Continuous spatial deadband ("play"): bounded stationary noise
            # leaves the target fixed. Slow deliberate motion still accumulates
            # past the band; the target does not jump by the band width.
            displacement = self.filtered_hand - self.stable_hand
            distance = np.linalg.norm(displacement)
            self.stable_hand += displacement * max(0., 1.-self.translation_deadband/max(distance, 1e-12))
            rotation = (self.filtered_rotation * self.stable_rotation.inv()).as_rotvec()
            angle = np.linalg.norm(rotation)
            rotation *= max(0., 1.-self.rotation_deadband/max(angle, 1e-12))
            self.stable_rotation = Rotation.from_rotvec(rotation) * self.stable_rotation
            mapped_delta = self.axis_rotation * Rotation.from_rotvec(self.rotation_scale * rotation) * self.axis_rotation.inv()
            self.filtered_target_rotation = mapped_delta * self.filtered_target_rotation
            self.filter_sample_id = sample_id
        self.target = self.robot_anchor + self.scale * (self.rotation @ (self.stable_hand-self.hand_anchor))
        self.target_rotation = self.filtered_target_rotation
        angular_error = (self.target_rotation * orientation(tcp_q).inv()).as_rotvec()
        self.velocity = np.concatenate((
            smooth_velocity(self.target-np.asarray(tcp), self.velocity[:3], self.max_speed,
                            self.linear_acceleration, .001, self.response_gain, dt,
                            self.measured_velocity[:3], self.translation_damping_gain),
            smooth_velocity(angular_error, self.velocity[3:], self.max_angular_speed,
                            self.angular_acceleration, np.deg2rad(.3), self.response_gain, dt,
                            self.measured_velocity[3:], self.rotation_damping_gain)))
        return self.velocity.copy()

    def reset_velocity_history(self):
        self.velocity_sample = None
        self.sample_age = 0.
        self.hand_velocity = np.zeros(6)
        self.raw_hand_velocity = np.zeros(6)
        self.sample_dt = None
        self.clamped = False

    def velocity_step(self, hand, tcp, pressed, healthy, hand_q, tcp_q, dt, sample_id, feedback_sample_id):
        from scipy.spatial.transform import Rotation
        if not np.isfinite(dt) or not 0 < dt <= .35:
            self.block('Invalid control interval; release A')
            return self.velocity.copy()
        was_active = self.state == 'ACTIVE'
        # Reuse raw coordinate, quaternion, jump and deadman guards. Velocity
        # workspace is checked against actual TCP, never historical hand offset.
        radius = self.radius
        self.radius = float('inf')
        try:
            Clutch.step(self, hand, tcp, pressed, healthy, hand_q, tcp_q)
        finally:
            self.radius = radius
        if not healthy or not pressed or self.state != 'ACTIVE':
            self.reset_command()
            return self.velocity.copy()
        self.target = np.asarray(tcp).copy()  # Display only; no position servo.
        self.target_rotation = orientation(tcp_q)
        if np.linalg.norm(self.target-self.home) > radius:
            self.block('Measured tool outside workspace; release A')
            return self.velocity.copy()
        if sample_id is None or not np.isfinite(sample_id) or sample_id <= 0:
            self.block('Velocity mode needs a valid sample timestamp; release A')
            return self.velocity.copy()
        if not self.update_feedback(tcp, tcp_q, dt, feedback_sample_id):
            return self.velocity.copy()
        if not was_active or self.velocity_sample is None:
            self.reset_velocity_history()
            self.velocity_sample = sample_id
            self.filtered_hand = self.rotation @ np.asarray(hand)
            self.filtered_rotation = self.axis_rotation * orientation(hand_q)
            self.stable_hand = self.filtered_hand.copy()
            self.stable_rotation = self.filtered_rotation
            self.velocity = np.zeros(6)
            return self.velocity.copy()
        self.sample_age += dt
        if sample_id < self.velocity_sample:
            self.block('Out-of-order pose timestamp; release A')
            return self.velocity.copy()
        if sample_id != self.velocity_sample:
            sample_dt = (sample_id-self.velocity_sample)/1e9
            if not .001 <= sample_dt <= .10:
                self.block('Pose sample interval outside 1–100 ms; release A')
                return self.velocity.copy()
            self.sample_dt = sample_dt
            self.sample_age = 0.
            self.velocity_sample = sample_id
            self.filtered_hand += self.translation_filter_alpha * (self.rotation @ np.asarray(hand)-self.filtered_hand)
            increment = (self.axis_rotation * orientation(hand_q) * self.filtered_rotation.inv()).as_rotvec()
            self.filtered_rotation = Rotation.from_rotvec(self.rotation_filter_alpha*increment) * self.filtered_rotation
            displacement = self.filtered_hand-self.stable_hand
            displacement *= max(0., 1.-self.translation_deadband/max(np.linalg.norm(displacement), 1e-12))
            rotation = (self.filtered_rotation*self.stable_rotation.inv()).as_rotvec()
            rotation *= max(0., 1.-self.rotation_deadband/max(np.linalg.norm(rotation), 1e-12))
            self.stable_hand += displacement
            self.stable_rotation = Rotation.from_rotvec(rotation)*self.stable_rotation
            self.raw_hand_velocity = np.r_[displacement, rotation]/sample_dt
            for part, tau in ((slice(0, 3), self.linear_velocity_filter_s), (slice(3, 6), self.angular_velocity_filter_s)):
                alpha = 1. if tau == 0 else -np.expm1(-sample_dt/tau)
                self.hand_velocity[part] += alpha*(self.raw_hand_velocity[part]-self.hand_velocity[part])
        if self.sample_age >= .10:
            self.block('Pose stream frozen or missing; release A')
            return self.velocity.copy()
        desired = self.hand_velocity * np.r_[np.full(3, self.scale), np.full(3, self.rotation_scale)]
        # Stop extrapolating promptly during gaps; the independent fault timeout
        # above latches release-required if the source does not recover.
        if self.sample_age >= .03:
            desired = np.zeros(6)
        self.clamped = False
        for part, limit, acceleration in ((slice(0, 3), self.max_speed, self.linear_acceleration),
                                         (slice(3, 6), self.max_angular_speed, self.angular_acceleration)):
            norm = np.linalg.norm(desired[part])
            self.clamped |= bool(norm > limit)
            desired[part] *= min(1., limit/max(norm, 1e-12))
            change = desired[part]-self.velocity[part]
            bound = acceleration*min(dt, .05)
            self.clamped |= bool(np.linalg.norm(change) > bound)
            self.velocity[part] += change*min(1., bound/max(np.linalg.norm(change), 1e-12))
        return self.velocity.copy()

    def displacement_metrics(self, hand, tcp, hand_q, tcp_q):
        """Distances from this clutch's anchors, never sums of noisy increments."""
        if self.state != 'ACTIVE':
            return None
        return {
            'hand_m': float(np.linalg.norm(np.asarray(hand) - self.hand_anchor)),
            'target_m': float(np.linalg.norm(self.target - self.robot_anchor)),
            'mapped_m': float(self.scale * np.linalg.norm(np.asarray(hand)-self.hand_anchor)),
            'tool_m': float(np.linalg.norm(np.asarray(tcp) - self.robot_anchor)),
            'error_m': float(np.linalg.norm(self.target - np.asarray(tcp))),
            'hand_deg': float(np.rad2deg((orientation(hand_q) * self.hand_rotation_anchor.inv()).magnitude())),
            'target_deg': float(np.rad2deg((self.target_rotation * self.robot_rotation_anchor.inv()).magnitude())),
            'tool_deg': float(np.rad2deg((orientation(tcp_q) * self.robot_rotation_anchor.inv()).magnitude())),
        }
