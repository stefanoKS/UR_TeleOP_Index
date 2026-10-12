"""ROS-independent clutch logic. This is not a physical-robot safety controller."""
import numpy as np
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET


def require_mock_urdf(xml):
    plugins = [p.text.strip() for p in ET.fromstring(xml).findall('./ros2_control/hardware/plugin') if p.text]
    allowed = {'mock_components/GenericSystem', 'fake_components/GenericSystem'}
    if len(plugins) != 1 or plugins[0] not in allowed:
        raise ValueError('This launch requires exactly one GenericSystem mock hardware plugin')


def rotation_matrix(values):
    r = np.asarray(values, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(r)) or not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1, atol=1e-6):
        raise ValueError('axis_map must be a proper 3x3 rotation, not a reflection or scale')
    return r


def orientation(quaternion):
    """Validate ROS xyzw quaternion; normalize modest numerical norm error."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or not .5 <= np.linalg.norm(q) <= 1.5:
        raise ValueError('invalid orientation quaternion')
    return Rotation.from_quat(q)


def bounded_velocity(error, limit, deadband):
    if np.linalg.norm(error) < deadband:
        return np.zeros(3)
    velocity = 2.0 * error
    return velocity * min(1.0, limit / np.linalg.norm(velocity))


class Clutch:
    def __init__(self, axis_map, scale=0.8, max_speed=0.04, radius=0.20, jump=0.05):
        self.rotation = rotation_matrix(axis_map)
        self.axis_rotation = Rotation.from_matrix(self.rotation)
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError('translation_scale must be finite and positive')
        self.scale, self.max_speed, self.radius, self.jump = scale, max_speed, radius, jump
        self.max_angular_speed = np.deg2rad(15.)
        self.angular_jump = np.deg2rad(45.)
        self.target_rotation = self.hand_rotation_anchor = self.robot_rotation_anchor = self.last_rotation = None
        self.home = self.target = self.hand_anchor = self.robot_anchor = self.last_hand = None
        self.state = 'WAIT_RELEASE'
        self.reason = 'release A before starting'

    def block(self, reason):
        self.state, self.reason = 'WAIT_RELEASE', reason
        self.last_hand = None
        self.last_rotation = None

    def step(self, hand, tcp, pressed, healthy, hand_q=(0, 0, 0, 1), tcp_q=(0, 0, 0, 1)):
        """Return [vx, vy, vz, wx, wy, wz] expressed in simulated base axes."""
        zero = np.zeros(6)
        if not healthy:
            self.block('waiting for fresh tracking, joint state and TF; release A')
            return zero
        hand, tcp = np.asarray(hand, dtype=float), np.asarray(tcp, dtype=float)
        if hand.shape != (3,) or tcp.shape != (3,) or not np.all(np.isfinite(hand)) or not np.all(np.isfinite(tcp)):
            self.block('invalid coordinates; release A')
            return zero
        try:
            hand_rotation, tcp_rotation = orientation(hand_q), orientation(tcp_q)
        except ValueError:
            self.block('invalid orientation; release A')
            return zero
        if self.home is None:
            self.home, self.target = tcp.copy(), tcp.copy()
            self.target_rotation = tcp_rotation
        if self.state == 'WAIT_RELEASE':
            if not pressed:
                self.state, self.reason = 'READY', 'hold A to move'
            return zero
        if not pressed:
            self.state, self.reason = 'READY', 'hold A to move'
            self.last_hand = None
            self.last_rotation = None
            return zero
        if self.state == 'READY':
            self.hand_anchor, self.robot_anchor = hand.copy(), tcp.copy()
            self.target, self.last_hand = tcp.copy(), hand.copy()
            self.hand_rotation_anchor = self.last_rotation = hand_rotation
            self.target_rotation = self.robot_rotation_anchor = tcp_rotation
            self.state, self.reason = 'ACTIVE', 'A held'
            return zero
        if np.linalg.norm(hand - self.last_hand) > self.jump:
            self.block('hand pose jumped more than 5 cm between samples; release A')
            return zero
        if (hand_rotation * self.last_rotation.inv()).magnitude() > self.angular_jump:
            self.block('hand orientation jumped more than 45 degrees between samples; release A')
            return zero
        self.last_hand = hand.copy()
        self.last_rotation = hand_rotation
        target = self.robot_anchor + self.scale * (self.rotation @ (hand - self.hand_anchor))
        if np.linalg.norm(target - self.home) > self.radius:
            self.block('target outside 20 cm simulation workspace; release A')
            return zero
        self.target = target
        # World-relative hand delta, conjugated into base axes, then applied
        # to the captured tool orientation. Never copy the absolute hand pose.
        delta = hand_rotation * self.hand_rotation_anchor.inv()
        self.target_rotation = self.axis_rotation * delta * self.axis_rotation.inv() * self.robot_rotation_anchor
        angular_error = (self.target_rotation * tcp_rotation.inv()).as_rotvec()
        return np.concatenate((bounded_velocity(target - tcp, self.max_speed, .001),
                               bounded_velocity(angular_error, self.max_angular_speed, np.deg2rad(.3))))
