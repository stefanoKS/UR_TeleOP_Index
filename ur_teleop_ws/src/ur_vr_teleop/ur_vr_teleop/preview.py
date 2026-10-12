"""Display a clutched target beside the measured robot; never command motion."""
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener, TransformException

# Reuse the tested ROS-independent math; do not instantiate the simulation node.
from ur_vr_sim_teleop.core import Clutch
from .validation import fresh, valid_joints
from .tracking import TrackingClutch, CONTROL_PERIOD_S


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class Preview(Node):
    def __init__(self, observation_only=True, node_name='real_teleop_preview'):
        if os.environ.get('ROS_DOMAIN_ID') != '43':
            raise RuntimeError('Preview requires ROS_DOMAIN_ID=43')
        super().__init__(node_name, namespace='vr_real')
        self.mode_label = 'PREVIEW ONLY' if observation_only else 'TARGET DISPLAY'
        if self.get_parameter('use_sim_time').value:
            raise RuntimeError('Preview requires live wall-time feedback')
        readonly = ParameterDescriptor(read_only=True)
        self.base = self.declare_parameter('base_frame', 'base_link', readonly).value
        self.tool = self.declare_parameter('tool_frame', 'tool0', readonly).value
        mapping = self.declare_parameter('axis_map', [1., 0., 0., 0., 1., 0., 0., 0., 1.], readonly).value
        scale = self.declare_parameter('translation_scale', .8 if observation_only else 1./1.2,
                                       readonly if observation_only else ParameterDescriptor()).value
        self.observation_timeout = self.declare_parameter(
            'observation_timeout_s', .20 if observation_only else .35, readonly).value
        if not np.isfinite(self.observation_timeout) or not .10 <= self.observation_timeout <= .50:
            raise ValueError('observation_timeout_s must be between 0.10 and 0.50 s')
        self.core = (Clutch if observation_only else TrackingClutch)(mapping, scale=scale)
        from .tracking_quality import PoseQuality
        self.pose_quality = PoseQuality()
        self.observed_hand_velocity = np.zeros(6)
        self.raw_observed_hand_velocity = np.zeros(6)
        self.diagnostic_hand = None
        self.observed_sample_dt = None
        self.records = {}
        self.hand = np.zeros(3)
        self.hand_q = [0., 0., 0., 1.]
        self.pressed = self.pose_ok = self.joy_ok = self.joints_ok = self.flag = False
        self.flag_at = -float('inf')
        self.diagnostic_feedback = None
        self.measured_tcp_velocity = None
        self.last_tick = time.monotonic()
        self.last_status = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, '/vr/controller/pose', self.on_pose, 5)
        self.create_subscription(Joy, '/vr/controller/joy', self.on_joy, 5)
        self.create_subscription(Bool, '/vr/controller/pose_fresh', self.on_flag, 5)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.markers = self.create_publisher(MarkerArray, '/vr_real/preview_markers', 5)
        self.status = self.create_publisher(String, '/vr_real/preview_state', 5)
        self.create_timer(.02 if observation_only else CONTROL_PERIOD_S, self.tick)
        if observation_only:
            self.get_logger().info('PREVIEW ONLY: no motion publisher. Axis mapping is unverified; optical validity is not checked. Tool frame: ' + self.tool)
        else:
            self.get_logger().info('Target display initialized. Follow the command gate mode and status for motion readiness. Tool frame: ' + self.tool)

    def remember(self, key, msg):
        self.records[key] = (time.monotonic(), stamp_ns(msg.header.stamp))

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        position, quaternion = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
        self.pose_ok = msg.header.frame_id == 'lighthouse' and self.pose_quality.accept(
            stamp_ns(msg.header.stamp), position, quaternion)
        if not self.pose_ok:
            self.diagnostic_hand = None
            self.observed_hand_velocity = np.zeros(6)
            self.core.block(self.pose_quality.reason + '; release A')
            if hasattr(self, 'gate'):
                self.trip(self.pose_quality.reason + '; release A')
            return
        from ur_vr_sim_teleop.core import orientation
        current = (stamp_ns(msg.header.stamp), np.array(position), orientation(quaternion))
        previous = self.diagnostic_hand
        if previous is not None:
            interval = (current[0]-previous[0])/1e9
            self.observed_sample_dt = interval
            if .001 <= interval <= .10:
                self.raw_observed_hand_velocity = np.r_[(current[1]-previous[1])/interval,
                    (current[2]*previous[2].inv()).as_rotvec()/interval]
                self.observed_hand_velocity += -np.expm1(-interval/.04)*(self.raw_observed_hand_velocity-self.observed_hand_velocity)
            else:
                self.observed_hand_velocity = np.zeros(6)
        self.diagnostic_hand = current
        self.remember('pose', msg)
        self.hand, self.hand_q = np.array(position), quaternion

    def on_joy(self, msg):
        self.remember('joy', msg)
        self.joy_ok = len(msg.buttons) > 4 and msg.buttons[4] in (0, 1)
        self.pressed = self.joy_ok and msg.buttons[4] == 1

    def on_flag(self, msg):
        self.flag_at, self.flag = time.monotonic(), msg.data

    def on_joints(self, msg):
        self.remember('joints', msg)
        self.joints_ok = valid_joints(msg.name, msg.position)

    def tick(self):
        t, now = time.monotonic(), self.get_clock().now()
        timeout = getattr(self, 'observation_timeout', .20)
        tick_age = t-self.last_tick
        missing = [key for key in ('pose', 'joy', 'joints')
                   if key not in self.records or not fresh(*self.records[key], t, now.nanoseconds, timeout)]
        healthy = (not missing and self.pose_ok and self.joy_ok and self.joints_ok
                   and self.flag and 0 <= t - self.flag_at < timeout
                   and 0 <= tick_age < timeout)
        self.last_tick = t
        tcp, tcp_q = np.zeros(3), [0., 0., 0., 1.]
        tf_ok, tf_age, tf_stamp = False, None, None
        try:
            transform = self.tf_buffer.lookup_transform(self.base, self.tool, Time())
            tf_stamp = stamp_ns(transform.header.stamp)
            tf_age = (now.nanoseconds-tf_stamp)/1e9
            tf_ok = fresh(t, tf_stamp, t, now.nanoseconds, timeout)
            p, q = transform.transform.translation, transform.transform.rotation
            tcp, tcp_q = np.array([p.x, p.y, p.z]), [q.x, q.y, q.z, q.w]
            from ur_vr_sim_teleop.core import orientation
            orientation(tcp_q)
            if not np.all(np.isfinite(tcp)):
                raise ValueError('Nonfinite tool TF')
        except (TransformException, ValueError):
            tf_ok = False
            tcp, tcp_q = np.zeros(3), [0., 0., 0., 1.]
        healthy = healthy and tf_ok
        # Standalone preview displays this target; commissioning also uses its velocity.
        if isinstance(self.core, TrackingClutch):
            self.preview_velocity = self.core.step(self.hand, tcp, self.pressed, healthy,
                                                   self.hand_q, tcp_q, dt=tick_age,
                                                   sample_id=self.records.get('pose', (0, 0))[1],
                                                   feedback_sample_id=tf_stamp)
            metrics = self.core.displacement_metrics(self.hand, tcp, self.hand_q, tcp_q) if healthy else None
            if metrics is not None:
                self.motion_metrics = metrics
            self.motion_metrics_current = metrics is not None
        else:
            self.preview_velocity = self.core.step(self.hand, tcp, self.pressed, healthy, self.hand_q, tcp_q)
        # The shared simulation core has historical fixed-distance error text.
        # Report this instance's configured limit without changing the guard.
        if self.core.reason.startswith('target outside '):
            self.core.reason = f'Requested target outside {self.core.radius*100:g} cm workspace radius; release A'
        self.observation_healthy = healthy and self.core.state != 'WAIT_RELEASE'
        self.observation_reason = self.core.reason
        if not healthy:
            failures = []
            for key in missing:
                record = self.records.get(key)
                if record is None:
                    failures.append(f'{key}: no samples')
                else:
                    receipt, stamp = record
                    source_age = 'invalid' if not stamp else f'{(now.nanoseconds-stamp)/1e6:.0f} ms'
                    failures.append(f'{key}: receipt age {(t-receipt)*1000:.0f} ms, source age {source_age}')
            if not tf_ok:
                failures.append('TF unavailable' if tf_age is None else f'TF age {tf_age*1000:.0f} ms or invalid timestamp')
            if not self.pose_ok: failures.append('Pose rejected: ' + self.pose_quality.reason + ' (expected lighthouse frame)')
            if not self.joy_ok: failures.append('A-button input invalid')
            if not self.joints_ok: failures.append('Joint names/positions invalid')
            if not self.flag: failures.append('Bridge reports pose not fresh')
            if not 0 <= t-self.flag_at < timeout: failures.append(f'Pose-fresh flag age {(t-self.flag_at)*1000:.0f} ms')
            if not 0 <= tick_age < timeout: failures.append(f'Teleop loop gap {tick_age*1000:.0f} ms')
            self.observation_reason = '; '.join(failures) + f' (observation limit {timeout*1000:.0f} ms); release A'
        # Independent telemetry continues through clutch release and software
        # stops; clearing the command estimator must not fake measured standstill.
        if tf_ok:
            from ur_vr_sim_teleop.core import orientation
            current = (tf_stamp, tcp.copy(), orientation(tcp_q))
            previous = getattr(self, 'diagnostic_feedback', None)
            if previous is not None and tf_stamp != previous[0]:
                interval = (tf_stamp-previous[0])/1e9
                self.measured_tcp_velocity = (np.r_[(tcp-previous[1])/interval,
                    (current[2]*previous[2].inv()).as_rotvec()/interval].tolist()
                    if 0 < interval <= .35 else None)
            self.diagnostic_feedback = current
        else:
            self.diagnostic_feedback = self.measured_tcp_velocity = None
        self.measured_tcp = tcp
        text = getattr(self, 'mode_label', 'PREVIEW ONLY') + ' / ' + self.core.state + ' / ' + self.core.reason
        if not healthy:
            text += f' / stale={missing} TF={tf_ok} pose_frame={self.pose_ok} joy={self.joy_ok} joints={self.joints_ok} pose_fresh={self.flag}'
        self.status.publish(String(data=text))
        if text != self.last_status:
            self.get_logger().info(text)
            self.last_status = text
        array = MarkerArray()
        if not healthy or self.core.state == 'WAIT_RELEASE' or self.core.target is None:
            marker = Marker()
            marker.action = Marker.DELETEALL
            array.markers.append(marker)
        else:
            # On release show measured pose; ACTIVE shows the requested relative target.
            target = self.core.target if self.core.state == 'ACTIVE' else tcp
            rotation = self.core.target_rotation.as_matrix() if self.core.state == 'ACTIVE' else None
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.base, now.to_msg()
            marker.ns, marker.id = 'preview', 0
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, target)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .025
            marker.color.r, marker.color.g, marker.color.a = 1., .55, 1.
            marker.lifetime = Duration(seconds=.3).to_msg()
            array.markers.append(marker)
            if rotation is None:
                from ur_vr_sim_teleop.core import orientation
                rotation = orientation(tcp_q).as_matrix()
            for i in range(3):
                axis = Marker()
                axis.header = marker.header
                axis.ns, axis.id = 'preview', i + 1
                axis.type, axis.action = Marker.LINE_LIST, Marker.ADD
                axis.pose.orientation.w = 1.
                axis.scale.x = .004
                axis.color.r, axis.color.g, axis.color.b = [float(i == j) for j in range(3)]
                axis.color.a = 1.
                axis.lifetime = marker.lifetime
                axis.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                               for p in (target, target + .10 * rotation[:, i])]
                array.markers.append(axis)
        self.markers.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Preview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
