import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, String, Int8
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException
from .core import Clutch, require_mock_urdf


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        if os.environ.get('ROS_DOMAIN_ID') != '42' or self.get_namespace() != '/vr_sim':
            raise RuntimeError('Use the supplied simulation.launch.py in ROS_DOMAIN_ID=42')
        xml = self.declare_parameter('robot_description', '').value
        require_mock_urdf(xml)
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('axis_map', [1., 0., 0., 0., 1., 0., 0., 0., 1.])
        self.declare_parameter('clutch_button', 4)
        self.declare_parameter('translation_scale', 0.8)
        if self.get_parameter('use_sim_time').value:
            raise RuntimeError('Mock hardware uses wall time; use_sim_time must be false')
        self.clutch_index = self.get_parameter('clutch_button').value
        if self.clutch_index < 0 or self.clutch_index >= 16:
            raise ValueError('clutch_button must be between 0 and 15')
        self.core = Clutch(self.get_parameter('axis_map').value,
                           scale=self.get_parameter('translation_scale').value)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.hand = np.zeros(3)
        self.hand_q = np.array([0., 0., 0., 1.])
        self.pressed = self.fresh = False
        self.pose_at = self.joy_at = self.flag_at = self.joint_at = -float('inf')
        self.status_code = None
        self.pose_stamp = self.joy_stamp = self.joint_stamp = None
        self.pose_frame_ok = False
        self.joy_shape_ok = False
        self.tf_buffer = Buffer(cache_time=Duration(seconds=2))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, '/vr/controller/pose', self.on_pose, 5)
        self.create_subscription(Joy, '/vr/controller/joy', self.on_joy, 5)
        self.create_subscription(Bool, '/vr/controller/pose_fresh', self.on_fresh, 5)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, '/vr_sim/joint_states', self.on_joints, sensor_qos)
        self.create_subscription(Int8, '/vr_sim/servo_node/status', self.on_servo_status, 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/vr_sim/servo_node/delta_twist_cmds', 5)
        self.marker_pub = self.create_publisher(Marker, '/vr_sim/target_marker', 5)
        self.state_pub = self.create_publisher(String, '/vr_sim/teleop_state', 5)
        self.last_message = ''
        self.previous_tick = time.monotonic()
        self.create_timer(.02, self.tick)
        self.get_logger().info(f'Simulation teleop v0.2.0: PREVIEW mode. A=6-DOF clutch, translation scale={self.core.scale}, rotation=1:1, max=0.04 m/s and 15 deg/s.')

    def on_parameters(self, params):
        for p in params:
            if p.name != 'enable_motion' or p.type_ != Parameter.Type.BOOL:
                return SetParametersResult(successful=False, reason='Only boolean enable_motion is mutable; restart for axis/button/scale changes')
        self.core.block('mode changed; release A then press again')
        return SetParametersResult(successful=True)

    def on_pose(self, msg):
        self.pose_at = time.monotonic()
        self.pose_stamp = msg.header.stamp
        self.pose_frame_ok = msg.header.frame_id == 'lighthouse'
        self.hand = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = msg.pose.orientation
        self.hand_q = np.array([q.x, q.y, q.z, q.w])

    def on_joy(self, msg):
        self.joy_at = time.monotonic()
        self.joy_stamp = msg.header.stamp
        self.joy_shape_ok = len(msg.buttons) > self.clutch_index
        self.pressed = self.joy_shape_ok and msg.buttons[self.clutch_index] == 1

    def on_fresh(self, msg):
        self.flag_at, self.fresh = time.monotonic(), bool(msg.data)

    def on_joints(self, msg):
        required = {'sim_' + x for x in ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')}
        if required.issubset(set(msg.name)) and len(msg.position) == len(msg.name) and np.all(np.isfinite(msg.position)):
            self.joint_at, self.joint_stamp = time.monotonic(), msg.header.stamp

    def on_servo_status(self, msg):
        if self.status_code != msg.data:
            self.get_logger().info('MoveIt Servo status code: %d' % msg.data)
            if msg.data in (-1, 2, 4, 5):
                self.core.block('Servo reported a halt; release A and inspect Servo status')
        self.status_code = msg.data

    def stamp_fresh(self, stamp):
        if stamp is None:
            return False
        age = (self.get_clock().now().nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)) / 1e9
        return -0.05 <= age < .20

    def tick(self):
        t = time.monotonic()
        healthy = (t - self.previous_tick < .20 and self.fresh and self.pose_frame_ok and self.joy_shape_ok and
                   all(0 <= t-x < .20 for x in (self.pose_at, self.joy_at, self.flag_at, self.joint_at)) and
                   all(self.stamp_fresh(x) for x in (self.pose_stamp, self.joy_stamp, self.joint_stamp)))
        self.previous_tick = t
        tcp = np.zeros(3)
        tcp_q = [0., 0., 0., 1.]
        try:
            trans = self.tf_buffer.lookup_transform('sim_base_link', 'sim_tool0', Time())
            healthy = healthy and self.stamp_fresh(trans.header.stamp)
            p = trans.transform.translation
            tcp = np.array([p.x, p.y, p.z])
            q = trans.transform.rotation
            tcp_q = [q.x, q.y, q.z, q.w]
        except TransformException:
            healthy = False
        velocity = self.core.step(self.hand, tcp, self.pressed, healthy, self.hand_q, tcp_q)
        enabled = self.get_parameter('enable_motion').value
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'sim_base_link'
        if enabled and healthy and self.core.state == 'ACTIVE':
            twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = map(float, velocity[:3])
            twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = map(float, velocity[3:])
        self.twist_pub.publish(twist)
        state = ('MOTION' if enabled else 'PREVIEW') + ' / ' + self.core.state + ' / ' + self.core.reason
        if state != self.last_message:
            self.get_logger().info(state)
            self.last_message = state
        self.state_pub.publish(String(data=state))
        if self.core.target is not None:
            marker = Marker()
            marker.header = twist.header
            marker.ns, marker.id = 'clutched_target', 0
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, self.core.target)
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .025
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .55, 0., 1.
            marker.lifetime = Duration(seconds=.3).to_msg()
            self.marker_pub.publish(marker)
            # Colored target axes make orientation visible beside the sphere.
            axes = self.core.target_rotation.as_matrix()
            for i in range(3):
                axis = Marker()
                axis.header = twist.header
                axis.ns, axis.id = 'target_axes', i
                axis.type, axis.action = Marker.LINE_LIST, Marker.ADD
                axis.pose.orientation.w = 1.
                axis.scale.x = .004
                color = [0., 0., 0.]
                color[i] = 1.
                axis.color.r, axis.color.g, axis.color.b = color
                axis.color.a = 1.
                for p in (self.core.target, self.core.target + .10 * axes[:, i]):
                    axis.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
                axis.lifetime = Duration(seconds=.3).to_msg()
                self.marker_pub.publish(axis)

    def stop_command(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'sim_base_link'
        self.twist_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Teleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if rclpy.ok():
                node.stop_command()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
