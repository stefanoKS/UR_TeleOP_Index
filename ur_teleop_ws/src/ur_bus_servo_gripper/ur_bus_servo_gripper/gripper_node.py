"""Read-only arm supervision and independent gripper topics/services."""
import json
import math
import os
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, String, Float64, Int32
from std_srvs.srv import Trigger
from .gripper_config import load
from .gripper_calibration import stationary_disarmed
from .gripper_worker import Worker


class MetadataExecutor(SingleThreadedExecutor):
    """Humble's stock executor discards take_message()[1]. Preserve DDS age.

    Restricted to subscriptions explicitly marked by this node; other nodes
    retain standard callback signatures. Covered by the real Humble ROS check.
    """
    def _take_subscription(self, sub):
        if not getattr(sub, '_gripper_metadata', False):
            return super()._take_subscription(sub)
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, message):
        if getattr(sub, '_gripper_metadata', False):
            if message is not None:
                await await_or_execute(sub.callback, *message)
        else:
            await super()._execute_subscription(sub, message)


class GripperNode(Node):
    def __init__(self, worker_factory=Worker):
        super().__init__('bus_servo_gripper')
        default = str(Path.home() / 'Documents/Teleop/ur_teleop/config/gripper.json')
        self.declare_parameter('config_file', default)
        self.worker = worker_factory(self.get_parameter('config_file').value)
        self.pending_joy = None
        self.last_joy_stamp = 0
        self.flag_times = {'pose': -float('inf'), 'optical': -float('inf')}
        self.flags = {'pose': False, 'optical': False}
        self.auto_connect = True
        self.trigger_min = self.trigger_max = None
        self.managed_session = os.environ.get('UR_TELEOP_MANAGED_SESSION', '')
        self.last_feedback = self.last_command = None
        self.state_pub = self.create_publisher(String, '/gripper/status', 1)
        self.joint_pub = self.create_publisher(JointState, '/gripper/joint_states', 1)
        self.fraction_pub = self.create_publisher(Float64, '/gripper/target_fraction', 1)
        self.target_pub = self.create_publisher(Int32, '/gripper/target_ticks', 1)
        self.subscribe(Joy, '/vr/controller/joy', self.joy, 1)
        self.subscribe(Bool, '/vr/controller/pose_fresh', self.pose, 1)
        self.subscribe(Bool, '/vr/controller/optical_fresh', self.optical, 1)
        self.subscribe(String, '/vr_real/operator_status', self.backend, 1)
        self.subscribe(Bool, '/vr_real/operator_present', self.heartbeat, 1)
        self.subscribe(Bool, '/gripper/calibration_present', self.calibration, 1)
        for action in ('connect', 'read', 'reload', 'jog_open', 'jog_close', 'open', 'close', 'follow'):
            self.create_service(Trigger, '/gripper/' + action, self.service(action))
        self.create_timer(.04, self.tick)
        self.worker.start()

    def subscribe(self, *args):
        sub = self.create_subscription(*args)
        sub._gripper_metadata = True
        return sub

    def source_fresh(self, info):
        stamp = info['source_timestamp'] if isinstance(info, dict) else info.source_timestamp
        return stamp > 0 and 0 <= self.get_clock().now().nanoseconds - stamp < 250_000_000

    def joy(self, msg, info):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        valid = (self.source_fresh(info) and 0 <= age < 250_000_000 and stamp > self.last_joy_stamp
                 and len(msg.axes) > 1 and math.isfinite(msg.axes[1]))
        self.last_joy_stamp = max(stamp, self.last_joy_stamp)
        if valid:
            self.pending_joy = (float(msg.axes[1]), time.monotonic())
        else:
            self.pending_joy = None
            self.worker.update('joy', None)

    def flag(self, name, msg, info):
        now = time.monotonic()
        value = msg.data and self.source_fresh(info)
        self.flags[name], self.flag_times[name] = value, now
        self.worker.update(name, value)
        if not value:
            self.pending_joy = None
            self.worker.update('joy', None)
        elif self.pending_joy is not None:
            raw, at = self.pending_joy
            # Corroborate a Joy with both subsequent healthy flags, rather than
            # interpreting the bridge's synthetic zero during a loss transition.
            if all(self.flags.values()) and all(t > at for t in self.flag_times.values()) and now - at < .1:
                self.trigger_min = raw if self.trigger_min is None else min(self.trigger_min, raw)
                self.trigger_max = raw if self.trigger_max is None else max(self.trigger_max, raw)
                self.worker.update('joy', raw)
                self.pending_joy = None

    def pose(self, msg, info):
        self.flag('pose', msg, info)

    def optical(self, msg, info):
        self.flag('optical', msg, info)

    def backend(self, msg, info):
        try:
            status = json.loads(msg.data) if self.source_fresh(info) else {}
            if (not isinstance(status, dict) or not status.get('session_id') or
                    (self.managed_session and status.get('managed_session_id') != self.managed_session)):
                status = {}
        except (ValueError, TypeError):
            status = {}
        self.worker.update('backend', status)
        if self.auto_connect and stationary_disarmed(status):
            self.auto_connect = False
            try:
                if load(self.worker.path).enabled:
                    self.worker.request('connect')
            except (OSError, ValueError, TypeError) as exc:
                self.get_logger().error(str(exc))
                self.worker.request('connect')

    def heartbeat(self, msg, info):
        self.worker.update('heartbeat', msg.data and self.source_fresh(info))

    def calibration(self, msg, info):
        self.worker.update('calibration', msg.data and self.source_fresh(info))

    def service(self, action):
        def request(_request, response):
            response.success = self.worker.request(action)
            response.message = 'Queued; inspect /gripper/status for completion' if response.success else 'Busy; request rejected'
            return response
        return request

    def tick(self):
        # Ambiguous graph ownership cannot authorize a gripper.
        for topic, key in (('/vr_real/operator_status', 'backend'),
                           ('/vr/controller/joy', 'joy'), ('/vr_real/operator_present', 'heartbeat'),
                           ('/gripper/calibration_present', 'calibration'), ('/vr/controller/pose_fresh', 'pose'), ('/vr/controller/optical_fresh', 'optical')):
            if self.count_publishers(topic) != 1:
                self.worker.update(key, None)
        status = self.worker.snapshot()
        fb = status.get('feedback')
        if fb and time.monotonic() - fb['received_at'] >= .25:
            status.update(feedback_valid=False, status='Fault', reason='Serial feedback stale')
        stamp = self.get_clock().now().to_msg()
        status['schema_version'] = 1
        status['observed_trigger_range'] = [self.trigger_min, self.trigger_max]
        status['stamp'] = {'sec': stamp.sec, 'nanosec': stamp.nanosec}
        if fb and status.get('feedback_valid') and fb['received_at'] != self.last_feedback:
            self.last_feedback = fb['received_at']
            msg = JointState()
            # Feedback receipt time, not repeated timer publication time.
            ns = status['feedback_wall_ns']
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
            msg.name = ['gripper_motor_joint']
            msg.position = [fb['ticks'] * 2. * math.pi / 4096.]
            # Raw speed units depend on firmware phase; no invented rad/s.
            self.joint_pub.publish(msg)
        command = status.get('command_wall_ns')
        if command is not None and command != self.last_command:
            self.last_command = command
            self.target_pub.publish(Int32(data=status['commanded_ticks']))
        if status.get('input_fresh') and status.get('requested_fraction') is not None:
            self.fraction_pub.publish(Float64(data=status['requested_fraction']))
        self.state_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    executor = MetadataExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
