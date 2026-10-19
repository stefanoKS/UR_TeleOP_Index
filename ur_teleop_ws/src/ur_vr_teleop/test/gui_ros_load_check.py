"""Isolated GUI ROS load regression; no driver, robot, or motion publisher."""
import json
import os
import statistics
import threading
import time

os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
os.environ.setdefault('ROS_DOMAIN_ID', '147')

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur_vr_teleop.dashboard_ros import RosDataBridge, RosExecutorThread
from ur_vr_teleop.startup import Startup
from ur_vr_teleop.validation import JOINTS


class Traffic:
    def __init__(self, node, camera=True):
        self.node = node
        self.status_enabled = True
        self.sequence = 0
        self.disarm_calls = 0
        self.heartbeat_false_at = None
        self.status = node.create_publisher(String, '/vr_real/operator_status', 1)
        self.gripper = node.create_publisher(String, '/gripper/status', 1)
        self.native = node.create_publisher(
            JointTrajectory, '/scaled_joint_trajectory_controller/smoothed_command', 1)
        self.camera = node.create_publisher(
            Image, '/wrist_camera/color/image_raw', qos_profile_sensor_data) if camera else None
        node.create_service(Trigger, '/vr_real/disarm', self.disarm)
        node.create_subscription(Bool, '/vr_real/operator_present', self.on_heartbeat, 1)
        node.create_timer(.01, self.publish_status)
        node.create_timer(.04, self.publish_gripper)
        node.create_timer(.01, self.publish_native)
        if self.camera is not None:
            node.create_timer(1 / 15, self.publish_camera)

    def disarm(self, _request, response):
        self.disarm_calls += 1
        response.success = True
        response.message = 'synthetic disarm complete'
        return response

    def on_heartbeat(self, message):
        if not message.data:
            self.heartbeat_false_at = time.monotonic()

    def publish_status(self):
        if self.status_enabled:
            self.sequence += 1
            self.status.publish(String(data=json.dumps({
                'session_id': 'load-session', 'sequence': self.sequence,
                'armed': False, 'enable_requested': False, 'reason': 'synthetic load',
            })))

    def publish_gripper(self):
        self.gripper.publish(String(data=json.dumps({
            'connection': 'Disconnected', 'status': 'serial unavailable',
            'feedback': {}, 'identity': {'servo_id': 1},
        })))

    def publish_native(self):
        message = JointTrajectory()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [0.] * 6
        point.velocities = [0.] * 6
        message.points = [point]
        self.native.publish(message)

    def publish_camera(self):
        message = Image()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.height, message.width = 480, 640
        message.encoding, message.step = 'rgb8', 640 * 3
        message.data = bytes(message.step * message.height)
        self.camera.publish(message)


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * fraction))]


def summarize(received):
    gaps = [right - left for left, right in zip(received, received[1:])]
    assert gaps
    return {'samples': len(received), 'max_ms': max(gaps) * 1000,
            'p99_ms': percentile(gaps, .99) * 1000,
            'mean_ms': statistics.mean(gaps) * 1000}


def legacy_measure(context, seconds=3.):
    node = rclpy.create_node('legacy_gui_load', context=context)
    received = []
    node.create_subscription(String, '/vr_real/operator_status',
                             lambda _message: received.append(time.monotonic()), 1)
    node.create_subscription(String, '/gripper/status', lambda _message: None, 1)
    node.create_subscription(JointTrajectory,
        '/scaled_joint_trajectory_controller/smoothed_command', lambda _message: None, 1)
    node.create_subscription(Image, '/wrist_camera/color/image_raw',
                             lambda _message: None, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0)
        time.sleep(.1)
    executor.shutdown();executor.remove_node(node);node.destroy_node()
    return summarize(received)


def threaded_measure(context, traffic, seconds=3.):
    node = rclpy.create_node('threaded_gui_load', context=context)
    bridge = RosDataBridge()
    received = []

    def status(message):
        received.append(time.monotonic())
        bridge.put_status(json.loads(message.data))

    node.create_subscription(String, '/vr_real/operator_status', status, 1)
    node.create_subscription(String, '/gripper/status',
                             lambda message: bridge.put_latest('gripper', message.data), 1)
    node.create_subscription(JointTrajectory,
        '/scaled_joint_trajectory_controller/smoothed_command',
        lambda message: bridge.put_latest('native', message), 1)
    node.create_subscription(Image, '/wrist_camera/color/image_raw',
                             lambda message: bridge.put_latest('camera', message), qos_profile_sensor_data)
    heartbeat = node.create_publisher(Bool, '/vr_real/operator_present', 1)
    disarm = node.create_client(Trigger, '/vr_real/disarm')
    executor = RosExecutorThread(node)
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            time.sleep(.1)
            items = bridge.drain()
            assert len([item for item in items if item[1] in ('gripper', 'native', 'camera')]) <= 3
        loaded = summarize(received)
        assert disarm.wait_for_service(timeout_sec=1.)
        stop_started = time.monotonic()
        heartbeat.publish(Bool(data=False))
        future = disarm.call_async(Trigger.Request())
        while not future.done() and time.monotonic() - stop_started < .5:
            time.sleep(.01)
        stop_ms = (time.monotonic() - stop_started) * 1000
        assert future.done() and future.result().success
        assert traffic.disarm_calls == 1 and traffic.heartbeat_false_at is not None
        assert node.count_publishers('/scaled_joint_trajectory_controller/joint_trajectory') == 0
        traffic.status_enabled = False
        silence_started = time.monotonic()
        while time.monotonic() - silence_started < .7:
            time.sleep(.1)
            bridge.drain()
        silence = time.monotonic() - received[-1]
        assert silence >= .5
        startup = Startup()
        startup.start('load-session')
        startup.seen_status = startup.enabled = True
        startup.step({'session_id': 'load-session'}, False, time.monotonic())
        assert startup.state == 'ERROR'
        assert 'Backend status lost' in startup.message
        return loaded, silence * 1000, stop_ms
    finally:
        executor.close();assert executor.join()
        node.destroy_node()


def run():
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=147)
    publisher = rclpy.create_node('gui_load_publishers', context=context)
    traffic = Traffic(publisher, camera=True)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(publisher)
    closing = threading.Event()

    def publish():
        while not closing.is_set() and rclpy.ok(context=context):
            executor.spin_once(timeout_sec=.01)

    thread = threading.Thread(target=publish, name='gui-load-publishers')
    thread.start()
    try:
        time.sleep(.5)
        legacy = legacy_measure(context)
        threaded, silence_ms, stop_ms = threaded_measure(context, traffic)
        assert legacy['max_ms'] >= 500, legacy
        assert threaded['max_ms'] < 250, threaded
        print('Legacy callback gaps:', json.dumps(legacy, sort_keys=True))
        print('Threaded callback gaps:', json.dumps(threaded, sort_keys=True))
        print(f'STOP heartbeat withdrawal and disarm service completed in {stop_ms:.1f} ms')
        print(f'Backend-only silence detected after {silence_ms:.1f} ms while other traffic continued')
    finally:
        closing.set();executor.wake();thread.join(2)
        executor.shutdown();executor.remove_node(publisher);publisher.destroy_node()
        rclpy.shutdown(context=context)


if __name__ == '__main__':
    run()
