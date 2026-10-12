"""Real Servo/RSP processes with synthetic joints in isolated domain 147.
No UR driver, controller manager, hardware plugin or robot command publisher.
"""
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import time
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import Trigger
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
from ur_vr_teleop.validation import JOINTS

os.environ['ROS_LOG_DIR'] = '/tmp/ur_servo_bench_logs'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '43'
spec = importlib.util.spec_from_file_location('commissioning_launch',
    Path(__file__).parents[1] / 'launch/commissioning.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
context = LaunchContext()
context.launch_configurations.update(config_file='/home/ubuntu-ros/Documents/Teleop/ur_teleop/config/real_teleop.yaml',
                                      motion_output_enabled='false')
servo_action = module.nodes(context)[0]
# Inspect the actual normalized launch settings, so this tests the shipped launch.
params = {}
for entry in evaluate_parameters(context, servo_action._Node__parameters): params.update(entry)
os.environ['ROS_DOMAIN_ID'] = '147'
directory = Path(tempfile.mkdtemp(prefix='ur_servo_bench_'))
parameter_file = directory / 'parameters.yaml'
parameter_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
log = (directory / 'processes.log').open('w')
processes = []
node = None
try:
    for binary, name, namespace in [('/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher', 'bench_rsp', '/'),
                                     ('/opt/ros/humble/lib/moveit_servo/servo_node_main', 'servo_node', '/vr_real'),
                                     ('/home/ubuntu-ros/Documents/Teleop/ur_teleop_ws/install/ur_vr_bridge/lib/ur_vr_bridge/servo_stamp_relay', 'servo_stamp_relay', '/vr_real')]:
        processes.append(subprocess.Popen([binary, '--ros-args', '--params-file', str(parameter_file),
                                            '-r', '__node:='+name, '-r', '__ns:='+namespace],
                                           stdout=log, stderr=subprocess.STDOUT))
    rclpy.init(domain_id=147)
    node = Node('isolated_servo_test')
    states = node.create_publisher(JointState, '/joint_states', 10)
    twists = node.create_publisher(TwistStamped, '/vr_real/servo_node/delta_twist_cmds', 1)
    trajectories = []
    def received(msg):
        source = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        trajectories.append((msg, source, node.get_clock().now().nanoseconds))
    node.create_subscription(JointTrajectory, '/vr_real/servo_joint_targets', received, 10)
    start = node.create_client(Trigger, '/vr_real/servo_node/start_servo')
    q = [0., -1.2, 1.6, -1.97, -1.57, 0.]
    def pump(seconds, move=False):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            stamp = node.get_clock().now().to_msg()
            msg = JointState()
            msg.header.stamp, msg.name, msg.position, msg.velocity = stamp, list(JOINTS), q, [0.]*6
            states.publish(msg)
            twist = TwistStamped()
            twist.header.stamp, twist.header.frame_id = stamp, 'base_link'
            twist.twist.linear.x = .001 if move else 0.
            twists.publish(twist)
            rclpy.spin_once(node, timeout_sec=.01)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Child exited; inspect '+str(directory/'processes.log'))
    pump(3.)
    assert start.service_is_ready(), 'Servo start service missing'
    future = start.call_async(Trigger.Request())
    pump(2.)
    assert future.done() and future.result().success, 'Servo did not start'
    trajectories.clear()
    pump(2., move=True)
    assert len(trajectories) >= 10, 'No meaningful Servo output'
    assert node.count_publishers('/scaled_joint_trajectory_controller/joint_trajectory') == 0
    horizons, ages = [], []
    for msg, source_ns, received_ns in trajectories:
        assert set(msg.joint_names) == set(JOINTS) and len(msg.points) == 1
        p = msg.points[0]
        horizon = p.time_from_start.sec + p.time_from_start.nanosec/1e9
        assert .005 <= horizon <= .10, horizon
        assert len(p.positions) == 6 and source_ns > 0
        assert 0 <= (received_ns-source_ns)/1e9 < .10
        horizons.append(horizon)
    displacement = max(max(abs(p-q[list(JOINTS).index(name)]) for name, p in zip(msg.joint_names, msg.points[0].positions)) for msg, _, _ in trajectories)
    assert displacement > 1e-6, f'Servo output never moved from input joints: {displacement}'
    print('Maximum requested joint displacement:', displacement, 'rad')
    print('Servo/RSP synthetic bench passed:', len(trajectories), 'targets; horizon range', min(horizons), max(horizons))
    print('No robot command publisher. Logs:', directory)
finally:
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    for p in processes:
        p.terminate()
    for p in processes:
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()
    log.close()
