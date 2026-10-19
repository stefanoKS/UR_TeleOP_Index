"""Real Servo/RSP processes with synthetic joints in isolated domain 147.
No UR driver, controller manager, hardware plugin or robot command publisher.
"""
import importlib.util
import math
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
from std_msgs.msg import Int8
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
tcp_test = os.environ.get('TELEOP_TEST_TCP') == '1'
if tcp_test:
    from ur_vr_teleop.preferences import load_settings
    test_settings = load_settings(context.launch_configurations['config_file'])
    test_settings['robot']['tcp_offset'] = [.04, -.02, .2, .2, -.1, .3]
    test_settings['teleop']['tool_frame'] = 'teleop_tcp'
    test_settings['effective_sources'] = {}
    test_config = Path(tempfile.mkdtemp(prefix='tcp_servo_config_'))/'config.yaml'
    test_config.write_text(yaml.safe_dump(test_settings))
    context.launch_configurations['config_file'] = str(test_config)
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
    servo_status = []
    node.create_subscription(Int8, '/vr_real/servo_node/status', lambda m: servo_status.append(m.data), 10)
    q = [0., -1.2, 1.6, -1.97, -1.57, 0.]
    def pump(seconds, move=False, oscillating_feedback=False, send_twist=True, rotate=False):
        began = time.monotonic()
        until = began+seconds
        while time.monotonic() < until:
            stamp = node.get_clock().now().to_msg()
            msg = JointState()
            positions, velocities = list(q), [0.]*6
            if oscillating_feedback:
                phase = 2*math.pi*2.6*(time.monotonic()-began)
                positions[0] += .004*math.sin(phase)
                velocities[0] = .004*2*math.pi*2.6*math.cos(phase)
            msg.header.stamp, msg.name = stamp, list(JOINTS)
            msg.position, msg.velocity = positions, velocities
            states.publish(msg)
            twist = TwistStamped()
            twist.header.stamp, twist.header.frame_id = stamp, 'base_link'
            twist.twist.linear.x = .001 if move else 0.
            twist.twist.angular.y = .001 if rotate else 0.
            if send_twist:
                twists.publish(twist)
            rclpy.spin_once(node, timeout_sec=.01)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Child exited; inspect '+str(directory/'processes.log'))
    pump(3.)
    # Verified against the installed MoveIt Servo 2.5.x: the status publisher exists as soon
    # as the process starts, but status is only published once start_servo begins its loop.
    assert node.count_publishers('/vr_real/servo_node/status') == 1, 'Servo status publisher missing'
    assert not servo_status, 'Servo must not report status before start_servo'
    assert start.service_is_ready(), 'Servo start service missing'
    future = start.call_async(Trigger.Request())
    pump(2.)
    assert future.done() and future.result().success, 'Servo did not start'
    assert servo_status and servo_status[-1] in (0, 1, 3, 6), 'Servo status not fresh/warning-free after start'
    if tcp_test:
        import xml.etree.ElementTree as ET
        import numpy as np
        from scipy.spatial.transform import Rotation
        model = ET.fromstring(params['robot_description'])
        links = {j.find('child').get('link'): j for j in model.findall('joint')}
        def fk(positions, link):
            if link not in links:
                return np.eye(4)
            joint = links[link]
            origin = joint.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3,3] = list(map(float, origin.get('xyz','0 0 0').split()))
                transform[:3,:3] = Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))).as_matrix()
            if joint.get('type') == 'revolute':
                axis = np.array(list(map(float,joint.find('axis').get('xyz').split())))
                rotation = np.eye(4)
                rotation[:3,:3] = Rotation.from_rotvec(axis*positions[joint.get('name')]).as_matrix()
                transform = transform@rotation
            return fk(positions,joint.find('parent').get('link'))@transform
        trajectories.clear()
        pump(1., rotate=True)
        assert len(trajectories) > 10
        before = dict(zip(JOINTS,q))
        tip0, flange0 = fk(before,'teleop_tcp'), fk(before,'tool0')
        moving = [msg for msg,_,_ in trajectories if max(map(abs,msg.points[0].velocities))>1e-6]
        assert moving, 'No angular response at configured tip'
        for msg in moving[-10:]:
            after = dict(zip(msg.joint_names,msg.points[0].positions))
            tip1, flange1 = fk(after,'teleop_tcp'), fk(after,'tool0')
            tip_shift = np.linalg.norm(tip1[:3,3]-tip0[:3,3])
            flange_shift = np.linalg.norm(flange1[:3,3]-flange0[:3,3])
            turn = Rotation.from_matrix(tip1[:3,:3]@tip0[:3,:3].T).magnitude()
            assert turn > 1e-7, turn
            assert flange_shift > 1e-8, flange_shift
            assert tip_shift < flange_shift*.02, (tip_shift, flange_shift)
        print('PASS: angular demand rotates about offset TCP; tip stays fixed and flange translates')
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
    # Feed measured oscillation back to actual Servo while keeping the requested
    # Cartesian velocity constant. A position-history filter turns this into
    # unwanted joint corrections; native Ruckig must own joint smoothing instead.
    trajectories.clear()
    pump(3., move=True, oscillating_feedback=True)
    assert len(trajectories) >= 100
    peak = max(abs(v) for msg, _, _ in trajectories[20:] for v in msg.points[0].velocities)
    assert peak < .01, f'Measured oscillation leaked into Servo velocity demand: {peak} rad/s'
    print('Oscillating-feedback regression peak joint request:', peak, 'rad/s')
    for label, send_twist in [('zero command', True), ('input timeout', False)]:
        if not send_twist:
            pump(.5, move=True)
        trajectories.clear()
        began_ns = node.get_clock().now().nanoseconds
        pump(.6, oscillating_feedback=True, send_twist=send_twist)
        halted = [msg for msg, _, received_ns in trajectories if received_ns-began_ns > 150_000_000]
        if send_twist:
            assert halted, 'No zero-command output'
            assert all(abs(v) < 1e-9 for msg in halted for v in msg.points[0].velocities), label
            print(label, 'produced zero velocity despite moving feedback')
        else:
            # Humble Servo suppresses stale-input output. The gate and native
            # watchdog own stopping when targets cease; do not expect a halt msg.
            assert trajectories, 'Timeout test never received pre-expiry targets'
            assert not halted, 'Servo continued publishing after input expiry'
            print(label, 'stopped publishing; downstream watchdog remains authoritative')
    trajectories.clear()
    pump(.5, move=True)
    assert any(abs(v) > 1e-6 for msg, _, _ in trajectories for v in msg.points[0].velocities), 'No recovery'
    assert node.count_publishers('/scaled_joint_trajectory_controller/joint_trajectory') == 0
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
