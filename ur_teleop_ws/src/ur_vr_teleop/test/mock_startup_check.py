"""Full generated-driver startup, localhost domain 148 and GenericSystem only."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from ur_vr_teleop.desktop import ProcessManager

os.environ['ROS_DOMAIN_ID'] = '148'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
with tempfile.TemporaryDirectory(prefix='teleop_mock_startup_') as directory:
    config = '/home/ubuntu-ros/Documents/Teleop/ur_teleop/config/real_teleop.yaml'
    tcp_test = os.environ.get('TELEOP_TEST_TCP') == '1'
    if tcp_test:
        import yaml
        from ur_vr_teleop.preferences import load_settings_with_sources
        settings, sources = load_settings_with_sources(config)
        settings['robot']['tcp_offset'] = [.04, -.02, .2, .2, -.1, .3]
        settings['teleop']['tool_frame'] = 'teleop_tcp'
        settings['effective_sources'] = sources
        config = Path(directory)/'config.yaml'
        config.write_text(yaml.safe_dump(settings))
    manager = ProcessManager(config)
    manager.directory = Path(directory)
    command = manager.commands(False)['driver']
    command = ['robot_ip:=127.0.0.1' if arg.startswith('robot_ip:=') else arg for arg in command]
    command.extend(['use_fake_hardware:=true', 'launch_dashboard_client:=false'])
    assert 'activate_joint_controller:=false' in command
    log_path = Path('/tmp/ur_teleop_mock_startup.log')
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node = None
        try:
            rclpy.init(domain_id=148)
            node = Node('mock_startup_observer')
            joints, descriptions = [], []
            node.create_subscription(JointState, '/joint_states', joints.append, qos_profile_sensor_data)
            node.create_subscription(String, '/robot_description', descriptions.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            deadline = time.monotonic()+25
            transform = None
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                try: transform = buffer.lookup_transform('base_link', 'teleop_tcp' if tcp_test else 'tool0', Time())
                except TransformException: pass
                if joints and descriptions and transform is not None: break
                assert process.poll() is None, log_path.read_text()[-4000:]
            assert joints and len(joints[-1].name) == 6, log_path.read_text()[-4000:]
            assert transform is not None, 'Missing base_link -> tool0 TF'
            plugins = ET.fromstring(descriptions[-1].data).findall('./ros2_control/hardware/plugin')
            assert len(plugins) == 1 and plugins[0].text.strip() in ('mock_components/GenericSystem', 'fake_components/GenericSystem')
            assert "Couldn't parse params file" not in log_path.read_text()
            print('PASS: generated ROS parameters, GenericSystem, six joint states and '+('teleop_tcp' if tcp_test else 'tool0')+' TF. Log:', log_path)
        finally:
            if node: node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
