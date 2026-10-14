"""Real rosbag smoke check in localhost domain 147; no driver or motion services."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_DOMAIN_ID'] = '147'
import rclpy
from sensor_msgs.msg import JointState
from ur_vr_teleop.validation import JOINTS
from ur_vr_teleop.sway_analysis import read_bag


def main():
    output = Path(tempfile.mkdtemp(prefix='teleop_sway_recording_check_'))
    config = Path(__file__).resolve().parents[4]/'ur_teleop/config/real_teleop.yaml'
    log = (output/'recorder.log').open('w')
    process = subprocess.Popen(['/usr/bin/python3', '-m', 'ur_vr_teleop.sway_recording',
        '--output-root', str(output), '--config', str(config)], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(domain_id=147)
    node = rclpy.create_node('sway_recorder_smoke')
    publisher = node.create_publisher(JointState, '/joint_states', 10)
    try:
        end = time.monotonic()+10
        while time.monotonic() < end and process.poll() is None:
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = list(JOINTS)
            message.position = message.velocity = [0.]*6
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.01)
        assert process.poll() is None, 'Recorder exited early: '+str(output/'recorder.log')
        assert node.count_publishers('/scaled_joint_trajectory_controller/joint_trajectory') == 0
        assert not any(name == '/vr_real/set_enabled' for name, _ in node.get_service_names_and_types())
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=25) == 0
    finally:
        if process.poll() is None:
            process.terminate(); process.wait(timeout=25)
        node.destroy_node(); rclpy.shutdown(); log.close()
    bag = next(output.glob('sway_*'))
    streams, _ = read_bag(bag)
    assert len(streams['actual_q']) > 10
    print('PASS: real rosbag recording, isolated domain 147,', len(streams['actual_q']),
          'synthetic joint samples, no motion publisher/service, clean finalization:', bag)


if __name__ == '__main__':
    main()
