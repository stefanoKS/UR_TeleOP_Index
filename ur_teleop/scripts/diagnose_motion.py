#!/usr/bin/python3
"""Read-only 20-second teleop capture. No publishers or service clients."""
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '43':
        raise SystemExit('Source ROS and set ROS_DOMAIN_ID=43 first')
    rclpy.init()
    node = Node('read_only_teleop_motion_diagnostic')
    start = time.monotonic()
    events = []
    joints = {}
    counts = {}
    peaks = {'twist_linear_m_s': 0., 'twist_angular_rad_s': 0.,
             'servo_target_minus_feedback_rad': 0., 'robot_command_minus_feedback_rad': 0.,
             'controller_reference_minus_feedback_rad': 0., 'smoothed_command_minus_feedback_rad': 0.,
             'smoothed_command_velocity_rad_s': 0., 'smoothed_command_acceleration_rad_s2': 0.}
    bounds = {}
    status = {}

    def add(kind, value):
        counts[kind] = counts.get(kind, 0)+1
        events.append({'t': round(time.monotonic()-start, 6), 'kind': kind, 'value': value})

    def state(msg):
        joints.update(zip(msg.name, msg.position))
        for name, q in joints.items():
            lo, hi = bounds.get(name, (q, q))
            bounds[name] = min(lo, q), max(hi, q)
        add('joints', dict(joints))

    def trajectory(kind, msg):
        points = [{'q': list(p.positions), 'v': list(p.velocities), 'a': list(p.accelerations),
                   'horizon': p.time_from_start.sec+p.time_from_start.nanosec/1e9} for p in msg.points]
        if points and all(n in joints for n in msg.joint_names):
            delta = max((abs(q-joints[n]) for n, q in zip(msg.joint_names, points[-1]['q'])), default=0.)
            key = kind+'_minus_feedback_rad'
            peaks[key] = max(peaks[key], delta)
        if kind == 'smoothed_command' and points:
            peaks['smoothed_command_velocity_rad_s'] = max(peaks['smoothed_command_velocity_rad_s'],
                max(map(abs, points[-1]['v']), default=0.))
            peaks['smoothed_command_acceleration_rad_s2'] = max(peaks['smoothed_command_acceleration_rad_s2'],
                max(map(abs, points[-1]['a']), default=0.))
        add(kind, {'names': list(msg.joint_names), 'points': points,
                   'stamp_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nanosec})

    def twist(msg):
        v, w = msg.twist.linear, msg.twist.angular
        linear, angular = [v.x, v.y, v.z], [w.x, w.y, w.z]
        peaks['twist_linear_m_s'] = max(peaks['twist_linear_m_s'], sum(x*x for x in linear)**.5)
        peaks['twist_angular_rad_s'] = max(peaks['twist_angular_rad_s'], sum(x*x for x in angular)**.5)
        add('twist', {'linear': linear, 'angular': angular})

    def controller(msg):
        q, actual = list(msg.reference.positions), list(msg.feedback.positions)
        peaks['controller_reference_minus_feedback_rad'] = max(peaks['controller_reference_minus_feedback_rad'],
            max((abs(a-b) for a,b in zip(q,actual)), default=0.))
        add('controller', {'names': list(msg.joint_names), 'reference': q, 'feedback': actual,
                           'output': list(msg.output.positions)})

    def operator(msg):
        status.update(json.loads(msg.data))
        add('gate', dict(status))

    subscriptions = []
    for typ, topic, callback in [
        (JointState, '/joint_states', state),
        (TwistStamped, '/vr_real/servo_node/delta_twist_cmds', twist),
        (JointTrajectory, '/vr_real/servo_joint_targets', lambda m: trajectory('servo_target',m)),
        (JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', lambda m: trajectory('robot_command',m)),
        (JointTrajectory, '/scaled_joint_trajectory_controller/smoothed_command', lambda m: trajectory('smoothed_command',m)),
        (JointTrajectoryControllerState, '/scaled_joint_trajectory_controller/controller_state', controller),
        (String, '/vr_real/operator_status', operator),
        (Float64, '/speed_scaling_state_broadcaster/speed_scaling', lambda m: add('speed_scaling_percent',m.data)),
    ]:
        subscriptions.append(node.create_subscription(typ, topic, callback, qos_profile_sensor_data))
    print('Read-only capture started: 20 seconds.', flush=True)
    try:
        while time.monotonic()-start < 20:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        report = {'counts': counts, 'peaks': peaks, 'joint_span_rad': {k:hi-lo for k,(lo,hi) in bounds.items()},
                  'last_status': status, 'events': events}
        path = Path('/home/ubuntu-ros/Documents/Teleop/ur_teleop') / ('motion_diagnostic_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
        path.write_text(json.dumps(report))
        print(json.dumps({k:v for k,v in report.items() if k != 'events'}, indent=2))
        print('Saved:', path)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
