"""Run explicitly: tests create ROS nodes in isolated localhost domain 147.

Not named test_*.py so ordinary unit runs do not require network privileges.
Transport/startup are real ROS; robot, tracking, TF and graph readiness are mocked.
No physical command publisher is created.
"""
import os
import json
from unittest.mock import Mock
from types import SimpleNamespace
import pytest
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
from ur_vr_teleop.commissioning import Commissioning
from ur_vr_teleop.validation import JOINTS
import time


@pytest.fixture
def node(monkeypatch):
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    # Explicit context domain wins over env; constructor still validates production env.
    monkeypatch.setenv('ROS_DOMAIN_ID', '43')
    rclpy.init(domain_id=147)
    n = Commissioning()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def healthy(n):
    now = n.get_clock().now().to_msg()
    pose = PoseStamped()
    pose.header.stamp, pose.header.frame_id = now, 'lighthouse'
    pose.pose.orientation.w = 1.
    if n.pose_quality.stamp is None:
        stamp = n.get_clock().now().nanoseconds
        for i in range(12):
            pose.header.stamp = rclpy.time.Time(nanoseconds=stamp-(12-i)*10_000_000).to_msg()
            n.on_pose(pose)
        pose.header.stamp = now
    n.on_pose(pose)
    joints = JointState()
    joints.header.stamp = now
    joints.name, joints.position, joints.velocity = list(JOINTS), [0.]*6, [0.]*6
    n.on_joints(joints)
    joy = Joy()
    joy.header.stamp, joy.buttons = now, [0]*16
    n.on_joy(joy)
    n.on_flag(Bool(data=True))
    n.on_optical(Bool(data=True))
    n.on_optical_detail(String(data='Gen2 accepted paired sensors (250 ms): LH0=4 LH1=4'))
    t = time.monotonic()
    n.optical_since = t-1.
    n.servo_status, n.servo_at = 0, t
    n.controller_ok, n.controller_at = True, t
    n.smoothing_controller_present = True
    n.hardware_smoothing = dict(velocity=.5, acceleration=1., jerk=10.)
    n.graph_ok, n.graph_at = True, t
    n.speed_scale, n.scale_at = 1., t
    n.robot_mode, n.safety_mode, n.program_running = 7, 1, True
    n.operator_at = t
    tf = TransformStamped()
    tf.header.stamp = now
    tf.transform.translation.x = .4
    tf.transform.rotation.w = 1.
    n.tf_buffer.lookup_transform = Mock(return_value=tf)
    result = n.set_parameters([Parameter('alignment_verified', value=True),
                               Parameter('optical_loss_test_verified', value=True)])
    assert all(r.successful for r in result)
    n.tick()


def test_dry_run_has_no_robot_publisher_and_arms_only_when_ready(node):
    assert node.command_pub is None
    assert not node.arm(Trigger.Request(), Trigger.Response()).success
    healthy(node)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.tick()
    assert node.gate.armed


def test_optical_loss_disarms_without_automatic_recovery(node):
    healthy(node)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.on_optical(Bool(data=False))
    assert not node.gate.armed
    node.on_optical(Bool(data=True))
    node.tick()
    assert not node.gate.armed


def test_settings_rejected_while_armed_and_mapping_invalidates_confirmation(node):
    healthy(node)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    assert not node.set_parameters([Parameter('max_linear_speed_m_s', value=.01)])[0].successful
    node.trip('test')
    assert node.set_parameters([Parameter('tracking_to_robot_rpy_deg', value=[0., 0., 90.])])[0].successful
    assert node.verified_mapping is None


def test_gui_loss_disarms(node):
    healthy(node)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.operator_at = time.monotonic()-1.
    node.tick()
    assert not node.gate.armed


def test_clutch_forwards_only_fresh_post_press_targets_in_dry_run(node):
    healthy(node)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.tick()  # release after arming
    joy = Joy()
    joy.header.stamp = node.get_clock().now().to_msg()
    joy.buttons = [0]*16
    joy.buttons[4] = 1
    node.on_joy(joy)
    node.tick()
    assert node.active_previous
    target = JointTrajectory()
    target.header.stamp = node.get_clock().now().to_msg()
    target.joint_names = list(JOINTS)
    p = JointTrajectoryPoint()
    p.positions = [.001]*6
    p.time_from_start = Duration(seconds=.06).to_msg()
    target.points = [p]
    node.on_trajectory(target)
    assert node.forwarded == 1
    joy.buttons[4] = 0
    node.on_joy(joy)
    node.on_trajectory(target)
    assert node.forwarded == 1
    node.tick()
    assert node.gate.armed


def test_malformed_servo_command_latches_disarm(node):
    healthy(node)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.tick()
    node.pressed = True
    node.tick()
    node.on_trajectory(JointTrajectory())
    assert not node.gate.armed


@pytest.mark.parametrize('fault', ['tracking', 'feedback', 'servo', 'scale', 'controller', 'graph', 'action'])
def test_health_faults_disarm(node, fault):
    healthy(node)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.tick()
    if fault == 'tracking': node.on_flag(Bool(data=False))
    elif fault == 'feedback': node.records['joints'] = (0., 1)
    elif fault == 'servo': node.servo_status = 2
    elif fault == 'scale': node.speed_scale = 0.
    elif fault == 'controller': node.controller_ok = False
    elif fault == 'graph': node.graph_ok = False
    elif fault == 'action': node.external_action = True
    node.tick()
    assert not node.gate.armed


def test_scale_is_percent_and_old_cartesian_ceiling_removed(node):
    node.on_scale(Float64(data=25.))
    assert node.speed_scale == .25
    assert node.set_parameters([Parameter('max_linear_speed_m_s', value=.03)])[0].successful


def test_live_output_serialization_and_hold_with_mock_publisher_only(node):
    healthy(node)
    # Never create a DDS publisher on the robot topic: inspect calls on a mock.
    node.motion_output = True
    node.command_pub = Mock()
    node.controller_timeout_ok = True
    node.controller_timeout_at = time.monotonic()
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.tick()
    node.pressed = True
    node.tick()
    target = JointTrajectory()
    target.header.stamp = node.get_clock().now().to_msg()
    target.joint_names = list(reversed(JOINTS))
    point = JointTrajectoryPoint()
    point.positions = [.001]*6
    point.time_from_start = Duration(seconds=.02).to_msg()
    target.points = [point]
    node.on_trajectory(target)
    output = node.command_pub.publish.call_args.args[0]
    assert output.joint_names == list(JOINTS)
    assert list(output.points[0].positions) == [.001]*6
    assert output.points[0].time_from_start.nanosec == 20_000_000
    assert not output.points[0].velocities
    assert not output.points[0].accelerations
    node.trip('Test optical loss')
    node.publish_hold()
    hold = node.command_pub.publish.call_args.args[0]
    assert list(hold.points[0].positions) == [0.]*6
    assert list(hold.points[0].velocities) == [0.]*6
    assert hold.points[0].time_from_start.nanosec == 100_000_000
    assert not node.gate.armed


def test_no_hold_fabricated_from_stale_feedback(node):
    healthy(node)
    node.command_pub = Mock()
    node.records['joints'] = (0., 1)
    node.begin_hold()
    node.publish_hold()
    node.command_pub.publish.assert_not_called()


def test_watchdog_requires_timeout_greater_than_goal_time(node):
    def read_timeout(timeout, goal_time):
        response = SimpleNamespace(values=[ParameterValue(type=3, double_value=timeout),
                                           ParameterValue(type=3, double_value=goal_time)])
        node.timeout_future = Mock()
        node.timeout_future.done.return_value = True
        node.timeout_future.result.return_value = response
        node.check_graph()
    read_timeout(.2, 0.)
    assert node.controller_timeout_ok
    read_timeout(.2, .3)
    assert not node.controller_timeout_ok
    read_timeout(0., 0.)
    assert not node.controller_timeout_ok


def test_live_gate_requires_final_smoothing_not_only_good_watchdog(node):
    healthy(node)
    node.motion_output = True
    node.controller_timeout_ok, node.controller_timeout_at = True, time.monotonic()
    assert not any('smoothing unavailable' in x for x in node.reasons())
    node.smoothing_controller_present = False
    assert any('smoothing unavailable' in x for x in node.reasons())
    node.smoothing_controller_present = True
    node.hardware_smoothing = None
    assert any('smoothing unavailable' in x for x in node.reasons())


def test_gui_cannot_silently_exceed_hardware_speed_limit(node):
    healthy(node)
    result = node.set_parameters([Parameter('max_joint_speed_rad_s', value=.7)])[0]
    assert not result.successful and 'Restart ROS' in result.reason


def test_gui_settings_over_actual_ros_parameter_service(node):
    caller = Node('operator_settings_regression_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(caller)
    try:
        client = caller.create_client(SetParametersAtomically, '/vr_real/teleop/set_parameters_atomically')
        assert client.wait_for_service(timeout_sec=2.)
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in [
            ('max_linear_speed_m_s', .01), ('max_angular_speed_deg_s', 5.),
            ('translation_scale', 1./1.2), ('rotation_scale', .5),
            ('vr_translation_deadband_m', .002), ('vr_rotation_deadband_deg', .4),
            ('tracking_to_robot_rpy_deg', [0., 0., 90.]),
            ('alignment_verified', True), ('optical_loss_test_verified', False)]]
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=3.)
        assert future.done()
        assert future.result().result.successful, future.result().result.reason
        assert tuple(node.get_parameter('tracking_to_robot_rpy_deg').value) == (0., 0., 90.)
        assert node.verified_mapping == (0., 0., 90.)
        node.configure()
        assert node.core.scale == pytest.approx(1./1.2)
        assert node.core.rotation_scale == .5
        assert node.core.translation_deadband == .002
        assert not node.gate.armed
    finally:
        executor.remove_node(caller)
        executor.remove_node(node)
        executor.shutdown()
        caller.destroy_node()


@pytest.mark.parametrize('future_name,client_name,time_name', [
    ('controller_future', 'controllers', 'controller_requested_at'),
    ('timeout_future', 'timeout_client', 'timeout_requested_at'),
])
def test_unanswered_readiness_request_retries_without_restoring_health(node, future_name, client_name, time_name):
    old = Mock()
    old.done.return_value = False
    replacement = Mock()
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.return_value = replacement
    setattr(node, future_name, old)
    setattr(node, client_name, client)
    setattr(node, time_name, time.monotonic()-2.)
    node.check_graph()
    client.remove_pending_request.assert_called_once_with(old)
    old.cancel.assert_called_once()
    assert getattr(node, future_name) is replacement
    assert not node.controller_ok
    assert not node.controller_timeout_ok
    assert not node.gate.armed
    client.call_async.reset_mock()
    replacement.done.return_value = False
    node.check_graph()
    client.call_async.assert_not_called()


def test_workspace_stop_reports_actual_limit_and_keeps_first_cause(node):
    assert node.set_parameters_atomically([Parameter('teleop_control_mode', value='position')]).successful
    node.workspace_radius = node.core.radius = .5  # Optional fence explicitly enabled.
    healthy(node)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.tick()
    node.pressed = True
    node.tick()
    # Small hand steps avoid the separate per-sample jump guard.
    for _ in range(16):
        node.hand[0] += .04
        node.tick()
    assert not node.gate.armed
    assert '50 cm workspace radius' in node.gate.reason
    first = node.gate.reason
    node.on_optical(Bool(data=False))
    node.tick()
    assert node.gate.reason == first
    assert not node.gate.armed


def test_configured_workspace_and_timeout_are_consistent_and_readonly(node):
    healthy(node)
    assert node.workspace_radius == 0
    assert node.core.radius == float('inf')
    assert node.observation_timeout == .35
    node.measured_tcp = node.core.home + [.49, 0., 0.]
    assert not any('Measured tool outside' in s for s in node.reasons())
    node.measured_tcp = node.core.home + [.51, 0., 0.]
    assert not any('Measured tool outside' in s for s in node.reasons())
    assert not node.set_parameters([Parameter('workspace_radius_m', value=.20)])[0].successful
    assert not node.set_parameters([Parameter('observation_timeout_s', value=.30)])[0].successful


@pytest.mark.parametrize('name', ['max_linear_speed_m_s', 'max_angular_speed_deg_s', 'max_joint_speed_rad_s', 'tracking_gain_s_inv'])
@pytest.mark.parametrize('value', [0., -1., float('inf'), float('nan')])
def test_speed_settings_reject_nonpositive_or_nonfinite(node, name, value):
    assert not node.set_parameters([Parameter(name, value=value)])[0].successful


def test_editable_cartesian_and_joint_limits_apply_together_without_arming(node):
    healthy(node)
    result = node.set_parameters_atomically([
        Parameter('max_linear_speed_m_s', value=.08),
        Parameter('max_angular_speed_deg_s', value=30.),
        Parameter('max_joint_speed_rad_s', value=.5)])
    assert result.successful
    node.tick()
    assert node.core.max_speed == .08
    assert node.core.max_angular_speed == pytest.approx(.5235987756)
    assert node.gate.max_joint_speed == .5
    node.joint_velocity[:] = .2
    assert not any('Measured joint velocity' in s for s in node.reasons())
    node.joint_velocity[:] = .61
    assert any('Measured joint velocity' in s for s in node.reasons())
    assert not node.gate.armed
    node.pressed = True
    assert not node.set_parameters([Parameter('max_joint_speed_rad_s', value=.6)])[0].successful
    node.pressed = False
    node.gate.armed = True
    assert not node.set_parameters([Parameter('max_joint_speed_rad_s', value=.6)])[0].successful


def enable_and_recover(node):
    healthy(node)
    assert node.set_enabled(SetBool.Request(data=True), SetBool.Response()).success
    node.tick()
    assert not node.gate.armed  # Stable recovery interval is required.
    node.recovery_since = time.monotonic()-.51
    node.tick()
    assert node.gate.armed
    assert not node.active_previous
    node.tick()  # Observe A released after establishing the new clutch reference.


def test_persistent_enable_survives_fault_but_held_a_never_rearms(node):
    enable_and_recover(node)
    node.pressed = True
    node.tick()
    assert node.active_previous
    node.on_optical(Bool(data=False))
    assert node.enable_requested and not node.gate.armed
    node.on_optical(Bool(data=True))
    node.optical_since = time.monotonic()-1.
    node.recovery_since = time.monotonic()-1.
    node.tick()
    assert not node.gate.armed and not node.active_previous
    assert node.recovery_since is None
    node.pressed = False
    node.tick()
    assert not node.gate.armed
    node.recovery_since = time.monotonic()-.51
    node.tick()
    assert node.gate.armed and not node.active_previous
    node.tick()
    node.pressed = True
    node.tick()
    assert node.active_previous


def test_uncheck_and_gui_loss_clear_enable_intent(node):
    enable_and_recover(node)
    assert node.set_enabled(SetBool.Request(data=False), SetBool.Response()).success
    assert not node.enable_requested and not node.gate.armed
    enable_and_recover(node)
    node.on_operator(Bool(data=False))
    assert not node.enable_requested and not node.gate.armed
    assert not node.set_enabled(SetBool.Request(data=True), SetBool.Response()).success


def test_heartbeat_expiry_clears_enable_and_motion(node):
    enable_and_recover(node)
    node.operator_at = time.monotonic()-1.
    node.tick()
    assert not node.enable_requested and not node.gate.armed


def test_held_a_or_unhealthy_input_prevents_enable_recovery(node):
    healthy(node)
    node.pressed = True
    assert node.set_enabled(SetBool.Request(data=True), SetBool.Response()).success
    node.recovery_since = time.monotonic()-1.
    node.tick()
    assert not node.gate.armed and node.recovery_since is None
    node.pressed = False
    node.controller_ok = False
    node.recovery_since = time.monotonic()-1.
    node.tick()
    assert not node.gate.armed and node.recovery_since is None


def test_motion_prevents_persistent_enable_recovery(node):
    healthy(node)
    assert node.set_enabled(SetBool.Request(data=True), SetBool.Response()).success
    node.joint_velocity[:] = .03
    node.recovery_since = time.monotonic()-1.
    node.tick()
    assert not node.gate.armed and node.recovery_since is None


def test_enable_checkbox_protocol_over_actual_ros_service(node):
    healthy(node)
    caller = Node('enable_checkbox_protocol_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(caller)
    try:
        client = caller.create_client(SetBool, '/vr_real/set_enabled')
        assert client.wait_for_service(timeout_sec=2.)
        node.operator_at = time.monotonic()
        future = client.call_async(SetBool.Request(data=True))
        executor.spin_until_future_complete(future, timeout_sec=2.)
        assert future.done() and future.result().success
        assert node.enable_requested
        future = client.call_async(SetBool.Request(data=False))
        executor.spin_until_future_complete(future, timeout_sec=2.)
        assert future.done() and future.result().success
        assert not node.enable_requested and not node.gate.armed
    finally:
        executor.remove_node(caller)
        executor.remove_node(node)
        executor.shutdown()
        caller.destroy_node()


def test_real_tracking_gain_and_scale_apply_without_a_fixed_home_fence(node):
    assert node.set_parameters_atomically([Parameter('teleop_control_mode', value='position')]).successful
    healthy(node)
    assert node.core.scale == pytest.approx(1./1.2)
    assert node.core.response_gain == 2.
    assert node.set_parameters_atomically([Parameter('vr_translation_filter_alpha', value=1.)]).successful
    assert node.set_parameters_atomically([Parameter('tracking_gain_s_inv', value=12.)]).successful
    node.tick()
    assert node.core.response_gain == 12.
    # Synthetic input only; no hardware publisher exists in this fixture.
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.tick()
    node.pressed = True
    node.tick()
    for _ in range(16):
        node.hand[0] += .04
        node.records['pose'] = (time.monotonic(), node.get_clock().now().nanoseconds)
        node.tick()
    assert node.gate.armed
    assert node.core.target[0]-node.core.home[0] == pytest.approx((.64-.001)/1.2)
    assert node.core.state == 'ACTIVE'


def test_displacement_display_works_without_enabling_motion(node):
    assert node.set_parameters_atomically([Parameter('teleop_control_mode', value='position')]).successful
    healthy(node)
    assert node.set_parameters_atomically([Parameter('vr_translation_filter_alpha', value=1.)]).successful
    node.tick()
    node.operator_status = Mock()
    node.twist = Mock()
    node.pressed = True
    node.tick()
    for _ in range(5):
        node.hand[0] += .02
        node.records['pose'] = (time.monotonic(), node.get_clock().now().nanoseconds)
        node.tick()
    data = json.loads(node.operator_status.publish.call_args.args[0].data)
    assert data['motion_metrics_current']
    assert data['motion_metrics']['hand_m'] == pytest.approx(.1)
    assert data['motion_metrics']['target_m'] == pytest.approx((.1-.001)/1.2)
    assert data['motion_metrics']['tool_m'] == pytest.approx(0.)
    assert not node.gate.armed and node.command_pub is None
    assert node.twist.publish.call_args.args[0].twist.linear.x == 0.
    node.pressed = False
    node.tick()
    data = json.loads(node.operator_status.publish.call_args.args[0].data)
    assert not data['motion_metrics_current']
    assert data['motion_metrics']['target_m'] == pytest.approx((.1-.001)/1.2)


@pytest.mark.parametrize('name,value', [('max_linear_acceleration_m_s2', 0.),
    ('max_angular_acceleration_rad_s2', -1.), ('joint_acceleration_warning_rad_s2', float('nan')),
    ('vr_translation_filter_alpha', 0.), ('vr_rotation_filter_alpha', 1.1)])
def test_invalid_smoothing_parameters_rejected(node, name, value):
    assert not node.set_parameters_atomically([Parameter(name, value=value)]).successful


def test_smoothing_parameters_apply_and_disabled_gate_resets_velocity(node):
    healthy(node)
    assert node.set_parameters_atomically([
        Parameter('max_linear_acceleration_m_s2', value=.4),
        Parameter('max_angular_acceleration_rad_s2', value=1.),
        Parameter('vr_translation_filter_alpha', value=1.),
        Parameter('vr_rotation_filter_alpha', value=.9),
        Parameter('joint_acceleration_warning_rad_s2', value=3.)]).successful
    node.tick()
    assert node.core.linear_acceleration == .4
    assert node.core.angular_acceleration == 1.
    assert node.core.translation_filter_alpha == 1.
    assert node.joint_acceleration_warning == 3.
    node.core.velocity[:] = 1.
    node.tick()
    assert all(v == 0. for v in node.core.velocity)


def test_native_ramp_lag_does_not_trip_but_large_target_recovers_with_release_only(node):
    enable_and_recover(node)
    node.motion_output = True
    node.controller_timeout_ok, node.controller_timeout_at = True, time.monotonic()
    node.command_pub = Mock()  # No real command publisher, localhost domain 147.
    node.pressed = True
    node.tick()
    target = JointTrajectory()
    target.joint_names = list(JOINTS)
    point = JointTrajectoryPoint()
    point.positions = [.012]*6  # Larger than the obsolete 5 mrad per-period bound.
    point.time_from_start = Duration(seconds=.01).to_msg()
    target.points = [point]
    target.header.stamp = node.get_clock().now().to_msg()
    node.on_trajectory(target)
    assert node.gate.armed and node.forwarded == 1
    assert node.command_pub.publish.call_count == 1
    point.positions = [.06]*6  # Still reject an excessive requested lead.
    target.header.stamp = node.get_clock().now().to_msg()
    node.on_trajectory(target)
    assert not node.gate.armed and node.enable_requested
    node.tick()  # Held A must never recover.
    assert not node.gate.armed
    node.pressed = False
    node.tick()
    node.recovery_since = time.monotonic()-.51
    node.tick()
    assert node.gate.armed and not node.active_previous
    node.tick()
    node.pressed = True
    node.tick()
    assert node.active_previous  # No disarm/start-Servo/re-enable cycle required.


def test_real_servo_velocities_preserved_with_joint_order_and_zero_stop(node):
    enable_and_recover(node)
    node.motion_output = True
    node.controller_timeout_ok, node.controller_timeout_at = True, time.monotonic()
    node.command_pub = Mock()
    node.pressed = True
    node.tick()
    target = JointTrajectory()
    target.joint_names = list(reversed(JOINTS))
    point = JointTrajectoryPoint()
    point.positions = [.003]*6
    point.velocities = [.1, .2, .3, .4, .5, .6]
    point.time_from_start = Duration(seconds=.01).to_msg()
    target.points = [point]
    target.header.stamp = node.get_clock().now().to_msg()
    node.on_trajectory(target)
    sent = node.command_pub.publish.call_args.args[0]
    assert list(sent.points[0].velocities) == list(reversed(point.velocities))
    assert sent.points[0].time_from_start == point.time_from_start
    point.velocities = [0.]*6
    target.header.stamp = node.get_clock().now().to_msg()
    node.on_trajectory(target)
    assert list(node.command_pub.publish.call_args.args[0].points[0].velocities) == [0.]*6


def test_lower_gui_joint_setting_cannot_claim_a_different_native_limit(node):
    healthy(node)
    result = node.set_parameters_atomically([Parameter('max_joint_speed_rad_s', value=.2)])
    assert not result.successful and 'Restart ROS' in result.reason


def test_damping_parameters_apply_and_are_reported_without_enabling(node):
    healthy(node)
    values = {'translation_damping_gain': .7, 'rotation_damping_gain': 0., 'damping_velocity_filter_s': .08}
    assert node.set_parameters_atomically([Parameter(k, value=v) for k, v in values.items()]).successful
    node.operator_status = Mock()
    node.tick()
    status = json.loads(node.operator_status.publish.call_args.args[0].data)
    for key, value in values.items():
        assert status[key] == pytest.approx(value)
        assert getattr(node.core, key) == pytest.approx(value)
    assert not node.gate.armed
    node.pressed = True
    assert not node.set_parameters_atomically([Parameter('translation_damping_gain', value=1.)]).successful


@pytest.mark.parametrize('key,value', [('translation_damping_gain', -1.), ('rotation_damping_gain', 1.01),
    ('translation_damping_gain', float('nan')), ('damping_velocity_filter_s', 0.),
    ('damping_velocity_filter_s', .201)])
def test_invalid_damping_is_rejected(node, key, value):
    assert not node.set_parameters_atomically([Parameter(key, value=value)]).successful


def test_velocity_mode_switch_requires_disarm_release_and_resets_history(node):
    healthy(node)
    assert node.core.control_mode == 'velocity'
    node.core.velocity[:] = .1
    node.core.velocity_sample = 123
    node.pressed = True
    assert not node.set_parameters_atomically([Parameter('teleop_control_mode', value='position')]).successful
    node.pressed = False
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    assert not node.set_parameters_atomically([Parameter('teleop_control_mode', value='position')]).successful
    node.disarm(None, Trigger.Response())
    assert node.set_parameters_atomically([Parameter('teleop_control_mode', value='position')]).successful
    assert node.core.velocity_sample is None
    assert not node.core.velocity.any()
    node.tick()
    assert node.core.control_mode == 'position'


def single_station_ready(node, counts="LH0=0 LH1=1"):
    healthy(node)
    node.on_optical_detail(String(data='Gen2 accepted paired sensors (250 ms): '+counts))
    node.optical_since = time.monotonic()-1.
    # Independent solver stream, not an optical-only readiness override.
    from ur_vr_teleop.tracking_quality import PoseQuality
    node.pose_quality = PoseQuality()
    now = node.get_clock().now().nanoseconds
    for i in range(16):
        pose = PoseStamped()
        pose.header.frame_id = 'lighthouse'
        pose.header.stamp = rclpy.time.Time(nanoseconds=now-(15-i)*10_000_000).to_msg()
        pose.pose.orientation.w = 1.
        pose.pose.position.x = i*.00001
        node.on_pose(pose)
    node.tick()


@pytest.mark.parametrize('counts', ['LH0=1 LH1=0', 'LH0=0 LH1=1', 'LH0=1 LH1=1'])
def test_either_station_signal_plus_valid_pose_qualifies_default_gate(node, counts):
    single_station_ready(node, counts)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    assert node.core.max_speed == pytest.approx(.1)
    assert node.core.linear_acceleration == pytest.approx(.25)
    assert node.core.max_angular_speed == pytest.approx(3.141592653589793/6)
    assert node.command_pub is None


def test_single_optical_observation_without_pose_cannot_arm(node):
    single_station_ready(node)
    node.pose_ok = False
    node.tick()
    assert not node.arm(Trigger.Request(), Trigger.Response()).success
    assert node.optical  # Optical presence is independent.


def test_single_station_loss_reacquisition_and_jump_require_fresh_clutch(node):
    single_station_ready(node)
    node.enable_requested = True
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.pressed = True
    node.on_optical_detail(String(data='Gen2 accepted paired sensors (250 ms): LH0=0 LH1=0'))
    assert not node.gate.armed
    node.on_optical_detail(String(data='Gen2 accepted paired sensors (250 ms): LH0=1 LH1=0'))
    node.optical_since = time.monotonic()-1.
    node.recover_enabled_gate([])
    assert not node.gate.armed
    pose = PoseStamped()
    pose.header.frame_id = 'lighthouse'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.orientation.w = 1.
    pose.pose.position.x = .4
    node.on_pose(pose)
    assert not node.pose_ok and node.core.state == 'WAIT_RELEASE'
    node.tick()
    assert not node.gate.armed and not node.core.velocity.any()


def test_effective_new_settings_in_status(node):
    healthy(node)
    node.operator_status = Mock()
    node.tick()
    data = json.loads(node.operator_status.publish.call_args.args[0].data)
    assert data['teleop_control_mode'] == 'velocity'
    assert data['tracking_station_policy'] == 'any_station'
    assert data['active_station_count'] == 2
    assert data['optical_stations'] == {'0': 4, '1': 4}
    assert data['linear_velocity_filter_s'] == .04


def test_offline_synthetic_bag_comparison(tmp_path):
    """Exercise actual ROS2 bag serialization/reading, without any ROS publisher."""
    import importlib.util
    from pathlib import Path
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from ur_vr_teleop.comparison import simulate
    path = tmp_path/'synthetic_comparison'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(name='/vr_real/operator_status',
        type='std_msgs/msg/String', serialization_format='cdr'))
    for offset, mode in enumerate(('position', 'velocity')):
        for row in simulate(mode, 'reversal'):
            diagnostic = {'observed_hand_velocity': [row['hand_speed'], 0., 0., 0., 0., 0.],
                'filtered_hand_velocity': [row['hand_speed'], 0., 0., 0., 0., 0.],
                'measured_tcp_velocity': [row['velocity'], 0., 0., 0., 0., 0.],
                'measured_tcp_position': [row['position'], 0., 0.],
                'command_age_s': .01, 'clamped': False}
            msg = String(data=json.dumps({'teleop_control_mode': mode, 'velocity_diagnostics': diagnostic}))
            writer.write('/vr_real/operator_status', serialize_message(msg), int((1+offset*10+row['t'])*1e9))
    del writer
    script = Path(__file__).resolve().parents[4]/'ur_teleop/scripts/compare_modes.py'
    spec = importlib.util.spec_from_file_location('compare_modes', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    report = module.bag_report(str(path))
    for mode in ('position', 'velocity'):
        assert report[mode]['samples'] == 800
        assert report[mode]['stop_latency_s']
        assert report[mode]['post_stop_events']
        assert report[mode]['reversal_latency_s']
        assert report[mode]['command_age_p50_s'] == pytest.approx(.01)


@pytest.mark.parametrize('field', ['position', 'quaternion'])
def test_invalid_tool_tf_faults_without_crashing_diagnostics(node, field):
    healthy(node)
    tf = node.tf_buffer.lookup_transform.return_value
    if field == 'position': tf.transform.translation.x = float('nan')
    else: tf.transform.rotation.w = 0.
    node.tick()
    assert not node.observation_healthy
    assert not node.gate.armed
    assert node.measured_tcp_velocity is None


@pytest.mark.parametrize('remaining', ['LH0=1 LH1=0', 'LH0=0 LH1=1'])
def test_one_station_disappears_without_disarming_valid_tracking(node, remaining):
    single_station_ready(node, 'LH0=1 LH1=1')
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    node.tick()  # Observe release after arming.
    node.pressed = True
    node.tick()
    assert node.core.state == 'ACTIVE'
    node.on_optical_detail(String(data='Gen2 accepted paired sensors (250 ms): '+remaining))
    node.tick()
    assert node.gate.armed and node.core.state == 'ACTIVE'
    assert node.policy_optical_ready()
    assert node.core.max_speed == pytest.approx(.1)
    node.on_optical(Bool(data=False))
    node.on_optical_detail(String(data='Gen2 accepted paired sensors (250 ms): LH0=0 LH1=0'))
    node.tick()
    assert not node.gate.armed and not node.core.velocity.any()
    node.on_optical(Bool(data=True))
    node.on_optical_detail(String(data='Gen2 accepted paired sensors (250 ms): '+remaining))
    node.optical_since = time.monotonic()-1.
    node.enable_requested = True
    node.tick()
    assert not node.gate.armed  # A still held after loss and recovery.


@pytest.mark.parametrize('legacy', ['two_station_strict', 'single_station_experimental'])
def test_legacy_policy_and_quarter_scale_do_not_restrict_single_station(node, legacy):
    assert node.set_parameters_atomically([
        Parameter('tracking_station_policy', value=legacy),
        Parameter('single_station_motion_scale', value=.25),
        Parameter('max_linear_speed_m_s', value=.07),
        Parameter('max_linear_acceleration_m_s2', value=.12),
    ]).successful
    single_station_ready(node)
    assert node.arm(Trigger.Request(), Trigger.Response()).success
    assert node.station_policy == 'any_station'
    assert node.motion_scale == 1.
    assert node.core.max_speed == pytest.approx(.07)
    assert node.core.linear_acceleration == pytest.approx(.12)
    assert node.core.angular_acceleration == pytest.approx(3.141592653589793/2)


def test_dashboard_readback_and_confirmations_match_actual_backend(node):
    from ur_vr_teleop.preferences import DEFAULTS
    node.operator_status = Mock()
    node.tick()
    payload = json.loads(node.operator_status.publish.call_args.args[0].data)
    assert set(payload['effective_parameters']) == set(DEFAULTS)
    assert not payload['alignment_verified']
    assert not payload['optical_loss_test_verified']
    healthy(node)
    payload = json.loads(node.operator_status.publish.call_args.args[0].data)
    assert payload['alignment_verified'] and payload['optical_loss_test_verified']
    assert payload['operator_heartbeat_fresh']
    assert payload['controller_state_fresh'] and payload['servo_status_fresh']
    node.controller_at = node.servo_at = -float('inf')
    node.tick()
    stale_payload=json.loads(node.operator_status.publish.call_args.args[0].data)
    assert not stale_payload['controller_state_fresh'] and not stale_payload['servo_status_fresh']
    assert payload['pose_fresh'] and payload['optical_stations'] == {'0': 4, '1': 4}
    node.set_parameters_atomically([Parameter('tracking_to_robot_rpy_deg', value=[0., 0., -40.])])
    node.tick()
    payload = json.loads(node.operator_status.publish.call_args.args[0].data)
    assert payload['effective_parameters']['tracking_to_robot_rpy_deg'] == [0., 0., -40.]
    assert not payload['alignment_verified']  # Parameter bool alone would still be true.
    assert node.get_parameter('alignment_verified').value
    assert node.command_pub is None
