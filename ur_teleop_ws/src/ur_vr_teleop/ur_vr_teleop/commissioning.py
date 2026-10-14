"""Disarmed-by-default Servo staging and bounded command gate.

Default dry run does not even create a publisher on the robot command topic.
No driver start, program start, controller switch, reset or fault clearing here.
"""
import json
import math
import os
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rcl_interfaces.srv import GetParameters
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, String, Int8, Float64
from std_srvs.srv import Trigger, SetBool
from controller_manager_msgs.srv import ListControllers
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from scipy.spatial.transform import Rotation
from .preview import Preview, stamp_ns
from .validation import JOINTS, fresh
from .gate import Gate
from .tracking import CONTROL_PERIOD_S, DEFAULT_RESPONSE_GAIN
from .joint_diagnostics import JointDiagnostics
from .preferences import DEFAULTS


COMMAND = '/scaled_joint_trajectory_controller/joint_trajectory'
STAGED = '/vr_real/servo_joint_targets'


SERVO_STATUS = {
    -1: 'Invalid / uninitialized Servo status', 0: 'Ready',
    1: 'Slowing near singularity', 2: 'Halted: singularity',
    3: 'Slowing near collision', 4: 'Halted: collision',
    5: 'Halted: joint position or velocity bound', 6: 'Slowing while leaving singularity',
}


def servo_status_detail(code):
    return SERVO_STATUS.get(code, f'Unknown Servo status ({code})')


class Commissioning(Preview):
    def __init__(self):
        super().__init__(observation_only=False, node_name='teleop')
        self.gate = Gate()
        self.session_id = str(uuid.uuid4())
        self.joint_diagnostics = JointDiagnostics()
        self.last_joint_warning = {}
        self.enable_requested = False
        self.recovery_since = None
        self.verified_mapping = None
        self.motion_output = self.declare_parameter('motion_output_enabled', False,
            ParameterDescriptor(read_only=True)).value
        if type(self.motion_output) is not bool:
            raise ValueError('motion_output_enabled must be boolean')
        for name, value in [('max_linear_speed_m_s', .10), ('max_angular_speed_deg_s', 30.),
                            ('max_joint_speed_rad_s', .50),
                            ('tracking_gain_s_inv', DEFAULT_RESPONSE_GAIN),
                            ('max_linear_acceleration_m_s2', .25),
                            ('max_angular_acceleration_rad_s2', math.pi/2),
                            ('vr_translation_filter_alpha', .8),
                            ('vr_rotation_filter_alpha', .8),
                            ('rotation_scale', 1.),
                            ('vr_translation_deadband_m', .001),
                            ('vr_rotation_deadband_deg', .3),
                            ('translation_damping_gain', 0.),
                            ('rotation_damping_gain', 0.),
                            ('damping_velocity_filter_s', .05),
                            ('joint_acceleration_warning_rad_s2', 5.),
                            ('tracking_to_robot_rpy_deg', [0., 0., 0.]),
                            ('alignment_verified', False), ('optical_loss_test_verified', False)]:
            self.declare_parameter(name, value)
        from .preferences import DEFAULTS
        for name in ('teleop_control_mode', 'tracking_station_policy', 'linear_velocity_filter_s',
                     'angular_velocity_filter_s', 'single_station_motion_scale'):
            self.declare_parameter(name, DEFAULTS[name])
        self.optical_stations = {}
        self.optical_detail_at = -float('inf')
        self.workspace_radius = self.declare_parameter('workspace_radius_m', 0.,
            ParameterDescriptor(read_only=True)).value
        if not math.isfinite(self.workspace_radius) or not (self.workspace_radius == 0 or .05 <= self.workspace_radius <= .50):
            raise ValueError('workspace_radius_m must be 0 (disabled) or between 0.05 and 0.50 m')
        self.core.radius = self.workspace_radius if self.workspace_radius else float('inf')
        # Deprecated wire input, retained for older launch files; never grants a bypass.
        self.declare_parameter('engineering_required', False,
            ParameterDescriptor(read_only=True)).value
        encoded = self.declare_parameter('engineering_context', '', ParameterDescriptor(read_only=True)).value
        from .configuration import Configuration
        self.configuration = Configuration(encoded) if encoded else None
        self.configuration_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='configuration-files')
        self.configuration_future = None
        self.configuration_checked = -float('inf')
        self.configuration_parameters = None
        self.configuration_valid = False
        self.configuration_reason = 'Waiting for configuration check'
        self.controller_preparation_allowed = False
        self.configure()
        self.add_on_set_parameters_callback(self.on_settings)
        self.joint_values = self.joint_velocity = None
        self.optical = False
        self.optical_at = self.optical_since = -float('inf')
        self.optical_detail = 'No optical data'
        self.servo_status = -1
        self.servo_at = self.scale_at = self.operator_at = -float('inf')
        self.speed_scale = 0.
        self.robot_mode = self.safety_mode = None
        self.program_running = False
        self.controller_ok = False
        self.smoothing_controller_present = False
        self.hardware_smoothing = None
        self.controller_at = self.graph_at = -float('inf')
        self.graph_ok = False
        self.controller_future = None
        self.timeout_future = None
        self.controller_requested_at = self.timeout_requested_at = -float('inf')
        self.controller_timeout_ok = False
        self.controller_timeout_at = -float('inf')
        self.external_action = False
        self.active_previous = False
        self.active_since = None
        self.forwarded = 0
        self.hold_until = 0.
        self.hold_q = None
        self.last_health = ['Waiting for observations']
        self.command_pub = self.create_publisher(JointTrajectory, COMMAND, 1) if self.motion_output else None
        self.twist = self.create_publisher(TwistStamped, '/vr_real/servo_node/delta_twist_cmds', 1)
        self.armed_pub = self.create_publisher(Bool, '/vr_real/armed', 1)
        self.operator_status = self.create_publisher(String, '/vr_real/operator_status', 1)
        self.create_subscription(Bool, '/vr/controller/optical_fresh', self.on_optical, 1)
        self.create_subscription(String, '/vr/controller/optical_status', self.on_optical_detail, 1)
        self.create_subscription(Int8, '/vr_real/servo_node/status', self.on_servo, 1)
        self.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling', self.on_scale, 1)
        self.create_subscription(JointTrajectory, STAGED, self.on_trajectory, 1)
        self.create_subscription(Bool, '/vr_real/operator_present', self.on_operator, 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(RobotMode, '/io_and_status_controller/robot_mode', self.on_mode, latched)
        self.create_subscription(SafetyMode, '/io_and_status_controller/safety_mode', self.on_safety, latched)
        self.create_subscription(Bool, '/io_and_status_controller/robot_program_running', self.on_program, latched)
        self.create_subscription(GoalStatusArray,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/status', self.on_action, latched)
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.timeout_client = self.create_client(GetParameters, '/scaled_joint_trajectory_controller/get_parameters')
        self.create_timer(.2, self.check_graph)
        self.create_service(Trigger, '/vr_real/arm', self.arm)
        self.create_service(SetBool, '/vr_real/set_enabled', self.set_enabled)
        self.create_service(Trigger, '/vr_real/disarm', self.disarm)
        self.get_logger().warn('COMMISSIONING: ' + ('LIVE OUTPUT enabled; disarmed' if self.motion_output else 'DRY RUN — no robot command publisher'))

    def configure(self):
        from .preferences import validate
        extra = {key: self.get_parameter(key).value for key in ('teleop_control_mode', 'tracking_station_policy',
                 'linear_velocity_filter_s', 'angular_velocity_filter_s', 'single_station_motion_scale')}
        validate(extra)
        self.core.control_mode = extra['teleop_control_mode']
        self.station_policy = 'any_station'  # Old saved policy names remain accepted aliases.
        self.core.linear_velocity_filter_s = extra['linear_velocity_filter_s']
        self.core.angular_velocity_filter_s = extra['angular_velocity_filter_s']
        # Station visibility never changes independently configured motion limits.
        self.motion_scale = 1.
        linear = self.get_parameter('max_linear_speed_m_s').value
        angular = self.get_parameter('max_angular_speed_deg_s').value
        joint = self.get_parameter('max_joint_speed_rad_s').value
        gain = self.get_parameter('tracking_gain_s_inv').value
        mapping = self.get_parameter('tracking_to_robot_rpy_deg').value
        if any(not math.isfinite(v) or v <= 0 for v in (linear, angular, joint, gain)) or not math.isfinite(1.2*joint):
            raise ValueError('Speed values must be finite positive numbers within numeric range')
        self.gate.max_joint_speed = joint
        self.core.response_gain = gain
        from .preferences import validate
        tuning = {key: self.get_parameter(key).value for key in
                  ('translation_scale', 'rotation_scale', 'vr_translation_deadband_m', 'vr_rotation_deadband_deg',
                   'translation_damping_gain', 'rotation_damping_gain', 'damping_velocity_filter_s')}
        validate(tuning)
        self.core.scale = tuning['translation_scale']
        self.core.rotation_scale = tuning['rotation_scale']
        self.core.translation_deadband = tuning['vr_translation_deadband_m']
        self.core.rotation_deadband = math.radians(tuning['vr_rotation_deadband_deg'])
        for key in ('translation_damping_gain', 'rotation_damping_gain', 'damping_velocity_filter_s'):
            setattr(self.core, key, tuning[key])
        for parameter, field in [('max_linear_acceleration_m_s2', 'linear_acceleration'),
                                 ('max_angular_acceleration_rad_s2', 'angular_acceleration'),
                                 ('vr_translation_filter_alpha', 'translation_filter_alpha'),
                                 ('vr_rotation_filter_alpha', 'rotation_filter_alpha')]:
            value = self.get_parameter(parameter).value
            if not math.isfinite(value) or value <= 0 or ('alpha' in parameter and value > 1):
                raise ValueError(parameter + ' must be positive finite; filter alpha must be <=1 (1 disables)')
            setattr(self.core, field, value)
        self.joint_acceleration_warning = self.get_parameter('joint_acceleration_warning_rad_s2').value
        if not math.isfinite(self.joint_acceleration_warning) or self.joint_acceleration_warning <= 0:
            raise ValueError('joint_acceleration_warning_rad_s2 must be finite positive')
        if len(mapping) != 3 or not all(math.isfinite(x) for x in mapping):
            raise ValueError('Alignment requires three finite RPY degrees')
        self.core.max_speed, self.core.max_angular_speed = linear, math.radians(angular)
        self.core.axis_rotation = Rotation.from_euler('xyz', mapping, degrees=True)
        self.core.rotation = self.core.axis_rotation.as_matrix()

    def on_settings(self, params):
        allowed = {'max_linear_speed_m_s', 'max_angular_speed_deg_s', 'max_joint_speed_rad_s', 'tracking_gain_s_inv',
                   'max_linear_acceleration_m_s2', 'max_angular_acceleration_rad_s2',
                   'vr_translation_filter_alpha', 'vr_rotation_filter_alpha', 'joint_acceleration_warning_rad_s2'}
        if self.gate.armed or self.pressed:
            return SetParametersResult(successful=False, reason='Disarm and release A before changing settings')
        for p in params:
            if p.name in ('teleop_control_mode', 'tracking_station_policy', 'linear_velocity_filter_s',
                          'angular_velocity_filter_s', 'single_station_motion_scale'):
                from .preferences import validate
                try:
                    validate({p.name: p.value})
                except ValueError as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
            elif p.name in ('translation_scale', 'rotation_scale', 'vr_translation_deadband_m', 'vr_rotation_deadband_deg',
                           'translation_damping_gain', 'rotation_damping_gain', 'damping_velocity_filter_s'):
                from .preferences import validate
                try:
                    if type(p.value) is not float:
                        raise ValueError(p.name + ' requires a floating point number')
                    validate({p.name: p.value})
                except ValueError as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
            elif p.name in allowed:
                if (type(p.value) is not float or not math.isfinite(p.value) or p.value <= 0
                    or (p.name == 'max_joint_speed_rad_s' and not math.isfinite(1.2*p.value))
                    or ('filter_alpha' in p.name and p.value > 1)):
                    return SetParametersResult(successful=False, reason=f'{p.name} must be a finite positive number within numeric range')
                if (p.name == 'max_joint_speed_rad_s' and self.hardware_smoothing and
                        not math.isclose(p.value, self.hardware_smoothing['velocity'], rel_tol=1e-9)):
                    return SetParametersResult(successful=False, reason='Joint speed is a final-controller startup setting; use Apply and save parameters, then Restart ROS')
            elif p.name == 'tracking_to_robot_rpy_deg':
                # ROS service deserialization yields array.array('d'), not list.
                if p.type_ != Parameter.Type.DOUBLE_ARRAY or len(p.value) != 3 or not all(math.isfinite(x) for x in p.value):
                    return SetParametersResult(successful=False, reason='Expected finite RPY degrees')
            elif p.name in ('alignment_verified', 'optical_loss_test_verified'):
                if type(p.value) is not bool:
                    return SetParametersResult(successful=False, reason='Expected boolean')
            else:
                return SetParametersResult(successful=False, reason='Setting is fixed; restart for configuration changes')
        values = {p.name: p.value for p in params}
        mapping = values.get('tracking_to_robot_rpy_deg', self.get_parameter('tracking_to_robot_rpy_deg').value)
        # An existing confirmation must not silently approve a new mapping.
        if 'tracking_to_robot_rpy_deg' in values and tuple(mapping) != tuple(self.get_parameter('tracking_to_robot_rpy_deg').value):
            self.verified_mapping = None
        if 'alignment_verified' in values:
            self.verified_mapping = tuple(mapping) if values['alignment_verified'] else None
        if 'tracking_station_policy' in values:
            self.optical_since = -float('inf')
        self.core.block('Settings changed; release A')
        return SetParametersResult(successful=True)

    def on_joints(self, msg):
        super().on_joints(msg)
        if self.joints_ok:
            self.joint_values = np.array([msg.position[msg.name.index(j)] for j in JOINTS])
            self.joint_velocity = (np.array([msg.velocity[msg.name.index(j)] for j in JOINTS])
                if len(msg.velocity) == len(msg.name) and all(math.isfinite(x) for x in msg.velocity) else None)
        else:
            self.joint_values = self.joint_velocity = None

    def on_joy(self, msg):
        was_pressed = self.pressed
        super().on_joy(msg)
        if was_pressed and not self.pressed and self.gate.armed:
            self.begin_hold()
            self.core.block('A released; fresh clutch reference required')
            self.active_previous = False

    def on_action(self, msg):
        self.external_action = any(s.status in (1, 2, 3) for s in msg.status_list)
        if self.external_action: self.trip('Another trajectory action is active')

    def on_optical(self, msg):
        t = time.monotonic()
        if not msg.data:
            self.optical_since = -float('inf')
            self.trip('Optical observations lost')
        elif not self.optical or t - self.optical_at >= .20:
            self.optical_since = t
        self.optical, self.optical_at = bool(msg.data), t

    def on_optical_detail(self, msg):
        self.optical_detail = msg.data
        try:
            import re
            self.optical_stations = {int(k): int(v) for k, v in re.findall(r'LH(\d+)=(\d+)', msg.data)}
            self.optical_detail_at = time.monotonic()
            if not self.policy_optical_ready():
                self.optical_since = -float('inf')
                self.trip('Optical station policy not satisfied; release A')
            elif not math.isfinite(self.optical_since):
                self.optical_since = time.monotonic()
        except (ValueError, TypeError):
            self.optical_stations = {}
            self.trip('Invalid optical diagnostic; release A')

    def policy_optical_ready(self):
        from .tracking_quality import optical_ready
        return (time.monotonic()-self.optical_detail_at < .20 and
                optical_ready(self.optical_stations, self.station_policy))
    def on_servo(self, msg):
        self.servo_status, self.servo_at = int(msg.data), time.monotonic()
        if self.servo_status not in (0, 1, 3, 6): self.trip(servo_status_detail(self.servo_status))
    def on_scale(self, msg):
        # Installed UR broadcaster publishes percent (state interface * 100).
        self.speed_scale, self.scale_at = msg.data / 100., time.monotonic()
    def on_operator(self, msg):
        self.operator_at = time.monotonic() if msg.data else -float('inf')
        if not msg.data:
            self.enable_requested = False
            self.recovery_since = None
            self.trip('Operator GUI disconnected')
    def on_mode(self, msg):
        self.robot_mode = msg.mode
        if msg.mode != 7: self.trip('Robot is not running')
    def on_safety(self, msg):
        self.safety_mode = msg.mode
        if msg.mode not in (1, 2): self.trip('Robot safety mode blocks motion')
    def on_program(self, msg):
        self.program_running = msg.data
        self.program_received = True
        if not msg.data: self.trip('External Control is not running')

    def check_graph(self):
        now = time.monotonic()
        self.check_configuration(now)
        # A request lost during service startup/restart must not wedge polling.
        # Readiness expires independently after 0.6 s; retries never renew it.
        for future_name, client, requested_at in (
                ('controller_future', self.controllers, self.controller_requested_at),
                ('timeout_future', self.timeout_client, self.timeout_requested_at)):
            future = getattr(self, future_name)
            if future is not None and not future.done() and now-requested_at >= 1.:
                client.remove_pending_request(future)
                future.cancel()
                setattr(self, future_name, None)
        if self.controller_future is not None and self.controller_future.done():
            try:
                cs = self.controller_future.result().controller
                active = {c.name for c in cs if c.state == 'active'}
                required = 'scaled_joint_trajectory_controller'
                self.smoothing_controller_present = any(c.name == required and c.type ==
                    'ur_teleop_controllers/SmoothScaledJointTrajectoryController' for c in cs)
                claiming = {c.name for c in cs if c.state == 'active' and
                            any(i.endswith(('/position', '/velocity', '/effort')) for i in c.claimed_interfaces)}
                self.controller_preparation_allowed = not claiming and any(c.name == required and c.state == 'inactive' for c in cs)
                self.controller_ok = (required in active and claiming == {required}) if self.motion_output else not claiming
                self.controller_at = now
            except Exception:
                self.controller_ok = False
                self.controller_preparation_allowed = False
                self.smoothing_controller_present = False
            self.controller_future = None
        if self.controller_future is None and self.controllers.service_is_ready():
            self.controller_future = self.controllers.call_async(ListControllers.Request())
            self.controller_requested_at = now
        if self.timeout_future is not None and self.timeout_future.done():
            try:
                values = self.timeout_future.result().values
                self.controller_timeout_ok = (len(values) >= 2 and all(v.type == 3 for v in values[:2])
                    and .10 <= values[0].double_value <= .25
                    and 0 <= values[1].double_value < values[0].double_value)
                self.hardware_smoothing = (dict(zip(('velocity', 'acceleration', 'jerk'),
                    (v.double_value for v in values[2:]))) if len(values) == 5 and all(
                        v.type == 3 and math.isfinite(v.double_value) and v.double_value > 0 for v in values[2:]) else None)
                self.controller_timeout_at = now
            except Exception:
                self.controller_timeout_ok = False
                self.hardware_smoothing = None
            self.timeout_future = None
        if self.timeout_future is None and self.timeout_client.service_is_ready():
            request = GetParameters.Request()
            request.names = ['cmd_timeout', 'constraints.goal_time', 'smoothing.max_velocity',
                             'smoothing.max_acceleration', 'smoothing.max_jerk']
            self.timeout_future = self.timeout_client.call_async(request)
            self.timeout_requested_at = now
        # One upstream source per required observation; reject accidental duplicate drivers/bridges.
        topics = ['/joint_states', '/vr/controller/pose', '/vr/controller/joy',
                  '/vr/controller/pose_fresh', '/vr/controller/optical_fresh', '/vr/controller/optical_status', STAGED,
                  '/vr_real/servo_unstamped_targets',
                  '/vr_real/operator_present',
                  '/io_and_status_controller/robot_mode', '/io_and_status_controller/safety_mode',
                  '/io_and_status_controller/robot_program_running']
        self.graph_ok = all(self.count_publishers(topic) == 1 for topic in topics)
        self.graph_ok = self.graph_ok and self.count_publishers(COMMAND) == (1 if self.motion_output else 0)
        self.graph_at = now

    def reasons(self):
        t = time.monotonic()
        problems = []
        if not getattr(self, 'observation_healthy', False):
            problems.append(getattr(self, 'observation_reason', 'Waiting for VR/joints/TF observations; release A'))
        ns = self.get_clock().now().nanoseconds
        if any(k not in self.records or not fresh(*self.records[k], t, ns, self.observation_timeout) for k in ('pose', 'joy', 'joints')):
            problems.append('Observation timestamps are stale')
        if self.joint_velocity is None or np.max(np.abs(self.joint_velocity)) > 1.2*self.gate.max_joint_speed:
            problems.append(f'Measured joint velocity missing or above {1.2*self.gate.max_joint_speed:g} rad/s (configured joint speed +20%)')
        if self.workspace_radius > 0 and self.core.home is not None and hasattr(self, 'measured_tcp') and np.linalg.norm(self.measured_tcp-self.core.home) > self.core.radius + .001:
            problems.append(f'Measured tool outside {self.core.radius*100:g} cm workspace radius')
        if not self.optical or not self.policy_optical_ready() or t-self.optical_at >= .20 or t-self.optical_since < .5:
            problems.append('Need stable optical observations from either calibrated station; signal presence alone does not validate a 6-DoF pose')
        if not self.pose_quality.continuous:
            problems.append('Tracking needs at least 100 ms of continuous independent 6-DoF poses')
        if t-self.servo_at >= .25: problems.append('Servo status stale (no update for 250 ms)')
        elif self.servo_status not in (0, 1, 3, 6): problems.append(servo_status_detail(self.servo_status))
        if self.robot_mode != 7:
            problems.append(f'Robot mode {self.robot_mode}: RUNNING (7) required')
        if self.safety_mode not in (1, 2):
            problems.append(f'Robot safety mode {self.safety_mode}: NORMAL/REDUCED required')
        if not self.program_running:
            problems.append('External Control program is not running' if getattr(self, 'program_received', False)
                            else 'No External Control program-running status received')
        if t-self.scale_at >= .25:
            problems.append('Speed scaling feedback missing/stale')
        elif not math.isfinite(self.speed_scale) or not 0 <= self.speed_scale <= 1.:
            problems.append('Speed scaling feedback invalid')
        elif self.speed_scale == 0:
            problems.append('Speed scaling is 0%: check pendant program, pause and speed slider')
        if not self.controller_ok or t-self.controller_at >= .6:
            problems.append('Controller state mismatch/stale (' + ('scaled controller must be active alone' if self.motion_output else 'all motion controllers must be inactive') + ')')
        if not self.graph_ok or t-self.graph_at >= .6: problems.append('Missing/duplicate ROS publishers')
        if self.external_action: problems.append('Another trajectory action is active')
        if self.motion_output and (not self.controller_timeout_ok or t-self.controller_timeout_at >= .6):
            problems.append('Controller cmd_timeout must be 0.10–0.25 s and readable')
        if self.motion_output and (not self.smoothing_controller_present or not self.hardware_smoothing
                                  or t-self.controller_timeout_at >= .6):
            problems.append('Final joint-command smoothing unavailable/stale; Restart ROS with the updated launcher')
        if self.motion_output and self.hardware_smoothing and self.gate.max_joint_speed > self.hardware_smoothing['velocity']:
            problems.append('GUI joint speed exceeds final controller speed limit; lower it or update config and restart')
        if t-self.operator_at >= .5: problems.append('Operator GUI heartbeat missing')
        if self.configuration is not None:
            if self.motion_output and not self.configuration_controller_matches():
                problems.append('Effective native smoothing limits differ from the startup configuration; inspect configuration and restart')
            if not self.configuration_ready():
                problems.append(self.configuration_status_reason())
        else:
            problems.append('Configuration context missing; use the configured managed launch')
        return problems

    def configuration_values(self):
        return dict({k: list(self.get_parameter(k).value) if k == 'tracking_to_robot_rpy_deg'
                     else self.get_parameter(k).value for k in DEFAULTS},
                    base_frame=self.base, tool_frame=self.tool)

    def check_configuration(self, now):
        # File I/O and hashing never run on the 100 Hz command executor.
        if self.configuration is None:
            return
        if self.configuration_future is not None and self.configuration_future.done():
            try:
                self.configuration_valid = self.configuration_future.result()
                self.configuration_reason = self.configuration.reason
                self.configuration_parameters = self.configuration_submitted
                self.configuration_checked = now
            except Exception as error:
                self.configuration_valid = False
                self.configuration_reason = 'Configuration could not be checked. Open Advanced Diagnostics.'
                self.get_logger().error('Configuration check: ' + repr(error))
            self.configuration_future = None
        if self.configuration_future is None and now-self.configuration_checked >= .5:
            self.configuration_submitted = self.configuration_values()
            self.configuration_future = self.configuration_pool.submit(self.configuration.refresh, self.configuration_submitted)

    def configuration_status_reason(self):
        if self.configuration_valid and not self.configuration_ready():
            return 'Waiting for current configuration validation'
        return self.configuration_reason

    def configuration_controller_matches(self):
        if not self.hardware_smoothing or self.configuration is None:
            return False
        settings = self.configuration.context['settings']
        expected = dict(velocity=settings['teleop']['max_joint_speed_rad_s'],
                        acceleration=settings['robot'].get('joint_command_acceleration_rad_s2', 1.),
                        jerk=settings['robot'].get('joint_command_jerk_rad_s3', 10.))
        return all(math.isclose(self.hardware_smoothing[k], v, rel_tol=1e-9)
                   for k, v in expected.items())

    def configuration_ready(self):
        return (self.configuration_valid and time.monotonic()-self.configuration_checked < 2.
                and self.configuration_parameters == self.configuration_values())

    def tracking_valid(self):
        t = time.monotonic()
        return bool(self.pose_ok and self.flag and t-self.flag_at < self.observation_timeout
                    and fresh(*self.records.get('pose', (-1e30, 0)), t,
                              self.get_clock().now().nanoseconds, self.observation_timeout)
                    and self.pose_quality.continuous and self.optical and self.policy_optical_ready()
                    and t-self.optical_at < .20 and t-self.optical_since >= .5)

    def robot_preparation_reason(self):
        if self.robot_mode != 7 or self.safety_mode not in (1, 2):
            return 'Robot mode / safety state blocks startup; inspect the pendant'
        if not self.program_running:
            return 'Start the External Control program on the pendant; waiting for confirmed running state'
        if time.monotonic()-self.scale_at >= .25 or not math.isfinite(self.speed_scale) or not 0 < self.speed_scale <= 1:
            return 'Waiting for fresh, nonzero speed scaling; check pendant program and slider'
        return ''

    def destroy_node(self):
        if hasattr(self, 'configuration_pool'):
            self.configuration_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def arm(self, request, response):
        # GUI must be live; A released; measured velocity must establish standstill.
        stationary = self.joint_velocity is not None and np.max(np.abs(self.joint_velocity)) < .02
        response.success = self.gate.arm(self.reasons(), self.pressed, stationary)
        if response.success:
            self.core.block('Armed; release then press A')
            self.hold_q = None
            self.hold_until = 0.
        response.message = self.gate.reason
        return response

    def set_enabled(self, request, response):
        if request.data and time.monotonic()-self.operator_at >= .5:
            response.success, response.message = False, 'GUI heartbeat missing; enable request rejected'
            return response
        self.enable_requested = bool(request.data)
        self.recovery_since = None
        if not self.enable_requested:
            self.disarm(None, Trigger.Response())
        response.success = True
        response.message = ('Enabled: waiting for healthy inputs and A released; then press A to move'
                            if self.enable_requested else 'Teleop disabled')
        return response

    def recover_enabled_gate(self, problems):
        # Intent can persist across faults, but a held deadman can never resume motion.
        now = time.monotonic()
        if now-self.operator_at >= .5:
            self.enable_requested = False
        if not self.enable_requested or self.gate.armed:
            self.recovery_since = None
            return
        stationary = self.joint_velocity is not None and np.max(np.abs(self.joint_velocity)) < .02
        if problems or self.pressed or not stationary:
            self.recovery_since = None
            return
        if self.recovery_since is None:
            self.recovery_since = now
        if now-self.recovery_since >= .5:
            self.arm(None, Trigger.Response())
            self.recovery_since = None

    def disarm(self, request, response):
        self.enable_requested = False
        self.recovery_since = None
        self.trip('Operator disarmed')
        self.gate.reason = 'Operator disarmed'
        response.success, response.message = True, 'Disarmed; use pendant stop for a physical emergency'
        return response

    def trip(self, reason):
        self.core.reset_command()
        self.joint_diagnostics.reset(keep_latest=True)
        if self.gate.armed or self.active_previous:
            self.begin_hold()
            self.core.block(reason + '; release A before recovery')
        # Preserve the first stop cause until explicit rearming or operator disarm.
        # Later optical/state callbacks must not erase the diagnostic evidence.
        self.gate.disarm(reason if self.gate.armed or self.active_previous else self.gate.reason)
        self.active_previous = False

    def begin_hold(self):
        t = time.monotonic()
        r = self.records.get('joints')
        if self.command_pub is not None and r and fresh(*r, t, self.get_clock().now().nanoseconds, self.observation_timeout) and self.joint_values is not None:
            self.hold_q = self.joint_values.copy()
            self.hold_until = t + .20

    def publish_hold(self):
        if self.command_pub is None or self.hold_q is None or time.monotonic() >= self.hold_until:
            return
        msg = JointTrajectory()
        msg.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = self.hold_q.tolist()
        point.velocities = [0.] * 6
        point.time_from_start = Duration(seconds=.10).to_msg()
        msg.points = [point]
        self.command_pub.publish(msg)

    def tick(self):
        self.configure()
        super().tick()
        problems = self.reasons()
        if self.gate.armed and problems:
            self.trip('; '.join(problems))
        self.recover_enabled_gate(problems)
        active = self.gate.armed and not problems and self.core.state == 'ACTIVE' and self.pressed
        now = self.get_clock().now()
        if active and not self.active_previous:
            self.joint_diagnostics.reset()
            self.hold_q, self.hold_until = None, 0.
            self.gate.accept_after_ns = now.nanoseconds
            self.gate.last_command = self.gate.last_command_at = None
            self.gate.last_source_ns = 0
            self.active_since = time.monotonic()
        if self.active_previous and not active:
            self.begin_hold()
            self.active_since = None
        if active and time.monotonic() - (self.gate.last_command_at or self.active_since) > .10:
            self.trip('Servo output timed out; release A before recovery')
            active = False
        self.active_previous = active
        if not active:
            self.core.reset_command()
        msg = TwistStamped()
        msg.header.stamp, msg.header.frame_id = now.to_msg(), self.base
        if active:
            v = self.preview_velocity
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v[:3])
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, v[3:])
        self.twist.publish(msg)
        self.publish_hold()
        self.armed_pub.publish(Bool(data=self.gate.armed))
        self.operator_status.publish(String(data=json.dumps({
            'mode': 'LIVE OUTPUT' if self.motion_output else 'DRY RUN — ROBOT OUTPUT DISABLED',
            'session_id': self.session_id,
            'configuration_ready': self.configuration_ready(),
            'robot_fault': self.safety_mode is not None and self.safety_mode not in (1, 2),
            'configuration_reason': self.configuration_status_reason(),
            'setup_feedback_fresh': bool(self.setup_tf_fresh and self.joints_ok and
                fresh(*self.records.get('joints', (-1e30, 0)), time.monotonic(), now.nanoseconds, self.observation_timeout)),
            'managed_session_id': os.environ.get('UR_TELEOP_MANAGED_SESSION', ''),
            'motion_output_enabled': self.motion_output,
            'command_publisher_present': self.command_pub is not None,
            'setup_checked': math.isfinite(self.configuration_checked),
            'setup_context': self.get_parameter('engineering_context').value,
            'setup_technical': self.configuration.technical if self.configuration else '',
            'setup_controllers_inactive': not self.motion_output and self.controller_ok,
            'tool_frame': self.tool, 'base_frame': self.base,
            'tracking_valid': self.tracking_valid(),
            'robot_preparation_ready': not self.robot_preparation_reason(),
            'robot_preparation_reason': self.robot_preparation_reason(),
            'controller_preparation_allowed': self.controller_preparation_allowed and time.monotonic()-self.controller_at < .6,
            # One complete readback after configure(), in persisted/ROS units.
            'effective_parameters': {key: (list(self.get_parameter(key).value)
                if key == 'tracking_to_robot_rpy_deg' else self.get_parameter(key).value)
                for key in DEFAULTS},
            'alignment_verified': bool(self.get_parameter('alignment_verified').value and
                self.verified_mapping == tuple(self.get_parameter('tracking_to_robot_rpy_deg').value)),
            'optical_loss_test_verified': bool(self.get_parameter('optical_loss_test_verified').value),
            'operator_heartbeat_fresh': time.monotonic()-self.operator_at < .5,
            'controller_state_fresh': time.monotonic()-self.controller_at < .6,
            'servo_started': math.isfinite(self.servo_at),
            'servo_status_fresh': time.monotonic()-self.servo_at < .25,
            'teleop_control_mode': self.core.control_mode,
            'tracking_station_policy': self.station_policy,
            'single_station_motion_scale': 1.,
            'linear_velocity_filter_s': self.core.linear_velocity_filter_s,
            'angular_velocity_filter_s': self.core.angular_velocity_filter_s,
            'optical_stations': self.optical_stations if time.monotonic()-self.optical_detail_at < .20 else {k: 0 for k in self.optical_stations},
            'active_station_count': sum(v > 0 for v in self.optical_stations.values()) if time.monotonic()-self.optical_detail_at < .20 else 0,
            'pose_fresh': self.pose_ok and self.flag and time.monotonic()-self.flag_at < self.observation_timeout and fresh(*self.records.get('pose', (-1e30, 0)), time.monotonic(), now.nanoseconds, self.observation_timeout),
            'tracking_rejection': self.pose_quality.reason,
            'motion_ready': not problems,
            'velocity_diagnostics': {
                'raw_pose': self.hand.tolist() + list(self.hand_q) if np.all(np.isfinite(self.hand)) and np.all(np.isfinite(self.hand_q)) else None,
                'filtered_pose': (self.core.filtered_hand.tolist() + self.core.filtered_rotation.as_quat().tolist()) if self.core.filtered_hand is not None and self.core.filtered_rotation is not None else None,
                'sample_dt_s': self.core.sample_dt if self.core.control_mode == 'velocity' else self.observed_sample_dt,
                'observed_hand_velocity': np.r_[self.core.rotation @ self.observed_hand_velocity[:3], self.core.rotation @ self.observed_hand_velocity[3:]].tolist(),
                'measured_tcp_position': self.measured_tcp.tolist(),
                'measured_tcp_orientation': self.measured_tcp_q,
                'filtered_pose_frame': self.base if self.core.control_mode == 'velocity' else 'lighthouse',
                'raw_hand_velocity': self.core.raw_hand_velocity.tolist() if self.core.control_mode == 'velocity' else np.r_[self.core.rotation @ self.raw_observed_hand_velocity[:3], self.core.rotation @ self.raw_observed_hand_velocity[3:]].tolist(),
                'filtered_hand_velocity': self.core.hand_velocity.tolist() if self.core.control_mode == 'velocity' else np.r_[self.core.rotation @ self.observed_hand_velocity[:3], self.core.rotation @ self.observed_hand_velocity[3:]].tolist(),
                'requested_twist': self.preview_velocity.tolist(),
                'published_twist': self.preview_velocity.tolist() if active else [0.]*6,
                'measured_tcp_velocity': self.measured_tcp_velocity,
                'clamped': self.core.clamped, 'motion_scale': self.motion_scale,
                'command_age_s': max(0., (now.nanoseconds-self.records['pose'][1])/1e9) if 'pose' in self.records else None,
            },
            'measured_joint_velocity': self.joint_velocity.tolist() if self.joint_velocity is not None and self.joints_ok and fresh(*self.records.get('joints', (-1e30, 0)), time.monotonic(), now.nanoseconds, self.observation_timeout) else None,
            'measured_joint_position': self.joint_values.tolist() if self.joint_values is not None and self.joints_ok and fresh(*self.records.get('joints', (-1e30, 0)), time.monotonic(), now.nanoseconds, self.observation_timeout) else None,
            'stationary': self.joint_velocity is not None and bool(np.max(np.abs(self.joint_velocity)) < .02),
            'observations_healthy': self.observation_healthy,
            'robot_ready': self.robot_mode == 7 and self.safety_mode in (1, 2) and self.program_running,
            'controller_ready': self.controller_ok,
            'enable_requested': self.enable_requested,
            'armed': self.gate.armed, 'clutch': self.core.state, 'a_held': self.pressed,
            'optical': self.optical and time.monotonic()-self.optical_at < .20,
            'optical_detail': self.optical_detail, 'servo': self.servo_status,
            'servo_detail': servo_status_detail(self.servo_status),
            'reason': self.gate.reason, 'blockers': problems, 'accepted_targets': self.forwarded,
            'workspace_radius_m': self.workspace_radius, 'observation_timeout_s': self.observation_timeout,
            'joint_speed': self.gate.max_joint_speed, 'tracking_gain': self.core.response_gain,
            'hardware_smoothing': self.hardware_smoothing if time.monotonic()-self.controller_timeout_at < .6 else None,
            'translation_scale': self.core.scale, 'control_period_s': CONTROL_PERIOD_S,
            'rotation_scale': self.core.rotation_scale,
            'vr_translation_deadband_m': self.core.translation_deadband,
            'vr_rotation_deadband_deg': math.degrees(self.core.rotation_deadband),
            'translation_damping_gain': self.core.translation_damping_gain,
            'rotation_damping_gain': self.core.rotation_damping_gain,
            'damping_velocity_filter_s': self.core.damping_velocity_filter_s,
            'damping_tool_velocity': self.core.measured_velocity.tolist(),
            'linear_acceleration': self.core.linear_acceleration,
            'angular_acceleration_deg': math.degrees(self.core.angular_acceleration),
            'max_linear_acceleration_m_s2': self.get_parameter('max_linear_acceleration_m_s2').value,
            'max_angular_acceleration_rad_s2': self.get_parameter('max_angular_acceleration_rad_s2').value,
            'vr_translation_filter_alpha': self.core.translation_filter_alpha,
            'vr_rotation_filter_alpha': self.core.rotation_filter_alpha,
            'joint_acceleration_warning_rad_s2': self.joint_acceleration_warning,
            'joint_diagnostics': self.joint_diagnostics.latest,
            'motion_metrics': getattr(self, 'motion_metrics', None),
            'motion_metrics_current': getattr(self, 'motion_metrics_current', False),
            'alignment_rpy_deg': list(self.get_parameter('tracking_to_robot_rpy_deg').value),
            'linear': self.get_parameter('max_linear_speed_m_s').value, 'angular': self.get_parameter('max_angular_speed_deg_s').value,
        }, allow_nan=False)))

    def on_trajectory(self, msg):
        # Monitoring only, before the existing command gate. Never clamp targets.
        if len(msg.points) == 1:
            estimate = self.joint_diagnostics.observe(list(msg.joint_names), msg.points[0].positions,
                stamp_ns(msg.header.stamp), self.get_clock().now().nanoseconds,
                self.joint_acceleration_warning, self.gate.max_joint_speed)
            t = time.monotonic()
            for joint in estimate.get('warning_joints', []):
                if t-self.last_joint_warning.get(joint, -float('inf')) >= .5:
                    i = estimate['joint_names'].index(joint)
                    acceleration = estimate['ddq_rad_s2']
                    self.get_logger().warn(f"Servo target diagnostic {joint}: dq={estimate['dq_rad_s'][i]:.4f} rad/s "
                        f"ddq={acceleration[i] if acceleration is not None else 'pending'} rad/s² "
                        f"dt={estimate['dt_s']:.6f}s (command estimate, not measured acceleration)")
                    self.last_joint_warning[joint] = t
        else:
            self.joint_diagnostics.reset()
        if not self.active_previous or not self.gate.armed:
            return
        problems = self.reasons()
        if problems or not self.pressed:
            self.trip('; '.join(problems) or 'A released at command receipt')
            return
        try:
            if len(msg.points) != 1:
                raise ValueError('Expected a single short Servo target')
            p = msg.points[0]
            # C++ relay preserves DDS publisher time when Servo uses stamp=0.
            # A missing stamp here remains an error, never replaced by receipt time.
            source_ns = stamp_ns(msg.header.stamp)
            if 0 < source_ns <= self.gate.accept_after_ns:
                return  # Drain pre-clutch samples without forwarding them.
            self.gate.native_smoothing = bool(self.motion_output and self.smoothing_controller_present
                and self.hardware_smoothing and self.controller_ok
                and time.monotonic()-self.controller_timeout_at < .6
                and time.monotonic()-self.controller_at < .6)
            q = self.gate.validate(list(msg.joint_names), p.positions, p.velocities, p.accelerations, p.effort,
                p.time_from_start.sec + p.time_from_start.nanosec/1e9,
                source_ns, self.get_clock().now().nanoseconds,
                self.joint_values, time.monotonic())
            self.forwarded += 1
            if self.command_pub is not None:
                # Preserve the validated Servo horizon. Stretching each 20 ms
                # target to 100 ms attenuates the stream when JTC replans from
                # measured positions on every incoming target.
                out = JointTrajectory()
                out.joint_names = list(JOINTS)
                point = JointTrajectoryPoint()
                # Preserve genuine Servo velocities (including its zero-input
                # stop). Never invent endpoint zero velocities for moving points.
                point.positions = q.tolist()
                if p.velocities:
                    point.velocities = [p.velocities[list(msg.joint_names).index(j)] for j in JOINTS]
                point.time_from_start = p.time_from_start
                out.points = [point]
                self.command_pub.publish(out)
        except (ValueError, IndexError, TypeError) as exc:
            self.trip(str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Commissioning()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.trip('Node exiting')
                node.publish_hold()
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
