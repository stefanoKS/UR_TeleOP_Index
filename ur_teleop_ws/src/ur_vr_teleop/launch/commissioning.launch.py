"""Start Servo, timestamp relay and gate; never start a driver or activate a controller."""
from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ur_vr_teleop.tracking import CONTROL_PERIOD_S


def nodes(context):
    import os
    if os.environ.get('ROS_DOMAIN_ID') != '43':
        raise RuntimeError('Commissioning requires ROS_DOMAIN_ID=43')
    config = Path(LaunchConfiguration('config_file').perform(context)).expanduser()
    from ur_vr_teleop.preferences import load_settings_with_sources, installation_root
    settings, sources = load_settings_with_sources(config)
    for key, source in sources.items():
        value = settings['robot'][key[6:]] if key.startswith('robot.') else settings['teleop'][key]
        print(f'EFFECTIVE {key}={value} [source: {source}]', flush=True)
    calibration = Path(settings['robot']['calibration_file']).expanduser()
    data = yaml.safe_load(calibration.read_text())
    if data.get('kinematics', {}).get('hash') != 'calib_2528998354424493788':
        raise RuntimeError('Unexpected calibration hash; review robot/config before use')
    description = Path(get_package_share_directory('ur_description'))
    moveit = Path(get_package_share_directory('ur_moveit_config'))
    xml = xacro.process_file(str(description / 'urdf/ur.urdf.xacro'), mappings={
        'name': 'ur10e', 'ur_type': 'ur10e', 'tf_prefix': '',
        'robot_ip': settings['robot']['ip'], 'kinematics_params': str(calibration),
        'safety_limits': 'true', 'safety_pos_margin': '0.15', 'safety_k_position': '20',
    }).toxml()
    semantic = xacro.process_file(str(moveit / 'srdf/ur.srdf.xacro'),
                                  mappings={'name': 'ur10e', 'prefix': ''}).toxml()
    servo = yaml.safe_load((moveit / 'config/ur_servo.yaml').read_text())
    servo.update({
        'command_in_type': 'speed_units', 'publish_period': CONTROL_PERIOD_S,
        'system_latency_compensation': .04,
        'command_out_type': 'trajectory_msgs/JointTrajectory',
        'publish_joint_positions': True, 'publish_joint_velocities': True,
        'publish_joint_accelerations': False, 'use_gazebo': False,
        'is_primary_planning_scene_monitor': True,
        'move_group_name': 'ur10e_manipulator', 'planning_frame': 'base_link',
        'ee_frame_name': 'tool0', 'robot_link_command_frame': 'base_link',
        'joint_topic': '/joint_states',
        'cartesian_command_in_topic': '/vr_real/servo_node/delta_twist_cmds',
        'joint_command_in_topic': '/vr_real/servo_node/delta_joint_cmds',
        'command_out_topic': '/vr_real/servo_unstamped_targets',
        'status_topic': '/vr_real/servo_node/status',
        'incoming_command_timeout': .10, 'num_outgoing_halt_msgs_to_publish': 10,
        'check_collisions': True, 'collision_check_rate': 20.,
        'lower_singularity_threshold': 17., 'hard_stop_singularity_threshold': 30.,
    })
    motion = LaunchConfiguration('motion_output_enabled').perform(context)
    if motion not in ('true', 'false'):
        raise RuntimeError('motion_output_enabled must be true or false')
    # Forward the same effective saved mode/policy/tuning snapshot as the GUI.
    from ur_vr_teleop.preferences import DEFAULTS, validate
    validate({key: settings['teleop'][key] for key in DEFAULTS})
    teleop = settings['teleop'].copy()
    teleop['motion_output_enabled'] = motion == 'true'
    from ur_vr_teleop.engineering import context as engineering_context
    from launch_ros.parameter_descriptions import ParameterValue
    engineering_session = LaunchConfiguration('engineering_session', default='false').perform(context)
    if engineering_session not in ('true', 'false'):
        raise RuntimeError('engineering_session must be true or false')
    # engineering_session is accepted for CLI compatibility; it grants no bypass.
    settings['effective_sources'] = sources
    teleop['engineering_context'] = ParameterValue(engineering_context(settings,
        settings.get('engineering_installation', str(installation_root(config))),
        settings.get('engineering_record_path', str(config.resolve().with_name('engineering_record.json')))), value_type=str)
    return [
        Node(package='moveit_servo', executable='servo_node_main', name='servo_node', namespace='vr_real',
             parameters=[{'robot_description': xml, 'robot_description_semantic': semantic,
                          'use_sim_time': False}, {'moveit_servo': servo}], output='screen'),
        Node(package='ur_vr_teleop', executable='commissioning', namespace='vr_real',
             parameters=[teleop], output='screen'),
        Node(package='ur_vr_bridge', executable='servo_stamp_relay', namespace='vr_real', output='screen'),
        Node(package='rviz2', executable='rviz2', name='teleop_rviz',
             arguments=['-d', str(Path(get_package_share_directory('ur_vr_teleop')) / 'rviz/commissioning.rviz')],
             condition=IfCondition(LaunchConfiguration('launch_rviz', default='true')),
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(Path.home() / 'Documents/Teleop/ur_teleop/config/real_teleop.yaml')),
        DeclareLaunchArgument('motion_output_enabled', default_value='false'),
        DeclareLaunchArgument('engineering_session', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        OpaqueFunction(function=nodes),
    ])
