import os
from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ur_vr_sim_teleop.core import require_mock_urdf


def launch_nodes(context):
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise RuntimeError('Set ROS_DOMAIN_ID=42 in every simulation terminal')
    here = Path(get_package_share_directory('ur_vr_sim_teleop'))
    description = Path(get_package_share_directory('ur_description'))
    moveit = Path(get_package_share_directory('ur_moveit_config'))
    xml = xacro.process_file(str(description / 'urdf/ur.urdf.xacro'), mappings={
        'name': 'ur10e', 'ur_type': 'ur10e', 'tf_prefix': 'sim_',
        'use_fake_hardware': 'true', 'robot_ip': '127.0.0.1',
        'initial_positions_file': str(here / 'config/initial_positions.yaml'),
    }).toxml()
    require_mock_urdf(xml)
    semantic = xacro.process_file(str(moveit / 'srdf/ur.srdf.xacro'),
                                  mappings={'name': 'ur10e', 'prefix': 'sim_'}).toxml()
    model = {'robot_description': xml, 'robot_description_semantic': semantic, 'use_sim_time': False}
    controllers = str(here / 'config/controllers.yaml')
    servo = yaml.safe_load((moveit / 'config/ur_servo.yaml').read_text())
    servo.update({
        'command_in_type': 'speed_units', 'publish_period': .01,
        'command_out_type': 'std_msgs/Float64MultiArray',
        'publish_joint_positions': True, 'publish_joint_velocities': False,
        'publish_joint_accelerations': False,
        'is_primary_planning_scene_monitor': True,
        'move_group_name': 'sim_ur10e_manipulator', 'planning_frame': 'sim_base_link',
        'ee_frame_name': 'sim_tool0', 'robot_link_command_frame': 'sim_base_link',
        'joint_topic': '/vr_sim/joint_states',
        'cartesian_command_in_topic': '/vr_sim/servo_node/delta_twist_cmds',
        'joint_command_in_topic': '/vr_sim/servo_node/delta_joint_cmds',
        'command_out_topic': '/vr_sim/forward_position_controller/commands',
        'status_topic': '/vr_sim/servo_node/status',
        'incoming_command_timeout': .10, 'num_outgoing_halt_msgs_to_publish': 10,
        'check_collisions': True, 'collision_check_rate': 20.,
    })
    yaw = float(LaunchConfiguration('axis_yaw_deg').perform(context))
    import math
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='vr_sim', parameters=[{'robot_description': xml, 'use_sim_time': False}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node', namespace='vr_sim',
             parameters=[{'robot_description': xml}, controllers], output='screen'),
        Node(package='controller_manager', executable='spawner', namespace='vr_sim',
             arguments=['joint_state_broadcaster', 'forward_position_controller',
                        '-c', '/vr_sim/controller_manager', '--controller-manager-timeout', '60',
                        '--param-file', controllers], output='screen'),
        Node(package='moveit_servo', executable='servo_node_main', name='servo_node', namespace='vr_sim',
             parameters=[model, {'moveit_servo': servo}], output='screen'),
        Node(package='ur_vr_sim_teleop', executable='teleop', name='teleop', namespace='vr_sim',
             parameters=[{'robot_description': xml, 'enable_motion': False,
                          'translation_scale': float(LaunchConfiguration('translation_scale').perform(context)),
                          'clutch_button': 4, 'axis_map': [c,-s,0.,s,c,0.,0.,0.,1.]}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='sim_rviz', namespace='vr_sim',
             arguments=['-d', str(here / 'rviz/simulation.rviz')], parameters=[model]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('translation_scale', default_value='0.8',
                              description='Hand displacement to target displacement scale'),
        DeclareLaunchArgument('axis_yaw_deg', default_value='0.0',
                              description='Preview rotation of controller XY displacement into simulated base axes'),
        OpaqueFunction(function=launch_nodes),
    ])
