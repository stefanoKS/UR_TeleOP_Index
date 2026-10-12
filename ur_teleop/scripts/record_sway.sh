#!/usr/bin/env bash
# Records observations only; never activates, enables or moves the robot.
set -e
source /opt/ros/humble/setup.bash
source /home/ubuntu-ros/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=43
output="/home/ubuntu-ros/Documents/Teleop/ur_teleop/diagnostics/sway_$(date +%Y%m%d_%H%M%S)"
exec ros2 bag record -o "$output" \
  /vr/controller/source_time /vr/controller/pose /vr/controller/joy /vr/controller/pose_fresh \
  /vr/controller/optical_fresh /vr/controller/optical_status \
  /vr_real/preview_markers /vr_real/preview_state \
  /vr_real/servo_node/delta_twist_cmds /vr_real/servo_node/status \
  /vr_real/servo_unstamped_targets /vr_real/servo_joint_targets \
  /vr_real/armed /vr_real/operator_status /parameter_events \
  /scaled_joint_trajectory_controller/joint_trajectory \
  /scaled_joint_trajectory_controller/controller_state \
  /scaled_joint_trajectory_controller/smoothed_command \
  /scaled_joint_trajectory_controller/control_diagnostics \
  /joint_states /tf /tf_static \
  /speed_scaling_state_broadcaster/speed_scaling \
  /io_and_status_controller/robot_program_running \
  /io_and_status_controller/robot_mode /io_and_status_controller/safety_mode
