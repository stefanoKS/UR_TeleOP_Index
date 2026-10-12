#!/usr/bin/env bash
set -e
# GUI only: never starts the robot, switches controllers or enables motion.
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/humble/setup.bash
source /home/ubuntu-ros/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=43
exec /home/ubuntu-ros/Documents/Teleop/ur_teleop_ws/install/ur_vr_teleop/lib/ur_vr_teleop/gui
