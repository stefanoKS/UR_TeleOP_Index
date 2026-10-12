#!/usr/bin/env bash
set -e
export PATH=/usr/bin:/bin:$PATH
source /opt/ros/humble/setup.bash
source "$HOME/Documents/Teleop/ur_teleop_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=43
exec /usr/bin/python3 "$HOME/Documents/Teleop/ur_teleop_ws/install/ur_vr_teleop/lib/ur_vr_teleop/desktop" "$@"
