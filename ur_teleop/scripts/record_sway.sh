#!/usr/bin/env bash
# Read-only recorder; no enable, controller switch or motion publisher.
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$root/ur_teleop_ws/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
exec /usr/bin/python3 -m ur_vr_teleop.sway_recording \
  --output-root "$root/ur_teleop/diagnostics" --config "$root/ur_teleop/config/real_teleop.yaml" "$@"
