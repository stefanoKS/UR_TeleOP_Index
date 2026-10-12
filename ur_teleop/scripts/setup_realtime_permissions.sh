#!/usr/bin/env bash
# Enable UR driver's FIFO scheduling for this desktop user only.
# Does not start/restart ROS, change the kernel, or start the robot.
set -euo pipefail
if [[ ${1:-} == --check ]]; then
  uname -r
  /usr/bin/python3 - <<'PY'
import resource
soft, hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
print(f'Current login RT priority: soft={soft}, hard={hard}; UR control thread requests 50')
print('Permissions available' if soft >= 50 else 'Permissions unavailable in this login')
PY
  exit 0
fi
if [[ $EUID -ne 0 || -z ${SUDO_USER:-} || $SUDO_USER == root ]]; then
  echo 'Run: sudo ~/Documents/Teleop/ur_teleop/scripts/setup_realtime_permissions.sh' >&2
  exit 1
fi
teleop_user=$SUDO_USER
teleop_uid=$(id -u "$teleop_user")
[[ $teleop_uid -ge 1000 && $teleop_user =~ ^[a-z_][a-z0-9_-]*\$?$ ]] || exit 1
limits_file=/etc/security/limits.d/90-ur-teleop-rt.conf
service_dir="/etc/systemd/system/user@${teleop_uid}.service.d"
service_file="$service_dir/90-ur-teleop-rt.conf"
stamp=$(date +%Y%m%d_%H%M%S)
install -d -m 0755 /etc/security/limits.d "$service_dir"
for file in "$limits_file" "$service_file"; do
  if [[ -e $file ]]; then cp -a -- "$file" "$file.backup.$stamp"; fi
done
printf '# UR teleop: FIFO scheduling permission for %s\n%s soft rtprio 99\n%s hard rtprio 99\n' \
  "$teleop_user" "$teleop_user" "$teleop_user" > "$limits_file"
printf '[Service]\nLimitRTPRIO=99\n' > "$service_file"
chmod 0644 "$limits_file" "$service_file"
systemctl daemon-reload
echo "Installed scheduling permissions for $teleop_user (UID $teleop_uid)."
echo 'Log out of the desktop completely and log back in, or reboot when convenient.'
echo 'Then run: ~/Documents/Teleop/ur_teleop/scripts/setup_realtime_permissions.sh --check'
echo 'No running user session, ROS process, robot, or kernel was restarted.'
