# UR Teleop

Local project root: `~/Documents/Teleop` (Ubuntu, ROS 2 Humble).

## What to put in GitHub

Use this directory as the repository root. Commit `.gitignore`,
`dependencies.repos`, this README, `ur_teleop_ws/src/`, and the configuration,
scripts, startup scripts and Markdown documentation in `ur_teleop/`.
Also include `ur_calibration_fix_ws/PROJECT_STATUS.md`.
The ignore file excludes generated builds, logs, recordings, backups and
upstream checkouts. It applies when using Git, not manual browser uploads.

The two upstream repositories were unmodified at migration time. Their exact
revisions are recorded in `dependencies.repos`; fetch them with
`vcs import . < dependencies.repos` from this directory (requires vcstool).

## Rebuild

The existing system dependencies, including ROS 2 Humble, librealsense2 and
libsurvive development dependencies, must be installed.

```bash
cd ~/Documents/Teleop
cmake -S ur_teleop/tools/libsurvive -B ur_teleop/tools/libsurvive/bin \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_SINGLE_PRECISION=OFF \
  -DUSE_HIDAPI=OFF -DUSE_EIGEN=ON -DBUILD_GATT_SUPPORT=OFF
cmake --build ur_teleop/tools/libsurvive/bin -j 4
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
cd ur_calibration_fix_ws
colcon build --base-paths src/Universal_Robots_ROS2_Driver/ur_calibration \
  --packages-select ur_calibration --allow-overriding ur_calibration \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
cd ../ur_teleop_ws
colcon build --base-paths src --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

`ur_teleop/` contains runtime configuration and launch scripts. Its old duplicate
ROS build was archived during migration; the active ROS packages are all built
in `ur_teleop_ws/`. The calibration workspace intentionally builds only the
calibration package; the remaining standard UR driver packages come from the
installed ROS distribution, as before.

Launch the desktop using `~/Documents/Teleop/ur_teleop/start_teleop.sh`.
Current configuration retains this machine's robot settings and calibration.
The configured absolute paths assume user `ubuntu-ros`; adjust these when
cloning on a different account or machine.
