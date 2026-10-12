# Installation

Supported target: **Ubuntu 22.04 (Jammy), ROS 2 Humble, system Python 3.10**. Commands assume Bash and checkout `~/Documents/Teleop`. This is a source-checked recipe, not a claim that a fresh second-machine installation was executed. No single bootstrap installer exists. Do not launch physical motion as an installation check.

## 1. Ubuntu and ROS

Use the [official Humble Ubuntu installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) for repository changes. The current official source recipe is also available in the [ROS documentation repository](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Installation/_Apt-Repositories.rst).

```bash
sudo apt update
sudo apt upgrade
sudo apt install locales software-properties-common curl git ca-certificates gnupg
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
ros_source_version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_source_version}/ros2-apt-source_${ros_source_version}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools
source /opt/ros/humble/setup.bash
```

Use a terminal without an active Conda/virtual environment. Builds explicitly select `/usr/bin/python3`; a different `python3` on PATH can otherwise contaminate ROS imports.

## 2. Checkout and pinned upstream dependencies

```bash
mkdir -p ~/Documents
cd ~/Documents
git clone https://github.com/stefanoKS/UR_TeleOP_Index.git Teleop
cd Teleop
sudo apt install python3-vcstool python3-rosdep python3-colcon-common-extensions
vcs import . < dependencies.repos
git -C ur_teleop/tools/libsurvive submodule update --init --recursive
```

[dependencies.repos](../dependencies.repos) pins libsurvive at `df22f2d6be34cb54d43c46e8838eae8b326b88ff` and the calibration-driver checkout at `b8e342428839ca8bcc4a49caf25633a712f9cd2f`. Do not silently substitute upstream HEAD. The checkouts are ignored by this repository.

## 3. System libraries

```bash
sudo apt install build-essential cmake pkg-config zlib1g-dev libx11-dev \
  libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev \
  libatlas-base-dev libeigen3-dev python3-numpy python3-scipy python3-yaml \
  python3-tk python3-pil python3-pil.imagetk python3-pytest \
  ros-humble-ur ros-humble-moveit-servo ros-humble-ruckig
```

Install the RealSense SDK from its [official Debian-package instructions](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md). Current repository setup:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
  gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install librealsense2-dev librealsense2-utils
pkg-config --modversion realsense2
```

`ur_d405_camera` needs librealsense2 at build time even with `camera.enabled: false`. It uses a native capture helper; `pyrealsense2` is not required. Kernel/USB support for a physical camera must be checked against the SDK's supported kernel guidance; this recipe does not install a patched kernel automatically.

Initialize rosdep once on a fresh system, then resolve declared ROS dependencies:

```bash
sudo rosdep init
rosdep update
cd ~/Documents/Teleop
rosdep install --from-paths ur_teleop_ws/src \
  ur_calibration_fix_ws/src/Universal_Robots_ROS2_Driver/ur_calibration \
  --ignore-src --rosdistro humble -r -y
```

If rosdep is already initialized, omit `sudo rosdep init`. Local verified versions at documentation time include ROS UR 2.14.0, Ruckig 0.9.2, and librealsense2 2.56.5. These observations are not an apt lockfile; revalidate when system packages change.

## 4. Build

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
source install/local_setup.bash
```

The calibration workspace intentionally builds only `ur_calibration`; other UR packages come from apt. The desktop sources Humble and the main workspace, not the calibration overlay. Source the calibration overlay separately when doing a deliberate calibration workflow.

The bridge reads libsurvive's CMake cache to match float precision. Changing `USE_SINGLE_PRECISION` requires rebuilding the bridge against the matching library. Project C++ components use C++17. Python packages use ament_python; the bridge, native controllers, and camera use ament_cmake.

## 5. Lighthouse and controller

Install upstream USB access rules, then reconnect the receiver/controller if required:

```bash
cd ~/Documents/Teleop
sudo cp ur_teleop/tools/libsurvive/useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The controller must be paired to its receiver and calibrated through libsurvive. This repository does not provision pairing or contain a portable station calibration. Preserve `~/.config/libsurvive/config.json` before recalibration; its format is not necessarily strict JSON. Moving a station or replacing its calibration may change tracking coordinates and invalidate the verified robot mapping.

The inspected local calibration uses the MPFIT poser and calibrated Gen2 station records. Neither this bridge nor the inspected solver forces two physical stations. A one-station solution still needs adequate sensor geometry. Validate each available station separately with robot motion disabled; do not reduce solver measurement thresholds to match the optical presence threshold.

Managed tracking requests serial `LHR-4632363E` in `desktop.py`. Standalone bridge default selection is `KN0` with no requested serial. Review actual device identity when moving installations. Do not run SteamVR/another tracking process that competes for the receiver.

## 6. Robot and machine-specific setup

The [UR External Control setup guide](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html) describes installing the URCap and adding its program node. Configure the workstation's reachable robot-network address in the pendant. The exact installed PolyScope/URCap versions were not established here; check compatibility for your robot before commissioning.

This checkout retains one machine's configuration:

| Item | Current value / required review |
|---|---|
| Robot IP | `192.168.1.251`; workstation IP is not specified in source |
| UR driver | `ur_type:=ur10e`, `headless_mode:=false`, script sender port `50002` |
| Calibration path | `/home/ubuntu-ros/Documents/Teleop/ur_teleop/config/ur10e_calibration.yaml` |
| Calibration check | Launch requires `calib_2528998354424493788` |
| Tracking serial | `LHR-4632363E` in managed launcher |
| D405 serial | `230322276078` |
| Active tool model | `tool0`; verify any physical tool/TCP separately |
| Operator overrides | Adjacent `operator_settings.json` changes the YAML limits and mapping |

Use an appropriate dedicated/reachable network and check both directions of the UR driver connection. Opening every firewall port or copying an invented workstation IP is not a setup procedure. The launch is deliberately tied to this robot calibration; adapting to another UR10e requires reviewed calibration changes, not bypassing the hash check.

`start_teleop.sh` follows `$HOME/Documents/Teleop`; `start_gui.sh`, calibration configuration, and some tests contain the original absolute account path. Audit relocation before launch. The existing desktop shortcut is a workstation artifact, not a tracked installer. Use the script if no shortcut exists.

For driver FIFO scheduling, first inspect permissions:

```bash
~/Documents/Teleop/ur_teleop/scripts/setup_realtime_permissions.sh --check
```

If appropriate for the host, the existing administrative helper installs per-user scheduling limits:

```bash
sudo ~/Documents/Teleop/ur_teleop/scripts/setup_realtime_permissions.sh
```

It backs up existing files, does not restart ROS or the robot, and requires a full logout/login or reboot to affect the desktop session. It does not install a real-time kernel or prove timing performance.

## 7. Verify without robot motion

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
ros2 pkg executables ur_vr_bridge
ros2 pkg executables ur_vr_teleop
ros2 pkg executables ur_d405_camera
ros2 launch ur_vr_teleop commissioning.launch.py --show-args
ros2 launch ur_vr_sim_teleop simulation.launch.py --show-args
```

Expected entry points: bridge `controller_bridge`, `servo_stamp_relay`; teleop `preview`, `commissioning`, `gui`, `desktop`; camera `d405_camera`, `d405_capture`. Run the [automated tests](COMMISSIONING.md#automated-validation) before operator commissioning. `--show-args` inspects launch arguments; it is not a live-stack test.

A clean build validates compilation and packaging. It does not establish tracking calibration, robot network readiness, safe physical stopping, or a fresh-machine GUI installation. Continue with [Commissioning](COMMISSIONING.md).
