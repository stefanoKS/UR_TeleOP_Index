# UR10e Valve Index teleoperation

This project maps a Valve Index controller's translation and rotation to a Universal Robots UR10e through ROS 2 Humble. A desktop panel manages tracking, robot connection, commissioning checks, and an A-button clutch. It is an engineering teleoperation application with software safeguards; deployment still requires supervised hardware commissioning.

The current implementation supports **either one or two calibrated Lighthouse stations**. One recent accepted paired optical sensor observation establishes signal presence. Motion separately requires a fresh, valid, continuous 6-DoF pose. Single-station operation needs no experimental selection and applies no extra speed multiplier. Cold-start tracking with one physical station still needs installation-specific verification.

## Capabilities and status

- Velocity mode (normal default): hand motion produces bounded Cartesian velocity; stationary hand motion produces zero demand.
- Position mode: a clutch-relative pose target is followed with bounded Cartesian velocity.
- Translation and rotation mapping, pose filters, spatial deadbands, configurable speed and acceleration limits.
- Hold A to command; release to stop. Tracking faults invalidate the clutch and require release and a new press.
- MoveIt Servo, collision/singularity monitoring, and a custom Ruckig controller upstream of the UR driver's position interface.
- A single-window operator dashboard with persistent tuning, session diagnostics, RViz, and an optional D405 wrist-camera display.
- Driver-connected Preview and a separate hardware-free mock simulator. These exercise different control paths.

**Review active settings before commissioning.** The committed YAML specifies 0.10 m/s linear speed; the committed saved operator profile overrides it to **1.5 m/s**. Angular speed is similarly overridden from 30 to **180 deg/s**, joint speed from 0.5 to **2 rad/s**, and alignment yaw from −90° to **−40°**. These are existing settings, not recommended starting limits. See [configuration precedence](docs/CONFIGURATION.md#configuration-precedence).

## System architecture

```mermaid
flowchart LR
    Index[Valve Index controller] --> Survive[libsurvive]
    Survive --> Bridge[vr_controller_bridge]
    Bridge --> Teleop[vr_real teleop and Gate]
    Teleop --> Servo[MoveIt Servo]
    Servo --> Relay[servo_stamp_relay]
    Relay --> Teleop
    Teleop --> Smooth[SmoothScaledJointTrajectoryController]
    Smooth --> Driver[UR driver position interface]
    Driver --> Robot[UR10e]
    Robot --> Feedback[Joint states and TF]
    Feedback --> Teleop
    Feedback --> Servo
    GUI[Operator GUI and heartbeat] --> Teleop
    Safety[Robot state and controller supervision] --> Teleop
```

The Cartesian command goes through Servo; the resulting joint trajectory returns through a timestamp relay and the gate before reaching the final controller. There is no direct `speedl`/`speedj` teleoperation path. [Architecture and ROS interfaces](docs/ARCHITECTURE.md) describes responsibilities and stop behavior.

## Requirements and installation

The supported environment is Ubuntu 22.04 with ROS 2 Humble and system Python 3.10. Hardware consists of the configured UR10e, a paired Valve Index controller/receiver, at least one calibrated Lighthouse, and optionally a RealSense D405. The robot needs External Control configured for the workstation's reachable IP. The committed robot address is `192.168.1.251`; the workstation address is site-specific.

Dependencies include the ROS UR driver, MoveIt Servo, ros2_control, Ruckig, libsurvive, NumPy/SciPy/YAML, Tk/Pillow, and the librealsense2 development SDK. The camera is optional at runtime, but its package requires the SDK to build. The project contains machine-specific serials, paths, and a robot calibration-hash check.

Follow [Installation](docs/INSTALLATION.md) for complete fresh-system commands, pinned upstream dependencies, build instructions, and verification. Start the checkout with:

```bash
mkdir -p ~/Documents
cd ~/Documents
git clone https://github.com/stefanoKS/UR_TeleOP_Index.git Teleop
cd Teleop
```

Do not run the application until dependencies are built and the hardware-specific configuration has been reviewed. A fresh installation on a second machine has not been reproduced as part of this documentation work.

## Quick start

After installation, use the existing **UR10e Teleop** desktop shortcut, if installed on this workstation, or:

```bash
~/Documents/Teleop/ur_teleop/start_teleop.sh
```

1. The managed desktop starts **Preview** by default. It connects the driver and starts tracking, but the teleop node cannot publish robot motion commands. This is not hardware-free simulation.
2. With A released and motion controllers inactive, check pose freshness, optical tracking, measured joints, `tool0`, and all three mapped movement directions. Complete [commissioning](docs/COMMISSIONING.md) before live operation.
3. Select position or velocity mode and review effective limits in Quick tuning; advanced settings remain in the same window. Apply changes while disarmed with A released. Joint-speed changes require a stack restart.
4. For an authorized physical session, select **Live setup**. Run the prepared External Control program on the pendant. Use **Start Servo solver** and **Prepare controller** as indicated by the panel, then complete the session confirmations.
5. Select **Enable teleop** only when readiness checks pass. Release A, then press and hold it to establish a new reference and command motion.
6. Release A for a normal stop; use **DISARM TELEOP** to clear enable intent. Use the pendant emergency stop for an emergency. **Stop ROS** shuts down the managed stack; close the desktop afterward.

For tracking alone, without starting the driver or teleoperation:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=43
ros2 run ur_vr_bridge controller_bridge
```

Do not start a second bridge alongside the managed desktop. `start_gui.sh` opens only the panel and does not manage the stack. See [Operation](docs/OPERATION.md).

## Testing

With no production stack running, use an isolated local ROS domain:

```bash
cd ~/Documents/Teleop
source /opt/ros/humble/setup.bash
source ur_teleop_ws/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=147
/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q
/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py -q
ctest --test-dir ur_teleop_ws/build/ur_vr_bridge --output-on-failure
ctest --test-dir ur_teleop_ws/build/ur_teleop_controllers --output-on-failure
```

For the separate mock simulator:

```bash
ROS_DOMAIN_ID=42 ros2 launch ur_vr_sim_teleop simulation.launch.py
```

It uses `mock_components/GenericSystem`, prefixed simulated joints, and starts with motion disabled. It does not reproduce the production Ruckig/gate chain. [Commissioning](docs/COMMISSIONING.md#automated-validation) lists additional mock and Servo checks and explains their limits.

## Troubleshooting

| Symptom | Start here |
|---|---|
| One station, no pose, stale/frozen tracking | [Tracking](docs/TROUBLESHOOTING.md#tracking) |
| A or Enable does not produce motion | [Readiness](docs/TROUBLESHOOTING.md#readiness-and-stops) |
| Wrong speed, lag, sway | [Settings and motion](docs/TROUBLESHOOTING.md#settings-and-motion) |
| Launch/build/ROS discovery failure | [Installation and processes](docs/TROUBLESHOOTING.md#installation-and-processes) |

## Repository map

| Path | Purpose |
|---|---|
| `ur_teleop_ws/src/ur_vr_bridge/` | libsurvive bridge, optical/pose guards, timestamp relay |
| `ur_teleop_ws/src/ur_vr_teleop/` | Cartesian control, gate, GUI, desktop supervisor, diagnostics |
| `ur_teleop_ws/src/ur_teleop_controllers/` | Native Ruckig/UR scaled controller plugin |
| `ur_teleop_ws/src/ur_vr_sim_teleop/` | Separate legacy mock simulation |
| `ur_teleop_ws/src/ur_d405_camera/` | Isolated camera capture helper and ROS publisher |
| `ur_teleop/config/` | Robot calibration, YAML defaults, saved operator preferences |
| `ur_teleop/scripts/` | Recording, analysis, real-time permission helper |
| `dependencies.repos` | Exact libsurvive and calibration-driver revisions |
| `ur_calibration_fix_ws/` | Calibration-only overlay and historical status notes |

The active ROS workspace is `ur_teleop_ws`; the old duplicate runtime build was archived during migration. Standard UR packages come from ROS packages; only `ur_calibration` is built in the calibration overlay. Generated builds, logs, recordings, backups, and upstream checkouts are ignored by Git. Keep source, runtime configuration, scripts, and dependency pins under version control; ignore rules do not apply to manual browser uploads.

## Documentation

- [Architecture](docs/ARCHITECTURE.md), [Installation](docs/INSTALLATION.md), [Operation](docs/OPERATION.md)
- [Configuration](docs/CONFIGURATION.md), [Commissioning](docs/COMMISSIONING.md), [Troubleshooting](docs/TROUBLESHOOTING.md)
- [AI agent engineering rules](AGENTS.md)

Historical design/session records are retained: [V2 notes](ur_teleop/TELEOP_V2.md), [teleop update](ur_teleop/TELEOP_UPDATE_20260924.md), [damping](ur_teleop/DAMPING_UPDATE_20260924.md), [state and camera](ur_teleop/DIRECT_STATE_AND_D405_20260924.md), [joint smoothing](ur_teleop/FINAL_COMMAND_SMOOTHING.md), [motion review](ur_teleop/SMOOTH_MOTION_REVIEW.md), and [old project status](ur_calibration_fix_ws/PROJECT_STATUS.md). They describe earlier states and may contain obsolete paths, two-station requirements, or limits. Current source and the guides above take precedence.
