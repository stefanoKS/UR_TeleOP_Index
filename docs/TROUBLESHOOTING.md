# Troubleshooting

Start with the panel's blocking reason and the current session logs. Release A and disarm before diagnosis that changes settings. The commands below observe state; they do not command motion. Source Humble and the workspace, and use domain 43 for a managed real session:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=43
```

## Tracking

| Symptom | Likely cause | Diagnostic command | Corrective action |
|---|---|---|---|
| Only one Lighthouse detected | Other station absent/occluded; not itself a fault | `ros2 topic echo /vr/controller/optical_status --once` | Either calibrated station is supported. Check valid pose next; do not select an experimental policy or add a 0.25 scale. |
| Optical signal but no valid pose | Insufficient geometry, calibration, bad/stale pose | `ros2 topic echo /vr/controller/pose --once` and `ros2 topic echo /vr_real/operator_status --once` | Improve visibility and inspect calibration. One accepted pair proves signal only; preserve pose checks. |
| Pose stale or frozen | Controller/receiver issue, blocked geometry, solver stopped, repeated samples | `ros2 topic hz /vr/controller/pose` and `ros2 topic echo /vr/controller/source_time --once` | Compare increasing timestamps and changing pose during gentle tracking-only motion. Reconnect/recover tracking with robot motion disabled; release A before recovery. |
| Calibration shifted or wrong axes | Station moved, libsurvive calibration changed, saved RPY override | `ros2 param get /vr_real/teleop tracking_to_robot_rpy_deg` | Preserve/review libsurvive config and robot calibration; repeat all-axis Preview verification. Do not repair a frame error with speed tuning. |
| No controller found | Serial mismatch, pairing, USB rules, competing tracking process | `ros2 topic echo /vr/controller/device --once` | Review managed serial and bridge log, receiver pairing, upstream udev rules, and device ownership. |
| One station works after loss but not cold start | Solver initialization/calibration geometry | Inspect bridge/libsurvive log and `~/.config/libsurvive/config.json` | Validate cold start separately with that station. Do not lower solver measurement requirements to the optical-presence threshold. |

A one-shot echo can wait indefinitely if its topic is absent; use Ctrl+C. A high publication rate does not prove fresh source samples or correct coordinates.

## Readiness and stops

| Symptom | Likely cause | Diagnostic command | Corrective action |
|---|---|---|---|
| A does not engage | Held through fault, not enabled, invalid Joy, invalid configuration | `ros2 topic echo /vr/controller/joy --once` and `ros2 topic echo /vr_real/operator_status --once` | Release A, resolve the displayed cause, wait for healthy readiness, then press again. A is buttons[4]. |
| GUI says motion not ready | Tracking/TF/joints, heartbeat, program, speed scale, Servo, controller, or watchdog check | `ros2 topic echo /vr_real/operator_status --once` | Address the named condition; do not fabricate healthy state messages. |
| Scaled controller inactive | Startup intentionally leaves it inactive | `ros2 control list_controllers -c /controller_manager` | In authorized Live setup, use Prepare controller after prerequisites pass. Preview must not activate motion controllers. |
| Competing controller or command source | Another position claimant/publisher/action exists | `ros2 topic info /scaled_joint_trajectory_controller/joint_trajectory --verbose` and controller listing | Identify the owner and shut down the conflicting session deliberately. GUI Prepare does not evict it. |
| Servo blocks movement | Collision/singularity/joint-bound halt or stale status | `ros2 topic echo /vr_real/servo_node/status --once` | Inspect model and panel explanation. Do not disable collision/singularity checks to force movement. |
| External Control unavailable | Pendant program not running or network issue | `ros2 topic echo /io_and_status_controller/robot_program_running --once` | Check robot program and correct workstation network address using the site's procedure. |
| Zero robot response despite connection | Speed slider/scaling zero or invalid state | `ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling --once` | Inspect pendant and panel; scaling feedback is a percentage converted by the gate. |
| UR protective stop | Physical/controller condition reported by robot | `ros2 topic echo /io_and_status_controller/safety_mode --once` | Investigate the cause and use the approved pendant recovery procedure. The app does not automatically clear stops. |
| Motion fails after tracking restoration | Clutch reference was invalidated | `ros2 topic echo /vr_real/operator_status --once` | Release A and wait for healthy stationary recovery. Some faults preserve Enable intent; a new press is always needed. |
| Preview has no motion output | Expected output-disabled mode | `ros2 param get /vr_real/teleop motion_output_enabled` | Complete commissioning before choosing Live setup. Do not add a trajectory publisher. |

Software Disarm requests normal downstream stopping. Use the pendant/physical emergency stop for an emergency; neither a status flag nor zero Twist establishes physical standstill.

## Settings and motion

| Symptom | Likely cause | Diagnostic command | Corrective action |
|---|---|---|---|
| Speed differs from YAML | Adjacent saved preferences or runtime update | `ros2 param get /vr_real/teleop max_linear_speed_m_s` | Compare panel, `operator_settings.json`, and session effective source log. Current saved value is 1.5 m/s versus YAML 0.10. |
| Saved joint limit did not change live motion limit | Native setting needs restart | `ros2 param get /scaled_joint_trajectory_controller smoothing.max_velocity` | Disarm, restart managed ROS, then verify both backend and native limits. |
| Robot follows with delay | Filters, acceleration bounds, Servo prediction, UR lookahead, timing or speed scaling | `ros2 topic hz /vr_real/servo_joint_targets` and native diagnostic recording | Compare input/Servo/gated/native/measured stages before tuning; do not remove watchdogs or limits. |
| Robot sway or oscillation | Mapping/target behavior, timing, loop interaction, tool/model or tuning | `~/Documents/Teleop/ur_teleop/scripts/record_sway.sh` | Record an already authorized session; analyze each stage. Do not add a second position loop on Servo velocity output. |
| Damping/gain changes have little effect | Velocity mode selected | `ros2 param get /vr_real/teleop teleop_control_mode` | Position error gain and measured-tool damping belong to position mode. Use the documented mode-specific controls. |
| Hand movement feels too large/small | GUI inverse ratio versus stored scale | `ros2 param get /vr_real/teleop translation_scale` | GUI 1.2:1 corresponds to stored 0.8333. Check saved values, mode, and deadband. |
| Joint acceleration warning | Upstream estimate exceeds diagnostic threshold | `ros2 topic echo /vr_real/operator_status --once` | Compare native smoothed reference and measured joints. Warning is diagnostic and does not itself change commands. |
| Camera view missing | Optional camera serial/USB/SDK issue | `ros2 topic echo /wrist_camera/status --once` | Review camera log and serial; teleop process health is separate. Mount transform is uncalibrated. |

See [Configuration](CONFIGURATION.md) for validation ranges and restart requirements. Existing saved values are not recommendations for a new installation.

## Installation and processes

| Symptom | Likely cause | Diagnostic command | Corrective action |
|---|---|---|---|
| Package or executable missing | Workspace not built/sourced | `ros2 pkg executables ur_vr_teleop` | Follow the build and overlay sourcing order in [Installation](INSTALLATION.md). |
| Python import failure | Conda Python or stale overlay | `/usr/bin/python3 -c 'import rclpy, numpy, scipy, yaml, tkinter, PIL'` | Use system Python and fresh sourced shell; rebuild affected Python packages. |
| Bridge build/library failure | Missing pinned libsurvive or precision mismatch | `git -C ~/Documents/Teleop/ur_teleop/tools/libsurvive rev-parse HEAD` | Import pins, initialize submodules, build library, then rebuild bridge with matching ABI. |
| Camera package build failure | Missing SDK/pkg-config metadata | `pkg-config --modversion realsense2` | Install `librealsense2-dev`; runtime camera disable does not remove build dependency. |
| Calibration hash error | Wrong path/robot calibration | `cat ~/Documents/Teleop/ur_teleop/config/ur10e_calibration.yaml` | Review robot identity and matching kinematics. Do not remove the launch hash check. |
| No ROS topics / mismatched TF | Wrong domain, stale overlay, competing graph or frame mismatch | `ros2 node list` and `ros2 run tf2_ros tf2_echo base_link tool0` | Production domain 43, simulator 42; consistent wall time, namespaces, and model. |
| Desktop opens but no stack | Panel-only launcher used, startup child failure, ownership lock | Inspect Open logs and `ur_teleop/start_teleop.sh` | Use managed launcher. Diagnose owner/process identity before removing locks or stopping anything. |
| Shortcut absent on fresh checkout | Desktop entry is not tracked | `test -x ~/Documents/Teleop/ur_teleop/start_teleop.sh` | Launch the script; repository has no shortcut installer. |
| FIFO scheduling warning | Session lacks real-time priority permission | `~/Documents/Teleop/ur_teleop/scripts/setup_realtime_permissions.sh --check` | Review helper installation and full logout/login; it does not change the kernel. |

The desktop uses ownership metadata and process groups to stop its own children. Avoid broad `pkill` cleanup or deleting arbitrary build/state directories. A failed driver/tracking/teleop child stops the managed stack; camera/RViz are optional. Preserve logs before restarting a failed session.

## Dashboard synchronization

- **Unable to verify running values:** inspect `effective_parameters` in
  `/vr_real/operator_status`. It must contain every saved tuning key. Rebuild and
  source the updated `ur_vr_teleop` package in both backend and GUI processes;
  an older backend's partial aliases are deliberately not assumed complete.
- **Applied — awaiting readback:** service acceptance arrived, but a complete
  matching configuration snapshot has not. Check fresh status and
  backend/session identity. After timeout, review/revert fresh values before
  retrying; a timeout does not undo an accepted request.
- **External parameter changes:** the draft was preserved. Revert to confirmed
  current values, then re-enter intended edits; do not blindly resubmit.
- **Applied to current session, but saving failed:** inspect the full action
  history for the filesystem error. Current runtime values are retained; repair
  the preference-directory problem and explicitly Apply + Save again.
- **Restart required:** compare active final limit with the separate saved value;
  perform the explicit supervised restart workflow and verify both readbacks.

Missing velocity feedback is displayed as Unavailable, not zero. Measured TCP
velocity is a TF-derived estimate. Clutch metrics stop being current on release
or status loss. See the [field contract](OPERATOR_STATUS.md) for each source and
its unknown-state behavior.

## V4 one-button startup

| Display | Action |
|---|---|
| Configuration unavailable | Inspect the exact backend reason: explicit RPY mapping, frames, robot IP/calibration, tracking file or typed tuning. A missing historical evidence file alone does not block Start. |
| Waiting for External Control | Start the correct program on the pendant; application never issues Play |
| Robot safety state / FAULT | Inspect pendant and follow site recovery; no automatic fault reset/protective-stop release |
| Waiting for valid VR tracking | Inspect valid 6-DoF pose/continuity and TF/joints in Advanced diagnostics; one calibrated station is sufficient, but optical signal alone is not |
| Controller ownership conflict | Resolve the competing controller/action explicitly; startup does not evict it |
| Service unavailable/rejected | Inspect exact stage and service/driver logs; fix the cause and press START for one controlled retry |
| Enable timed out; outcome unknown | STOP remains active; use Engineering Restart ROS, then a fresh START after status returns |
| Servo stale (`WAITING — SERVO STATUS`) or `Servo process not running` | START attempts one bounded `start_servo` restart; if fresh status still does not arrive, or the process is missing, use Engineering Restart ROS then a fresh START. A stale status is never treated as healthy and never waits indefinitely |
| Controller activation acknowledged but ownership not confirmed | Inspect `/controller_manager/list_controllers` and the driver log; resolve the competing claimant/plugin issue, then press START |
| Effective limits mismatch | Compare visible active values and controller readback; native limits need restart; review the actual envelope physically before operation |
| Tracking restored but no movement | Release A, wait for healthy stationary readiness, then make a fresh press |

For Preview and optional observations, see [Engineering and rollback](OPERATION.md#engineering-and-rollback). The domain-42 simulator remains available; no physical robot session is needed for automated tests.

## Sway after hand motion stops

Use **RECORD SWAY DIAGNOSTICS**, then the [offline analyzer](SWAY_DIAGNOSTICS.md). Compare hand/Twist, Servo, final native reference and feedback before changing a controller. Null metrics mean unavailable data, inadequate observation duration or an ambiguous signal. Do not increase gain, speed, acceleration or jerk to mask oscillation. A recording failure appears in the recorder log; a second recorder is rejected by the shared lock. An interrupted/incomplete bag must not be treated as complete evidence.
