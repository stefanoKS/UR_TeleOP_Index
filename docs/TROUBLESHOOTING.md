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
| External Control unavailable | In automatic mode: Remote Control, Dashboard service, identity, Load/Play or running readback failed. In manual mode: pendant program is not running or the network is wrong. | `ros2 topic echo /io_and_status_controller/robot_program_running --once`; `ros2 service list` (filter `dashboard_client`) | Automatic mode: check Remote Control, the exact approved program identity, Dashboard services, Load/Play response and program-running/driver readback in the dashboard. Manual mode: start the approved External Control program through the pendant workflow and verify the workstation address. The app never changes Remote Control mode or resets safety faults. |
| Zero robot response despite connection | Speed slider/scaling zero or invalid state | `ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling --once` | Inspect pendant and panel; scaling feedback is a percentage converted by the gate. |
| UR protective stop | Physical/controller condition reported by robot | `ros2 topic echo /io_and_status_controller/safety_mode --once` | Investigate the cause and use the approved pendant recovery procedure. The app does not automatically clear stops. |
| Motion fails after tracking restoration | Clutch reference was invalidated | `ros2 topic echo /vr_real/operator_status --once` | Release A and wait for healthy stationary recovery. Some faults preserve Enable intent; a new press is always needed. |
| Preview has no motion output | Expected output-disabled mode | `ros2 param get /vr_real/teleop motion_output_enabled` | Complete commissioning before choosing Live setup. Do not add a trajectory publisher. |

Software Disarm requests normal downstream stopping. Use the pendant/physical emergency stop for an emergency; neither a status flag nor zero Twist establishes physical standstill.

## ST3215 abrupt temperature changes

A single high sample is **TEMPERATURE UNVERIFIED**, immediately inhibits movement, and triggers bounded read-only confirmation. Confirmed high temperature, suspected transient readings and feedback errors are separate outcomes. Preserve the raw event and counter/timing evidence; do not assume sensor failure or genuine heating from the GUI value alone. Review [V6.2 diagnostics and read-only hardware procedure](V6_2_GRIPPER_RELIABILITY.md). Recovery requires Connect read-only, fresh normal feedback, explicit ENABLE and new trigger pickup; cool and investigate a confirmed high event first. Never raise the limit to suppress faults.

## Gripper and Dashboard program start

| Symptom | Likely cause | Diagnostic command | Corrective action |
|---|---|---|---|
| ENABLE GRIPPER rejected | Profile absent/disabled/uncalibrated, duplicate node/owner | `ros2 topic echo /gripper/status --once` | Connect read-only in Gripper Setup, review distinct endpoints/travel, save, reconnect, then enable. |
| Gripper says "session not enabled" | Latched enable was not requested or was withdrawn | `ros2 topic echo /gripper/status --once` | Press ENABLE GRIPPER. A, START TELEOP, Servo and External Control are not required. |
| ENABLE rejected: heartbeat withdrawn | STOP withdrew the gripper heartbeat; a late pre-STOP ENABLE is rejected | `ros2 topic echo /gripper/status --once` | Wait for the GUI heartbeat, then press ENABLE again explicitly. |
| GRIPPER STOP UNCONFIRMED | Disable service timeout/rejection or node unavailable | `ros2 topic echo /gripper/status --once`; process log | Heartbeat remains withdrawn and ENABLE is blocked. Restore the node, then retry STOP. Do not treat the stop as confirmed. |
| PROGRAM UNVERIFIED / PROGRAM STOPPED | Dashboard running/identity readback failed, stopped, or changed | `ros2 service call /dashboard_client/program_running ur_dashboard_msgs/srv/IsProgramRunning` | Inspect the approved program in the dashboard/pendant and the driver. Arm intent is withdrawn; press START TELEOP only after fresh running and identity verification. |
| Program identity mismatch | Configured value differs by path from Dashboard readback | Verify action in **Settings → External Control program** | Enter the exact normalized value Dashboard reports; bare filenames are not inferred to match directories. |
| ARM STOP UNCONFIRMED on Close/Restart | No fresh disarmed backend status before timeout | `ros2 topic echo /vr_real/operator_status --once` | Bounded Dashboard/process shutdown still proceeds. Check the backend and pendant before the next session. |
| Gripper connected but never follows | Pickup not completed, stale Joy/pose/optical, no GUI heartbeat | `ros2 topic echo /gripper/status --once` while moving the trigger | Move the trigger >=5% and match actual closure; restore tracking; check the GUI is open. |
| Serial adapter not found / permission denied / busy | USB, udev/dialout, or another serial owner | `ls -l /dev/serial/by-id/ /dev/ttyUSB*` | Fix access through the workstation administrator. Do not globally relax permissions or kill an unknown owner. |
| START TELEOP says Local Control | Pendant not in Remote Control | `ros2 service call /dashboard_client/is_in_remote_control ur_dashboard_msgs/srv/IsInRemoteControl` | Select Remote Control on the pendant. The app never changes it. |
| Dashboard services unavailable | Driver/dashboard_client not running or wrong network | `ros2 service list` (filter for `dashboard_client`) | Check the managed driver and robot network; press START TELEOP after recovery. |
| Program Play acknowledged but not running | Program rejected/paused or wrong identity | `ros2 service call /dashboard_client/program_running ur_dashboard_msgs/srv/IsProgramRunning` | Inspect the pendant and driver log. The app does not retry Play automatically. |
| Close reports unconfirmed program stop | Driver already gone or Dashboard readback timed out | Driver/teleop log and pendant | Expect bounded termination was still applied. A pendant socket-disconnect popup may occur; investigate the driver before the next session. |

## Settings and motion

| Symptom | Likely cause | Diagnostic command | Corrective action |
|---|---|---|---|
| Speed differs from YAML | Adjacent saved preferences or runtime update | `ros2 param get /vr_real/teleop max_linear_speed_m_s` | Compare panel, `operator_settings.json`, and session effective source log. Committed saved value is 0.5 m/s versus YAML 0.10; the effective runtime value is authoritative. |
| Saved joint limit did not change live motion limit | Native setting needs restart | `ros2 param get /scaled_joint_trajectory_controller smoothing.max_velocity` | Disarm, restart managed ROS, then verify both backend and native limits. |
| Robot follows with delay | Filters, acceleration bounds, Servo prediction, UR lookahead, timing or speed scaling | `ros2 topic hz /vr_real/servo_joint_targets` and native diagnostic recording | Compare input/Servo/gated/native/measured stages before tuning; do not remove watchdogs or limits. |
| Robot sway or oscillation | Mapping/target behavior, timing, loop interaction, tool/model or tuning. The October 15 Servo position-history correction addressed the previously identified 2.6 Hz mechanism; the operator reports the oscillation no longer occurred, but no separate post-fix physical measurement was recorded in this audit. | `~/Documents/Teleop/ur_teleop/scripts/record_sway.sh` | Record an already authorized session; analyze each stage. Do not add a second position loop on Servo velocity output. Treat any recurrence as a new observation, not as proof about the earlier correction. |
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

The desktop uses ownership metadata and process groups to stop its own children. Avoid broad `pkill` cleanup or deleting arbitrary build/state directories. A UR driver or teleop failure retires arm authorization, stops the affected arm groups and retains healthy tracking/gripper processes; independent gripper follow still needs fresh input and its own authorization. A tracking bridge failure stops the affected arm and tracking processes but retains gripper diagnostics while its node remains available; stale tracking is never fed to the gripper. A gripper failure is reported independently and must not terminate a healthy arm session. Camera/RViz are optional. Full Stop ROS or application close terminates all owned processes. Preserve logs before restarting a failed session.

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
| Waiting for External Control | **Automatic Dashboard mode enabled:** check Remote Control, the exact approved program identity, Dashboard service availability, the Load/Play response, program-running readback and the driver connection. **Automatic mode disabled:** start the approved External Control program through the pendant workflow. The application never changes Remote Control mode or resets safety faults. |
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

## Calibration-file warning during normal tracking

The former generic “Calibration files changed during this session” warning could be triggered by libsurvive's normal configuration writes. The tracking persistence file is now checked for initial availability only; routine rewrites do not revoke readiness. Rebuild/source `ur_vr_teleop` and restart the managed backend once to load this fix. Do not delete or regenerate tracking calibration to clear the old warning. Robot kinematics changes still block with a robot-specific message; live tracking loss, stale pose and invalid mapping still block independently.
