# Commissioning and validation

Software tests are not a substitute for hardware safety verification. This workflow separates read-only inspection, mock tests, and explicitly supervised physical validation. Do not activate physical robot motion as part of an automated test or documentation check.

## 1. Static inspection

Record the repository revision, dependency revisions, ROS/UR versions, robot identity, and calibration hash. Review [configuration precedence](CONFIGURATION.md#configuration-precedence), including the committed saved profile. Set any site-approved initial limits deliberately; no numerical setting in this guide is a universal safe physical limit.

```bash
cd ~/Documents/Teleop
git status --short
git rev-parse HEAD
vcs export --exact .
cat ur_teleop/config/real_teleop.yaml
cat ur_teleop/config/operator_settings.json
```

Confirm that the configured robot calibration belongs to this robot, the tool model is appropriate, and the tracking-to-base transform has not been invalidated. Preserve libsurvive calibration before any tracking recalibration. Check [Installation](INSTALLATION.md) for serials, paths, and external dependencies.

## 2. ROS and hardware configuration

Verify domain 43 for the managed real stack, wall-time operation, the robot network route, External Control installation, and workstation address on the pendant. Check scheduling permissions with the supplied `--check` helper. Do not bypass a protective stop or run an unfamiliar pendant program as a connectivity test.

Inspect the session's generated driver command and controller YAML. The managed driver starts with `activate_joint_controller:=false`; this is a startup setting, not a promise that no other process has activated a controller.

## 3. Tracking-only checks

Start only the bridge as shown in the [README](../README.md#quick-start), with no robot-control stack. Observe:

```bash
ros2 topic echo /vr/controller/device --once
ros2 topic echo /vr/controller/optical_status --once
ros2 topic echo /vr/controller/pose --once
ros2 topic hz /vr/controller/pose
ros2 topic echo /vr/controller/joy --once
```

Confirm controller identity, increasing pose timestamps, finite position/orientation, expected frame, button index 4, and plausible motion. Test LH0 alone, LH1 alone, and both if both are installed. Recheck from cold start separately: warm loss of one station does not prove single-station initialization. A station must retain a valid calibration and enough visible geometry for the solver.

With both blocked, optical freshness must expire. With optical signal but invalid/stale pose, tracking readiness must remain unavailable. Never equate a positive paired-sensor count with validated 6-DoF tracking.

## 4. Dry-run and simulation

Managed **Preview** runs the commissioning path with `motion_output_enabled=false`; it creates no final joint-command publisher and expects inactive position controllers. It connects the real driver. Keep the physical robot's motion controllers inactive while checking tracking loss and mapping.

The separate simulator is hardware-free and uses domain 42. It tests the legacy position clutch/Servo path, not the final native production controller. Automated ROS integration and Servo pipeline checks below cover additional production components using synthetic feedback.

## 5. Joint state and TF

In driver-connected Preview, verify actual measured joints and `base_link → tool0`, including the physical tool definition:

```bash
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link tool0
ros2 control list_controllers -c /controller_manager
ros2 topic info /vr_real/servo_joint_targets --verbose
ros2 topic info /scaled_joint_trajectory_controller/joint_trajectory --verbose
```

The final command publisher should be absent in Preview. Confirm all six named joints, usable velocities, fresh TF, and one expected publisher per supervised interface. Stop these observation commands with Ctrl+C when finished.

Verify all three translation directions and rotation axes in RViz before applying the alignment confirmation. Check tool orientation as well as position. Moving a Lighthouse, replacing calibration, or changing the mapping requires repeating this step.

## 6. Readiness and controller checks

Test the panel's blocking reasons without physical motion: missing heartbeat, stale optical status, invalid pose, missing program, zero speed scaling, unavailable Servo status, inactive/wrong controller, and unverified confirmations. Use automated fixtures for injected faults; do not publish fabricated healthy states into a physical robot's domain.

For a later authorized Live setup, review the final native controller values:

```bash
ros2 param get /vr_real/teleop max_linear_speed_m_s
ros2 param get /vr_real/teleop max_joint_speed_rad_s
ros2 param get /scaled_joint_trajectory_controller smoothing.max_velocity
ros2 param get /scaled_joint_trajectory_controller smoothing.max_acceleration
ros2 param get /scaled_joint_trajectory_controller smoothing.max_jerk
ros2 param get /scaled_joint_trajectory_controller cmd_timeout
ros2 topic echo /vr_real/operator_status --once
```

A parameter query is evidence of the configured value, not a measured guarantee of physical response. The gate additionally verifies controller ownership and watchdog configuration. Use the GUI's Prepare operation in the physical procedure; do not manually switch controllers to evade readiness checks.

## 7. Supervised physical validation

This stage requires an authorized operator, established work area, pendant access, approved tool/payload/network configuration, and site-selected low initial limits. It was **not performed for this documentation task**.

Follow [Operation](OPERATION.md#authorized-live-session). Verify effective limits after startup, enable with A released, and command small single-axis movements before combined translation/rotation. Compare requested and measured motion. Validate each mode separately; successful position-mode operation does not prove velocity-mode behavior. Increase limits only under the site's commissioning process.

## 8. Stop and recovery matrix

First exercise fault cases with mocks/Preview. Physical stop timing and stopping distance require a separate supervised plan. The expected software behaviors are:

| Scenario | Expected result |
|---|---|
| LH0 alone, valid pose | Eligible tracking without experimental policy or station penalty |
| LH1 alone, valid pose | Same |
| Both stations, valid pose | Eligible tracking |
| One disappears, remaining pose stays valid | No station-count-only disarm |
| Both disappear | Readiness lost, command path stops |
| Optical signal, no valid 6-DoF pose | Motion blocked |
| Stale/nonfinite/jumping/frozen pose | Fault; old reference cannot continue |
| A released | Zero demand / normal downstream stop |
| Tracking returns while A held | No automatic motion; release required |
| Healthy recovery, A released, then fresh press | New baseline, zero initial demand, controlled motion only after readiness |
| GUI heartbeat lost / Disarm | Enable intent cleared and stop requested |
| Servo halt, invalid robot state, competing controller/action, command timeout | Gate blocks; investigate before recovery |

Record software transitions and measured motion. Do not claim “immediate physical stop” from a Boolean flag or zero Twist message. The hold sequence, native controller, UR speed scaling, driver watchdogs, and physical robot dynamics all contribute.

## 9. Diagnostics and evidence

The managed session directory under `ur_teleop/logs/` contains component logs, startup command arguments, effective values with sources, generated controller settings, and the driver description copy. Record runtime parameter changes separately.

Read-only recording/analysis tools:

```bash
cd ~/Documents/Teleop
./ur_teleop/scripts/record_sway.sh
```

Stop recording with Ctrl+C. It writes a ROS bag under `ur_teleop/diagnostics/` and does not enable or move the robot. It includes hand pose/timing, Cartesian demand, Servo targets, gated targets, native smoothed reference, joint states, TF, robot state, and parameter/status events.

```bash
export ROS_DOMAIN_ID=43
/usr/bin/python3 ur_teleop/scripts/compare_modes.py
/usr/bin/python3 ur_teleop/scripts/diagnose_motion.py
```

The comparison without `--bag` is a synthetic offline comparison, not a robot-performance measurement. `--bag` accepts an existing SQLite ROS bag directory for read-only analysis. `diagnose_motion.py` observes the running ROS graph and writes a JSON report after its 20-second observation window (or earlier interruption). It does not activate motion. These scripts retain the original workstation paths; inspect them before relocating.

`JointDiagnostics` reports upstream joint estimates/warnings; it does not change commands. Native `control_diagnostics` separates measured, Servo, and Ruckig references. Compare stages before attributing sway to a particular filter.

## Automated validation

Run from the repository root with the installed workspace matching the source. A Python import from an old overlay can otherwise test stale installed code. No production stack is needed.

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

The default `test_*.py` discovery does not run the separately named ROS check scripts. Run these explicitly when relevant:

```bash
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/servo_pipeline_check.py
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/mock_startup_check.py
```

The first launches actual Servo/RSP with synthetic joints on localhost domain 147 and no robot driver. The second uses a generated UR driver launch with loopback IP, `GenericSystem`, dashboard disabled, and motion controller inactive on domain 148. Both scripts contain original absolute paths. `gui_render_check.py` is an additional display-dependent GUI check, not a physical test.

Interactive mock simulation:

```bash
ROS_DOMAIN_ID=42 ros2 launch ur_vr_sim_teleop simulation.launch.py
```

For hand input, a separate bridge must run in domain 42, without a second process competing for its USB device. Once the mock model and domain are verified, simulated motion can be enabled in another sourced terminal:

```bash
ROS_DOMAIN_ID=42 ros2 param set /vr_sim/teleop enable_motion true
```

This command targets simulation only. Do not translate it into a production bypass. Release A and press again after mode changes. Shut down the simulation with Ctrl+C.

## Evidence boundaries

Unit tests cover math, filtering, quality, gating, preferences, GUI logic, and process ownership. ROS integration checks use mocked state and exercise the gate; C++ tests cover optical/pose guards and the native smoother/controller. These do not validate real tracking occlusion geometry, networking under load, payload/TCP accuracy, physical collision models, stopping distances, or one-station cold start.

For a change report, record exact commands, pass/fail counts, skipped checks, package builds performed, and remaining physical tests. Earlier dated notes are historical evidence only; do not present their results as new measurements.

### Documentation-update verification (2026-10-12)

The checkout and remote HEAD both resolved to `c7373d97012ede52c1bcf1e28041031105a5c396`. This update changed Markdown only.

| Check actually run | Result |
|---|---|
| Main workspace build using the Installation command | All five packages built successfully |
| Python test directories above | 164 passed; one existing xacro/Python compatibility warning |
| Explicit `ros_integration_checks.py` | 78 passed |
| Bridge CTest | 2/2 passed |
| Native controller CTest | 2/2 suites passed, containing 11 GoogleTest cases |
| Installed executables and both launches' `--show-args` | Matched documented entry points/arguments |
| Local Markdown links/anchors, source paths, Bash syntax, saved-key/YAML coverage | Passed |
| Mermaid parser | All three diagrams passed |
| Git whitespace/scope check | Passed; only the eight requested Markdown deliverables changed |

The fresh-system apt recipe, calibration-overlay rebuild, separate Servo/mock-startup scripts, display-dependent GUI check, interactive simulation, and all physical tracking/robot procedures were not executed in this documentation update. The main build reused the installed system and existing dependencies; it is not a clean-machine reproduction.

### V3.1 dashboard validation

After building/source-refreshing `ur_vr_teleop`, run the ordinary Python suites
and explicit ROS checks above. The display-dependent check now exercises real
Tk widgets and ROS parameter services on **localhost domain 147** with a fake
backend and temporary preferences; it never launches a driver or connects to a
robot:

```bash
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py
```

It checks Apply/Save/rejection/readback/conflicts, session changes, stale status,
mode visibility, restart staging and close. It captures actual window images at
1440×900, 1366×768 and 1280×720 under `/tmp/ur_teleop_ui_v3_1/`. A working X display
is required (or run under Xvfb). Pure `test_dashboard_state.py` and
`test_dashboard_async.py` cover normalized diffs, malformed telemetry,
notification lifetime, process ownership serialization and request failures.
Physical commissioning remains a separate supervised activity.
