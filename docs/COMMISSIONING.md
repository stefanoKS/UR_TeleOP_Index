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

Verify all three translation directions and rotation axes in RViz before physical operation. Check tool orientation as well as position. Moving a Lighthouse, replacing calibration, or changing the mapping requires repeating this step.

## 6. Readiness and controller checks

Test the panel's blocking reasons without physical motion: missing heartbeat, stale optical status, invalid pose, missing program, zero speed scaling, unavailable Servo status, inactive/wrong controller, and missing/invalid configuration. Use automated fixtures for injected faults; do not publish fabricated healthy states into a physical robot's domain.

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

A parameter query is evidence of the configured value, not a measured guarantee of physical response. The gate additionally verifies controller ownership and watchdog configuration. Use START TELEOP's preparation sequence in the physical procedure; do not manually switch controllers to evade readiness checks.

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
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/gui_ros_load_check.py
/usr/bin/python3 ur_teleop_ws/src/ur_bus_servo_gripper/test/ros_gripper_check.py
```

The Servo check launches actual Servo/RSP with synthetic joints on localhost domain 147 and no robot driver. It also holds Cartesian demand constant while injecting a 2.6 Hz measured-position oscillation: Servo velocity must remain below 0.01 rad/s for the 0.001 m/s test input. It checks zero-command output, suppression of stale-input output and recovery (the gate/native watchdogs own stopping after output ceases). The native C++ tests separately check the loaded Servo plugin, invalid input, moving-feedback rejection, reset and final Ruckig derivative limits. The mock-startup check uses a generated UR driver launch with loopback IP, `GenericSystem`, dashboard disabled, and motion controller inactive on domain 148.

`gui_ros_load_check.py` compares the former 10 Hz single-callback polling behavior with the dedicated GUI executor under synthetic 100 Hz backend status, 25 Hz gripper status, 100 Hz native references and optional 15 FPS 640x480 camera images. It asserts that threaded backend delivery stays below the 0.5 s watchdog and that stopping only backend status still retires startup while other traffic continues. `ros_gripper_check.py` uses fake serial feedback and verifies startup, pickup, loss, recovery, stale timestamps, STOP and absence of an arm-command publisher. `gui_render_check.py` is an additional display-dependent GUI check, not a physical test.

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
1440×900, 1366×768 and 1280×720 under `/tmp/ur_teleop_ui_v4/`. A working X display
is required (or run under Xvfb). Pure `test_dashboard_state.py` and
`test_dashboard_async.py` cover normalized diffs, malformed telemetry,
notification lifetime, process ownership serialization and request failures.
Physical commissioning remains a separate supervised activity.

## V4 engineering record

Historical `verified_setup.json` and legacy engineering records are optional review evidence. Routine startup does not read them, require expiry/signatures, or require eleven observed results. The [optional observations tool](INITIAL_SETUP.md) remains available in Engineering tools. Missing history never automatically opens a wizard.

Physical workcell commissioning remains necessary: robot identity/calibration, tracking frame alignment, tool/TCP/payload, mounting, workspace/collision model, appropriate limits and physical stopping/recovery. Configuration syntax checks and automated tests cannot establish these. Repeat the applicable physical assessment when the installation changes; preserve the configuration and site records.

### V4 software validation

`test_startup.py` tests asynchronous stage ordering, acknowledgments/readback, idempotency, missing prerequisites, failures/timeouts, STOP races, session retirement and tracking recovery. `test_engineering.py` retains legacy evidence regression coverage; `test_setup.py` tests new local persistence, invalidation, migration and observed setup transitions using synthetic fixtures only. The isolated ROS checks ensure legacy flags cannot satisfy missing configuration readiness and that optical loss and mapping-change revalidation remain effective. `gui_render_check.py` additionally uses real ROS services with a synthetic backend to test one-button preparation, duplicate Start, STOP, and reuse of active components. Screenshots from this check are simulated telemetry, not a robot commissioning result.

### Startup-repair validation (2026-10-14)

Changed sources: `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/{commissioning,startup,gui,desktop}.py` and matching tests (`test/test_startup.py`, `test/ros_integration_checks.py`, `test/gui_render_check.py`, `test/servo_pipeline_check.py`). The backend no longer publishes the permanent `servo_started` flag; it publishes `servo_state`, `servo_process_present`, `servo_status_seen` and `servo_status_fresh`. MoveIt Servo 2.5.x was inspected and its installed library confirmed (`status_codes.h`, `libmoveit_servo_lib.so`) to create the `status` publisher at process start and publish `Int8` only after `start_servo` begins its loop.

| Check actually run | Result |
|---|---|
| Python test directories (both packages, workspace overlays sourced) | 365 passed; one existing xacro/Python 3.11 warning |
| Explicit `ros_integration_checks.py` (localhost domain 147) | 87 passed |
| Display-dependent `gui_render_check.py` (`DISPLAY=:0`) | Passed; three layouts plus session/clutch/Servo banner assertions |
| Real `servo_pipeline_check.py` (real Servo/RSP on domain 147) | Passed; status publisher present before start, no status before `start_servo`, fresh warning-free status after start, 201 targets forwarded |
| `mock_startup_check.py` (GenericSystem, domain 148) | Passed |
| `colcon build --packages-select ur_vr_teleop` | Built successfully |

Not executed here: the C++ bridge/native-controller CTest suites (unaffected packages), the `calibration_ros_check.py` capture check, and all physical tests. Remaining hardware-only checks: real one-button startup with a live Servo/controller/driver, physical Servo-stale recovery, controller activation readback on the real controller manager, duplicate-backend/session behaviour, and the separate UR10e oscillation investigation.

## V5.1 controller capture validation

After rebuilding/sourcing `ur_vr_teleop`, run the additional explicit check serially with other ROS tests:

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=147 /usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/calibration_ros_check.py
```

This script explicitly initializes domain 147, uses synthetic inputs and the real motion-disabled Commissioning backend, verifies actual managed Preview command generation, and writes a synthetic report under `/tmp/ur_teleop_calibration_v51`. It never launches the physical driver or enables motion. The normal pytest directories include the pure capture and persistence tests; `gui_render_check.py` additionally checks combined review, missing-check save refusal, retries, STOP, cleanup and scrollable wizard layouts. See [V5.1 validation](CALIBRATION_V51.md).

Direction calibration does not resolve or validate the reported post-movement sway. Authorized supervised Live tests must separately measure actual stopping, recovery, tool/TCP/payload, alignment accuracy and tracking occlusion behavior. Do not synthesize healthy states in production or save a production setup from these automated tests.

## Current sway tools

See [Sway diagnostics](SWAY_DIAGNOSTICS.md) for recording, timestamp handling, stage comparison, synthetic results and limitations. Recording never enables the robot. The startup simplification does not fix or identify the physical sway cause.

The real recorder smoke check is separate from default pytest discovery:

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=147 /usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/sway_recording_check.py
```

It records synthetic joint states using the installed rosbag CLI, checks that no final motion publisher or enable service exists, finalizes the bag and reads it back. It never launches a driver.

## Tool endpoint regression

In the sourced, localhost-only test environment above, additionally run:

```bash
TELEOP_TEST_TCP=1 /usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/servo_pipeline_check.py
TELEOP_TEST_TCP=1 /usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/mock_startup_check.py
```

These use temporary profiles. The Servo check verifies through forward kinematics that angular demand holds the offset tip stationary while the flange translates. The mock check verifies the generated driver publishes the tip transform with GenericSystem. Unit tests cover rotation-vector conversion, immutable profiles, robot identity, output-only RTDE framing/rejections, disarmed editing, and mismatch/read-loss recovery. GUI checks use a mocked read and temporary profile; they do not connect to a robot.

Remaining physical commissioning: confirm TCP calibration and coordinates against the pendant, actual RTDE availability alongside the installed driver, tip pose in Preview, supervised low-speed rotation/translation and stop/re-engagement behavior, plus tool payload/collision geometry. Automated results do not establish these physical properties.

### Tool endpoint implementation validation (2026-10-16)

Executed with system Python 3.10 and the rebuilt Humble overlay. ROS checks used localhost domains 147/148; no production driver/desktop or physical motion was launched.

| Command (repository root unless noted) | Result |
|---|---|
| `colcon build --base-paths src --packages-select ur_vr_teleop --executor sequential` (inside `ur_teleop_ws`, system PATH, Humble sourced) | Passed; one package built |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q` | 391 passed; one upstream Python compatibility warning |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py -q` | 91 passed, including TCP mismatch, unavailable feedback, expiry, disarm and recovery |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/servo_pipeline_check.py` | Passed; 201 targets; 0.01 s horizon; zero/timeout/recovery checks passed |
| `TELEOP_TEST_TCP=1 /usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/servo_pipeline_check.py` | Passed, including fixed-tip angular-command FK check and existing pipeline checks |
| `TELEOP_TEST_TCP=1 /usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/mock_startup_check.py` | Passed; generated GenericSystem driver and offset-tip TF |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py` | Passed, including mocked TCP read, mm conversion and temporary-profile save; editor screenshot inspected |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/calibration_ros_check.py` | Passed; Preview capture/isolation/cleanup regression |
| `/usr/bin/python3 -m compileall -q ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop ur_teleop_ws/src/ur_vr_teleop/launch` | Passed |
| `git diff --check` and changed-document local-link checks | Passed |

The initial pytest invocation lacked the overlay and failed collection; sourcing the rebuilt overlay resolved it. The first offset mock-startup test used incomplete synthetic snapshot source metadata; the test fixture was corrected and rerun successfully. Unaffected C++ packages were not rebuilt or retested. Real-robot RTDE/TCP import, physical tool calibration, collision geometry and motion/stopping validation remain unperformed. No live TCP profile was created and existing operator settings were preserved.

## Gripper regression and physical assessment

The independent ST3215 package adds mock-worker, actual SDK/pseudo-terminal, ROS lifecycle and GUI calibration coverage. Follow [Gripper automated validation](GRIPPER.md#automated-validation) and its separate supervised physical sequence. Never point automated serial tests at a physical adapter.
