# Startup simplification and sway diagnostics — implementation report

Implemented 2026-10-14. No physical motion was performed, and no hardware sway cause or improvement is claimed.

## Dependency map and removed requirements

| Boundary | Classification | Result |
|---|---|---|
| `startup.py`: engineering_verified and supervised=True alternative | A — procedure | Replaced with backend configuration_ready; no supervised bypass |
| `commissioning.py`: engineering.Verification, verified_setup.json, legacy alignment/optical confirmations | A — procedure | Removed from reasons()/enable readiness; deprecated ROS inputs remain inert compatibility fields |
| `gui.py`: automatic wizard, normal confirmation controls, manual supervised Enable | A — procedure | Removed; optional observations remain in Engineering tools |
| `engineering.py`, `legacy_engineering.py`, setup_wizard/setup_checks: eleven results, physical_pass, record identity/expiry | A — optional history | Retained solely for optional historical records; never consulted by runtime readiness |
| desktop/launch engineering_session | A — old workaround | Accepted for command compatibility; cannot change readiness policy |
| preferences, configuration.py, launch calibration hash, native-limit readback | B — configuration | Explicit mapping provenance, finite typed tuning, robot IP, matching frames, calibration files/hash and effective native limits remain required |
| capture/observer isolation, tracking, Servo, joint gate, controller/driver supervision | C — runtime protection | Preserved |

The initial repository status was clean. Changes preserve saved operator settings, configuration precedence and the effective snapshot. No approval flag is forced true. A mapping supplied only by built-in defaults cannot establish configuration readiness. Configured mapping is not labeled physically verified. Configuration file checks run off the command executor; missing/invalid/stale results or calibration changes during a session block readiness. A parameter change requires validation of the current values.

Optional historical evidence retains its strict source identity and existing compatibility exceptions. This readiness change was not added to the old observational-hash exception. Historical evidence is not needed to start the configured workcell.

## Runtime protections and workflow

The single gate remains authoritative. Fresh pose/source data, continuity/freeze/jump rejection, optical presence from either station, valid joints/TF, robot and safety modes, External Control, speed scaling, Servo collision/singularity states, controller/topic/action ownership, native smoothing/limits, command age/watchdogs, GUI heartbeat, stationary preparation, A deadman and fresh release/repress recovery remain in place. STOP retires in-flight requests and withholds heartbeat; uncertain enable outcomes still require a new backend session.

Normal operation: open app → START TELEOP with A released → automatic controller/Servo preparation → wait for READY — PRESS A TO MOVE → hold A. Release A to stop; STOP TELEOP disarms and requires a new Start. Launch/restart remains disarmed. Physical commissioning of a new or changed workcell is still required, without mandatory GUI evidence generation.

No changes were made to `gate.py`, `tracking.py`, pose filtering/velocity estimation, Servo tuning, Ruckig/native controller code, UR servoj settings or committed operator preferences.

## Recording and analysis

Engineering / diagnostics → Advanced diagnostics / sway recording exposes RECORD SWAY DIAGNOSTICS and STOP RECORDING. The recorder only invokes rosbag recording and parameter reads. It owns its child process, uses a shared OS lock to prevent duplicate GUI/CLI recordings, finalizes on stop/close, reports failures and stays independent of the camera and enable workflow.

Each timestamped recording contains the ROS bag, topic inventory, selected/managed configuration, runtime parameter readbacks at start/stop, and any readback errors. Recorded operator status and parameter events preserve runtime tuning changes. Native/controller diagnostics and hand/source timestamps are included. See [Sway diagnostics](SWAY_DIAGNOSTICS.md) for commands, coverage and interpretation.

`analyze_sway.py` reads an existing bag without creating a ROS node. It produces JSON/Markdown reports, six individual joint plots, Cartesian/hand comparisons, raw pose and scaling plots when data exists. It compares Servo, gated, native and actual joint stages; reports timing/gaps, reversals, frequency candidates, position error, correlation lag, stop duration, overshoot/settling and acceleration estimates. Missing data stays unavailable. Native speed limits appear as overlays when runtime status supplies them.

Comparisons use bag receive time; source/header clocks are audited separately. Receive-time correlation includes transport/sampling and is not absolute actuator latency. Servo trajectories also have future horizons. Interpolation does not extrapolate or cross gaps over 100 ms. Stop/settling needs an observed transition and at least 0.5 s of sustained evidence. Frequency/reversal metrics alone do not identify a physical root cause.

A read-only parameter query in production domain 43 returned `Node not found`. Therefore no current hardware runtime settings were verified. Existing selected configuration retains gain 500, lookahead 0.03 s and saved overrides of 1.5 m/s, 180 deg/s, 2 rad/s and −40° yaw; these are not presented as measured current robot settings.

## Synthetic analysis results

| Known input | Observed analyzer result |
|---|---|
| 10 s, 2 Hz, 0.05 rad/s sinusoidal feedback; stationary Servo/native commands | Approximately 2.00 Hz and 39 feedback reversals on all six joints; zero command reversals; constant-command oscillation frequency and lag unavailable |
| Nonoscillatory command stops at 1 s; measured velocity stops at 1.2 s | 0.200 s standstill delay |
| Final reference 0.10 rad; feedback ends at 0.12 rad | 0.020 rad directional overshoot; no settling falsely reported |
| Gaussian command with 0.12 s delayed feedback | Approximately 0.120 s correlation lag |
| Missing interval / short recording tail | No interpolation across the gap; insufficient settling reported as unavailable |
| Serialized SQLite bag with reversed joint name order and separate device/ROS clocks | Correct named-joint reconstruction, clock audit and six PNG plots; no ROS node instantiated |

These are synthetic software results, not robot measurements. Generated example plots/report are in `/tmp/teleop_sway_validation/`; GUI screenshots with synthetic telemetry are in `/tmp/ur_teleop_ui_v4/`.

## Commands and results actually executed

All Python used `/usr/bin/python3` with Humble and the rebuilt overlay. ROS tests used localhost domains 147/148; no managed physical desktop was launched.

| Command (from repository root unless noted) | Result |
|---|---|
| In `ur_teleop_ws`: `colcon build --base-paths src --packages-select ur_vr_teleop` | Passed; rebuilt affected package against installed Humble dependencies |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q` | 351 passed; one existing xacro/Python compatibility warning |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py -q` | 86 passed |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/calibration_ros_check.py` | Passed: Preview isolation, capture, tracking loss and cleanup |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py` | Passed: actual Tk/ROS settings, one-button preparation, STOP, no automatic wizard, optional capture lifecycle, 1440×900 / 1366×768 / 1280×720 layouts |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/servo_pipeline_check.py` | Passed: 201 Servo targets, 10 ms horizons, no final robot-command publisher |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/mock_startup_check.py` | Passed: generated parameters, GenericSystem, six joints and TF |
| `ctest --test-dir ur_teleop_ws/build/ur_vr_bridge --output-on-failure` | 2/2 suites passed using existing native build |
| `ctest --test-dir ur_teleop_ws/build/ur_teleop_controllers --output-on-failure` | 2/2 suites passed using existing native build |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/sway_recording_check.py` | Passed: actual rosbag CLI recorded/read back 470 synthetic joint samples on localhost domain 147, no motion publisher/enable service, clean finalization |
| Latest focused `test_sway.py` after recorder log-error/QoS handling | 8 passed |
| `bash -n ur_teleop/scripts/record_sway.sh`, Python compileall, `git diff --check`, local Markdown file-link check | Passed |

The first real recorder smoke test exposed missing Humble QoS history/depth fields; explicit history/depth fixed the failure, and both the real smoke test and a parser regression now pass. Initial regression failures were old mandatory-wizard expectations and a fixture needing current configuration validation after a tuning change; these were updated without removing runtime fault/recovery tests. An attempted `xvfb-run` invocation was unavailable; GUI verification subsequently passed on the existing display. No new physical setup record was created. No fresh-machine dependency installation or real-hardware recording was tested.

## Remaining physical validation

Use an explicitly supervised workcell session to verify calibration ownership, physical axis mapping, TCP/payload/mounting/collision model, appropriate effective limits, tracking occlusion and actual stopping/recovery. Collect a real sway bag before assigning the oscillation to hand estimation, Servo, native trajectory handling, speed scaling or the UR response. No gains, limits or control algorithms were changed to mask the symptom.

## Actual modified and added files

Paths below are relative to the repository root.

- [AGENTS.md](../AGENTS.md)
- [README.md](../README.md)
- [docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md)
- [docs/COMMISSIONING.md](../docs/COMMISSIONING.md)
- [docs/CONFIGURATION.md](../docs/CONFIGURATION.md)
- [docs/INITIAL_SETUP.md](../docs/INITIAL_SETUP.md)
- [docs/INSTALLATION.md](../docs/INSTALLATION.md)
- [docs/OPERATION.md](../docs/OPERATION.md)
- [docs/OPERATOR_STATUS.md](../docs/OPERATOR_STATUS.md)
- [docs/SIMPLIFICATION_REPORT.md](../docs/SIMPLIFICATION_REPORT.md)
- [docs/SWAY_DIAGNOSTICS.md](../docs/SWAY_DIAGNOSTICS.md)
- [docs/TROUBLESHOOTING.md](../docs/TROUBLESHOOTING.md)
- [ur_teleop/scripts/analyze_sway.py](../ur_teleop/scripts/analyze_sway.py)
- [ur_teleop/scripts/record_sway.sh](../ur_teleop/scripts/record_sway.sh)
- [ur_teleop_ws/src/ur_vr_teleop/launch/commissioning.launch.py](../ur_teleop_ws/src/ur_vr_teleop/launch/commissioning.launch.py)
- [ur_teleop_ws/src/ur_vr_teleop/package.xml](../ur_teleop_ws/src/ur_vr_teleop/package.xml)
- [ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py](../ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py)
- [ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py](../ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py)
- [ur_teleop_ws/src/ur_vr_teleop/test/sway_recording_check.py](../ur_teleop_ws/src/ur_vr_teleop/test/sway_recording_check.py)
- [ur_teleop_ws/src/ur_vr_teleop/test/test_calibration_capture.py](../ur_teleop_ws/src/ur_vr_teleop/test/test_calibration_capture.py)
- [ur_teleop_ws/src/ur_vr_teleop/test/test_configuration.py](../ur_teleop_ws/src/ur_vr_teleop/test/test_configuration.py)
- [ur_teleop_ws/src/ur_vr_teleop/test/test_setup.py](../ur_teleop_ws/src/ur_vr_teleop/test/test_setup.py)
- [ur_teleop_ws/src/ur_vr_teleop/test/test_startup.py](../ur_teleop_ws/src/ur_vr_teleop/test/test_startup.py)
- [ur_teleop_ws/src/ur_vr_teleop/test/test_startup_launch.py](../ur_teleop_ws/src/ur_vr_teleop/test/test_startup_launch.py)
- [ur_teleop_ws/src/ur_vr_teleop/test/test_sway.py](../ur_teleop_ws/src/ur_vr_teleop/test/test_sway.py)
- [ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/commissioning.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/commissioning.py)
- [ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/configuration.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/configuration.py)
- [ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/desktop.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/desktop.py)
- [ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/gui.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/gui.py)
- [ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/setup_checks.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/setup_checks.py)
- [ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/setup_wizard.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/setup_wizard.py)
- [ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/startup.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/startup.py)
- [ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/sway_analysis.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/sway_analysis.py)
- [ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/sway_recording.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/sway_recording.py)
