# Engineering instructions for coding agents

## Scope and context

This repository teleoperates a UR10e from a Valve Index controller.
The supported platform is Ubuntu 22.04, ROS 2 Humble, and system Python 3.10.
The real stack uses libsurvive, MoveIt Servo, ros2_control, a custom Ruckig plugin, and the UR driver.
Treat robot-control changes as changes to a layered, supervised system.
Read [README](README.md) and the relevant [architecture](docs/ARCHITECTURE.md) section before editing.
For operator steps, use [Operation](docs/OPERATION.md); for validation, use [Commissioning](docs/COMMISSIONING.md).
Do not present implemented software behavior as hardware-validated performance.

## Codebase map

Paths below are relative to the repository root.

| Responsibility | Primary source |
|---|---|
| libsurvive acquisition and ROS tracking | `ur_teleop_ws/src/ur_vr_bridge/src/controller_bridge.cpp` |
| Optical signal guard | `ur_teleop_ws/src/ur_vr_bridge/include/ur_vr_bridge/optical_guard.hpp` |
| Bridge pose sanity | `ur_teleop_ws/src/ur_vr_bridge/include/ur_vr_bridge/pose_guard.hpp` |
| Source-time relay | `ur_teleop_ws/src/ur_vr_bridge/src/servo_stamp_relay.cpp` |
| Cartesian clutch/control/filtering | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/tracking.py` |
| Pose continuity/jump/freeze checks | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/tracking_quality.py` |
| Observation, TF, preview | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/preview.py` |
| Dashboard program start state machine | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/program_control.py` |
| Stop-dominant gripper authorization | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/gripper_control.py` |
| Bounded Dashboard shutdown adapter | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/dashboard_services.py` |
| Robot supervision and command orchestration | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/commissioning.py` |
| Joint command validation and gating | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/gate.py` |
| Shared freshness/joint validation | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/validation.py` |
| Servo and commissioning launch | `ur_teleop_ws/src/ur_vr_teleop/launch/commissioning.launch.py` |
| Native UR controller integration | `ur_teleop_ws/src/ur_teleop_controllers/src/smooth_scaled_controller.cpp` |
| Ruckig state/reference logic | `ur_teleop_ws/src/ur_teleop_controllers/include/ur_teleop_controllers/smoothing.hpp` |
| One-button workflow | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/startup.py` |
| Optional historical setup and legacy evidence import | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/engineering.py` |
| Configuration readiness | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/configuration.py` |
| Sway recording and offline analysis | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/sway_recording.py`, `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/sway_analysis.py` |
| GUI | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/gui.py` |
| Settings validation/persistence | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/preferences.py` |
| Managed process lifecycle/driver generation | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/desktop.py` |
| Process identity/ownership | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/process_ownership.py` |
| Joint diagnostics | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/joint_diagnostics.py` |
| Offline comparisons | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/comparison.py` |
| Camera viewer | `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/camera_view.py` |
| Shared original clutch/math | `ur_teleop_ws/src/ur_vr_sim_teleop/ur_vr_sim_teleop/core.py` |
| Separate mock simulator | `ur_teleop_ws/src/ur_vr_sim_teleop/launch/simulation.launch.py` |
| Camera capture/publisher | `ur_teleop_ws/src/ur_d405_camera/src/` |
| YAML defaults and saved overrides | `ur_teleop/config/real_teleop.yaml`, `ur_teleop/config/operator_settings.json` |
| Controller template | `ur_teleop/config/commissioning_controllers.yaml` |
| Robot kinematics | `ur_teleop/config/ur10e_calibration.yaml` |
| Operator startup | `ur_teleop/start_teleop.sh`, `ur_teleop/start_gui.sh` |
| Upstream revision pins | `dependencies.repos` |
| Python tests | `ur_teleop_ws/src/ur_vr_teleop/test/`, `ur_teleop_ws/src/ur_vr_sim_teleop/test/` |
| C++ tests | `ur_teleop_ws/src/ur_vr_bridge/test/`, `ur_teleop_ws/src/ur_teleop_controllers/test/` |

## Mandatory architecture rules

1. Inspect relevant existing implementations and dependency behavior before changing code.
2. Preserve ROS 2 Humble compatibility; use system Python and compatible C++17 dependencies.
3. Preserve ROS topics, types, services, frames, and semantics unless a deliberate migration is required.
4. Preserve operator startup through the managed desktop, including defaults and child ownership.
5. Maintain saved-settings compatibility where practical; validate and explicitly migrate changed schemas.
6. Keep tracking, Cartesian control, joint control, and robot supervision responsibilities separate.
7. Never introduce competing motion publishers or bypass the joint gate.
8. Never inadvertently combine incompatible position and velocity control loops.
9. Preserve coordinate frames, units, quaternion order, and rotation composition conventions.
10. Do not duplicate filters or create conflicting feedback/compensation stages.
11. Use source timestamps for differentiation; preserve source age through relays.
12. Keep bounded speed, acceleration, and jerk behavior and distinguish normal stops from hard faults.
13. Preserve A-button clutch reset, tracking-loss stop, and fresh-engagement behavior.
14. Never silently bypass robot state checks, watchdogs, collision checks, or protective functions.
15. Never treat one optical observation as proof of a valid 6-DoF pose.
16. Preserve diagnostics and clear blocking/failure messages; avoid misleading readiness labels.
17. Never introduce automatic physical robot motion in tests.
18. Never delete tests merely to obtain a passing build.

## Tracking invariants

Either calibrated Lighthouse is sufficient for optical presence.
At least one recent accepted paired sensor observation from either station establishes signal.
There is no four-sensor-per-station or mandatory two-station condition.
`any_station` is normal; legacy station policy names are compatibility aliases.
`single_station_motion_scale` is legacy and has no effect; do not restore a station-count penalty.
A valid pose still requires fresh source data, finite values, valid orientation, and continuity.
Retain stale, nonfinite, jump, exact-freeze, and tracking-loss rejection.
Loss of one station must not disarm solely on count if the remaining pose stays valid.
Loss/restoration must invalidate the old clutch reference and require release plus a new press.
Inspect the pinned libsurvive solver when modifying calibration or initialization assumptions.
Its geometric solver requirements are independent of the optical-presence threshold.
Preserve `~/.config/libsurvive/config.json` and robot alignment; neither is disposable test state.
Do not claim physical single-station cold start was verified by a unit test.

## Control stages and limits

The production command chain is:
bridge → TrackingClutch/Commissioning → Servo → timestamp relay → Gate → native scaled controller → UR driver.
Measured joints/TF and robot/Servo/controller state supervise that chain independently.
Commissioning normally uses velocity mode; standalone TrackingClutch's API default remains position.
Position mode follows a clutch-relative target using pose-error gain and optional measured-velocity damping.
Velocity mode differentiates distinct hand samples, filters velocity, and does not add that position loop.
A new velocity clutch captures a baseline and starts with zero demand.
Repeated timer callbacks must not differentiate the same sample again.
Change VR pose filters, deadbands, and velocity filters in `tracking.py` and their configuration paths.
Change pose quality in `tracking_quality.py`; optical acceptance belongs in the bridge guard.
Cartesian speed and acceleration limits belong upstream in TrackingClutch.
Servo converts Cartesian demand to joint targets with collision/singularity handling.
The gate checks age, joint membership, bounds, controller ownership, and independent readiness.
Native Ruckig bounds final joint-reference velocity, acceleration, and jerk at the driver update rate.
With supplied Servo velocities, the native controller must not add another position-error loop.
The position-only fallback is a distinct compatibility path; test it separately.
UR speed scaling, driver watchdogs, and robot protective functions remain independent.
`JointDiagnostics` is observational; its warning threshold is not an actuator limit.
Do not add direct `speedl`, `speedj`, or socket robot commands as an alternate teleop path.
See [Architecture](docs/ARCHITECTURE.md) for rates, command horizons, and stop timing.

## Configuration and runtime truth

Read [Configuration](docs/CONFIGURATION.md) before adding/changing a setting.
The order is defaults → selected YAML → adjacent validated operator preferences → effective snapshot.
Accepted runtime atomic updates can supersede startup teleop values.
The desktop passes the same snapshot to driver generation and commissioning.
A snapshot marked with `effective_sources` must not reread mutable preferences.
Legacy confirmation parameters reset false and no longer grant readiness. Routine Live requires explicit valid configuration and runtime readiness, not verified_setup.json. Never manufacture physical validation or persist enable intent.
Native joint limits and servoj tuning require restart; changing a ROS value alone is insufficient.
Verify effective runtime settings rather than assuming YAML values are active.
The committed saved linear speed is 1.5 m/s, versus YAML 0.10 m/s.
The saved alignment yaw is −40°, versus YAML −90°; preserve this distinction in reports.
Do not silently “correct” operator preferences in unrelated work.
GUI ratios are inverse stored scales; deadbands/time constants may have display-unit conversions.
If adding a tuning key, inspect DEFAULTS order and GUI indexing as well as validation and persistence.
Apply settings only while disarmed with A released; preserve atomic rejection behavior.
Changing alignment invalidates cached configuration readiness until current values are checked. It also invalidates prior physical mapping assumptions; numeric validation is not physical proof.
Do not rename legacy keys without checking saved-profile migration tests.
Do not bypass the calibration hash check to make a launch work on a different robot.

## Startup and process boundaries

The active source workspace is `ur_teleop_ws`; old duplicate builds are not authoritative.
The calibration workspace builds only `ur_calibration`; standard UR packages come from ROS packages.
`start_teleop.sh` opens the managed desktop in Live, disarmed/inactive. `--preview` selects engineering Preview with no final command publisher.
`start_gui.sh` opens only a panel and does not own a robot stack.
Default Live startup starts disabled with the motion controller inactive; it is not permission to move.
The separate domain-42 simulator uses GenericSystem and does not reproduce all production checks.
Production uses domain 43 and wall time; mock test domains are localhost 147/148.
The desktop stops only owned process groups after checking identity.
Preserve ownership tags, failure handling, and the distinction between required and optional children.
Do not use broad process killing or remove an unknown session lock to “fix” startup.
Camera/RViz failures are optional; driver/tracking/teleop failures stop the managed stack.
The D405 view is not calibrated collision sensing.

## Required development workflow

For every code change:

1. Inspect source, relevant dependencies, Git status, and existing tests; preserve unrelated user changes.
2. Identify affected nodes, parameters, interfaces, operator behavior, and test coverage.
3. Implement the smallest coherent change within the requested scope.
4. Add or update relevant tests for changed behavior; avoid tests that only mirror implementation.
5. Run relevant static checks and automated tests, including fault/recovery cases when affected.
6. Build affected ROS packages against Humble and the pinned/native dependencies.
7. Explain the behavioral change, ownership of limits, and compatibility implications.
8. Report commands and results actually executed; distinguish skipped or unavailable tests.
9. Identify remaining real-robot validation without automatically performing physical motion.

Documentation-only work should remain documentation-only; validate links, examples, and claims.
Use [Installation](docs/INSTALLATION.md#4-build) for the known build procedure.
Use [Commissioning](docs/COMMISSIONING.md#automated-validation) for test commands and isolation.
Run Python with `/usr/bin/python3`; do not let Conda silently select the ROS interpreter.
A stale installed overlay can hide source changes; rebuild/source the affected package before ROS tests.
Default pytest discovery omits the separately named integration/Servo/mock startup scripts.
Run `ros_integration_checks.py` explicitly when the ROS gate is affected.
Run `servo_pipeline_check.py` for Servo/relay integration changes.
Run `mock_startup_check.py` for driver description/controller/startup generation changes.
Use the C++ guard and native-controller tests for their respective components.
Never inject synthetic healthy robot states into a physical production domain.
Do not launch the managed real desktop as an unattended test substitute.
Keep hardware tests explicitly scoped to authorized supervised commissioning.

## Common errors to avoid

- Confusing managed Preview, standalone observation-only Preview, and the legacy mock simulator.
- Treating `/vr/controller/optical_fresh` as a reliable-pose flag.
- Restamping stale commands on receipt or differentiating duplicate samples.
- Applying RPY twice, mixing lighthouse/base frames, or swapping quaternion conventions.
- Adding position-error feedback to velocity-mode hand differentiation or native Servo velocities.
- Assuming saved joint-speed changes immediately alter the native controller.
- Calling software Disarm an emergency stop or guaranteeing zero physical stopping distance.
- Saying all faults clear Enable intent; some recover to readiness, but still require a new clutch press.
- Assuming an old project note describes current policy, settings, or launch behavior.
- Claiming automated tests validate physical occlusion, payload/TCP, or one-station cold start.

## Documentation maintenance

Update README or the appropriate guide when changing startup commands, architecture, or dependencies.
Update the parameter tables when changing defaults, ranges, persistence, units, or restart behavior.
Update the interface tables when changing ROS topics, services, frames, types, or publication timing.
Update Operation when changing operator controls, readiness, confirmations, or recovery behavior.
Update Commissioning/Troubleshooting when changing tests, safety-related behavior, or failure messages.
Keep source links and example commands valid; check Mermaid syntax when editing diagrams.
Preserve useful historical notes, but label their obsolete assumptions rather than copying them forward.
Keep this file actionable; put detailed parameter inventories and procedures in `docs/`.
Do not add nested AGENTS files unless a package has genuinely distinct engineering requirements.

V4 coordinator requests existing services only; the backend remains authoritative. STOP must retire in-flight requests and withhold heartbeat. Unknown enable outcomes require a fresh backend session before retry. Configuration file checking runs off the command executor; missing/invalid/stale configuration blocks motion. Historical commissioning evidence and software identity are not routine readiness dependencies. The optional commissioning observation tool records observed checks plus supervised physical assessments, never checkbox-only proof; it never opens automatically and its records do not grant motion permission. `--engineering-session` is a deprecated compatibility option, with no readiness bypass. Optional physical observations remain explicitly supervised.

V5.1 Preview capture lives in `calibration_capture.py` (pure bounded engine), `calibration_observer.py` (dedicated read-only ROS executor) and `calibration_panel.py` (Tk only). Keep capture isolation separate from A-released `SetupChecks.preview()`. Preserve source ordering, post-release fresh pose/flag corroboration, managed-session/read-only-parameter/controller-manager/graph checks, bounded mailboxes and joined shutdown. Never infer an absolute mapping from arbitrary hand motion. `setup_source_compatibility.json` permits only the exact reviewed observational backend hash transition; never broaden it to hide a motion-control change. Run `calibration_ros_check.py` explicitly in addition to the documented regression/GUI checks.

V6 gripper authorization is independent: `/gripper/set_enabled` (SetBool) latches a session permission, `/gripper/operator_present` is the gripper heartbeat, and normal follow must not consult arm `enable_requested`, `motion_ready`, `motion_output_enabled`, armed state, A, Servo, controller ownership or External Control. Connect/read/save are read-only and robot-independent; calibration motion needs explicit gripper setup consent, never fabricated arm feedback. STOP GRIPPER withdraws target advancement without opening or releasing; STOP ALL withdraws both; loss of GUI heartbeat withdraws follow. Keep the serial worker's pre-write recheck, bounded travel, pickup and no-replay rules. Arm/driver/tracking failure stops only those process groups; a full Stop ROS/close terminates the gripper too.

V6 Dashboard start/shutdown lives in `program_control.py` (pure state machine), `dashboard_services.py` (bounded ROS adapter) and `gui.py` (async adapter). Verify Remote Control, safety mode, RUNNING mode, approved program identity, Dashboard program-running and fresh driver readiness; a Play acknowledgment alone is never READY. Never change Remote Control, reset safety faults, release brakes, power the robot or execute arbitrary URScript. Program stop must be requested and confirmed via Dashboard readback before signaling the UR driver on Stop ROS/restart/close; a timeout is reported as unconfirmed while bounded termination still proceeds. Keep `headless_mode:=false` unless a deliberate separate mode migration is reviewed. The program identity override is `program_control.json` beside the selected YAML.

V6.1 makes gripper STOP dominant: `/gripper/set_enabled` ENABLE is rejected unless a fresh gripper heartbeat is present, and GUI request generations discard late ENABLE acknowledgments after STOP. Keep `gripper_control.py`, automatic heartbeat restoration only after an explicit ENABLE, and `GRIPPER STOP UNCONFIRMED` when status readback is unavailable. Program identity is compared as an exact normalized path/name; never infer it from a basename. READY requires fresh periodic Dashboard program-running/identity readback. Keep the tracking bridge alive when only the UR arm subsystem fails, and use a full owned-stack stop/start when retained children exist. Close/Restart wait for fresh disarmed arm readback before Dashboard program shutdown.
