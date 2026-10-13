# V4 one-button startup implementation report

Implemented locally against `ddcad899e8e378aa62c71dee5f6feacbd17447b8`; remote HEAD matched that revision when checked. The pre-existing edit to `ur_teleop/config/operator_settings.json` was preserved. No physical robot session was launched, no engineering record was issued, and no hardware performance is claimed.

## Operator result

The normal launcher starts the managed Live stack with motion disarmed and the scaled controller inactive. For a V4-commissioned installation: **open → START TELEOP → wait for READY → hold A to move**. Release A requests a controlled stop; STOP TELEOP clears intent and interrupts pending startup.

Separate Preview/Live selection, Servo start, controller preparation and session verification are removed from ordinary operation. Engineering tools retain the relevant individual controls. The normal status panel contains TRACKING and ROBOT indicators; combined observation validators still run and their detailed reasons remain in Advanced diagnostics. Effective TCP caps and the active native joint limit remain visible independently of draft edits.

## Changed implementation

| File | Main changes/functions |
|---|---|
| [startup.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/startup.py) | New pure `Startup.start/step/acknowledge/stop/fail` state machine |
| [gui.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/gui.py) | `operations`, `start_teleop`, `stop_teleop`, `poll_startup`, `on_status`, `poll`, `update_dashboard`, `close`: asynchronous adapter, cancellation/retirement, two main controls, technical controls hidden, truthful status and limits |
| [engineering.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/engineering.py) | New identity/envelope verification, explicit engineering attestation CLI, atomic record creation |
| [commissioning.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/commissioning.py) | `check_engineering`, `engineering_ready`, `engineering_controller_matches`, `reasons`, `tracking_valid`, `robot_preparation_reason`, controller-list/status handling: evidence gating and additive status fields; file hashing off command executor |
| [desktop.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/desktop.py) | `parse_options`, `main`, `commands`: Live default, explicit Preview and supervised engineering-session option |
| [preferences.py](../ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/preferences.py) | `installation_root`, `write_effective_snapshot`: preserve original installation/record identity through snapshots without rereading preferences |
| [commissioning.launch.py](../ur_teleop_ws/src/ur_vr_teleop/launch/commissioning.launch.py) | `nodes`, `generate_launch_description`: explicit record-required Live policy and read-only context; engineering session is opt-in |
| [setup.py](../ur_teleop_ws/src/ur_vr_teleop/setup.py) | Installed `engineering_record` command |
| Tests | New `test_startup.py`, `test_engineering.py`, `test_startup_launch.py`; expanded desktop, isolated ROS and actual Tk/ROS checks |
| Documentation | README, AGENTS, Architecture, Operation, Commissioning, Configuration, Operator Status, Troubleshooting, this report and screenshots; `.gitignore` excludes local workcell records |

No control-law, optical-guard, pose-quality, gate, native Ruckig or UR-driver implementation was changed. Existing ROS command/service interfaces and operator preference schema remain compatible; status fields and launch options are additive.

## Startup and cancellation

The coordinator waits for stack/status and heartbeat, released A, valid engineering evidence, actual robot/program/speed scaling, valid pose/joints/TF, stationary joints and fresh controller ownership. It activates only the existing scaled controller when inactive and no competing motion controller owns interfaces. It never requests deactivation of a competitor.

It then starts the existing Servo solver only if needed, waits for fresh status and complete backend readiness, requests the existing `set_enabled` service, and displays READY only after backend `armed && motion_ready`. An already prepared controller or started Servo is reused. Once Servo status has been seen, stale status cannot cause duplicate start calls.

Each stage issues at most one request per attempt. Acknowledgment is separate from status readback. Five-second service/readback timeouts fail visibly; physical prerequisites wait without repeated clicks or automatic service retries. Fix a failed service and press START for a controlled retry.

STOP retires the workflow, publishes absent operator heartbeat and requests disarm. In-flight requests are retained until acknowledged/timed out and followed by another disarm; late responses cannot continue startup. An unknown enable outcome requires a new ROS/backend session before another START. Old-session results are ignored. GUI close stops the existing owned-process worker and retires requests. No blocking ROS waits or sleep-based startup chain were added.

## Commissioning and protections

Routine Live no longer depends on daily checkbox submissions. It requires a matching, unexpired engineering evidence record with an identified engineer/workcell and durable report. Identity covers robot calibration, tracking calibration, transformation/frames, installation/host, controller serial, relevant source/dependency/template files and fixed tuning. Approved control modes and speed/acceleration/ratio envelopes are explicit. Changes outside the approved envelope, mapping/calibration changes, missing/expired evidence or stale verification block readiness. Native controller limits must match the startup configuration.

The record is administrative evidence, not an automatic physical test or cryptographic proof. Workstation permissions must separate authorized engineers. Physical tool/workcell/Lighthouse movement without a corresponding file change cannot be detected by this record; the site must revoke it and recommission. See [engineering procedure](COMMISSIONING.md#v4-engineering-record).

Runtime pose continuity/freshness, optical loss, TF/joints, GUI heartbeat, Servo halt/collision/singularity, controller ownership, command uniqueness/age, native limits, speed scaling and UR protective checks remain active. The A-button clutch and stationary/fresh-press recovery are unchanged. Tracking recovery does not repeat healthy software preparation and cannot resume motion with A held from before the loss. Software STOP is not an emergency stop.

Either calibrated Lighthouse with a continuing valid 6-DoF pose remains sufficient; there is no two-station requirement, four-sensor minimum, or one-station speed multiplier. The operator sees one or two Lighthouses without an absent-second-station warning. Raw optical signal with invalid pose remains blocked. Unit and C++ checks do not establish physical one-station cold-start performance.

## Validation actually executed

All ROS synthetic tests used localhost isolation; none targeted physical production domain 43. The GUI and commissioning constructors' environment guard is satisfied by the fixtures, but their actual ROS context is domain 147. Mock driver uses domain 148 and GenericSystem/loopback. Native CTest was run with the sourced workspace and localhost domain 149.

| Command / check | Result |
|---|---|
| `colcon build --base-paths src --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3` from `ur_teleop_ws` | All five packages built |
| Final `colcon build --base-paths src --packages-select ur_vr_teleop --executor sequential` | Passed after final Python changes |
| `/usr/bin/python3 -m pytest src/ur_vr_teleop/test src/ur_vr_sim_teleop/test src/ur_vr_teleop/test/ros_integration_checks.py -q` from sourced workspace | **415 passed**, one existing xacro/Python warning |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py` | Passed actual Tk/ROS Apply/Save/rejection/readback, one-button service order, double Start, STOP, reuse, close and layouts |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/servo_pipeline_check.py` | Passed; 201 Servo targets, 0.01-second horizons, no robot command publisher |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/mock_startup_check.py` | Passed generated driver parameters, GenericSystem, six joints and base-to-tool TF |
| `ctest --test-dir ur_teleop_ws/build/ur_vr_bridge --output-on-failure` | 2/2 passed |
| `ctest --test-dir ur_teleop_ws/build/ur_teleop_controllers --output-on-failure` | 2/2 passed with workspace sourced |
| Python compilation, shell syntax, Markdown local links/anchors and `git diff --check` | Passed; edited Mermaid sequence label reviewed for syntax |

An initial native CTest invocation without the workspace overlay could not discover the custom plugin; sourcing the workspace resolved it. Xvfb was unavailable, so GUI validation used the working display. The Servo check caught a launch-context naming collision during development; it was fixed, parameter-evaluation regression tests were added, and the check passed afterward. These were resolved software/test-environment failures, not hardware tests.

Remaining physical validation: site-selected low-speed profile, TCP/payload, calibrated workcell/collision model, actual stop response, both approved control modes, tracking occlusion/recovery and one-station cold starts, then signed V4 review. The saved 1.5 m/s cap and −40° alignment were preserved, not approved by this work.

## Screenshots

These are actual Tk captures with visibly labeled synthetic status, no robot connected. Primary controls were checked for visibility at each size. Main views and the 1280×720 advanced view were inspected.

- [1280×720](images/v4-dashboard-1280x720.png)
- [1366×768](images/v4-dashboard-1366x768.png)
- [1440×900](images/v4-dashboard-1440x900.png)
- [1280×720 advanced tuning](images/v4-dashboard-1280x720-advanced.png)

## Rollback

Close the managed app after STOP and verified physical standstill. Launch `ur_teleop/start_teleop.sh --preview --engineering-session` to use the prior supervised workflow through Engineering tools, including its original per-session confirmations and manual Enable. This does not create a V4 approval or disable runtime safeguards. Return to the normal launcher for record-based operation. For a source-version rollback, use a separate checkout of the baseline revision above and rebuild it, preserving local calibration and preferences; do not discard the user's working-tree settings edit.
