# V6.1 — Stop-priority, shared tracking, identity and verification stabilization

> **Update after this report:** the separate Servo position-history correction and the operator-reported disappearance of the wrist/arm oscillation are described in [Sway diagnostics](SWAY_DIAGNOSTICS.md) and [Commissioning](COMMISSIONING.md). The limitation recorded below reflects the state at the time of this report.

Date: 2026-10-19

## Baseline analyzed

- Branch: `main`
- HEAD: `489c2db694362b5dc3e5697856d0902002fbb976`
- Working tree was clean at the start of this release.

## Gate 0 audit — confirmed defects

| Suspected defect | Confirmed source behavior |
|---|---|
| `set_gripper_enabled(False)` blocked by a pending request | Yes. A single `gripper_request` returned early, so STOP ALL during ENABLE could leave authorization ambiguous. |
| STOP ALL did not withdraw the gripper heartbeat first | Yes. `stop_all()` called `stop_teleop()` then the service helper; heartbeat state was not changed before the service attempt. |
| Driver failure also stopped tracking | Yes. The partial scope was `{driver, teleop, tracking, rviz}`, leaving the gripper node without Joy/pose/optical inputs. |
| `ProcessManager.start()` refused retained children | Yes. Any retained tracking/gripper process raised `Stop the managed stack before starting another`. |
| `same_program()` used basenames | Yes. `/programs/a.urp` could match a different directory's `a.urp`. |
| Dashboard RUNNING was not continuously verified | Yes. `ProgramStartup` stopped polling after READY; the GUI displayed `PROGRAM: RUNNING` indefinitely. |
| Close did not wait for disarmed arm readback | Yes. Close initiated manager shutdown immediately after asynchronous disarm. |

Affected paths: `gui.py`, `desktop.py`, `program_control.py`, `dashboard_services.py`, `dashboard_process.py`, `startup.py`, `commissioning.py`, `gripper_node.py`, `gripper_worker.py`, and their tests. No motion algorithms, Servo/Ruckig tuning, calibration, tool TCP or speed limits were changed.

## Implemented fixes

### Stop-dominant gripper authorization

- New pure `gripper_control.py` with `READY → ENABLING → ENABLED` and `STOPPING → DISABLED` / `UNCONFIRMED`.
- STOP increments a request generation and is always sent, even while an ENABLE is pending. Late ENABLE acknowledgments from older generations are ignored.
- STOP/UNCONFIRMED immediately withdraws the gripper heartbeat. The heartbeat is restored only by a new explicit ENABLE.
- `/gripper/set_enabled` rejects ENABLE while the node's gripper heartbeat is stale, so a late pre-STOP request cannot re-enable follow.
- STOP is reported as confirmed only after fresh status shows `session_enabled: false` and `authorized: false`. Timeout/rejection/node loss displays `GRIPPER STOP UNCONFIRMED`, keeps the heartbeat withdrawn and blocks enabling.
- STOP ALL displays a pending banner until arm and gripper withdrawal are both confirmed. It remains a software stop.

### Shared tracking and process lifecycle

- Driver/teleop failure stops only `driver`, `teleop` and RViz; the libsurvive tracking bridge and gripper are retained. The GUI shows `ROBOT FAULT — GRIPPER AVAILABLE` only when fresh gripper status is connected, input-fresh and authorized.
- Tracking failure stops the arm groups and tracking; the gripper node remains for read-only diagnostics, and stale inputs prevent trigger motion.
- `ProcessManager.start()` with retained children no longer refuses or duplicates: it performs one full owned-stack stop/start. Full Stop ROS/Close still terminates all owned groups and releases the serial port.
- A partial-stop completion keeps the session token/manifest while any owned child remains; full completion clears them.

### Strict program identity and continuous verification

- `same_program()` now requires exact normalized path/name equality. A bare filename does not match a configured directory.
- After READY, the GUI re-reads `program_running` every second and `get_loaded_program` every three seconds. READY is displayed only while that verification is ≤2.5 s old.
- A stopped program, changed identity, failed query or timeout becomes `PROGRAM STOPPED`/`PROGRAM UNVERIFIED` and withdraws arm intent. The GUI never displays a stale cached state as verified RUNNING.

### Ordered Close/Restart

- Close and Restart withdraw arm intent and wait (bounded) for fresh backend status showing `enable_requested: false` and `armed: false` before initiating Dashboard program shutdown.
- Close also sends/confirms gripper disable, keeps the heartbeat withdrawn, and cancels pending program/gripper requests. A late enable cannot restore permission.
- Unconfirmed arm/gripper stops are reported explicitly while bounded fail-safe termination still proceeds.

## Automated verification executed

| Command | Result |
|---|---|
| `colcon build --symlink-install --packages-select ur_bus_servo_gripper ur_vr_teleop` | Both packages built; no errors |
| Full Python suite (`ur_vr_teleop`, `ur_bus_servo_gripper`, `ur_vr_sim_teleop`) | **513 passed**, 1 existing warning |
| `ros_gripper_check.py` (domain 147) | Passed, including heartbeat-gated late ENABLE rejection |
| `program_dashboard_check.py` (mocked Dashboard, domain 147) | Passed, including continuous running readback and confirmed stop |
| `ros_integration_checks.py` (domain 147) | Exit 0 |
| `gui_render_check.py` (domain 43) | Passed |

Not executed: C++ bridge/native-controller CTest and `servo_pipeline_check.py` (packages untouched); all physical robot/gripper tests. Physical validation remains **PENDING**. Never move the robot or gripper automatically during testing.

## Remaining hardware validation

Use the existing supervised checklist in [Commissioning](COMMISSIONING.md#v6-hardware-validation-checklist-pending-unless-actually-performed), plus:

1. STOP ALL during a pending ENABLE with the real gripper node; confirm the final state is disabled and no target packet follows.
2. Driver failure with a healthy tracking bridge: verify the gripper continues only with valid tracking and fresh status.
3. Close/Restart: verify fresh disarmed readback before Dashboard shutdown and no pendant program left running.
4. Dashboard program stop/identity change while telemetry is displayed: verify RUNNING disappears within the verification window.
5. At the time of this report, the previously reported UR10e wrist/arm oscillation remained unresolved; this work did not claim to fix it. The later Servo correction and operator-reported outcome are documented separately, as noted above.

## Known limitations

- A service request already executing at the server cannot be canceled locally. Stop-dominance is enforced by generation checks, heartbeat withdrawal, and node-side heartbeat gating; a late ENABLE may still set the session bit briefly but cannot drive without a fresh heartbeat and new pickup.
- If the gripper node is unavailable, stop is explicitly unconfirmed rather than successful.
- If Dashboard readback or the tracking bridge is unavailable, the GUI reports verification loss; it does not substitute synthetic data.
- Physical holding torque, grasp safety and stopping distance are not established by software tests.

## Rollback

1. Revert this change set with Git.
2. Rebuild `ur_bus_servo_gripper` and `ur_vr_teleop` with `colcon build --symlink-install --packages-select ...`.
3. Preserve `gripper.json`, `program_control.json`, `~/.config/libsurvive/config.json` and robot alignment.
