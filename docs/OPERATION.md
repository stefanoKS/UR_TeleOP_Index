# Operator guide

Complete [installation](INSTALLATION.md) and [commissioning](COMMISSIONING.md) before a physical session. Review the effective settings shown by the panel: saved preferences can override the YAML. The committed saved profile selects Position and higher limits than the YAML; it is not a commissioning recommendation, and the effective runtime values remain authoritative.

## Start the application

Use the workstation's **UR10e Teleop** shortcut or run:

```bash
~/Documents/Teleop/ur_teleop/start_teleop.sh
```

The desktop starts **Live automatically, with motion disarmed and the scaled controller inactive**. Press START TELEOP only when the work area is ready. No wizard or historical setup record is required and none opens automatically. Configuration must explicitly supply the robot calibration, mapping and frames. The desktop manages the robot driver, tracking, commissioning nodes, and optional camera/RViz. Preview connects to real robot state but does not publish teleop joint commands. It expects motion controllers to remain inactive. If you need a completely hardware-free environment, use the [mock simulator](COMMISSIONING.md#automated-validation).

`start_gui.sh` opens only the operator panel for an existing stack. It does not start tracking or the driver. Avoid launching duplicate panels or bridges: duplicate command publishers and missing ownership checks can block readiness.

## Understand the dashboard

The operator panel is a single resizable dashboard. At the supported 1366×768
layout, START TELEOP, STOP TELEOP, tracking/robot status, and
quick tuning remain visible without scrolling. The header shows the output mode
(Preview/Live) separately from actual readiness. A green optical indicator is
not a valid-pose or robot-ready claim.

Quick tuning contains the control mode, mapping ratios, and TCP speed limits.
Use **Show advanced tuning** for motion, tracking, position-only, and
engineering settings. Position-only fields are hidden in Velocity mode; hiding
them does not reset their values. The camera button opens the existing optional
viewer in a separate window; it is not collision sensing.

| Control or display | Meaning |
|---|---|
| START TELEOP | Asynchronously prepare the controller, start or recover Servo as needed, request enable, then wait for actual backend arming; a prominent banner names the current stage and blocker |
| STOP TELEOP | Interrupt arm startup, clear enable intent, and request a software stop; does not stop an independently enabled gripper |
| ENABLE GRIPPER / STOP GRIPPER | Latched gripper-session follow permission; works before START TELEOP, with A released or held, and without the UR driver or External Control |
| STOP ALL | Withdraws both arm and gripper motion permission; still a software stop, not an emergency stop. Displays pending withdrawal until fresh status confirms both. |
| Engineering / diagnostics | Explicit Preview/Live selection, managed restart/stop, External Control program identity, individual preparation, optional observations and read-only sway recording |
| Open RViz / D405 camera | Optional visualization; camera is not collision protection |
| Apply changes | Submit the edited named settings to the running backend while disarmed with A released |
| Apply + save | Save accepted persistent tuning for next launch; a changed joint speed still requires Restart ROS |
| Revert edits / Restore defaults | Revert to current running values, or stage built-in defaults for review; neither silently changes a running stack |

Status explains what is blocking motion. Measured TCP and joint feedback continue when A is released. “Optical present” alone is not “tracking ready”: a valid, fresh pose and continuity checks must also pass.

## Preview and tracking readiness

1. Keep A released and motion controllers inactive.
2. Confirm the expected controller identity and fresh 6-DoF pose. Either Lighthouse 0 or Lighthouse 1 is sufficient when its calibration and pose solution are valid.
3. Check measured `tool0` against the actual robot/tool arrangement. A custom pendant TCP is not automatically the model's `tool0`.
4. In Preview, verify the sign of all three translations and rotations with small hand movements. Assess physical alignment; the software does not certify it.
5. Verify optical loss with motion controllers inactive. Both stations blocked must remove readiness; one station remaining can preserve readiness if its pose remains valid.
6. Record physical findings through the site commissioning process; the [optional observations tool](INITIAL_SETUP.md) can help. No session confirmation booleans or eleven-step evidence record are required for startup.

If tracking has been recalibrated or a station moved, repeat alignment verification. Do not use a station-count policy switch to repair bad tracking; the supported policy is already `any_station`.

## Choose a mode and settings

**Velocity** is the built-in and YAML default. Hand speed requests tool speed; stopping the hand removes velocity demand after filtering/deceleration. A press captures a fresh baseline, so the first sample cannot create a velocity spike.

**Position** is selected by the committed saved operator profile. It follows a relative target from where the hand and robot were when A was pressed. A displaced hand can continue requesting movement until the robot reaches that target. Tracking gain and measured-tool damping affect this mode.

The saved file does not prove what a currently running session uses. Check the effective mode shown by the panel or `/vr_real/operator_status`; a saved change applies only in the next managed-stack snapshot.

Release A and disarm before switching modes or applying tuning. Mapping fields show a **VR:robot ratio**: 1.2 means 12 cm of hand travel requests 10 cm of robot travel. The persisted `translation_scale` is its inverse. Deadbands are displayed in millimetres/degrees; some time constants are displayed in milliseconds.

A saved joint-speed change requires restarting the ROS stack because the final native controller reads its limits at startup. The panel retains the current live limit until restart. Check the new effective value afterward. Do not assume clicking Save has altered a startup-only hardware-controller parameter. The panel initially loads effective running values from the commissioning status; edited values are visibly marked as unapplied. Saved preferences are loaded only at the next managed-stack snapshot, where they override YAML tuning as described in Configuration.

## Authorized live session

1. Open the application. It starts the managed Live stack disarmed.
2. Press **START TELEOP** once, with A released. When the approved External Control `.urp` is configured, the app checks Remote Control, safety and robot mode, verifies/loads the program, presses Dashboard Play, and requires Dashboard program-running plus fresh driver readback before controller preparation. Otherwise it waits for the pendant-run program.
3. The app never changes Remote Control mode, resets protective/emergency stops, releases brakes, powers the robot, or executes arbitrary URScript. A Local Control result displays the exact pendant instruction.
4. Wait for **READY — PRESS A TO MOVE**, derived from backend `armed` and `motion_ready`.
5. Hold A to move; release A to request a controlled stop.
6. Press **STOP TELEOP** to clear enable intent. A new START is required afterward.

A prominent banner names the current startup stage and blocker (for example `WAITING — RELEASE A`, `PREPARING CONTROLLER`, `STARTING SERVO`, `WAITING — SERVO STATUS`, `WAITING — CONTROLLER CONFLICT`, `WAITING — EXTERNAL CONTROL`, `WAITING — INVALID TRACKING`, `READY — PRESS A TO MOVE`). The dashboard also shows the A-button input, the backend motion authorization and the effective clutch separately; the clutch is never labelled ACTIVE merely because A is held while the gate is disarmed.

A previously commissioned, unchanged installation needs no daily checkboxes or separate Servo/controller buttons. A configuration error names the missing or inconsistent input. Resolve it in the selected configuration; do not assume a numeric RPY value proves physical alignment. Effective TCP speed caps and the active final joint limit remain visible even while editing drafts. Saved high limits are not evidence of approval.

Missing hardware prerequisites wait without repeated clicks. Failed services show the failing stage and exact response, and require a controlled START retry after correction. An **enable service timeout with unknown outcome requires Restart ROS**: local cancellation cannot undo a request already received by the server. STOP remains available; the GUI withholds its heartbeat to prevent a delayed enable from restoring motion permission.

A Servo whose status has expired is never treated as healthy. If a previously started Servo goes stale, START TELEOP attempts one bounded `start_servo` restart; if fresh status still does not arrive, or the Servo process is missing, startup fails and displays the exact **Restart ROS (Engineering → Restart ROS)** instruction instead of waiting indefinitely. A Start Servo timeout with unknown outcome withholds further automatic starts until fresh Servo status or a ROS restart.

If the backend session changes (a genuine commissioning-backend restart), the GUI disarms, retires in-flight requests and clears old session state, then waits for fresh backend status; press START TELEOP again. Repeated session changes are reported with the number of distinct session IDs as a possible duplicate-backend condition. A managed child crash names the process, its exit reason and the log path, and stops the owned stack without restarting motion.

Tracking loss preserves the existing bounded recovery: release A, wait for valid tracking and stationary readiness, then press A again. Healthy Servo/controller components are not prepared again. ROS/backend restart or GUI heartbeat loss requires a new START. STOP is a software stop, not an emergency stop.

## Stop and recover

- **Normal stop:** release A. This requests zero Cartesian demand and downstream stopping; it is not an instantaneous physical halt.
- **Disarm:** click **STOP TELEOP** to clear enable intent. The panel explicitly describes this as a ramped software stop.
- **Emergency:** use the pendant/physical emergency stop under the site's procedure. Software Disarm is not a replacement.
- **Tracking fault:** stop hand movement, release A, and diagnose the cause. Restored tracking while A is held cannot resume motion. Once healthy and stationary, the stack may rearm if Enable intent survived; a new A press is required. If enable intent was cleared, press START TELEOP again after checks.
- **Protective stop:** investigate and recover through the robot's established procedure. The application does not silently clear it.

Losing one station should not stop a valid remaining pose. If it does, inspect pose quality and timing rather than assuming a hidden two-station rule.

## Shutdown and evidence

Release A, disarm, and wait for measured motion to stop. Closing the desktop or Engineering **Stop ROS** first retires further Start/Enable requests, withdraws arm and gripper permission, requests Dashboard Stop while the UR services are still alive, and requires program-running readback before terminating the managed driver. If that confirmation is unavailable, the app reports the bounded fallback and warns that a pendant socket-disconnect popup may still occur. The desktop stops only processes it owns; unrelated ROS processes are not automatically killed. **Restart ROS** uses the same ordered stop.

Session material is under `ur_teleop/logs/`: component logs, `commands.json`, `effective_config.yaml`, and `effective_settings.log`. These record startup settings; accepted runtime changes may be newer. Capture current panel/status and ROS parameter values when diagnosing a session. Logs and recordings are ignored by Git.

## Dashboard settings and engineering confirmations

The parameter-state line distinguishes **No pending edits**, **Unsaved parameter
changes**, invalid text, applying, waiting for confirmation, restart required,
and unverified running values. Initial defaults are not assumed to be running.
Apply waits for a fresh complete backend snapshot, disarmed state and released A.
Rejected drafts remain editable, with the backend's rejection reason visible.

**Apply** changes the current session. **Apply + Save** also atomically saves the
submitted tuning after backend acceptance. A successful service response shows
**Applied — awaiting readback** until matching running values arrive. If saving
fails, **Applied to current session, but saving failed** does not imply rollback.
A five-second readback timeout releases the pending form lock and keeps the
outcome warning visible. Review fresh values before retrying.

The active native joint limit and saved restart-pending limit appear separately.
Restart ROS remains an explicit operator action; the pending label clears only
when backend and final-controller readback match the saved value. Revert restores
confirmed runtime values, retains the separate saved restart setting, and does
not submit changes. Restore Defaults only stages editable defaults; hidden compatibility settings remain intact. A mapping edit requires physical review before operating.

Configuration readiness checks explicit mapping, frames, robot calibration and current typed tuning. It does not claim that mounting, TCP/payload or physical alignment has been assessed. Stale backend status is unavailable, never ready.

External changes to a field being edited preserve the draft and show a conflict.
Use Revert to accept current values, then re-enter intended edits. This avoids
silently overwriting another operator's observed changes.

Only the advanced pane scrolls. The pointer wheel and keyboard focus scroll that
pane without moving operation controls or quick tuning. Position-only fields
hide in Velocity mode while preserving their text. Action notifications are
separate from live readiness: successes remain for five seconds; errors remain
until dismissed or corrected. **Action history** shows the last 40 actions and
all current blockers. Long process messages are available there in full.

## Engineering and rollback

Engineering Preview is available under **Engineering / diagnostics** or with `start_teleop.sh --preview`. The separate domain-42 simulator remains hardware-free. Optional commissioning observations are accessible in Engineering tools; they never open automatically. `--engineering-session` is accepted for old launch commands but does not alter readiness or provide a manual-enable bypass.

## Record sway

Open **Engineering / diagnostics → Advanced diagnostics / sway recording** and press **RECORD SWAY DIAGNOSTICS** before the separately authorized diagnostic session. Recording does not start or enable teleoperation. Press **STOP RECORDING** afterward and wait for the saved/failed message. STOP TELEOP remains separate and always visible. Camera availability does not affect recording. See [analysis and interpretation](SWAY_DIAGNOSTICS.md).

## Loading the sway correction

After rebuilding the native-controller and teleop packages, stop teleoperation and restart the managed stack to load the new Servo plugin. Starting a new panel alone does not reload it. Production now leaves Servo joint increments intact and uses native Ruckig for final joint smoothing; existing speed, acceleration, jerk and safety checks remain active. Saved tuning and alignment are unchanged.

The correction removes the delayed position-history feedback term identified in the October 15 recording while preserving VR hand filtering, Cartesian bounds, collision/singularity handling and final Ruckig limits. Synthetic Servo regression results are documented in [Sway diagnostics](SWAY_DIAGNOSTICS.md). The operator reports that the earlier wrist/arm oscillation disappeared after this correction; a separately documented post-fix physical measurement was not available in this audit. In supervised commissioning, record a comparable slow movement and A-release stop and compare Servo, final-reference and measured motion. Do not infer physical stopping performance from synthetic tests.

## Change the tool tip / rotation centre

1. Press **STOP TELEOP**, release A, and wait for fresh disarmed status.
2. Define/calibrate and select the intended TCP on the UR pendant.
3. Open **Settings → Tool tip / rotation centre**. Select **Read TCP from robot**.
4. Review XYZ in millimetres and Rx/Ry/Rz as a **rotation vector in radians**, not RPY. Manual entry is also available. Click **Save for restart**.
5. Close the editor and use **Engineering / diagnostics → Restart ROS**. The new session starts disarmed. The tip is now `teleop_tcp` in both the robot model and Servo.
6. Check the orange/RGB target marker at the tip in Preview (the separate measured `tool0` axes remain the flange reference); then perform authorized, supervised low-speed tests of translation, rotation about the tip, clutch release and tracking-loss stops. Review payload and tool collision geometry separately.

Saving does not change an already-running session. A configured profile requires a successful current read of the pendant TCP; a mismatch or unavailable read blocks readiness. Correct/select the intended pendant TCP, or read/review/save the new profile and restart. A fresh clutch release/press is required after faults. This monitor is a polled software check, not an instantaneous pendant-change interlock or a robot protective function.

The editor saves `tool_tcp.json` beside the selected YAML, tied to its robot IP. Existing installations without this profile or a YAML `robot.tcp_offset` keep their original `tool0` behavior. A zero offset explicitly selects the flange point while retaining TCP consistency checks. Do not change the robot calibration file to represent a tool.

The added tip link contains no physical tool geometry. Robot collision checks remain active, but an attached tool needs its own reviewed collision model; the TCP point alone does not describe its shape.

## Optional gripper

Open **Settings → Gripper Setup** to connect read-only, capture real encoder endpoints, review travel, and save. Connection, diagnostics and saving work without a robot connection. Normal trigger follow requires **ENABLE GRIPPER**, a saved calibrated profile, the gripper heartbeat and deliberate trigger pickup; it does not require START TELEOP, A, Servo, the native controller or External Control. A release stops only arm demand. STOP GRIPPER withdraws target advancement and preserves holding torque; it does not guarantee a safe grasp. STOP ALL withdraws both subsystems. See [Gripper operation and supervised calibration](GRIPPER.md).

A short VR/USB interruption now holds the gripper and resumes follow automatically after verified coherent trigger input (with pose/optical gates), heartbeat and motor feedback are verified, provided the original session stayed enabled and the interruption was within the configured window (2 s by default). Do not press/release the trigger to force recovery during that window. A longer interruption, STOP, or a fault displays **PICKUP REQUIRED**; move the trigger deliberately to re-engage. See [continuous-follow behavior](V6_3_GRIPPER_CONTINUOUS_FOLLOW.md).

### External Control program identity

Use **Settings → External Control program** (or the Engineering button) to enter and verify the approved `.urp` and enable automatic Dashboard start. The override is saved atomically beside `real_teleop.yaml`. Verify that Dashboard reports the same loaded program before enabling automatic start. This panel never loads an unrelated program or commands motion.

For ST3215 temperature warnings, follow the [V6.2 gripper recovery procedure](V6_2_GRIPPER_RELIABILITY.md#temperature-policy). Normal confirmation never resumes motion: review the warning, Connect read-only, explicitly ENABLE GRIPPER and repeat trigger pickup.

V6.4 pickup accepts a deliberate crossing of measured closure; an exact sampled match is unnecessary. **TRIGGER INPUT UNVERIFIED** means the displayed cache is historical and movement is held. **CONTROLLER SIGNAL LOST** means recent device activity is unavailable. Brief verified recovery resumes automatically; a long interruption or STOP requires pickup. Gripper Setup shows event age, provenance, pickup method/error and transition reason. See [supervised checks](V6_4_TRIGGER_RELIABILITY.md#supervised-hardware-commissioning-still-required).
