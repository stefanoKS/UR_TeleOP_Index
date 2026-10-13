# Operator guide

Complete [installation](INSTALLATION.md) and [commissioning](COMMISSIONING.md) before a physical session. Review the effective settings shown by the panel: saved preferences can override the YAML substantially. The current saved profile is not a low-speed commissioning profile.

## Start the application

Use the workstation's **UR10e Teleop** shortcut or run:

```bash
~/Documents/Teleop/ur_teleop/start_teleop.sh
```

The desktop starts **Live automatically, with motion disarmed and the scaled controller inactive**. Press START TELEOP only when the work area is ready. A matching saved setup is required for routine operation. On first launch, the [Initial Setup Wizard](INITIAL_SETUP.md) opens automatically after backend verification. It manages the robot driver, tracking, commissioning nodes, and optional camera/RViz. Preview connects to real robot state but does not publish teleop joint commands. It expects motion controllers to remain inactive. If you need a completely hardware-free environment, use the [mock simulator](COMMISSIONING.md#automated-validation).

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
| START TELEOP | Asynchronously prepare the controller, start Servo if needed, request enable, then wait for actual backend arming |
| STOP TELEOP | Interrupt startup, clear enable intent, and request a software stop; always available |
| Engineering / diagnostics | Explicit Preview/Live selection, managed restart/stop, individual preparation, legacy supervised controls, full backend diagnostics |
| Open RViz / D405 camera | Optional visualization; camera is not collision protection |
| Apply changes | Submit the edited named settings to the running backend while disarmed with A released |
| Apply + save | Save accepted persistent tuning for next launch; a changed joint speed still requires Restart ROS |
| Revert edits / Restore defaults | Revert to current running values, or stage built-in defaults for review; neither silently changes a running stack |

Status explains what is blocking motion. Measured TCP and joint feedback continue when A is released. “Optical present” alone is not “tracking ready”: a valid, fresh pose and continuity checks must also pass.

## Preview and tracking readiness

1. Keep A released and motion controllers inactive.
2. Confirm the expected controller identity and fresh 6-DoF pose. Either Lighthouse 0 or Lighthouse 1 is sufficient when its calibration and pose solution are valid.
3. Check measured `tool0` against the actual robot/tool arrangement. A custom pendant TCP is not automatically the model's `tool0`.
4. In Preview, verify the sign of all three translations and rotations with small hand movements. Confirm the alignment only after checking the actual mapping.
5. Verify optical loss with motion controllers inactive. Both stations blocked must remove readiness; one station remaining can preserve readiness if its pose remains valid.
6. In an explicit supervised engineering session, apply the two legacy session confirmations only after those checks. Record the evidence under [V4 commissioning](COMMISSIONING.md#v4-engineering-record). Routine operators do not submit these confirmations.

If tracking has been recalibrated or a station moved, repeat alignment verification. Do not use a station-count policy switch to repair bad tracking; the supported policy is already `any_station`.

## Choose a mode and settings

**Velocity** is the default. Hand speed requests tool speed; stopping the hand removes velocity demand after filtering/deceleration. A press captures a fresh baseline, so the first sample cannot create a velocity spike.

**Position** follows a relative target from where the hand and robot were when A was pressed. A displaced hand can continue requesting movement until the robot reaches that target. Tracking gain and measured-tool damping affect this mode.

Release A and disarm before switching modes or applying tuning. Mapping fields show a **VR:robot ratio**: 1.2 means 12 cm of hand travel requests 10 cm of robot travel. The persisted `translation_scale` is its inverse. Deadbands are displayed in millimetres/degrees; some time constants are displayed in milliseconds.

A saved joint-speed change requires restarting the ROS stack because the final native controller reads its limits at startup. The panel retains the current live limit until restart. Check the new effective value afterward. Do not assume clicking Save has altered a startup-only hardware-controller parameter. The panel initially loads effective running values from the commissioning status; edited values are visibly marked as unapplied. Saved preferences are loaded only at the next managed-stack snapshot, where they override YAML tuning as described in Configuration.

## Authorized live session

1. Open the application. It starts the managed Live stack disarmed.
2. Press **START TELEOP** once, with A released. Preparation waits for real robot, tracking, joints/TF and controller ownership status.
3. If instructed, start the External Control program on the pendant. The application does not press Play, change robot mode, or release protective stops.
4. Wait for **READY — PRESS A TO MOVE**, derived from backend `armed` and `motion_ready`.
5. Hold A to move; release A to request a controlled stop.
6. Press **STOP TELEOP** to clear enable intent. A new START is required afterward.

A previously commissioned, unchanged installation needs no daily checkboxes or separate Servo/controller buttons. “Open Setup Wizard” means setup is missing, damaged, changed, or outside its verified tuning envelope. Do not invent confirmations to clear it. Effective TCP speed caps and the active final joint limit remain visible even while editing drafts. Saved high limits are not evidence of approval.

Missing hardware prerequisites wait without repeated clicks. Failed services show the failing stage and require a controlled START retry after correction. An **enable service timeout with unknown outcome requires Restart ROS**: local cancellation cannot undo a request already received by the server. STOP remains available; the GUI withholds its heartbeat to prevent a delayed enable from restoring motion permission.

Tracking loss preserves the existing bounded recovery: release A, wait for valid tracking and stationary readiness, then press A again. Healthy Servo/controller components are not prepared again. ROS/backend restart or GUI heartbeat loss requires a new START. STOP is a software stop, not an emergency stop.

## Stop and recover

- **Normal stop:** release A. This requests zero Cartesian demand and downstream stopping; it is not an instantaneous physical halt.
- **Disarm:** click **STOP TELEOP** to clear enable intent. The panel explicitly describes this as a ramped software stop.
- **Emergency:** use the pendant/physical emergency stop under the site's procedure. Software Disarm is not a replacement.
- **Tracking fault:** stop hand movement, release A, and diagnose the cause. Restored tracking while A is held cannot resume motion. Once healthy and stationary, the stack may rearm if Enable intent survived; a new A press is required. If enable intent was cleared, press START TELEOP again after checks.
- **Protective stop:** investigate and recover through the robot's established procedure. The application does not silently clear it.

Losing one station should not stop a valid remaining pose. If it does, inspect pose quality and timing rather than assuming a hidden two-station rule.

## Shutdown and evidence

Release A, disarm, and wait for measured motion to stop. Close the desktop to stop its owned stack; Engineering **Stop ROS** is also available. Follow the pendant procedure for ending External Control. The desktop stops only processes it owns; unrelated ROS processes are not automatically killed.

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
not submit changes or alter session checkboxes. Restore Defaults only stages
editable defaults; hidden compatibility settings remain intact. A mapping edit
clears the local alignment checkbox so an old check cannot approve a new mapping.

In explicit supervised engineering sessions, legacy checkboxes express draft intent. Routine Live uses the locally saved verified setup. The checklist uses backend confirmation;
checking a box alone shows **Pending application**, not **Verified**. Backend
mapping invalidation revokes verification. Restart clears local session checks;
stale status shows **Unknown / status unavailable**.

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

Engineering Preview is available under **Engineering / diagnostics** or with `start_teleop.sh --preview`; the separate domain-42 simulator remains hardware-free. The technical panel is intended for authorized engineers, and its manual Enable is disabled during routine V4 Live operation.

To temporarily use the previous supervised workflow, stop the owned stack and close the app, then run `ur_teleop/start_teleop.sh --preview --engineering-session`. Under Engineering, select Live, perform the documented mapping and optical-loss checks, apply the session confirmations, prepare the controller, start Servo, and use **Enable (supervised commissioning)**. This explicit mode uses the original session checks; it does not fabricate a V4 record or clear robot protections. Return to the normal launcher to restore saved-setup startup.

Normal setup uses **Settings → Recalibrate / Initial Setup**; see the [wizard guide](INITIAL_SETUP.md). The legacy expert commands above remain available for debugging.
