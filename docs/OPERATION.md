# Operator guide

Complete [installation](INSTALLATION.md) and [commissioning](COMMISSIONING.md) before a physical session. Review the effective settings shown by the panel: saved preferences can override the YAML substantially. The current saved profile is not a low-speed commissioning profile.

## Start the application

Use the workstation's **UR10e Teleop** shortcut or run:

```bash
~/Documents/Teleop/ur_teleop/start_teleop.sh
```

The desktop starts Preview automatically. It manages the robot driver, tracking, commissioning nodes, and optional camera/RViz. Preview connects to real robot state but does not publish teleop joint commands. It expects motion controllers to remain inactive. If you need a completely hardware-free environment, use the [mock simulator](COMMISSIONING.md#automated-validation).

`start_gui.sh` opens only the operator panel for an existing stack. It does not start tracking or the driver. Avoid launching duplicate panels or bridges: duplicate command publishers and missing ownership checks can block readiness.

## Understand the dashboard

The operator panel is a single resizable dashboard. At the supported 1366×768
layout, session controls, readiness, Enable, Disarm, live tracking status, and
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
| Preview / Live setup | Restart managed ROS in the selected output mode; Live still starts disabled |
| Stop ROS | Stop this desktop's owned processes |
| Open logs | Session logs, commands, and effective configuration |
| Open RViz | View model, measured tool pose, and requested movement |
| Open D405 camera | Optional RGB/depth viewer; not collision protection |
| Start Servo solver | Start Cartesian-to-joint solver; does not enable robot motion |
| Prepare controller | Request activation of the scaled controller when prerequisites pass; cannot evict a competing controller |
| Enable teleop | Request readiness/arming; A still controls active movement |
| DISARM TELEOP | Clear enable intent and request a software stop |
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
6. Apply the two session confirmation checkboxes only after those checks. They are intentionally not saved as future motion permission.

If tracking has been recalibrated or a station moved, repeat alignment verification. Do not use a station-count policy switch to repair bad tracking; the supported policy is already `any_station`.

## Choose a mode and settings

**Velocity** is the default. Hand speed requests tool speed; stopping the hand removes velocity demand after filtering/deceleration. A press captures a fresh baseline, so the first sample cannot create a velocity spike.

**Position** follows a relative target from where the hand and robot were when A was pressed. A displaced hand can continue requesting movement until the robot reaches that target. Tracking gain and measured-tool damping affect this mode.

Release A and disarm before switching modes or applying tuning. Mapping fields show a **VR:robot ratio**: 1.2 means 12 cm of hand travel requests 10 cm of robot travel. The persisted `translation_scale` is its inverse. Deadbands are displayed in millimetres/degrees; some time constants are displayed in milliseconds.

A saved joint-speed change requires restarting the ROS stack because the final native controller reads its limits at startup. The panel retains the current live limit until restart. Check the new effective value afterward. Do not assume clicking Save has altered a startup-only hardware-controller parameter. The panel initially loads effective running values from the commissioning status; edited values are visibly marked as unapplied. Saved preferences are loaded only at the next managed-stack snapshot, where they override YAML tuning as described in Configuration.

## Authorized live session

1. Establish the physical work area and pendant stop access according to the site's robot procedure.
2. Select **Live setup** in the desktop. This allows a command publisher to exist; it does not enable motion.
3. Run the prepared External Control program on the pendant. Check robot state, safety state, speed scaling, joints, TF, and tracking in the panel.
4. Use **Start Servo solver** and **Prepare controller** as required. Prepare needs a released clutch, a disarmed and stationary robot, and valid robot readiness. It will not stop a competing controller for you.
5. Review mode, mapping, and actual effective speed/acceleration limits. Complete and apply the session confirmations.
6. Select **Enable teleop**. Wait for readiness with A released.
7. Press and hold A. Start with the small supervised movements defined in the commissioning procedure. Release A to end each movement.

Enable does not clear a protective stop, start the pendant program, or activate a missing controller automatically. Never use command-line trajectory publishers to work around a blocked panel.

## Stop and recover

- **Normal stop:** release A. This requests zero Cartesian demand and downstream stopping; it is not an instantaneous physical halt.
- **Disarm:** click **DISARM TELEOP** to clear enable intent. The panel explicitly describes this as a ramped software stop.
- **Emergency:** use the pendant/physical emergency stop under the site's procedure. Software Disarm is not a replacement.
- **Tracking fault:** stop hand movement, release A, and diagnose the cause. Restored tracking while A is held cannot resume motion. Once healthy and stationary, the stack may rearm if Enable intent survived; a new A press is required. If Enable was cleared, enable again after checks.
- **Protective stop:** investigate and recover through the robot's established procedure. The application does not silently clear it.

Losing one station should not stop a valid remaining pose. If it does, inspect pose quality and timing rather than assuming a hidden two-station rule.

## Shutdown and evidence

Release A, disarm, and wait for measured motion to stop. Use **Stop ROS**, then close the desktop. Follow the pendant procedure for ending External Control. The desktop stops only processes it owns; unrelated ROS processes are not automatically killed.

Session material is under `ur_teleop/logs/`: component logs, `commands.json`, `effective_config.yaml`, and `effective_settings.log`. These record startup settings; accepted runtime changes may be newer. Capture current panel/status and ROS parameter values when diagnosing a session. Logs and recordings are ignored by Git.
