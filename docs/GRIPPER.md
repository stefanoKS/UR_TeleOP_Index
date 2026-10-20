# Single ST3215 rack-and-pinion gripper

The optional `ur_bus_servo_gripper` process controls one ST3215 through the Waveshare Bus Servo Adapter (A). It has no UR command publisher and does not change Cartesian control, Servo, Ruckig, servoj, alignment, TCP, A-button clutch, or camera processing. Software tests do not establish physical grip force, stopping distance, or safe mechanical travel.

## Install and identify

Use Ubuntu 22.04, ROS 2 Humble and `/usr/bin/python3`. Install `python3-serial` with apt, then run the SDK installer from the repository root:

```bash
sudo apt-get install python3-serial
/usr/bin/python3 ur_teleop/scripts/install_gripper_sdk.py
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
source /opt/ros/humble/setup.bash
cd ur_teleop_ws
colcon build --base-paths src --packages-select ur_bus_servo_gripper ur_vr_teleop --executor sequential
source install/local_setup.bash
```

The installer extracts only `scservo_sdk/*.py` into the system Python user's site-packages. It pins the vendor archive SHA-256 and refuses an existing SDK directory or a changed archive. Do not run the vendor's example movement, ID-changing, or wheel-mode programs during installation.

Sources inspected:

- [Adapter documentation](https://docs.waveshare.com/Bus_Servo_Adapter_A)
- [Python usage](https://docs.waveshare.com/ST3215_Servo/Python-Use)
- [ST register table](https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation)
- [Compatible adapter SDK archive](https://files.waveshare.com/wiki/Bus_Servo_Adapter_A/STServo_Python.zip), SHA-256 `f39b7a07324b68571071ce8b36a8e1ded4da5215b77b883ea601dc8e7a21172b`.

The Python usage page's older HAT archive uses `STservo_sdk.sts`; use the adapter archive above for `scservo_sdk.sms_sts`. Installed signatures were inspected: `PortHandler(port_name)`, `openPort()`, `setBaudRate(baudrate)`, `sms_sts(portHandler)`, `ping(scs_id)`, `ReadPos(scs_id)`, `ReadSpeed(scs_id)`, `ReadPosSpeed(scs_id)`, `readTxRx(scs_id, address, length)`, and `WritePosEx(scs_id, position, speed, acc)`. Read methods return values followed by communication result and motor error; write returns communication result and motor error. Every result is checked.

The adapter needs a compatible external motor supply; USB alone is not motor power. Select `/dev/serial/by-id/...` in **Settings → Gripper Setup**, or enter a manual `/dev/ttyUSB...` path. An empty port auto-selects only when exactly one by-id device exists. With ID `-1`, Connect pings IDs 0–253 and requires exactly one responding servo. It does not assume ID 1, alter IDs, or save discovery automatically. A configured ID is pinged directly. The SDK reads mode, angle limits, resolution, phase and actual feedback. Position mode 0, resolution 1 and a nonzero single-turn firmware angle range are required; other modes need engineering review. It never changes EEPROM or operating mode.

Linux access must already permit opening the device. Inspect ownership with `ls -l /dev/serial/by-id/ /dev/ttyUSB*` and arrange the appropriate `dialout` membership/udev policy through the workstation administrator; log in again after group changes. Do not relax permissions globally or kill another serial owner. The driver uses an exclusive port lock, nonblocking reads, 20 ms packet/write deadlines and a monotonic SDK clock. It replaces the SDK's unbounded output-drain call with input-buffer clearing. Discovery can take about six seconds; it runs outside Tk and ROS callbacks and is cancellable between transactions.

## Setup and persistence

The managed desktop starts the gripper process as an optional child. `gripper.process_enabled: false` in the selected YAML disables that process; restart the managed stack after changing this setting. A missing SDK/device or gripper process failure does not stop arm-only operation. There is no gripper-required arm mode.

The separate adjacent `gripper.json` defaults to disabled and uncalibrated when absent. No calibration or servo ID is shipped. The desktop passes its path explicitly; the gripper does not read or change `operator_settings.json`. Connect, read and save do **not** require the UR driver, a fresh arm session, disarmed arm state or A released. Physical calibration motion requires a deliberate gripper calibration consent and a clear, supervised mechanism; the application never fabricates arm feedback. Settings are independent of the arm's effective parameter snapshot.

1. Open **Settings → Gripper Setup**. For a manual connection, enter port/ID, leave follow disabled and save the draft before Connect. Connect/Read Servo is read-only and can run with the robot unavailable. Feedback is continuously read while connected; Read Current Position requests the next read.
2. Confirm the detected ID and actual encoder position. Capture buttons copy real feedback, never the last target, into OPEN/CLOSED. Capturing also copies the detected ID. Review the travel bounds and both endpoints independently. A disabled draft can remain incomplete; movement and enabled profiles require valid distinct endpoints inside the reviewed range.
3. Before jogging, establish a small reviewed interval around the current position and provisional distinct endpoints indicating direction; save it. Check **Authorize low-speed calibration** only after confirming the mechanism and workcell are clear and supervised. This consent is gripper-owned and independent of the arm. Jog Open/Close Direction uses the saved endpoint ordering, a small increment, and the reviewed bounds. It never searches for a hard stop.
4. Capture/refine OPEN and CLOSED inside the mechanism's safe travel. Either encoder direction is supported. Save Settings atomically replaces the JSON after validation, flush and fsync; failed replacement preserves the previous file. Save withdraws calibration consent and reconnects read-only. Restore Settings loads the saved file into the editor; it does not move anything.
5. Test Open/Close only by deliberate button press with calibration consent. **Test trigger (arm disarmed)** allows the same pickup/follow policy with calibration consent and reduced target rate; it does not enable the UR arm. Closing the panel or clearing the checkbox withdraws calibration consent. The consent is a 100 ms heartbeat and expires after 250 ms. STOP TELEOP also withdraws calibration consent, but does not disable normal gripper follow.
6. Observe the displayed raw trigger and observed range while deliberately releasing/pressing the physical controller with healthy tracking. Enter and save its measured minimum/maximum (and inversion if needed). The observed range is informational, not automatic calibration.
7. Enable trigger follow in the saved profile for normal sessions. No repeated endpoint calibration is required unless the mechanism changes.

Saving does not command movement. The first deliberate authorized movement may enable RAM holding torque **after** writing a bounded position target if torque was off. Connection, discovery, startup, and saving never enable torque or send position targets. Firmware damping/other torque states are rejected.

### Settings (JSON version 1)

All changes take effect on a read-only reconnect/reload, while stationary and disarmed. GUI labels distinguish SDK raw units from software ticks/s.

| Key | Default | Validation / units |
|---|---:|---|
| `enabled` | false | Boolean; permit normal follow after an explicit **ENABLE GRIPPER** and fresh gripper heartbeat. START TELEOP is not required. |
| `port` | empty | `/dev/serial/by-id/...` preferred; `/dev/...` fallback |
| `servo_id` | -1 | -1 discovery or integer 0–253; enabled profile requires verified ID |
| `open_ticks`, `closed_ticks` | 0, 0 | Integer 0–4095; distinct for movement |
| `min_ticks`, `max_ticks` | 0, 0 | Reviewed travel, contains both endpoints for movement |
| `speed` | 100 | Integer 1–1000 SDK raw units; zero (unlimited) forbidden |
| `acceleration` | 5 | Integer 1–254 SDK raw units; zero (maximum) forbidden |
| `rate_ticks_s` | 100 | 1–500 software target ticks/s |
| `trigger_min`, `trigger_max` | 0, 1 | Finite; span at least 0.01; replace with observed calibration |
| `deadzone` | 0.02 | 0 ≤ value < 0.25, endpoint fraction deadzones |
| `invert` | false | Boolean trigger inversion |
| `threshold_ticks` | 2 | Integer 1–20; suppress small target changes |
| `jog_ticks` | 5 | Integer 1–20 per deliberate jog |
| `temperature_limit` | 60 | Integer 20–70 °C; fault at/above threshold |
| `stall_error_ticks` | 40 | Integer 5–200 requested-position error |
| `stall_seconds` | 2 | 0.2–5 s without at least two ticks of progress |

No settings alter firmware limits/offsets, disable motor protection, or treat wraparound as a short path. More than one revolution requires an explicit engineering review, not an automatic mode change.

## Operation and withdrawal policy

Normal operation: **ENABLE GRIPPER** when the gripper profile is calibrated and connected, synchronize the trigger, then use A for the arm and the trigger for the gripper. START TELEOP, Servo, External Control and the UR driver are **not** required for gripper follow. Releasing A does not open or close the gripper and does not withdraw follow. Gripper follow separately requires: a latched session enable from `/gripper/set_enabled`, an enabled calibrated profile, fresh serial feedback, a healthy motor, fresh Joy/pose/optical bridge signals, the gripper operator heartbeat, and deliberate trigger pickup. The node rejects ENABLE while the gripper heartbeat is stale/withdrawn, so a late request sent before STOP cannot re-enable follow. STOP is dominant over an in-flight ENABLE; the GUI discards a late ENABLE acknowledgment by request generation. A gripper fault does not withdraw arm authorization. Conversely, an arm fault does not stop an otherwise healthy gripper session while the shared tracking bridge remains healthy.

Pickup requires a fresh deliberate trigger excursion of at least 5 percentage points and then matching the measured encoder-derived closure within 3 percentage points. No target is written during pickup. This repeats after an explicit new **ENABLE GRIPPER**, reconnect, tracking/input loss, backend session change or authorization withdrawal. START TELEOP is not a gripper pickup trigger. A stationary matching trigger alone cannot re-enable follow. If the actual position is outside the endpoint interval, use supervised calibration to bring it into the reviewed interval.

The fraction is normalized, optionally inverted and rescaled between endpoint deadzones. The target is `round(open + fraction * (closed - open))`, clamped to the reviewed travel. Examples 1000/2300 at fractions 0/.25/.5/.75/1 yield 1000/1325/1650/1975/2300 when deadzone is zero. The serial worker runs at at most **25 Hz**, retains only latest input and one pending explicit action, and never queues a stream of motor commands. Software target increments use elapsed monotonic time capped at 40 ms and the configured ticks/s limit; target lead is at most 20 ticks from the latest feedback. Calibration additionally caps target rate at 25 ticks/s, SDK speed at 50 and acceleration at 5.

The vendor speed field is phase/firmware dependent; raw speed feedback is deliberately reported as raw. SDK acceleration uses a device-specific scale. Their physical effectiveness with streaming setpoints has **not** been verified. Software setpoint bounds are not measured shaft-speed/acceleration guarantees. No arm smoothing stage is changed.

**STOP GRIPPER, stale/invalid tracking, loss of the gripper heartbeat, or input loss:** stop advancing targets, retire pickup/calibration state, and leave the last acknowledged bounded target and holding torque unchanged. STOP immediately publishes/retains a false gripper heartbeat. A packet already in flight cannot be revoked. The servo may finish its small remaining target travel; this is not an emergency stop or zero-distance guarantee. Neither input loss nor a bridge-generated zero triggers automatic opening. Joy header age/order, DDS source age, independent pose/optical corroboration after Joy, unique publisher counts and duplicate-node detection are checked. Humble's executor adapter retains DDS metadata otherwise discarded by its standard Python executor. **STOP ALL** additionally withdraws arm enable intent and displays a pending banner until fresh status confirms both withdrawals.

STOP is reported as **GRIPPER STOPPED** only after fresh `/gripper/status` shows `session_enabled: false` and `authorized: false`. If the disable service times out, is rejected, or the node disappears, the GUI displays **GRIPPER STOP UNCONFIRMED**, keeps the heartbeat withdrawn, and blocks ENABLE until status resolves the uncertainty or the operator explicitly retries STOP. It never reports an unconfirmed stop as successful.

**USB/feedback failure or motor error:** no further position packets, invalidate feedback, close the link, and require explicit Connect. No automatic reconnect/replay. Status also exposes `attempted_ticks`, `write_started_wall_ns` and `write_acknowledged`. A pending command whose acknowledgment failed has an unknown outcome; inspect actual feedback after reconnect. Persistent requested error without encoder progress faults rather than continually advancing into an obstruction. An object, obstruction and jam are not distinguished as a successful grasp. Holding torque is never automatically disabled to recover a fault. Loss of motor power can release the mechanism, and holding position does not guarantee a safe grasp. Load/current are diagnostics, not direct force control.

**Temperature at/above the unchanged configured limit:** immediately inhibit target advancement and withdraw permission. A read-only register-63 confirmation and, if normal, a fresh full-block confirmation distinguish confirmed high temperature, suspected transient and communication failure. No automatic resumption: review, cool if needed, Connect read-only, explicitly ENABLE and repeat pickup. Holding torque is unchanged. See [V6.2 policy, diagnostics and hardware procedure](V6_2_GRIPPER_RELIABILITY.md).

## ROS observation contract

All topics are outside the UR command chain. Queue depth is one; no transient-local replay.

| Topic | Type | Meaning |
|---|---|---|
| `/gripper/status` | `std_msgs/String` | Schema-version-2 JSON observation, published 25 Hz; `stamp` ROS publication time; `feedback_wall_ns` feedback receipt time; `command_wall_ns` last acknowledged write time; nullable raw/normalized trigger, requested fraction, commanded ticks; `session_enabled`, `heartbeat_fresh`, `authorized`, `setup_authorized`, `duplicate_owner`, `profile_enabled`, `calibration_complete`; connection, feedback validity, input freshness, follow state, reason; identity/config and diagnostic feedback |
| `/gripper/target_fraction` | `std_msgs/Float64` | Latest valid operator request 0–1, including during pickup; not measured closure; use timestamped status for recording |
| `/gripper/target_ticks` | `std_msgs/Int32` | Emitted only on acknowledged position commands, integer encoder ticks; use status for timestamp |
| `/gripper/joint_states` | `sensor_msgs/JointState` | Fresh actual encoder feedback only; joint `gripper_motor_joint`, position = ticks × 2π/4096 radians; zero is the servo's configured encoder zero, positive is increasing ticks. No velocity/effort values are invented from phase-dependent speed or load |
| `/gripper/calibration_present` | `std_msgs/Bool` | Explicit panel calibration consent heartbeat; independent of the arm |
| `/gripper/operator_present` | `std_msgs/Bool` | Gripper-owned GUI heartbeat; loss withdraws follow after 250 ms |

The additive V6.2 `temperature`, `serial_diagnostics` and `last_temperature_event` fields are documented in the [status contract](V6_2_GRIPPER_RELIABILITY.md#additive-status-contract-schema-version-2-retained). Feedback is marked invalid during temperature uncertainty; historical verified temperatures must not be displayed as current.

Status feedback includes `ticks`, signed `speed_raw`, signed `load_raw`, `voltage_v`, `temperature_c`, signed `current_raw`, `motor_status`, `moving`, and monotonic `received_at`. Current is the register raw value (nominal 6.5 mA/unit); load is the signed motor-drive duty register, not force. Faults clear current feedback availability; commanded position is never substituted. Recorders must use feedback validity/age and distinguish request, acknowledged command and measured state. Encoder angle is not jaw opening. No rack displacement or jaw-gap measurement is claimed, and no LeRobot recorder/exporter is implemented.

Services `/gripper/{connect,read,reload,jog_open,jog_close,open,close,follow}` use `std_srvs/Trigger`. `/gripper/set_enabled` uses `std_srvs/SetBool` and latches the independent session enable; it never inherits a saved `enabled` value after restart. ENABLE is rejected unless the gripper heartbeat is fresh; STOP is always accepted. A Trigger success means queued, **not** completed movement; observe status for actual results. One pending action is accepted; requests expire after 500 ms. All motion services recheck gripper-session/calibration authorization in the serial worker. There is no direct external target topic and no arm enable/motion-ready dependency.

## Automated validation

With overlays rebuilt/sourced and no competing test nodes, run serially on isolated localhost domain 147:

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=147
/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_bus_servo_gripper/test -q
/usr/bin/python3 ur_teleop_ws/src/ur_bus_servo_gripper/test/ros_gripper_check.py
/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/mock_startup_check.py
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/calibration_ros_check.py
```

The transport tests use a pseudo-terminal and actual SDK packets; worker/ROS tests use fake motors. No automated test uses a physical serial adapter or sends physical movement. See [Commissioning](COMMISSIONING.md) for the remaining arm checks. A display is required for GUI checks.

## Supervised physical validation still required

1. Establish safe power, mounting, workspace, payload, mechanical stop margins and emergency procedures. Keep the arm disarmed.
2. Detect the adapter, ping and confirm the actual servo ID. Read actual encoder, firmware mode/limits and diagnostics without motion.
3. Review a small safe interval around current position. Deliberately authorize one very small, slow jog and verify opening direction.
4. Establish OPEN and CLOSED inside mechanical limits, without striking stops, and save them. Never automatically command 0 or 4095.
5. Validate intermediate positions, backlash, load/current/temperature interpretation, physical speed and stopping/holding behavior. Verify settings against the motor firmware.
6. Measure the actual trigger range, validate pickup and proportional control using the explicit disarmed trigger test.
7. Test stale Joy, tracking loss, USB disconnect, feedback failure, STOP and reconnect. Verify no automatic opening/replay and appropriate load retention without treating it as guaranteed.
8. Under authorized supervision, validate combined arm/gripper operation and A-release independence; keep D405 status separate.

Implementation-session hardware limits: the by-id adapter was visible, but read-only opening failed with **permission denied**. No servo ID, physical mode, endpoint, trigger range, motor supply, or physical speed/force was verified. No physical motor motion was commanded and no live calibration was created.
