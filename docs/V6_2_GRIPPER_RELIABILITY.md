# V6.2 ST3215 feedback reliability

## Evidence and hypotheses

The reported 35 → 70/79/107 → restart → 35 °C sequence is an operator observation. No physical serial connection or motor motion was performed for this change. Physical root cause remains **PENDING**.

| Hypothesis | Evidence / unknown | Response in this change |
|---|---|---|
| Sensor/firmware transient or stale device data | A checksum-valid packet can contain an incorrect register value. No raw physical packets were available. | Preserve original bytes; independently read register 63 and corroborate normal results with a fresh full block. |
| Serial corruption | SDK detects checksum failure, but its additive 8-bit checksum cannot detect every corruption. Physical error rates unknown. | Separate corruption from checksum-valid high readings; never accept failed confirmation as normal. |
| Timeout / Linux scheduling / USB latency | Existing 20 ms override is shorter than SDK default (~50.24 ms for 21 response bytes). Real adapter latency was not measured. | Retain 20 ms; measure transactions and enforce deadline on every receive read. Do not invent evidence for a larger timeout. |
| Malformed response accepted by SDK | SDK slices requested data without checking response payload length. In a short packet the slice can include the checksum. | Require exact response length and ID before scalar or block decoding; validate byte count/range at adapter boundary. |
| Genuine heating / motor protection | Abrupt change and low value after restart do not rule out heating or firmware fault. | Every at/above-limit reading inhibits immediately; an independent high confirms the fault. Motor error bits remain immediate faults. |
| Supply, USB cable, adapter, grounding, electrical noise | No voltage-under-load, USB logs or supply measurements collected. | Record voltage/current/load and provide supervised observation procedure below. |

The original source was confirmed: `readTxRx(id, 56, 15)`, temperature at index 7/register 63, 20 ms packet timeout, and immediate `IOError` for nonzero status or one at/above-limit temperature. No temperature setting, travel bound, speed, acceleration, trigger mapping or arm algorithm was changed.

## Protocol and deadline findings

Verified the installed files byte-for-byte against the checksum-verified pinned archive. Reviewed `scservo_sdk` sources: `sms_sts.py`, `protocol_packet_handler.py`, `port_handler.py`, and `scservo_def.py`. The repository installer pins the Waveshare adapter archive SHA-256 `f39b7a07324b68571071ce8b36a8e1ded4da5215b77b883ea601dc8e7a21172b` (see [Gripper](GRIPPER.md#install-and-identify)).

The SDK sends an eight-byte read request. Full feedback has a 21-byte reply; temperature alone has a seven-byte reply. It resynchronizes on `ff ff`, checks ID/length/error header ranges and an inverted-sum checksum, and skips other IDs. It does not verify that the received payload length equals the requested length. Some parser branches and continuous other-ID packets can avoid its timeout check. The adapter now validates exact packet length/ID and checks the existing monotonic deadline before every serial read. Partial data at timeout can be classified as SDK corruption or receive-deadline timeout depending on which check detects it first; neither yields feedback.

The stock clock is wall time, the stock timeout adds a 50 ms latency allowance, and stock `clearPort()` calls output drain. Existing overrides remain: monotonic clock, nonblocking reads, exclusive pyserial ownership, 20 ms receive timeout, 20 ms write timeout, and input-buffer flush. No unbounded drain or retry is added. A read can use up to the write deadline plus receive deadline, with additional OS scheduling delay; this is not a real-time guarantee. Shutdown retains its bounded eight-second worker join, and discovery remains cancellable between transactions. Tests exercise shutdown during a pending actual SDK read on a pseudo-terminal.

Independent temperature reads use `readTxRx(id, 63, 1)` on the same serial owner. This is the same register transaction as `read1ByteTxRx`, while preserving the payload length for explicit validation. No EEPROM, ID, mode, torque or target operation occurs in confirmation.

## Temperature policy

| State | Action / exit |
|---|---|
| `NORMAL` | Fresh valid below-limit feedback permits existing authorization and pickup logic. |
| `TEMPERATURE_UNVERIFIED` | First at/above-limit reading withdraws session permission, clears pending motion/pickup/calibration and immediately blocks writes. Next worker cycle reads register 63 once. If normal, the next cycle reads the full feedback block once. |
| `TEMPERATURE_CONFIRMED_HIGH` | An independent at/above-limit value latches high temperature; close the link without torque release. Cool and inspect before deliberate read-only Connect. |
| `SENSOR_OR_COMMUNICATION_FAULT` | Failed, stale, malformed or overdue confirmation never implies normal temperature; close the link and require controlled Connect. Other worker faults also withdraw permission and retain their exact fault reason. |
| `RECOVERY_REQUIRED` | Both confirmation reads below limit indicate a suspected transient, not proof of its cause. Retain warning and inhibit; deliberately Connect read-only before ENABLE. |

Confirmation has at most two additional reads, one per worker cycle, and a 250 ms overall freshness/deadline budget. A register confirmation at/above limit needs no further read. A nonzero motor status/error faults immediately, including during confirmation. No retries continue after a failed confirmation. Repeated Connect cannot bypass an unfinished confirmation. Pending motion requests are discarded, not released later.

**Recovery:** review the event, resolve the cause/cool if necessary, choose **Connect / Read Servo**, observe fresh normal feedback, then explicitly **ENABLE GRIPPER** and repeat trigger pickup. Reconnect with high temperature enters verification again. Valid calibration, fresh tracking/trigger/heartbeat and all existing bounds remain required for follow. Calibration consent is retired and must be withdrawn before being deliberately granted again. No command, holding-torque release or automatic reauthorization is used to recover. A packet already transmitted cannot be revoked; software withdrawal does not guarantee physical standstill.

## Additive status contract (schema version 2 retained)

Existing topics/services and fields remain. `/gripper/status` adds:

- `temperature`: `state`, `verified_temperature_c`, monotonic `verified_at`, `suspicious_temperature_c`, `confirmation_readings`, `reason`. Verified history is not necessarily current feedback.
- `serial_diagnostics.counters`: successful `feedback_reads`, `rx_timeouts`, `rx_corruption`, `invalid_responses`, `motor_errors`, `other_errors`, `suspicious_temperatures`, `confirmed_high_events`, successful `connections`, subsequent `reconnects`, `disconnects`, `consecutive_failures`.
- `serial_diagnostics.timings`: count, mean, maximum and latest duration in ms for feedback, independent temperature, position-write, torque-write, ping and configuration-read transactions. Discovery timeout counts include unanswered candidate IDs; establish the hardware observation baseline after discovery. Write timings include response acknowledgment, not just wire transmission.
- `serial_diagnostics.last_transaction`: kind, duration, SDK communication result, motor error bits, wall timestamp, error category. `last_event` contains the latest rare event. A 32-entry in-memory ring retains detailed errors/temperature events for the node lifetime; counters persist across reconnects in that lifetime.
- `last_fault_reason`: retained fault/warning text across reconnects, shown in Gripper Setup.
- `last_temperature_event`: original feedback (including raw 15 bytes, decoded voltage, ticks, current/load, status and monotonic receipt), previous verified temperature, original transaction, wall timestamp, validation results and latest confirmation transaction. Retained across reconnects.
- Feedback adds `raw_bytes` and `transaction_ms`. Existing `feedback_valid` is false while temperature is unverified or latched; no suspect joint feedback is published as healthy. Diagnostic history must not be treated as a current measurement.

The dashboard and Gripper Setup share temperature wording. Unknown, confirmed-high and suspected-transient states are distinct; historical temperatures are labelled. Setup shows communication counters. A previous suspect value remains visible after healthy reconnect. Existing authoritative STOP acknowledgment behavior is unchanged.

## Supervised read-only hardware diagnostic procedure — PENDING

Keep arm and gripper authorization withdrawn and calibration consent off. Use the existing managed gripper owner and **Connect / Read Servo** only. Do not launch a second SDK process or open the USB adapter from another application. Do not run movement tests as part of this procedure.

In a terminal on the workstation:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=43
ros2 topic echo /gripper/status | tee /tmp/st3215-status.yaml
```

In another terminal:

```bash
dmesg --follow --human
```

If kernel log access is restricted, request access through the workstation administrator. Observe USB disconnect/reconnect messages and timestamps alongside status events. `ros2 node list` and `ros2 topic info /gripper/status --verbose` can check that exactly one gripper node/publisher exists. These ROS observers do not open the serial port.

Record an observation interval and starting/ending counters. Calculate timeout/corruption frequency from counter differences, separately from successful feedback reads; compare typical mean and worst observed full-block/register/write durations. Idle read-only observation produces no position-write measurements: those require separately authorized supervised motion and remain pending. Save the suspicious value, original bytes, voltage, current/load, motor bits, confirmations, receipt/event timestamps, permission state and reconnect count. Distinguish status publication time from feedback receipt and historical verified time. Record power interruptions and kernel events; do not infer causality solely from correlation.

With the mechanism secured and the workstation's electrical procedures followed, inspect USB cable/connectors, adapter and servo power connectors, supply rating/voltage, grounding, and routing near motors/industrial noise. A competent operator should measure supply stability under separately authorized load if needed. Readback voltage alone cannot exclude short power transients. Do not raise temperature thresholds or change firmware protections to hide the event.

## Automated evidence

Validation used the rebuilt Humble overlay and system Python 3.10. Tests use fake motors and pseudo-terminals only; ROS uses localhost domain 147. Physical USB latency, sensor correctness, actual heating, power integrity and mechanical stopping/holding performance remain unvalidated.


| Executed command (repository root unless stated) | Result |
|---|---|
| `colcon build --base-paths src --packages-select ur_bus_servo_gripper ur_vr_teleop --executor sequential` (inside `ur_teleop_ws`, system PATH, Humble sourced) | Both packages built successfully |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_bus_servo_gripper/test ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q` | 575 passed; one existing upstream Python compatibility warning |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py -q` | 91 passed |
| `/usr/bin/python3 ur_teleop_ws/src/ur_bus_servo_gripper/test/ros_gripper_check.py` | Passed: independent follow, loss/STOP, schema, thermal inhibition, rejection, reconnect, new pickup; one node and no arm publisher |
| `/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/gui_render_check.py` | Passed: actual Tk normal/unverified/transient/high/error text plus existing layout, settings and close checks; setup screenshot inspected |
| `/usr/bin/python3 -m compileall -q ur_teleop_ws/src/ur_bus_servo_gripper/ur_bus_servo_gripper ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop` | Passed |
| `git diff --check`; changed-document relative file/anchor checks | Passed; no Mermaid diagrams changed |

Development failures were corrected rather than suppressed: the former immediate-overtemperature assertion was updated for the intentional new state machine; new active-follow tests were corrected to count only writes after the suspicious sample; incomplete packets accept either SDK corruption or hard-deadline timeout classification. An intermediate GUI helper quoting error caused collection/byte-compilation failure; it was corrected and the overlay rebuilt before successful validation.

Unchanged C++ controller/bridge suites and separate Servo/native smoothing integration were not rerun; no files in those control paths changed. No physical validation, managed production desktop launch or hardware movement was performed.

### Coverage map

- `test_temperature.py`: normal hundreds-of-cycles history; isolated/repeated spikes; confirmed high and exact threshold; timeout/checksum/invalid confirmation; immediate motor status fault; stale/duplicate/overdue samples; active and holding cases; explicit reconnect with normal/still-high feedback; calibration retirement; STOP GRIPPER/STOP ALL/heartbeat/tracking/quit during confirmation; retained pickup/no replay and bounded event history.
- `test_sdk_transport.py`: actual pinned SDK over pseudo-terminals; register decoding and independent temperature read; checksum-valid high data versus corrupted, short, long, incomplete, wrong-ID and motor-error packets; all communication categories and invalid SDK results; injected latency; endless other-ID input deadline; exclusive ownership; port cleanup; worker shutdown during a pending read; confirmation sends only read/ping instructions, never position/torque writes.
- Existing gripper authorization, process lifecycle and arm tests remain active. GUI STOP generations/readback tests are retained unchanged. ROS independently exercises thermal recovery with fake hardware and fresh trigger pickup. The Tk test checks both dashboard and setup wording for each state and existing close behavior; worker shutdown under pending I/O is tested separately.

### Files changed

- `AGENTS.md`; `docs/GRIPPER.md`, `docs/TROUBLESHOOTING.md`, `docs/OPERATION.md`, `docs/COMMISSIONING.md`, this report.
- `ur_teleop_ws/src/ur_bus_servo_gripper/ur_bus_servo_gripper/`: `st3215_driver.py`, `gripper_worker.py`, `gripper_node.py`; new `temperature.py`, `serial_diagnostics.py`.
- `ur_teleop_ws/src/ur_bus_servo_gripper/test/`: `test_gripper.py`, `test_sdk_transport.py`, `ros_gripper_check.py`; new `test_temperature.py`.
- `ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/`: `gui.py`, `gripper_panel.py`; new `gripper_diagnostics.py`.
- `ur_teleop_ws/src/ur_vr_teleop/test/`: `test_gripper_gui.py`, `gui_render_check.py`.

The delivery commit SHA is provided with the final handoff; this report is part of that commit.
