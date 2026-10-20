# V6.3 ST3215 continuous follow and transient recovery

## Scope

This change is limited to the independent ST3215 gripper:

- `ur_bus_servo_gripper/gripper_worker.py`
- `ur_bus_servo_gripper/gripper_mapping.py`
- `ur_bus_servo_gripper/gripper_config.py`
- `ur_bus_servo_gripper/gripper_node.py`
- `ur_vr_teleop/gripper_diagnostics.py`
- `ur_vr_teleop/gui.py`
- `ur_vr_teleop/gripper_panel.py`

Arm tracking, Cartesian control, MoveIt Servo, Ruckig, the gate, the UR driver, Dashboard program management and A-button clutch behavior are unchanged. The gripper remains independent of arm enable intent, Servo, External Control, START TELEOP and A.

## Confirmed code-level root causes

These were verified by tracing the current source. They are mechanisms, not a claim that every field report has the same physical cause.

1. **Follow continuity was destroyed by any invalid sample.** `Worker.update()` incremented the authorization epoch on any `pose`/`optical`/`joy` false/None transition (and on stale-key refresh). `step()` reacted to every epoch change with `Follow.reset()`, so a single 40 ms tracking gap discarded the active pickup and required a new 5%/3% trigger excursion.
2. **Any invalid input also recreated `Follow`.** The non-authorized branch executed `self.follow = Follow(self.config)` each cycle, discarding `active`, `reference` and the pickup baseline even when the session was still valid.
3. **Heartbeat staleness and explicit STOP were conflated.** The node computed `value = msg.data and source_fresh(info)`. A delayed delivery of a true heartbeat therefore became `False`, indistinguishable from an operator STOP. Separately, the GUI publishes at 10 Hz while the worker's freshness window was 250 ms, so occasional Tk/executor scheduling delay could look like loss.
4. **A synthetic zero Joy could pass the corroboration test.** During bridge loss the bridge publishes a zero-filled Joy. The node accepted it as a valid Joy sample if source timestamps were current, then could hand it to the worker when pose/optical recovered; that is not operator input.
5. **Target bookkeeping advanced before transmission.** `Follow.step()` set `last_sent = target` when it merely planned a command. If the pre-write authorization check rejected the packet, suppression state had already advanced and the intended target could be lost until another threshold or rate change.
6. **Any feedback read exception was immediately terminal.** `driver.read()` exceptions propagated to `handle_fault()`, which inhibited the session, disconnected and required manual Connect even for a single isolated timeout.

Field reports of repeated trigger pressing are consistent with causes 1–3. The exact USB/VR electrical causes of every interruption remain unconfirmed without hardware logs.

## Old versus new behavior

| Situation | V6.2 behavior | V6.3 behavior |
|---|---|---|
| One pose/optical/Joy gap | Pickup state reset; operator had to release/press | `SIGNAL_PAUSED`; no writes; automatic resume after verified input |
| Heartbeat message slightly late | Treated as loss and follow reset | `SIGNAL_PAUSED`; recoverable if the session remains valid |
| Explicit false heartbeat | STOP | Latched STOP until a new explicit ENABLE |
| Recovery after short loss | New 5% excursion required | Rebase on fresh measured feedback; bounded target uses current trigger |
| Long loss | Pickup required | Still pickup required (`follow_resume_window_s`) |
| Isolated feedback timeout | Communication fault, disconnect | Bounded read-only retries, two fresh blocks, then resume or latch |
| Planned target rejected pre-write | `last_sent` might already have advanced | Planned target is discarded; acknowledged state unchanged |
| Bridge zero Joy during loss | Could be accepted after corroboration | Rejected while either tracking flag is unhealthy |

## Follow state machine

| State | Writer permission | Entry | Exit |
|---|---|---|---|
| `DISABLED` | None | No session, disabled profile, duplicate publisher, or latched explicit heartbeat STOP | New explicit ENABLE → `PICKUP_REQUIRED` |
| `PICKUP_REQUIRED` | None (no writes during pickup) | New enable, connect/reload, long interruption, recovered fault/temperature | 5% deliberate excursion + 3% measured-closure match → `FOLLOWING` |
| `FOLLOWING` | Bounded planned targets only | Valid inputs and active pickup | Any required signal becomes invalid → `SIGNAL_PAUSED`; fault → `FAULT` |
| `SIGNAL_PAUSED` | None | Invalid/stale Joy, pose, optical or heartbeat; feedback verification | Continuous healthy input → `RECOVERY_VERIFYING`; window exceeded → pickup after recovery |
| `RECOVERY_VERIFYING` | None | Healthy observations are back but verification is incomplete | Verification interval satisfied → `FOLLOWING` (or pickup if continuity expired); any invalid sample → `SIGNAL_PAUSED` |
| `FAULT` | None | Motor, mechanical, configuration, unknown or exhausted communication fault | Manual recovery, read-only Connect, explicit ENABLE, pickup |

For calibration/test motion, the existing calibration paths remain separate and retain 25 ticks/s, a 20-tick lead, SDK speed 50 and acceleration 5.

## Recovery timing and limitations

Defaults:

| Parameter | Default | Validated range | Meaning |
|---|---:|---:|---|
| `follow_resume_window_s` | 2.0 s | 0.5–10 s | Maximum interruption that may retain automatic resume |
| `follow_verify_s` | 0.15 s | 0.05–1 s | Continuous healthy-input time before movement may resume |
| `comm_retry_count` | 2 | 0–5 | Read-only retries after the first failed feedback read |
| `comm_retry_window_s` | 0.5 s | 0.1–2 s | Total elapsed limit for those retries |

The window never permits motor commands while input is invalid. It only decides whether `FOLLOWING` may be restored instead of requiring pickup. Verification is continuous: any invalid sample restarts it. On success the worker calls `Follow.resume(measured)`, which sets the internal reference and acknowledged baseline to the fresh measured encoder. The next target must still pass the normal rate, lead and travel limits; there is no replay and no jump to the trigger endpoint.

## Tracking and heartbeat loss

- `pose_fresh`, `optical_fresh`, Joy timestamp/order, DDS source age and publisher uniqueness checks are unchanged.
- A false or stale required signal immediately prevents new writes. The last acknowledged target and holding torque are left unchanged; no synthetic opening command is generated.
- A false→true transition that occurs entirely between worker cycles is latched by `Worker.update()` and still requires verification.
- Stale heartbeat delivery is sent as an unknown (`None`) value. Only an explicit `false` message latches STOP. A new session ENABLE is the only operation that clears that latch.
- STOP during pause or verification retires the planned target, follow continuity and automatic-resume permission; a later state confirmation cannot restore motion.

## Serial communication policy

- A failed full-block read invalidates current feedback and stops all new targets.
- Each subsequent worker cycle performs one read-only retry. The first failure plus `comm_retry_count` retries must fit inside `comm_retry_window_s`.
- After a success, a further failure resets the confirmation count. Two consecutive fresh blocks are required, with normal motor status, temperature state, reviewed encoder travel and unchanged authorization.
- Recovery uses the fresh measured position. A previously queued target is never replayed.
- Exhausted retries, a failed/ambiguous position write, unverified torque write, motor status fault, confirmed over-temperature, repeated temperature anomaly escalation, encoder/travel deviation, mechanical stall, identity/configuration mismatch and explicit STOP are latched manual-recovery conditions.
- There is no unbounded reconnect loop and no automatic torque or target retry.

## Operator STOP precedence

STOP GRIPPER and STOP ALL withdraw the session and publish an explicit false heartbeat. The worker latches `heartbeat_stop` and requires a new ENABLE even if a true heartbeat arrives later. GUI request generations still discard late ENABLE acknowledgments after STOP. The last bounded hold target is retained; STOP is not an emergency stop and does not guarantee zero stopping distance.

## Manual intervention remains required

- Confirmed over-temperature or a failed temperature confirmation.
- Repeated independently disproven temperature anomalies (5/60 s).
- Nonzero motor status bits.
- A failed or ambiguous motor write.
- Encoder outside reviewed travel or outside the commanded discrepancy guard.
- Persistent stall/obstruction.
- Servo identity, mode, resolution or angle-limit mismatch.
- Exhausted bounded communication retries or genuine USB removal.
- Interruption longer than the resume window.
- Explicit STOP, including a false heartbeat unrelated to transport delay.

## ROS status contract

Schema version 2 remains additive. New fields in `/gripper/status`:

| Field | Meaning |
|---|---|
| `follow_state` | One of the six explicit states |
| `recovery_reason` | Current pause/verification/pickup reason |
| `recovery_count` | Successful automatic resumes since startup |
| `last_signal_loss_reason` | First invalid observation of the latest episode |
| `signal_loss_duration_ms` | Duration of the last completed episode |
| `resume_pending` | Interruption may retain follow if verified within the window |
| `pickup_required` | Operator pickup is currently required |
| `planned_target_ticks` | Rate/lead-bounded target planned this cycle, not yet acknowledged |
| `target_error_ticks` | Planned target minus fresh measured encoder |
| `last_acknowledged_ticks` | Last successfully written target |
| `communication_retry_count` | Retry attempts in the current feedback episode |
| `communication_recovery_count` | Recovered isolated feedback episodes |
| `communication_state` | `OK`, `RETRYING` or `FAULT` |
| `last_communication_error` | Most recent read/write transport text |

Existing signal, target, temperature, serial and performance fields are preserved.

## Automated validation

Executed on 2026-10-20 with fake drivers and controllable monotonic time; no physical motor was operated.

| Check | Result |
|---|---|
| `pytest test` in `ur_bus_servo_gripper` | **168 passed** |
| `test_gripper_gui.py` | **23 passed** |
| `pytest test` in `ur_vr_teleop` | **441 passed**, one unchanged upstream Python warning |
| `pytest test` in `ur_vr_sim_teleop` | **32 passed** |
| `ros_gripper_check.py` (fake serial, domain 147) | Passed: pickup, short loss hold/auto-resume, stale timestamps, STOP/heartbeat dominance, temperature pause |
| `colcon build --packages-select ur_bus_servo_gripper ur_vr_teleop` | Passed against Humble with system Python |

Covered regressions include continuous follow without re-pickup, short pose and optical loss, false→true events between cycles, no writes during pause/verification, heartbeat delay versus explicit STOP, STOP during recovery, long-loss pickup, no replay after loss, bounded first recovered target, threshold convergence, pre-write rejection, successful acknowledgment, ambiguous-write latching, one isolated read timeout, repeated read failure, reconnect without auto-enable, unsafe encoder/motor faults, legacy configuration loading and temperature behavior.

## Physical commissioning checklist

Not performed by these tests:

1. Confirm adapter identity, USB power/grounding and stable `by-id` path.
2. Verify the actual servo ID, single-turn mode, resolution, firmware angle limits and calibrated travel.
3. With the mechanism clear and supervised, verify STOP, heartbeat loss, explicit ENABLE and pickup.
4. Inject a brief tracking occlusion while holding a trigger position. Confirm the gripper holds without a new position command, then resumes toward the current trigger after input returns.
5. Confirm the resume target does not exceed the configured lead or rate at the physical mechanism.
6. Repeat for a heartbeat/executor delay and for a planned USB disturbance; distinguish recoverable delivery delay from explicit STOP.
7. Confirm an interruption longer than the resume window requires pickup.
8. Inspect `/gripper/status` `follow_state`, `recovery_reason`, timing and retry counters during the fault.
9. Verify that temperature confirmation, confirmed high temperature and motor faults still require deliberate recovery.
