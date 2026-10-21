# V6.4 trigger reliability

## Diagnosis recorded before implementation

Default branch `main` was fetched and compared with local HEAD
`92c3a75795f484a0072541d1da048403ed21c202` before editing.

* `controller_bridge.cpp` clears its Joy arrays when pose is stale. The pinned
  libsurvive `driver_vive.c::registerButtonEvent` emits trigger axis 1 only when
  its value changes. Therefore pose recovery can publish a cleared zero with a
  fresh Joy header without a physical release. Moving the trigger regenerates
  the cache. This is a software defect, not evidence of faulty hardware.
* `gripper_node.py` combines Joy with subsequent flags on separate topics. Their
  callback order can discard otherwise usable input; a new Joy header proves
  publication, not physical acquisition.
* `Follow.step` tests proximity only. Samples at .35 and .55 skip the .47–.53
  window around measured .5, despite deliberate movement.
* `Worker.update` compares Joy numerically with False; Python's `0.0 == False`
  misclassifies a genuine release as a signal interruption.
* V6.3 preserves continuity during short loss, rebases on measured feedback,
  and requires pickup after long loss. Flickering without completing continuous
  verification intentionally consumes the original two-second window. This is
  retained. STOP, enable generation, connect/reload, calibration, faults and
  long loss legitimately reset pickup.

## Source capability and optical decision

The simple API timestamps button events when its host callback executes, not
at a hardware sampling clock. Axis getters return cached state. It exposes no
input-packet acknowledgement or periodic full input snapshot. Raw IMU callbacks
prove selected-device activity but do not prove every trigger packet arrived.
V6.4 therefore retains both pose and optical gating. A known physical trigger
value may be held only across uninterrupted advancing selected-device IMU data;
an IMU gap invalidates that continuity and requires a new genuine axis event.
Optical recovery alone never revalidates a cache. This is a conservative
change-event continuity model, not an assertion of lossless radio input.

Implementation and validation results follow.

## Implemented contract

`/vr/controller/gripper_input` uses reliable depth-one `std_msgs/String` with a
strict versioned JSON object. This keeps Humble compatibility without generating
a new interface package, while atomically pairing value, evidence and tracking
flags. There is **no Joy fallback** in the gripper node. Deploy/restart the bridge
and gripper together. The arm's Joy contents, clearing behavior, A button, pose
and optical interfaces are unchanged.

| Field | Meaning |
| --- | --- |
| `schema` | Contract version 1; unknown versions inhibit follow |
| `session`, `device` | Bridge startup identity and selected controller name/serial, UTF-8 bytes hex-encoded |
| `stamp_ns`, `observation_seq` | ROS wall publication time and increasing publication sequence; not acquisition time |
| `raw` | Last genuine axis-1 value, or null before any event; historical even when invalid |
| `event_seq`, `event_time` | Genuine event sequence and libsurvive host callback timestamp in seconds |
| `receipt_ns`, `event_age_s` | ROS wall receipt time at queue drain, and increasing age from host callback time |
| `imu_seq`, `imu_age_s` | Advancing selected-device hardware timecode observations and their receipt age |
| `loss_seq` | Monotonic loss generation; a dropped invalid publication cannot conceal an intervening loss |
| `pose`, `optical` | Coherent tracking gates sampled in the same bridge tick |
| `state` | One of the four classifications below |
| `rejected_events` | Bridge-side invalid/reordered/stale axis event count |

`GENUINE_FRESH_INPUT` denotes a newly observed physical change, not a generated
release. `VALID_HELD_INPUT` retains that value while device activity and tracking
remain healthy. `TEMPORARILY_UNVERIFIED` retains history but inhibits commands.
`DISCONNECTED_OR_STALE` means there is no recent advancing IMU observation or the
library is stopped/drain-limited. Event ages are never reset by publication.
A device gap of 250 ms invalidates the held cache until another genuine event;
returning pose, optical data or IMU alone cannot restore it.

The source timestamp is **not a hardware trigger acquisition timestamp**. The
pinned libsurvive revision is `df22f2d6be34cb54d43c46e8838eae8b326b88ff`.
Its simple API does not expose radio packet loss or input queue latency before
the host callback. Unnoticed loss within an otherwise active radio stream cannot
be excluded by software tests. No claim of optical-independent input assurance
or physical radio/USB reliability is made.

The Python parser checks JSON types, finite ranges, DDS/publication ages,
identity, ordering, stable event identity/value/time, nondecreasing event age,
and the paired liveness/tracking evidence. A repeated IMU sequence cannot refresh
worker input freshness. Identity changes retire pickup via the authorization
epoch. A `loss_seq` change records an interruption even if the invalid message
itself was overwritten in the depth-one ROS queue.

## Pickup and recovery

The worker buffers at most 64 genuine events between cycles. Overflow inhibits
continuity. Pickup uses only fresh events received after ENABLE; a 5-percentage-
point excursion is still mandatory. It succeeds by proximity within 3 points,
or by two consecutive event sequences straddling measured closure with strictly
increasing source times separated by at most 250 ms. Normalization applies the
saved trigger calibration, inversion and deadzone first; encoder direction may
be reversed. Signal loss clears incomplete pickup. Missing event sequences,
reordering, stale samples and synthetic values cannot establish a crossing.

Pickup emits no motor command. It initializes reference/acknowledged suppression
baseline from the fresh encoder reading. Subsequent cycles retain the existing
rate, lead and threshold-convergence behavior. Acknowledged command reporting
still advances only after successful writes; a pickup baseline is not reported
as a motor write.

V6.3 state names and the two-second default resume window remain. A short loss
holds, then verifies advancing observations for the configured interval and
rebases on fresh motor feedback. Verification cannot finish using a static
observation repeatedly sampled by the worker. Repeated flickering without a
completed verification consumes the same original window; it does not extend
it. Long loss requires pickup. Explicit STOP, faults and device/session changes
retain their deliberate recovery requirements. Serial read retries remain
separate from controller recovery; ambiguous writes are not retried.

Input callbacks update value/evidence under one lock. A loss revision prevents a
read-time invalidation from being erased by recovery completion. The existing
pre-write gate remains, and the driver checks authorization again before each
SDK packet, including torque enable after the position acknowledgment. If that
check withdraws permission, no subsequent packet is sent and pickup is retired.
An already transmitted packet cannot be recalled. Mechanical, temperature,
travel, speed and acceleration policy and saved profiles are unchanged.

## Diagnostics

Status schema 2 remains additive. `trigger_input` contains the observation,
consumer rejection count and genuine-event count; `measured_closure_fraction`,
`pickup_error`, `pickup_crossing` and `pickup_method` explain synchronization.
`signal_loss_count`, existing recovery count/duration, `last_transition_reason`,
`state_durations_s` and current state duration describe interruptions.
`follow_history` retains at most 32 observed state transitions, evaluated at the
25 Hz worker rate; no per-publication ROS logging is added. Existing serial
counters, measured position and acknowledged target fields remain available.
The existing Gripper Setup diagnostics show provenance, age, pickup method/error
and the most recent transition without redesigning the controls.

## Automated validation

The final results are recorded below. Tests use fake motor feedback;
none establishes physical stopping distance, temperature accuracy, radio
reliability, load retention or safe travel calibration.

## Supervised hardware commissioning still required

1. With the mechanism clear and reviewed calibration loaded, connect read-only
   and confirm selected controller identity, raw range and genuine-event age.
   Do not change the saved motor speed, acceleration, travel or thermal limits.
2. Explicitly ENABLE GRIPPER. Move through actual closure once; confirm
   proximity/crossing pickup and no motor target during pickup itself.
3. Hold the trigger near 70%. Briefly interrupt Lighthouse visibility while
   retaining controller activity, then restore it. Inspect state/history and
   encoder/acknowledged-target data: hold during loss, verified short recovery,
   no false zero or opening, no repeated pickup.
4. Repeat brief flickers, then a loss longer than the resume window. Confirm
   the latter requires fresh deliberate pickup and never replays old travel.
5. Under appropriate load precautions, test controller power/receiver loss.
   IMU restoration alone must leave the cached trigger unverified until a
   physical trigger event; do not interpret the historical value as live input.
6. Confirm STOP GRIPPER and STOP ALL remain dominant during recovery; verify
   late acknowledgments do not restore permission. Use the existing supervised
   fault/thermal/serial commissioning procedures, without deliberately creating
   unsafe heat, a mechanical jam or an unrestrained dropped load.
7. Save `/gripper/status` and application logs with incident wall time. Compare
   host event age, IMU activity, loss generation, transition reason and motor
   feedback. Physical results remain pending until explicitly observed.

## Final software validation results

Validated on Ubuntu / ROS 2 Humble using `/usr/bin/python3`; no physical driver
or managed real desktop was launched.

| Executed check | Result |
| --- | --- |
| `colcon build --base-paths src --packages-select ur_vr_bridge ur_bus_servo_gripper ur_vr_teleop --executor sequential` | All three packages built; subsequent affected-package rebuilds also passed |
| `ctest --test-dir ur_teleop_ws/build/ur_vr_bridge --output-on-failure` | 3 passed: trigger, pose and optical guards; assertions enabled in Release |
| `/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_bus_servo_gripper/test ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q` | 671 passed; one existing upstream Python compatibility warning |
| Gripper-only rerun after recovery-lock review | 197 passed |
| `ros_gripper_check.py`, `ROS_LOCALHOST_ONLY=1`, explicit domain 147 | Passed after final C++ rebuild: fake event → production guard → real ROS metadata → worker → fake motor |
| `gui_render_check.py`, display `:0`, explicit ROS context 147 | Passed; Gripper Setup screenshot visually inspected, controls remain scrollable |
| `test_gripper_gui.py` after final status wording | 24 passed |
| Local documentation links / Mermaid parser | 94 links resolved; 3 diagrams parsed with Mermaid 11 |
| Arm compatibility source comparison | Joy generation and pose acceptance byte-identical to pre-change HEAD |
| `git diff --check` | Passed |

During validation, removal of the node's `math` import broke JointState
publication. The import was restored, the installed package rebuilt, and the
full ROS lifecycle check rerun successfully. The earlier stale-overlay retry is
not counted as a pass. No hardware-dependent result is included in this table.

## Changed implementation files

Paths are relative to `ur_teleop_ws/src`:

* `ur_vr_bridge/include/ur_vr_bridge/trigger_guard.hpp` (new),
  `ur_vr_bridge/src/controller_bridge.cpp`, and `ur_vr_bridge/CMakeLists.txt`.
* `ur_bus_servo_gripper/ur_bus_servo_gripper/trigger_input.py` (new),
  `gripper_node.py`, `gripper_mapping.py`, `gripper_worker.py`, and
  `st3215_driver.py` in that same Python package directory.
* `ur_vr_teleop/ur_vr_teleop/gripper_diagnostics.py` and `gripper_panel.py`.
* New `ur_vr_bridge/test/test_trigger_guard.cpp`,
  `ur_vr_bridge/test/trigger_input_fixture.cpp`, and
  `ur_bus_servo_gripper/test/test_trigger_reliability.py`; updated
  `ur_bus_servo_gripper/test/ros_gripper_check.py` and
  `ur_vr_teleop/test/test_gripper_gui.py`.

AGENTS, README and the architecture, configuration, operation, commissioning,
gripper and gripper implementation guides were updated. Saved operator and
motor profiles, arm control, Servo, Ruckig and wrist filtering were not changed.
