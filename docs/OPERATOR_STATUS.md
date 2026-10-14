# Dashboard status contract

The GUI consumes `/vr_real/operator_status` (`std_msgs/String`, JSON object) from
Commissioning. This is read-only reporting, not a safety-rated interface. The
producer publishes after its existing configuration/control update. A missing,
malformed, or incomplete field is unknown, never a positive readiness result.
The GUI requires a string `session_id` and treats receipt age ≥0.5 s as stale.
All fields below require fresh status. No GUI subscription commands motion.

| GUI value / source | Type and units | Meaning / derivation | Missing or stale behavior |
|---|---|---|---|
| Session / `session_id` | string UUID | Backend lifetime; a new ID cancels old requests and clears local confirmations | No trusted session |
| Mode / `mode` | string | Backend output-enabled or dry-run mode | Unknown |
| Header readiness / `motion_ready` | bool | Existing backend `not problems`; GUI does not reconstruct gating | Readiness unconfirmed |
| Armed / `armed` | bool | Existing gate state | Unknown; no READY inference |
| Enable / `enable_requested` | bool | Backend enable intent, independent of clutch | Enable unavailable; pending request uses separate label |
| Robot / `robot_ready` | bool | Existing latched robot mode, safety mode and program predicate; not a heartbeat or complete readiness | Unknown / not ready |
| Controller / `controller_ready`, `controller_state_fresh` | bool, bool | Existing ownership result and controller-list age <0.6 s | Unknown unless the source check is fresh |
| Advanced diagnostics only / `observations_healthy` | bool | Combined tracking, Joy, joint and TF validation; not a dedicated joint-only measurement | Unknown / not ready |
| Pose / `pose_fresh` | bool | Existing pose/frame flag and source freshness predicate | Unknown / not fresh |
| Optical / `optical` | bool | Existing optical presence with its own age check; not proof of 6-DoF pose | Unknown / absent |
| Stations / `optical_stations` | object of station IDs to nonnegative integer paired-sensor counts | Existing bridge diagnostic counts, zeroed by backend on optical diagnostic expiry; either station suffices for presence | Unavailable, never invented zero |
| Rejection / `tracking_rejection` | string | Existing pose-quality reason | Unavailable |
| Alignment / `alignment_verified` | bool | **Effective** parameter AND `verified_mapping == current mapping`, exactly as checked by backend | Unknown; local checkbox cannot verify it |
| Optical-loss confirmation / `optical_loss_test_verified` | bool | Actual backend session parameter | Unknown; local checkbox cannot verify it |
| Heartbeat / `operator_heartbeat_fresh` | bool | Existing backend receipt age <0.5 s | Unknown; GUI connection alone is not confirmation |
| Clutch / `clutch`, `a_held` | string, bool | Existing clutch state and button observation | Unknown |
| Servo / `servo`, `servo_detail`, `servo_status_fresh` | integer, string, bool | Existing solver diagnostic and age <0.25 s; fresh halt codes 2/4/5 use red | Stale / Unavailable |
| Blockers / `blockers`, `reason` | list of strings, string | Existing backend supervision explanations | Unavailable; not independently inferred |
| Running settings / `effective_parameters` | complete object of every `preferences.DEFAULTS` key | ROS parameter values after `configure()`, in stored units; RPY is a JSON array. Hidden compatibility inputs remain preserved even where ignored by control | Incomplete/invalid snapshot cannot confirm form; Apply disabled |
| Active native joint limit / `hardware_smoothing.velocity` | finite number, rad/s | Existing controller readback, invalidated by backend at 0.6 s; separate from teleop parameter | Unavailable; never substitute saved limit |
| Published command / `velocity_diagnostics.published_twist` | 6 finite numbers: xyz m/s, xyz rad/s | Existing published Cartesian command, including genuine zero when inactive | Unavailable |
| Measured TCP estimate / `velocity_diagnostics.measured_tcp_velocity` | 6 finite numbers: xyz m/s, xyz rad/s | Existing estimate differentiated from measured TF; labeled **estimate**, not a direct velocity sensor | Unavailable (backend nulls it on invalid TF) |
| Requested velocity / `velocity_diagnostics.requested_twist` | 6 finite numbers: xyz m/s, xyz rad/s | Existing preview Cartesian demand | Unavailable |
| Clutch displacement / `motion_metrics.hand_m`, `hand_deg`, `target_m`, `target_deg` | finite numbers, m and deg | Existing distances from clutch anchors; position mode shows target displacement, velocity mode shows requested twist | Not current after release, missing metrics, or status loss |
| Metric validity / `motion_metrics_current` | bool | Existing validity flag; GUI also requires `a_held == true` | Old retained metrics never shown live |

The new read-only fields are `effective_parameters`, `alignment_verified`,
`optical_loss_test_verified`, `operator_heartbeat_fresh`,
`controller_state_fresh`, and `servo_status_fresh`. Pose, stations,
readiness, rejection and velocity diagnostics already existed in the reviewed
commit. Those V3.1 additions did not change gating. Current startup uses configuration readiness without procedural evidence; dynamic guards remain.

## Parameter and service lifecycle

`ParameterState` keeps confirmed effective values, draft text, submitted snapshot,
saved values and native restart-pending values separately. Editable values are
parsed through the named registry, validated in stored units and compared using
relative tolerance 1e-9 / absolute tolerance 1e-10. Arrays compare elementwise;
enums compare exactly. Hidden keys and manual confirmations are not editable
parameter-diff keys.

The complete status snapshot is the single parameter-readback method. The GUI
does not mix partial status aliases with ROS parameter guesses. Atomic Apply
sends changed editable runtime keys. It
never sends the restart-only joint-speed key. Apply + Save writes the submitted
full preference snapshot atomically only after successful service acceptance.
Confirmation still requires a subsequent complete matching status snapshot.
Edits made while the request is pending remain drafts. External changes to an
edited field are retained as conflicts; Apply is blocked until Revert reconciles
the form. The existing ROS API has no compare-and-swap revision: another writer
can still race between the last status and a service call. Subsequent readback
detects mismatches; there is no claim of transactional isolation across writers.

Requests retain future identity, local session generation, start time and (for
Enable) desired state. Duplicate operations are blocked individually; Disarm is
independent. Five-second failures cancel/remove pending client futures. A new
session cancels all prior requests. A readback timeout reports an unknown outcome
and releases the form lock; it never claims synchronization. Fresh complete
status is needed again before Apply or Revert. Closing cancels futures and stops
Tk callbacks. Service cancellation does not undo a request already received by
the backend.

`ManagerWorker` exclusively calls process start/stop/poll/RViz/close. A bounded
snapshot queue feeds the Tk thread. A brief mailbox lock protects only queued
actions, never process work. Stop supersedes queued restart; close cancels queued
actions and cleans up after an in-flight start. The desktop joins this owner
before destroying ROS context; process identity, ownership and escalation remain
inside the unchanged `ProcessManager` implementation.

## V4 authoritative readiness and coordinator

| Field | Meaning |
|---|---|
| `configuration_ready` | Current validated effective configuration; never physical commissioning proof |
| `configuration_reason` | Exact configuration failure or pending-check explanation |
| `tracking_valid` | Fresh accepted pose and bridge flag, pose continuity, stable fresh optical data from either station; signal alone cannot set it |
| `robot_preparation_ready`, `robot_preparation_reason` | Existing mode/safety/program checks plus fresh finite nonzero speed scaling; exact pendant/wait instruction |
| `robot_fault` | Received safety mode outside NORMAL/REDUCED |
| `controller_preparation_allowed` | Fresh controller list, required controller inactive, no competing active motion claimant |
| `servo_state` | Authoritative Servo lifecycle: `absent`, `unstarted`, `running`, `stale`, or `stopped`. Derived from publisher presence plus status freshness — never a permanent historical flag |
| `servo_process_present` | The Servo process advertises its `status` publisher (proves the process exists, even before it is started) |
| `servo_status_seen` | At least one Servo status has been received in this backend session |
| `servo_status_fresh` | A Servo status arrived within 250 ms (MoveIt Servo publishes at `1/publish_period`, ~100 Hz, only after `start_servo`) |

The normal dashboard shows TRACKING VALID/WAITING/LOST and ROBOT READY/WAITING/FAULT. Paired-sensor counts and combined observation reasons remain in Advanced diagnostics. READY requires both backend `armed` and `motion_ready`; submitted service requests are never readiness.

`Startup` transitions through IDLE, WAITING_FOR_STACK, WAITING_FOR_ROBOT, WAITING_FOR_TRACKING, PREPARING_CONTROLLER, STARTING_SERVO, ENABLING_TELEOP, WAITING_FOR_READINESS, READY and ERROR. The adapter retains canceled in-flight operations while stopping. Each preparation stage waits for its acknowledgment and then authoritative readback. Service/readback deadlines are five seconds; missing physical inputs wait without automatic service retries. No sleep chaining or new motion publisher is used. A prominent dashboard banner names the current stage (for example `WAITING — RELEASE A`, `PREPARING CONTROLLER`, `WAITING — SERVO STATUS`, `WAITING — EXTERNAL CONTROL`, `WAITING — INVALID TRACKING`, `READY — PRESS A TO MOVE`), the active blocker and, on failure, the previous stage plus the exact service/readback response.

A Servo whose status has expired is never treated as healthy. If a previously started Servo goes stale, `Startup` attempts exactly one bounded `start_servo` recovery; if fresh status still does not arrive it fails and displays the exact `Restart ROS (Engineering -> Restart ROS)` instruction rather than waiting indefinitely. An absent Servo process fails immediately with the same instruction, and a Start Servo timeout with unknown outcome withholds further automatic starts.

STOP immediately retires Start intent and publishes absent heartbeat plus the existing disarm request. It preserves an in-flight request until completion/timeout, then disarms again; heartbeat remains absent until a fresh Start. Unknown enable outcomes require a new backend session, since canceling a client Future cannot undo a server request. Session generations discard old responses. GUI close retires requests and the existing manager closes only owned children. Stack/session restart and heartbeat loss require a new Start. Ordinary tracking recovery stays in the backend and still requires release, stationary recovery and a fresh A press.

The backend session ID identifies one commissioning backend process. On a genuine session change the GUI disarms, retires in-flight requests, clears old session state, waits for fresh backend status, and permits a new Start; it does not leave stale notice text as the explanation and never re-enables motion automatically. Distinct session IDs and change rate are counted and reported (for example `N backend session IDs, M restarts in 60 s`) as a duplicate-backend risk. A managed child crash exposes the process name, exit reason (or fatal launch line) and its log path, flags repeated crashes, and stops the owned stack without auto-restarting motion. The dashboard shows the A-button input, the backend motion authorization and the effective clutch separately, and never labels the robot clutch ACTIVE while the motion gate is disarmed.


### Initial Setup additions

Observation fields retain their types. `engineering_verified`/`engineering_reason` are retired from runtime status; use `configuration_ready`/`configuration_reason`. `setup_checked` is boolean and becomes true only after the first file check completes, indicating configuration validation has run; it never opens a wizard. `setup_context` is the immutable startup context JSON string used for file identity; `setup_technical` is an exception string shown in Advanced Diagnostics. `setup_controllers_inactive` means Preview with confirmed inactive motion claimants (also require `controller_state_fresh`). `setup_feedback_fresh` requires fresh measured tool TF and valid fresh joints. `base_frame` and `tool_frame` name the measured transform. `velocity_diagnostics.measured_tcp_orientation` is its quaternion in x,y,z,w order. These are observational fields, not new motion permissions or command interfaces.

## V5.1 Preview observation fields

The backend adds `managed_session_id` (managed-process ownership token), `motion_output_enabled` (actual read-only configuration), and `command_publisher_present` (whether its final joint publisher exists). These are observational fields; no command semantics changed. The capture observer matches the managed token and current backend session, verifies the parameter independently using `/vr_real/teleop/get_parameters`, reads `/controller_manager/list_controllers`, and checks the final command publisher graph. A mode label alone is insufficient.

`/vr_real/preview_calibration_observer` subscribes directly to `/vr/controller/pose` (PoseStamped, lighthouse, xyzw), `/vr/controller/joy` (Joy, 16 buttons/12 axes, A at index 4), `/vr/controller/pose_fresh`, `/vr/controller/optical_fresh` (Bool), and `/vr_real/operator_status` (String JSON). It requires one publisher per observed topic. Its own executor checks state every 20 ms; it delivers bounded cumulative snapshots to Tk without invoking widgets. This timer rate is not a guarantee of new sensor samples. Pose ordering and differentiation intervals use source timestamps; button correlation uses receipt timing and bounded freshness. See [V5.1](CALIBRATION_V51.md) for measured synthetic throughput and lifecycle tests.

## Sway observation additions

`measured_joint_position` and `measured_joint_velocity` contain six values in `validation.JOINTS` order (rad and rad/s), or null. The GUI independently subscribes to `smoothed_command`, checks joint names, finite values and source age, and displays final joint velocity plus reference-minus-measured position. Missing/stale data displays Unavailable. These observations never feed the gate. The old alignment/optical-confirmation fields remain deprecated observation compatibility fields and are not operator controls or readiness requirements.
