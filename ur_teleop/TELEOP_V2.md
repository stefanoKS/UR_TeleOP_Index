# UR10e teleoperation V2 — implementation and commissioning

Physical-robot smoothness, single-station accuracy and reliability are **not yet
validated**. No real robot was activated during implementation. These checks are
software regression evidence, not certification of robot safety.

## Actual command path and changes

The bridge wraps the existing libsurvive optical callback without changing its
solver or calibration. Accepted controller poses enter `Preview` validation and
`TrackingClutch`. The commissioning node publishes a base-frame `TwistStamped`
at the existing 100 Hz rate. MoveIt Servo performs Cartesian-to-joint conversion,
collision/singularity handling and joint constraints. The timestamp relay and
commissioning gate validate the short trajectory. The existing scaled trajectory
controller applies Ruckig before the UR driver's position interface and `servoj`.

Position mode retains its clutched, filtered positional/orientation target and
position-error response (plus optional measured-velocity damping). Velocity mode
uses consecutive **solver-timestamped** hand samples, transformed into the command
frame. It filters position/orientation, applies continuous spatial play deadbands,
differentiates accepted samples, scales/filter velocities, clamps vector norms,
and slews the requested twist with vector acceleration limits. Angular increments
use world-relative quaternion rotation vectors conjugated into robot axes. It
never uses TCP position error or `tracking_gain_s_inv` to request velocity.
The marker in velocity mode is the measured TCP, not a target to catch up with.

The existing coordinate alignment, robot calibration/IP, ROS topic names/message
types, driver launch, joint safeguards, speed scaling, watchdogs, Ruckig law and
v/a/jerk, and `servoj` settings are preserved. No direct `speedl`/`speedj` path was
added. The launch forwards the same effective saved configuration used by the GUI.

Release and faults clear command/estimation history. Re-engagement initializes
at zero at the current hand pose. Repeated timer reads do not become samples.
Source intervals outside 1–100 ms fault; no update for 30 ms requests bounded
ordinary deceleration; 100 ms without a distinct sample latches a tracking fault.
Fault/release commands go to zero upstream immediately and use the existing
bounded downstream stop/hold. This differs intentionally from ordinary slew-limited
hand stopping. A held A-button cannot rearm after a fault.

## Optical signal versus motion readiness

`/vr/controller/optical_fresh` now means **signal presence only**: at least one
fresh paired sensor from either calibrated station within 250 ms. Duplicate,
out-of-order, invalid, uncalibrated and rejected optical events cannot refresh it.
IMU updates never refresh optical observations. All observed station counts remain
in `/vr/controller/optical_status` as `LHn=count`; commissioning also checks that
this diagnostic source is unique and fresh. No new custom message is required.

- `two_station_strict` (default): at least four accepted pairs on each of two
  stations, with the existing 500 ms optical stability requirement.
- `single_station_experimental`: at least one accepted pair on either station,
  plus an independently accepted stream of at least three 6-DoF poses spanning
  100 ms and the same optical stability requirement. Both linear/angular speeds
  and accelerations are reduced by `single_station_motion_scale` for the entire
  experimental session, even when two stations become visible.

Both policies retain all robot, TF, joint, controller, operator, Servo, watchdog,
and calibration-confirmation checks. Pose validation checks finite coordinates,
normalized valid quaternions, monotonic timestamps, source/receipt freshness,
translation jumps over 5 cm, rotation jumps over 45 degrees, and gaps over 350 ms.
Exactly unchanged solver output for more than 500 ms is conservatively rejected;
this can cause nuisance stops for a perfectly still or quantized solver. Ordinary
noisy stationary measurements are allowed. Recovery requires releasing A.

These checks establish continuity and plausibility, **not metric pose accuracy or
full optical observability**. A plausible but drifting solver output can escape
these checks. Single-station tracking remains explicitly experimental. If the
workcell trial cannot establish reliable 6-DoF tracking, leave teleop disabled;
an optical-presence indicator or GUI confirmation is not proof of accuracy.

Pose header timestamps now use a fixed source-to-ROS clock anchor, preserving
actual solver intervals when the queue drains multiple updates in a burst. The
original solver time remains on `/vr/controller/source_time`. This is a timestamp
semantic change, not a topic/type change. Subscribers still check source age and
receipt age. Restart the bridge if its source clock resets; clocks are not silently
rebased during an active session. Initial queue age and long-term clock drift need
physical measurement; sample-to-command age is an estimate, not hardware latency.

## Configuration and starting values

Saved version-1 `operator_settings.json` files remain compatible. New missing keys
use defaults, while saved values override YAML. Permissions, enable state and
verification checkmarks are not restored. GUI changes require disarming and A
released; changing modes clears motion history. Use **Apply and save parameters**
to persist. The startup log records effective values and their sources.

| Parameter | Initial value | Meaning |
|---|---:|---|
| `teleop_control_mode` | `velocity` | `position` restores the regression controller |
| `tracking_station_policy` | `two_station_strict` | Explicitly select `single_station_experimental` for a one-station trial |
| `translation_scale` | 0.8333333333 | Hand speed to TCP speed ratio; GUI uses inverse VR:robot ratio 1.2 |
| `rotation_scale` | 1.0 | Hand angular speed to TCP angular speed ratio |
| `vr_translation_filter_alpha` | 0.8 | Input position EMA; 1 disables |
| `vr_rotation_filter_alpha` | 0.8 | Input quaternion-increment filter; 1 disables |
| `vr_translation_deadband_m` | 0.001 | Continuous spatial jitter band |
| `vr_rotation_deadband_deg` | 0.3 | Continuous angular jitter band |
| `linear_velocity_filter_s` | 0.04 | Time-based EMA; 0 disables, maximum 0.2 s |
| `angular_velocity_filter_s` | 0.04 | Time-based EMA; 0 disables, maximum 0.2 s |
| `max_linear_speed_m_s` | 0.10 | Before experimental multiplier |
| `max_angular_speed_deg_s` | 30.0 | Before experimental multiplier |
| `max_linear_acceleration_m_s2` | 0.25 | Before experimental multiplier |
| `max_angular_acceleration_rad_s2` | 1.5707963268 | Before experimental multiplier |
| `single_station_motion_scale` | 0.25 | Allowed 0.01–1; initial effective limits 0.025 m/s, 7.5 deg/s, 0.0625 m/s², 0.3927 rad/s² |
| `tracking_gain_s_inv` | 2.0 | Position mode only |
| `translation_damping_gain`, `rotation_damping_gain` | 0.0 | Position mode only |
| `damping_velocity_filter_s` | 0.05 | Existing measured-velocity filter |
| `max_joint_speed_rad_s` | 0.50 | Preserve final-controller comparison setting |
| `workspace_radius_m` | 0.0 | Existing optional measured workspace fence; startup-only |
| `observation_timeout_s` | 0.35 | Existing general observation freshness bound |

These are software starting values, not validated safe settings for every payload
or workcell. For initial authorized trials, use the experimental reduced envelope
and a suitably reduced pendant speed. Keep final joint acceleration 1 rad/s²,
jerk 10 rad/s³, `servoj_gain=500` and `servoj_lookahead_time=0.03` unchanged when
comparing modes. Use the already calibrated mapping; do not recalibrate by hand.

## Dry run and offline validation

The existing operator-facing shortcut and `ur_teleop/start_gui.sh` are unchanged.
Select **Preview**, leave motion controllers inactive, and verify **DRY RUN — ROBOT
OUTPUT DISABLED**. The commissioning node creates no robot-command publisher in
this mode. Preview may connect to the existing driver for observations; it is not
the hardware-free simulation. Do not select Live setup for a dry run.

To launch just the dry-run commissioning components (requires existing observation
sources, but does not start the driver or activate controllers):

```bash
cd /home/ubuntu-ros/Documents/Teleop
source /opt/ros/humble/setup.bash
source ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=43
ros2 launch ur_vr_teleop commissioning.launch.py motion_output_enabled:=false
```

Hardware-free V2 comparison:

```bash
python3 ur_teleop/scripts/compare_modes.py --delay 0.08 --lag 0.10 --acceleration 0.3
```

This configurable plant includes transport delay, velocity lag and acceleration
limits. It tests move/hold, reversals and small deliberate movements in both modes.
It is not a UR10e dynamics model. The legacy `ur_vr_sim_teleop` RViz simulation
remains unchanged; use the comparison script for V2 closed-loop regression.

Automated ROS checks run with localhost-only contexts in domains 147/148:

```bash
/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test ur_teleop_ws/src/ur_vr_sim_teleop/test -q
/usr/bin/python3 -m pytest ur_teleop_ws/src/ur_vr_teleop/test/ros_integration_checks.py -q
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/servo_pipeline_check.py
/usr/bin/python3 ur_teleop_ws/src/ur_vr_teleop/test/mock_startup_check.py
```

The Servo check runs actual Servo/RSP/relay processes with synthetic joints and no
UR driver. The mock startup check verifies `GenericSystem`, six joint states and
TF, using loopback IP with dashboard disabled and motion controllers inactive.

## Repeatable recording and comparison

1. Record the effective settings and station policy. Keep downstream settings,
   payload, initial robot posture and workcell geometry constant across trials.
2. Start `bash ur_teleop/scripts/record_sway.sh`; it records observations and
   existing command/feedback topics without enabling motion.
3. In each mode record: 10 s stationary hold; a slow translation then 5 s hold;
   controlled reversal; five small movements separated by holds; pure rotation;
   combined translation/rotation; A-release during motion. Repeat each three times.
4. End recording with Ctrl-C. Disarm and release A before selecting the other mode.
5. Run `python3 ur_teleop/scripts/compare_modes.py --bag /absolute/bag/directory`.
   Compare the JSON results from matching trials and retain the full bags.

The bag includes raw accepted VR poses, source times, clutch, optical status,
Servo twists/outputs, relay outputs, final smoothed commands/controller diagnostics,
actual joint position/velocity, TF and operator status. Operator status includes
filtered pose (with frame), sample interval, raw/filtered and independently observed
hand velocity, requested/published twist, measured TCP position/velocity, scaling,
clamping, readiness and rejection reasons. The independent measured TCP derivative
continues through release so software reset cannot masquerade as measured stopping.

The offline script reports stationary-hand travel, stop latency, post-stop overshoot
relative to the measured position when the hand stopped, late-hold projected
peak-to-peak oscillation, reversal latency, sample-to-command age percentiles,
command-period jitter and measured acceleration RMS. Stop thresholds are 1 mm/s;
review raw data and thresholds for your noise level. Position/velocity mode have
no common exact endpoint promise, so the overshoot metric is a stopping-distance
comparison, not endpoint accuracy. Bag analysis currently summarizes translation;
angular traces remain available in all six-component velocities for inspection.
Do not combine unrelated sessions into one bag comparison.

## Authorized supervised physical trial (not performed here)

Use the established safeguarded workcell procedure, a trained operator at the
pendant and an observer able to stop the robot. Verify payload/tool and clearance.
First run Preview with robot motion controllers inactive. Check all three
translation and rotation directions and the actual `tool0` frame. With experimental
policy selected, test station 0 alone and station 1 alone. Verify the correct paired
sensor counts and independent pose freshness; signal presence without a valid pose
must leave motion readiness false.

Cover the only visible station: optical signal must expire within its 250 ms bound,
tracking readiness must drop and the gate must disarm. Restore visibility while A
remains held: it must remain disarmed. Release A, wait for stable readiness, then
press again; the first request must be zero without catch-up. Repeat by blocking
both stations under strict policy. Also test stale pose output and the GUI heartbeat
loss in dry run. Only then confirm mapping and optical-loss verification.

For an explicitly authorized live trial, use the existing **Live setup** workflow,
start the pendant External Control program, prepare the controller and start Servo
through the existing controls. Inspect every readiness blocker. Keep the reduced
Cartesian envelope and low pendant speed, enable teleop with A released, and start
with tiny isolated translation/rotation movements. Release A frequently and verify
measured stopping. Record the sequence above. Stop on drift, oscillation, unexpected
mapping, jumps or unreliable tracking. A GUI disarm is a software stop; use the
pendant emergency procedure for an emergency. Never clear protective stops through
this tool or bypass readiness checks to complete a trial.

## Rollback

Disable teleop, release A, choose `position`, choose `two_station_strict`, then
Apply and save. This restores position-following control and strict optical motion
readiness without changing calibration, driver or Ruckig settings. The new optical
Bool continues to mean signal presence; strict authorization is in commissioning.
To restore the entire old implementation, first stop the owned ROS stack, preserve
operator settings, revert this change set using version control, rebuild the four
packages and relaunch disarmed. Do not mix an old gate with the new bridge's relaxed
optical Bool: deploy/revert the bridge and commissioning node together.

## Files and compatibility

- `ur_vr_teleop/tracking.py`: selectable velocity estimator/limiter and state reset;
  the position branch retains its regression behavior.
- `ur_vr_teleop/tracking_quality.py` (new): independent pose continuity and optical
  policy functions.
- `ur_vr_teleop/preview.py`: per-pose validation and independent measured telemetry.
- `ur_vr_teleop/commissioning.py`: settings, policy gate, reduced envelope and status.
- `ur_vr_teleop/preferences.py`, `gui.py`, `launch/commissioning.launch.py`,
  `ur_teleop/config/real_teleop.yaml`: validated persistence, controls, diagnostics
  and effective launch propagation.
- `ur_vr_bridge/include/ur_vr_bridge/optical_guard.hpp`,
  `src/controller_bridge.cpp`: optical presence separation and source timing.
- `ur_vr_bridge/test/test_optical_guard.cpp`: single/dual/strict/expiry regressions.
- `ur_vr_teleop/test/test_velocity.py`, `test_comparison.py` (new),
  `test/ros_integration_checks.py`, `test/gui_render_check.py`: new and extended checks.
- `ur_vr_teleop/comparison.py`, `ur_teleop/scripts/compare_modes.py` (new): regression
  plant and offline ROS bag comparison; `scripts/record_sway.sh` adds source times.
- This guide and package READMEs describe operation and timestamp/Bool semantics.

`ur_vr_teleop/*` above refers to
`ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/*`; test/launch files are in that package.
No controller source, UR calibration, network setting or launcher shortcut was edited.

## Validation record — 2026-10-12

See the final test record appended after verification. No physical tracking bag was
available in the workspace; offline bag validation uses explicitly synthetic recorded
messages. Remaining validation includes real solver clock behavior, stationary
tracking noise and freezes, single-station geometry/occlusion/drift, tool/payload
response, collision/singularity transitions, physical stop distance and actual
latency/jitter. No claim of improved physical stability is made.

Final verified results:

- Python unit/regression suites: **164 passed**, including nine delayed-plant cases.
- Explicit ROS integration suite: **72 passed**, including single-station readiness,
  invalid/frozen/jumped pose handling, mode switching, dry-run publisher checks,
  invalid TF diagnostics and synthetic ROS bag serialization/analysis.
- C++ bridge: **2/2 CTest executables passed** (pose and optical guards).
- Unchanged final-controller tests: **11 GTests passed** across two executables.
- `colcon build`: **all four packages succeeded** (`ur_vr_bridge`,
  `ur_vr_sim_teleop`, `ur_vr_teleop`, `ur_teleop_controllers`).
- Actual Servo/RSP/relay synthetic bench: **passed**, 201 targets, 10 ms horizon,
  no robot command publisher. GenericSystem driver startup: **passed**.
- GUI render/effective-settings check: **passed** on the local display; all new
  controls are reachable by scrolling. The optional `xvfb-run` wrapper was absent,
  so the isolated GUI check used the existing display instead.
- No outstanding test failures. An initial unsourced-workspace test invocation
  failed package discovery; sourcing the installed workspace resolved it. Initial
  integration fixtures were updated to supply station counts and explicitly select
  position mode for position-target assertions; no old test cases were removed.
- One existing xacro/Python deprecation warning remains. Real hardware and real
  recorded tracking were not tested; the bag fixture and plant are synthetic.
