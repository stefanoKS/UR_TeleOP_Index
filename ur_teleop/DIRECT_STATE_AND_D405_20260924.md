## Velocity-mode sway fix — 2026-09-25

Kept the restored velocity-mode Ruckig ramp. For commands with explicit Servo
velocities, removed only the extra 5/s correction against internal reference
position. Position-only fallback and existing hold behavior remain. No mapping,
user settings, Servo timing, velocity/acceleration/jerk limits or protections changed.

Built ur_teleop_controllers only. Eleven native tests passed, including positive
command direction, opposing internal-position error, braking, final hardware write
bounds, and stationary outer-loop convergence with simulated feedback lag. The
simulation covered gain 2 at 40/100 ms lag and gain 10 at 40 ms lag. Exploratory
high-gain/high-lag combinations can still oscillate: this is not a guarantee of
physical stability at arbitrary tuning. Physical robot acceptance remains pending;
no real robot command was issued. Restart ROS to load the rebuilt plugin.

Changed source: smoothing.hpp, smooth_scaled_controller.cpp, test_smoothing.cpp.
Backup: /home/ubuntu-ros/ur_teleop/backups/before_velocity_sway_fix_20260925_090336.

## Targeting rollback — 2026-09-25

The user reported wrong-direction motion. Offline reproduction confirmed that the
new position-mode Ruckig path, given q=+0.003 rad and dq=+0.3 rad/s from rest,
initially moved negative (−0.0117 rad after 0.2 s) to meet the terminal velocity.
The previous velocity-ramp motion path and hold behavior were restored. The same
reproduction now moves positive. Acceleration/jerk protections, camera, settings
persistence and diagnostic topics remain. This rollback does not establish that
the earlier physical sway is resolved. No robot motion was commanded during testing.
Restart the managed ROS stack to load the rebuilt controller.

The original report below describes the superseded position-mode implementation.

# Stationary-target sway investigation and D405 integration — 2026-09-24

Implemented and built locally. Physical robot stability is **not yet accepted**: no robot movement, controller activation, fault reset, or restart of the user's running stack was performed during this update. Camera capture was tested on the connected D405, independently of robot control.

## Findings

The current path was: VR pose → clutch mapping/filter/deadband → Cartesian proportional tracking and optional measured-tool damping → Cartesian velocity/acceleration limits → MoveIt Servo → timestamp relay and commissioning gate → UR scaled trajectory controller → native Ruckig smoother → position interfaces → UR ServoJ.

The native stage was in **velocity control mode**, computing `servo_dq + 5*(servo_q - internal_ruckig_q)`. That is another position correction loop against an internal command reference, rather than the actual robot. It can oppose the outer tracking loop while measured joints lag. This is a concrete control-structure issue, but the source inspection alone cannot prove it explains every physical oscillation.

Normal single-point streaming now sends the incoming position endpoint and supplied velocity into **Ruckig position mode**. The extra P term is removed. Existing native velocity/acceleration/jerk limits remain, and UR speed scaling still applies. A zero hardware speed-scaling factor requests bounded braking rather than convergence toward a distant target.

The actual current Servo period is **10 ms (100 Hz)**. It was not changed. The gate preserves each incoming horizon, including the existing 20 ms regression case. The explicit 100 ms, zero-velocity hold message is unchanged.

Position-only streaming derives velocity from consecutive received target positions and their observed receipt times, bounded by the existing velocity limit. Its first sample seeds history; it does not start motion using an invented velocity. Supplied Servo velocities are preferred. A regular 10 ms point with a zero velocity remains a position target; it is not confused with an explicit 100 ms hold.

At activation, Ruckig initializes from valid measured position and velocity under the existing standstill requirement. On a stream restart, a bounded transition reconciles its reference with a captured measured position before resetting at that settled anchor. It does not jump directly from a lagging command reference to measurement. Release/disable/fault holds and native watchdog holds brake and settle; hardware faults still require explicit recovery. Existing following-error, timing, gate, optical, collision and singularity checks remain.

## Why changing settings seemed to help, then changed on reboot

Precedence remains: built-in defaults → `real_teleop.yaml` → `operator_settings.json`. **Apply settings** changes the running session; **Apply and save parameters** also writes the next-launch profile. Joint speed is a driver startup setting and requires a restart. Editing a field alone does not apply it; the GUI's **Applied** summary is the running value.

The profile inspected in this update had linear speed 1 m/s, angular speed 40°/s, joint speed 1 rad/s, gain 2, damping 0.2/0.2 with a 200 ms measured-velocity filter, VR alphas 0.2/0.2, translation deadband 10 mm, rotation deadband 2°, translation/rotation scale 1/1, and alignment yaw −40°. These differed substantially from YAML defaults. Historical reports of gain 10 describe an earlier profile, not the one inspected here.

Each managed launch now reads settings **once** and uses the same snapshot for driver and teleop. The run directory contains `effective_config.yaml` and `effective_settings.log`. The log records effective tuning and the exact YAML/JSON/default source, plus native velocity/acceleration/jerk. Teleop startup also logs the effective values. Saved tuning behavior is preserved.

Most relevant controls:

| Setting | Main effect |
|---|---|
| Tracking response / gain | How strongly Cartesian velocity responds to position/orientation error; high values can amplify feedback lag. |
| Native acceleration and jerk | How rapidly the final joint reference can change. They constrain the command, but do not by themselves prove closed-loop stability. |
| Damping and its velocity filter | Feedback from measured tool motion. Filtering adds delay, so increasing damping is not automatically more stable. |
| VR filter alpha / jitter deadband | Input jitter and target lag. More input filtering cannot repair a downstream oscillation when the target is already stationary. |
| Cartesian and joint speeds | Ceilings; raising them does not remove delay or make an inconsistent control loop stable. |
| ServoJ gain/lookahead | Robot-side tracking response. Left at the user's confirmed **500 / 0.03 s** in this update. |

For the baseline, saved gain is **2**, damping **0 / 0**, linear speed **0.1 m/s**, angular speed **30°/s**, joint speed **0.5 rad/s**. Native acceleration remains **1 rad/s²**, jerk **10 rad/s³**. Current saved Cartesian accelerations (1 m/s² and 0.8 rad/s²), mappings, deadbands and VR alphas were preserved to avoid changing several additional variables. Damping remains available in the GUI.

Original configuration backups:

- `/home/ubuntu-ros/ur_teleop/config/real_teleop.yaml.before_direct_state_20260924_192508`
- `/home/ubuntu-ros/ur_teleop/config/operator_settings.json.before_direct_state_20260924_192508`

## Installed MoveIt Servo semantics

Installed package: `ros-humble-moveit-servo 2.5.10-1jammy.20260908.204514`. In `ServoCalcs::applyJointUpdate`, Servo updates joint position by its calculated increment, applies its smoothing plugin, and calculates velocity as `(smoothed_target_position - original_measured_position) / publish_period`. Thus this field is a desired target-state velocity tied to measured feedback and filtering; it is not an independent, clean feed-forward measurement. No MoveIt patch was made. [Exact MoveIt 2.5.10 implementation](https://github.com/moveit/moveit2/blob/2.5.10/moveit_ros/moveit_servo/src/servo_calcs.cpp).

Ruckig accepts current and target position/velocity/acceleration states under velocity, acceleration and jerk constraints. This update uses that position-target interface directly. [Ruckig documentation](https://docs.ruckig.com/).

## Servo halt messages

The GUI now distinguishes the installed status codes: −1 invalid/uninitialized, 0 ready, 1 slowing near singularity, 2 singularity halt, 3 slowing near collision, 4 collision halt, 5 joint position/velocity bound halt, and 6 slowing while leaving singularity. Missing status for 250 ms is reported separately as stale. A Servo halt is not necessarily a physical UR joint fault. No halt or protection was bypassed.

## Diagnostic signals

`/scaled_joint_trajectory_controller/smoothed_command` remains the 100 Hz reference actually written to the interfaces, including q/dq/ddq.

New `/scaled_joint_trajectory_controller/control_diagnostics` is a preallocated numeric topic at 10 Hz; it avoids text formatting/allocation in the hardware update loop. Message type: `std_msgs/Float64MultiArray`. `data[0]` is ROS time in seconds; layout `data_offset=1`. Following that are six rows, in `smoothed_command.joint_names` order, with eight columns:

`measured_q, measured_dq, incoming_servo_q, incoming_servo_dq, ruckig_q, ruckig_dq, servo_minus_measured, ruckig_minus_measured`.

## Physical A/B validation

1. Release A, disable teleop, use **Stop ROS**, and close the old GUI. Start the updated application with `/home/ubuntu-ros/ur_teleop/start_teleop.sh`. A fresh GUI is required to load its new code. No terminal group needs to be constructed manually.
2. Check the **Applied** summary and the new run's `effective_settings.log` against the baseline above. Start in Preview, verify current mapping, then use the existing Live setup / External Control / Prepare controller / Start Servo solver workflow. Keep the baseline limits and a clear small-motion workspace; stop the test if vibration or a joint fault recurs.
3. Start `/home/ubuntu-ros/ur_teleop/scripts/record_sway.sh` in one terminal. It only records observations. It captures VR pose/inputs, Cartesian Twist, raw and stamped Servo targets, forwarded trajectory, final Ruckig output, controller feedback, measured joints/TF, status, scaling and parameter events into **one bag**. Stop recording with Ctrl+C after the trial.
4. **Test A:** gain 2, both damping gains 0. Enable, press A, translate about 2 cm along one verified axis over roughly 2 seconds, then keep the controller physically still for 10 seconds while holding A. Release A. Repeat for the other axes and a small 5° rotation. Expect a stationary target, converging command and measured joints, and decaying velocities rather than repeated sustained reversals. Do not deliberately reproduce the previous severe-fault configuration.
5. **Test B:** save that exact configuration, stop/restart the application, and repeat the same movements without changing tuning. Compare the two effective-config logs and settling traces. This tests the reboot inconsistency directly.
6. If sway remains, do not add another filter first. In the bag, identify the earliest persistent oscillation: Twist with stable target → outer feedback; Servo q/dq with quiet Twist → Servo/feedback interpretation; final Ruckig q/dq with quiet incoming targets → native generation; quiet final reference with oscillating measured q/dq → physical tracking/robot-side investigation. Feedback can couple these signals, so inspect timing as well as amplitude, especially wrists 1–3. Keep both run directories with their bags.

Optional later damping comparison: only after baseline settling is acceptable, change one damping gain from 0 to 0.2 with all other values unchanged, repeat the same capture, then restore/save 0. This is not needed to validate the nested-loop repair.

## D405 camera

Detected D405 serial **230322276078**, firmware **5.12.14.100**, installed SDK **2.56.5**. It starts with the managed stack as an optional capture process at 640×480 / 15 FPS. Use **Open D405 camera** for RGB or a grayscale depth view (0–1 m). Camera capture retries after failure; camera loss does not stop robot ROS nodes. Closing the viewer removes its image subscriptions but leaves camera topics available.

Topics:

- `/wrist_camera/color/image_raw` — RGB8.
- `/wrist_camera/depth/image_raw` — 32FC1 **metres**, invalid zero sensor counts converted to NaN.
- `/wrist_camera/color/camera_info` and `/wrist_camera/depth/camera_info` — factory intrinsics.
- `/wrist_camera/status` — connection/freshness information.

This unit's raw depth scale is **0.0001 m/count**, so publishing raw counts as ROS 16UC1 millimetres would be wrong. Depth is not aligned to RGB. Its SDK reports inverse-Brown distortion for a stream: the calibration preserves that model explicitly as `inverse_brown_conrady` instead of mislabeling it as forward `plumb_bob`. Standard ROS rectification tools may not support this model; use SDK deprojection or a proper rectification stage for later 3D processing.

RGB/depth timestamps are host receipt times, not hardware synchronization with UR. No tool0-to-camera mount transform, payload change, collision geometry, or hand-eye calibration was invented. Those require the actual mounting measurements/calibration before using camera pixels as robot-frame 3D coordinates. Image viewing/recording works without that transform. ACT remains deferred.

The installed SDK embeds Fast DDS symbols incompatible with the Humble middleware in the same process; an initial combined-node test reproduced a startup crash. The final integration isolates SDK capture in a small C++ child process and ROS publishing in a separate Python process. Robot middleware and system SDK packages were not changed.

## Validation

- Final suite: **171 Python/isolated ROS tests passed**. Existing 20 ms forwarding / 100 ms hold preservation checks passed. Native results: **9 tests, zero errors/failures/skips**.
- Eight native smoothing/history tests and the actual controller plugin test passed. The latter exercised lagged mock hardware, reversals, position-only fallback, zero-velocity normal targets, explicit holds, watchdog expiry, re-enable, and invalid-period rejection. Stationary-stream peak command velocity decayed to numerical zero (~7e−15 rad/s). Finite-difference peaks: 0.254 rad/s, 1 rad/s², ~10 rad/s³ (finite-difference numerical tolerance).
- Generated controller YAML passed the real ROS parser; a GenericSystem driver startup produced all six joint states and base_link→tool0 TF.
- Main GUI and real-camera RGB/depth viewer rendered and inspected. Terminating only the test SDK capture child recovered fresh frames automatically; mock launcher tests confirmed optional-camera exit does not stop robot nodes. SDK capture measured 15.00 FPS; the ROS read-only capture received about 13 RGB FPS and 14 depth FPS with best-effort QoS on the running desktop. No lossless recording claim is made.
- Only `ur_teleop_controllers`, `ur_vr_teleop`, and new `ur_d405_camera` were built. Existing installation mode was retained.

Final results checked again on 2026-09-25. Test camera processes were closed. The saved baseline remains gain 2/damping 0 and ServoJ lookahead 0.03 s.

## Modified and added source/configuration files

Paths below are absolute. Generated build/install files are not separate source changes.

| File | Change |
|---|---|
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_teleop_controllers/include/ur_teleop_controllers/smoothing.hpp` | Removed the extra P term and selected direct position/velocity target generation; retained derivative constraints and zero-scaling braking. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_teleop_controllers/include/ur_teleop_controllers/target_history.hpp` | Added receipt-time finite differences for position-only streaming. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_teleop_controllers/src/smooth_scaled_controller.cpp` | Preserves raw streaming targets, separates hold handling, reconciles restarts without command jumps, and publishes 10 Hz diagnostics. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_teleop_controllers/CMakeLists.txt` | Declares the numeric diagnostic message dependency. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_teleop_controllers/package.xml` | Declares the corresponding std_msgs dependency. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_teleop_controllers/test/test_smoothing.cpp` | Tests direct target states, missing-velocity history, holding, convergence and existing bounds. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_teleop_controllers/test/test_controller.cpp` | Extends real-plugin mock tests across watchdog/hold/re-enable transitions and stationary zero-velocity targets. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/preferences.py` | Adds effective-value provenance and per-run snapshots; changes default damping to zero for the baseline. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/desktop.py` | Launches driver and gate from one configuration snapshot and owns an optional camera child without treating camera failure as a robot-stack failure. The existing tf_prefix quoting/parser repair is retained. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/launch/commissioning.launch.py` | Logs effective startup values and their sources. Servo timing and protection parameters are unchanged. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/commissioning.py` | Gives specific Servo halt/stale messages and baseline damping defaults. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/tracking.py` | Changes only the damping defaults to zero; damping support remains. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/gui.py` | Shows specific Servo status, baseline damping defaults and camera controls/status; cleans up the camera viewer. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/camera_view.py` | Adds a latest-frame RGB/depth viewer at a capped display rate. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/package.xml` | Declares camera and Pillow viewer dependencies. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/test/test_preferences.py` | Tests snapshot immutability/source precedence and specific Servo status labels. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/test/test_desktop.py` | Tests that optional-camera failure leaves the robot stack running. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_d405_camera/CMakeLists.txt` | Builds only the SDK capture helper and installs the separate ROS publisher. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_d405_camera/package.xml` | Defines the new optional camera package and dependencies. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_d405_camera/src/d405_capture.cpp` | Streams RGB, raw depth and factory calibration from the serial-selected D405 without ROS library conflicts. |
| `/home/ubuntu-ros/ur_teleop_ws/src/ur_d405_camera/src/d405_camera.py` | Publishes metric depth/RGB/calibration and reconnects capture independently of robot control. |
| `/home/ubuntu-ros/ur_teleop/config/real_teleop.yaml` | Selects damping-zero defaults and the optional D405. ServoJ remains 500/0.03 s. |
| `/home/ubuntu-ros/ur_teleop/config/operator_settings.json` | Saves the requested conservative diagnostic baseline after backing up the prior profile; other current tuning is preserved. |
| `/home/ubuntu-ros/ur_teleop/scripts/record_sway.sh` | Records the full motion signal chain in one bag without enabling motion. |
| `/home/ubuntu-ros/ur_teleop/DIRECT_STATE_AND_D405_20260924.md` | This investigation, validation plan, camera limitations and complete source change list. |
| `/home/ubuntu-ros/ur_calibration_fix_ws/PROJECT_STATUS.md` | Adds the current outcome and supersedes earlier velocity-mode/damping-default descriptions as historical. |
