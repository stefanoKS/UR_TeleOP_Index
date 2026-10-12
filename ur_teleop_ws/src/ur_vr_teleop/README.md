# V2 velocity teleoperation

The commissioning default is now velocity-following; position-following remains selectable.
See [the V2 implementation and commissioning guide](../../../ur_teleop/TELEOP_V2.md)
for parameters, single-station policy, validation, recording and rollback.
The older position-mode notes below remain useful for the regression mode.

# UR10e teleop commissioning v0.1.0

This is a local implementation following the other chat's intended workflow;
the generated ZIP from that chat was not recovered. Existing preview behavior
is retained. New `commissioning` and `gui` executables are **dry-run by default**.
Source/build/local tests are not hardware acceptance.

Current implementation/research and regression results are documented in
`/home/ubuntu-ros/Documents/Teleop/ur_teleop/TELEOP_UPDATE_20260924.md`. This includes the corrected
streaming feed-forward, RViz startup repair, jitter controls and saved settings.

## Adjustable damping

Translation and rotation damping default to 1.0; each accepts 0–1, with 0 disabling.
The damping velocity filter defaults to 50 ms and accepts 5–200 ms. These controls
oppose filtered **measured tool velocity**, rather than slowing the VR pose filter.
They are dimensionless outer velocity-loop gains, not UR firmware damping settings.
They can slow following and cannot guarantee stability with arbitrary response gain
or delay. Apply while disarmed/A released; Apply and save parameters persists them.
Existing saved speed, mapping and response preferences are retained. The applied
status shows the values currently in use. See
`/home/ubuntu-ros/Documents/Teleop/ur_teleop/DAMPING_UPDATE_20260924.md` for tests and limitations.

## Mapping, saved tuning and smoother command response

The real profile defaults to VR:robot translation **1.2:1**, `translation_scale: 1/1.2`: the requested tool0 target
equals the tool pose captured on pressing A plus the scaled/aligned displacement of the
controller from that same press. Alignment is a rotation, so it cannot magnify
distance. Rotation defaults to 1:1 with a separate editable mapping ratio. Each new A
press captures fresh hand/tool anchors; targets are never integrated repeatedly.
This describes requested displacement, not instantaneous physical tracking.

Real tracking now ramps ordinary velocity changes at configurable defaults
0.25 m/s² and 90 deg/s² and defaults to response gain 2/s. The vector-norm slew
bound applies to acceleration, ordinary deceleration and reversal; release, disable,
tracking loss and faults immediately clear history. Raw jumps are checked before
light pose filtering. New-sample translation EMA and rotation interpolation use
alpha 0.8; alpha 1 disables either filter. Additional continuous spatial deadbands
default to 1 mm and 0.3 degrees; zero disables them. These add a small bounded
position/angle offset while rejecting stationary tracking noise; filtering adds transient lag, not spatial magnification. Robot dynamics, tracking noise and
latency still require physical validation. The separate joint-target lead/rate
checks now distinguish upstream requests from native acceleration-limited output.
Driver servoj tuning is unchanged by this update.

The GUI displays hand, target and measured tool0 displacement (cm/degrees) from
the current A-press anchors, plus target–tool position error. After release it
labels retained numbers as **Last clutch sample**; these are not a post-stop
measurement. Compare the tool/flange, not the elbow or another point on the arm.
If a custom physical tool is fitted, tool0 may differ from its tip/pendant TCP.

First verify mapping with `motion_output_enabled:=false`: move the controller
along a ruler by 10 cm without rotating it, using the same controller reference
point throughout. Hand and target should both read about 10 cm; measured tool
should remain still. A wrong hand distance indicates a tracking/calibration issue;
a persistent different target distance after settling indicates mapping/configuration. Only a subsequent
controlled live trial can measure physical following/overshoot. Do not treat the
software changes as confirmation that the previously reported sound is resolved.

## First hardware check: no robot motion

Keep the existing UR driver/RViz terminal running with all motion controllers
inactive. Stop only the old bridge and old `preview` node using Ctrl+C in their
own terminals. Do not run the old and new preview at the same time (duplicate markers).
Do not stop unrelated processes. Outside Conda, source each terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=43
```

Terminal A — restart the updated bridge (banner must say v0.2.0):

```bash
ros2 run ur_vr_bridge controller_bridge --ros-args -p device_serial:=LHR-4632363E
```

Terminal B — start Servo, timestamp relay and commissioning gate, **no UR driver**:

```bash
ros2 launch ur_vr_teleop commissioning.launch.py
```

Terminal C — local operator GUI:

```bash
ros2 run ur_vr_teleop gui
```

The panel must say **DRY RUN — ROBOT OUTPUT DISABLED**. Commissioning now opens
RViz with `/vr_real/preview_markers` enabled automatically. The robot mesh shows
measured joints; the orange marker shows the requested tool target. Dry run does
not move the robot mesh or the physical arm. Close the older driver RViz window
if desired; at the next driver startup use `launch_rviz:=false` to avoid two windows.
Use `launch_rviz:=false` on commissioning if you want no additional RViz.
Click **Start Servo solver** in the GUI.
This starts computation on a staging topic; it does not activate a robot controller.
The GUI lists readiness blockers rather than silently bypassing them.

1. With both base stations visible to the controller, verify Optical becomes fresh
   and each of at least two LH entries has >=4 paired sensors. Move/rotate through
   the intended range while observing the counts.
2. While the robot remains inactive, obscure the controller's optical sensors
   from both stations. Optical must become LOST even if pose_fresh remains true.
   Test loss of one station too: this commissioning configuration requires two.
   Do not move base stations during this test. Restore visibility and verify recovery.
3. Verify all three translation directions and small rotations in preview. The
   reference is model `tool0`, not a custom pendant TCP. The physical setup must
   match the bare-flange/zero-offset model before any real-motion trial.
4. Release A; check the two GUI confirmations only after those tests pass, then
   Apply settings, or **Apply and save parameters** to restore tuning next launch.
   **Reset to default parameters** applies/saves defaults. Joint speed changes require
   saving and Restart ROS. Enable and verification checkmarks are not restored.
5. With no remaining blockers, **Enable teleop in DRY RUN**, release A and wait for ARMED, then hold A and
   move a few centimetres. Accepted targets should increase; the real arm stays still.
6. Release A, then test optical loss while A is held. The gate must disarm, target
   acceptance must stop. Keep A held while restoring tracking: motion must NOT resume.
   Release A and wait for healthy/stationary recovery; then a new A press is required.
   Close the GUI while dry-run armed: heartbeat loss must disarm the backend.
7. Inspect `ros2 topic info /scaled_joint_trajectory_controller/joint_trajectory`:
   dry run must have zero publishers on this topic. `ros2 control list_controllers`
   must still show motion controllers inactive.

If optical remains false, inspect `/vr/controller/optical_status`; a pose in RViz
is not sufficient. Do not lower thresholds to make the status green. The optical
monitor supports the current Gen2 Lighthouse setup only and measures recent
accepted light observations, **not pose accuracy or a certified safety signal**.

## What the command path checks

`VR + measured joints/TF → clutch → Twist → MoveIt Servo → timestamp relay → gate
→ scaled_joint_trajectory_controller` (last publisher absent in dry run).

- A released at startup/recovery, explicit enable request, valid named finite joints,
  measured standstill at arm, wall-clock and receipt freshness, input/target jumps.
- Both calibrated Lighthouse stations, four sensor pairs per station within
  250 ms, and 500 ms stable optical recovery before arm is possible.
- Robot mode RUNNING, safety NORMAL/REDUCED, External Control running, fresh
  speed scaling (the UR topic reports percent), and fresh Servo status without halt.
- Optional startup workspace fence (disabled by default), operator-selected Cartesian speed limits;
  Servo self-collision/singularity/joint-limit handling remains enabled.
- GUI heartbeat, expected controller state, one publisher per required input,
  no competing command publisher and no active trajectory action.
- Canonical six-joint absolute-position targets, one short point, measured-position
  lead and consecutive step <=0.05 rad with verified native smoothing (legacy unsmoothed
  path retains speed × horizon/rate checks), finite bounded requested derivatives,
  and fresh strictly increasing source timestamps. No gripper placeholders.

Humble Servo can publish stamp=0 for immediate execution. A C++ relay captures
the DDS publisher timestamp into the staging message header, and rejects missing,
future or >=100 ms old source times. Python gate checks that timestamp again,
including that it was published after the clutch press. It never makes a queued
message fresh by using its local receipt time. Nodes must run on the same PC
using wall time. Moving targets retain genuine Servo velocities; no fabricated zero
endpoint velocities are added to streaming targets. Stop holds still
include zero velocities. The final robot trajectory has stamp=0 and preserves the validated Servo horizon
(normally 10 ms, accepted range 5–100 ms); future recording must preserve both command receipt and raw timing.

The installed Humble Python subscription API does not expose DDS MessageInfo;
the relay is intentional. Upstream timestamp behavior is also visible in
[Servo source](https://github.com/moveit/moveit2/blob/humble/moveit_ros/moveit_servo/src/servo_calcs.cpp).

## Before a separately authorized live-motion trial

Do not simply switch the controller active during dry run: that is a readiness
fault. The later live trial requires stopping the dry-run commissioning launch,
reviewing the physical workspace/TCP/calibration and direction mapping, explicitly
authorizing the controller activation, and relaunching with live output enabled.
Nothing in this launch or GUI starts the driver/program, switches controllers,
clears safety faults or runs learned policies.

For live output, the gate additionally requires the scaled trajectory controller
to be the sole active joint-command controller and its `cmd_timeout` parameter
to be readable and within 0.10–0.25 seconds, greater than `constraints.goal_time`.
The installed driver default is not sufficient. A new copy of the installed
controller configuration is prepared at
`/home/ubuntu-ros/Documents/Teleop/ur_teleop/config/commissioning_controllers.yaml`, preserving all
other controller settings and adding `cmd_timeout: 0.2`. Load this file at the
next deliberately supervised driver startup using its `controllers_file` argument.
Do not treat a runtime parameter-set response as proof of the effective watchdog:
the controller's internal timeout is initialized during configuration. The gate
only reads parameters and never changes them or restarts the driver. Physical
watchdog behavior must still be measured during the authorized motion trial.
Launch argument `motion_output_enabled:=true` creates the robot command publisher
but still starts disarmed. **Do not use it until the stationary/dry-run tests pass
and the live-motion step has explicit authorization.**

On A release or a fault, zero Twist is sent and, when fresh joint feedback exists,
the gate requests a bounded hold at the captured measured positions. On feedback
loss no new hold is fabricated from stale measurements. In-flight finite targets
and controller timeout bound continued command execution, but pendant speed
scaling and robot dynamics affect physical stopping. Process-kill, release and
fault stopping distances still need supervised low-speed hardware measurement.
Disarm is a software stop request, not an emergency stop.

The collision model has no table, jig, gripper, cables or people. Do not treat
Servo self-collision checking as environmental collision protection. Camera
capture, recording and ACT remain deferred.

## Preserved observation-only preview

The `preview` executable is **preview only**. It cannot command robot motion. It subscribes
to live robot/controller observations and publishes MarkerArray and String only.
It has no arming service, controller client, trajectory publisher or Servo output.

Keep every robot motion controller inactive. Leave the existing driver and bridge
running in domain 43. In a new terminal outside Conda:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=43
ros2 run ur_vr_teleop preview --ros-args \
  --params-file ~/Documents/Teleop/ur_teleop/config/real_preview.yaml
```

Open the bundled view (also available through RViz File → Open Config):

```bash
rviz2 -d "$(ros2 pkg prefix ur_vr_teleop)/share/ur_vr_teleop/rviz/commissioning.rviz"
```

It includes the measured robot and target MarkerArray in `base_link`. Release A: an orange sphere with RGB axes
should appear at measured `tool0`. Hold A and slowly translate/rotate the Index:
the target moves, **the robot must stay still**. Release A: target returns to
measured tool pose. Reposition your hand and press A: no target jump on capture.
RGB axes are X red, Y green, Z blue. `tool0` is the model flange reference, not
necessarily the active custom TCP on the pendant.

Move your hand roughly 5 cm along each physical direction separately, observe
the requested target direction, and record which robot-base axis it corresponds
to. The initial identity axis mapping is unverified. Do not assume robot
`base_link` axes equal the pendant's Base coordinates (the UR model includes a
rotation between `base_link` and `base`). No fake lighthouse-to-base TF is added.

```bash
ros2 topic echo /vr_real/preview_state
```

WAIT_RELEASE means release A after healthy observations return. Markers disappear
for invalid/stale observations, stale TF, or clutch faults. They expire after
0.3 s if the node dies. The preview reuses the simulation's tested clutch math,
scale 0.8, 20 cm workspace and jump checks. Its discarded velocity calculation
does not rate-limit marker position or establish any real robot speed limit.

This legacy `preview` executable checks only pose-stream freshness, not optical validity. Tracking
loss can leave IMU-predicted poses streaming. Use `commissioning` for the guarded
dry run described above; do not remap this executable into a motion path.

Build without launching anything:

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/Teleop/ur_teleop_ws
PATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --packages-select ur_vr_bridge ur_vr_teleop
```


## Revised commissioning workspace and observation timing

`real_teleop.yaml` disables the fixed startup-centered workspace fence and uses
a 350 ms maximum pose/joy/joint/TF observation age.
These startup-only parameters are `workspace_radius_m` (0 disables; optional 0.05–0.50 m) and
`observation_timeout_s` (0.10–0.50 s). The same radius applies to the requested
target and measured TCP (with the existing 1 mm measurement tolerance).
The GUI shows the applied radius, timeout and speed settings; default trial
speeds in this file are now 0.10 m/s and 30 deg/s.

Optical sensor-pair freshness is now 250 ms; Servo command freshness, robot
watchdog, heartbeat and motion limits are unchanged. A fresh RViz display is not
proof that every observation source is fresh. Failures report source/receipt ages,
TF availability, loop gaps, or the actual clutch/workspace cause. The first stop
cause remains latched until successful recovery/arming or operator disarm.


## Operator-adjustable speed limits

The GUI accepts finite positive linear (m/s), angular (deg/s) and joint (rad/s)
speed limits without the former hard-coded 0.02 m/s, 10 deg/s and 0.10 rad/s upper
ceilings. Changes require disarm and A released. Apply settings, then verify the
Applied line before rearming. Use **Apply and save parameters** for persistence. Startup defaults in
`real_teleop.yaml` are 0.10 m/s, 30 deg/s and 0.50 rad/s respectively.

`max_joint_speed_rad_s` configures the native final speed limit at startup and a
measured-speed check at 120% of that limit. Save a changed joint limit and Restart ROS.
Verified native smoothing uses a 0.05 rad upstream lead/step guard instead of the old
one-period speed proxy, which conflicted with acceleration-limited lag.
The selected speeds are ceilings, not guaranteed speeds: the position-error
controller, Servo and robot controller can command slower movement. Streaming
segments preserve Servo timing (normally 10 ms) and genuine desired velocities. This change does not remove robot/model
joint limits, collision/singularity handling, optical stops, workspace limits,
source timestamp checks, heartbeat checks, standstill-before-arm or watchdogs.


## Persistent enable checkbox

Enable teleop sets an intent via `/vr_real/set_enabled` (`std_srvs/SetBool`),
not an unconditional motion permission. Faults disarm the command gate but retain
that intent. With A held, recovery never rearms. Release A, restore all required
observations/controller states and standstill, and wait for 500 ms of readiness;
the gate then arms with a new clutch reference. A subsequent press is needed to
move. The checkbox stays checked during this wait. Optical stability has its own
500 ms requirement, so recovery can take longer than half a second.

Unchecking or DISARM TELEOP clears intent and requests a hold. Closing/losing the
GUI heartbeat also clears intent; process restarts start disabled. Uncheck before
editing settings. The legacy Trigger arm service remains manual (it does not
create persistent enable intent). The checkbox never starts Servo, controllers,
the driver or External Control, and never clears robot safety faults.

The optional radius, if explicitly reenabled, is measured around the first healthy
tool position after launch. The default is OFF; robot reach and model limits still apply. The updated bridge accepts a
250 ms window of optical pairs and defaults to a 350 ms pose timeout; four paired
sensors per station and two calibrated stations are still required. Restart the
bridge as well as commissioning/GUI to load these changes. Controller and Servo
command watchdogs are unchanged.


## Tracking response and matched trajectory timing

Hand displacement defaults to VR:robot 1.2:1, using `translation_scale: 1/1.2`;
rotation defaults to 1:1 via `rotation_scale: 1.0`. Both ratios are editable in GUI. Each fresh A press captures new hand/tool anchors.
This is geometric target mapping, not a guarantee of instantaneous physical tracking.

Commissioning and Servo now run at 100 Hz with 10 ms targets. The gate preserves each
validated target horizon instead of stretching it to 100 ms. Genuine Servo velocities
are preserved and used as native feed-forward for short streaming points. Explicit
stop holds still use 100 ms and zero endpoint velocities. Requested lead/step bounds
remain distinct from the native final velocity/acceleration/jerk bounds.

`tracking_gain_s_inv` controls velocity from target error before the selected
speed limit is applied. Default is now 2/s, followed by the Cartesian slew limiter;
the GUI exposes it as Tracking response (1/s). The simulation package is unchanged.
Startup defaults are 0.10 m/s, 30 deg/s, 0.50 rad/s; they are a testable response profile,
not a claim that maximum possible robot speed or 1:1 physical tracking is verified.
The last applied alignment [0, 0, -90] is saved; direction confirmations still reset.

UR's streaming-control guidance and hardware parameter documentation were checked:
- https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/usage/position_velocity_control.html
- https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/hardware_interface_parameters.html

The original installed description hard-codes servoj_gain 2000 and lookahead 0.03 s.
The managed launcher described below generates a local description with configured
gain 500 and lookahead 0.08 s; it does not modify /opt or an already running driver.
The scaled trajectory controller
is retained with its watchdog. Code/isolated tests cannot establish physical lag,
tracking accuracy, overshoot or stopping performance; measure those on the robot.

## Single-window startup and recovery (2026-09-24)

The managed scaled controller now uses `ur_teleop_controllers` for final 500 Hz
joint-command acceleration/jerk limiting, after UR trajectory interpolation.
Build that package alongside this one. The launcher generates its limits from
`real_teleop.yaml`: velocity from `teleop.max_joint_speed_rad_s`, acceleration/jerk
from `robot.joint_command_acceleration_rad_s2` and `robot.joint_command_jerk_rad_s3`.
Defaults are 0.5 rad/s, 1 rad/s² and 10 rad/s³. The GUI reports the loaded limits;
changing this final layer requires restarting the driver. Software braking can
take about 0.6 s from the maximum joint speed; the 100 ms hold message is not a
guaranteed physical stop time. Pendant stop paths remain independent. See
`~/Documents/Teleop/ur_teleop/FINAL_COMMAND_SMOOTHING.md` for the validation, scheduling-permissions
fix and hardware verification procedure. Vibration elimination is not yet verified.

Stop the previous manually launched stack in its own terminals once. The manager
refuses to start alongside a detected competing stack and never kills/adopts it.
Open **UR10e Teleop** from the desktop/application menu, or run:

```bash
~/Documents/Teleop/ur_teleop/start_teleop.sh
```

The panel starts the driver, tracker, Servo/gate and RViz in **preview**. For live
setup use the **Live setup** button or `~/Documents/Teleop/ur_teleop/start_teleop.sh --live`.
Every start/restart leaves the scaled motion controller inactive and teleop disabled.
No robot power-on, brake release, protective-stop unlock, fault reset, program Play
or motion enable is automated.

After resolving faults on the pendant and starting External Control, release A,
keep Enable teleop unchecked, and use **Prepare controller** (live setup only).
This requests strict activation of the scaled trajectory controller without stopping
any other controller. Robot status must show ready and measured standstill.
Then use **Start Servo solver**, review/apply settings and confirmations, and enable
teleop separately. Preview does not need controller activation.

**Restart ROS** stops/restarts only this panel's owned processes in the selected
mode. It clears pending GUI requests; a new backend session resets GUI confirmations
and reloads defaults. The panel stays open through driver failure. Unexpected child
exit of a driver/tracking/teleop process stops its companions; recovery is a deliberate button press, not an
automatic fault-reset loop. **Stop ROS** leaves the panel open; closing it stops its
owned processes, including RViz. Terminal hangup, Ctrl+C and termination follow the
same cleanup path. A session ownership record lets the next launcher clean up its
own abandoned descendants after a crash; manually started stacks are never adopted.
Ownership uses an inherited session tag plus process start/boot identity, not names.

RViz is managed separately: closing RViz leaves ROS running, and **Open RViz**
reopens it with the commissioning marker display. Starting or restarting the stack
opens RViz automatically. **Open logs** opens `~/Documents/Teleop/ur_teleop/logs/<session>/`, containing
`driver.log`, `tracking.log`, `teleop.log`, `rviz.log`, generated xacros and launch arguments.

The new managed driver profile in `~/Documents/Teleop/ur_teleop/config/real_teleop.yaml` uses gain500
and lookahead0.08s as an experimental smoother starting point, not a guaranteed
optimum. Driver/controller-manager update rate remains500Hz; teleop/Servo remains100Hz.
Changing the robot settings `servoj_gain` or `servoj_lookahead_time` requires
Restart ROS. Cartesian acceleration/filter/diagnostic parameters are editable in
the scrollable GUI while disarmed and A released. Use Apply and save parameters
for persistence in `config/operator_settings.json`; plain Apply is session-only.

## Joint diagnostics and known limits

All five requested ROS parameters are supported: `max_linear_acceleration_m_s2`,
`max_angular_acceleration_rad_s2`, `vr_translation_filter_alpha`,
`vr_rotation_filter_alpha`, `joint_acceleration_warning_rad_s2`.
Alpha must be in(0,1]; acceleration and warning thresholds must be positive finite.
Alpha0 would freeze input and is rejected; use1 to disable filtering.

Before the existing forwarding gate, ordered Servo targets produce
dq=(q-current minus q-previous)/source_dt and ddq=(dq-current minus dq-previous)/source_dt.
Canonical joint names, source timestamps, stale/reordered/gapped baseline resets,
per-joint throttled warnings and GUI warning names make wrist spikes visible.
The default5rad/s² warning is a diagnostic threshold, not a UR limit. Targets are
not modified/clamped by this monitor. These estimates are neither measured joint
acceleration nor the UR internal sanity-check signal. A source burst can differ
from actuator timing; use robot-side logs and feedback for confirmation.

Incoming moving-point time_from_start is preserved (currently10ms, also supports20ms),
no artificial zero endpoint velocities are added, and existing100ms zero-velocity
hold points remain unchanged. Requested lead/step and final derivative checks remain.
Cartesian acceleration limiting does not guarantee joint acceleration limits near
poorly conditioned configurations. Quaternion sign changes and180-degree target
crossings are tested; rotation uses incremental filtered SO(3) deltas, not Euler
angle subtraction. Large target–robot orientation lag remains ambiguous and the
shortest-path error may change direction gradually under the slew bound; this is
not a multi-turn wrist winding controller.

Research and physical acceptance are recorded in
`~/Documents/Teleop/ur_teleop/SMOOTH_MOTION_REVIEW.md`. No claim of fault-free physical operation or
maximum speed is made by the software tests.
