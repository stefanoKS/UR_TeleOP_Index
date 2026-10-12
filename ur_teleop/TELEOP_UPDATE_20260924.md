# Teleop update — 24 September 2026

## Findings

The earlier blank/broken RViz robot was caused by a controller-manager startup failure,
not a failed ping. PyYAML removed quotation marks around `$(var tf_prefix)`; ROS
substituted an empty prefix, leaving an invalid null parameter. The current local
`desktop.py` already restores those quotes. The supplied four-line `safe_dump`
snippet alone does **not** fix that issue. The launcher now additionally validates
the substituted file using ROS's native parameter parser before starting processes.
It detects nested launch-child crashes and reports/stops its owned stack. It does
not stop unrelated/manual ROS sessions.

Active source is `/home/ubuntu-ros/ur_teleop_ws/src/ur_vr_teleop/ur_vr_teleop/desktop.py`;
`/home/ubuntu-ros/ur_teleop/src` does not exist on this machine.

The later `20260924_115804_ea4f2d22` log confirms both FIFO scheduling and final
native smoothing were active. It also contains a native controller error with the
old generic timing/Ruckig message. That log does not record enough detail to identify
its precise cause. New failure logs distinguish timing, scaling, invalid input and
output-derivative failures and include the period, joint and Ruckig result.

## Why the MoveIt example can be smooth

Installed `ur_moveit_config/config/controllers.yaml` uses the scaled controller's
`FollowJointTrajectory` action; the planning adapters include time parameterization.
This is still UR position control, not evidence that velocity hardware control is
required. Complete planned trajectories have a different temporal structure from
replacing a short target every 10 ms.

[Official UR MoveIt usage](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/move.html#using-moveit)
and [ROS Humble interpolation](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)
explain the controller route and position-only velocity discontinuities. Position
plus consistent velocity gives cubic interpolation; position, velocity and acceleration
give acceleration-continuous quintic interpolation. Inconsistent derivatives can
overshoot, so no fabricated endpoint zeros or arbitrary acceleration fields are added.

The prior native smoother used the short segment's interpolated velocity as feed-forward.
With delayed feedback, that slope acts like an additional high-gain position loop.
A new test repeatedly sent a stationary position after moving, with 40 ms simulated
actuator lag. The old implementation oscillated enough to hit its 0.10 rad following
guard despite bounded acceleration. This reproduces a software problem, not the
robot's C306A3 itself.

Correction: enable Servo's genuine velocity output, preserve its joint ordering and
10 ms horizon, and use that explicit velocity for single-point streaming feed-forward.
Legacy position-only single points get modest position correction without the
artificial interpolation-slope feed-forward. Genuine all-zero Servo velocity requests
brake through the native ramp. The 100 ms explicit hold and watchdog remain. The
native final position reference still obeys configured velocity, acceleration and
jerk limits. Multi-point trajectories retain the base interpolator feed-forward.

Source inspected: [MoveIt Humble Servo calculations](https://github.com/moveit/moveit2/blob/humble/moveit_ros/moveit_servo/src/servo_calcs.cpp).
[UR ServoJ documentation](https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/servoj_qavt0-008lookahead_time.htm)
warns about noisy setpoints/gain/lookahead; its `a` and `v` arguments do not implement
acceleration limiting. No additional blind gain/lookahead changes were made here.

## Target trips and recovery

The old gate assumed requested joint positions must be attained in one 10 ms period,
so its 0.005 rad allowance conflicted with intentional acceleration-limited lag.
Only when the expected native smoothing controller/settings are verified and fresh,
the gate now allows a bounded 0.05 rad requested-position lead and consecutive step.
The final native stage still limits actual commanded derivatives. Input velocity has
a separate gross 10 rad/s sanity bound; the native applied speed remains 0.5 rad/s
by default. Large jumps, invalid/future/old/duplicate timestamps, stale feedback,
optical loss, native following error and command watchdogs remain checked.

For a recoverable gate trip: keep Enable teleop checked, release A, wait for fresh
inputs and stationary joints (0.5 s), then press A again. No Disarm/Start Servo/re-enable
cycle is needed for this case. A real robot or native-controller fault still requires
resolving the fault and explicit controller preparation; it is never auto-reset.

## Mapping, jitter and saved settings

GUI mapping fields express **VR input per one robot output**:
- Translation default **1.2:1** (robot/VR scale 0.8333333333333334).
- Rotation default **1:1**, configurable separately; incremental rotation remains continuous.
- Translation jitter band **1 mm**, rotation band **0.3 degrees**. Zero disables either.
- Translation/rotation filter alpha remains **0.8**. Lower adds smoothing and lag; **1** disables.

The spatial deadbands keep a target fixed for bounded tracking noise and advance
continuously beyond the band, rather than quantizing target jumps. Their maximum
small displacement offset is the configured band times the mapping scale. They
cannot remove arbitrary slow optical drift. Settling velocity also now approaches
zero continuously at the pose-error tolerance, avoiding the former threshold jump.
Neither filtering nor command bounds prove the physical robot will be vibration-free.

**Apply and save parameters** writes validated tuning atomically to
`/home/ubuntu-ros/ur_teleop/config/operator_settings.json`, only after the backend accepts
the request. Next launch loads that file over the base profile. **Reset to default
parameters** applies and saves the documented defaults. Robot address/calibration,
Enable and verification confirmations are not stored as tuning.

Joint speed is a native-controller startup parameter. Saving a changed joint speed
preserves the request for **Restart ROS**, while other accepted tuning applies now.
The GUI's Applied/Final joint command lines report the currently running values;
editing a field alone does not change them. Restart always starts disabled.

## Verification

- **154 Python/ROS checks passed**, covering mapping, new-sample filters, stationary-noise rejection,
  slow movement through the band, rotation beyond 180 degrees, atomic saved preferences,
  pending/rejected GUI saves, startup-only speed settings, stale inputs and release-only
  recovery. ROS transport tests use localhost domain 147, no robot command publisher.
- Seven native tests pass. Actual controller/plugin interpolation with a 40 ms mock
  feedback lag produced maximum command-write finite differences of about 0.270 rad/s,
  1 rad/s² and 10 rad/s³. The last stationary-input interval peaked at about
  0.000016 rad/s without requiring a hold. This extended test failed before the correction.
- Full generated UR driver startup using GenericSystem mock hardware, localhost domain
  148 and robot address 127.0.0.1 passed: six joint states and base_link-to-tool0 TF.
  The physical hardware plugin, dashboard connection and motion controllers were not activated.
- GUI rendered/inspected; mapping fields and both save/reset buttons are reachable by scrolling.
- No real robot motion or protective-stop reset was performed by the agent. Hardware
  smoothness and tracking must still be verified with the updated stack.

Close the old launcher window and reopen `~/ur_teleop/start_teleop.sh` once to load the
new GUI/code. Continue using the GUI for Preview, Live setup and explicit preparation.
