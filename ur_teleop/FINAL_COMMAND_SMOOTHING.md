# Wrist acceleration fault and final-command smoothing — 2026-09-24

Current correction and GUI workflow: [24 September update](TELEOP_UPDATE_20260924.md).
The evidence below describes the earlier session unless stated otherwise.

## Evidence and diagnosis

The user reports wrist-1 C306A3 during downward teleoperation and continuing vibration.
The saved `logs/20260924_084314_bba424b6/driver.log` records C306A3 at
1790207179.503. Its startup also reports failure to enable FIFO scheduling.
That session ran `6.8.0-138-generic` with RT-priority soft/hard limits of 0.
Upstream Servo diagnostics in the same session show wrist-1 finite-difference
acceleration excursions exceeding 100 rad/s². Some occur at clutch transitions;
they are not measurements of robot acceleration and do not establish the fault's
exact cause.

[UR's C306 documentation](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-err-codes/topics/CODE_306.html)
identifies A3 as an acceleration sanity-check failure, without identifying a unique
software cause. [UR's ServoJ manual](https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/servoj_qavt0-008lookahead_time.htm)
warns that noisy targets, high gain, short lookahead and low update rates can cause
vibration. ServoJ's `a` and `v` arguments do not implement acceleration limiting.

[Humble JTC interpolation](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)
uses linear interpolation for position-only segments; velocity continuity is not
guaranteed at their boundaries. The installed UR scaled controller then samples
these segments at 500 Hz. Therefore limiting Cartesian acceleration before IK or
only filtering 100 Hz waypoints cannot bound the final joint-reference acceleration.

## Implemented correction

New plugin `ur_teleop_controllers/SmoothScaledJointTrajectoryController` derives
from the installed UR scaled controller. Its update calls the existing scaling,
watchdog and trajectory logic, then replaces the position command interfaces
before `hardware.write()` with a continuous Ruckig-generated reference.
[Ruckig](https://github.com/pantor/ruckig) provides jerk-constrained online motion
generation. This implementation uses the locally installed version 0.9.2.

The velocity reference now combines explicit Servo velocity for single-point streams
(or the interpolator velocity for multi-point trajectories) with a
5/s position-error correction and measured speed scaling. Ruckig limits velocity,
acceleration and jerk, integrating the previous commanded state. It does not
re-anchor to noisy measured positions every cycle. Standstill is required on
activation. Nonfinite input, a control period outside (0,20 ms], or a following
error above 0.10 rad fails the controller; unfiltered targets are never intentionally
passed through on rejection. These are software command bounds, not certified
stopping or physical motor-acceleration guarantees.

Defaults are 0.5 rad/s, 1 rad/s², and 10 rad/s³. The launcher takes velocity from
`teleop.max_joint_speed_rad_s`, and acceleration/jerk from
`robot.joint_command_acceleration_rad_s2` and `robot.joint_command_jerk_rad_s3` in
`config/real_teleop.yaml`. It writes a session-local `controllers.yaml`. These limits
are startup-only; the GUI reports their actual controller parameter values and
requires saved changes to joint speed to take effect after Restart ROS. Other tuning
can be applied while disabled. Live arming requires the new plugin and readable settings.

Clutch geometry remains anchored to each press. Default translation is now VR:robot
1.2:1, rotation 1:1, with configurable spatial jitter bands. Incoming 10/20 ms moving
segments remain unchanged, with no artificial zero velocities or 100 ms stretching.
The existing software-stop message remains a 100 ms zero-velocity hold; the new
final stage treats it as a request to brake to zero velocity instead of returning
to a stale measured pose. Acceleration-limited braking takes time and distance:
at the maximum 0.5 rad/s, a 1 rad/s², 10 rad/s³ stop from zero acceleration takes
approximately 0.6 seconds. The pendant's emergency/protective-stop paths remain
independent. Do not interpret the old 100 ms message horizon as a guaranteed stop
time. Low limits can add lag and change the transient joint path; collisions,
singularities and robot limits remain relevant.

The new `/scaled_joint_trajectory_controller/smoothed_command` topic samples the
actual final reference at 100 Hz, including position, velocity and acceleration.
`scripts/diagnose_motion.py` now records it and its peak derivatives. The existing
Servo warning log is explicitly labelled as upstream of final smoothing.

## Validation and remaining hardware work

131 Python unit/isolated ROS checks passed. Six native smoothing checks plus a
real plugin test using local doubles as hardware interfaces passed. The latter
runs the actual UR/JTC interpolation, reversals, a simulated 40 ms actuator lag,
software hold, watchdog expiry and a deliberately invalid control period. It
checks finite differences of final position interface writes, not only Ruckig's
reported derivatives. Command bounds remained within 0.5 rad/s, 1 rad/s² and
10 rad/s³. The invalid period returns ERROR and retains the previous output.
No hardware plugin, real command publisher, robot program or motion was started
by these tests. Physical vibration and the C306A3 fault have not been retested.

## Host scheduling and next run

[UR recommends real-time scheduling and a real-time or low-latency kernel](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_client_library/doc/real_time.html)
for e-Series motion. The current missing FIFO permission is a separate unresolved
source of timing risk; command smoothing cannot recover a missed hardware update.

Run once in your own terminal (sudo needs your password):

```bash
sudo ~/ur_teleop/scripts/setup_realtime_permissions.sh
```

This writes user-specific PAM and systemd-user RT-priority limits. It does not
restart anything or install a kernel. Log out completely and log back in; then:

```bash
~/ur_teleop/scripts/setup_realtime_permissions.sh --check
~/ur_teleop/start_teleop.sh
```

The check should report permission for priority 50. UR also recommends a low-latency
or PREEMPT_RT kernel; the host still has a generic kernel. Installing/booting a new
kernel is separate host maintenance, not performed here. On a future live run,
confirm the GUI shows final joint limits and the driver no longer reports FIFO
failure. Use the existing Live setup / Prepare controller workflow and modest
movement first; inspect a read-only diagnostic capture before increasing limits.
If C306A3 persists with bounded final references, stop repeated fault-producing
trials and export a UR support log for payload/mechanical/controller diagnosis.
