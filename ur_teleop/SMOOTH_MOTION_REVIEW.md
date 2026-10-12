# UR teleop motion review — 2026-09-24

The reported acceleration sanity-check message is consistent with C306A3. The
complete pendant codes/support log have not yet been supplied. Neither that text
nor wrist3 entering fault establishes that singularity, acceleration commands,
gain, payload, firmware or hardware is the sole cause. Motion after the changes
below has NOT been tested on the real robot by this agent.

## Primary sources and what they support

- [UR C306 error reference](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-err-codes/topics/CODE_306.html)
  names C306A3 as an acceleration sanity-check failure. It does not prescribe a
  numeric ROS acceleration limit or identify a unique cause for A3.
- [UR servoj manual](https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/servoj_qavt0-008lookahead_time.htm)
  explains that higher gain sharpens response and short lookahead reduces delay,
  but either can contribute to instability/vibration with noisy or infrequent
  target updates. This supports trying less aggressive tuning, not a guarantee.
- [UR driver hardware parameter documentation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/hardware_interface_parameters.rst)
  connects higher servoj gain with jerk/vibration and longer lookahead with smoother
  motion at the cost of latency. The installed Humble description was inspected:
  it hard-codes gain2000/lookahead0.03s. The current running driver previously
  reported those same values through robot_description.
- [UR driver discussion325: UR5 Servoing](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/discussions/325)
  reports simulation working but rough vibration on a physical UR5 using MoveIt
  Servo and JTC. The follow-up reports URSim success, not a demonstrated hardware
  fix for this UR10e. It is a related report, not evidence to copy its controller
  choice blindly. This project retains the scaled controller/watchdog.
- [First-hand UR10e/Humble lower-rate report](https://forum.universal-robots.com/t/looking-for-advice-on-achieving-smooth-operation-of-ur10e-at-lower-controller-manager-frequencies/32026)
  reports vibration/protective stops after reducing the controller-manager rate.
  Our driver rate remains500Hz; Servo target generation is a separate100Hz loop.

## Changes implemented

1. 1:1 anchored spatial mapping retained; default proportional response2/s.
   New vector-norm linear/angular slew limits cover ordinary braking/reversals.
   Disabled/released/stale/faulted command history resets immediately.
2. Separate new-sample translation EMA and rotation interpolation alpha0.8, each
   configurable and disabled with alpha1. Raw pose validity/jump checks run first.
3. Filtered controller rotation increments are conjugated into robot axes and
   accumulated from the clutch anchor. Quaternions are handled as rotations, not
   Euler angles; sign flips do not change targets. Angular commands remain slew
   bounded through a shortest-path error branch crossing at180degrees. This does
   not promise multi-turn wrist tracking or bypass joint/singularity limits.
4. Source-timestamp finite differences of Servo joint targets produce dq/ddq,
   canonical joint names, dt and throttled warnings. They never modify targets.
   Warning threshold5rad/s² is only a diagnostic setting. These values are command
   estimates, not measured acceleration or the internal UR sanity-check signal.
5. Moving horizons/position-only interpolation and100ms stop holds preserved.
6. Managed launcher generates local xacros with configurable gain500/lookahead0.08s
   for the next driver startup. These are experimental starting values chosen to
   reduce aggressiveness, not an official optimal pair. Installed UR files and
   currently running hardware settings are untouched.
7. One desktop/application shortcut opens the panel and preview stack. Live setup,
   Prepare controller, Start Servo solver, Restart ROS, Stop ROS and Open logs are
   buttons. Failures stop owned companion processes. Restart is operator-requested
   and always resets teleop intent/confirmations; fault reset, brake release and
   program Play remain on the pendant. Existing foreign stacks are not killed.

## Review before the next live trial

Stop the old manually launched stack once and use the new launcher so the driver
receives the new tuning. First check a ruler-measured10cm hand translation in
preview with little wrist rotation; hand and settled target should read10cm, while
the stationary measured tool stays still. Check each axis and a small rotation.
Confirm tool0 versus the fitted tool/TCP and payload on the pendant.

After resolving the pendant fault, use short low-speed live trials to compare
hand/target/measured tool, tracking error and wrist dq/ddq warnings. Release before
large lag develops. Record the full pendant error/support log if C306A3 recurs.
Do not repeatedly recreate the fault while raising speeds/gain. Continued faults
or vibration with conservative commands require UR/service investigation; these
changes cannot certify the joint healthy or guarantee maximum safe performance.

Logs are in ~/ur_teleop/logs/<session>/. The old joint-target lead check is unchanged;
if it trips, retain its message and command diagnostics rather than raising limits
to hide it. Robot-side physical acceleration, stopping and sound remain unverified.
