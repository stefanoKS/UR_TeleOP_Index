# Sway recording and offline comparison

The startup simplification changes no Ruckig, Servo, servoj or hand-velocity algorithm. Its goal is trustworthy measurement of the reported oscillation. No hardware root cause or physical improvement is claimed.

## Record

1. Open Engineering / diagnostics → Advanced diagnostics → **RECORD SWAY DIAGNOSTICS**. Wait for the recording path.
2. Perform only a separately authorized supervised diagnostic session. Recording cannot enable a controller, request motion or bypass readiness. It can begin while disarmed.
3. Press **STOP RECORDING** and wait for completion. Use STOP TELEOP separately to disarm. Closing the application requests recorder shutdown and bag finalization.

The CLI uses the same duplicate lock and output directory:

```bash
./ur_teleop/scripts/record_sway.sh
# Ctrl+C finalizes the bag.
```

Each timestamped directory under `ur_teleop/diagnostics/` contains `bag/`, `recording.json` and recorder QoS. GUI feedback links the recorder log. Camera availability is irrelevant. The lock is held through finalization; an interrupted recorder releases its OS lock automatically. Do not delete unknown process/session locks.

Recorded topics include raw hand pose/source time, Joy/A, validity/optical status, raw/filtered hand velocity and measured TCP estimates in operator_status, actual published Twist, Servo targets before/after timestamp relay, gated trajectories, smoothed position/velocity, native control diagnostics, measured joints, TF, Servo state, speed scaling, robot mode/safety/program state and parameter events. Parameter reads at start/stop capture the teleop, Servo, native controller and controller-manager configuration. Missing reads are recorded as errors, not fabricated values. Operator status captures effective tuning changes throughout the session.

The selected/managed snapshot is also saved. It records driver-generation settings, including servoj tuning, but is not independent readback from the robot. Existing committed overrides are 1.5 m/s linear speed, 180 deg/s angular speed, 2 rad/s joint speed and −40° yaw, versus YAML 0.10 m/s, 30 deg/s, 0.5 rad/s and −90°. Configured servoj gain/lookahead are 500/0.03 s. Review runtime dumps/status before attributing those values to an actual session. No limits are increased by these tools.

## Analyze an existing recording

```bash
source /opt/ros/humble/setup.bash
source ur_teleop_ws/install/local_setup.bash
/usr/bin/python3 ur_teleop/scripts/analyze_sway.py ur_teleop/diagnostics/sway_TIMESTAMP
```

This reads an existing bag without creating a ROS node or connecting to the robot. `--output` chooses a report directory. JSON and Markdown reports accompany six separate joint plots (including individual wrist 1/2/3 plots in `joint_4.png`, `joint_5.png`, `joint_6.png`), Cartesian/hand-velocity plots, raw hand pose and speed scaling when available.

The report includes stage sign reversals, dominant frequency where supported, receive-time correlation lag, position error, command-stop to measured standstill, directional overshoot relative to a stationary final reference, settling time, receive-time acceleration estimates, effective native speed-cap overlays, frequency/gaps, speed-scaling variation, effective parameters and clock audits. Analysis defaults are 0.001 rad/s standstill, 0.002 rad position tolerance and 0.5 s observed dwell. These are analysis thresholds, not actuator limits. Change thresholds explicitly if appropriate; missing/insufficient data produces null metrics.

All cross-stream comparisons use bag receive time. Actual header/source timestamps are retained in the bag and audited separately. Device time is never subtracted directly from ROS epoch. Receive-time lag includes transport and sampling; Servo targets also have a future trajectory horizon. Correlation on periodic data is ambiguous and is not absolute actuator latency. No interpolation crosses gaps larger than 100 ms or extrapolates beyond a stream. Frequency estimates need at least two cycles and concentrated spectral energy; they are candidates for inspection, not a root-cause verdict. Stop metrics require an observed stop transition and sufficient continuous feedback; a truncated tail cannot prove standstill or settling.

## Interpret the stages

| Observation | Investigate |
|---|---|
| Stationary hand but oscillating Cartesian demand | Tracking noise, sample timing, velocity estimation |
| Stable Cartesian demand but reversing Servo velocity | Servo, feedback timing, IK and singularity scaling |
| Stable Servo request but oscillating native reference | Trajectory replacement/interpolation, Ruckig state handling |
| Stationary final reference but oscillating feedback | UR servo response/timing, speed scaling, TCP/payload and mechanical behavior |
| Same-sign decaying reference and feedback | Continued commanded deceleration rather than repeated corrections |

Inspect the actual time plots and configuration together. Whole-record reversal counts also include deliberate operator reversals. An FFT peak alone does not establish unwanted oscillation. Physical payload/TCP, stopping distance and wrist behavior require supervised validation.
