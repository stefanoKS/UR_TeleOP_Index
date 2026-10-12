# Controller tracking → ROS 2 Humble

## Local v0.2.0 commissioning update (2026-09-23)

Adds `/vr/controller/optical_fresh` (Bool) and `/vr/controller/optical_status`
(String) for the current Gen2 Lighthouse setup. An optical callback wraps and
preserves the installed libsurvive sweep-angle handler, then observes accepted
sensor timestamps in that same polling thread. A mutex protects only the monitor
state exchanged with the ROS timer; mutable libsurvive activations are not read
concurrently from ROS. Existing pose/input filtering is retained.

Stale-stream warnings also report selected-device raw IMU packet and Gen2 sweep
angle counts and their receipt ages (`-1` means never received). These diagnostics
preserve the original libsurvive callbacks and do not change freshness decisions.
`running=yes` means the library thread is running, not that the device is tracking.
The simple API can emit pose updates following button events with an uninitialized
pose; only `accepted` counts validated poses. If both event counters stop, leave
teleop disabled, check controller power/charge and receiver connection, power-cycle
the controller, then restart only the bridge. If sensor arrivals continue but poses
remain invalid, retain the calibration and collect the solver logs for diagnosis.

V2 optical signal freshness requires at least one sensor with both planes recently
accepted from either calibrated station within 250 ms, independently of pose output.
Commissioning accepts either calibrated station by default (`any_station`), with
separate fresh continuous 6-DoF pose validation. Legacy station policies are aliases;
there is no station-count speed reduction.
Pose headers now preserve solver sample intervals using a fixed ROS/source clock
anchor; `/vr/controller/source_time` retains the original solver timestamp.
Repeated/reordered light timestamps do not renew freshness. These are conservative
commissioning criteria, not a measure of pose accuracy or a safety-rated interlock.
Gen1 remains optically false in this implementation. Verify loss/recovery on the
actual device while robot motion controllers are inactive before any motion trial.

The package also includes `servo_stamp_relay`, an observation-only C++ relay used
by the real commissioning launch. It forwards `/vr_real/servo_unstamped_targets`
to `/vr_real/servo_joint_targets`, retaining publisher time from DDS metadata when
Servo's header is zero; it rejects missing/future/stale times. Neither executable
publishes robot motion commands. Backups of v0.1.1 are under
`/home/ubuntu-ros/Documents/Teleop/ur_teleop/backups/pre_commissioning/ur_vr_bridge`.

The historical v0.1.1 instructions below describe the unchanged base observation
interface. The updated startup banner now says v0.2.0.

Version 0.1.1 adds startup version/precision logging, libsurvive verbosity, and
throttled stale-stream diagnostics. It does not relax pose validation or change
motion behavior. Version 0.1.0 hid useful libsurvive startup information with
verbosity zero, making a missing pose hard to diagnose.

This package is the next observation-only step in the UR10e teleoperation setup.
It reads the already working libsurvive installation and publishes controller
position, orientation, inputs, identity and pose-stream freshness. It has no robot
command publisher, trajectory client, controller-switching client or robot IP.

Environment: Ubuntu 22.04, ROS 2 Humble, libsurvive tested at the API level against
revision `df22f2d`. The existing checkout is expected at
`~/Documents/Teleop/ur_teleop/tools/libsurvive`, built using its `make` command into `bin/`.

## 1. Prepare

Keep the real robot's External Control program stopped. Close survive-cli,
survive-buttons, api_example and SteamVR; only this bridge should open the receiver.
The controller must already be paired and tracking must already work in libsurvive.
Keep base stations fixed in their calibrated positions. Release all buttons before
starting the bridge. Leave the existing calibration file in place.

In each new terminal, run `conda deactivate` if the prompt shows `(base)`.

Download `ur_vr_bridge.zip` into `~/Downloads`, then run:

```bash
sudo apt update
sudo apt install unzip build-essential cmake python3-colcon-common-extensions \
  ros-humble-rclcpp ros-humble-geometry-msgs ros-humble-sensor-msgs \
  ros-humble-std-msgs ros-humble-tf2-ros ros-humble-rviz2

mkdir -p ~/Documents/Teleop/ur_teleop_ws/src
unzip ~/Downloads/ur_vr_bridge.zip -d ~/Documents/Teleop/ur_teleop_ws/src
```

The resulting package folder is `~/Documents/Teleop/ur_teleop_ws/src/ur_vr_bridge`.

## 2. Build

```bash
source /opt/ros/humble/setup.bash
cd ~/Documents/Teleop/ur_teleop_ws

colcon build --packages-select ur_vr_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DLIBSURVIVE_ROOT="$HOME/Documents/Teleop/ur_teleop/tools/libsurvive"
```

Expected: `Summary: 1 package finished`. Stop and send the first error if the build
fails. Do not source an unsuccessful build.

The package links to your existing `bin/libsurvive.so`; it does not download a
second tracking library or install a different Python binding. CMake reads the
existing libsurvive cache to match its floating point precision. Keep that checkout
and build directory in place. If you change libsurvive's precision, rebuild this
package with `--cmake-clean-cache` too.

## 3. Start the bridge (Terminal 1)

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run ur_vr_bridge controller_bridge
```

The updated startup banner must say `v0.2.0`. The default libsurvive verbosity is 10;
override with `--ros-args -p libsurvive_verbosity:=10` if desired. While stale, the
bridge reports a diagnostic every two seconds with selected-device pose samples,
accepted samples, input events, last source timestamp and quaternion norm.
Counters are cumulative since startup, not rates. Initial all-zero poses can be
reported before tracking is solved; these remain rejected.

If stale persists, send the diagnostic line and surrounding libsurvive output.
`pose_samples=0` means no pose event for the selected device reached validation;
`pose_samples>0 accepted=0` means samples arrived but none passed validation.
An accepted count that stops increasing indicates a stalled or invalid stream.
Do not force freshness true or increase timeouts to bypass a missing pose.

The bridge selects `KN0` by default. It prints the device serial and libsurvive's
reported subtype. The receiver code name does not identify the physical controller
model. It selects only one device and never intentionally switches to another
object while running. Restart the bridge after reconnecting a device or resetting
tracking calibration.

Once you know the serial, you can select by serial instead:

```bash
ros2 run ur_vr_bridge controller_bridge --ros-args -p device_serial:=YOUR_SERIAL
```

Serial selection takes precedence over the default `device_name` parameter.

## 4. Check ROS topics (Terminal 2)

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 topic echo /vr/controller/device --qos-durability transient_local --once
ros2 topic echo /vr/controller/pose --once
ros2 topic echo /vr/controller/joy --once
ros2 topic echo /vr/controller/pose_fresh --once
```

Expect a device name/serial/subtype, a finite pose with `frame_id: lighthouse`,
and `data: true` for freshness while pose samples are arriving.

To inspect input changes:

```bash
ros2 topic echo /vr/controller/joy
```

Press and release each intended control separately, then use Ctrl+C to stop echo.
Test both press AND release before assigning a hold-to-move control. Releasing and
pressing after startup ensures event-based input state is initialized.

## 5. Display the controller in RViz (Terminal 2)

```bash
rviz2 -d "$(ros2 pkg prefix ur_vr_bridge)/share/ur_vr_bridge/rviz/controller.rviz"
```

This displays the controller pose and TF axes, not the robot model. Fixed Frame is
`lighthouse`; the child frame is `vr_controller`. If the controller is out of view,
adjust the RViz camera or select `vr_controller` as the Orbit view's Target Frame.
The grid spacing is 0.1 m. A Pose display can retain its last value after tracking
stops: check the freshness topic and message stamps rather than treating a visible
marker as proof of live data.

Perform these checks:

1. Rest the controller on a table: the pose should remain near one position.
2. Translate it about 10 cm without rotating: the displayed translation should
   have a similar magnitude. Note which axes increase/decrease for up, forward,
   and right in your work area.
3. Rotate it in place: the orientation axes should rotate coherently.
4. Power the controller off: pose updates should stop and `pose_fresh` should turn
   false after the timeout; Joy is then published as all zeros.
5. Turn it on and restart the bridge; repeat the input press/release check.

Keep domain 42 in every terminal for this visualization/simulation stage. The
robot connection used the default domain earlier. When returning to the real
driver later, deliberately choose the intended domain in every participating
terminal; do not assume an exported value disappears when a process ends.

## Topic contract

| Topic | Type | Meaning |
|---|---|---|
| `/vr/controller/pose` | `geometry_msgs/PoseStamped` | Position in metres, unit quaternion, native tracking frame |
| `/vr/controller/joy` | `sensor_msgs/Joy` | Input snapshot; raw libsurvive indices below |
| `/vr/controller/device` | `std_msgs/String` | Latched name, serial and library-reported subtype |
| `/vr/controller/pose_fresh` | `std_msgs/Bool` | Accepted pose arrived within timeout; NOT optical-tracking confidence |
| `/vr/controller/source_time` | `std_msgs/Float64` | Original libsurvive pose timestamp in seconds, as reported by the API |
| `/tf` | `tf2_msgs/TFMessage` | `lighthouse` → `vr_controller` on new accepted poses |

Publishing is capped at 100 Hz by the bridge's timer. If several pose events arrive
between ticks, only the newest accepted pose is published. This does not imply that
optical measurements occur at 100 Hz. Pose and TF are never periodically restamped
when no new pose arrives. Nonfinite positions, invalid quaternions, and duplicate or
non-increasing source times are rejected. Near-unit quaternions are normalized.

ROS header stamps are local ROS receipt times, not synchronized optical acquisition
times. The original libsurvive time is retained separately; this version is not yet
a synchronized camera/robot training-data recorder.

### Input indices

The arrays preserve libsurvive enum indices. Unsupported controls can stay zero;
the mapping must be confirmed on the actual device.

| Joy field | Meaning |
|---|---|
| `buttons[0]` | Trigger click (or face button for some tracker types) |
| `buttons[1]` | Trackpad click |
| `buttons[2]` | Thumbstick click |
| `buttons[3]` | System |
| `buttons[4]` | A |
| `buttons[5]` | B |
| `buttons[6]` | Menu |
| `buttons[7]` | Grip click if supported |
| `axes[1]` | Trigger |
| `axes[2:4]` | Trackpad X, Y |
| `axes[4:8]` | Finger proximity values |
| `axes[8]` | Grip force if supported |
| `axes[9]` | Trackpad force if supported |
| `axes[10:12]` | Joystick X, Y |

Axis indices 0–11 and button indices 0–15 are published. Touch events are not
treated as button presses. Grip force is a controller input, not robot contact
force. After a stale interval the input cache is cleared; operate each input again
to establish its current state.

## Limits and next step

There is intentionally no transform between `lighthouse` and `base_link`. We have
not measured their relationship. Do not add an arbitrary identity transform to
make the controller and robot appear aligned.

Libsurvive may continue predicting pose using IMU data when optical visibility is
lost. Therefore `pose_fresh: true` means a stream is updating, not that tracking is
accurate or optically locked. This is an observation tool, not a safety-rated
tracking monitor, motion gate or emergency stop.

Next: choose a verified digital button for clutching, measure the coordinate-axis
mapping, and implement relative controller motion against the simulated robot.
That mapper must capture a new reference on each press, stop on release/staleness,
reject large jumps, limit speed/workspace, and require release/re-press after a
tracking fault. No physical-arm motion is implemented in this package.

## Validation and troubleshooting

The included standalone PoseGuard checks passed in the preparation environment:
quaternion order, normalization, initial invalid state, nonfinite samples,
duplicate/out-of-order source time, and freshness timeout/recovery. The libsurvive
API declarations and data layouts were checked against actual upstream headers.
A full ROS build and hardware run require the user's Ubuntu/ROS machine; they have
not been executed in the preparation environment.

Optional local test after building:

```bash
cd ~/Documents/Teleop/ur_teleop_ws
colcon test --packages-select ur_vr_bridge
colcon test-result --verbose
```

If libsurvive cannot open the receiver, close other tracking applications first.
If the selected device is missing, check the startup output for its code name and
use `--ros-args -p device_name:=THE_NAME`, or select the printed serial.
Do not change robot or firewall settings to fix a USB tracking issue.

## Sources

- [libsurvive README](https://github.com/cntools/libsurvive)
- [Simple API](https://github.com/cntools/libsurvive/blob/df22f2d/include/libsurvive/survive_api.h)
- [Input enum definitions](https://github.com/cntools/libsurvive/blob/df22f2d/include/libsurvive/survive_types.h)
- [Quaternion convention](https://github.com/cntools/libsurvive/blob/df22f2d/redist/linmath.h)

These links document the upstream API; `ur_vr_bridge` is a custom integration package.
