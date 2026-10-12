# UR10e VR simulation teleop v0.2.0

This is the next stage after `ur_vr_bridge` v0.1.1: preview a clutched target,
then drive a mock UR10e in RViz. Ubuntu 22.04 / ROS 2 Humble.

The supplied launch creates GenericSystem mock hardware, not a UR network
connection. It checks the expanded URDF's hardware plugin, requires domain 42,
uses `/vr_sim` topics and `sim_` joint/link names, and starts in PREVIEW mode.
Keep the real robot's External Control program stopped during this exercise.
Domain 42 and namespaces organize communication; they are not safety interlocks.
Do not remap these outputs to the physical arm.

## Upgrade from v0.1.0

Stop the simulation launch with Ctrl+C; leave the observation bridge running.
Install the new dependency `sudo apt install python3-scipy`, unzip the v0.2.0
archive into `~/Documents/Teleop/ur_teleop_ws/src`, and rebuild this package. Source the workspace
and restart the simulation. It starts in PREVIEW again. After checking target
axes, call `start_servo` again and enable motion as shown below.

Defaults are now translation scale 0.8 and relative rotation 1:1. A 5 cm hand
movement requests 4 cm tool movement. The 4 cm/s speed cap stays the same, so
fast hand movement can produce visible lag. From the initial tool position,
25 cm hand displacement reaches the 20 cm target-workspace boundary. Re-clutching
captures the current tool pose but does not move that fixed workspace center.

A sphere alone cannot show rotation: watch the RGB target axes (X red, Y green,
Z blue). The existing Tool axes display shows the actual simulated tool pose.
Pressing A captures both current poses; releasing A commands zero linear and
angular velocity. Re-gripping captures the current measured tool orientation,
not the previous requested orientation. Nothing snaps to the controller's
absolute orientation.

Optional launch override: `ros2 launch ur_vr_sim_teleop simulation.launch.py
translation_scale:=0.8` (on one line). Scale changes require a restart.

## 1. Install and build

Download the ZIP to `~/Downloads`. Stop any previous simulated UR driver/RViz
launch with Ctrl+C. Keep only one process accessing the Index receiver: the
observation bridge. A Ctrl+Z-suspended survive-cli still owns its USB device;
resume that job with `fg` and exit it with Ctrl+C.

In a terminal outside Conda (`conda deactivate` if the prompt shows `(base)`):

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-ur-moveit-config ros-humble-moveit-servo \
  ros-humble-controller-manager ros-humble-position-controllers \
  ros-humble-joint-state-broadcaster ros-humble-ros2controlcli \
  ros-humble-rviz2 ros-humble-tf2-ros ros-humble-xacro \
  python3-numpy python3-scipy python3-yaml
mkdir -p ~/Documents/Teleop/ur_teleop_ws/src
unzip -o ~/Downloads/ur_vr_sim_teleop_v0_2_0.zip -d ~/Documents/Teleop/ur_teleop_ws/src
cd ~/Documents/Teleop/ur_teleop_ws
colcon build --packages-select ur_vr_sim_teleop
source install/local_setup.bash
```

This adds a separate package; the existing observation bridge stays as it is.

## 2. Terminal 1: observation bridge

If the bridge is already running successfully in domain 42, leave it running.
Otherwise:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 run ur_vr_bridge controller_bridge
```

Wait for stable tracking with the controller visible to both lighthouses.
Do not run survive-cli or SteamVR alongside the bridge.

## 3. Terminal 2: mock arm and target preview

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 launch ur_vr_sim_teleop simulation.launch.py
```

RViz should show a bent UR10e, tool axes and an orange sphere with colored target axes at its tool.
Wait for `PREVIEW / READY`. Release A once after startup, then hold A and move
the controller slowly. The sphere should move; the simulated arm stays still.
Releasing A stops target updates. Each press anchors to the current tool
position, so the sphere can reset to the tool when you press again in preview.

The initial axis mapping is the identity from Lighthouse coordinates to the
simulated base. It is a preview convention, not a physical frame calibration.
Check that the sphere moves smoothly in three directions. If horizontal axes
need rotating, stop this launch and try, for example:

```bash
ros2 launch ur_vr_sim_teleop simulation.launch.py axis_yaw_deg:=90.0
```

This only rotates XY directions about Z. A tilted tracking frame needs a full
rotation calibration in a later stage. Both translation and rotation use this axis mapping.

## 4. Terminal 3: verify and enable simulated motion

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Teleop/ur_teleop_ws/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 control list_controllers -c /vr_sim/controller_manager
```

Both `joint_state_broadcaster` and `forward_position_controller` should be
`active`. If either is missing/inactive, inspect launch errors before continuing.

Start Servo and check the service response reports `success: true`:

```bash
ros2 service call /vr_sim/servo_node/start_servo std_srvs/srv/Trigger '{}'
ros2 param set /vr_sim/teleop enable_motion true
```

Release A, then hold it and make small, slow movements. The virtual tool should
follow the orange target position and its colored target axes. Try a slow 10-degree wrist rotation first.
Release A to command a stop. Reposition your hand while released; pressing
again captures new hand/tool anchors instead of pulling toward an old target.

| Setting | Value |
|---|---|
| Clutch | Index A button, bridge Joy index 4 |
| Position scale | 0.8: 10 cm hand movement requests 8 cm tool displacement |
| Cartesian command speed limit | 0.04 m/s |
| Target workspace | 20 cm radius around the initial simulated tool position |
| Input freshness limit | 0.20 s for pose, Joy, tracking flag and simulated state |
| Input jump limit | 5 cm between control-loop samples while clutched |
| Servo command timeout | 0.10 s |
| Rotation control | 1:1 relative hand rotation, captured anew on every A press |
| Angular command speed limit | 15 degrees/s |
| Orientation jump limit | 45 degrees between control-loop samples while clutched |

These are software limits for this simulation exercise. Servo filtering and
controller updates mean the arm may settle briefly after release. The target
sphere retains its last position on release, but commanded twist becomes zero.
The workspace check limits targets; it is not a hard boundary on actual TCP.

To return to preview:

```bash
ros2 param set /vr_sim/teleop enable_motion false
```

Stop Terminal 2 with Ctrl+C when finished.

## 5. Acceptance checks

- Holding A at startup cannot activate motion until you release and press again.
- A held + small hand movement produces smooth motion; A released commands zero.
- Repositioning your hand while released does not move the simulated arm.
- Turn the controller off: stale input should enter WAIT_RELEASE. Restore tracking;
  keeping A held must not resume motion until another release and press.
- Exceeding the workspace or a position/orientation jump enters WAIT_RELEASE; release A to rearm.
- Repeat small movements in all three directions without Servo halt messages.

`pose_fresh` means recent libsurvive output, not verified optical tracking.
Covering the sensors can leave IMU-predicted poses streaming; freshness alone
does not reliably detect that. This package is not ready for physical teleop.
Before connecting the real arm we still need tracking-quality assessment,
frame calibration, robot-side limits, stopping behavior verification, and a
review of the command path. Cameras and recording come after the control path
works; no learning dataset is produced by this package.

## Troubleshooting

If the orange target display is red, expand it in RViz and set its `Topic`
to `/vr_sim/target_marker`, Reliability to Reliable, and Durability to Volatile.
The initial archive used `Marker Topic` instead of Humble's `Topic` property;
this corrected archive fixes that display configuration. No motion logic changed.

```bash
ros2 topic echo /vr_sim/teleop_state
ros2 topic echo /vr/controller/joy
ros2 topic echo /vr_sim/servo_node/status
ros2 topic hz /vr_sim/joint_states
```

Use Ctrl+C to finish each continuous command. Verify button index 4 becomes 1
only while pressing A. `WAIT_RELEASE` may also mean stale inputs, missing TF,
or wrong domains. All three terminals must use domain 42.

Servo status: 0 normal, 1 approaching-singularity slowdown, 2 singularity halt,
3 collision slowdown, 4 collision halt, 5 joint-bound halt, 6 leaving-singularity
slowdown. A newly reported halt resets the clutch. Do not disable collision
or singularity checks to force motion. For this mock exercise, restarting the
simulation restores the initial joint pose.

If only the target moves in MOTION mode, check the start_servo response,
controller states and Servo status. Send the first launch error if startup fails.

## Validation and references

The ROS-independent clutch tests cover six-axis release/reanchor, held-at-start and
held-through-tracking-loss behavior, speed/workspace/jump limits, invalid input,
axis rotations, quaternion sign equivalence, angle wrapping, relative-orientation composition, angular speed/jump limits, and rejection of non-mock hardware descriptions. Python syntax
and configuration files were checked. The prior translation-only ROS launch was confirmed working on the user's Humble installation. The new rotation path and axes display still require that runtime validation.

Run core tests with `PYTHONPATH=. python3 -m pytest -q test` from this package
directory after installing `python3-pytest`.

Interfaces checked against upstream sources:

- [UR driver 2.14 MoveIt launch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_moveit_config/launch/ur_moveit.launch.py)
- [UR driver 2.14 Servo parameters](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_moveit_config/config/ur_servo.yaml)
- [UR semantic model](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/2.14.0/ur_moveit_config/srdf/ur_macro.srdf.xacro)
- [Humble controller spawner](https://github.com/ros-controls/ros2_control/blob/humble/controller_manager/controller_manager/spawner.py)
- [Humble Servo status codes](https://github.com/moveit/moveit2/blob/humble/moveit_ros/moveit_servo/include/moveit_servo/status_codes.h)

This is a kinematic mock-hardware exercise, not Gazebo or a force/contact model.

Rotation implementation uses [SciPy Rotation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html).
For developers, with A mapping tracking axes into base axes, H/H0 the current/
clutched hand rotations and T0 the clutched tool rotation, the target is
`T_target = A * H * inverse(H0) * inverse(A) * T0`. The angular error is the
shortest rotation vector of `T_target * inverse(T_actual)`, expressed in base
coordinates. ROS and SciPy quaternions use xyzw ordering in this package.
