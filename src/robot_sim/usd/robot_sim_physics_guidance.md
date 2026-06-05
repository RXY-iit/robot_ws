# robot_sim_physics.usd check guide

## What changed

`robot_sim_physics.usd` is now an Isaac Sim 5.0 overlay that references
`robot_sim_flattened.usda` and adds the pieces needed for a real ROS2 bridge
smoke test:

- moves Articulation Root API onto `/World/robot_sim/base_footprint`
- deactivates the imported `/World/robot_sim/root_joint`
- deactivates the imported base collision set that included high sensor pieces
- adds a simpler chassis collider above wheel ground contact
- lowers COM so the robot should not fall forward when Play starts
- adds a high-friction physics material for wheels and ground
- adds a local copy of the Isaac Sim D455 asset under `camera_link`
- authors `/World/ROS2_ActionGraph` for `/clock`, `/joint_states`, and
  `/sim/target_joint_states`
- leaves MID360 RTX LiDAR creation to `scripts/setup_isaac_mid360_ros.py`

The screenshot problem is mainly physics/root setup, not only missing sensors:
the imported model had a mobile robot rooted through a fixed `root_joint`, and
the physical root/collider/COM made the robot tip onto the body edge instead of
resting on three wheel colliders.

The later Isaac shutdown on Play was likely triggered by the hand-authored RTX
LiDAR/render-product ROS graph. Isaac Sim 5.0 is fragile around RTX sensor
render-product graphs authored directly in USD, so the default USD now avoids
starting that pipeline automatically.

The ActionGraph targets the articulation root:

```text
/World/robot_sim/base_footprint
```

If you reference this USD under another prim, Isaac may show the resolved path
as something like:

```text
/World/robot_sim_physics/robot_sim/base_footprint
```

That is fine. The important point is that the target must end in
`base_footprint`, not just `robot_sim`.

In the ActionGraph, prefer `targetPrim` and leave `robotPath` empty. Isaac's
Articulation Controller ignores `targetPrim` when `robotPath` is populated, and
string paths do not remap correctly if this USD is referenced under another
prim.

## Required extensions

For Isaac Sim 5.0, make sure these are enabled:

```text
isaacsim.ros2.bridge
isaacsim.core.nodes
omni.graph.action
```

Launch from a ROS-sourced shell:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
./isaac-sim.sh /home/ruan-x/robot_ws/src/robot_sim/usd/robot_sim_physics.usd
```

## Expected Stage paths

Physics:

```text
/World/robot_sim/base_footprint
/World/robot_sim/base_footprint/sim_chassis_collision
/World/robot_sim/root_joint      inactive
/World/GroundPlane
```

Sensors:

```text
/World/robot_sim/base_footprint/base_link/livox_frame/mid360_mount
/World/robot_sim/base_footprint/base_link/camera_bottom_screw_frame/camera_link/d455_sensor
```

ActionGraph:

```text
/World/ROS2_ActionGraph
```

## Expected ROS topics

When Isaac is playing:

```text
/clock
/joint_states
```

Your existing ROS node should still provide:

```text
/cmd_vel -> /sim/target_joint_states
```

## Test sequence

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch robot_sim sim_manual_bringup.launch.py start_joy:=false direct_cmd_vel:=true
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
ros2 topic hz /clock
ros2 topic echo /sim/target_joint_states --once
```

Motion command:

```bash
ros2 topic pub -r 20 /sim/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

Use `-r 20` or another continuous rate for movement tests. A single
`--once` command, or the default low-rate `ros2 topic pub`, can make
`/sim/target_joint_states` alternate between nonzero and zero because
`sim_omni_drive_command_node` stops the drive command after
`cmd_vel_timeout`.

Frequency checks:

```bash
ros2 topic hz /cmd_vel
ros2 topic hz /sim/cmd_vel
ros2 topic hz /sim/target_joint_states
ros2 topic hz /joint_states
```

Expected during manual testing:

```text
/sim/cmd_vel:             10-30 Hz, depending on your publisher
/sim/target_joint_states: about 60 Hz
/joint_states:            about 50-60 Hz
```

For isolated Isaac motion tests, use `direct_cmd_vel:=true` and publish to
`/sim/cmd_vel`. This avoids fighting `cmd_vel_safety_node`, which also publishes
to `/cmd_vel` and will publish zeros when `/cmd_vel_raw` is absent.

For full sim-real chain tests, use `direct_cmd_vel:=false` and publish upstream:

```bash
ros2 topic pub -r 20 /teleop/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

## If Isaac complains

If the D455 asset fails to load, check your default asset root. Isaac Sim 5.0
normally uses:

```text
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0
```

If the D455 camera prim name differs in your asset pack, update the graph node:

```text
create_d455_depth_render_product.inputs:cameraPrim
```

Current value:

```text
/World/robot_sim/base_footprint/base_link/camera_bottom_screw_frame/camera_link/d455_sensor/Camera_Pseudo_Depth
```

## LiDAR after Play is stable

After the robot can Play without Isaac shutting down, enable:

```text
isaacsim.sensors.rtx
```

Then run this file from Isaac Sim Script Editor:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_ros.py").read())
```

It creates:

```text
/World/robot_sim/base_footprint/base_link/livox_frame/localization_lidar
```

and publishes:

```text
/point_cloud
```

The script defaults to `LIDAR_MODE = "os1_128"`, Isaac Sim's built-in Ouster OS1
128ch 10Hz/1024res RTX LiDAR profile. This is not a MID360 model; it is the
current localization-test LiDAR because it gives stable 3D geometry.

Supported `LIDAR_MODE` values:

```text
os1_128
os1_32
hesai_xt32
sick_multiscan136
```

The ROS adapter converts the raw stream into:

```text
/livox/lidar_pc2_accumulated
/livox/lidar
```

If this script crashes Isaac, the problem is isolated to RTX LiDAR/rendering,
not the robot physics or ROS joint bridge.

## D455 camera publishers

The visible D455 model is stored locally at:

```text
src/robot_sim/usd/assets/realsense_d455/rsd455.usd
```

`robot_sim_physics.usd` references it through:

```text
./assets/realsense_d455/rsd455.usd
```

After Play is stable, run this from Isaac Sim Script Editor:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())
```

The script auto-discovers D455 Camera prims. It handles direct-open stages such
as `/World/robot_sim/...` and referenced stages such as
`/World/robot_sim_physics/robot_sim/...`.

Expected ROS2 topics, matching the real RealSense naming:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
```

Check:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /camera/camera/depth/camera_info --once
```

## Combined sensor setup and RViz check

After Play is stable, the preferred Isaac Script Editor command is now:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_sensors_ros.py").read())
```

It starts both:

```text
setup_isaac_d455_ros.py
setup_isaac_mid360_ros.py
```

The combined script defaults to `LIDAR_MODE = "os1_128"`. To use a lighter
profile:

```python
LIDAR_MODE = "os1_32"
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_sensors_ros.py").read())
```

ROS2 side:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20

ros2 launch robot_sim sim_sensor_check.launch.py
```

This starts the sim bringup, PointCloud2 to Livox CustomMsg adapter, RViz, and
`sim_sensor_check_node`, which reports camera, LiDAR, `/clock`, and
`/joint_states` health in the terminal.

RViz should display `/livox/lidar_pc2_accumulated`, not
`/point_cloud`. The raw topic is Isaac's partial RTX stream and can
look like a flat arc. The accumulated topic is published by the ROS adapter at
about 10 Hz with about 20k points per packet.

For a simple vertical geometry check, add one 2 m box in front of the robot:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_lidar_check_box.py").read())
```

Then compare the `raw_pc2` and `accum_pc2` z ranges printed by
`sim_sensor_check_node`.

Use this to separate scene-hit problems from scan-pattern problems before
tuning the Livox-like RTX LiDAR script.

If the ROS2 CLI fails before subscribing with:

```text
selected interface "lo" is not multicast-capable
rmw_create_node: failed to create domain
```

the D455 publishers are not the failing part. The terminal is probably loading
`/home/ruan-x/cyclonedds.xml`, which binds CycloneDDS to `lo` and may also
request a socket receive buffer larger than the current system limit. If the
normal three-line setup works, keep using it. If ROS2 CLI cannot create a node,
override it with the workspace config:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ruan-x/robot_ws/src/robot_sim/config/cyclonedds_isaac_local.xml

ros2 topic list
ros2 topic hz /camera/camera/color/image_raw
```

## Livox CustomMsg adapter

To get both the simulated PointCloud2 and the Livox CustomMsg output, launch:

```bash
ros2 launch robot_sim sim_manual_bringup.launch.py \
  start_joy:=false \
  direct_cmd_vel:=true \
  start_pc2_adapter:=true
```

Then run `setup_isaac_mid360_ros.py` in Isaac Script Editor:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_ros.py").read())
```

Expected topics:

```text
/point_cloud
/livox/lidar_pc2_accumulated
/livox/lidar
```

If the robot still tips over:

1. Confirm `/World/robot_sim/root_joint` is inactive.
2. Confirm `/World/robot_sim/base_footprint` has Articulation Root.
3. Turn on collider visualization.
4. Check the only base collider is `sim_chassis_collision`.
5. Check wheel colliders touch the ground before the chassis box.

If the console repeats:

```text
Failed to find articulation
Provided pattern list did not match any articulations
```

open `/World/ROS2_ActionGraph` and check:

```text
publish_joint_state.inputs:targetPrim
articulation_controller.inputs:targetPrim / robotPath
```

They should point to the resolved `base_footprint` articulation root.

If `/sim/target_joint_states` changes but `/joint_states` barely moves, check
that unused command modes are `NaN`:

```text
steer joints: position = angle, velocity = NaN
drive joints: position = NaN, velocity = wheel rad/s
```

Isaac Sim's JointState subscriber treats non-NaN entries as active commands.
Using `0.0` for an unused mode can make a wheel receive position and velocity
commands at the same time.
