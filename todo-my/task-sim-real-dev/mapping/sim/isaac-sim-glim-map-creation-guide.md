# Isaac Sim GLIM Map Creation Guide

> Created: 2026-06-04
> Scope: Isaac Sim PC, ROS 2 Humble, `ROS_DOMAIN_ID=20`
> Reference: `todo-my/task-navigation/guidence-0425.md`

This is the sim-side version of the real robot GLIM mapping flow. Use the same ROS topic names and the same GLIM config as the real robot whenever possible.

Current default flow:

```text
D455 camera-only recording + Isaac true IMU + MID360-like RTX LiDAR + GLIM GPU
```

D455 is recorded to the bag as a camera-only side channel. It is not used by
GLIM, and it must not participate in robot PhysX.

Important policy:

```text
Use real robot config:
  /home/ruan-x/robot_ws/glim_config

Do not create a separate sim GLIM config unless there is a proven sim-only reason.
Save sim outputs with sim-specific names:
  sim_bag/*
  maps/sim_*
  log/sim_glim_mapping_*
```

The quickstart opens one Terminator window with every process in a separate
split pane. Each run creates one debug session directory under:

```text
/home/ruan-x/robot_ws/log/sim_glim_mapping_<bag>_<timestamp>/
```

It contains generated pane startup files, ROS native logs under `ros/`, and
`run-info.txt` with the launch parameters. Pane output is kept on the real
interactive TTY instead of being redirected through `tee`, because GLIM must
receive `Ctrl-C` exactly like a manually launched foreground process.

The quickstart archives any old `/tmp/dump` before GLIM starts. Stop GLIM with
`Ctrl-C` in its pane. Every pane is an interactive foreground terminal and
remains open after its command stops. GLIM runs directly on the pane TTY,
matching a manual terminal launch, so `Ctrl-C` reaches GLIM directly. Wait for
`saved` and `MAP SAVE OK` before closing the Terminator window. After GLIM
exits, the script validates `graph.txt` and the trajectory, then copies a valid
dump into `maps/` automatically.

The Isaac true IMU publish graph uses `OnPhysicsStep -> ROS2PublishImu`.
`OnPlaybackTick` must not be used here: recorded tests showed intermittent IMU
publication gaps up to 7.5 seconds while `/clock` and LiDAR remained stable.

GPU exception:

```text
Default mapping uses the real robot CPU config:
  /home/ruan-x/robot_ws/glim_config

If the Isaac Sim PC has a usable NVIDIA GPU and CUDA GLIM package, use:
  /home/ruan-x/robot_ws/glim_config/sim

That sim config only changes GLIM CPU/GPU backend selection and reuses the real
robot ROS/topic/sensor config files.
```

---

## 0. Current Target

Create a 3D GLIM map in Isaac Sim from:

```text
src/robot_sim/usd/scenes/full_warehouse_r.usd
```

Expected data path:

```text
Isaac Sim /point_cloud
  -> robot_sim pointcloud2_to_livox_custom_msg_node
  -> /livox/lidar               Livox CustomMsg, same topic as real robot

Isaac true IMU graph
  -> /livox/imu                 same frame as real robot LiDAR IMU path

GLIM
  /livox/lidar + /livox/imu
  -> /glim_ros/aligned_points
  -> /tmp/dump when GLIM is stopped

D455 camera-only
  -> color/depth/CameraInfo
  -> rosbag only
```

Before Play, remove non-robot articulations from the warehouse scene. GLIM
mapping expects only our robot articulation; extra forklifts/vehicles can make
PhysX tensor APIs return `(2, 6)` velocity tensors and crash CUDA rendering.

---

## 1. Install / Check GLIM First

Current workspace check:

```bash
cd /home/ruan-x/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep -E '^glim$|^glim_ros$'
ros2 pkg executables glim_ros
```

If `glim_ros` is missing, install/overlay GLIM before starting map creation.

CPU install:

```bash
sudo apt install -y curl gpg
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
sudo apt update
sudo apt install -y ros-humble-glim-ros
sudo ldconfig
```

GPU install on this Isaac Sim PC:

```bash
nvidia-smi

# CPU and CUDA GLIM packages install overlapping files, so replace CPU with CUDA.
sudo apt remove -y ros-humble-glim-ros ros-humble-glim libgtsam-points-dev
sudo apt install -y ros-humble-glim-ros-cuda12.6
sudo ldconfig

source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
ros2 pkg executables glim_ros
```

If the CUDA install previously failed with `trying to overwrite`, run:

```bash
sudo dpkg --configure -a
sudo apt remove -y ros-humble-glim-ros ros-humble-glim libgtsam-points-dev
sudo apt install -y ros-humble-glim-ros-cuda12.6
sudo ldconfig
```

If `apt` still reports broken dependencies after the CPU packages are removed:

```bash
sudo apt --fix-broken install
```

Current known-good GPU host example:

```text
NVIDIA GeForce RTX 4090
Driver Version: 580.159.03
CUDA Version shown by nvidia-smi: 13.0
Use GLIM CUDA 12.6 package first unless a specific CUDA 13.1 runtime issue appears.
```


## 2. Confirm GLIM Config Matches Real Robot Policy

Use:

```text
/home/ruan-x/robot_ws/glim_config
```

For GPU mapping only, use the sim config root:

```text
/home/ruan-x/robot_ws/glim_config/sim
```

This file references the real robot config files with `../` and only switches:

```json
"config_ros": "config_ros.json",
"config_odometry": "../config_odometry_gpu.json",
"config_sub_mapping": "../config_sub_mapping_gpu.json",
"config_global_mapping": "../config_global_mapping_gpu.json"
```

The sim `config_ros.json` keeps the same frames and IMU topic as real, but
changes GLIM points input:

```json
"points_topic": "/livox/lidar_pc2_accumulated"
```

Reason: GLIM subscribes to `sensor_msgs/msg/PointCloud2`. In this sim pipeline,
`/livox/lidar` is `livox_ros_driver2/msg/CustomMsg` for FAST-LIO compatibility,
so GLIM must use the adapter's PointCloud2 output.

The important file is:

```text
glim_config/config_ros.json
```

Required policy:

```json
"imu_frame_id": "livox_frame",
"lidar_frame_id": "livox_frame",
"base_frame_id": "glim_base",
"odom_frame_id": "glim_odom",
"map_frame_id": "glim_map",
"publish_imu2lidar": false,
"imu_topic": "/livox/imu",
"points_topic": "/livox/lidar_pc2_accumulated"
```

Reason: GLIM must not publish into the main robot TF chain. Keep this separation:

```text
Main robot TF:
  map -> odom -> base_footprint -> base_link -> livox_frame

GLIM mapping TF:
  glim_map -> glim_odom -> glim_base
```

Bad signs:

```bash
ros2 run tf2_ros tf2_echo odom livox_frame
ros2 run tf2_ros tf2_echo glim_odom livox_frame
```

`livox_frame` should not become a direct child of `odom` or `glim_odom`.

---

## 3. Start Isaac Sim Scene

Open this scene in Isaac Sim:

```text
/home/ruan-x/robot_ws/src/robot_sim/usd/scenes/full_warehouse_r.usd
```

## Isaac sensor 启动

打开 Isaac:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/home/ruan-x/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib"

cd /home/ruan-x/isaac-sim
./isaac-sim.sh
```

Before pressing Play, run the default cleanup and sensor preparation. This is
the current default flow and includes D455 camera-only preparation:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/configure_mapping_physx_cpu.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/prepare_d455_camera_only.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_livox_imu_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/inspect_rtx_lidar_pipeline.py").read())
```

Press Play.

Expected preparation output includes:

```text
Disabled D455 non-camera physics sensor: .../RSD455/Imu_Sensor
OK: D455 cameras remain active without participating in robot physics.
Found 1 articulation prim(s):
ACTIVE robot /World/robot_sim/base_footprint
```

After Play, create MID360-like LiDAR and D455 ROS publishers exactly once:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/inspect_rtx_lidar_pipeline.py").read())
```

Run MID360 cleanup only before Play. Never detach/delete an active RTX
Replicator or RenderGraph while Play is running; this can leave an illegal cycle
and trigger `CUDA error 700`. Run MID360 setup exactly once after Play. If it
was already run, restart Isaac Sim instead of rerunning setup.

The LiDAR setup uses Isaac's `ROS2RtxLidarHelper` ActionGraph and publishes only
complete rotations (`fullScan=True`). Do not replace it with a directly attached
`RtxLidarROS2PublishPointCloud` writer: repeated tests created unhealthy
`DispatchSync` / `WriterSyncGate` state before CUDA error 700. With full scans,
`/point_cloud` should normally follow the LiDAR scan rate instead of the
rendering frame rate.

Do not dock, undock, or rearrange Isaac Sim UI windows while the RTX LiDAR is
running. NVIDIA documents that docking windows during RTX LiDAR simulation is
likely to crash Isaac Sim. Stop playback before changing the UI layout.

The mapping USD already disables PhysX GPU dynamics, but the CPU script is kept
for current Isaac sessions opened before the USD change. RTX LiDAR still uses
GPU rendering.

Use Isaac true IMU in this mainline. The current implementation uses Python
`ImuSensorInterface` to read the Isaac physics IMU and a minimal
`ROS2PublishImu` graph to publish `/livox/imu`. The IMU message timestamp must
reuse the existing `/clock` source
`/World/ROS2_ActionGraph/read_sim_time.outputs:simulationTime`. The Python
callback copies that value into `ROS2PublishImu.inputs:timeStamp`; it must not use
the sensor reading local time or a newly created time node with a different epoch.

The key safety rule is to remove non-robot articulations, remove old Isaac
IMU/MID360 runtime prims, and create the true IMU before Play.

### D455 recording-only side channel

D455 is part of the default recording flow, but it is not used by GLIM. It is
kept outside robot physics and used only as a camera renderer.


The default before-Play block above already includes:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/prepare_d455_camera_only.py").read())
```

This keeps Camera prims active, first disables the D455 asset's internal
`Imu_Sensor`, then removes RigidBody/Mass APIs and disables collisions and
joints. The order matters: removing the parent `RSD455` rigid body before
disabling its IMU can leave a stale PhysX tensor reader and trigger `(2,6)`.
Merely setting
`rigidBodyEnabled=false` is insufficient: PhysX hierarchy validation still sees
the nested RigidBodyAPI and prints the RSD455 xformstack error.

The internal D455 `Imu_Sensor` must also be inactive. If it remains active after
the RSD455 RigidBodyAPI is removed, Play reports:

```text
Failed to find rigid body at .../d455_sensor/RSD455
getVelocities ... shape (2, 6)
```

Do not continue that session; restart Isaac and rerun the updated camera-only
preparation before Play.

After Play, run MID360 setup first, then run D455 setup exactly once:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())
```

The current D455 profile is intentionally conservative:

```text
640x360 @ 10 Hz
```

Increasing resolution or frame rate adds RTX load and can lower LiDAR output
frequency or increase CUDA 700 risk. The latest default run recorded D455 at
about `6.3 Hz` and kept GLIM stable.

Start quickstart with camera recording enabled:

```bash
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_factory_mid360_trueimu_d455_001 \
  --gpu \
  --direct-cmd-vel \
  --record-d455 \
  --cleanup
```

It waits for and records:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
```

If `(2,6)`, nested RigidBody warnings, RSD455 missing rigid body, CUDA 700, or
a large LiDAR rate drop returns, stop the run and restart Isaac. Do not continue
mapping in a polluted session.

### RTX SDGPipeline warning and LiDAR rate

The current `ROS2RtxLidarHelper + fullScan=True` path can still print a few
Isaac-internal `DispatchSync` or `Illegal cycle connection` warnings. There is
no confirmed script-level way to remove all of them. Treat them as warning-only
when all of the following remain true:

- no CUDA error 700 or rendering failure follows;
- `/point_cloud` continues publishing;
- `/point_cloud` Publisher count is exactly 1;
- header timestamps continue advancing.

`fullScan=True` publishes only after a complete rotation, so `/point_cloud`
frequency follows completed scan rate rather than render FPS. The configured
scan rate is an upper target; scene RTX load and simulation real-time factor can
lower the actual rate. Diagnose it by comparing:

```bash
ros2 topic hz /clock --window 30
ros2 topic hz /point_cloud --window 30
ros2 topic hz /livox/lidar_pc2_accumulated --window 30
nvidia-smi
```

The LiDAR setup uses a `1x1` render product because RTX LiDAR point data comes
from the sensor AOV and does not need an image-sized render target. LiDAR-only
A/B testing remains useful for performance diagnosis, but the default mapping
flow records D455.

Do not raise scan rate first. Reduce extra RTX load, keep one LiDAR publisher,
disable unnecessary viewport rendering, and measure again.

Expected topics:

```bash
export ROS_DOMAIN_ID=20
ros2 topic hz /point_cloud
ros2 topic hz /livox/imu
ros2 topic list | grep -E 'point_cloud|livox|clock'
```

---

## 4. Run Mapping Quickstart

Use the sim tool under `tools/sim`. It uses Isaac true IMU by default. The
current default also records D455:

```bash
# 1. 关闭 quickstart 打开的所有 terminal，或 Ctrl+C
# 2. 关闭/重开 Isaac Sim，避免 /point_cloud 三个 publisher 残留
# 3. Isaac Sim 打开 scene 后，只执行一次 sensor setup scripts
# 4. 重新启动
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_factory_mid360_trueimu_d455_XXX \
  --gpu \
  --direct-cmd-vel \
  --record-d455 \
  --cleanup
```

```
ros2 bag record \
  -o sim_bag/sim_mapping_trueimu_d455_$(date +%Y%m%d_%H%M%S) \
  /clock \
  /tf \
  /tf_static \
  /point_cloud \
  /livox/lidar \
  /livox/lidar_pc2_accumulated \
  /livox/imu \
  /glim_ros/odom \
  /glim_ros/pose \
  /glim_ros/map \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info

```

The script intentionally uses the same local sim ROS environment:

```bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20
```

Reason: `/home/ruan-x/cyclonedds.xml` binds CycloneDDS to `lo`, and on this PC
that can make ROS nodes fail with `selected interface "lo" is not
multicast-capable`.

LiDAR-only diagnostic run, only when isolating D455/RTX load:

```bash
cd /home/ruan-x/robot_ws
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_factory_mid360_trueimu_lidaronly_debug_XXX \
  --gpu \
  --direct-cmd-vel \
  --cleanup
```

The script opens terminals for:

```text
1. Isaac sensor command reminder
2. sim bringup + Joy-Con + RViz
3. GLIM mapping
4. rosbag record
5. status monitor
```

Default bag output:

```text
/home/ruan-x/robot_ws/sim_bag/sim_factory_mid360_trueimu_d455_XXX
```

Drive slowly with Joy-Con:

- first lap: slow straight lines, gentle turns
- avoid fast spin at the start
- close loop by returning near the start pose
- stop and wait a few seconds before ending GLIM

---

## 5. Live Health Checks

The status terminal already checks the important topics. Manual checks:

```bash
export ROS_DOMAIN_ID=20
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash

ros2 topic hz /point_cloud
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /glim_ros/aligned_points
ros2 run tf2_ros tf2_echo base_footprint livox_frame
```

RViz:

```text
Fixed Frame: glim_map
PointCloud2: /glim_ros/aligned_points
```

If RViz fixed frame is `map`, GLIM data may show TF warnings because `map` and `glim_map` are intentionally separate.

---

## 6. Stop And Save

When mapping is finished:

1. Stop `ros2 bag record` with Ctrl+C.
2. Stop `glim_rosnode` with Ctrl+C.
3. Check whether GLIM wrote a dump:

```bash
ls -lah /tmp/dump
```

Save it with a sim-specific name:

```bash
cd /home/ruan-x/robot_ws
mkdir -p maps
cp -a /tmp/dump maps/sim_glim_dump_$(date +%Y%m%d_%H%M%S)
```

Do not overwrite real robot maps.

---

## 7. Review / Export Map

Open the saved GLIM dump:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash

```

In the GLIM offline viewer, export point cloud:

```text
File -> Save -> Export Points
```

Recommended output naming:

```text
maps/sim_factory_mapping_points_YYYYMMDD.ply
```

After PLY export, create a Nav2 2D map if needed:

```bash
/usr/bin/python3 tools/ply_to_nav2_map.py \
  maps/sim_factory_mapping_points_YYYYMMDD.ply \
  --output maps/sim_factory_mapping_2d_YYYYMMDD \
  --resolution 0.05 \
  --z-min -1.0 \
  --z-max 0.3
```

Tune `--z-min` / `--z-max` after checking where the floor, shelves, and walls appear in the exported PLY.

---

## 8. Troubleshooting

### 2026-06-04 first sim GLIM run diagnosis

Observed:

```text
No rosbag file was found under /home/ruan-x/robot_ws or /home/ruan-x.
Live ROS graph only showed Isaac raw topics:
  /point_cloud
  /camera/camera/*
  /clock
  /joint_states
  /sim/target_joint_states

Missing:
  /livox/lidar
  /livox/imu
  /glim_ros/aligned_points
```

Meaning:

```text
GLIM did not start mapping because it had no converted Livox topic and no IMU.
The recorded bag either was never created, or was created outside the checked
paths. Check terminal 5 in the quickstart for "Recording bag to:".
```

Fixes added after this run:

- `tools/sim/create_sim_glim_map_quickstart.sh` now waits for `/livox/lidar` and `/livox/imu` before launching GLIM or rosbag record.
- `sim_livox_imu_node` now publishes stationary gravity (`linear_acceleration.z=1.0`) instead of all-zero acceleration, matching `glim_config/config_ros.json` `acc_scale=9.80665`.
- The quickstart forces the repo CycloneDDS config instead of inheriting `/home/ruan-x/cyclonedds.xml`.

### `Missing ROS package: glim_ros`

GLIM is not installed or not sourced:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
ros2 pkg executables glim_ros
```

If still missing, complete Section 1.

### CUDA GLIM install says `trying to overwrite`

This means the CPU GLIM stack is already installed. Replace it with the CUDA
stack:

```bash
sudo dpkg --configure -a
sudo apt remove -y ros-humble-glim-ros ros-humble-glim libgtsam-points-dev
sudo apt install -y ros-humble-glim-ros-cuda12.6
sudo ldconfig
```

Then verify:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
ros2 pkg executables glim_ros
dpkg -l | grep -E 'gtsam-points|ros-humble-glim'
```

### No `/livox/lidar`

Check raw Isaac LiDAR first:

```bash
ros2 topic hz /point_cloud
```

If `/point_cloud` exists but `/livox/lidar` does not, the sim bringup adapter is not running:

```bash
ros2 launch robot_sim sim_sensor_check.launch.py start_joy:=false start_rviz:=true
```

If only the raw Isaac topics appear, restart through the quickstart after
sourcing the workspace:

```bash
cd /home/ruan-x/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
tools/sim/create_sim_glim_map_quickstart.sh --bag sim_factory_mid360_like_002 --gpu --cleanup
```

### No `/livox/imu`

For the default flow, `/livox/imu` must come from the Isaac true IMU. Restart
Isaac and rerun the before-Play block, especially:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_livox_imu_ros.py").read())
```

Do not use `sim_livox_imu_node` in the default mapping flow. It is retained
only for emergency fault isolation.

### GLIM starts but no aligned points

Check:

```bash
ros2 topic echo /livox/lidar --once
ros2 topic echo /livox/imu --once
ros2 topic echo /clock --once
ros2 topic hz /glim_ros/aligned_points
```

Also confirm GLIM is using the real config root:

```bash
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/home/ruan-x/robot_ws/glim_config
```

### `/tmp/dump` does not exist

Usually GLIM did not shut down cleanly or never accepted data. Stop GLIM with Ctrl+C after `/glim_ros/aligned_points` has been publishing, then check again:

```bash
ls -lah /tmp/dump
```

---

## 9. Completion Checklist

- [ ] `glim_ros` is installed and visible from this workspace
- [ ] Isaac scene opens and publishes `/point_cloud`
- [ ] `robot_sim` publishes `/livox/lidar`
- [ ] `/livox/imu` exists
- [ ] GLIM uses `/home/ruan-x/robot_ws/glim_config`
- [ ] GLIM publishes `/glim_ros/aligned_points`
- [ ] rosbag saved under `sim_bag/`
- [ ] GLIM dump saved under `maps/sim_*`
- [ ] exported PLY saved as `maps/sim_*`
- [ ] optional Nav2 2D map saved as `maps/sim_*`
