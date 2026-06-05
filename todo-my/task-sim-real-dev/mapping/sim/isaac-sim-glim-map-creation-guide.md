# Isaac Sim GLIM Map Creation Guide

> Created: 2026-06-04
> Scope: Isaac Sim PC, ROS 2 Humble, `ROS_DOMAIN_ID=20`
> Reference: `todo-my/task-navigation/guidence-0425.md`

This is the sim-side version of the real robot GLIM mapping flow. Use the same ROS topic names and the same GLIM config as the real robot whenever possible.

Important policy:

```text
Use real robot config:
  /home/ruan-x/robot_ws/glim_config

Do not create a separate sim GLIM config unless there is a proven sim-only reason.
Save sim outputs with sim-specific names:
  sim_bag/*
  maps/sim_*
```

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

Before pressing Play, run cleanup. Do not create the Isaac physics IMU in the
default mapping flow:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/configure_mapping_physx_cpu.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_d455_for_mapping.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_livox_imu_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
```

Press Play.

After Play, create only the MID360-like LiDAR:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())
```

The mapping USD already disables PhysX GPU dynamics, but the CPU script is kept
for current Isaac sessions opened before the USD change. RTX LiDAR still uses
GPU rendering.

Use the corrected ROS fake IMU in this mainline. Isaac true IMU currently gives
correct gravity readings, but both `IsaacReadIMU` and direct
`ImuSensorInterface.get_sensor_reading()` trigger Isaac PhysX tensor
`getVelocities (2,6)` in this scene.

The key safety rule is to remove non-robot articulations, disable D455/RSD455,
and remove any Isaac IMU runtime prim before Play.

D455 is optional for mapping debug. Add it later only after GLIM is stable:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())
```

Expected topics:

```bash
export ROS_DOMAIN_ID=20
ros2 topic hz /point_cloud
ros2 topic hz /livox/imu
ros2 topic list | grep -E 'point_cloud|livox|clock'
```

---

## 4. Run Mapping Quickstart

Use the sim tool under `tools/sim`. It starts corrected ROS fake IMU by default:

```bash
# 1. 关闭 quickstart 打开的所有 terminal，或 Ctrl+C
# 2. 关闭/重开 Isaac Sim，避免 /point_cloud 三个 publisher 残留
# 3. Isaac Sim 打开 scene 后，只执行一次 sensor setup scripts
# 4. 重新启动
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_warehouse_mid360_fakeimu_001 \
  --gpu \
  --direct-cmd-vel \
  --cleanup
```

tools/sim/create_sim_glim_map_quickstart.sh   --bag sim_factory_mid360_like_gpu_trueimu_003   --gpu   --direct-cmd-vel   --cleanup   --no-fake-imu

The script intentionally uses the same local sim ROS environment:

```bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20
```

Reason: `/home/ruan-x/cyclonedds.xml` binds CycloneDDS to `lo`, and on this PC
that can make ROS nodes fail with `selected interface "lo" is not
multicast-capable`.

GPU run:

```bash
cd /home/ruan-x/robot_ws
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_factory_mid360_like_gpu_001 \
  --gpu \
  --cleanup
```

The script opens terminals for:

```text
1. Isaac sensor command reminder
2. sim bringup + Joy-Con + RViz
3. fake Livox IMU
4. GLIM mapping
5. rosbag record
6. status monitor
```

Default bag output:

```text
/home/ruan-x/robot_ws/sim_bag/sim_factory_mid360_like_001
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

ros2 run glim_ros offline_viewer \
  --map_path /home/ruan-x/robot_ws/maps/sim_glim_dump_YYYYMMDD_HHMMSS \
  --config_path /home/ruan-x/robot_ws/glim_config
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

For current bringup, start the temporary IMU node:

```bash
ros2 run robot_sim sim_livox_imu_node --ros-args \
  -p use_sim_time:=true \
  -p topic:=/livox/imu \
  -p frame_id:=livox_frame \
  -p publish_rate:=100.0 \
  -p accel_z_g:=1.0
```

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
