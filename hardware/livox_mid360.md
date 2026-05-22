# Livox MID360 — 3D LiDAR

## Physical

| Item | Value |
|---|---|
| Model | Livox MID360 |
| FOV | 360° horizontal, -7°~52° vertical |
| Range | 0.1–40 m |
| Point rate | ~200,000 pts/s |
| IMU | Built-in 6-axis IMU |
| Interface | Ethernet (UDP) |

## Network Config

| Item | Value |
|---|---|
| Host IP | `192.168.1.50` |
| LiDAR IP | `192.168.1.147` |
| Config file | `src/livox_ros_driver2/config/MID360_config.json` |

Ports (MID360):
- cmd: host=56101, lidar=56100
- push_msg: host=56201, lidar=56200
- point_data: host=56301, lidar=56300
- imu_data: host=56401, lidar=56400

## ROS2

| Item | Value |
|---|---|
| Package | `livox_ros_driver2` |
| Node | `livox_lidar_publisher` |
| Launch | `livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` |
| PointCloud topic | `/livox/lidar` (sensor_msgs/PointCloud2) |
| IMU topic | `/livox/imu` (sensor_msgs/Imu) |
| Publish rate | 10 Hz (configurable) |
| xfer_format | 1 = Livox custom pointcloud |

## TF

```
base_link
└── livox_frame    ← fixed joint, defined in robot.urdf.xacro
```

frame_id in driver: `livox_frame` (set in launch file, matches URDF).

**TODO: measure and update actual mount xyz in robot.urdf.xacro (currently xyz="0.10 0.0 0.10")**

## Debug Tips

- `ping 192.168.1.147` — confirm LiDAR reachable
- `ip addr` — confirm host has address `192.168.1.50` on the ethernet interface
- If points visible in RViz but rotated: adjust `roll/pitch/yaw` in `MID360_config.json` extrinsic_parameter
- If no points: check firewall (`sudo ufw disable` temporarily), check MID360_config.json IPs
- IMU frame is hardcoded to `livox_frame` in `src/lddc.cpp`
