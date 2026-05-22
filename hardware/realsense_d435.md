# Intel RealSense D435 — Depth Camera

## Physical

| Item | Value |
|---|---|
| Model | Intel RealSense D435 |
| Depth technology | Stereoscopic IR |
| Depth range | 0.2–10 m (recommended 0.3–3 m) |
| Color resolution | Up to 1920×1080 @ 30fps |
| Depth resolution | Up to 1280×720 @ 30fps |
| Interface | USB 3.0 (USB-C) |

## ROS2

| Item | Value |
|---|---|
| Package | `realsense2_camera` (inside `realsense-ros/`) |
| Launch | `realsense2_camera/launch/rs_launch.py` |
| Called from | `robot_bringup/launch/bringup.launch.py` |
| camera_name / namespace | `camera` / `camera` |

Key topics:

| Topic | Type | Description |
|---|---|---|
| `/camera/camera/color/image_raw` | sensor_msgs/Image | RGB color stream |
| `/camera/camera/color/camera_info` | sensor_msgs/CameraInfo | Intrinsics |
| `/camera/camera/depth/image_rect_raw` | sensor_msgs/Image | Depth (uint16, mm) |
| `/camera/camera/depth/camera_info` | sensor_msgs/CameraInfo | Depth intrinsics |
| `/camera/camera/aligned_depth_to_color/image_raw` | sensor_msgs/Image | Depth aligned to color |

## TF

```
base_link
└── camera_link                    ← fixed joint in robot.urdf.xacro
    ├── camera_color_frame
    │   └── camera_color_optical_frame    ← Z forward, X right, Y down
    ├── camera_depth_frame
    │   └── camera_depth_optical_frame
    └── camera_infra1_frame / camera_infra2_frame
```

`base_frame_id = camera_link` is set in bringup.launch.py so the driver tree attaches to the URDF.

**TODO: measure and update actual mount xyz in robot.urdf.xacro (currently xyz="0.22 0.0 0.05")**

## Debug Tips

- `rs-enumerate-devices` — confirm camera detected
- `ros2 topic hz /camera/camera/color/image_raw` — confirm stream running
- If TF tree broken: ensure `publish_tf: true` in bringup launch args
- Depth + Color alignment: use `/camera/camera/aligned_depth_to_color/image_raw` topic
- USB bandwidth issues: use USB 3.x port, avoid USB hubs
