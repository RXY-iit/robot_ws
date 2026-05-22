# Camera Swing Motor

## Physical

| Item | Value |
|---|---|
| Function | Controls camera pitch — tilts camera up/down during operation |
| Controller | Robotis OpenRB-150 microcontroller board |
| Interface | USB serial → `/dev/ttyACM*` (shared with linear motor on same board) |
| Communication | Custom serial protocol via `serial_transciever` package |

## ROS2

| Item | Value |
|---|---|
| Package | `serial_transciever` |
| Main node | `chokudo_cameraswing_air_serial_node.py` |
| Combined with | Linear motor (Chokudo) — same serial node, same OpenRB-150 board |

Topics:

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/cameraswingmotor/target_angle` | (custom or Float32) | Subscribe | Target angle command |
| `/cameraswingmotor/angle` | (custom or Float32) | Publish | Current angle feedback |

Automatic control (during object chasing): `object_chaser/object_chaser_node.py`
Manual + integrated control: `manipulator_control/integrated_control_node.py`

## Debug Tips

- Shares serial port with linear motor — only one `chokudo_cameraswing_air_serial_node.py` needed for both
- If camera image tilted unexpectedly: check `/cameraswingmotor/angle` feedback
- Camera angle affects point cloud registration — keep camera_swing at known angle when capturing data for calibration
