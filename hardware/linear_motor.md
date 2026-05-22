# Linear Motor (Chokudo / 直動)

## Physical

| Item | Value |
|---|---|
| Function | Linear actuator — extends/retracts hose or mechanism vertically |
| Controller | Robotis OpenRB-150 microcontroller board |
| Interface | USB serial → `/dev/ttyACM*` (shared with camera_swing_motor on same board) |
| Communication | Custom serial protocol via `serial_transciever` package |

## ROS2

| Item | Value |
|---|---|
| Package | `serial_transciever` |
| Main node | `chokudo_cameraswing_air_serial_node.py` |
| Combined with | Camera swing motor (same serial node, same OpenRB-150) |

Topics:

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/chokudomotor/target_angle` | (custom or Float32) | Subscribe | Target position command |
| `/chokudomotor/angle` | (custom or Float32) | Publish | Current angle/position feedback |

Manual control node: `manipulator_control/motor_manual_chokudo_node.py`
Integrated control: `manipulator_control/integrated_control_node.py`

## Debug Tips

- `ls /dev/ttyACM*` — confirm OpenRB-150 visible
- `sudo chmod 666 /dev/ttyACM0` — fix permissions
- Both linear and camera swing motors share the same serial node; starting `chokudo_cameraswing_air_serial_node.py` controls both
- If no response: confirm correct `/dev/ttyACM*` port number in node params
- OpenRB-150 firmware must be running on the board
