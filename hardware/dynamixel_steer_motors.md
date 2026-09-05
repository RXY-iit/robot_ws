# Dynamixel — Steer Motors (x3)

## Physical

| Item | Value |
|---|---|
| Brand | Robotis Dynamixel |
| Count | 3 (wheel steering, one per wheel) |
| Communication | Dynamixel Protocol (UART half-duplex) |
| Interface | USB-to-Dynamixel (U2D2 or similar) |
| Motor IDs | 11 (wheel 1), 12 (wheel 2), 13 (wheel 3) |
| Mode | Position control |

## Runtime identity confirmed on 2026-08-19

Protocol 2.0 ping returned model number `1100` from all three configured IDs:

| ID | Reported model number | Official model mapping |
|---:|---:|---|
| 11 | 1100 | XH540-W270 |
| 12 | 1100 | XH540-W270 |
| 13 | 1100 | XH540-W270 |

The mapping is from the ROBOTIS official XH540-W270 control table, not from a
local model-name assumption:

- https://emanual.robotis.com/docs/en/dxl/x/xh540-w270/
- Hardware Error Status: address 70, 1 byte
- Present Current: address 126, 2 bytes, signed, 2.69 mA/unit
- Present Input Voltage: address 144, 2 bytes, 0.1 V/unit
- Present Temperature: address 146, 1 byte, 1 °C/unit

The real driver samples these slow health registers at 1 Hz and includes them
in `/dynamixel/health_summary`. Position feedback retains its existing request
path; register-read failures and nonzero Hardware Error Status are reported
directly and are not inferred from steering motion.

## ROS2

| Item | Value |
|---|---|
| Package | `DynamixelSDK` → `dynamixel_sdk_examples` |
| Driver node | `omni_base_driver` (`steer_motor_node`) |

Topics:

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/steer_ang` | `my_messages/SteerMotor` | Subscribe | Target steer angles (rad, per wheel) |
| `/steer_odom` | `my_messages/SteerMotor` | Publish | Actual steer angles feedback |

Services used internally:
- `get_position` (dynamixel_sdk_custom_interfaces/srv/GetPosition) — reads current position of IDs 11/12/13

## Kinematics

The steer + drive combination forms a 3-wheel omnidirectional (holonomic) base.
Kinematics calculation: `omni_base_driver/include/omni_base_driver/picking_robot_matrix.hpp`

Odometry node (`robot_odom_node`):
- Subscribes: `/steer_odom` + `/drive_odom`
- Publishes: `/wheel_odom` (nav_msgs/Odometry, frame: `odom` → child: `base_link`)
- Also broadcasts TF: `odom → base_link`

## Debug Tips

- `ros2 service call /get_position ...` — manually query motor position
- If motor not found: confirm U2D2 connected, check `/dev/ttyUSB*`
- Position units: Dynamixel raw ticks → converted to radians in `convertPositionRadian()`
- Home position (straight) defined in `motor_param.hpp`
