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
