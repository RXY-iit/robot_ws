# Oriental Motor BLV-R — Drive Motors (x3)

## Physical

| Item | Value |
|---|---|
| Model | Oriental Motor BLV-R series |
| Count | 3 (Right / Left / Back wheel) |
| Communication | Modbus RTU over RS-485 |
| Interface | USB-to-RS485 adapter → `/dev/ttyUSB0` |
| Baudrate | 230400 |
| Motor IDs | 1 (right), 2 (left), 3 (back) |
| Global ID | 10 (ID-share broadcast) |
| Mode | Continuous velocity control (speed control) |

## ROS2

| Item | Value |
|---|---|
| Package | `om_modbus_master_V201` |
| Main node | `drive_motor.py` |
| Location | `om_modbus_master/sample/BLV_R/drive_motor.py` |

Topics:

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/drive_vel` | `my_messages/DriveMotor` | Subscribe | Target velocity for each wheel |
| `/drive_odom` | `my_messages/DriveMotor` | Publish | Measured velocity feedback |
| `/odom` | `nav_msgs/Odometry` | Publish | Wheel odometry (frame: base_link) |
| `/om_response0` | `om_msgs/Response` | Publish | Raw Modbus response |
| `/om_state0` | `om_msgs/State` | Publish | Modbus driver state |
| `/om_query0` | `om_msgs/Query` | Subscribe | Raw Modbus query (internal) |

## Launch

```bash
# From pickup_ws or robot_ws after build:
ros2 launch om_modbus_master_V201 <launch_file> \
  com:=/dev/ttyUSB0 topicID:=1 baudrate:=230400 \
  updateRate:=1000 secondGen:="1,2,3" globalID:=10 axisNum:=3
```

## Debug Tips

- `ls /dev/ttyUSB*` — confirm USB-RS485 adapter visible
- `sudo chmod 666 /dev/ttyUSB0` — fix permission if needed (or add user to `dialout` group)
- `/om_state0` topic: state=0 means ready, state=1 means busy (mid-transaction)
- If motors not responding: check wiring polarity (A/B), confirm ID-share mode enabled on driver
- Velocity = 0 after timeout: `drive_motor.py` has a watchdog — send cmd continuously
