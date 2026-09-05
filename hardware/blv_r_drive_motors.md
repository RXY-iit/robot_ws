# Oriental Motor BLV-R — Drive Motors (x3)

## Physical

### Installed nameplate evidence (2026-08-19)

Six archived real-robot photographs directly establish the installed mapping:

| Modbus ID / handwritten mark | Physical position | Driver trace code | Gear-head trace code | Evidence files |
|---:|---|---|---|---|
| 1 | Back | `RX5 F178611` | `RX9 A127705` | `motor-image/1-back.jpeg`, `motor-image/1-back.jpg` |
| 2 | Front-left | `RX5 F178612` | `RX9 A127704` | `motor-image/2-front-left.jpeg`, `motor-image/2-front-left.jpg` |
| 3 | Front-right | `RX5 F178620` | `RX9 A127706` | `motor-image/3-front-right.jpeg`, `motor-image/3-front-right.jpg` |

All three photographs show driver model `BLVD-KRD`, rated input
`24–48 VDC / 10.5 A`, IP20, 2024/10, and gear-head model `GFS5G30FR`.
The `RX5`/`RX9` strings are recorded as trace/manufacturing codes, not asserted
to be serial numbers without a manufacturer definition. Motor model
`BLMR5100K-GFV-B` remains a deployment record because its nameplate is not
visible in the supplied photographs.

| Item | Value |
|---|---|
| Driver model | Oriental Motor BLVD-KRD |
| Motor model | BLMR5100K-GFV-B (deployment record) |
| Gear-head model | GFS5G30FR (physical nameplate) |
| Driver rated input | 24–48 VDC, 10.5 A (rating, not a live measurement) |
| Protection | IP20 (self-declaration on physical nameplate) |
| Count | 3 (Right / Left / Back wheel) |
| Communication | Modbus RTU over RS-485 |
| Interface | USB-to-RS485 adapter → `/dev/ttyUSB0` |
| Baudrate | 230400 |
| Motor IDs | 1 (back), 2 (front-left), 3 (front-right) — physical labels |
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

## Low-rate electrical monitor

The runtime ID-share configuration uses the remaining Share Read slots without
adding a new transaction:

| Share Read slot | Official NET-ID | Value |
|---:|---:|---|
| 8 | 106 | Direct I/O, including qualified DIN3 red-stop input |
| 9 | 155 | Main power supply current, `0.001 A/unit` |
| 10 | 163 | Inverter voltage, `0.1 V/unit` |
| 11 | 164 | Main power supply voltage, `0.1 V/unit` |

One low-rate combined response returns these four values for IDs 1/2/3.
`/drive/health_summary` exposes raw and scaled values; the 2026-08-20 powered
test verified correct axis grouping and finite values. The 24–48 VDC / 10.5 A
nameplate is a rating, not an automatic runtime acceptance threshold.

The six position-labelled photographs physically verify the installed
BLVD-KRD/GFS5G30FR identity and ID-to-wheel mapping. The Dashboard uses
`VERIFIED` for this installed identity. This remains static physical evidence,
not a live protocol identity query; live communication is evaluated separately.

## Per-driver alarm monitoring

The serialized 25 Hz schedule replaces selected velocity reads with low-rate
unicast health reads, so it does not add Modbus transactions. Each physical
driver is sampled approximately every 1.3 seconds:

| Register | Official NET-ID | Direct evidence |
|---:|---:|---|
| 128/129 | 64 | Present alarm |
| 130/131 | 65 | Alarm history 1 |

A nonzero Present alarm is an error. Alarm history is baselined when the drive
process starts; a change within that session remains degraded after the present
alarm clears. Pre-existing history is retained as context and is not attributed
to the current session. Codes are displayed in hexadecimal without guessing
their meaning.


COMM 白色常亮/闪烁：正常
COMM 红色：通信错误
PWR/SYS 白色常亮：正常供电
PWR/SYS 白色闪烁：power removal/ETO 状态，不应当作普通 COMM 闪烁
PWR/SYS 红色闪烁：驱动报警