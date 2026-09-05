# AZD-KD ABZO / limit read-only interface

## Conclusion

The recommended path does **not** connect AZD-KD RS-485 directly to Arduino UNO pins.
The UNO's UART is 5 V TTL, while CN6/CN7 is a two-wire EIA-485 interface. Direct
connection is electrically incompatible.

Keep the existing motion path unchanged:

```text
PC --USB--> Arduino UNO --D8/D9--> MOSFET level interface --24 V FW/RV--> AZD-KD CN4
```

Add an independent, read-only monitoring path:

```text
PC --USB--> isolated two-wire USB-RS485 adapter
             D+/TR+  --------------------------> AZD-KD CN6 or CN7 pin 3 (TR+)
             D-/TR-  --------------------------> AZD-KD CN6 or CN7 pin 6 (TR-)
             isolated signal GND --------------> AZD-KD CN6 or CN7 pin 2 (GND)
```

CN6 and CN7 are parallel RS-485 connectors. Use either one. Do not connect this lift
driver to the BLVD drive-motor bus: use a separate adapter and stable
`/dev/serial/by-id/...` path.

## Required hardware

1. Isolated, Linux-compatible, two-wire half-duplex USB-RS485 adapter supporting
   115,200 bit/s, 8 data bits, even parity, 1 stop bit.
2. RJ45 breakout or a verified straight CAT5e cable. Identify contacts by RJ45 pin
   number with continuity measurement; do not rely only on wire color.
3. 120 ohm termination at the master end if the adapter does not include switchable
   termination.

Some adapters label the differential pair `A/B` with opposite conventions. Connect by
the adapter manual's explicit `+/-` or `D+/D-` definition:

| AZD-KD CN6/CN7 | Function | USB-RS485 side |
|---|---|---|
| Pin 2 | Signal GND | Isolated signal GND |
| Pin 3 | TR+ | D+ / `+` |
| Pin 6 | TR- | D- / `-` |
| Pins 1, 4, 5, 7, 8 | N.C. | Do not connect |

The driver manual states that CN6/CN7 is not isolated. An isolated USB-RS485 adapter is
therefore strongly preferred. Before wiring or changing switches, turn off both driver
power and robot power.

## AZD-KD switch settings for the first point-to-point test

All switch changes are made with power off and become effective after power cycling.

| Control | Initial test value | Meaning |
|---|---:|---|
| ID | `1` | Modbus slave address 1; address 0 is reserved for broadcast |
| SW1 No.1 | OFF | ID range 0..15; together with ID=1 gives address 1 |
| SW1 No.2 | ON | Modbus RTU protocol |
| BAUD | `4` | 115,200 bit/s |
| SW1 No.3 and No.4 | both ON | 120 ohm termination at this end |

Never turn on only one of SW1 No.3/No.4. The official manual warns that this can cause
communication errors.

## MEXE02 values to confirm, not guess

In `Communication / I/F function`, record or export:

- communication parity: default `1 = even`;
- communication stop bits: default `0 = one bit`;
- transmission wait time: default `30 = 3.0 ms`;
- silent interval: default `0 = automatic`.

Also record/export:

- exact actuator information copied from the ABZO sensor;
- resolution/electronic gear and position unit;
- preset/home coordinate;
- forward/reverse software-limit values and whether enabled;
- direct-I/O assignments for FW-LS, RV-LS, and HOMES;
- sensor logic inversion settings.

The official product page confirms a 200 mm stroke, but that does not prove the driver's
current steps-to-mm setting. Until the actuator lead, MEXE02 values, and two physical
reference points are checked, ROS publishes the ABZO value as raw steps and does not
label it as millimetres.

## What is read from the driver

The implementation uses only Modbus function `03` and exposes no write-register method.

| Evidence | Official register(s) | Interpretation |
|---|---|---|
| Present alarm | `0x0080/0x0081` | Current 32-bit alarm code |
| Command position | `0x00C6/0x00C7` | Signed 32-bit steps |
| ABZO feedback position | `0x00CC/0x00CD` | Signed 32-bit steps |
| Direct I/O | `0x00D4/0x00D5` | Physical direct-I/O state |
| Driver temperature | `0x00F8/0x00F9` | 0.1 degrees C |
| Motor temperature | `0x00FA/0x00FB` | 0.1 degrees C |
| Feedback 32-bit counter | `0x0120/0x0121` | Wrap-independent position counter |
| DC supply voltage | `0x0148/0x0149` | 0.1 V |
| Internal input state 1 | `0x0170` | FW-LS bit 12, RV-LS bit 13, HOMES bit 14 |
| Internal output state 1 | `0x0178` | HOME-END bit 0, FW-SLS bit 9, RV-SLS bit 10 |
| Internal output state 2 | `0x0179` | ALM-A/B, SYS-RDY, READY, MOVE, INFO |

`FW-LS/RV-LS/HOMES = false` is not accepted as physical evidence until the actual sensor
wiring and MEXE02 assignments are confirmed. Likewise, FW-SLS/RV-SLS are driver software
limits, not independent physical limit switches.

## Software status

Implemented but disabled by default until hardware/configuration is confirmed:

- executable: `lift_abzo_monitor_node`;
- raw topic: `/lift/abzo_position_raw` (`std_msgs/Int32`);
- structured topic: `/lift/abzo_state` (`std_msgs/String`, JSON);
- default poll rate: 5 Hz;
- default position scale: zero, meaning raw/un-calibrated rather than a guessed mm value;
- existing Arduino motion path is not modified.

After the adapter is connected, first discover its stable path:

```bash
ls -l /dev/serial/by-id/
```

Build without starting the robot:

```bash
cd /home/matsunaga-h/robot_ws
set +u
source /opt/ros/humble/setup.bash
colcon build --packages-select serial_transciever robot_bringup --symlink-install
source install/setup.bash
```

The first test must be stationary and read-only. Replace the example port with the actual
`by-id` path:

```bash
ros2 run serial_transciever lift_abzo_monitor_node --ros-args \
  -p serial_port:=/dev/serial/by-id/REPLACE_WITH_ACTUAL_ADAPTER \
  -p slave_id:=1 \
  -p baud_rate:=115200 \
  -p parity:=E \
  -p stop_bits:=1 \
  -p position_mm_per_step:=0.0
```

Then inspect without sending lift commands:

```bash
ros2 topic echo /lift/abzo_state --once
ros2 topic hz /lift/abzo_position_raw
```

Do not enable calibrated millimetres or the physical-limit confirmation flags until the
MEXE02 export and endpoint observations have been reviewed.

## Official references

- Local official driver manual: `hardware/lift_mechanism/HM-60313J.pdf`, sections 7-3,
  7-5, 8-2, and 9-1.
- Oriental Motor AZD-KD product specification:
  <https://www.orientalmotor.co.jp/ja/products/detail?hinmei=AZD-KD>
- Oriental Motor AZ Series function manual, HM-60262-11E:
  <https://www.orientalmotor.com/products/pdfs/opmanuals/HM-60262-11E.pdf>
- Oriental Motor Modbus setup guide, HM-60252J:
  <https://www.orientalmotor.co.jp/system/files/product_detail/manual/HM-60252J.pdf>
