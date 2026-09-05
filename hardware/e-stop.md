现有硬件文档则用于补充 BLV-R 的 ROS/Modbus 接口信息：系统为 3 台 BLV-R、Modbus RTU over RS-485，已有 `/om_query0`、`/om_response0` 等接口。

### 1. 已确认硬件与通信信息

| 项目         | 已确认信息                           | 状态 |
| ---------- | ------------------------------- | -- |
| Driver     | Oriental Motor **BLVD-KRD**     | ✅  |
| Motor      | **BLMR5100K-GFV-B**             | ✅  |
| Gear head  | **GFS5G30FR**                   | ✅  |
| 数量         | 3 个驱动轮                          | ✅  |
| 通信         | Modbus RTU / RS-485             | ✅  |
| Baudrate   | 230400 bps                      | ✅  |
| Parity     | Even                            | ✅  |
| Stop bit   | 1 bit                           | ✅  |
| ROS RS-485 | FTDI USB-RS485 → `/dev/ttyUSB0` | ✅  |
| Motor IDs  | 1 / 2 / 3                       | ✅  |
| 红色停止按钮     | 会影响 BLVD-KRD 的 `DIN3:FREE`      | ✅  |

2026-08-19 `hardware/motor-image/` 的六张真机铭牌照片直接确认三套安装组件
使用相同型号：驱动器 `BLVD-KRD`，齿轮头 `GFS5G30FR`。手写编号与位置映射为
ID1=后轮、ID2=左前轮、ID3=右前轮。照片中的驱动器额定输入为
`24–48 VDC, 10.5 A`、IP20、制造日期 2024/10。电机型号
`BLMR5100K-GFV-B` 仍来自部署记录，不把未出现在照片中的文字描述成铭牌证据。
各位置追踪码已逐台记录于 `hardware/blv_r_drive_motors.md`，但在没有厂商定义前
只称为 trace/manufacturing code，不自行解释成序列号。

现有硬件文档也记录了 `/dev/ttyUSB0`、230400 baud、ID 1/2/3，与历史调查基本吻合。

### 2. ROS → BLVD-KRD 的命令链路

| 顺序 | 信号/组件                   | 作用                       | 已确认情况             |
| -: | ----------------------- | ------------------------ | ----------------- |
|  1 | Joystick / Navigation   | 产生机器人速度命令                | ✅                 |
|  ↓ | `/cmd_vel`              | 机器人速度指令                  | ✅ 有非零值            |
|  2 | `cmd_vel_to_motor_node` | 转换成三个轮子的速度               | ✅                 |
|  ↓ | `/drive_vel`            | 三轮目标速度                   | ✅ 有非零值            |
|  3 | `drive_motor`           | 转换为 BLV-R Modbus command | ✅                 |
|  ↓ | `/om_query0`            | Modbus query/write       | ✅ 有非零写入           |
|  4 | `om_modbusRTU_node`     | 实际执行 Modbus RTU 通信       | ✅                 |
|  ↓ | `/dev/ttyUSB0`          | USB → RS-485             | ✅                 |
|  5 | BLVD-KRD                | 接收速度 command             | ✅ 通信成立            |
|  6 | Motor                   | 是否真正转动                   | **受 FREE/励磁状态控制** |

因此这次故障已经证明：

> **“ROS 发出了速度” ≠ “电机一定会转”。**

中间还有 BLVD-KRD 自身的安全/enable 状态。

### 3. 物理红色按钮 → 电机停止的关键链路

这是这次调查最重要的结论：

| 顺序 | 状态                | 结果                |
| -: | ----------------- | ----------------- |
|  1 | **红色物理停止按钮按下并锁定** | 物理输入状态改变          |
|  ↓ | `DIN3`            | BLVD-KRD 外部数字输入   |
|  2 | `DIN3 = FREE` 有效  | FREE 功能触发         |
|  ↓ | `FREE = ON`       | Driver 进入 FREE    |
|  3 | 电机非励磁             | Servo-on 实际未成立    |
|  ↓ | `SON-MON = OFF`   | Driver 没有进入实际励磁状态 |
|  4 | ROS 即使继续发送速度      | Driver 不执行运动      |
|  ↓ | **Motor 不转**      | 最终安全效果            |

可以简化成：

**物理按钮锁定 → DIN3 active → FREE ON → SON-MON OFF → 非励磁 → 电机不转**

### 4. 按钮释放时的正常逻辑

反过来：

| 红色按钮     | DIN3 / FREE | SON-MON     | Motor   |
| -------- | ----------- | ----------- | ------- |
| 🔴 按下/锁定 | FREE 有效     | OFF         | ❌ 不允许运动 |
| ⚪ 释放     | FREE 不触发    | ON（其他条件正常时） | ✅ 可以运动  |

所以目前的理解是：

**按钮释放 → FREE OFF → SON-MON 可成立 → Motor 可运行**

**按钮按下 → FREE ON → SON-MON 不成立 → Motor 不运行**

### 5. MEXE02 实际观察到的状态

故障发生时：

| MEXE02 状态   |       值 | 含义                  |
| ----------- | ------: | ------------------- |
| 当前 Alarm    |    `00` | 没有当前 Alarm          |
| INFO        |      ON | 已观察                 |
| S-ON        |      ON | Servo-on command 存在 |
| **SON-MON** | **OFF** | 实际励磁没有成立            |
| **FREE**    |  **ON** | FREE 输入正在生效         |

### 6. 官方通信来源与 V1.2 接口

官方 BLV-R Function Manual `HP-5142-4E`：

- https://www.orientalmotor.com/products/pdfs/opmanuals/HP-5142-4E.pdf
- ID-share NET-ID `106 (0x006A)` 为 Direct I/O；
- Direct I/O 的 Modbus monitor register 中，DIN0..DIN3 对应低 4 bit，DIN3 为 bit 3；
- NET-ID `63 (0x003F)` 为 Driver output status，具体输出功能可由参数重新分配；
- NET-ID `64 (0x0040)` 为 Present alarm；
- NET-ID `124/125` 为 driver/motor temperature（0.1 °C）；
- NET-ID `155` 为 main power supply current（0.001 A）；
- NET-ID `163/164` 为 inverter/main power supply voltage（0.1 V）。

V1.2 先接入 NET-ID 106：三台驱动器约 1 Hz 读取 raw Direct I/O，并在 `/drive/health_summary` 输出 DIN3。

2026-08-19 已在同一接线、同一 drive process、机器人静止时取得双状态真机直接证据：

| 物理按钮状态 | ID 1 | ID 2 | ID 3 | 三台 raw 值 |
|---|---:|---:|---:|---:|
| 释放 | DIN3=1 | DIN3=1 | DIN3=1 | 196616 |
| 按下并锁定（首次标定 session） | DIN3=0 | DIN3=0 | DIN3=0 | 196608 |

因此当前安装接线的运行时映射已限定为：全 1 = `RELEASED/OK`，全 0 = `PRESSED/BLOCKED`，三台不一致 = `ERROR`。这证明的是三台 BLVD 看到的 DIN3 输入状态，不把软件 `/safety_status` 当作物理按钮，也不额外声称已经监测到 SON-MON。断线行为仍需单独的断线/故障注入测试；在完成前不能宣称该接口达到安全 PLC 或 safety relay 的故障诊断等级。

V1.3 第一次重启时，按钮按下状态仍为三台 DIN3=0，但完整 raw 值为 ID1/2=196608、ID3=0。NET-ID 106 中除 DIN0..DIN3 外的位没有在当前监测中定义语义，因此运行判定只比较官方定义的 DIN3 bit 3，不要求完整 raw 值相等。表中的 raw 仅是当次样本，不是状态标准。
