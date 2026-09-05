# Vertical Lift Mechanism (上下移動ステージ)

## Physical Configuration

| Item | Value |
|---|---|
| Function | Vertical linear actuator — raises/lowers arm support bracket |
| Actuator | EASM2XF020AZAK / EAS 電動スライダー（ボールネジ昇降機構） |
| Driver | Oriental Motor AZD-KD (パルス列入力 / RS-485付き) |
| Encoder | ABZO（絶対値エンコーダー内蔵）|
| Control mode | JOG モード（GPIO FW/RV 信号で連続動作） |
| Travel range | 0 mm（下限）〜 200 mm（上限）|
| Home position | 100 mm（起動時のホーミング完了後の基準位置）|
| JOG speed | 60 mm/s（AZD-KD パラメータ 21 で設定） |
| JOG acceleration | 0.30000 m/s²（パラメータ 23） |

## Hardware Stack

```
Ubuntu 22 PC
  ↓ USB Serial (identity-stable /dev/serial/by-id path)
Arduino UNO
  ↓ 5V GPIO  D8(FW=UP) / D9(RV=DOWN)
MOSFET モジュール（信号レベル変換 5V → 24V）
  ↓ 24V 工業入力信号
AZD-KD ドライバー
  ↓ モーター線 + エンコーダー線
EASM2XF020AZAK 電動スライダー
  ↓
ボールネジ昇降機構
```

## Arduino

| Item | Value |
|---|---|
| Board | Arduino UNO |
| Sketch | `src/serial_transciever/arduino/lift_control/lift_control.ino` |
| Baud rate | 9600 |
| Serial port | `/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351E062C1-if00` |
| Verified USB identity | Arduino UNO `2341:0043`, serial `03536383236351E062C1` |
| Pin 8 (FW) | MOSFET gate → AZD-KD FW 入力（UP 方向）|
| Pin 9 (RV) | MOSFET gate → AZD-KD RV 入力（DOWN 方向）|

Serial commands accepted:

| Command | Action |
|---|---|
| `UP\n` | FW HIGH, RV LOW → モーター上昇 |
| `DOWN\n` | FW LOW, RV HIGH → モーター下降 |
| `STOP\n` | FW LOW, RV LOW → モーター停止（ドライバー内部減速）|
| `IDENTIFY\n` | 動作せず `IDENTITY LIFT_CONTROL_V2` を返信 |

Firmware V2 replies with `READY LIFT_CONTROL_V2` after boot and `ACK UP`,
`ACK DOWN`, or `ACK STOP` after applying each command. A successful PC-side
`write()` alone is not treated as proof that the Arduino received a command.

## AZD-KD Driver Parameter Reference

| No. | 項目 | 設定値 |
|---|---|---|
| 20 | (JOG) 移動量 | 1.00 mm |
| 21 | (JOG) 運転速度 | 60.00 mm/s |
| 22 | (JOG) 加減速 | 0.30000 m/s² |
| 23 | (JOG) 起動速度 | 5.00 mm/s |
| 28 | (HOME) 原点復帰方法 | 3センサ |
| 29 | (HOME) 原点復帰開始方向 | +側 |
| 15 | 機構リミットパラメータ設定 | ABZO設定に従う |
| 16 | 機構保護パラメータ設定 | ABZO設定に従う |

## ROS2

| Item | Value |
|---|---|
| Package | `serial_transciever` |
| Serial bridge node | `lift_serial_node` |
| Joy control node | `lift_joy_node` |
| Standalone launch | `ros2 launch robot_bringup lift_control.launch.py` |
| Full bringup | `test_all.launch.py` with `lift:=true` (default) |

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/lift/command` | `std_msgs/String` | Subscribe | `"UP"` / `"DOWN"` / `"STOP"` / `"HOME"` / `"SET_MIN"` / `"SET_MAX"` |
| `/lift/position` | `std_msgs/Float32` | Publish | 推定位置 \[mm\]（速度積分による推定）|
| `/lift/state` | `std_msgs/String` | Publish | 現在の動作状態 |
| `/lift/health_summary` | `std_msgs/String` JSON | Publish | 1 Hz 串口、命令、软件积分位置、运行参数与硬件反馈缺口 |

### Joy-Con Button Mapping

| 操作 | ボタン |
|---|---|
| UP（上昇）| LT を押し続ける |
| DOWN（下降）| RT を押し続ける |
| STOP | LT/RT を離す、または両方押す |

ボタン番号の確認: `ros2 topic echo /joy`

### Parameters (`lift_serial_node`)

| Parameter | Default | Description |
|---|---|---|
| `serial_port` | Arduino固有の `/dev/serial/by-id/...` | Arduino シリアルポート。変動する `ttyACM*` は使用しない |
| `expected_usb_vendor_id` / `expected_usb_product_id` | `2341` / `0043` | 誤ったUSBシリアルを開かないための実機確認済みID |
| `baud_rate` | `9600` | シリアルボーレート |
| `jog_speed_mm_s` | `60.0` | AZD-KD JOG 速度（ドライバー設定と一致させること）|
| `position_min_mm` | `0.0` | ソフトウェア下限 \[mm\] |
| `position_max_mm` | `200.0` | ソフトウェア上限 \[mm\] |
| `home_position_mm` | `100.0` | ホーミング完了後に設定する位置値 \[mm\] |
| `auto_home_on_start` | `false` in full Phase 2 | 起動 2.5 秒後に定时软件 homing を実行するか |
| `enable_soft_limits` | `false` in full Phase 2 | 软件积分位置到达配置边界时是否停止；不是物理限位反馈 |

## Homing Sequence

1. 起動 2.5 秒後（Arduino USB リセット待ち）に自動実行
2. `current_position_mm` を最大値（200 mm）に設定（保守的スタート）
3. DOWN 信号を Arduino に送信
4. 位置推定が `position_min_mm`（0 mm）に達したら STOP
5. `current_position_mm` を `home_position_mm`（100 mm）にセット
6. 以降は通常制御モードへ移行

**注意**: ABZO エンコーダーによるドライバー内部リミットにより、物理的なオーバーランは防止される。

## Position Estimation

エンコーダーフィードバックは ROS2 側では取得しない。
位置は `jog_speed_mm_s × 経過時間` の積分で推定する。

- ホーミング後の累積誤差はホーミングでリセットされる
- 電源OFF中に手動で動かした場合、ソフトウェア位置は必ず無効になる。実機が端点にあることを操作者が確認してから `SET_MIN` または `SET_MAX` で参照を再設定する。
- `HOME` は時間積分による動作であり、ABZO/limit feedback がない現在は直接観測された homing ではない。

断电中の手動移動は ROS から観測できないため、起動時の位置は
`UNREFERENCED_SOFTWARE_ESTIMATE` とする。2026-09-03 の調査では実機が下端にある一方、
旧ノードは別デバイスの `/dev/ttyACM0` を開き、位置だけを 185.6 mm まで積算していた。
この再発を防ぐため USB identity 検証と Arduino ACK を追加した。

- `UNREFERENCED_SOFTWARE_ESTIMATE`：启动后未建立位置参考；
- `TIMED_HOME_ESTIMATE`：仅通过速度×时间完成的软件定时 homing；
- `SOFTWARE_TIME_INTEGRATION`：不是 ABZO encoder 反馈。

要把位置从 ESTIMATED 提升为直接观测，至少需要以下一种返回链：AZD-KD RS-485
读取 ABZO position，或把上限/下限/HOME complete 接入 Arduino。为了同时判断能否运动，
还建议接入 alarm 与 ready。若只采用人工顶端校准，还需要一个带操作者确认、时间戳和
目标端点的显式 `set reference` 接口；它仍只能标为人工校准后的估计值。

## Read-only ABZO interface

The first software implementation is documented in
[`azd_kd_abzo_interface.md`](azd_kd_abzo_interface.md). It adds an optional,
read-only Modbus monitor while leaving the Arduino FW/RV control path unchanged.
It is disabled by default until an independent isolated USB-RS485 adapter, the AZD-KD
switch settings, and the MEXE02 coordinate/limit configuration are confirmed.

## Debug Tips

```bash
# シリアルポート確認
ls -l /dev/serial/by-id

# 現在の推定位置を確認
ros2 topic echo /lift/position

# 現在の動作状態を確認
ros2 topic echo /lift/state

# terminal からコマンド送信
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'UP'"
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'DOWN'"
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'STOP'"
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'HOME'"

# 実機が下端/上端にあることを目視確認した後だけ使用（モーターは動かさない）
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'SET_MIN'"
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'SET_MAX'"

# Joy-Con ボタン番号確認
ros2 topic echo /joy
```

## Files

| File | Description |
|---|---|
| `src/serial_transciever/arduino/lift_control/lift_control.ino` | Arduino スケッチ |
| `src/serial_transciever/serial_transciever/lift_control/lift_serial_node.py` | ROS2 シリアルブリッジ + 位置管理 |
| `src/serial_transciever/serial_transciever/lift_control/lift_joy_node.py` | Joy-Con ボタンマッピング |
| `src/robot_bringup/launch/lift_control.launch.py` | 単体起動 launch |

## port info
Lift Arduino UNO：/dev/ttyACM2
稳定路径：/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351E062C1-if00
