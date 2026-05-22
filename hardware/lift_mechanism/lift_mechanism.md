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
  ↓ USB Serial (/dev/ttyACM0)
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
| Serial port | `/dev/ttyACM0` |
| Pin 8 (FW) | MOSFET gate → AZD-KD FW 入力（UP 方向）|
| Pin 9 (RV) | MOSFET gate → AZD-KD RV 入力（DOWN 方向）|

Serial commands accepted:

| Command | Action |
|---|---|
| `UP\n` | FW HIGH, RV LOW → モーター上昇 |
| `DOWN\n` | FW LOW, RV HIGH → モーター下降 |
| `STOP\n` | FW LOW, RV LOW → モーター停止（ドライバー内部減速）|

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
| `/lift/command` | `std_msgs/String` | Subscribe | `"UP"` / `"DOWN"` / `"STOP"` / `"HOME"` |
| `/lift/position` | `std_msgs/Float32` | Publish | 推定位置 \[mm\]（速度積分による推定）|
| `/lift/state` | `std_msgs/String` | Publish | 現在の動作状態 |

### Joy-Con Button Mapping

| 操作 | ボタン |
|---|---|
| UP（上昇）| L1（Enable=4）+ Y（button 3）同時押し |
| DOWN（下降）| L1（Enable=4）+ A（button 0）同時押し |
| STOP | ボタンを離す |

ボタン番号の確認: `ros2 topic echo /joy`

### Parameters (`lift_serial_node`)

| Parameter | Default | Description |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Arduino シリアルポート |
| `baud_rate` | `9600` | シリアルボーレート |
| `jog_speed_mm_s` | `60.0` | AZD-KD JOG 速度（ドライバー設定と一致させること）|
| `position_min_mm` | `0.0` | ソフトウェア下限 \[mm\] |
| `position_max_mm` | `200.0` | ソフトウェア上限 \[mm\] |
| `home_position_mm` | `100.0` | ホーミング完了後に設定する位置値 \[mm\] |
| `auto_home_on_start` | `true` | 起動 2.5 秒後に自動ホーミング実行 |

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
- 誤差が大きくなった場合は `ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'HOME'"` で再ホーミング

## Debug Tips

```bash
# シリアルポート確認
ls /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0

# 現在の推定位置を確認
ros2 topic echo /lift/position

# 現在の動作状態を確認
ros2 topic echo /lift/state

# terminal からコマンド送信
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'UP'"
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'DOWN'"
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'STOP'"
ros2 topic pub --once /lift/command std_msgs/msg/String "data: 'HOME'"

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
