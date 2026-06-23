# Navigation 後の追加開発メモ

目的: sim navigation が一通り動く状態になった後、真機に近い可動部と移動特性を sim に追加し、Nav2 の挙動を安定させる。

最終更新: 2026-06-16

---

## 現状

- mapping / sim-env / calibration は基本完了。
- localization は真機と同じ FAST-LIO2 + GICP localization を使う。AMCL は使わない。
- navigation は `tools/sim/open_sim_nav2_windows.sh --cleanup` から起動する。
- 3D map: `/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply`
- 2D map: `/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.yaml`
- 直近テストでは、起動順序が正しければ大きな問題なく走行できた。前回の一度だけの localization drift は起動順序/TF/localization settling の影響が濃く、速度変更そのものが直接原因とは断定しない。

---

## 変更分類

| 分類 | 内容 | sim-only / real 関連 |
|---|---|---|
| 環境・起動 | sim navigation 最終起動スクリプト、debug recorder、cleanup、AUTO 初期化 | sim-only |
| モデル | lift base、camera pan/swing holder、D455 の movable holder への移動 | sim-only。ただし寸法・topic は真機に寄せる |
| Joy / topic | sim 用 Joy teleop、lift/camera Joy node の再利用、`/sim/arm_joint_states` | Joy node は真機と共用、Isaac bridge は sim-only |
| Navigation parameter | MPPI 速度・加速度・critic weight、local costmap | 基本は sim 用調整。真機に反映する場合は別途実機検証が必要 |
| Motion policy | AUTO 時だけ `/nav2/cmd_vel` を直進寄りに整形 | sim/real 共通化可能な考え方。ただし現状は sim navigation 用 |
| Wheel behavior | steer 角制限、steer rate、drive gating | 真機制約に合わせた sim 側再現 |

---

## 追加・変更ファイル

| ファイル | 役割 | 影響範囲 |
|---|---|---|
| `src/robot_sim/usd/robot_sim_physics.usd` | lift/camera holder/D455 movable sensor/OmniGraph を追加 | sim-only |
| `src/robot_sim/usd/robot_sim_flattened.usda` | 元モデル側の旧 D435/holder visual を整理 | sim-only |
| `src/robot_sim/robot_sim/sim_lift_command_node.py` | 真機用 topic を Isaac joint command に変換 | sim-only bridge |
| `src/robot_sim/setup.py` | `sim_lift_command_node` entry point 追加 | sim-only |
| `src/robot_sim/launch/sim_manual_bringup.launch.py` | sim 手動起動で lift/camera holder 制御を起動 | sim-only |
| `tools/sim/open_sim_nav2_windows.sh` | 最終 sim nav 起動、Joy/lift/camera/debug を統合 | sim-only |
| `src/robot_sim/config/sim_joy_teleop.yaml` | sim 用 Joy teleop | sim-only |
| `src/robot_sim/config/sim_omni_drive.yaml` | sim wheel bridge の steering 制約 | 真機制約を sim に反映 |
| `src/robot_sim/robot_sim/sim_omni_drive_command_node.py` | `/cmd_vel` から steer/drive joint command を生成 | sim-only bridge |
| `src/nav_pkg/scripts/nav_mode_switch_node.py` | AUTO/MANUAL 切替 + motion policy | navigation logic |
| `src/nav_pkg/launch/navigation.launch.py` | motion policy / safety / depth pointcloud params | navigation launch |
| `src/nav_pkg/config/nav2_params.yaml` | MPPI / local costmap 調整 | navigation params |
| `tools/nav2_debug_recorder.py` | `/motion_policy/status` を debug に追加 | debug |
| `src/robot_sim/scripts/sim/prepare_d455_camera_only.py` | D455 asset 内の physics/IMU を camera-only に整理 | sim-only |

---

## Lift motor / camera holder 追加

### 目的

真機側で予定している arm base と camera holder を sim に追加し、Joy-Con で挙動確認できるようにした。

- `lift_joint`: future arm base。上下のみ。移動範囲 0.30 m。
- `camera_pan_joint`: camera yaw。
- `camera_swing_joint`: camera pitch。
- D455 sensor は固定 camera frame から movable holder 配下へ移動。

### 現在の仕様

```
base_footprint
  ├── lift_joint
  │     └── lift_link
  ├── camera_pan_joint
  │     └── camera_pan_link
  │           └── camera_swing_joint
  │                 └── camera_swing_link
  │                       └── d455_sensor
```

`lift_joint`:
- 可動範囲: `0.0..0.3 m`
- 位置: camera より 25-55 cm 上
- 外観: simple light-gray cube

`camera_pan_joint`:
- 可動範囲: `-90..90 deg`
- 操作: left D-pad/stick 左右

`camera_swing_joint`:
- 初期角度: `42 deg` downward
- 可動範囲: `-90..132 deg`
- 操作: left D-pad/stick 上下
- navigation/AUTO 中、または robot 移動中は初期角度へ戻す。local costmap 用の camera pose を安定させるため。

### Topic / node

真機側 node は可能な限り流用する。

| Node | 入出力 | 扱い |
|---|---|---|
| `lift_joy_node` | Joy -> `/lift/command` | 真機と共用 |
| `camera_motor_joy_node` | Joy -> `/chokudomotor/target_angle`, `/cameraswingmotor/target_angle` | 真機と共用 |
| `lift_serial_node` | `/lift/command` -> serial motor | sim では起動しない |
| `sim_lift_command_node` | real-like topic -> `/sim/arm_joint_states` | sim-only |

`/sim/arm_joint_states` は wheel の `/sim/target_joint_states` と分離した。
理由: wheel command は 60 Hz で流れるため、同じ topic に arm/camera joint を混ぜると低頻度の holder command が毎 tick 上書きされやすい。

### 変換ロジック

- Lift: `UP/DOWN/STOP/HOME` を 0.06 m/s で積分し、`0.0..0.3 m` に clamp。
- Camera pan: `/chokudomotor/target_angle` degree -> `(deg - 0.0) * pi/180`。
- Camera swing: `/cameraswingmotor/target_angle` degree -> `42 deg + (deg - 67 deg)`。
- Camera lock: `robot_mode == AUTO` または `/cmd_vel` が閾値以上なら、pan=0、swing=42 deg に戻す。

### 遭遇した問題と解決

| 問題 | 原因 | 解決 |
|---|---|---|
| Joy-Con で holder が動かない | navigation script が lift/camera node を起動していなかった | `tools/sim/open_sim_nav2_windows.sh` と `sim_manual_bringup.launch.py` に起動を追加 |
| Lift と camera が wheel command に上書きされる | wheel と arm を同じ joint state topic に載せる設計だと高頻度 wheel command が勝つ | `/sim/arm_joint_states` と専用 OmniGraph subscriber/controller に分離 |
| camera が二重に見える | 旧固定 D435/D455 visual と新 movable D455 が同時に存在 | `robot_sim_physics.usd` / `robot_sim_flattened.usda` で旧 visual を整理し、D455 は movable holder 配下に一本化 |
| D455 asset 追加後に PhysX tensor error | D455 asset 内の non-camera physics/IMU/RigidBody が robot articulation と干渉 | `prepare_d455_camera_only.py` で camera 以外の physics sensor / RigidBodyAPI を無効化 |
| camera が local costmap に地面を入れやすい | camera 位置・角度変更で depth の見え方が変化 | navigation/AUTO 中は初期 42 deg に戻し、local costmap は `min_obstacle_height=0.10` に戻した |

---

## Navigation / MPPI parameter 調整

### 方針

単純に速度上限だけ上げると、3輪 steer が急に切り替わる。そこで以下の順で調整した。

1. 速度上限を上げる。
2. 横移動・角速度・加速度のサンプル幅を抑える。
3. path alignment / forward preference を強める。
4. sim wheel bridge 側で真機と同じ steering 制限を入れる。
5. 最後に AUTO motion policy で「できるだけ直進、必要なら先に向きを変える」ルールを追加する。

### 現在の主要値

`src/nav_pkg/config/nav2_params.yaml`:

| パラメータ | 現在値 | 意図 |
|---|---:|---|
| `vx_max / vx_min` | `0.24 / -0.24` | 前進速度を上げ、短距離後退も許可 |
| `vy_max / vy_min` | `0.05 / -0.05` | 横移動は残すが控えめにする |
| `wz_max / wz_min` | `0.40 / -0.40` | 過剰旋回を抑える |
| `vx_std` | `0.28` | doubled speed に合わせる |
| `vy_std` | `0.02` | 横移動サンプルを抑える |
| `wz_std` | `0.18` | 急旋回サンプルを抑える |
| `ax_max / ax_min` | `0.5 / -0.5` | 前後加速度 |
| `ay_max` | `0.12` | 横加速度を抑える |
| `az_max` | `0.45` | 角加速度を抑える |
| `PathAlignCritic.cost_weight` | `18.0` | path 追従を強める |
| `PathAngleCritic.forward_preference` | `true` | 前向きに揃えて進む傾向を強める |
| `PathAngleCritic.cost_weight` | `1.5` | forward preference が強すぎる副作用を避ける |
| local costmap `camera_marking.min_obstacle_height` | `0.10` | 地面混入を避ける。直近テストでこの値は OK |

RPP fallback:
- `desired_linear_vel: 0.20`
- `rotate_to_heading_angular_vel: 0.6`
- `min_approach_linear_velocity: 0.10`

### SteeringChangeCritic について

`src/omni_mppi_critics/` に custom MPPI critic を追加したが、controller_server segfault が出たため現状は active critics list から外している。

考え方:
- 3輪それぞれの steer 角を `atan2(vy + rx*wz, vx - ry*wz)` で推定。
- `±0.71*pi` を超える場合は真機と同じように `pi` 折り返し + drive 反転。
- 時系列の steer angle jump を cost にする。

現状:
- package/code は残す。
- `nav2_params.yaml` の active critics list には入れない。
- 再有効化する場合は controller_server の安定確認を先に行う。

---

## AUTO motion policy

### 目的

MPPI だけでは omni drive として横移動・旋回を選びやすく、見た目も真機らしくない。そこで Nav2 は維持しつつ、AUTO 時だけ `/nav2/cmd_vel` を `/cmd_vel_raw` に流す直前で軽く整形する。

MANUAL は従来通り Joy command を通す。

### 実装

`src/nav_pkg/scripts/nav_mode_switch_node.py`:

- subscribe: `/plan`, `/nav2/cmd_vel`, `/teleop/cmd_vel`, `/joy`, `/nav_mode/command`
- publish: `/cmd_vel_raw`, `/robot_mode`, `/motion_policy/status`
- TF: `map -> base_footprint`

policy status:

| Status | 意味 |
|---|---|
| `FORWARD_TRACK` | 通常追従。`linear.y` を抑え、横成分を少し yaw に寄せる |
| `ALIGN_THEN_FORWARD` | path lookahead 方向が大きく横/後方なら、先に向きを変える |
| `SHORT_REVERSE` | 近距離で後方にある goal は短距離後退 |
| `FINAL_YAW_ALIGN` | 到達直前で姿勢だけ整える |
| `FINAL_APPROACH` | 到達直前の低速前後移動 |
| `PASS_THROUGH_NO_GOAL_TF` | plan/TF 未取得。安全側で lateral だけ抑える |
| `PASS_THROUGH_DISABLED` | policy 無効化時 |
| `PASS_THROUGH_POLICY_ERROR` | policy exception 時。Nav2 command をそのまま通す |

現在の主要値:

| parameter | value |
|---|---:|
| `align_angle_rad` | `0.50` |
| `align_distance_m` | `0.25` |
| `finish_distance_m` | `0.18` |
| `finish_yaw_gate_rad` | `0.45` |
| `finish_max_vx` | `0.08` |
| `finish_max_wz` | `0.25` |
| `reverse_distance_m` | `0.55` |
| `reverse_lateral_window_m` | `0.25` |
| `policy_lookahead_m` | `1.2` |
| `lateral_scale` | `0.10` |
| `lateral_to_yaw_gain` | `1.4` |
| `min_align_wz` | `0.18` |
| `max_align_wz` | `0.55` |
| `max_reverse_vx` | `0.12` |

無効化して比較:

```bash
ros2 launch nav_pkg navigation.launch.py use_sim_time:=true joy_estop_button:=-1 enable_motion_policy:=false
```

### 遭遇した問題と解決

| 問題 | 原因 | 解決 |
|---|---|---|
| AUTO に入った後 robot が動かない | `_yaw_from_quaternion(goal_pose)` と渡して node が落ちた | `goal_pose.orientation` を渡すよう修正し、policy exception 時は pass-through |
| `enable_motion_policy:=false` が無効にならない | launch parameter の string `"false"` を bool として正しく解釈していなかった | `_param_bool()` を追加 |
| 到達時に wheel が転向途中で残る | final zone で横移動/旋回が残る | `FINAL_YAW_ALIGN` / `FINAL_APPROACH` を追加し、final では `linear.y=0` |
| policy が効いているか分かりにくい | debug に policy status がなかった | `/motion_policy/status` と `tools/nav2_debug_recorder.py` を追加 |

---

## Sim wheel bridge / 真機 steering 制約の反映

### 目的

真機の steering は無限回転ではなく、`±0.71*pi` の範囲で drive 反転を使う。sim でも同じ制約を入れないと、Nav2 の小さな `cmd_vel` 変化が過大な wheel angle jump に見える。

### 実装

`src/robot_sim/robot_sim/sim_omni_drive_command_node.py`:

- steer angle limit: `±0.71*pi`
- steer target は「現在角度に最も近い、制限内の候補」を選ぶ。
- 必要なら drive velocity sign を反転する。
- wheel speed が deadband 未満なら steer 角を保持し drive=0。
- steering が target から大きく外れている間は drive=0。
- steer rate limit を入れて瞬間的な角度 jump を避ける。

`src/robot_sim/config/sim_omni_drive.yaml`:

```yaml
steer_angle_limit_rad: 2.23053   # 0.71*pi
steer_rate_limit_rad_s: 0.9
wheel_speed_deadband_m_s: 0.055
drive_disable_steer_error_rad: 0.25
```

これは真機パラメータを直接変更したものではなく、真機の運動制約を sim 側に近似したもの。

調整目安:
- 動き出しが遅すぎる: `drive_disable_steer_error_rad: 0.25 -> 0.35`
- steer が遅すぎる: `steer_rate_limit_rad_s` を少し上げる
- 曲がり切れない: MPPI `wz_max` または policy `max_align_wz` を少し戻す

---

## 起動スクリプト改善

`tools/sim/open_sim_nav2_windows.sh`:

- sim 用 Joy config `src/robot_sim/config/sim_joy_teleop.yaml` を使う。
- FAST-LIO2 + GICP localization、Nav2、RViz、debug recorder を一括起動。
- lift/camera holder node も同時に起動。
- `--cleanup` で古い Nav2/localization/Joy/lift/camera/debug/RViz process を終了。
- AUTO 初期化時に `/emergency_stop=false`、neutral Joy、Joy A、neutral Joy を流す。

理由:
- 前回、Y などを誤って押した状態が ESTOP/古い Joy edge として残り、AUTO に入っても robot が動かないケースがあった。
- startup 時に neutral Joy を入れることで stale button edge の影響を下げた。
- sim では lift 操作に L1+Y/A を使うため、navigation launch の `joy_estop_button` は `-1` にして topic-based ESTOP のみにした。

---

## Costmap / camera まわり

local costmap は camera depth 由来の `/camera/depth/points` を使う。

今回 camera sensor を movable holder 配下へ移したため、camera が下を向きすぎる・holder が動いたまま navigation へ入ると、地面が obstacle として入りやすくなる。

現在の対策:
- AUTO 中は camera pan/swing を初期位置へ戻す。
- robot 移動中も camera pan/swing command を受け付けない。
- local costmap `camera_marking.min_obstacle_height` は `0.10` に戻した。直近テストではこの値で OK。

注意:
- camera position を大きく変えた場合、local costmap の height filter と camera TF を再確認する。
- `/camera/camera/depth/camera_info` と `/camera/depth/points` が正しく出ているかを見る。
- RViz で地面が local costmap に混ざる場合、まず camera が AUTO 初期角度へ戻っているか確認する。

---

## 検証コマンド

起動:

```bash
tools/sim/open_sim_nav2_windows.sh --cleanup
```

主要 topic:

```bash
ros2 topic echo /motion_policy/status
ros2 topic echo /cmd_vel
ros2 topic echo /nav2/cmd_vel
ros2 topic echo /sim/steer_target
ros2 topic echo /sim/drive_target
ros2 topic echo /sim/arm_joint_states
ros2 topic echo /robot_mode
```

debug:

```bash
ls -1dt debug-output/sim_nav2_* | head
sed -n '1,220p' debug-output/<latest>/latest_state.md
```

---

## 今後の改善候補

- localization drift を追う場合、debug recorder に `/gicp_loc/score`, `/gicp_loc/pose`, `/fast_lio/odometry` を追加する。
- 起動順序由来の drift を減らす場合、Nav2 goal 送信前に `/wheel_odom`, `odom -> base_footprint`, `map -> odom` が安定するまで待つ check を入れる。
- motion policy が強すぎて狭所で曲がりにくい場合、`lateral_scale: 0.10 -> 0.15` または `max_align_wz: 0.55 -> 0.45/0.50` を比較する。
- SteeringChangeCritic は segfault 原因を切り分けてから再導入する。
