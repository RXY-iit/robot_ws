# Robot Harness System 中間サマリー

## 1. システムの目的

Robot Harness System は、Isaac Sim 上のロボットを対象に、自然言語コマンドを安全な ROS2 Skill 実行へ変換するための階層型エージェント基盤です。

現在の第一段階では、以下を重視しています。

```text
自然言語コマンド -> Intent 解析 -> Skill Plan -> ROS2 実行 -> Dashboard 可視化
```

低レベル制御を LLM に直接任せず、LLM はあくまで「曖昧な言語解析の補助」として使います。
実際の移動、カメラ、リフト、認識、停止、安全確認は typed Skill 経由で実行します。

## 2. 現在のレイヤ構成

### L2 Skill Layer

ROS2 package:

```text
src/robot_skills
```

役割:

```text
ROS topic / action / sensor / semantic map を安全な再利用可能 API に変換する層
```

L2 は基本的に Python / rclpy で実装されています。
LLM は使いません。

### L3 Agent Layer

ROS2 package:

```text
src/robot_agent
```

役割:

```text
ユーザーコマンドを解析し、複数 Skill を組み合わせた実行計画を作る層
```

L3 は Mission Agent / Navigation Agent / Health / Recovery Agent / Perception Agent に分かれています。
ここで deterministic parser、embedding parser、llama.cpp fallback parser を使います。

### Dashboard

ROS2 node:

```text
robot_agent dashboard_node
```

役割:

```text
mission state
agent activity
skill plan
skill result
robot health
human confirmation choices
detection image artifacts
```

をローカル Web UI で表示します。

## 3. Skills

### `get_robot_status`

ロボット状態を取得します。

技術:

```text
ROS2 topic monitor
/robot_mode
/safety_status
/gicp_loc/pose
/cmd_vel
/gicp_loc/score
```

出力:

```text
robot_mode
safety_status
localization_warning
stationary_drift_m
gicp_score
latest_odom_pose
last_trusted_pose
```

補足:

```text
latest_odom_pose は現在は /gicp_loc/pose 由来の map pose として扱う
wheel_odom は base motion の参考値であり、localization health の主入力にはしない
0 velocity command は「移動中」として扱わない
```

### `await_robot_auto_mode`

ナビゲーション前に AUTO mode を確認します。
人が A / confirm を押す設計です。

理由:

```text
Agent が勝手に real robot を AUTO にしないため
```

### `check_localization_health`

ナビゲーション前に localization が健康か確認します。

確認内容:

```text
map frame の localization pose があるか
stationary drift が大きくないか
GICP score が悪くないか
localization_warning が OK か
```

### `recover_localization_spin`

localization が悪い場合の回復 Skill です。

動作:

```text
1. robot を停止
2. latest trusted pose を /initialpose に publish
3. その場でゆっくり回転
4. GICP / localization が回復したら mission 継続
5. timeout したら NEEDS_HUMAN
```

技術:

```text
geometry_msgs/Twist -> /cmd_vel_raw
geometry_msgs/PoseWithCovarianceStamped -> /initialpose
GICP score monitor
```

この Skill は Health / Recovery Agent から呼ばれます。
通常 navigation より優先され、localization が BAD の時は mission を一時停止して recovery を試します。

### `resolve_semantic_target`

自然言語の目標名を map frame の approach pose に変換します。

技術:

```text
semantic_map.yaml
region polygon
region_side_offset approach pose generator
alias matching
ambiguity detection
```

例:

```text
06棚 -> shelf_06 -> generated approach pose
04 -> shelf_03_04_region or shelf_04_05_region -> NEEDS_HUMAN
```

### `list_known_targets`

semantic map に登録されている目標一覧を返します。

### `goto_pose`

Nav2 で map frame の pose に移動します。

技術:

```text
nav2_msgs/action/NavigateToPose
tf2 map -> base_footprint final pose check
Nav2 SimpleGoalChecker
```

Nav2 が success を返しても、最終 pose が目標からずれていれば失敗扱いにします。

最終誤差:

```text
Final error xy = navigation 終了時の robot pose と target pose の XY 差
Final error yaw = navigation 終了時の robot yaw と target yaw の差
```

これは localization drift ではなく、navigation goal に対する到達残差です。

timeout / final error が起きた場合、Health / Recovery Agent が一度だけ tolerance profile を調整できます。

現在の方針:

```text
1回目 timeout:
  health が OK で、残差が tolerance 調整で吸収可能なら retry

2回目 timeout:
  NEEDS_HUMAN

localization BAD:
  tolerance 調整より先に localization recovery
```

profile 例:

```text
default: fine
shelf_approach: 棚番号確認用なので xy/yaw を少し広めに許可
medium / loose: policy DB に無い task 用の fallback 候補
```

### `cancel_motion`

現在の Nav2 goal を cancel します。

### `move_relative`

robot の現在姿勢を基準に相対移動します。

例:

```text
go forward 1m
go back a little
左へ50cm
```

現在の設計:

```text
Skill は「相対移動したい」という typed command を作る
実際の経路追従 / 障害物回避は Nav2 / motion policy / safety layer に任せる
```

距離の解釈:

```text
a little / 少し -> 0.20m
明示距離 1m / 50cm -> その値を使用
```

### `rotate_relative`

現在 yaw を基準に相対回転します。

例:

```text
turn around -> 180deg
turn right -> -90deg
```

### `look_direction`

base を動かさず、camera pan / tilt のみで指定方向を見ます。

例:

```text
see left
左を見る
see front up
```

これは semantic target ではなく camera operation として扱います。

### `set_camera_pose`

カメラ pan / tilt を指定します。

技術:

```text
/chokudomotor/target_angle
/cameraswingmotor/target_angle
/chokudomotor/angle feedback
/cameraswingmotor/angle feedback
```

移動中のカメラ角度は固定せず、robot が停止した後に Perception Agent が scan profile を使って観察します。

### `capture_observation`

現在の camera image を保存します。

技術:

```text
/camera/camera/color/image_raw
debug-output/perception/<mission_id>/observation_*.jpg
observation_*.json
```

### `detect_known_object`

撮影済み observation から対象を検出します。

現在の方式:

```text
Perception query planner
primitive target registry
YOLO11s object detection
OCR text/digit detection
HSV color-region rule
composite matcher
annotated image artifact with bbox overlay
```

使用モデル / 技術:

```text
perception_targets.yaml
perception_models.yaml
models/perception/yolo11s.pt
Ultralytics YOLO
OpenCV
Tesseract OCR
EasyOCR fallback
HSV color rule
digit / letter OCR rule
query primitive mapping
color-object overlap matcher
```

`check **` の後段は、単に query を YOLO に投げるのではなく、まず query を perception primitive に分解します。

primitive:

```text
color: red / blue / green / yellow / orange / black / white
object: car / bottle / bucket / box / person / forklift / pallet
text: digit / letter / label / sign
abstract: unsafe / dangerous / hazard
unsupported attribute: small / big
```

query 例:

```text
06 -> text/digit -> OCR
A letter -> text/letter -> OCR
red -> color -> HSV color region
red car -> composite(color=red, object=car) -> color region + YOLO + overlap
red small car -> red car までは検出し、small は未検証なので NEEDS_HUMAN
red book -> red は検出可能だが book は未登録 object なので NEEDS_HUMAN
桶 / バケツ -> bucket object。YOLO11s に直接 bucket が無い場合は fallback 候補を出し human confirmation
不安全なもの -> abstract concept なので第一版は NEEDS_HUMAN
```

定義済み target は以下で管理します。

```text
src/robot_skills/config/perception_targets.yaml
```

モデル backend は以下で管理します。

```text
src/robot_skills/config/perception_models.yaml
```

これにより、将来 `yolo11_car` や `yolo11_box_custom` のような custom model を追加しても、上位の `red car` / `blue car` の planner logic は変更せず、object target の detector / classes だけを差し替えられます。

例:

```text
06番号 -> OCR text contains 06
04番号 05番号 -> both 04 and 05 must be observed
赤い箱 -> red color region + box-like YOLO target + overlap check
車 -> YOLO car/truck/bus/vehicle label
```

Dashboard は `annotated_*.jpg` を優先表示します。
緑の枠は matched target、黄色の枠は detected candidate、低 confidence は赤系の枠として表示します。

### `move_lift_bounded`

リフトを指定距離だけ上下させます。

自然言語 mapping:

```text
少し / ちょっと / a little -> 5cm
大きく / a lot -> 15cm
10cm -> 0.10m
5cm -> 0.05m
```

現在の sim bridge は `/lift/command` の `UP / DOWN / STOP` だけを受け取るため、Skill 側で距離を時間に変換します。

```text
duration_sec = distance_m / 0.06 m/s
```

## 4. Agents

### Mission Agent

全体の司令塔です。

担当:

```text
command parsing
intent validation
skill plan creation
skill execution
mission state management
human prompt
retry / recovery flow
dashboard event publishing
```

Mission Agent は LLM の出力を直接実行しません。
必ず Skill registry と Skill payload に変換してから実行します。

### Navigation Agent

semantic target を Nav2 移動 plan に変換します。

基本 plan:

```text
resolve_semantic_target
await_robot_auto_mode
check_localization_health
goto_pose
```

shelf 系目標では、移動後に visual arrival confirmation を追加します。

### Health / Recovery Agent

robot health を定期的に評価し、必要なら mission を pause / recovery する Agent です。

理由:

```text
位置推定が壊れている時に別ルートを試すより、まず pose estimate を回復するべきだから
navigation timeout が単なる tolerance 問題なのか、localization 問題なのかを分ける必要があるから
```

監視対象:

```text
robot_mode
safety_status
/gicp_loc/pose
/gicp_loc/score
stationary drift
last trusted pose
active recovery state
```

優先順位:

```text
1. localization BAD -> stop / cancel / recover_localization_spin
2. safety BAD -> stop / human
3. navigation timeout with health OK -> tolerance profile retry
4. second timeout -> NEEDS_HUMAN
```

Health state は Dashboard の Robot Health block に表示されます。
通常は詳細を隠し、BAD / WARN の時だけ原因を読めるようにします。

### Perception Agent

robot 停止後に周囲を観察し、対象物や棚番号を確認します。

現在の Perception Agent は、camera scan の後に `detect_known_object` を呼びます。
`detect_known_object` は互換性のため skill 名を維持していますが、内部では query planner を持ちます。

設計方針:

```text
すべての composite を手書き定義しない
red / blue / car / bottle / text など primitive を定義する
red car / blue bottle は primitive から自動的に composite plan を生成する
未定義 object や未検証 attribute は NEEDS_HUMAN
abstract safety concept は第一版では human / future VLM に委譲する
```

現在の scan:

```text
ground left / center / right
middle left / center / right
upper left / center / right
return camera to navigation neutral pose
detect_known_object
```

合計 9 枚の画像を使います。

## 5. 技術選定と理由

### ROS2 Python / rclpy

理由:

```text
既存 workspace が ROS2 Humble ベース
Skill を ROS topic / action と自然に接続できる
sim と real の境界を作りやすい
```

### Nav2

移動は Nav2 の NavigateToPose を使います。

理由:

```text
経路計画と障害物回避を手書きしない
既存 navigation stack と統合しやすい
失敗 status / cancel / timeout を扱いやすい
```

現在の command flow:

```text
RViz / Agent goal
  -> Nav2 MPPI /nav2/cmd_vel
  -> nav_mode_switch_node motion policy
  -> /cmd_vel_raw
  -> cmd_vel_safety_node
  -> /cmd_vel
  -> Isaac / base driver
```

方針:

```text
長距離の path planning / obstacle avoidance は Nav2 / costmap に任せる
motion policy は近距離の後方目標など限定された補正だけ行う
safety layer は速度・加速度・jerk の制限を担当する
base driver は停止後の steering neutral と motor-level safety を担当する
```

最近追加した motion policy:

```text
NEAR_BACK_REVERSE:
  target が後方 ±60deg かつ 1.5m 以内なら、turn around せず後退する
```

最近追加した safety:

```text
cmd_vel slew-rate limiter
acceleration limit
jerk limit
watchdog zero output
ESTOP immediate zero output
steering neutral after stop
```

MPPI trajectory visualization は RViz では通常非表示にし、Odometry history も現在値だけを表示します。

### semantic_map.yaml + polygon approach pose

固定 waypoint を覚えるのではなく、棚の region polygon から approach pose を生成します。

理由:

```text
一度停めた具体座標に依存しない
棚の範囲を後から修正できる
approach side / label end / distance を map data として管理できる
```

### YOLO11s + OCR/rule

最初の perception は YOLO11s / OCR / color rule / composite matcher の hybrid です。

理由:

```text
YOLO は物体候補に強い
棚番号は OCR/rule の方が確認しやすい
red box は「red region + box target + overlap」として検証する方が YOLO 単独より自然
将来 custom YOLO training に移行しやすい
```

### SentenceTransformer + llama.cpp fallback

自然言語解析は local-first hybrid です。

構成:

```text
deterministic parser
SentenceTransformer / multilingual-e5-small
llama.cpp server
qwen2.5-3b-instruct-q4_k_m.gguf
```

理由:

```text
よく使うコマンドは deterministic parser で高速・安定
少し言い方が変わった command は embedding fast path
本当に曖昧な command だけ local LLM に fallback
LLM latency と誤動作リスクを抑える
外部 API に依存しない
```

## 6. LLM が呼ばれるタイミング

LLM は毎回呼ばれません。

解析順序:

```text
1. DeterministicParser
2. EmbeddingParser
3. llama.cpp fallback
```

### DeterministicParser

以下は LLM なしで直接解析されます。

```text
キャンセル
ロボット状態
知っている場所を教えて
06棚に行って
04-05棚を確認
06棚に赤い箱がありますか
リフトを10センチ上げて
move forward 1m
go back a little
see left
```

### EmbeddingParser

template に近いが完全一致しない command を分類します。

技術:

```text
intfloat/multilingual-e5-small
accept_threshold: 0.94
clarify_threshold: 0.75
```

### llama.cpp fallback

deterministic / embedding で処理できない command のみ呼びます。

使用モデル:

```text
qwen2.5-3b-instruct-q4_k_m.gguf
```

endpoint:

```text
http://127.0.0.1:8080/v1/chat/completions
```

役割:

```text
自然言語を MissionIntent JSON に変換するだけ
ROS topic 名や座標は作らせない
motion / safety / contact を直接許可しない
```

### Dynamic navigation tolerance

navigation timeout 時に毎回すぐ失敗扱いにするのではなく、Health / Recovery Agent が一度だけ tolerance を調整できます。

判断順序:

```text
1. localization health を確認
2. localization BAD なら recovery を優先
3. health OK かつ final error が tolerance profile 内なら retry / accept
4. profile DB に無い task は LLM に fine / medium / loose の選択だけを依頼可能
5. 2回目 timeout は NEEDS_HUMAN
```

LLM に許可すること:

```text
task_type から tolerance class を選ぶ
```

LLM に許可しないこと:

```text
数値座標を作る
Nav2 parameter を直接変更する
safety limit を変更する
```

## 7. 実行できるコマンド例

### 状態確認

```text
ロボット状態
status
health
robot status
```

### 既知目標一覧

```text
知っている場所を教えて
登録場所一覧
list known targets
```

### ナビゲーション

```text
06棚に行って
04-05棚に行って
go to shelf 06
go to home
move forward 1m
go back a little
move left 50cm
turn around
```

### 確認 / inspection

```text
06棚を確認して
04-05棚を確認
inspect shelf 06
check shelf 04-05
```

### 探索 / detection

```text
06棚に赤い箱がありますか
04-05棚に赤い箱がありますか
車がありますか
check for a car
find red box near shelf 06
is there any box?
```

### 視線 / camera direction

```text
左を見る
前方上を見る
see left
see front up
look right
```

### リフト

```text
lift up 10cm
lift down 5cm
リフトを10センチ上げて
リフトを少し下げて
リフトを大きく下げて
```

### キャンセル

```text
キャンセル
cancel
stop
```

## 8. Dashboard の現在の役割

Dashboard は operator / developer 向けの local Web UI です。

表示:

```text
現在の command
Mission status
Intent
Intent Routing
Skill Plan
現在動いている Agent
Skill result
Localization warning
Robot Health blocks
Health / Recovery action
Dynamic tolerance profile / final error
Detection result image
Human confirmation choices
Raw mission state
Mission events
```

Intent Routing block:

```text
command が Deterministic / Embedding / LLM Fallback のどこで解析されたか
最終 MissionIntent JSON
parser の reject / accept 理由
```

Robot Health block:

```text
Overall
Mode
Safety
Localization
drift_m
gicp_score
recommended_action
```

Skill Plan block:

```text
各 skill の実行順
担当 Agent
payload
result code
final error
tolerance profile
recovery result
```

Detection Result block:

```text
matched observation image
annotated bbox image
detected candidates
matched target
```

曖昧な semantic target の場合:

```text
04 -> shelf_03_04_region / shelf_04_05_region
```

Dashboard に選択ボタンが出ます。
選択すると、候補 id を使った新しい command を publish します。

## 8.5 Typed Perception Query

Perception Agent は `PerceptionQueryAnalyzer` で query を先に構造化します。
曖昧な自由文字列をそのまま YOLO / OCR に渡さず、実行可能な query type に正規化します。

現在の主な形式:

```text
shelf_label_06 -> OCR exact token "06"
text_BOX       -> OCR exact token "BOX"
object_box     -> YOLO object class / alias
color_red      -> HSV color region
red_box        -> color + object composite
```

`check 6` や `check A` のような bare query は曖昧なので、Deterministic では実行せず Human confirmation に送ります。
現在の OCR は棚ラベル確認を優先し、単独 digit / single letter の汎用認識は第一版では扱いません。

表示方針:

```text
metrics には raw detections を残す
annotated image / dashboard には matched または高 confidence の bbox を優先表示
低 confidence OCR ノイズはユーザー向け画像では抑制
```

## 9. 現在採用している方針

採用した方針:

```text
Local-first
Skill-first
LLM fallback only
Semantic map driven navigation
Perception after stop
Human confirmation for AUTO / ambiguity
Localization recovery before navigation retry
```

採用しなかった方針:

```text
LLM が直接 ROS topic を publish する
LLM が座標を生成する
一度停めた座標を固定 waypoint として覚える
camera pose を一つだけ固定して判定する
red box / shelf label を完全に cloud VLM に任せる
```

理由:

```text
robot control は再現性と安全性が必要
sim から real へ移す時に typed Skill boundary が必要
失敗時に dashboard / log で原因を追える必要がある
```

## 10. 次の課題

```text
red box detector の実データ評価
shelf label OCR の accuracy 改善
semantic map の 01-06 棚 region 完成
Dashboard の日本語 UI polish
Localization recovery の実機安全確認
YOLO11s custom training dataset 作成
dynamic tolerance profile DB の拡張
reverse / near-back motion policy の実機検証
cmd_vel acceleration / jerk limit の実機 parameter tuning
motor current / temperature / fault feedback の Health 連携
box / object detector の false positive 削減
```
