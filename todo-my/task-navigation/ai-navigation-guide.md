# AI-Ready Navigation システム構築方案

> 作成: 2026-05-20  
> 対象: robot_ws（全向底盘 + Livox Mid360 + RealSense D435 pan/tilt + Lift）  
> 前提: Nav2 + FAST-LIO2/GICP localization は既に動作確認済み。Nav2 の残課題は別途対処する。  
> 目的: **次フェーズのシステム設計方針を整理し、議論すべき論点を明確にする。**

---

## 0. 現在地の確認

### 0.1 確定している TF チェーン

```text
map -> odom               : gicp_localizer_node（FAST-LIO2 hint + GICP）
odom -> base_footprint    : robot_odom_node（wheel odom）
base_footprint -> base_link -> livox_frame : URDF（static）
base_link -> camera_link  : URDF（固定、現在は pan/tilt TF 未対応）
```

### 0.2 現在のパッケージ構成

| パッケージ | 状態 | 役割 |
|---|---|---|
| `robot_bringup` | ✅ | センサー起動、Joy-Con、lift、camera motor |
| `omni_base_driver` | ✅ | `/cmd_vel` → モーター + wheel odom |
| `localization_pkg` | ✅ | map_server + GICP + FAST-LIO2 |
| `nav_pkg` | ✅ | Nav2 + RPP controller + nav_mode_switch_node |
| `robot_description` | ✅ | URDF/xacro |
| `serial_transciever` | ✅ | lift + camera pan/tilt (Joy-Con) |
| `tf_tools` | ✅ | TF utility |
| `my_messages` | ✅ | カスタム msg |

### 0.3 現在の Nav2 制御チェーン

```text
Nav2 Goal
  -> planner_server（Smac Hybrid-A* or NavFn）
  -> controller_server（RPP）
  -> /nav2/cmd_vel
  -> nav_mode_switch_node
     ├─ AUTO: relay -> /cmd_vel
     └─ MANUAL: zero /cmd_vel + cancel goal
  -> omni_base_driver
  -> モーター
```

### 0.4 現在の BT（Phase 1 安全 BT）

`navigate_to_pose_phase1_wait_only.xml`:
- planning 失敗 → global costmap clear → retry
- control 失敗 → local costmap clear → retry
- 両方失敗 → 両 costmap clear → wait 2s
- Spin / BackUp は **無効化**（実機安全重視）

---

## 1. 目標システム設計：レイヤーアーキテクチャ

TsukuTsuku レポートの「State Manager + Skill API」の思想を採用し、以下の5層構造を目標とする。

```
┌──────────────────────────────────────────────────────┐
│  Layer 5: LLM / AI Task Planner                      │
│  タスク解釈・目標地点選択・状態判断                         │
│  Claude API / Vision LLM / Memory                    │
└───────────────────┬──────────────────────────────────┘
                    │ 受限 Task API のみ
┌───────────────────▼──────────────────────────────────┐
│  Layer 4: Mission Manager                            │
│  タスク状態機（IDLE/NAVIGATING/WAITING/BLOCKED/         │
│  OBSERVE/FAILED）・Nav2 goal 発行・retry/cancel 判断   │
└───────────────────┬──────────────────────────────────┘
                    │ Nav2 Action / Semantic Events
┌───────────────────▼──────────────────────────────────┐
│  Layer 3: Perception / Semantic Layer                │
│  RealSense depth → local costmap                     │
│  物体検出 → 目標候補・Semantic Zone・Signal              │
└───────────────────┬──────────────────────────────────┘
                    │ /scan, /camera/scan, /map, /costmap
┌───────────────────▼──────────────────────────────────┐
│  Layer 2: Nav2 Navigation                            │
│  Global Planner / Controller / BT / Costmap           │
│  （Layer 4 の指示通りに安全移動する）                     │
└───────────────────┬──────────────────────────────────┘
                    │ /nav2/cmd_vel
┌───────────────────▼──────────────────────────────────┐
│  Layer 1: Motor Safety Layer                         │
│  AUTO/MANUAL 仲裁・速度上限・cmd_vel watchdog・急停      │
└───────────────────┬──────────────────────────────────┘
                    │ /cmd_vel
┌───────────────────▼──────────────────────────────────┐
│  Layer 0: Hardware                                   │
│  omni_base_driver / lift / camera motor              │
└──────────────────────────────────────────────────────┘
```

**設計原則:**
- AI（Layer 5）は `/cmd_vel` を直接触らない
- AI は受限 Task API 経由でのみ Mission Manager に指示を出す
- Nav2（Layer 2）は低層安全移動を担当する
- Safety Layer（Layer 1）は AI / Nav2 の両方より下位にあり、常に有効

---

## 2. TsukuTsuku レポートをこのロボットに写す

TsukuTsuku 報告（`ref/TsukuTsuku.md`）の借鉴点を、このシステムに対応させると以下になる。

### 2.1 センサー分離原則

| TsukuTsuku | このロボット |
|---|---|
| 上部 VLP-16: localization | Livox Mid360: FAST-LIO2 + GICP localization（確定） |
| 前方 Mid-360 × 2: 障害物 | **RealSense D435 depth: 近距離補完**（これから実装） |
| 魚眼カメラ: 信号/ラベル識別 | RealSense RGB + AI: 物体・ゾーン認識（Phase 4-5） |

**現在の Livox 近地盲区:** Livox は 1.7m 以内の地面低い障害物を見にくい。RealSense depth で補う。

### 2.2 TF Switcher の思想を引き継ぐ

TsukuTsuku では Autoware localization ↔ LIO-SAM を TF Switcher で切り替えた。

このロボットで将来使える場面:

```text
通常屋内: FAST-LIO2 + GICP（現在確定）

地図外・新規エリア: Real-time SLAM モード（将来）

localization 破綻時: wheel odom + IMU フォールバック

視覚再定位成功時: 位姿補正（将来）
```

Nav2 は `map -> odom -> base_footprint` だけを見る。後ろの localization 実装が変わっても Nav2 は変更不要。

### 2.3 State Manager / Waypoint Publisher パターン

TsukuTsuku の「State Manager → Waypoint Publisher → Nav2」を、  
このロボットの「Mission Manager → Nav2 Action」として実装する。

```text
Mission Manager の状態:
  IDLE
  NAVIGATING        ← Nav2 goal を実行中
  WAITING           ← 障害前で停止・待機
  OBSERVE_AROUND    ← 停止してカメラスキャン
  APPROACH_PRECISE  ← 低速・高精度接近
  MANIPULATE        ← lift 動作・物体把持
  BLOCKED           ← 複数回 retry 失敗
  FAILED            ← human 介入要求
  MANUAL            ← Joy-Con 手動
```

### 2.4 Binary Filter / Speed Filter

TsukuTsuku の区域別「避让 or 等待」「速度制限」をこのロボット用に置き換える:

| 区域タイプ | 避障策略 | 速度 |
|---|---|---|
| 通常走廊 | 通常避让 | normal |
| 狭小通路 | 禁止大幅回避・低速 | slow |
| 扉・エレベーター前 | 等待、禁止回避 | zero |
| 機器・棚 付近 | 禁止回避、精密接近 | precision |
| リフト動作中 | 停止保持 | zero |
| カメラスキャン中 | 停止保持 | zero |

**実装:** Nav2 の SpeedFilter Layer + MissionManager による「移動許可ゲート」の組み合わせ。

### 2.5 Recovery BT の拡張

現在の Phase1 BT:

```text
plan fail → clear global → retry
control fail → clear local → retry
both fail → clear both → wait 2s
```

次フェーズで追加すべき recovery:

```text
still blocked → backup 0.3m → replan
still blocked → ask Mission Manager
Mission Manager → BLOCKED 状態遷移
Mission Manager → LLM に状況報告 → 対処指示待ち
```

### 2.6 DWB/Controller の「等待中の振れ動作」問題

TsukuTsuku で報告された「待機中に前後ゆらゆら」問題は RPP でも起きる可能性がある。

対処方針:
- WAITING 状態では Mission Manager が `zero /cmd_vel` を強制発行
- Nav2 goal は一旦 cancel して pure waiting にする（controller が動かない状態を保証）
- Nav2 再開は Mission Manager が障害消失を確認してから

---

## 3. フェーズ別ロードマップ

### Phase 1: Nav2 基盤安定化（進行中）

目標: 低速短距離 goal でエンドツーエンド安定動作を確認。

- [x] FAST-LIO2 + GICP localization 確認
- [x] Nav2 RPP + Phase1 BT 実装
- [x] AUTO/MANUAL ジョイコン切替
- [x] behavior_server が `/cmd_vel` を直接 publish しないことを確認
- [ ] 実機 short-distance goal で TF / costmap / cmd_vel が一貫していることを確認
- [ ] localization drop を検出する仕組みを確認

### Phase 2: RealSense 近距離センシング統合

目標: Livox 近地盲区を RealSense depth で補完。

```text
実装手順:
  1. depthimage_to_laserscan → /camera/scan 発行
  2. local costmap に camera_scan observation source 追加
  3. RViz で近距離障害物が local costmap に入ることを確認
  4. 障害物移動で costmap が clearing されることを確認
  5. Phase1 実機テストで Nav2 が近距離を認識できることを確認
```

**注意:** カメラ pan/tilt 中は TF がずれるため、nav 中はカメラを Nav pose に固定する。

Nav pose policy:
```text
NAVIGATING       → camera 固定（Nav pose）
WAITING / MANUAL → camera スキャン可能
OBSERVE_AROUND   → robot 停止後に camera scan → Nav pose 復帰後に nav 再開
```

### Phase 3: Safety Layer + Footprint

目標: `robot_radius` を実機 polygon footprint に置き換え、safety layer を独立実装。

- [ ] 実機寸法から polygon footprint を計測・設定
- [ ] inflation_radius と cost_scaling_factor を実機に合わせて調整
- [ ] motor safety node: cmd_vel watchdog + 速度上限 + 急停
- [ ] MANUAL 時の teleop が `/teleop/cmd_vel` → safety layer → `/cmd_vel` を通るように整理

### Phase 4: Mission Manager 実装

目標: Nav2 goal の直接発行から、状態機ベースの Mission Manager に移行。

```text
新パッケージ: src/mission_manager/
  - Nav2 action client
  - タスク状態機
  - waypoint list 管理
  - retry / cancel / wait policy
  - /camera_pose_state と連動（カメラが Nav pose に戻るまで nav 再開しない）
```

**この時点で Nav2 は「Mission Manager のサブシステム」になる。**

### Phase 5: AI / Semantic 接続

目標: Semantic Layer と LLM Task Planner を接続。

```text
Semantic Layer:
  RealSense RGB → 物体検出（人/棚/扉/危険物）
  検出結果 → SemanticObject[] msg
  SemanticObject → Mission Manager の目標候補リスト更新

LLM Task Planner:
  タスク: "棚 A の前に移動して状態を報告"
  → Tool API: go_to(shelf_A_wait_point)
  → Mission Manager が Nav2 goal 発行
  → 到着後 Tool API: report_status() → LLM 判断

LLM が使える Tool API（制限付き）:
  go_to(target_id)          ← 登録済み waypoint のみ
  cancel_navigation()
  wait(seconds)
  get_status()
  report_obstacle(description)
  ask_human(message)
```

---

## 4. 議論・確認が必要な設計判断

以下は現時点で方針が固まっていない項目。一緒に検討したい。

### 4.1 カメラ TF 動的更新の優先度

現在 camera_link は URDF 固定。Pan/tilt が動くと TF とセンサー位置がずれる。

選択肢:
- **A: Nav 中はカメラ角度固定**（シンプル、現実的）
- B: Joint state publisher で pan/tilt 角度を TF に反映する（精度高いが実装コスト高）

→ **現時点は A 推奨。Phase 4 以降で B を評価する。**

### 4.2 RPP から MPPI への切り替えタイミング

RPP は安定しているが、全向移動・動的障害物での柔軟性は MPPI が有利。

論点:
- MPPI のパラメータチューニングのコスト vs 得られる柔軟性
- RPP を Phase 3 まで使い続けて Phase 4 でシミュレーション評価するのが現実的か

→ **RPP を baseline として継続。Phase 4 で仮想環境での比較評価をする。**

### 4.3 Mission Manager の実装言語

選択肢:
- Python（スピード重視、既存スクリプトと統一）
- C++（レイテンシ重視）

→ **Python で先行実装、性能問題が出たら C++ 移植を検討。**

### 4.4 Localization 失敗検出と recovery 方針

GICP score が下がった時（現在 threshold 0.5、観測値 0.32）に何をするか。

選択肢:
- **A: score 低下を検出したら Mission Manager に通知 → BLOCKED 遷移 → stop**
- B: GICP score が閾値を下回ったら自動で wheel odom フォールバックに切り替え
- C: 現在の場所で停止して人間介入を要求

→ **現時点は A が最安全。B・C は Phase 4 で検討。**

### 4.5 Semantic Map / Zone 管理の形式

将来の「エリア別避障策略」や「スピードフィルター」のために、地図にゾーン情報を付与する方法。

選択肢:
- Nav2 の KeepoutFilter + SpeedFilter zone を YAML で定義
- 独自の SemanticMap node で polygon zone を管理
- LLM が自然言語でゾーンを定義する（長期目標）

→ **Phase 4 から始める。まず KeepoutFilter + SpeedFilter で試す。**

### 4.6 Lift と Navigation の連携ルール

リフト動作中に Nav2 goal を受け付けてよいか。

方針（確認したい）:
```text
lift 動作中 → Navigation 禁止（Mission Manager が WAITING にする）
Navigation 中 → lift 大幅移動 禁止（camera weight shift によるリスク）
Navigation 到着後 → lift 動作 OK（Mission Manager が MANIPULATE 状態に遷移）
```

### 4.7 LLM 接続のタイミング

Phase 5 を「いつ」始めるか。  
Semantic detection（物体認識）なしに LLM だけ接続しても意味が薄い。

提案:
```text
Phase 4 完了 → Mission Manager が安定
Phase 5 前半 → RealSense RGB + 物体検出モデル（YOLOv8 等）のみ
Phase 5 後半 → LLM Task Planner 接続
```

---

## 5. 新規実装が必要なもの

### 5.1 今後追加するパッケージ

```text
src/mission_manager/         ← Phase 4 で新規作成
  mission_manager_node.py    ← タスク状態機 + Nav2 action client
  task_state.py              ← 状態定義
  waypoint_map.yaml          ← 登録 waypoint 一覧

src/safety_layer/            ← Phase 3 で新規作成
  cmd_vel_safety_node.py     ← watchdog + 速度上限 + 急停
  (将来 cmd_vel_mux を統合)

src/perception_semantic/     ← Phase 5 で新規作成
  semantic_object_pub.py     ← 物体検出結果の正規化
  semantic_zone_manager.py   ← ゾーン情報管理
```

### 5.2 既存パッケージへの追加

| パッケージ | 追加内容 |
|---|---|
| `nav_pkg` | BT に wait-for-human / blocked-check / semantic condition ノード追加 |
| `nav_pkg` | KeepoutFilter + SpeedFilter 用 zone.yaml |
| `nav_pkg` | MPPI controller 設定ファイル（評価用） |
| `serial_transciever` | camera_pose_manager_node: Nav pose 自動制御 |
| `robot_bringup` | navigation_logic_test.launch.py: fake sensors + Nav2 logic テスト |
| `localization_pkg` | localization_monitor_node: score + TF freshness 監視 |

---

## 6. この robot の想定タスクシナリオ（議論用）

最終的に何ができる robot を目指すかを共有しておく。

### シナリオ例 1: 棚への物品搬送

```text
1. LLM "棚 A に移動して物品を置いて"
2. Mission Manager: NAVIGATING → 棚 A wait point へ
3. 到着: OBSERVE_AROUND → カメラスキャン → 棚 A 位置確認
4. APPROACH_PRECISE → 0.05 m/s で棚に近づく
5. MANIPULATE → lift 上昇 → 物品を置く → lift 下降
6. NAVIGATING → 元の位置に戻る
7. LLM に "完了" を報告
```

### シナリオ例 2: 障害物前でブロックされた場合

```text
1. Nav2 → global path 失敗 × 2
2. BT → costmap clear × 2 → wait 2s
3. still blocked → Mission Manager に通知
4. Mission Manager: BLOCKED 状態
5. LLM に "障害物で 〇〇前でブロック" を報告
6. LLM: "10秒待って retry" → wait → retry
7. still blocked → LLM: "人間に介入を要求"
8. Mission Manager: ask_human()
```

### シナリオ例 3: localization 低下

```text
1. localization_monitor: /gicp_loc/score < 0.5
2. Mission Manager 通知
3. Mission Manager: WAITING → robot 停止
4. LLM に "localization 信頼度低下" を報告
5. LLM: "その場で停止して recovery を待て"
6. localization が回復したら自動で NAVIGATING 再開
7. 回復しない場合: ask_human()
```

---

## 7. 参考ファイルとリソース

| ファイル | 内容 |
|---|---|
| `todo-my/ref/TsukuTsuku.md` | Tsukuba Challenge 報告の分析とこのシステムへの借鉴点 |
| `todo-my/step7-navigation-ai-simulation-plan.md` | Phase 1-5 詳細実装計画（v1） |
| `todo-my/status-0425.md` | 現状の実機確認結果 |
| `src/nav_pkg/behavior_trees/navigate_to_pose_phase1_wait_only.xml` | 現在の BT |
| `src/nav_pkg/config/nav2_params.yaml` | 現在の Nav2 パラメータ |
| `src/localization_pkg/launch/fast_lio_localization_live.launch.py` | 実機 localization 起動 |
| `debug-output/nav2_*/latest_state.md` | 実機テストログ |

---

## 8. 議論メモ（随時更新）

> ここに議論の結果や決定事項を追記していく。

- 2026-05-20: 初版作成。Phase 1-5 ロードマップと設計論点を整理。
