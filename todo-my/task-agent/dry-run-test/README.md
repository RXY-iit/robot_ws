# Agent / Skill Dry-Run 测试

## 简介

这是 `robot_agent` / `robot_skills` 的 agent-skill 层回归测试套件。核心思路：**先 dry-run，后仿真** —— 不启动 Isaac Sim / Nav2 / YOLO，只验证"自然语言指令 → 意图 → skill 计划"这条决策链本身是否合理，把设计问题（解析漏了哪种说法、语义地图数据有没有脏东西、感知 query 会不会路由错检测器）挡在真机测试之前。

这份文档记录**现在实际是怎么运作的**（不是最初的设计草案），以及从第一次跑通到现在的修改历史。

## 目录结构

```text
dry-run-test/
  README.md              本文件
  cases/                 测试用例（CSV），按 场景 × 语言 拆分，可单独运行
    parser_real_lab_en.csv
    parser_real_lab_jp.csv
    parser_warehouse_en.csv
    parser_warehouse_jp.csv
    perception_capability.csv   能力驱动、场景无关；run_all_dry_run.sh 会用两套 ROBOT_MAP_ID 各跑一遍同一份文件
  scripts/
    run_all_dry_run.sh   一键跑全部矩阵，或指定单个 case 文件
  results/                每次运行的结果 CSV（带时间戳，可覆盖旧的）
  reports/
    dry-run-design-findings.md   历次发现的设计问题（F1…F12）及修复记录
  viewer/
    results-viewer.html  结果可视化控制台（浏览器直接打开，或通过 Artifact 查看）
```

## 整体架构

```text
自然语言 command
  -> HybridParser
     -> DeterministicParser（正则规则，最快最可预测）
     -> EmbeddingParser（语义相似度匹配模板）
     -> LlamaCppParser / LLM Fallback（本地 LLM 兜底，可选）
     -> Ambiguity Gate（结构性歧义直接生成候选项，不必等到 Embedding/LLM）
  -> MissionAgent -> NavigationAgent / PerceptionAgent -> typed Skill
  -> robot_skills executors -> Nav2 / camera / perception / semantic map
```

`DeterministicParser` 现在会读取当前 `ROBOT_MAP_ID` 对应地图的已知目标名单（`known_targets`），
作为消歧的优先级信号——地图在解析开始前就已经固定，所以用它来判断"候选词是不是已知目标名本身"
是安全的，不会破坏 parser 层和语义解析层的职责划分。同样的名单也用来判断 `check X` 到底是
"搜索一个叫 X 的物体"还是"去检查一个叫 X 的地方"（`find`/`search`/`look for` 没有这个歧义）。

**Parser → Perception 的职责边界**（Round 4 起）：解析器只负责把动词/代词之外的部分原样交出去
作为 `object_query`（例如 `"red rectangle"`，不再预先分词成 `"red_rectangle"`），颜色/物体/文字
的拆解和"能不能检测"完全交给 `PerceptionSkills` 判断。物体能力不是猜出来的——`perception_targets.yaml`
里的 `yolo_open_vocab_classes` 就是当前 `models/perception/yolo11s.pt` 这个 checkpoint 真实的
80 个类别（用 `torch.load(...)['model'].names` 直接读出来的），加上各物体自己配置的
`fallback_classes`。物体/颜色/文字里只要有任意一个维度是已知的，就仍然会触发实际检测（例如
"golden car"里 car 已知、颜色未知，一样会去检测 car）；只有当颜色、物体、文字全都判不出来时，
`PerceptionAgent` 才会跳过整个摄像头扫描序列，直接汇报需要人工确认。

## 测试范围与现状

| 分层 | 状态 | 覆盖方式 |
|---|---|---|
| A. Parser Matrix | **已固化为 CSV harness** | `cases/parser_*.csv` + `ros2 run robot_agent parser_dry_run_matrix` |
| D. Perception Planner Matrix | **已固化为 CSV harness** | `cases/perception_capability.csv`（能力驱动，场景无关，两套地图跑出来结果逐字节相同）+ `ros2 run robot_skills perception_dry_run_matrix` |
| B. Semantic Resolve Matrix | 人工 / 脚本验证，未固化 | 直接调用 `SemanticMap.find_matches` / `SemanticSkills.resolve_semantic_target`；见 `reports/dry-run-design-findings.md` 里的验证记录 |
| C. Mission Plan Matrix | 人工验证，未固化 | `ROBOT_MAP_ID=... ros2 run robot_agent agent_dry_run "<command>"`，核对 skill 顺序 |
| E. Dashboard Example Matrix | **已放弃** | `dashboard_examples.yaml` 从未被 `dashboard_node.py` 读取（是孤立配置），已直接删除；dashboard 的 quick-command 目前由 `_build_quick_commands()` 硬编码 + 动态查询 `SemanticMap` 生成 |

B、C 两层目前工作量不大（各自只有几个场景），暂时靠人工核对；如果之后要长期维护，值得补一个类似
A/D 的 CSV harness（`reports/dry-run-design-findings.md` 的"下一步"里也提到过）。

## 怎么跑

```bash
cd /home/ruan-x/robot_ws
source install/setup.bash

# 全部六份矩阵
bash todo-my/task-agent/dry-run-test/scripts/run_all_dry_run.sh

# 只跑一份（不带 .csv 后缀）
bash todo-my/task-agent/dry-run-test/scripts/run_all_dry_run.sh parser_warehouse_jp
```

脚本会按文件名自动给 parser 矩阵设置匹配的 `ROBOT_MAP_ID`（`*real_lab*` → `sim_real_lab_001`，
`*warehouse*` → `warehouse`），因为 known-target 优先级机制需要读到对的地图才有意义。

跑完之后用浏览器打开 `viewer/results-viewer.html` 看结果——首页有完整的判定逻辑说明和示例讲解，
之后每份矩阵都有筛选、搜索、点行展开明细。

## 判定逻辑（摘要，完整版见 viewer 首页）

- 每行用例有一组 `expected_*` 字段和一组 `actual_*` 字段；`expected_*` 留空 = 这一项不做硬性比对。
- 凡是 `expected_*` 非空的字段都会生成一个 `*_match`；只要有一个相关的 `*_match` 为 false，
  `overall_pass` 就判定 **FAIL**。
- 如果这一行期望走 **LLM Fallback**，但本机没有可用的本地 LLM 服务，判定 **SKIPPED**（除非用
  `--require-llm`）。
- FAIL 按设计问题分类：`parser_layer_error` / `semantic_map_error` / `mission_plan_error` /
  `skill_payload_error` / `perception_routing_error` / `dashboard_context_error`。

## 修改历史

### Round 1 — 2026-07-06

第一次把 README 设计的思路跑起来，对着真实代码验证，发现 **F1–F8** 共 8 个问题（parser 层漏词、
语义地图重复定义、子串误匹配、cwd 依赖等）。详见 `reports/dry-run-design-findings.md` §0-1（旧版）。

### Round 2 — 2026-07-06

- 修复 F1（dashboard 模板）、F2（front_door/back_door 重复定义 + 防重复保护）、F3（"look for" /
  "is there a" verb 识别）、F4（代词 stoplist）、F5（方向词+角度组合形式）、F7（子串误匹配）、
  F8（cwd 依赖）。
- F6（拼写纠错 / 模糊匹配）按讨论结果**暂缓**——这是设计决策，不是简单 bug fix。
- 把测试用例结构改成 **场景 × 语言** 四份 parser CSV + 两份 perception CSV，覆盖 `HybridParser`
  支持的全部 13 种 task_type。
- 扩充测试过程中又发现两个问题并顺手修了：**F10**（Embedding 层日文后缀正则漏吃"が"）、
  **F12**（下划线形式的 `object_car` / `red_box` 未被识别为已定型 query）。
- 新发现但未修的两个问题：**F9**（"front door" 被方位词剥离逻辑误伤成"door"）、**F11**（日文
  棚号区间格式 `04-05棚` 只有单个数字能被识别）。

### Round 3 — 2026-07-06

按讨论结果修复 F9、F11，且不是简单打补丁：

- **F9**：把方位词剥离从"逐词黑名单替换"改成"结构化短语识别"（`in front of X` / `near X` /
  `Xの前` 这类才会被剥离，"front door" 这种复合名词不会被误伤）；同时给 `DeterministicParser`
  加了一个 `known_targets` 优先级机制——`HybridParser` 构造时从当前地图读一次已知目标名单，
  候选词如果本身就是已知目标名，直接采用，跳过方位词剥离。
- **F11**：日文 search-near 正则的棚号部分从"只认单个数字"改成"单个数字和区间都认"，跟
  navigation 那边的写法保持一致。
- 顺带完成两项日文确定性解析增强：识别"戻って/戻る"作为导航动词；解耦リフト命令里数量词和
  上げて/下げて之间必须紧邻的限制。
- 删除孤立配置 `dashboard_examples.yaml`（从未被代码读取）。
- 新增本可视化工具 `viewer/results-viewer.html`。

跑完之后：**6 份矩阵、120 条用例，全部 PASS。**

### Round 4 — 2026-07-07（能力模型重构）

不是打补丁，是按讨论结果做的一次真正架构调整：

- **拆掉 parser 层的感知 query 分词逻辑**：以前 `DeterministicParser._typed_perception_query`
  会把 `"green door"` 预先分词成 `"green_door"`，`HybridParser._normalize_perception_query`
  又独立做一遍类似的校验——三个地方（含 `PerceptionSkills` 自己）各自判断同一件事，F12 就是这么
  出来的。现在两个 parser 侧的分词逻辑都删掉了，`object_query` 就是清理过动词/代词之后的原始短语，
  颜色/物体/文字的拆解 100% 交给 `PerceptionSkills`。
- **重写物体能力判定**：`_object_like_candidate`（16 个英文单词的写死后缀名单）换成对
  `yolo_open_vocab_classes` 的真实查表——这份名单是从当前配置的 `models/perception/yolo11s.pt`
  真实读出来的 80 个 COCO 类别，不是猜的。按你的要求：能力 = 当前模型真的能做什么；以后换成自己
  训练的模型，重新生成这份名单，能力矩阵就跟着扩展，不需要改判断逻辑。这也是行为变化，不只是重构：
  `door/cart/sofa/desk/cabinet/shelf/bin/lamp/board/monitor/fridge` 都不是真实 COCO 类别，现在
  正确变成"未知"；`chair/laptop` 是真实类别但原来的后缀名单没收录，现在能正常检测了。
- **对称的"部分能力"处理**（按你的纠正——不只是颜色部分已知，物体已知、颜色未知同样要处理）：
  `"golden car"`（物体已知、颜色未知）能正常检测 car；`"turquoise chair"`（更难的对称情形）
  在我自己写的新逻辑里暴露了一个真 bug——没匹配上的颜色词会在字符串压缩时和物体词粘在一起变成
  `"turquoisechair"`，谁都匹配不上；修复方式是先分词再逐词匹配，未识别的词单独作为一条"无法识别的
  颜色/修饰词"提示出来，而不是连物体一起判定失败。
- **任务规划层加了"跳过扫描"判断**：`PerceptionAgent.build_search_plan` 以前不管查询内容如何都会
  先转 9 个摄像头姿态、拍 9 张图，再去判断能不能识别；现在会先用同一套能力模型预判，如果颜色/物体/
  文字全都判不出来（`"check for salfjogasf"`），直接跳过整个扫描序列，只报告需要人工确认。为此也
  把 `detect_known_object` 内部顺序调整了一下：查询分类判断现在排在"有没有拍到图"检查之前。
- 顺带修了两个新发现的 bug：**"check X" 动词歧义**（`"check meeting table"` 一度被误判成
  search_object_here，用已有的 `known_targets` 名单区分"check 已知目标=inspect"和"check
  未知描述=search"）；**`look_direction` 正则没有末尾锚定**（`"check front door"` 被
  `"check front"` 前缀误吃掉，现在加了 `$`）。
- 还发现并统一了一个不一致：严格类型化 query（`"red_bucket"`）和自由文本（`"red bucket"`）以前
  走两套独立实现，能力模型只更新了其中一套——`PerceptionQueryAnalyzer` 自己的 `_object_plan`
  还在无脑倾倒所有配置类别。现在两条路径统一走同一个能力过滤步骤，验证过两种写法结果完全一致。
- Parser Matrix 按你的确认重新设计：`3.1` 穷举 Deterministic 分支、`3.2` 每个 task_type 类别配
  几条真实验证过的 Embedding-Ambiguous 用例。`3.3`（每类 3 条 LLM Fallback）在实测中发现**几乎
  不可能靠自然语言改写触发**——当前 embedding 模型（`intfloat/multilingual-e5-small`）
  配合现有阈值（accept=0.86, clarify=0.72），几乎任何英文句子（包括"the stock market closed
  lower today"这种完全无关的话）都能在某个模板上拿到 0.79-0.92 的分数，一旦有第二个模板分数
  接近（很常见），就会先进 Ambiguity Gate 式的 ask_human，根本轮不到 LLM Fallback。已如实记录
  这个发现，没有伪造凑数用例。
- Perception Matrix 用一份能力驱动的 `perception_capability.csv` 取代了原来按场景拆分的两个
  文件，并且验证过：同一份文件在 `ROBOT_MAP_ID=sim_real_lab_001` 和 `warehouse` 下跑出来的结果
  逐字节相同。

跑完之后：**6 份矩阵、201 条用例，全部 PASS。**

### 仍然悬而未决

- **F6**：拼写纠错 / 模糊目标匹配——需要先定设计方案（阈值、候选提示格式），再实现。
- JP 开放词汇物体的组合查询（如"青い椅子"）识别不出物体部分——"椅子"没有任何 JP 别名，且开放
  词汇类别表本身是纯英文（COCO 类别名）；已注册且带 JP 别名的物体（box/car/forklift/pallet/
  person）不受影响。
- `"find red_box near shelf 06"` 这种"查询词在前、地点在后"的词序，search-near 正则识别不出来，
  会退化成 search_object_here 并丢失 target 关联；优先级较低，已在 `parser_warehouse_en.csv` 里
  记录。
- LLM Fallback 层在当前 embedding 模型/阈值组合下几乎不可达（见上）；如果这层的覆盖率对你有意义，
  可能需要调低 accept_threshold 或收紧 ambiguity 判定窗口。
