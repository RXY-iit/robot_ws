# NLU Pipeline 逻辑参考文档

> 目的：给出当前代码库中 Deterministic / Embedding / LLM 三层解析逻辑、以及 Parser Matrix / Perception Matrix 两套测试矩阵的**精确**现状描述，供后续 review 使用。所有内容基于本次会话重构后的源码逐文件重新读取确认，不是从记忆推断。
>
> 涉及文件：
> - `src/robot_agent/robot_agent/parsers/deterministic_parser.py`
> - `src/robot_agent/robot_agent/parsers/embedding_parser.py`
> - `src/robot_agent/robot_agent/parsers/llama_cpp_parser.py`
> - `src/robot_agent/robot_agent/parsers/hybrid_parser.py`
> - `src/robot_agent/config/model_policy.yaml`
> - `src/robot_agent/robot_agent/agents/perception_agent.py`
> - `src/robot_skills/robot_skills/executors/perception.py`
> - `src/robot_skills/config/perception_targets.yaml`
> - `src/robot_agent/robot_agent/tools/parser_dry_run_matrix.py`
> - `src/robot_skills/robot_skills/tools/perception_dry_run_matrix.py`

---

## 0. 整体流程（HybridParser 调度顺序）

```
输入文本
  → 1) DeterministicParser.parse()
       task_type != ask_human ?  → 直接采用（做 slot postprocess 后返回）
       task_type == ask_human ?  → 继续往下
  → 2) _detect_command_ambiguity()（Ambiguity Gate，结构性歧义，纯规则）
       命中 → 返回 ask_human + choices，流程结束
  → 3) EmbeddingParser.parse()
       accepted（score>=accept_threshold 且不 ambiguous）→ 采用
       ambiguous 或 needs_clarification → 返回 ask_human + choices（embedding_multiple_candidates），流程结束
       否则（score 太低、无模板匹配）→ needs_fallback=True，继续往下
  → 4) LlamaCppParser.parse()（仅当 3) needs_fallback 时才会调用）
       解析成功 → _validate_llm_intent() 二次校验（主要防止 bare digit/单字符逃过歧义门）
       解析失败/超时/端点不可达 → ask_human
```

`HybridParser.last_metrics["accepted_layer"]` 记录最终采纳的层（`Deterministic` / `Embedding` / `LLM Fallback` / `None`），`route_steps` 记录每一层尝试的详细轨迹（score、reason、slot_error 等），是两套 dry-run 矩阵判断 `actual_layer` 的依据（见 §4 `classify_layer`）。

---

## 1. Deterministic Parser — 逐层过滤 / 匹配逻辑

`DeterministicParser.__init__(known_targets: frozenset[str] | None)`：`known_targets` 来自 `HybridParser._load_known_targets()`，即从 `SemanticMap`（受 `ROBOT_MAP_ID` 环境变量控制加载哪张地图）里取出所有已知目标的 `id` + `names`，做 `_normalize_light` 归一化后存成一个 frozenset，作为**消歧的优先信号**（不是匹配逻辑本身，而是在多种解释都成立时用于选边）。

解析顺序（`parse()` 内部按此顺序尝试，命中即返回）：

1. 字面量：cancel / status / list_known_targets
2. `_parse_look_direction`（看向某方向）
3. `_parse_rotate_relative`（原地旋转）
4. `_parse_move_relative`（相对平移）
5. 升降台：`re.match(r"^(?:lift|リフトを)", lowered)` 且包含 `up/down/上げ/下げ/下ろ` 之一
6. `_parse_structured_search_near`（"在 X 货架找 Y"）
7. `_parse_structured_search_here`（"看看有没有 Y"，会传入 `known_targets`）
8. `_parse_structured_navigation`（移动到某地）
9. `_parse_structured_inspect`（确认/检查某地）
10. 都不匹配 → `ask_human`

### 1.1 EN 正则一览

| task_type | 正则 |
|---|---|
| search_object_near_place | `^(?:check\|find\|search\|look)\s+(?:for\s+)?(shelf[_\s-]*0?[1-6]\|shelf[_\s-]*0?[1-5][_ -]0?[2-6])\s+(?:for\|with)\s+(.+)$` |
| search_object_here | `^(?:check\|find\|search\|look)\s+(?:for\s+)?(.+)$` 、`^is there\s+(?:a\s+\|an\s+)?(.+?)\??$` |
| navigate_to_place/object | `^(?:move to\|go to)\s+(.+)$` |
| inspect_place | `^(?:inspect\|check)\s+(.+)$` |
| look_direction | `^(look\|view\|check)\s+(left\|right\|front\|forward\|up\|down\|front left\|front right)$`（本次会话修复：补上了结尾 `$` 锚点，否则 `"check front door"` 会被错误当成 look_direction 前缀匹配掉 "check front" 而丢弃 "door"） |
| rotate_relative | 字面 `turn around/back`，`turn left/right`，以及 `^(?:turn\|rotate)\s+(left\|right)\s+(-?\d+(?:\.\d+)?)\s*(?:deg\|degree\|degrees)?$` |

### 1.2 JP 正则一览

| task_type | 正则 |
|---|---|
| search_object_near_place | `^(0?[1-6]棚\|0?[1-5][-_ー〜~]0?[2-6]棚)に(.+)(?:がありますか\|あります\|ある\|見える)[？?]?$` |
| navigate（货架专用） | JP shelf-specific 变体 |
| navigate（通用） | 动词 `(?:行って\|行く\|移動して\|移動\|戻って\|戻る)` 结尾判断 |
| inspect_place | 动词 `(?:確認して\|確認\|見て\|見てください)` 结尾判断 |
| rotate_relative | `(左\|右)(?:に\|へ)?\s*(\d+(?:\.\d+)?)\s*度\s*(?:回転して\|回転\|回って\|回る)` |

### 1.3 字符过滤 / 清洗逻辑（`_clean_target`, `_extract_perception_query`）

**目标短语清洗（`_clean_target`，用于 navigate/inspect 的 target 字段）**，按顺序执行：

1. 前缀剥离循环：反复剥离 `move to / go to / inspect / check / 確認して / 確認 / 見て` 等动词前缀（循环剥离，直到不再匹配）
2. 再剥离一次英文冠词 `the / a / an`
3. 剥离结尾的 `確認して / 確認`
4. **条件性**调用 `_strip_relational_phrase`：如果剥离后的候选短语已经命中 `known_targets`（比如 "front door" 本身就是地图里的已知目标名），则**跳过**这一步，不去剥离 "front" 之类的方位限定词——避免把合法目标名错误拆碎（这是本次修的一个真实 bug：以前无条件剥离会把 "front door" 错误处理成 "door"）
5. 剥离结尾的日语助词 `に/へ/で/を`
6. 最终去除首尾空白和标点

**`_strip_relational_phrase`**（方位短语剥离，是**短语锚定**而非全局 token 替换）：
- EN: `^(?:in front of|near|next to|beside)\s+(.+)$`，`^(.+?)\s+near$`
- JP: `^(?:近く|そば)(?:の|に)?\s*(.+)$`，`^(.+?)の?(?:近く|そば|前側)$`，`^(.+?)の前$`

**感知查询清洗（`_extract_perception_query`，用于 search_object_here 的 object_query 字段）**——本次会话 Round-4 重构后**大幅简化**：
1. 剥离开头的 `a/an/the`
2. 剥离标点 `" ，,。.?？"`
3. 交给 `_is_plausible_phrase` 做合理性检查
4. **直接返回原始短语**，不再做任何"打类型标签"（如以前的 `green_door` 这种预分类），交由 Perception 层自己判断颜色/物体/文字的可检测性

**`_is_plausible_phrase`**（脏词/空泛代词黑名单，判定这个候选短语是否值得往下传）：
- 脏 token 黑名单：`探/search/find/あります/ある/見える/できます/できる/" for "/?/？`（这些残留说明正则切割位置不对，短语本身不干净）
- 空泛代词黑名单（`vague_targets`）：`it/that/this/them/there/here/something/someone/somewhere/anything/それ/あれ/これ/どこか/なにか/何か` —— 命中则视为不可用的查询，不往下传给 Perception

**`_looks_like_place`**（辅助判断某短语是否像"地点"而非"物体"）：包含 token 列表 `home/ホーム/origin/start/entrance/door/corridor/入口/廊下`（本次为修复回归新增了 `ホーム`）。

### 1.4 known_targets 消歧机制

除了上面 §1.3 提到的 `_strip_relational_phrase` 跳过逻辑外，`known_targets` 还用在 `_parse_structured_search_here`：如果动词是裸 `check` 且候选短语命中某个已知目标名，函数返回 `None`（放弃 search_object_here 的解释），把解析权交还给后面的 `_parse_structured_inspect`。这是为了解决 "check meeting table"（meeting table 是已知地点）应该走 inspect_place 而不是 search_object_here 的问题。

---

## 2. Embedding Ambiguous — 逻辑 / 阈值 / 含义

**模型**：`intfloat/multilingual-e5-small`（`sentence_transformers`），对模板库 `parser_templates.yaml` 里每条 `examples`（EN+JP）计算 embedding，`normalize_embeddings=True`，用点积（=cosine similarity）算相似度。若 `sentence_transformers` 导入失败，退化为 `_top_matches_tfidf`（简单 token TF-IDF cosine）。

**阈值 —— 代码默认值 vs 实际生效值（重要，两者不同）**：

| 参数 | 代码默认值（`EmbeddingParser.__init__` 签名） | 实际生效值（`model_policy.yaml`，src 与 install 副本一致） |
|---|---|---|
| `accept_threshold` | 0.86 | **0.94** |
| `clarify_threshold` | 0.72 | **0.75** |

`model_policy.yaml` 是 `HybridParser` 实际加载的配置，因此 **0.94 / 0.75 才是当前真实生效的判断线**，代码里的 0.86/0.72 只是没有配置文件时的兜底值。

**匹配流程（`parse()`）**：
1. 取 top-3 模板匹配
2. `metrics["needs_clarification"] = score >= clarify_threshold`（即最高分只要过了 0.75 这条线，就打上"值得澄清"的标记，不代表会真的进 ask_human）
3. `ambiguous = _is_ambiguous(matches)`
4. 判定：`example is None`（无任何模板）或 `score < accept_threshold（0.94）`或 `ambiguous` → 返回 `ask_human`；否则采用最高分模板对应的 task_type + slot 抽取结果

**`_is_ambiguous(matches)` 的真实逻辑（发现一处硬编码不一致，见 §6）**：
```
if best_score < 0.72: return False   # 注意：这里硬编码 0.72，不是 self.clarify_threshold
# 否则检查 top-3 里第2、3名是否有任何一个：
#   score >= 0.72  且  与最高分差距 <= 0.04（0.04 是"并列"容差窗口）
#   且 task_type/template_id 与最高分不同
# 命中任一条件 → 判定为 ambiguous（同一分数区间内出现了不同意图的模板，无法确定唯一解释）
```

**"ambiguous" 与 "needs_clarification" 的含义区别**：
- `needs_clarification`：只是"分数达到了 0.75 门槛"这一单一事实的记录（更多是给 dry-run/调试用的信号位，不直接决定路由）
- `ambiguous`：真正决定是否要返回 `ask_human` 的判断——最高分候选和次高分候选"太接近、但意图不同"，机器无法自动选择，必须交给人/上层澄清
- 只有 `ambiguous=True` 才会在 `constraints.ambiguity = {reason: "embedding_multiple_candidates", choices: top_matches}` 里携带候选列表返回给上层（HybridParser 直接把这个当作 ask_human 结果返回，流程终止，不会再往 LLM 层走）

**未采纳（`score < accept_threshold` 但不 ambiguous）** 的情况，则设置 `needs_fallback=True`，交由 `HybridParser` 调用 LLM 层。

---

## 3. LLM 层（LlamaCppParser）— 模型 / Prompt / 格式限制

**实际生效配置**（来自 `model_policy.yaml`，非代码默认值）：
- endpoint: `http://127.0.0.1:8080/v1/chat/completions`（llama.cpp 的 OpenAI 兼容 API）
- model: `qwen2.5-3b-instruct-q4_k_m`
- `temperature: 0`
- `timeout_sec: 2.0`
- `allowed_for_contact: false`（配合 `control_critical.llm_can_authorize_motion: false` / `llm_can_override_safety: false`，即 LLM 层严禁被用来直接授权任何运动/安全相关动作，只能做语言理解）

**输出格式限制**：请求体带 `response_format: {"type": "json_object"}`，强制模型输出合法 JSON object。

**System Prompt** 核心措辞：
> "You are a robot command parser. Output one JSON object only. Do not apologize. Do not chat. Do not explain. Never output ROS topics, coordinates, velocity commands, or prose."

**User Prompt（`_prompt(text)`）结构**：
- 声明允许的 `task_type` 枚举（14 种 + `ask_human`）
- JSON schema 示例
- 8 个 few-shot 示例（EN+JP 混合）
- 提示："Bare commands like check 6 or check A are ambiguous; use ask_human"
- ⚠️ **关于 object_query 格式的指示**：prompt 里仍然写着 "Perception object_query must be typed when possible: `shelf_label_06` for shelf labels, `text_BOX` for exact OCR text, `object_box` for YOLO object, `color_red` for color, `red_box` for color+object." —— 见 §6 标记的不一致项。

**解析响应（`_parse_json_object`）**：先剥离 markdown 代码围栏 ` ```json `，再截取第一个 `{` 到最后一个 `}` 之间内容做 JSON 解析。

**`_intent_from_dict`**：对 `task_type` 做白名单校验（14 种 + ask_human，非法值一律不采用），`relation` 字段缺省或非法时默认填 `"near"`。

**`HybridParser._validate_llm_intent` 二次校验**：如果 LLM 返回的 `search_object_here` / `search_object_near_place` 的 `object_query` 是裸数字或单字符，会重新导向 `_detect_command_ambiguity`（即即使 LLM 给出了看似确定的答案，系统仍然不信任裸数字/单字符查询，强制走一遍结构化歧义判断）。

**（附）`choose_tolerance_profile()`**：`LlamaCppParser` 里还有一个独立的 LLM 调用，用于挑选导航容差档位（fine/medium/loose），与意图解析无关，用在任务执行阶段，不属于本文档讨论的 NLU pipeline。

---

## 4. Parser Matrix 总结

**工具**：`src/robot_agent/robot_agent/tools/parser_dry_run_matrix.py`，直接调用 `HybridParser.parse()` 并读取 `last_metrics`。

**测试文件**（`todo-my/task-agent/dry-run-test/cases/`）：`parser_real_lab_en.csv`（54行）、`parser_real_lab_jp.csv`（30行）、`parser_warehouse_en.csv`（25行）、`parser_warehouse_jp.csv`（24行），分别对应两套地图场景（real_lab / warehouse）× 两种语言，运行时用 `ROBOT_MAP_ID` 环境变量切换地图（影响 `known_targets`）。

**CSV 字段**：
- 输入端：`id, category, input_text, expected_layer, expected_task_type, expected_target, expected_object_query, expected_constraints, expected_ambiguity_reason, expected_choice_count, expected_choice_task_types, notes`
- 结果端追加：`actual_layer, actual_task_type, actual_target, actual_object_query, actual_constraints, actual_ambiguity_reason, actual_choice_count, actual_choice_task_types, actual_choice_labels, layer_match, task_type_match, target_match, object_query_match, constraints_match, ambiguity_reason_match, choice_count_match, overall_pass, route_trace_summary`

**`actual_layer` 判定（`classify_layer`）**：优先取 `last_metrics.accepted_layer`（Deterministic/Embedding/LLM Fallback 之一）；若为空但 `source == "ambiguity_gate"` → `Ambiguity Gate`；若 `route_steps` 里出现过 LLM Fallback 步骤但最终没被采纳 → `Rejected`；否则 → `Embedding Ambiguous`（即卡在 Embedding 阶段没有被采纳，也没有走到 LLM）。这个函数存在的原因是 `accepted_layer` 单独无法区分"Embedding 阶段就被判 ambiguous/需要澄清而终止"和"到了 LLM Fallback 但被拒绝"这两种None的情况，需要看 `route_steps` 细节才能分开。

**判定字段含义**：
- `layer_match` / `task_type_match`：分别判断实际经过的层、实际输出的 task_type 是否与预期一致
- `target_match` / `object_query_match`：**若 `expected_layer == "LLM Fallback"` 则不参与硬性判断**（`soft_fields=True`），因为 LLM 输出的措辞不受模板锁定，只做记录不做强校验
- `constraints_match`：`constraints_subset_match`——只要求 expected 里出现的 key 在 actual 里存在且值相等（数值有 1e-6 容差），expected 未提及的 key 不影响判断（子集匹配，非全等）
- `ambiguity_reason_match` / `choice_count_match` / `choice_task_types_match`：用于校验 ask_human 分支是否真的携带了正确的候选列表——`choice_task_types_match` 只有当 `expected_layer == "Ambiguity Gate"` 时才计入硬性判断（Embedding-ambiguous 的候选是 top-3 模板，可能因浮点噪声重排/并列，只记录不强判）
- `overall_pass`：`layer_match, task_type_match, constraints_match` 恒定参与；`target_match/object_query_match/ambiguity_reason_match/choice_count_match/choice_task_types_match` 仅在对应 expected 字段非空时才计入。全部通过才是 `PASS`
- **`SKIPPED`**：当 `expected_layer == "LLM Fallback"` 且 LLM 端点实际不可达（`route_steps` 中该 layer 记录了 `error`）时，不计为 FAIL，而是标记 `SKIPPED`（跑 `--require-llm` 参数时会把 SKIPPED 强制升级为 FAIL，用于 CI 场景要求 LLM 服务必须在线）

**已知发现（本次会话记录在案，非本文档新增）**：由于当前 Embedding 阈值（accept=0.94）相对宽松、模板覆盖较广，绝大多数英文自然句在 0.79–0.92 分区间内就会命中某个模板，真正意义上"自然落到 LLM Fallback"（score < 0.75）的场景很难构造，这是文档里如实记录的一个测试局限，而非用编造用例强行凑数。

---

## 5. Perception Matrix 总结

**工具**：`src/robot_skills/robot_skills/tools/perception_dry_run_matrix.py`，直接调用 `PerceptionSkills._build_detection_plan(query)`（白盒测试 planner，不跑真实 YOLO/OCR/HSV）。

**测试文件**：单一文件 `perception_capability.csv`（34行），在两个不同 `ROBOT_MAP_ID` 场景下各跑一次，已验证两次输出**逐字节相同**——因为 Perception 层的能力模型（YOLO 能识别什么）与地图场景无关，只与 `perception_targets.yaml` 里注册的 colors/objects/text 以及真实的 YOLO 类别表相关，因此这套矩阵被设计成"场景无关"（capability-driven, scenario-agnostic），不需要像 Parser Matrix 那样按场景分文件。

**能力模型（capability model）**：
- `yolo_open_vocab_classes`：真实的 COCO-80 类别名，从 `models/perception/yolo11s.pt` 用 `torch.load(path, weights_only=False)['model'].names` 提取而来（yaml 里附有重新生成的操作说明注释）——这是"能力=真实模型事实"原则的落地，而不是靠人工猜测/heuristic
- 每个注册物体（`objects` 下）有能力等级：
  - **direct**：物体自己的 `classes` 列表与真实 YOLO 类别有交集
  - **fallback**：物体自己没有直接类别，但 `fallback_classes` 与真实类别有交集（比如用能识别的近似物体代替，如 `bucket` 用 `bottle/cup/bowl` 做 fallback）
  - **none**：`classes` 和 `fallback_classes` 都没有真实类别命中（比如 `pallet` 完全没有 fallback，属于零能力）
- 对称的部分能力处理：只要查询里有任意一个可识别信号（颜色 OR 物体 OR OCR 文字），都会触发检测尝试；只有完全没有任何已知信号的查询才会跳过整个摄像头扫描直接返回 needs_human（对应 `PerceptionAgent._is_fully_unknown_query`，见 §6）

**CSV 字段**：
- 输入端：`id, category, query, expected_type, expected_colors, expected_objects, expected_text_kind, expected_yolo_classes, expected_detectors, expected_unsupported_reason_count, expected_forces_needs_human, notes`
- 结果端追加：`actual_type, actual_colors, actual_objects, actual_text_kind, actual_yolo_classes, actual_fallback_classes, actual_fallback_requires_human, actual_detectors, actual_unsupported_reasons, actual_forces_needs_human, type_match, colors_match, objects_match, text_kind_match, detectors_match, overall_pass`

**`plan["type"]` 的取值**：`abstract`（如"unsafe"这类抽象概念）、`unsupported`（识别到了某种未支持的属性如"size"）、`text`、`color`、`object`、`composite`（颜色+物体组合）、`legacy`（未归入以上任何一类的兜底自由文本路径，见下）

**`detectors_for_plan`**：根据 plan type 反推**会调用哪些检测器**（不实际调用，只做静态推断）——`abstract/unsupported` → 无检测器；`color/composite` 且有 colors → `hsv_color_region`；`text/composite` 且有 text_kind → `ocr`；`object/composite` 且有 objects → `yolo`；`composite` 额外加 `composite_overlap`（多检测器结果做重叠比对）

**判定字段含义**：
- `type_match/colors_match/objects_match/text_kind_match/detectors_match`：均为"expected 为空则跳过判断，否则要求 actual 完全等于（或作为 set 相等）expected"
- `unsupported_count_match`（不在 RESULT_EXTRA_FIELDS 展示列，但参与 hard_checks）：`len(actual_unsupported) == expected_unsupported_reason_count`
- `forces_needs_human_match`：`actual_forces_needs_human == (expected_forces_needs_human == "true")`，其中 `actual_forces_needs_human = actual_type in ("abstract", "unsupported")` —— 即"这个查询无论摄像头拍到什么，都注定要问人类"的判定
- `overall_pass`：以上 7 项全部通过才是 PASS，否则 FAIL（此矩阵没有 SKIPPED 状态，因为不依赖任何外部服务）

---

## 6. 额外发现（供 review 参考，非用户直接提问但认为有价值的信息）

1. **LLM prompt 与 Round-4 重构后的"原始短语"契约不一致**：`LlamaCppParser._prompt()` 里仍然指示模型输出旧式带类型前缀的 `object_query`（如 `shelf_label_06` / `text_BOX` / `object_box` / `color_red` / `red_box`），但 Deterministic 和 Embedding 两层在本次重构后已经统一改为"直接传原始短语，不预分类"（交给 Perception 层自己判断能力）。这意味着**如果某条查询真的走到了 LLM Fallback**，它产出的 `object_query` 格式可能与另外两层不一致，进而影响下游 `PerceptionQueryAnalyzer` 的严格匹配路径（见第 8 点）。这是一个真实的技术债，建议要么更新 prompt 改为要求原始短语，要么在 `HybridParser` 里对 LLM 输出的 `object_query` 做一次归一化。

2. **Embedding 阈值：代码默认值与实际生效值不同**：代码里 `accept_threshold=0.86, clarify_threshold=0.72` 只是没有配置文件时的兜底，实际运行时永远读取 `model_policy.yaml` 里的 `0.94 / 0.75`。如果以后单独看 `embedding_parser.py` 源码，容易被这两个默认参数误导。

3. **`_is_ambiguous` 内部硬编码 0.72，未跟随 `clarify_threshold` 走**：即使把 `model_policy.yaml` 里的 `clarify_threshold` 改成别的值（比如 0.80），`_is_ambiguous` 判断"最高分是否达到考虑并列的门槛"这一步仍然用字面量 `0.72` 判断，不会跟着配置变化。这是一个潜在的一致性隐患：调阈值时容易忽略这个内部硬编码点。

4. **Perception 层存在两条平行路径**：`PerceptionQueryAnalyzer`（严格类型匹配，处理 `shelf_label_06` / `text_X` / `color_object` 等**已经打好类型标签**的查询）与 legacy 自由文本路径（处理普通短语，走 `_match_named_primitives` + `_open_vocab_match` 开放词表匹配）。本次会话已经把两条路径的能力计算（`yolo_classes`/`fallback_classes`）统一收敛到共享的 `_finalize_detection_plan`，但类型判定的**入口分支**本身依然是两条路径并存，未来若新增查询格式需要同时考虑这两处。

5. **`_normalize_text` vs `_normalize_token` 的差异**：前者去除所有空格/下划线/连字符做紧凑比较（用于别名匹配），后者把空格/连字符转成下划线做规范 token（用于生成 `yolo_classes` 的最终输出格式）。两者容易混用，如果要新增归一化逻辑需要明确是用于"匹配"还是"输出"。

6. **`_extract_clean_target` / `known_targets` 的裁决优先级**：`HybridParser._extract_clean_target` 里有一套独立的正则清洗（region-range、shelf-after/before、别名表），与 `DeterministicParser._clean_target` 是两套相似但独立维护的清洗逻辑（分别用于 Deterministic 直接接受时的 slot postprocess，以及 Embedding/LLM 层接受后的统一 postprocess）。如果以后要改"目标短语怎么清洗"，需要同时改两处，否则会出现"同一句子从 Deterministic 走和从 Embedding 走清洗结果不同"的情况。

7. **`PerceptionAgent._is_fully_unknown_query` 决定是否跳过 9 姿态扫描**：任务级别的另一层判断——`build_search_plan` 在真正生成 9 个摄像头姿态扫描计划前，会先问一次 `PerceptionSkills()._build_detection_plan(query)`，如果 `plan["type"] in {"abstract","unsupported"}` 就直接返回一个不移动摄像头、立即报 needs_human 的单步计划。这是任务规划层面的优化（避免做无意义的物理动作），逻辑上依赖与 Perception Matrix 完全相同的 `_build_detection_plan`，因此 Perception Matrix 的 PASS/FAIL 同时也在间接验证这条"是否跳过扫描"的判断是否正确。

8. **`HybridParser._postprocess_slots` 里 `needs_fallback` 在 Deterministic 分支是死效果**：Deterministic 直接接受时也会计算 `needs_fallback`/`slot_error`（missing_target/dirty_target/missing_object_query/ambiguous_object_query），但这些值在该分支下**不会被 HybridParser 实际使用**（不会触发降级到 Embedding/LLM）——只有 Embedding/LLM 分支才会依据它们决定是否继续往下一层走。阅读代码时容易误以为 Deterministic 层也会因为 slot 有问题而被打回重新解析，实际不会。

9. **`control_critical` 配置的安全边界**：`model_policy.yaml` 里 `llama_cpp_parser.allowed_for_contact: false` 及顶层 `control_critical.llm_can_authorize_motion: false` / `llm_can_override_safety: false` 表明 LLM 层的角色被明确限定为"自然语言 → 结构化意图"的翻译器，不允许绕开 Deterministic/Ambiguity Gate 的安全约束直接授权动作——这条策略性配置在阅读三层代码时不容易注意到，但对理解"为什么 LLM 层还要被 `_validate_llm_intent` 二次校验裸数字/单字符"这一设计动机很关键。
