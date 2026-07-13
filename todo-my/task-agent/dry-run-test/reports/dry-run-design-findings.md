# Dry-Run Design Findings — sim_real_lab_001 & warehouse

> Round 4 — 2026-07-07. A real architecture refactor, not another patch round: redrew the
> Parser/Perception contract (parser hands off a raw phrase; PerceptionSkills owns 100% of
> color/object/text decomposition), replaced the perception "object-like" heuristic with a
> real, data-derived capability model (the actual yolo11s.pt checkpoint's class list, not a
> guess), added symmetric partial-capability handling, and added a mission-plan-level
> capture-skip so the robot doesn't move the camera through a 9-pose scan for a query with
> zero detectable signal. Found and fixed four more bugs along the way (two of them only
> visible once the new capability model existed to expose them). Rebuilt the Parser Matrix
> (raw-phrase contract, category-scoped Embedding-Ambiguous coverage) and replaced the two
> scenario-split Perception Matrix files with one scenario-agnostic capability matrix,
> verified byte-identical under both map bundles. **201/201 rows green** across all six
> matrices. See §5 below for the full account; prior rounds (1-3, 2026-07-06) are preserved
> beneath it.

## Round 4 — capability model refactor (2026-07-07)

*(Section numbers below restart from this round; §0-4 further down are rounds 1-3 from
2026-07-06, preserved as-is.)*

### The redesigned contract

**Parser → Perception handoff.** Previously `DeterministicParser` pre-typed perception
queries into canonical forms (`"green door"` → `"green_door"`), duplicating logic that
`PerceptionSkills` already did better — three separate places
(`DeterministicParser._typed_perception_query`, `HybridParser._normalize_perception_query`,
`PerceptionSkills`) each tried to classify the same query, and the first two caused F12
(canonical underscore forms getting rejected) simply by disagreeing with the third. Fixed by
deleting the classification logic from both parser-side normalizers — `object_query` is now
just the lightly-cleaned raw phrase (`"red box"`, `"anything dangerous"`, `"salfjogasf"`),
and `PerceptionSkills` is the only place that decides what it means.

**Capability model.** Replaced `PerceptionSkills._object_like_candidate` (a hardcoded
16-word English suffix list — `door, cart, chair, table, desk, cabinet, shelf, sofa, box,
bin, lamp, sign, board, monitor, fridge, microwave`) with a real lookup against
`yolo_open_vocab_classes` in `perception_targets.yaml`: the literal 80 COCO class names
extracted from the actually-configured checkpoint
(`torch.load("models/perception/yolo11s.pt")['model'].names`). Per your call: capability =
what the real model can do today; when a new model (e.g. one trained on your own data)
lands, regenerate this list from it and the capability matrix expands automatically — no
heuristic in between.

This changed real behavior, not just code structure: `door`, `cart`, `sofa`, `desk`,
`cabinet`, `shelf`, `bin`, `lamp`, `board`, `monitor`, `fridge` are **not** real COCO-80
classes, so they're no longer treated as attemptable — they correctly resolve to
"unsupported" now. `chair` and `laptop` *are* real classes and weren't in the old
suffix list, so they newly work. The existing `door`/`cart` pytest assertions were rewritten
against `chair` to preserve the actual thing being tested (open-vocabulary matching without
pre-registration) without asserting something the real model can't do.

**Symmetric partial capability**, per your explicit correction — not just
color-known/object-unknown, every direction: object-known/color-unknown, text-known/
color-unknown, etc. `"golden car"` (object known, color not registered) still detects the
car. `"turquoise chair"` (same shape, harder) exposed a real bug in my own rewrite: the
open-vocab matcher glued the unmatched color word to the object word during string
compaction (`"turquoise chair"` → `"turquoisechair"`, which matches nothing), so `chair` was
missed. Fixed by tokenizing before compaction and trying each word independently, plus
surfacing the leftover unmatched word as its own "not a recognized color/qualifier" reason
rather than silently dropping it.

**Mission-plan capture-skip.** `PerceptionAgent.build_search_plan` used to unconditionally
build the full 9-pose camera scan (18 skill calls) before ever checking whether the query
was even classifiable — `"check for salfjogasf"` would move the camera through the entire
scan pattern before finally reporting NEEDS_HUMAN. Now `PerceptionAgent` pre-classifies the
query via the same capability model; if there's zero signal at all (`abstract`/
`unsupported`), it skips straight to a single `detect_known_object` call with no camera
motion. That required reordering `detect_known_object` itself too — it used to require at
least one captured observation before it would even check the query's classification,
so a zero-observation "unsupported" call fell into a generic stub message instead of
NEEDS_HUMAN; the classification check now runs first, before anything that depends on
captured images.

```
check for red box / red rectangle  -> full 9-pose scan (some signal -- color, and object for red box)
check for anything dangerous       -> await_robot_stationary + detect_known_object only, no scan
check for salfjogasf               -> same, no scan
```

### New bugs found and fixed this round

- **"check X" verb ambiguity** — `"check meeting table"` regressed to
  `search_object_here("meeting table")` once the old typed-query gate (which used to
  accidentally reject "meeting table" and let it fall through to `inspect_place`) was
  removed. Fixed properly: `"check"` is genuinely ambiguous between "search for an object
  described as X" and "go inspect the place named X" (unlike `find`/`search`/`look for`,
  which are unambiguously search verbs) — resolved using the same `known_targets` map-aware
  list added for F9: if X is already a known target's own name, prefer `inspect_place`.
- **`look_direction` regex not end-anchored** — `"check front door"` matched the
  `look_direction` pattern as a prefix (`"check front"`) and silently ignored the trailing
  `"door"`, becoming `look_direction` instead of `inspect_place`. Pre-existing bug, unrelated
  to this round's refactor, surfaced by testing "front door" more thoroughly. Fixed by
  anchoring the regex with `$`.
- **Strict-typed and free-text queries disagreed** — `PerceptionQueryAnalyzer` (the
  older strict-pattern matcher for `"red_bucket"`, `"object_car"`, `color_X` forms) had its
  own independent, pre-capability-model `_object_plan`/`_composite_plan` that dumped every
  configured class regardless of real capability — so `"red bucket"` (free text) and
  `"red_bucket"` (canonical form) produced *different* `yolo_classes`. Unified: both paths
  now funnel through one `_finalize_detection_plan` capability step, verified to produce
  byte-identical plans for both forms.
- **Duplicate unsupported-reason messages** — a fully-unknown query used to get two
  near-identical reasons (`'"X" is not one of the current model's detectable classes'` and
  `'No configured perception target matches "X"'`). Fixed by only emitting the generic
  fallback message when nothing more specific was already recorded.

### Other findings, not fixed (documented in cases/, not code changes)

- **`find red_box near shelf 06`** (query-before-place word order) still isn't recognized
  as `search_object_near_place` — the search-near regex only matches `"shelf N for/with
  QUERY"`. Low priority; noted in `parser_warehouse_en.csv`.
- **JP open-vocabulary gap**: `青い椅子` ("blue chair") loses the object half entirely —
  `椅子` has no JP alias anywhere, and the open-vocab class list is English-only (COCO class
  names). Registered objects with JP aliases (box/car/forklift/pallet/person) are unaffected.
- **LLM Fallback is architecturally hard to reach via natural paraphrase.** With the
  configured embedding model (`intfloat/multilingual-e5-small`) and thresholds
  (accept=0.86, clarify=0.72), nearly every English sentence — including deliberately
  off-domain ones like *"the stock market closed lower today"* — scores 0.79-0.92 against
  *some* template, and the moment a second template ties within 0.04 of the top score (very
  common), `HybridParser` routes to the Ambiguity-Gate-style `ask_human` response *before*
  ever trying LLM Fallback, regardless of how low that top score actually is. The reliable
  way to reach LLM Fallback today isn't a low embedding score at all — it's a *high-scoring,
  non-ambiguous* template match whose slot extraction then fails (the shelf-range and
  underscore-form cases from Round 2/3, both of which got promoted to fully Deterministic
  this round and no longer need it). Net effect: your requested "3 LLM Fallback cases per
  category" turned out to not really be constructible with genuine natural-language
  examples — I've documented verified Embedding-Ambiguous examples per category instead (2-3
  each) and flagged this finding rather than fabricate LLM-landing rows. Worth knowing if
  LLM Fallback coverage matters to you: it may need either a stricter ambiguity-tie window
  or a lower accept_threshold to ever fire on natural phrasing.

### Test suite after this round

```
cases/parser_real_lab_en.csv       (54 cases)
cases/parser_real_lab_jp.csv       (30 cases)
cases/parser_warehouse_en.csv      (25 cases)
cases/parser_warehouse_jp.csv      (24 cases)
cases/perception_capability.csv    (34 cases -- replaces the two scenario-split files;
                                     run under both ROBOT_MAP_ID=sim_real_lab_001 and
                                     ROBOT_MAP_ID=warehouse via scripts/run_all_dry_run.sh,
                                     verified byte-identical output both times)
```

**201/201 rows pass.** `perception_capability.csv` is organized by capability cell, not by
scenario or by "what we happen to have registered" — color capable/not-capable, object
direct/fallback-only/registered-but-zero-capability/unregistered-but-real-class/not-a-real-
class-at-all/abstract, text/OCR, and every combination of "one half capable, other half
not" in both directions, plus JP variants. `pytest` also expanded (5→6 perception tests)
to cover the same capability distinctions at the unit level.

## 0. What changed since round 1

Fixed, rebuilt, and regression-tested (12/12 pytest still green throughout):

- **F1** — `maps/sim_real_lab_001/dashboard_examples.yaml`'s `inspect` template
  `"look at {target}"` → `"inspect {target}"` (parses correctly now). Note: this file
  turned out to be **orphaned config** — `dashboard_node.py`'s real button generator
  (`_build_quick_commands`) never reads it; it queries `SemanticMap` directly and appends a
  hardcoded, Japanese-only primitive list. Fixing the template string keeps the data file
  internally correct for whenever it does get wired up, but the actual live dashboard still
  isn't using it. Flagging as a separate, not-yet-actioned observation.
- **F2** — Deleted the stale placeholder `front_door`/`back_door` entries under `places:` in
  `maps/sim_real_lab_001/semantic_map.yaml` (the calibrated versions under `objects:` are
  now the only copies). Also added `SemanticMap._drop_duplicate_ids` so a future
  places/objects id collision can't silently reintroduce this class of bug.
- **F3** — `DeterministicParser` now recognizes `look for` and `is there (a/an)?` as search
  verbs, alongside `check/find/search`.
- **F4** — `_is_plausible_semantic_target` now rejects a pronoun/vague-word stoplist (`it`,
  `that`, `this`, `there`, `here`, `something`, `それ`, `あれ`, `これ`, `どこか`, `なにか`, …), so
  `check it` / `それを確認して` correctly reach `ask_human` instead of `inspect_place`.
- **F5** — `_parse_rotate_relative` now handles the combined `turn/rotate <left|right> <N>
  degrees` form in both English and Japanese (`turn right 90 degrees`, `右に90度回転して`).
- **F7** — `SemanticMap.find_matches` now does whole-word/token containment instead of raw
  substring containment, so `"love"` no longer false-matches the sofa's `loveseat` alias.
- **F8** — `SemanticMap._path_for_map_id` now also tries a `__file__`-relative repo-root
  fallback, so map-bundle lookup no longer depends on the process's cwd.
- **F6** — deliberately **not** fixed (per your call): typo/fuzzy target matching is a
  design decision, not a quick bug fix.

Two more small, low-risk fixes came out of building the expanded test matrices below
(discovered while testing "all command types," fixed opportunistically the same way F1-F8
were, not part of the original list):

- **F10** — `EmbeddingParser._extract_query`'s JP suffix-stripping regex stripped `見える` /
  `ある` / `ありますか` but not the subject particle `が` immediately before them, so
  `"赤い箱が見える"` — a literal, verbatim example from `parser_templates.yaml` — always
  left a dangling `が` in the extracted query and needed the (optional, not always running)
  LLM fallback to recover. Fixed the regex to consume the optional `が`; now resolves
  cleanly at the Embedding layer.
- **F12** — `DeterministicParser._typed_perception_query`'s `object`/`color`/composite
  regexes only recognized a **space** between the type prefix and value (`"object car"`,
  `"red box"`), not the **underscore** form (`"object_car"`, `"red_box"`) that
  `parser_templates.yaml` and `HybridParser._normalize_perception_query` both already treat
  as canonical. This silently broke three of the warehouse scenario's own template examples
  (`"check shelf 06 for red_box"`, `"check object_car"`, `"find red_box"` all fell through to
  Embedding/LLM Fallback or the wrong task type). Changed `\s+` to `[\s_]+` in those four
  regexes so both forms are recognized identically.

## 1. Round 3 — F9 and F11 fixed properly, plus a new parser-level mechanism

### F9 — `parser_layer_error`: `_clean_target` mis-strips "front" from compound target names — fixed

Your read on this was right and changed the design, not just the patch: a blind
token-blacklist (`target.replace("front", "")`) can never tell "front" the direction from
"front" the noun-modifier. Two changes landed:

1. **`_strip_relational_phrase`** replaces the blind replace loop. It only strips a
   relational qualifier when it's structurally a separate phrase wrapping the target —
   `^(?:in front of|near|next to|beside)\s+(.+)$`, a trailing `... near`, and the JP
   equivalents (`近く`/`そば`/`前側`/`の前`) — never a bare substring match. `"front door"`
   doesn't match any of these patterns, so it passes through untouched; `"near sofa"` still
   correctly reduces to `"sofa"`.
2. **Known-target priority**: `DeterministicParser` now optionally takes a `known_targets`
   frozenset (normalized id/name strings from the current map bundle). `HybridParser` loads
   this once at construction via `SemanticMap()` — since `ROBOT_MAP_ID` is fixed before
   parsing ever starts, this doesn't compromise the parser/semantic-resolve layer split, it
   just lets the parser use already-known map facts as a priority signal. If the
   action-verb-stripped candidate matches a known target's own name verbatim, that wins and
   relational stripping is skipped entirely — this is what makes `"front door"` (a real
   target in `sim_real_lab_001`) resolve correctly even though it contains "front". Falls
   back to the phrase-pattern-only behavior when no map is loaded (e.g. standalone unit
   tests), so nothing regresses in a map-less context.

```
'go to front door' -> navigate_to_place target='front door'   (was 'door')
'go to near sofa'  -> navigate_to_object target='sofa'         (regression check, still works)
```

### F11 — `parser_layer_error`: JP shelf-range search-near pattern only accepted a single digit — fixed

Widened the JP search-near regex's shelf-target group from `0?[1-6]棚` to
`0?[1-6](?:[-_ー〜~]0?[1-6])?棚`, mirroring the range syntax navigation already accepted.
Both forms now resolve deterministically; per your call, anything wilder (3+ segment spans,
non-adjacent ranges) is still fine to leave falling through to Embedding.

```
'04-05棚にobject_carがありますか' -> search_object_near_place target='04-05棚' object_query='object_car'
'04棚にobject_carがありますか'    -> search_object_near_place target='04棚' object_query='object_car'
```

## 2. Round-1 findings status recap

| id | summary | status |
|----|---------|--------|
| F1 | dashboard inspect template didn't parse | fixed (data file); dashboard wiring itself still orphaned (new sub-observation) |
| F2 | front_door/back_door defined twice | fixed + defensive dedupe added |
| F3 | "look for X" / "is there a X" don't parse | fixed |
| F4 | "check it" silently accepted | fixed |
| F5 | "turn right 90 degrees" not deterministic | fixed |
| F6 | no typo tolerance / no suggestions | **deferred by decision** — needs a design call, not just a fix |
| F7 | "love" substring-matches "loveseat" | fixed |
| F8 | SemanticMap lookup is cwd-dependent | fixed |
| F9 | "front door" → target lost to "door" | **fixed in round 3** — phrase-anchored stripping + known-target priority (see §1) |
| F10 | JP suffix regex left a dangling が | fixed (round 2) |
| F11 | JP shelf-range search-near only accepted a single digit | **fixed in round 3** (see §1) |
| F12 | canonical underscore forms (`red_box`, `object_car`) not recognized | fixed (round 2) |

## 3. dashboard_examples.yaml — deleted

Per your call: since it was orphaned (never read by `dashboard_node.py`, see F1 above) and
just sitting there looking authoritative, deleted
`maps/sim_real_lab_001/dashboard_examples.yaml` outright rather than fixing its template
string. Only other references were doc mentions (this report, the README, and an unrelated
`todo-my/task-sim2real/temp/next-step-thinking-summary.md` file-inventory listing) — left
that last one alone per your instruction, no code depended on the file.

## 4. Test coverage

Case-file structure split **scenario × language**, each independently runnable.

```
cases/parser_real_lab_en.csv     (34 cases, all command types)
cases/parser_real_lab_jp.csv     (20 cases, all command types)
cases/parser_warehouse_en.csv    (20 cases, all command types)
cases/parser_warehouse_jp.csv    (19 cases, all command types)
cases/perception_real_lab.csv    (12 cases, EN + JP queries)
cases/perception_warehouse.csv   (15 cases, EN + JP queries)
scripts/run_all_dry_run.sh       # runs everything, or ./run_all_dry_run.sh <name> for one file
```

`run_all_dry_run.sh` now exports the matching `ROBOT_MAP_ID` (`sim_real_lab_001` or
`warehouse`) before each parser run, since the parser's known-target priority mechanism (§1)
needs the right map bundle loaded to do anything useful.

Command types covered in every parser file (scenario-appropriate vocabulary swapped in):
`robot_status`, `cancel`, `list_known_targets`, `navigate_to_object`, `navigate_to_place`,
`search_object_near_place`, `search_object_here`, `inspect_place`, `move_relative`,
`move_relative_xy`, `rotate_relative`, `look_direction`, `move_lift_bounded`, and the
ambiguous/invalid (`ask_human`) case.

### Results (`scripts/run_all_dry_run.sh`, all green)

| file | pass/total |
|------|-----------|
| parser_real_lab_en.csv | 34/34 |
| parser_real_lab_jp.csv | 20/20 |
| parser_warehouse_en.csv | 20/20 |
| parser_warehouse_jp.csv | 19/19 |
| perception_real_lab.csv | 12/12 |
| perception_warehouse.csv | 15/15 |

**120/120.** Every row that documented a gap in round 2 (F9, F11) now has its expectation met
for real, plus new regression-check rows added alongside each fix (a "still strips a genuine
relational qualifier" check for F9, a "plain single digit still works" check for F11) so a
future change can't silently re-break either fix without the matrix catching it.

### Notable things confirmed working across both scenarios/languages

- Every one of the 13 supported task types parses correctly, deterministically, in both
  scenarios and (with two known exceptions below) both languages.
- JP composite perception (`赤い箱` / red box) works identically in both scenarios since the
  perception planner doesn't consult the map at all — only `perception_targets.yaml`, which
  is shared.
- Warehouse-specific registered objects (`forklift`/`フォークリフト`, `pallet`/`パレット`) and
  their JP aliases route correctly to YOLO.
- `戻って`/`戻る` and lift-with-magnitude now resolve deterministically in JP (previously
  Embedding-only) — see §1 recap below.

### One remaining known JP gap (not fixed, out of this round's scope)

JP composite queries on **open-vocabulary** objects like door still don't work (`青いドア`
comes back color-only, since the open-object-like heuristic in `PerceptionSkills` only
recognizes Latin-script nouns) — box/forklift/pallet/person are fine because they're
*registered* objects with JP aliases in `perception_targets.yaml`. This is a perception-planner
gap, not a `DeterministicParser` one, so it's a different fix location than everything else in
this report.

## 5. Suggested next steps

1. F6 (typo tolerance) still needs the design conversation flagged in round 1 before any
   implementation — it changes matching semantics for every existing alias, not just typos.
2. The JP open-vocabulary composite gap above, if it matters for your near-term test plan.
3. Consider extending the known-target priority mechanism (§1) to `search_object_near_place`'s
   shelf-target extraction too, if a similar "informative word looks like a relational
   qualifier" collision ever shows up there — not needed today, no such case was found.
