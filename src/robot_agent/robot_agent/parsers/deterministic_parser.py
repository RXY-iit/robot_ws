from __future__ import annotations

import re
import unicodedata

from robot_agent.schemas import MissionIntent


class DeterministicParser:
    def __init__(self, known_targets: frozenset[str] | None = None):
        # Normalized (lowercase, whitespace-collapsed) known target ids/names from
        # the currently-selected map bundle, if one was loaded. The map is fixed
        # before parsing ever starts (ROBOT_MAP_ID is read once at process start),
        # so it's safe to use this as a priority signal: a candidate that already
        # matches a known target's own name wins over stripping a relational
        # qualifier (e.g. "front door" must not be mistaken for "near/in front of
        # <something>" just because it contains the word "front").
        self.known_targets = known_targets or frozenset()

    def parse(self, text: str) -> MissionIntent:
        raw = text.strip()
        # NFKC-normalize a separate copy used only for matching, so fullwidth
        # digits/units (e.g. "１０ｃｍ") behave identically to their halfwidth
        # ASCII form -- found 2026-07-14: "リフトを10ｃｍ上げてください" was
        # classified correctly but silently fell back to the vague-amount
        # default distance because the cm/mm/m regexes are ASCII-only. Keep
        # raw_text on MissionIntent as the verbatim user string, unnormalized.
        compact = unicodedata.normalize("NFKC", re.sub(r"\s+", " ", raw))
        lowered = compact.lower()
        if not raw:
            return MissionIntent(task_type="ask_human", raw_text=raw)
        if lowered in {"cancel", "stop task", "abort", "キャンセル", "タスクを停止"}:
            return MissionIntent(task_type="cancel", raw_text=raw)
        if lowered in {"status", "robot status", "robot health", "ロボット状態", "ロボットのステータス"}:
            return MissionIntent(task_type="robot_status", raw_text=raw)
        if lowered in {"known targets", "known places", "list targets", "list known targets", "場所一覧"}:
            return MissionIntent(task_type="list_known_targets", raw_text=raw)

        look = self._parse_look_direction(compact)
        if look is not None:
            return look

        rotation = self._parse_rotate_relative(compact)
        if rotation is not None:
            return rotation

        relative = self._parse_move_relative(compact)
        if relative is not None:
            return relative

        if re.match(r"^(?:lift|リフトを)", lowered) and any(
            token in lowered for token in ["up", "down", "上げ", "下げ", "下ろ"]
        ):
            direction = self._lift_direction(compact)
            distance_m = self._lift_distance_m(compact)
            return MissionIntent(
                task_type="move_lift_bounded",
                constraints={"direction": direction, "distance_m": distance_m},
                raw_text=raw,
            )

        search_match = self._parse_structured_search_near(compact)
        if search_match:
            target, query = search_match
            return MissionIntent(
                task_type="search_object_near_place",
                target=target,
                relation=self._relation_from_text(raw),
                object_query=query,
                raw_text=raw,
            )

        search_here = self._parse_structured_search_here(compact, self.known_targets)
        if search_here:
            query = search_here
            return MissionIntent(
                task_type="search_object_here",
                relation=self._relation_from_text(raw),
                object_query=query,
                raw_text=raw,
            )

        nav_match = self._parse_structured_navigation(compact, self.known_targets)
        if nav_match:
            target = nav_match
            task_type = "navigate_to_place" if self._looks_like_place(target) else "navigate_to_object"
            return MissionIntent(
                task_type=task_type,
                target=target,
                relation=self._relation_from_text(raw),
                raw_text=raw,
            )

        inspect_target = self._parse_structured_inspect(compact, self.known_targets)
        if inspect_target:
            return MissionIntent(
                task_type="inspect_place",
                target=inspect_target,
                relation=self._relation_from_text(raw),
                raw_text=raw,
            )

        return MissionIntent(task_type="ask_human", raw_text=raw)

    @staticmethod
    def _parse_structured_search_near(text: str) -> tuple[str, str] | None:
        match = re.search(
            r"^(?:check|find|search|look)\s+(?:for\s+)?(shelf[_\s-]*0?[1-6]|shelf[_\s-]*0?[1-5][_ -]0?[2-6]|0?[1-6]棚)\s+(?:for|with)\s+(.+)$",
            text,
            re.IGNORECASE,
        )
        if match:
            query = DeterministicParser._extract_perception_query(match.group(2))
            return (DeterministicParser._clean_target(match.group(1)), query) if query else None
        jp_match = re.search(
            r"^(0?[1-6]棚|0?[1-5][-_ー〜~]0?[2-6]棚)に(.+)(?:がありますか|あります|ある|見える)[？?]?$", text
        )
        if jp_match:
            query = DeterministicParser._extract_perception_query(jp_match.group(2))
            return (DeterministicParser._clean_target(jp_match.group(1)), query) if query else None
        return None

    @staticmethod
    def _parse_structured_search_here(text: str, known_targets: frozenset[str] | None = None) -> str | None:
        patterns = [
            r"^(?:check|find|search|look)\s+(?:for\s+)?(.+)$",
            r"^is there\s+(?:a\s+|an\s+)?(.+?)\??$",
        ]
        is_check_verb = bool(re.match(r"^check\s", text, re.IGNORECASE))
        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if match:
                candidate = match.group(1)
                # "check X" is structurally ambiguous between "search for an
                # object described as X" and "go inspect the place named X" --
                # "find"/"search"/"look for" are unambiguously search verbs, but
                # bare "check" isn't. If X is already a known target's own name,
                # prefer inspect_place (handled by _parse_structured_inspect)
                # over treating the target's own name as a perception query.
                # "check ON X" is the same idiom ("verify X's status") as bare
                # "check X" -- strip the "on" before the known-target check so
                # "check on the meeting table" defers to inspect_place too,
                # instead of becoming search_object_here(object_query="on the
                # meeting table") (F15 fix, 2026-07-14).
                known_target_candidate = re.sub(
                    r"^on\s+(?:the|a|an)?\s*", "", candidate.strip(), flags=re.IGNORECASE
                )
                if is_check_verb and DeterministicParser._matches_known_target(known_target_candidate, known_targets or frozenset()):
                    return None
                query = DeterministicParser._extract_perception_query(candidate)
                if query:
                    return query
        return None

    @staticmethod
    def _extract_perception_query(text: str) -> str | None:
        """Parser layer only extracts the raw perception-query phrase; it does
        NOT classify it into color/object/text/shelf_label forms. PerceptionSkills
        already does that decomposition robustly for both raw free text ("green
        door") and canonical underscore forms ("green_door", "shelf_label_06"),
        in English and Japanese alike -- duplicating a second, independently
        maintained classifier here only risks the two drifting apart (as
        happened before: this used to reject "red_box"/"object_car" because its
        own regexes required a space where the canonical form uses "_").
        """
        query = re.sub(r"^(?:a|an|the)\s+", "", text.strip(" ，,。.?？"), flags=re.IGNORECASE)
        query = query.strip(" ，,。.?？")
        if not query:
            return None
        if not DeterministicParser._is_plausible_phrase(query):
            return None
        # Defense-in-depth backstop for the F14 look_direction gap: a leftover
        # phrase that is JUST a direction word (plus optional filler) is
        # always a botched look_direction match, never a real perception
        # query -- reject it here too so any future verb combination the
        # look_direction regex doesn't cover still fails gracefully to
        # ask_human instead of returning a nonsense object_query.
        if DeterministicParser._LOOK_DIRECTION_RE.match("look " + query.lower()):
            return None
        return query

    @staticmethod
    def _parse_structured_navigation(text: str, known_targets: frozenset[str] | None = None) -> str | None:
        match = re.search(r"^(?:move to|go to)\s+(.+)$", text, re.IGNORECASE)
        if match:
            target = DeterministicParser._clean_target(match.group(1), known_targets)
            if not DeterministicParser._is_plausible_phrase(target):
                return None
            return target if DeterministicParser._target_is_recognizable(target, known_targets) else None
        jp_match = re.search(
            r"^(0?[1-6]棚|0?[1-5][-_ー〜~]0?[2-6]棚)(?:に|へ)?\s*(?:行って|行く|移動して|移動|戻って|戻る)$", text
        )
        if jp_match:
            return DeterministicParser._clean_target(jp_match.group(1), known_targets)
        jp_free_match = re.search(r"^(.+?)(?:に|へ)?\s*(?:行って|行く|移動して|移動|戻って|戻る)$", text)
        if jp_free_match:
            target = DeterministicParser._clean_target(jp_free_match.group(1), known_targets)
            if not DeterministicParser._is_plausible_phrase(target):
                return None
            return target if DeterministicParser._target_is_recognizable(target, known_targets) else None
        return None

    @staticmethod
    def _parse_structured_inspect(text: str, known_targets: frozenset[str] | None = None) -> str | None:
        match = re.search(r"^(?:inspect|check)\s+(.+)$", text, re.IGNORECASE)
        if match:
            target = DeterministicParser._clean_target(match.group(1), known_targets)
            if re.fullmatch(r"0?[1-6]", target.strip()):
                return None
            if not DeterministicParser._is_plausible_phrase(target):
                return None
            return target if DeterministicParser._target_is_recognizable(target, known_targets) else None
        jp_match = re.search(r"^(0?[1-6]棚|0?[1-5][-_ー〜~]0?[2-6]棚)(?:を)?\s*(?:確認して|確認|見て|見てください)$", text)
        if jp_match:
            return DeterministicParser._clean_target(jp_match.group(1), known_targets)
        jp_free_match = re.search(r"^(.+?)(?:を)?\s*(?:確認して|確認|見て|見てください)$", text)
        if jp_free_match:
            target = DeterministicParser._clean_target(jp_free_match.group(1), known_targets)
            if not DeterministicParser._is_plausible_phrase(target):
                return None
            return target if DeterministicParser._target_is_recognizable(target, known_targets) else None
        return None

    @staticmethod
    def _relation_from_text(text: str) -> str:
        if any(token in text for token in ["前", "front", "前面"]):
            return "front"
        if any(token in text for token in ["近く", "near"]):
            return "near"
        return "near"

    @staticmethod
    def _clean_target(text: str, known_targets: frozenset[str] | None = None) -> str:
        target = text.strip()
        previous = None
        while previous != target:
            previous = target
            target = re.sub(r"^(move to|go to|inspect|check on|check|on|確認|確認して|見て)\s*", "", target, flags=re.IGNORECASE)
            target = re.sub(r"^(?:the|a|an)\s+", "", target, flags=re.IGNORECASE)
        target = re.sub(r"(確認して|確認)$", "", target)
        # A relational qualifier ("near X", "in front of X") should only be
        # stripped when the phrase is structurally wrapping a separate target.
        # If what's left already matches a known target's own name verbatim
        # (e.g. "front door"), that match wins and nothing further is stripped.
        if not (known_targets and DeterministicParser._matches_known_target(target, known_targets)):
            target = DeterministicParser._strip_relational_phrase(target)
        target = re.sub(r"(?:に|へ|で|を)$", "", target)
        return target.strip(" ，,。.")

    @staticmethod
    def _normalize_light(text: str) -> str:
        return re.sub(r"\s+", " ", text.strip().lower())

    @staticmethod
    def _matches_known_target(candidate: str, known_targets: frozenset[str]) -> bool:
        if not known_targets:
            return False
        return DeterministicParser._normalize_light(candidate) in known_targets

    @staticmethod
    def _target_is_recognizable(target: str, known_targets: frozenset[str] | None) -> bool:
        """2026-07-14 design decision: no fuzzy/edit-distance matching here.
        If a navigate/inspect target doesn't look like anything the loaded map
        actually knows about, Deterministic declines the match entirely and
        lets Embedding/LLM -- which are far better suited to judge "is this a
        typo or paraphrase of a known place" -- take over, instead of
        silently accepting a possible typo (the old behavior for e.g. "go to
        sfoa"). A *whole-word* containment relationship in either direction is
        still allowed: a generic category word like "table" is a real word
        inside the registered "meeting table"/"meeting_table", and that
        multi-candidate case is already handled downstream by
        SemanticMap.find_matches, not by the parser. This is deliberately NOT
        raw substring matching -- "sofa" is a substring of the typo "sofaa" too,
        which would defeat the whole point of this gate, so containment is
        checked over tokens instead (same approach as
        SemanticMap._token_phrase_contains) -- split on whitespace/underscore/
        hyphen AND on script transitions (Latin-alnum run vs. everything
        else), not whitespace alone: JP text has no spaces at all between a
        registered Latin-script name and a following particle/suffix (e.g.
        "RXYのところ" for known target "rxy"), so whitespace-only splitting
        would wrongly treat the whole string as one token and reject a
        legitimate case the same way it correctly rejects "sofaa" for "sofa"
        (where there is no script boundary to exploit).
        """
        if not known_targets:
            return True  # no map loaded (e.g. a standalone unit test) -- don't gate
        normalized = DeterministicParser._normalize_light(target)
        if not normalized:
            return False
        if normalized in known_targets:
            return True
        target_tokens = DeterministicParser._script_tokens(normalized)
        for kt in known_targets:
            kt_tokens = DeterministicParser._script_tokens(kt)
            if DeterministicParser._token_run_contains(kt_tokens, target_tokens):
                return True
            if DeterministicParser._token_run_contains(target_tokens, kt_tokens):
                return True
        return False

    @staticmethod
    def _script_tokens(text: str) -> list[str]:
        return [t for t in re.findall(r"[a-z0-9]+|[^\sa-z0-9_\-]+", text) if t]

    @staticmethod
    def _token_run_contains(haystack_tokens: list[str], needle_tokens: list[str]) -> bool:
        if not needle_tokens or len(needle_tokens) > len(haystack_tokens):
            return False
        span = len(needle_tokens)
        return any(haystack_tokens[i : i + span] == needle_tokens for i in range(len(haystack_tokens) - span + 1))

    @staticmethod
    def _strip_relational_phrase(target: str) -> str:
        """Strip a relational qualifier only when it is structurally a separate
        phrase wrapping the target (e.g. "in front of X", "X near", "Xの前").
        Never strip a bare token match, since that would also corrupt a compound
        name like "front door" or "front_left" that merely contains the word.
        """
        patterns = [
            r"^(?:in front of|near|next to|beside)\s+(.+)$",
            r"^(.+?)\s+near$",
            r"^(?:近く|そば)(?:の|に)?\s*(.+)$",
            r"^(.+?)の?(?:近く|そば|前側)$",
            r"^(.+?)の前$",
        ]
        for pattern in patterns:
            match = re.match(pattern, target, re.IGNORECASE)
            if match:
                return match.group(1).strip()
        return target

    @staticmethod
    def _clean_query(text: str) -> str:
        query = text.strip()
        if not re.match(r"^[A-Za-z]\s+(?:letter|alphabet|character)$", query, re.IGNORECASE):
            query = re.sub(r"^(for|a|an|the)\s+", "", query, flags=re.IGNORECASE)
        query = re.sub(r"(?:がありますか|あります|ある|見える|を|ですか)$", "", query)
        return query.strip(" ，,。.?？")

    @staticmethod
    def _looks_like_place(target: str) -> bool:
        normalized = target.strip().lower().replace(" ", "")
        return any(
            token in normalized
            for token in [
                "home",
                "ホーム",
                "origin",
                "start",
                "entrance",
                "door",
                "corridor",
                "入口",
                "廊下",
            ]
        )

    @staticmethod
    def _is_plausible_phrase(target: str) -> bool:
        normalized = target.strip().lower().replace(" ", "")
        if not normalized:
            return False
        dirty_tokens = [
            "探",
            "search",
            "find",
            "あります",
            "ある",
            "見える",
            "できます",
            "できる",
            " for ",
            "?",
            "？",
        ]
        if any(token in normalized for token in dirty_tokens) or any(token in target for token in dirty_tokens):
            return False
        vague_targets = {
            "it",
            "that",
            "this",
            "them",
            "there",
            "here",
            "something",
            "someone",
            "somewhere",
            "anything",
            "それ",
            "あれ",
            "これ",
            "どこか",
            "なにか",
            "何か",
        }
        if normalized in vague_targets:
            return False
        return True

    @staticmethod
    def _lift_direction(text: str) -> str:
        lowered = text.lower()
        if any(token in lowered for token in ["up", "上げ"]):
            return "UP"
        if any(token in lowered for token in ["down", "下げ", "下ろ"]):
            return "DOWN"
        return "UP"

    @staticmethod
    def _lift_distance_m(text: str) -> float:
        lowered = text.lower()
        cm_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:cm|センチ)", lowered)
        if cm_match:
            return float(cm_match.group(1)) / 100.0
        mm_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:mm|ミリ)", lowered)
        if mm_match:
            return float(mm_match.group(1)) / 1000.0
        m_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|メートル)", lowered)
        if m_match:
            return float(m_match.group(1))
        if any(token in text for token in ["大きく", "大きめ"]):
            return 0.15
        if any(token in text for token in ["少し", "ちょっと"]):
            return 0.05
        return 0.05

    @staticmethod
    def _parse_move_relative(text: str) -> MissionIntent | None:
        patterns = [
            (r"^(?:move|go)\s+(forward|back|backward|left|right)\s+(.+)$", 1, 2),
            (r"^(?:move|go)\s+(front left|front right)\s+(.+)$", 1, 2),
            (r"^(forward|back|backward|left|right)\s+(.+)$", 1, 2),
            (r"^(front left|front right)\s+(.+)$", 1, 2),
            (r"^(左前方|右前方|前進|後退|左へ|右へ|左|右|前へ|後ろへ)\s*(.+)$", 1, 2),
        ]
        for pattern, direction_group, distance_group in patterns:
            match = re.search(pattern, text, re.IGNORECASE)
            if not match:
                continue
            direction = DeterministicParser._relative_direction(match.group(direction_group))
            distance_m = DeterministicParser._distance_m(match.group(distance_group), default=None)
            if direction and distance_m is not None:
                if isinstance(direction, tuple):
                    dx_unit, dy_unit = direction
                    return MissionIntent(
                        task_type="move_relative_xy",
                        constraints={"dx_m": dx_unit * distance_m, "dy_m": dy_unit * distance_m},
                        raw_text=text,
                    )
                return MissionIntent(
                    task_type="move_relative",
                    constraints={"direction": direction, "distance_m": distance_m},
                    raw_text=text,
                )
        return None

    @staticmethod
    def _relative_direction(text: str):
        normalized = text.lower().strip()
        if normalized in {"forward", "前進", "前へ"}:
            return "forward"
        if normalized in {"back", "backward", "後退", "後ろへ"}:
            return "back"
        if normalized in {"left", "左", "左へ"}:
            return "left"
        if normalized in {"right", "右", "右へ"}:
            return "right"
        if normalized in {"front left", "左前方"}:
            return (0.70710678, 0.70710678)
        if normalized in {"front right", "右前方"}:
            return (0.70710678, -0.70710678)
        return None

    @staticmethod
    def _distance_m(text: str, default: float | None = 0.5) -> float | None:
        lowered = text.lower().strip()
        cm_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:cm|センチ)", lowered)
        if cm_match:
            return float(cm_match.group(1)) / 100.0
        mm_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:mm|ミリ)", lowered)
        if mm_match:
            return float(mm_match.group(1)) / 1000.0
        m_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|メートル)", lowered)
        if m_match:
            return float(m_match.group(1))
        number_match = re.search(r"(\d+(?:\.\d+)?)", lowered)
        if number_match:
            return float(number_match.group(1))
        if any(token in lowered for token in ["a lot"]) or any(token in text for token in ["大きく"]):
            return 1.0
        if any(token in lowered for token in ["a little", "slightly", "little"]) or any(token in text for token in ["少し", "ちょっと"]):
            return 0.2
        return default

    @staticmethod
    def _parse_rotate_relative(text: str) -> MissionIntent | None:
        lowered = text.lower().strip()
        yaw_deg = None
        if lowered in {"turn around", "turn back"} or any(token in text for token in ["振り向いて", "振り向く"]):
            yaw_deg = 180.0
        elif lowered in {"turn left", "rotate left"} or any(token in text for token in ["左を向いて", "左向いて"]):
            yaw_deg = 90.0
        elif lowered in {"turn right", "rotate right"} or any(token in text for token in ["右を向いて", "右向いて"]):
            yaw_deg = -90.0
        else:
            combined_match = re.search(
                r"(?:turn|rotate)\s+(left|right)\s+(-?\d+(?:\.\d+)?)\s*(?:deg|degree|degrees)?$", lowered
            )
            jp_combined_match = re.search(
                r"(左|右)(?:に|へ)?\s*(\d+(?:\.\d+)?)\s*度\s*(?:くらい|ほど)?\s*(?:回転して|回転|回って|回る)", text
            )
            if combined_match:
                magnitude = float(combined_match.group(2))
                yaw_deg = magnitude if combined_match.group(1) == "left" else -magnitude
            elif jp_combined_match:
                magnitude = float(jp_combined_match.group(2))
                yaw_deg = magnitude if jp_combined_match.group(1) == "左" else -magnitude
            else:
                match = re.search(r"(?:turn|rotate)\s+(-?\d+(?:\.\d+)?)\s*(?:deg|degree|degrees)?", lowered)
                if match:
                    yaw_deg = float(match.group(1))
        if yaw_deg is None:
            return None
        return MissionIntent(task_type="rotate_relative", constraints={"yaw_deg": yaw_deg}, raw_text=text)

    # F14 fix (2026-07-14): the direction-word match used to be anchored with a
    # bare "$" right after the direction token, so any leading filler ("check
    # to the left") or trailing filler ("look up please") made it miss --
    # and the command would then fall through to the much looser
    # search_object_here catch-all, which happily swallowed "to the left" /
    # "up please" as a bogus perception query instead of failing gracefully.
    # Tolerating a small, explicit filler whitelist (not a generic wildcard)
    # closes that gap without risking "front left"/"front right" being
    # mis-split by the "front"/"left" alternatives (backtracking still finds
    # the two-word alternative first because the filler whitelist can't
    # absorb "left"/"right" itself).
    _LOOK_DIRECTION_RE = re.compile(
        r"^(look|view|check)\s+(?:to\s+the\s+|towards?\s+|at\s+)?"
        r"(front left|front right|left|right|front|forward|up|down)"
        r"(?:\s+(?:please|now|for me|thanks|thank you))?[.!?]*$",
        re.IGNORECASE,
    )

    @staticmethod
    def _parse_look_direction(text: str) -> MissionIntent | None:
        lowered = text.lower().strip()
        direction = None
        band = "middle"
        look_match = DeterministicParser._LOOK_DIRECTION_RE.match(lowered)
        if look_match:
            target = look_match.group(2)
            direction, band = DeterministicParser._look_slots(target)
        elif re.search(r"(左|右|前方|正面|上|下|左前方|右前方)(?:に|を)?\s*(?:ちょっと|少し)?\s*(?:見て|確認して|確認|見てください)$", text):
            direction, band = DeterministicParser._look_slots(text)
        if direction is None:
            return None
        return MissionIntent(
            task_type="look_direction",
            constraints={"direction": direction, "band": band},
            raw_text=text,
        )

    @staticmethod
    def _look_slots(text: str) -> tuple[str | None, str]:
        lowered = text.lower()
        direction = None
        if "front left" in lowered or "左前方" in text:
            direction = "front_left"
        elif "front right" in lowered or "右前方" in text:
            direction = "front_right"
        elif "left" in lowered or "左" in text:
            direction = "left"
        elif "right" in lowered or "右" in text:
            direction = "right"
        elif "front" in lowered or "forward" in lowered or "前方" in text or "正面" in text:
            direction = "front"
        elif "up" in lowered or "上" in text:
            direction = "front"
        elif "down" in lowered or "下" in text:
            direction = "front"

        band = "middle"
        if "up" in lowered or "upper" in lowered or "上" in text:
            band = "upper"
        elif "down" in lowered or "ground" in lowered or "下" in text:
            band = "ground"
        return direction, band
