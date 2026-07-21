from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any

from robot_agent.schemas import MissionIntent


@dataclass
class TemplateExample:
    template_id: str
    task_type: str
    text: str


class EmbeddingParser:
    def __init__(
        self,
        templates: list[dict[str, Any]],
        model_name: str = "intfloat/multilingual-e5-small",
        accept_threshold: float = 0.86,
        clarify_threshold: float = 0.72,
    ):
        self.templates = templates
        self.model_name = model_name
        self.accept_threshold = accept_threshold
        self.clarify_threshold = clarify_threshold
        self.examples = self._flatten_examples(templates)
        self._model = None
        self._example_embeddings = None
        self._tfidf_vocab: dict[str, int] = {}
        self._tfidf_vectors: list[list[float]] = []
        self.last_error: str | None = None

    def parse(self, text: str) -> tuple[MissionIntent, dict[str, Any]]:
        if not self.examples:
            return MissionIntent(task_type="ask_human", raw_text=text), {"source": "embedding", "reason": "no_templates"}
        matches = self._top_matches(text, limit=3)
        score, example = matches[0] if matches else (0.0, None)
        ambiguous = self._is_ambiguous(matches)
        metrics = {
            "source": "embedding",
            "score": score,
            "template_id": example.template_id if example else None,
            "model": self.model_name,
            "engine": "sentence_transformers" if self._model is not None else "tfidf_fallback",
            "embedding_error": self.last_error,
            "top_matches": [
                {
                    "score": item_score,
                    "template_id": item.template_id,
                    "task_type": item.task_type,
                    "text": item.text,
                }
                for item_score, item in matches
            ],
            "ambiguous": ambiguous,
        }
        if example is None or score < self.accept_threshold or ambiguous:
            metrics["needs_fallback"] = True
            metrics["needs_clarification"] = score >= self.clarify_threshold
            return MissionIntent(task_type="ask_human", raw_text=text), metrics
        intent = self._intent_from_template(example, text)
        return intent, metrics

    def _top_matches(self, text: str, limit: int = 3) -> list[tuple[float, TemplateExample]]:
        try:
            self.last_error = None
            return self._top_matches_sentence_transformers(text, limit)
        except Exception as exc:
            self.last_error = repr(exc)
            return self._top_matches_tfidf(text, limit)

    def _top_matches_sentence_transformers(self, text: str, limit: int) -> list[tuple[float, TemplateExample]]:
        if self._model is None:
            from sentence_transformers import SentenceTransformer

            self._model = SentenceTransformer(self.model_name)
            self._example_embeddings = self._model.encode([item.text for item in self.examples], normalize_embeddings=True)
        query_embedding = self._model.encode([text], normalize_embeddings=True)[0]
        scored: list[tuple[float, TemplateExample]] = []
        for index, emb in enumerate(self._example_embeddings):
            score = float(sum(float(a) * float(b) for a, b in zip(query_embedding, emb)))
            scored.append((max(0.0, min(1.0, score)), self.examples[index]))
        scored.sort(key=lambda item: item[0], reverse=True)
        return scored[:limit]

    def _top_matches_tfidf(self, text: str, limit: int) -> list[tuple[float, TemplateExample]]:
        if not self._tfidf_vectors:
            self._build_tfidf()
        query = self._vectorize(text)
        scored: list[tuple[float, TemplateExample]] = []
        for index, vec in enumerate(self._tfidf_vectors):
            score = self._cosine(query, vec)
            scored.append((max(0.0, min(1.0, score)), self.examples[index]))
        scored.sort(key=lambda item: item[0], reverse=True)
        return scored[:limit]

    def _is_ambiguous(self, matches: list[tuple[float, TemplateExample]]) -> bool:
        if len(matches) < 2:
            return False
        best_score, best = matches[0]
        if best_score < self.clarify_threshold:
            return False
        high = [
            (score, item)
            for score, item in matches[1:]
            if score >= self.clarify_threshold and best_score - score <= 0.04
        ]
        return any(item.task_type != best.task_type or item.template_id != best.template_id for score, item in high)

    def _build_tfidf(self):
        vocab = sorted({token for item in self.examples for token in self._tokens(item.text)})
        self._tfidf_vocab = {token: index for index, token in enumerate(vocab)}
        self._tfidf_vectors = [self._vectorize(item.text) for item in self.examples]

    def _vectorize(self, text: str) -> list[float]:
        vec = [0.0] * len(self._tfidf_vocab)
        for token in self._tokens(text):
            index = self._tfidf_vocab.get(token)
            if index is not None:
                vec[index] += 1.0
        return vec

    @staticmethod
    def _cosine(left: list[float], right: list[float]) -> float:
        dot = sum(a * b for a, b in zip(left, right))
        lnorm = math.sqrt(sum(a * a for a in left))
        rnorm = math.sqrt(sum(b * b for b in right))
        if lnorm == 0.0 or rnorm == 0.0:
            return 0.0
        return dot / (lnorm * rnorm)

    @staticmethod
    def _tokens(text: str) -> list[str]:
        lowered = text.lower()
        words = re.findall(r"[a-zA-Z0-9_]+|[\u4e00-\u9fff]", lowered)
        return words

    @staticmethod
    def _flatten_examples(templates: list[dict[str, Any]]) -> list[TemplateExample]:
        flattened: list[TemplateExample] = []
        for item in templates:
            template_id = str(item.get("id") or "")
            task_type = str((item.get("intent") or {}).get("task_type") or "ask_human")
            for example in item.get("examples", []):
                flattened.append(TemplateExample(template_id=template_id, task_type=task_type, text=str(example)))
        return flattened

    @staticmethod
    def _intent_from_template(example: TemplateExample, raw: str) -> MissionIntent:
        if example.task_type in {"robot_status", "list_known_targets", "cancel"}:
            return MissionIntent(task_type=example.task_type, raw_text=raw)
        if example.task_type == "move_lift_bounded":
            lowered = raw.lower()
            if any(token in lowered for token in ["up", "上げ"]):
                direction = "UP"
            elif any(token in lowered for token in ["down", "下げ", "下ろ"]):
                direction = "DOWN"
            else:
                direction = "UP"
            return MissionIntent(
                task_type="move_lift_bounded",
                constraints={"direction": direction, "distance_m": EmbeddingParser._lift_distance_m(raw)},
                raw_text=raw,
            )
        if example.task_type == "move_relative":
            direction, distance_m = EmbeddingParser._extract_relative_slots(raw)
            return MissionIntent(
                task_type="move_relative",
                constraints={"direction": direction or "forward", "distance_m": distance_m or 0.5},
                raw_text=raw,
            )
        if example.task_type == "move_relative_xy":
            dx_m, dy_m = EmbeddingParser._extract_relative_xy_slots(raw)
            return MissionIntent(
                task_type="move_relative_xy",
                constraints={"dx_m": dx_m, "dy_m": dy_m},
                raw_text=raw,
            )
        if example.task_type == "rotate_relative":
            return MissionIntent(
                task_type="rotate_relative",
                constraints={"yaw_deg": EmbeddingParser._extract_yaw_deg(raw)},
                raw_text=raw,
            )
        if example.task_type == "look_direction":
            direction, band = EmbeddingParser._extract_look_slots(raw)
            return MissionIntent(
                task_type="look_direction",
                constraints={"direction": direction, "band": band},
                raw_text=raw,
            )
        if example.task_type == "search_object_here":
            return MissionIntent(
                task_type="search_object_here",
                relation="near",
                object_query=EmbeddingParser._extract_query(raw),
                raw_text=raw,
            )
        if example.task_type == "search_object_near_place":
            target, query = EmbeddingParser._extract_search_slots(raw)
            return MissionIntent(
                task_type="search_object_near_place",
                target=target,
                relation=EmbeddingParser._relation_from_text(raw),
                object_query=query,
                raw_text=raw,
            )
        target = EmbeddingParser._extract_target(raw)
        return MissionIntent(
            task_type=example.task_type,
            target=target,
            relation=EmbeddingParser._relation_from_text(raw),
            raw_text=raw,
        )

    @staticmethod
    def _extract_search_slots(text: str) -> tuple[str | None, str | None]:
        for sep in ["find", "search for", "look for"]:
            if sep in text:
                left, right = text.split(sep, 1)
                return EmbeddingParser._extract_target(left), right.strip(" ，,。.")
        return EmbeddingParser._extract_target(text), None

    @staticmethod
    def _extract_target(text: str) -> str | None:
        cleaned = re.sub(r"^(move to|go to|inspect|check)\s*", "", text.strip(), flags=re.IGNORECASE)
        cleaned = re.sub(r"(near|front|近く|前側)$", "", cleaned.strip(), flags=re.IGNORECASE)
        cleaned = cleaned.strip(" ，,。.")
        return cleaned or None

    @staticmethod
    def _relation_from_text(text: str) -> str:
        return "front" if any(token in text.lower() for token in ["front", "前側"]) else "near"

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
        if any(token in text for token in ["大きく", "大きめ", "a lot"]):
            return 0.15
        if any(token in text for token in ["少し", "ちょっと", "a little"]):
            return 0.05
        return 0.05

    @staticmethod
    def _extract_query(text: str) -> str | None:
        cleaned = re.sub(r"^(check|find|search|look for)\s+(for\s+)?", "", text.strip(), flags=re.IGNORECASE)
        cleaned = re.sub(r"が?(?:ありますか|あります|ある|見える)$|を(?:探して|さがして)$", "", cleaned)
        return cleaned.strip(" ，,。.?？") or None

    @staticmethod
    def _extract_relative_slots(text: str) -> tuple[str | None, float | None]:
        lowered = text.lower()
        direction = None
        if any(token in lowered for token in ["forward", "前進", "前へ"]):
            direction = "forward"
        elif any(token in lowered for token in ["backward", "back", "後退", "後ろ"]):
            direction = "back"
        elif any(token in lowered for token in ["left", "左"]):
            direction = "left"
        elif any(token in lowered for token in ["right", "右"]):
            direction = "right"
        return direction, EmbeddingParser._distance_m(text)

    @staticmethod
    def _distance_m(text: str) -> float | None:
        lowered = text.lower()
        cm_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:cm|センチ)", lowered)
        if cm_match:
            return float(cm_match.group(1)) / 100.0
        m_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|メートル)", lowered)
        if m_match:
            return float(m_match.group(1))
        if any(token in lowered for token in ["a lot"]) or any(token in text for token in ["大きく"]):
            return 1.0
        if any(token in lowered for token in ["a little", "slightly", "little"]) or any(token in text for token in ["少し", "ちょっと"]):
            return 0.2
        return None

    @staticmethod
    def _extract_relative_xy_slots(text: str) -> tuple[float, float]:
        distance = EmbeddingParser._distance_m(text) or 0.2
        lowered = text.lower()
        if "front right" in lowered or "右前方" in text:
            return 0.70710678 * distance, -0.70710678 * distance
        return 0.70710678 * distance, 0.70710678 * distance

    @staticmethod
    def _extract_yaw_deg(text: str) -> float:
        lowered = text.lower()
        match = re.search(r"(-?\d+(?:\.\d+)?)\s*(?:deg|degree|degrees)", lowered)
        if match:
            return float(match.group(1))
        if "left" in lowered or "左" in text:
            return 90.0
        if "right" in lowered or "右" in text:
            return -90.0
        return 180.0

    @staticmethod
    def _extract_look_slots(text: str) -> tuple[str, str]:
        lowered = text.lower()
        direction = "front"
        if "front left" in lowered or "左前方" in text:
            direction = "front_left"
        elif "front right" in lowered or "右前方" in text:
            direction = "front_right"
        elif "left" in lowered or "左" in text:
            direction = "left"
        elif "right" in lowered or "右" in text:
            direction = "right"
        band = "middle"
        if "up" in lowered or "上" in text:
            band = "upper"
        elif "down" in lowered or "下" in text:
            band = "ground"
        return direction, band
