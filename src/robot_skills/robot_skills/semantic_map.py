from __future__ import annotations

import math
import os
import re
from typing import Any

import yaml

from robot_skills.registry import package_config_path
from robot_skills.schemas import Pose2D, SemanticTarget


class SemanticMap:
    def __init__(self, path: str | None = None):
        self.path = path or self._selected_path()
        self.data = self._load(self.path)
        self.frame_id = self.data.get("frame_id", "map")
        self.objects = self.data.get("objects", [])
        self.places = self._drop_duplicate_ids(self.data.get("places", []), self.objects)

    @classmethod
    def _selected_path(cls) -> str:
        explicit_path = os.environ.get("ROBOT_SEMANTIC_MAP")
        if explicit_path:
            return cls._resolve_path(explicit_path)

        map_id = os.environ.get("ROBOT_MAP_ID")
        if map_id:
            path = cls._path_for_map_id(map_id)
            if path:
                return path

        return package_config_path("semantic_map.yaml")

    @staticmethod
    def _resolve_path(path: str) -> str:
        expanded = os.path.expanduser(os.path.expandvars(path))
        if os.path.isabs(expanded):
            return expanded
        return os.path.abspath(expanded)

    @classmethod
    def _path_for_map_id(cls, map_id: str) -> str | None:
        safe_id = map_id.strip()
        if not safe_id:
            return None
        bundle_ids = [safe_id]
        bundle_aliases = {
            "real_lab": "sim_real_lab_001",
            "warehouse": "sim_factory_mid360_like_gpu_trueimu_011",
        }
        alias_bundle_id = bundle_aliases.get(safe_id)
        if alias_bundle_id and alias_bundle_id not in bundle_ids:
            bundle_ids.insert(0, alias_bundle_id)
        config_names = [
            f"{safe_id}_semantic_map.yaml",
            f"{safe_id}.semantic_map.yaml",
        ]
        aliases = {
            "warehouse": "warehouse_semantic_map.yaml",
            "sim_factory_mid360_like_gpu_trueimu_011": "warehouse_semantic_map.yaml",
            "sim_l402_seed_v0": "warehouse_semantic_map.yaml",
        }
        alias_name = aliases.get(safe_id)
        if alias_name:
            config_names.insert(0, alias_name)

        roots = [
            os.getcwd(),
            os.environ.get("ROBOT_WS", ""),
            os.environ.get("ROBOT_WORKSPACE", ""),
            cls._repo_root(),
        ]
        candidates: list[str] = []
        for root in roots:
            if not root:
                continue
            for bundle_id in bundle_ids:
                candidates.append(os.path.join(root, "maps", bundle_id, "semantic_map.yaml"))
            candidates.append(os.path.join(root, "maps", f"{safe_id}_semantic_map.yaml"))
            candidates.extend(os.path.join(root, "src", "robot_skills", "config", name) for name in config_names)
        for name in config_names:
            package_path = cls._optional_package_config_path(name)
            if package_path:
                candidates.append(package_path)

        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        return None

    @staticmethod
    def _repo_root() -> str:
        # .../src/robot_skills/robot_skills/semantic_map.py -> workspace root.
        # This is a source-tree fallback (harmless when running from an installed
        # share/ copy, since that path just won't contain a maps/ directory), kept
        # so map-bundle lookup works regardless of the process's cwd.
        return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

    @staticmethod
    def _optional_package_config_path(name: str) -> str | None:
        try:
            return package_config_path(name)
        except Exception:
            return None

    @staticmethod
    def _drop_duplicate_ids(places: list[dict[str, Any]], objects: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """A place entry (often an uncalibrated placeholder) can be superseded by a
        later, fully-calibrated entry with the same id under objects:. Without this,
        find_matches sees the same target twice and reports it as falsely ambiguous.
        """
        object_ids = {item.get("id") for item in objects if item.get("id")}
        return [place for place in places if place.get("id") not in object_ids]

    @staticmethod
    def _load(path: str) -> dict[str, Any]:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def list_known_targets(self) -> list[dict[str, Any]]:
        targets = []
        for item in self.objects + self.places:
            targets.append(
                {
                    "id": item.get("id"),
                    "names": item.get("names", []),
                    "category": item.get("category", "place"),
                }
            )
        return targets

    def find_matches(self, name: str) -> list[dict[str, Any]]:
        normalized = self._normalize(name)
        query_tokens = self._tokenize(name)
        exact: list[dict[str, Any]] = []
        partial: list[dict[str, Any]] = []
        for item in self.objects + self.places:
            names = [item.get("id", "")] + item.get("names", [])
            if any(self._normalize(alias) == normalized for alias in names):
                exact.append(item)
            elif any(self._token_phrase_contains(self._tokenize(alias), query_tokens) for alias in names):
                partial.append(item)
        if self._is_ambiguous_single_shelf_label(normalized):
            combined = exact[:]
            for item in partial + self._find_shelf_number_matches(normalized):
                if item not in combined and str(item.get("category", "")).startswith("shelf"):
                    combined.append(item)
            if len(combined) > 1:
                return combined
        return exact if exact else partial

    def resolve(self, name: str, relation: str = "near") -> SemanticTarget | None:
        candidates = self.find_matches(name)

        if not candidates:
            return None

        item = candidates[0]
        pose_data, approach_id, approach_source = self._select_pose(item, relation)
        if pose_data is None:
            return None
        pose = Pose2D(
            x=float(pose_data.get("x", 0.0)),
            y=float(pose_data.get("y", 0.0)),
            yaw=float(pose_data.get("yaw", 0.0)),
            frame_id=pose_data.get("frame_id", self.frame_id),
        )
        alternatives = [candidate.get("id", "") for candidate in candidates[1:]]
        return SemanticTarget(
            object_id=item.get("id", ""),
            canonical_name=(item.get("names") or [item.get("id", "")])[0],
            category=item.get("category", "place"),
            pose=pose,
            approach_pose_id=approach_id,
            approach_source=approach_source,
            confidence=float(item.get("confidence", 1.0)),
            needs_clarification=len(candidates) > 1,
            alternatives=alternatives,
            region=item.get("map_region", {}),
            approach_policy=item.get("approach_policy", {}),
        )

    def approach_candidates(self, name: str, relation: str = "near") -> list[dict[str, Any]]:
        matches = self.find_matches(name)
        if not matches:
            return []
        item = matches[0]
        region_candidates = self._generate_pose_candidates_from_region(item, relation)
        if region_candidates:
            return region_candidates
        approaches = item.get("approach_poses", [])
        if approaches:
            return [
                {
                    "id": str(approach.get("id", f"configured_{index}")),
                    "pose": approach.get("pose", approach),
                    "source": "configured_approach_pose",
                }
                for index, approach in enumerate(approaches)
            ]
        if "pose" in item:
            return [{"id": "configured_pose", "pose": item["pose"], "source": "configured_pose"}]
        return []

    def _select_pose(self, item: dict[str, Any], relation: str) -> tuple[dict[str, Any] | None, str | None, str]:
        region_pose = self._generate_pose_from_region(item, relation)
        if region_pose is not None:
            return region_pose, f"generated_{relation}", "generated_from_region"
        if item.get("navigation_ready") is False:
            return None, None, "not_navigation_ready"
        approaches = item.get("approach_poses", [])
        if approaches:
            for approach in approaches:
                if approach.get("id") == relation:
                    return approach.get("pose", approach), approach.get("id"), "configured_approach_pose"
            return approaches[0].get("pose", approaches[0]), approaches[0].get("id"), "configured_approach_pose"
        if "pose" in item:
            return item["pose"], None, "configured_pose"
        return None, None, "missing_pose"

    def _generate_pose_from_region(self, item: dict[str, Any], relation: str) -> dict[str, Any] | None:
        candidates = self._generate_pose_candidates_from_region(item, relation)
        if candidates:
            return candidates[0]["pose"]
        return None

    def _generate_pose_candidates_from_region(self, item: dict[str, Any], relation: str) -> list[dict[str, Any]]:
        region = item.get("map_region") or {}
        polygon = region.get("polygon") or []
        if len(polygon) < 3:
            return []

        points = [(float(point["x"]), float(point["y"])) for point in polygon]
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        policy = item.get("approach_policy") or {}
        region_center_x = sum(xs) / len(xs)
        region_center_y = sum(ys) / len(ys)
        anchor_x, anchor_y = self._region_anchor(points, policy)
        distance = float(policy.get("distance_m", item.get("safety_zone_m", 0.5)))
        preferred_side = policy.get("preferred_side") or region.get("front_side") or "west"
        sides = [preferred_side] + [side for side in ["west", "east", "south", "north"] if side != preferred_side]
        candidates: list[dict[str, Any]] = []

        for side in sides:
            if side == "east":
                x, y = max_x + distance, anchor_y
            elif side == "north":
                x, y = anchor_x, max_y + distance
            elif side == "south":
                x, y = anchor_x, min_y - distance
            else:
                x, y = min_x - distance, anchor_y

            yaw_mode = policy.get("yaw_mode", "face_region_center")
            if policy.get("face_region_center", True) and yaw_mode == "face_region_center":
                yaw = math.atan2(region_center_y - y, region_center_x - x)
            elif yaw_mode == "face_anchor":
                yaw = math.atan2(anchor_y - y, anchor_x - x)
            else:
                yaw = float(policy.get("yaw", 0.0))
            yaw += math.radians(float(policy.get("yaw_offset_deg", 0.0)))
            candidates.append(
                {
                    "id": f"generated_{side}",
                    "side": side,
                    "preferred": side == preferred_side,
                    "source": "generated_from_region",
                    "pose": {"x": x, "y": y, "yaw": yaw, "frame_id": self.frame_id},
                }
            )

        return candidates

    @staticmethod
    def _region_anchor(points: list[tuple[float, float]], policy: dict[str, Any]) -> tuple[float, float]:
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        center_x = sum(xs) / len(xs)
        center_y = sum(ys) / len(ys)

        if policy.get("anchor_mode") != "label_end":
            return center_x, center_y

        offset = max(0.0, float(policy.get("label_end_offset_m", 1.0)))
        label_end = str(policy.get("label_end", "")).lower()
        if label_end == "north":
            return center_x, max_y + offset
        if label_end == "south":
            return center_x, min_y - offset
        if label_end == "east":
            return max_x + offset, center_y
        if label_end == "west":
            return min_x - offset, center_y
        return center_x, center_y

    @staticmethod
    def _normalize(text: str) -> str:
        return text.strip().lower().replace(" ", "").replace("_", "").replace("-", "")

    @staticmethod
    def _tokenize(text: str) -> list[str]:
        return [token for token in re.split(r"[\s_\-]+", text.strip().lower()) if token]

    @staticmethod
    def _token_phrase_contains(alias_tokens: list[str], query_tokens: list[str]) -> bool:
        """Whole-word containment check: query_tokens must appear as a contiguous
        run inside alias_tokens. Plain substring matching on joined strings would
        let a short query like "love" falsely match an alias like "loveseat".
        """
        if not query_tokens or not alias_tokens or len(query_tokens) > len(alias_tokens):
            return False
        return any(
            alias_tokens[i : i + len(query_tokens)] == query_tokens
            for i in range(len(alias_tokens) - len(query_tokens) + 1)
        )

    def _find_shelf_number_matches(self, normalized: str) -> list[dict[str, Any]]:
        number = self._single_shelf_number(normalized)
        if number is None:
            return []
        matches = []
        for item in self.objects:
            if not str(item.get("category", "")).startswith("shelf"):
                continue
            names = [item.get("id", "")] + item.get("names", [])
            for alias in names:
                numbers = re.findall(r"(?<!\d)0?([1-9])(?:棚|shelf)?(?!\d)", self._normalize(str(alias)))
                if number in {int(value) for value in numbers}:
                    matches.append(item)
                    break
        return matches

    @staticmethod
    def _is_ambiguous_single_shelf_label(normalized: str) -> bool:
        # Single shelf numbers sit on shelf/aisle boundaries in the current sim
        # map. If the semantic map has multiple number-bearing shelf regions,
        # the agent must ask the user instead of silently choosing one side.
        number = SemanticMap._single_shelf_number(normalized)
        return number in {1, 2, 3, 4, 5}

    @staticmethod
    def _single_shelf_number(normalized: str) -> int | None:
        compact = normalized.replace("棚", "").replace("shelf", "")
        match = re.fullmatch(r"0?([1-9])", compact)
        return int(match.group(1)) if match else None
