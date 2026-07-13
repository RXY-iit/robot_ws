from __future__ import annotations

from robot_skills import result_codes as codes
from robot_skills.executors.base import make_result, now
from robot_skills.semantic_map import SemanticMap
from robot_skills.schemas import SkillContext, SkillResult


class SemanticSkills:
    def __init__(self, semantic_map: SemanticMap | None = None):
        self.semantic_map = semantic_map or SemanticMap()

    def resolve_semantic_target(
        self,
        context: SkillContext,
        target_name: str,
        relation: str = "near",
    ) -> SkillResult:
        started = now()
        target = self.semantic_map.resolve(target_name, relation)
        if target is None:
            matches = self.semantic_map.find_matches(target_name)
            if matches:
                item = matches[0]
                region = item.get("map_region") or {}
                polygon = region.get("polygon") or []
                if region and len(polygon) < 3:
                    detail = (
                        "It has a semantic region entry, but map_region.polygon needs at least "
                        "3 map-frame points before an approach pose can be generated."
                    )
                else:
                    detail = "It has no calibrated map-frame approach pose yet."
                return make_result(
                    "resolve_semantic_target",
                    False,
                    codes.NEEDS_HUMAN,
                    (
                        f'Known semantic target "{target_name}" exists as {item.get("id")}, '
                        f"but it is not navigation-ready. {detail}"
                    ),
                    started,
                    {
                        "target_name": target_name,
                        "relation": relation,
                        "object_id": item.get("id"),
                        "segment_id": item.get("segment_id"),
                        "navigation_ready": item.get("navigation_ready", False),
                        "map_region": region,
                        "required_input": [
                            "map-frame polygon points around the shelf body",
                            "preferred approach side: west/east/north/south",
                            "desired approach distance in meters",
                        ],
                    },
                )
            return make_result(
                "resolve_semantic_target",
                False,
                codes.NEEDS_HUMAN,
                f'Unknown semantic target: "{target_name}".',
                started,
                {"target_name": target_name, "relation": relation},
            )
        if target.needs_clarification:
            return make_result(
                "resolve_semantic_target",
                False,
                codes.NEEDS_HUMAN,
                (
                    f'Ambiguous semantic target "{target_name}". '
                    f"Candidates include {target.object_id} and {', '.join(target.alternatives)}."
                ),
                started,
                target.to_dict(),
            )
        metrics = target.to_dict()
        metrics["approach_candidates"] = self.semantic_map.approach_candidates(target_name, relation)
        return make_result(
            "resolve_semantic_target",
            True,
            codes.OK,
            f'Resolved "{target_name}" to {target.object_id}.',
            started,
            metrics,
        )

    def list_known_targets(self, context: SkillContext) -> SkillResult:
        started = now()
        targets = self.semantic_map.list_known_targets()
        return make_result(
            "list_known_targets",
            True,
            codes.OK,
            f"Known targets: {len(targets)}.",
            started,
            {"targets": targets},
        )
