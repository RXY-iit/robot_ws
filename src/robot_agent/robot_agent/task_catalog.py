"""Canonical task vocabulary and the explicit legacy compatibility boundary."""
from copy import deepcopy
from pathlib import Path
import re

import yaml

from robot_agent.schemas import MissionIntent


class TaskCatalog:
    def __init__(self, path=None):
        if path is None:
            source = Path(__file__).resolve().parents[1] / "config/task_catalog.yaml"
            if source.exists():
                path = source
            else:
                from ament_index_python.packages import get_package_share_directory
                path = Path(get_package_share_directory("robot_agent")) / "config/task_catalog.yaml"
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        self.tasks = data["tasks"]
        self.aliases = data.get("aliases", {})
        for name, spec in self.tasks.items():
            if spec["owner"] not in {"mission", "navigation", "perception", "arm"}:
                raise ValueError(f"Unknown owner for {name}")
        if set(self.aliases.values()) - set(self.tasks):
            raise ValueError("Task alias refers to an unknown task")

    def normalize(self, intent):
        intent = deepcopy(intent)
        old = intent.task_type
        if old == "sequence":
            intent.constraints["steps"] = [self.normalize(MissionIntent(**x)).to_dict()
                                           for x in intent.constraints.get("steps", [])]
            return intent
        # Legacy compound intents become separate task contracts, not skill lists.
        compounds = {"search_object_near_place": "search_object",
                     "inspect_place": "search_object",
                     "touch_object_near_place": "interact_with_object",
                     "press_object_near_place": "interact_with_object",
                     "grasp_object_near_place": "manipulate_object"}
        if old in compounds:
            nav = MissionIntent("navigate_to", target=intent.target, relation=intent.relation,
                                constraints={"visual_confirmation": False}, raw_text=intent.raw_text)
            child = MissionIntent(compounds[old], object_query=intent.object_query,
                                  constraints=deepcopy(intent.constraints), raw_text=intent.raw_text)
            child.constraints["dependency_policy"] = "required_condition"
            if old == "inspect_place":
                child.object_query = intent.target
                child.constraints["goal_mode"] = "verify"
            if old == "press_object_near_place":
                child.constraints["operation"] = "press"
            return MissionIntent("sequence", constraints={"steps": [nav.to_dict(), self.normalize(child).to_dict()]},
                                 raw_text=intent.raw_text)
        intent.task_type = self.aliases.get(old, old)
        if old == "move_relative_xy":
            intent.constraints["translation_mode"] = "xy"
        if old in {"look_direction", "observe_scene"}:
            intent.constraints.setdefault("profile", "directional")
            if re.search(r"danger|hazard|unsafe|describe|anything wrong|危険|危ない|危险|描述", intent.raw_text, re.I):
                intent.constraints["analysis_kind"] = "image_describe"
                intent.object_query = intent.object_query or intent.raw_text
        if old == "press_object":
            intent.constraints["operation"] = "press"
        if intent.task_type == "interact_with_object":
            intent.constraints.setdefault("operation", "touch")
        if old in {"verify_target", "place_object", "release_object"}:
            key, value = {"verify_target": ("goal_mode", "verify"),
                          "place_object": ("desired_end_state", "placed_at"),
                          "release_object": ("desired_end_state", "released")}[old]
            intent.constraints[key] = value
        if intent.task_type == "manipulate_object":
            intent.constraints.setdefault("desired_end_state", "held")
        if intent.task_type == "search_object":
            intent.constraints.setdefault("goal_mode", "find")
            if str(intent.object_query or "").lower() in {"around", "surroundings", "周り", "周囲"}:
                intent.task_type = "observe_scene"
                intent.constraints["profile"] = "surroundings"
                intent.object_query = None
        return intent

    def legacy(self, intent):
        intent = self.normalize(intent)
        if intent.task_type not in self.tasks:
            return intent
        kind = intent.task_type
        intent.task_type = self.tasks[kind]["legacy"]
        if kind == "move_relative" and (intent.constraints.get("translation_mode") == "xy"
                                       or "dx_m" in intent.constraints or "dy_m" in intent.constraints):
            intent.task_type = "move_relative_xy"
        if kind == "interact_with_object" and intent.constraints.get("operation") == "press":
            intent.task_type = "press_object"
        if kind == "observe_scene" and intent.constraints.get("profile") == "surroundings":
            intent.task_type, intent.object_query = "search_object_here", "around"
        return intent

    def flatten(self, intent):
        intent = self.normalize(intent)
        if intent.task_type != "sequence":
            return [intent]
        return [child for item in intent.constraints.get("steps", [])
                for child in self.flatten(MissionIntent(**item))]
