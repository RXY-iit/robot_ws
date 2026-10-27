from __future__ import annotations

import os
from typing import Any

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ModuleNotFoundError:  # Allows offline tests/tools without sourcing ROS.
    get_package_share_directory = None


def package_config_path(filename: str) -> str:
    if get_package_share_directory is None:
        return os.path.join(os.path.dirname(os.path.dirname(__file__)), "config", filename)
    return os.path.join(get_package_share_directory("robot_skills"), "config", filename)


class SkillRegistry:
    def __init__(self, path: str | None = None):
        self.path = path or package_config_path("skill_registry.yaml")
        self.data = self._load(self.path)
        self.skills = {item["name"]: item for item in self.data.get("skills", [])}
        if path is None:
            from robot_skills.extensions import extend_definitions
            extend_definitions(self.skills)

    @staticmethod
    def _load(path: str) -> dict[str, Any]:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def get(self, name: str) -> dict[str, Any] | None:
        return self.skills.get(name)

    def list_names(self) -> list[str]:
        return sorted(self.skills)
