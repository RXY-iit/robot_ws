from __future__ import annotations

import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


def package_config_path(filename: str) -> str:
    return os.path.join(get_package_share_directory("robot_skills"), "config", filename)


class SkillRegistry:
    def __init__(self, path: str | None = None):
        self.path = path or package_config_path("skill_registry.yaml")
        self.data = self._load(self.path)
        self.skills = {item["name"]: item for item in self.data.get("skills", [])}

    @staticmethod
    def _load(path: str) -> dict[str, Any]:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def get(self, name: str) -> dict[str, Any] | None:
        return self.skills.get(name)

    def list_names(self) -> list[str]:
        return sorted(self.skills)
