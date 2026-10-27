"""One extension manifest shared by registration, catalog, and dispatch."""
from copy import deepcopy
from pathlib import Path

import yaml


def load_extensions():
    source = Path(__file__).resolve().parents[1] / "config/skill_extensions.yaml"
    if not source.exists():
        from robot_skills.registry import package_config_path
        source = Path(package_config_path("skill_extensions.yaml"))
    return yaml.safe_load(source.read_text(encoding="utf-8"))


def extend_definitions(definitions, *, catalog=False, supervision_profiles=None):
    spec = load_extensions()
    for name, alias in spec["aliases"].items():
        if name in definitions:
            raise ValueError(f"Duplicate skill definition: {name}")
        definitions[name] = {**deepcopy(definitions[alias["target"]]), "name": name,
                             "alias_of": alias["target"]}
        if catalog:
            definitions[name].setdefault("jev", {})["expose_to_command_parse"] = False
    for name in spec["checks"]:
        item = {"name": name, "group": "capability", "allowed_modes": ["dry_run", "sim", "real"],
                "sim_supported": True, "real_supported": True, "description": "Read-only domain capability snapshot"}
        if catalog:
            item["supervision"] = deepcopy((supervision_profiles or {}).get("capability", {}))
            item.update(args_schema={"type": "object", "properties": {
                "operation": {"type": "string"}, "query": {"type": "string"}, "provider": {"type": "string"}}},
                        preconditions={"required": [], "irrelevant": []}, effects={"motion": False})
        definitions[name] = item
    for name, entry in spec.get("composites", {}).items():
        definitions[name] = {**deepcopy(definitions[entry["contract"]]), "name": name,
                             "description": "Bounded stationary observation composite"}
        if catalog:
            definitions[name]["args_schema"] = {"type": "object", "properties": {
                "profile": {"type": "string"}, "query": {"type": "string"},
                "checks": {"type": "array"}, "unresolved_requirements": {"type": "array"},
                "aggregation": {"type": "enum", "values": ["all", "any"]}, "scan_band": {"type": "string"},
                "provider": {"type": "string"}, "pan_deg": {"type": "number"}, "tilt_deg": {"type": "number"}}}
            definitions[name].setdefault("jev", {})["expose_to_command_parse"] = False
            definitions[name]["preconditions"]["required"] = ["robot_stationary", "camera_ready"]
            if name == "search_at_place":
                capture_contract = definitions["capture_observation"].get("supervision", {})
                supervision = definitions[name].setdefault("supervision", {})
                supervision.setdefault("questions", {}).update(capture_contract.get("questions", {}))
                supervision["evidence_fields"] = sorted(set(supervision.get("evidence_fields", [])) |
                                                        set(capture_contract.get("evidence_fields", [])))
    return definitions


def resolve_alias(spec, name, payload):
    alias = spec["aliases"][name]
    payload = {**payload, **alias.get("defaults", {})}
    target = alias["target"]
    if name == "move_base_relative" and ("dx_m" in payload or "dy_m" in payload):
        target = "move_relative_xy"
    if name == "set_arm_pose":
        payload["name"] = payload.get("pose_label", payload.get("name", ""))
    return target, payload
