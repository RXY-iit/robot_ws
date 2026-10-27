from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml
from .launch_binding import quickstart_context


RISK_LEVELS = ("A0", "A1", "A2", "A3", "A4")


def risk_rank(risk: str) -> int:
    try:
        return RISK_LEVELS.index(str(risk).upper())
    except ValueError as exc:
        raise ValueError(f"Unknown risk level {risk!r}; expected one of {RISK_LEVELS}") from exc


@dataclass
class ItemNode:
    """One health item in the env dependency graph."""

    id: str
    layer: str
    depends_on: list[str] = field(default_factory=list)
    producer: str | None = None


@dataclass
class Component:
    """A software/hardware unit a probe or action can address by name."""

    id: str
    ros_node: str | None = None
    ros_nodes: list[str] = field(default_factory=list)
    process_pattern: str | None = None
    topics: list[str] = field(default_factory=list)
    note: str = ""
    # {kill_pattern, command: [...], settle_sec}; absent = cannot be restarted
    # automatically (the executor falls back to asking the operator).
    restart: dict[str, Any] | None = None
    restart_risk: str = "A1"
    restart_preconditions: list[str] = field(default_factory=list)


@dataclass
class LaunchProfile:
    """What a given launcher mode actually starts.

    Items produced by an absent component (or downstream of one) are not
    faults in that mode, e.g. sim arm_test starts no LiDAR / GICP / Nav2.
    """

    name: str
    capabilities: list[str]
    absent_components: list[str] = field(default_factory=list)
    note: str = ""


@dataclass
class EnvProfile:
    """Env binding: everything that differs between sim and real.

    Playbooks only use symbolic names (items, components, devices, named
    preconditions); this profile maps them to the concrete env.
    """

    kind: str
    layers: list[str]
    items: dict[str, ItemNode]
    components: dict[str, Component]
    devices: dict[str, dict[str, Any]]
    preconditions: dict[str, dict[str, Any]]
    approval: dict[str, str]
    safety_items: set[str]
    ignored_items: set[str]
    # Published only while something happens (e.g. /cmd_vel_raw while
    # commanding): STALE / UNKNOWN means idle, only ERROR / BLOCKED count.
    event_driven_items: set[str]
    debounce_sec: float
    # An incident closes only after its root has been healthy this long.
    resolve_after_sec: float
    launch_profiles: dict[str, LaunchProfile]
    # Backend settings for action primitives (service names, tool paths).
    actions: dict[str, Any] = field(default_factory=dict)
    # Real: only `verified` playbooks may execute; others are suggestions.
    require_verified: bool = False
    capability_aliases: dict[str, list[str]] = field(default_factory=dict)
    component_aliases: dict[str, str] = field(default_factory=dict)
    startup_units: list[dict[str, Any]] = field(default_factory=list)

    def normalize_capabilities(self, names: list[str]) -> list[str]:
        out: list[str] = []
        for name in names:
            for cap in self.capability_aliases.get(name, [name]):
                if cap not in out:
                    out.append(cap)
        return out

    def normalize_components(self, names: list[str]) -> list[str]:
        return [self.component_aliases.get(name, name) for name in names]

    def layer_index(self, item_id: str) -> int:
        node = self.items.get(item_id)
        if node is None or node.layer not in self.layers:
            return len(self.layers)
        return self.layers.index(node.layer)

    def layer_of(self, item_id: str) -> str:
        node = self.items.get(item_id)
        return node.layer if node is not None else "unmapped"

    def upstream(self, item_id: str) -> list[str]:
        node = self.items.get(item_id)
        return list(node.depends_on) if node is not None else []

    def producer_of(self, item_id: str) -> str | None:
        node = self.items.get(item_id)
        return node.producer if node is not None else None

    def items_of_component(self, component_id: str) -> list[str]:
        return [item_id for item_id, node in self.items.items() if node.producer == component_id]

    def items_not_launched(self, profile: LaunchProfile | None) -> set[str]:
        """Items produced by an absent component, or depending on such an item."""
        if profile is None or not profile.absent_components:
            return set()
        absent = set(profile.absent_components)
        result: set[str] = set()

        def not_launched(item_id: str, trail: frozenset[str]) -> bool:
            if item_id in result:
                return True
            node = self.items.get(item_id)
            if node is None or item_id in trail:
                return False
            if node.producer in absent or any(
                not_launched(dep, trail | {item_id}) for dep in node.depends_on
            ):
                result.add(item_id)
                return True
            return False

        for item_id in self.items:
            not_launched(item_id, frozenset())
        return result

    def approval_for(self, risk: str) -> str:
        return self.approval.get(str(risk).upper(), "forbidden")

    def action_approval(self, risk: str, primitive: str | None) -> str:
        rule = self.approval_for(risk)
        if primitive == "cancel_motion" and rule == "confirm":
            return "auto"  # protective stop must not wait for restart approval
        if primitive == "restart_component" and self.actions.get("restart_requires_approval") and rule == "auto":
            return "confirm"
        return rule


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping")
    return data


def load_env_profile(config_dir: Path, kind: str) -> EnvProfile:
    common = _load_yaml(config_dir / "recovery.yaml")
    env_data = _load_yaml(config_dir / "envs" / f"{kind}.yaml")
    if env_data.get("kind") != kind:
        raise ValueError(f"envs/{kind}.yaml declares kind={env_data.get('kind')!r}")

    layers = [str(layer) for layer in common.get("layers", [])]
    items: dict[str, ItemNode] = {}
    for item_id, spec in (env_data.get("items") or {}).items():
        spec = spec or {}
        layer = str(spec.get("layer", "unmapped"))
        if layer not in layers:
            raise ValueError(f"envs/{kind}.yaml item {item_id}: unknown layer {layer!r}")
        items[str(item_id)] = ItemNode(
            id=str(item_id),
            layer=layer,
            depends_on=[str(dep) for dep in spec.get("depends_on", []) or []],
            producer=spec.get("producer"),
        )
    for node in items.values():
        missing = [dep for dep in node.depends_on if dep not in items]
        if missing:
            raise ValueError(f"envs/{kind}.yaml item {node.id}: depends_on unknown items {missing}")

    components = {
        str(comp_id): Component(
            id=str(comp_id),
            ros_node=(spec or {}).get("ros_node"),
            ros_nodes=list((spec or {}).get("ros_nodes") or []),
            process_pattern=(spec or {}).get("process_pattern"),
            topics=list((spec or {}).get("topics", []) or []),
            note=str((spec or {}).get("note", "")),
            restart=(spec or {}).get("restart"),
            restart_risk=str((spec or {}).get("restart_risk", "A1")),
            restart_preconditions=list((spec or {}).get("restart_preconditions") or []),
        )
        for comp_id, spec in (env_data.get("components") or {}).items()
    }
    for comp in components.values():
        risk_rank(comp.restart_risk)
        if any(name not in (env_data.get("preconditions") or {}) for name in comp.restart_preconditions):
            raise ValueError(f"{comp.id}: unknown restart precondition")
    launch_context = quickstart_context() if kind == "sim" else {}
    if "motion_policy" in launch_context and "mode_switch" in components:
        restart = components["mode_switch"].restart
        if restart:
            restart["command"] = [
                "enable_motion_policy:=" + str(launch_context["motion_policy"]).lower()
                if str(part).startswith("enable_motion_policy:=") else part for part in restart["command"]]
    for node in items.values():
        if node.producer is not None and node.producer not in components:
            raise ValueError(f"envs/{kind}.yaml item {node.id}: unknown producer {node.producer!r}")

    approval = {
        str(risk).upper(): str(rule)
        for risk, rule in {**(common.get("approval_defaults") or {}), **(env_data.get("approval") or {})}.items()
    }
    for risk in approval:
        risk_rank(risk)

    launch_profiles = {}
    for name, spec in (env_data.get("launch_profiles") or {}).items():
        spec = spec or {}
        absent = [str(comp) for comp in spec.get("absent_components", []) or []]
        unknown = [comp for comp in absent if comp not in components]
        if unknown:
            raise ValueError(f"envs/{kind}.yaml launch_profiles.{name}: unknown components {unknown}")
        launch_profiles[str(name)] = LaunchProfile(
            name=str(name),
            capabilities=[str(cap) for cap in spec.get("capabilities", []) or []],
            absent_components=absent,
            note=str(spec.get("note", "")),
        )

    startup_units = list(env_data.get("startup_units") or [])
    ids = [unit.get("id") for unit in startup_units]
    if len(set(ids)) != len(ids) or any(not name for name in ids):
        raise ValueError("startup_units require unique non-empty ids")
    for unit in startup_units:
        if any(c not in components for c in unit.get("components", [])):
            raise ValueError(f"{unit['id']}: unknown startup component")
        if any(i not in items for i in unit.get("verify_items", [])):
            raise ValueError(f"{unit['id']}: unknown startup verification item")
        if any(d not in ids for d in unit.get("depends_on", [])):
            raise ValueError(f"{unit['id']}: unknown startup dependency")
        if unit.get("recovery") == "authorized_start" and any(
            not components[c].restart for c in unit.get("components", [])
        ):
            raise ValueError(f"{unit['id']}: authorized start has no command")

    return EnvProfile(
        kind=kind,
        layers=layers,
        items=items,
        components=components,
        devices={str(k): dict(v or {}) for k, v in (env_data.get("devices") or {}).items()},
        preconditions={str(k): dict(v or {}) for k, v in (env_data.get("preconditions") or {}).items()},
        approval=approval,
        safety_items={str(item) for item in env_data.get("safety_items", []) or []},
        ignored_items={str(item) for item in env_data.get("ignored_items", []) or []},
        event_driven_items={str(item) for item in env_data.get("event_driven_items", []) or []},
        debounce_sec=float(env_data.get("debounce_sec", common.get("debounce_sec", 3.0))),
        resolve_after_sec=float(env_data.get("resolve_after_sec", common.get("resolve_after_sec", 10.0))),
        launch_profiles=launch_profiles,
        actions={**dict(env_data.get("actions") or {}), "launch_context": launch_context},
        require_verified=bool(env_data.get("require_verified_for_execution", False)),
        capability_aliases={
            str(k): [str(c) for c in (v or [])] for k, v in (common.get("capability_aliases") or {}).items()
        },
        component_aliases={str(k): str(v) for k, v in (env_data.get("component_aliases") or {}).items()},
        startup_units=startup_units,
    )
