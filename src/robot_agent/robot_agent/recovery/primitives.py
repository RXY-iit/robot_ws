from __future__ import annotations

import math
import json
import os
import re
import subprocess
from pathlib import Path
from dataclasses import dataclass, field
from typing import Any, Callable, Protocol

from .environment import EnvProfile, risk_rank
from .snapshot import (
    ABNORMAL_CLASSES,
    DEGRADED,
    FAULT,
    HEALTHY,
    HealthSnapshot,
    StatusHistory,
    classify_status,
)


PASS = "PASS"
FAIL = "FAIL"
UNKNOWN = "UNKNOWN"


@dataclass
class ProbeResult:
    """PASS = the checked condition holds; FAIL = it does not; UNKNOWN = no evidence."""

    outcome: str
    detail: str
    value: Any = None

    def to_dict(self) -> dict[str, Any]:
        return {"outcome": self.outcome, "detail": self.detail, "value": self.value}


class GraphAdapter(Protocol):
    def node_names(self) -> set[str]: ...

    def publisher_count(self, topic: str) -> int: ...


class SystemAdapter(Protocol):
    def process_alive(self, pattern: str) -> bool: ...

    def path_exists(self, path: str) -> bool: ...


def _ros_domain(environ: dict[str, str]) -> str:
    return environ.get("ROS_DOMAIN_ID", "0") or "0"


def matches_node_process(argv: list[str], pattern: str) -> bool:
    """Match the executing program, not a launcher's embedded future commands."""
    if not argv:
        return False
    program = Path(argv[0]).name
    executable = argv[0]
    if program.startswith("python"):
        # ROS Python entry points are scripts. Inline/module commands and their
        # arguments are not evidence that the named ROS node is running.
        if len(argv) < 2 or argv[1].startswith("-"):
            return False
        executable = argv[1]
    if Path(executable).name in {"sh", "bash", "dash", "zsh", "fish", "ros2", "pgrep", "rg",
                                 "gnome-terminal", "gnome-terminal-server", "recovery_terminal_runner.py"}:
        return False
    if re.search(pattern, executable):
        return True
    # Composed nodes share a binary: configured node-name remaps distinguish
    # their containers/managers, but never match an arbitrary shell payload.
    return pattern.startswith("__node:=") and any(re.fullmatch(pattern, arg) for arg in argv[1:])


def same_domain_pids(pattern: str) -> list[int]:
    """PIDs matching `pattern` that run in this process's ROS_DOMAIN_ID.

    pkill -f is host-wide; two ROS domains on one host (e.g. a live sim and a
    test stack) must never kill each other's nodes.
    """
    result = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True, check=False)
    own_domain = _ros_domain(dict(os.environ))
    pids = []
    for line in result.stdout.split():
        pid = int(line)
        if pid == os.getpid():
            continue
        try:
            raw = Path(f"/proc/{pid}/environ").read_bytes()
            argv = Path(f"/proc/{pid}/cmdline").read_bytes().decode(errors="replace").rstrip("\0").split("\0")
        except OSError:
            continue  # gone, or not ours to inspect -> never touch it
        environ = dict(item.split("=", 1) for item in raw.decode(errors="replace").split("\0") if "=" in item)
        if _ros_domain(environ) == own_domain and matches_node_process(argv, pattern):
            pids.append(pid)
    return pids


class LocalSystemAdapter:
    def simulation_evidence(self, now: float) -> dict[str, Any]:
        # Unlike the kill helper, missing permissions are UNKNOWN, never proof
        # that hardware-equivalent Isaac is off. This is a local-host contract.
        domain = _ros_domain(dict(os.environ))
        try:
            result = subprocess.run(["pgrep", "-f", "isaac-sim"], capture_output=True, text=True, timeout=2)
            if result.returncode not in {0, 1}:
                return {"process_alive": None, "source": "process query failed"}
            alive, uncertain = [], False
            for token in result.stdout.split():
                pid = int(token)
                if pid == os.getpid():
                    continue
                try:
                    cmd = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
                    # Ignore editors, grep and shell commands merely mentioning Isaac.
                    if not cmd or not any(Path(part.decode(errors="replace")).name in {"kit", "isaac-sim.sh"} for part in cmd[:2]):
                        continue
                    raw = Path(f"/proc/{pid}/environ").read_bytes()
                    env = dict(v.split("=", 1) for v in raw.decode(errors="replace").split("\0") if "=" in v)
                    if _ros_domain(env) == domain:
                        alive.append(pid)
                    else:
                        uncertain = True  # domain mismatch is not evidence of hardware OFF
                except FileNotFoundError:
                    continue
                except OSError:
                    uncertain = True
            evidence: dict[str, Any] = {"process_alive": True if alive else None if uncertain else False,
                                        "pids": alive, "domain": domain, "source": "local /proc and timeline heartbeat"}
            records = []
            for pid in alive:
                try:
                    value = json.loads(Path(f"/tmp/robot-isaac-state-{os.getuid()}/{pid}.json").read_text())
                    if isinstance(value, dict) and value.get("pid") == pid and str(value.get("domain")) == domain:
                        age = now - float(value["monotonic"])
                        if (0 <= age <= 3 and isinstance(value.get("timeline"), str)
                                and value["timeline"] in {"PLAYING", "PAUSED", "STOPPED"}):
                            records.append((value.get("timeline"), age))
                except (OSError, ValueError, KeyError, TypeError):
                    continue
            states = {state for state, _ in records}
            if len(states) == 1:
                evidence.update(timeline=records[0][0], timeline_age_sec=max(age for _, age in records))
            elif len(states) > 1:
                evidence["timeline_conflict"] = True
            return evidence
        except (OSError, ValueError, subprocess.TimeoutExpired):
            return {"process_alive": None, "source": "process observation unavailable"}

    def process_alive(self, pattern: str) -> bool:
        # Domain-scoped: a node of the same name in another ROS domain on this
        # host (e.g. a test stack) is not "our" process.
        return bool(same_domain_pids(pattern))

    def path_exists(self, path: str) -> bool:
        return os.path.exists(path)


@dataclass
class ProbeContext:
    env: EnvProfile
    snapshot: HealthSnapshot
    history: StatusHistory
    graph: GraphAdapter | None = None
    system: SystemAdapter | None = None
    # sim: devices that the fault injector has "unplugged". Empty = all present.
    virtual_devices_missing: set[str] = field(default_factory=set)
    launch_profile_name: str | None = None
    cache: dict[str, Any] = field(default_factory=dict)
    environment_ready: bool = True

    def node_names(self) -> set[str] | None:
        if self.graph is None:
            return None
        if "node_names" not in self.cache:
            self.cache["node_names"] = self.graph.node_names()
        return self.cache["node_names"]


ProbeFn = Callable[..., ProbeResult]


@dataclass
class PrimitiveSpec:
    name: str
    kind: str  # "probe" | "action"
    risk: str
    description: str
    fn: Callable[..., Any] | None = None
    envs: tuple[str, ...] = ("sim", "real")

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "kind": self.kind,
            "risk": self.risk,
            "description": self.description,
            "envs": list(self.envs),
            "implemented": self.fn is not None,
        }


class PrimitiveRegistry:
    """Small, reviewed set of probes/actions that playbooks compose.

    New failure types should normally need only a new playbook YAML; a new
    primitive is added only when no existing probe/action can express a check.
    """

    def __init__(self) -> None:
        self._specs: dict[str, PrimitiveSpec] = {}

    def probe(self, name: str, description: str) -> Callable[[ProbeFn], ProbeFn]:
        def register(fn: ProbeFn) -> ProbeFn:
            self._add(PrimitiveSpec(name, "probe", "A0", description, fn))
            return fn

        return register

    def declare_action(
        self,
        name: str,
        risk: str,
        description: str,
        *,
        envs: tuple[str, ...] = ("sim", "real"),
        fn: Callable[..., Any] | None = None,
    ) -> None:
        risk_rank(risk)
        self._add(PrimitiveSpec(name, "action", risk.upper(), description, fn, envs))

    def _add(self, spec: PrimitiveSpec) -> None:
        if spec.name in self._specs:
            raise ValueError(f"Primitive {spec.name!r} registered twice")
        self._specs[spec.name] = spec

    def get(self, name: str) -> PrimitiveSpec | None:
        return self._specs.get(name)

    def names(self, kind: str | None = None) -> list[str]:
        return sorted(name for name, spec in self._specs.items() if kind is None or spec.kind == kind)

    def catalog(self) -> list[dict[str, Any]]:
        return [self._specs[name].to_dict() for name in self.names()]

    def run_probe(self, name: str, ctx: ProbeContext, args: dict[str, Any] | None = None) -> ProbeResult:
        spec = self._specs.get(name)
        if spec is None or spec.kind != "probe" or spec.fn is None:
            return ProbeResult(UNKNOWN, f"Unknown probe {name!r}")
        try:
            return spec.fn(ctx, **(args or {}))
        except Exception as exc:  # A broken probe must not stop diagnosis.
            return ProbeResult(UNKNOWN, f"Probe {name} raised {exc!r}")

    def run_precondition(self, name: str, ctx: ProbeContext) -> ProbeResult:
        binding = ctx.env.preconditions.get(name)
        if binding is None:
            return ProbeResult(UNKNOWN, f"Precondition {name!r} has no binding in env {ctx.env.kind}")
        return self.run_probe(str(binding.get("probe")), ctx, binding.get("args") or {})


def build_default_registry() -> PrimitiveRegistry:
    registry = PrimitiveRegistry()

    @registry.probe("fresh_observation", "Measurement delivery is fresh; payload may legitimately report a fault")
    def fresh_observation(ctx: ProbeContext, item: str, max_age_sec: float = 5.0) -> ProbeResult:
        entry = ctx.snapshot.item(item) or {}
        age = entry.get("receive_age_sec")
        valid = isinstance(age, (float, int)) and math.isfinite(age) and 0 <= age <= max_age_sec
        return ProbeResult(PASS if valid else UNKNOWN, f"{item}: receive age={age}, limit={max_age_sec}s", age)

    @registry.probe("evidence_source", "Require an explicit provenance tag; never infer real hardware from a sim proxy")
    def evidence_source(ctx: ProbeContext, item: str, evidence: str) -> ProbeResult:
        entry = ctx.snapshot.item(item) or {}
        actual = entry.get("evidence")
        return ProbeResult(PASS if actual == evidence else UNKNOWN, f"{item}: evidence={actual!r}, required={evidence}")

    @registry.probe("sim_clock_advancing", "Fresh clock delivery, advancing simulation time and measurable RTF")
    def sim_clock_advancing(ctx: ProbeContext) -> ProbeResult:
        value = (ctx.snapshot.item("sim_rtf") or {}).get("value") or {}
        ages = [value.get("clock_receive_age_sec"), value.get("not_advancing_sec")]
        rtf = value.get("rtf")
        valid = all(isinstance(v, (int, float)) and math.isfinite(v) and 0 <= v <= 2 for v in ages)
        valid = valid and isinstance(rtf, (int, float)) and math.isfinite(rtf) and rtf > 0
        return ProbeResult(PASS if valid else UNKNOWN, "Fresh advancing clock with numeric RTF" if valid
                           else "Clock stopped, unavailable, or RTF not measurable", value)

    @registry.probe("health_item", "Health item is healthy, or in one of status_in / class_in")
    def health_item(
        ctx: ProbeContext,
        item: str,
        status_in: list[str] | None = None,
        class_in: list[str] | None = None,
    ) -> ProbeResult:
        entry = ctx.snapshot.item(item)
        if entry is None:
            return ProbeResult(UNKNOWN, f"{item} is not in the health snapshot")
        status = str(entry.get("status", "UNKNOWN"))
        health_class = classify_status(status)
        detail = f"{item}={status}: {entry.get('summary', '')}"
        if status_in is not None:
            return ProbeResult(PASS if status in status_in else FAIL, detail, status)
        if class_in is not None:
            return ProbeResult(PASS if health_class in class_in else FAIL, detail, status)
        if ctx.env.kind == "sim" and status == "WARN" and entry.get("warning_kind") == "low_rate":
            return ProbeResult(PASS, detail, status)
        if health_class == HEALTHY:
            return ProbeResult(PASS, detail, status)
        if health_class in (FAULT, DEGRADED):
            return ProbeResult(FAIL, detail, status)
        # UNCERTAIN / NEUTRAL: no evidence either way (unknown_is_not_ok).
        return ProbeResult(UNKNOWN, detail, status)

    @registry.probe("node_alive", "Component's ROS node is present in the ROS graph")
    def node_alive(ctx: ProbeContext, component: str) -> ProbeResult:
        comp = ctx.env.components.get(component)
        if comp is None or not (comp.ros_node or comp.ros_nodes):
            return ProbeResult(UNKNOWN, f"Component {component!r} has no ros_node in env {ctx.env.kind}")
        names = ctx.node_names()
        if names is None:
            return ProbeResult(UNKNOWN, "ROS graph adapter unavailable")
        expected = comp.ros_nodes or [comp.ros_node]
        if hasattr(ctx.graph, "node_counts"):
            if "node_counts" not in ctx.cache:
                ctx.cache["node_counts"] = ctx.graph.node_counts()
            duplicates = {name: ctx.cache["node_counts"].get(name, 0) for name in expected
                          if ctx.cache["node_counts"].get(name, 0) > 1}
            if duplicates:
                return ProbeResult(UNKNOWN, f"Duplicate ROS nodes: {duplicates}; no automatic group restart", duplicates)
        missing = [name for name in expected if name not in names]
        return ProbeResult(FAIL if missing else PASS,
                           f"Missing ROS nodes: {missing}" if missing else f"Unique expected ROS nodes present: {expected}",
                           {"expected": expected, "missing": missing})

    @registry.probe("process_alive", "Component's OS process is running (pgrep -f)")
    def process_alive(ctx: ProbeContext, component: str) -> ProbeResult:
        comp = ctx.env.components.get(component)
        if comp is None or not comp.process_pattern:
            return ProbeResult(UNKNOWN, f"Component {component!r} has no process_pattern in env {ctx.env.kind}")
        if ctx.system is None:
            return ProbeResult(UNKNOWN, "System adapter unavailable")
        alive = ctx.system.process_alive(comp.process_pattern)
        return ProbeResult(PASS if alive else FAIL, f"process /{comp.process_pattern}/ {'running' if alive else 'not running'}")

    @registry.probe("topic_has_publisher", "Topic (or component's first topic) has at least one publisher")
    def topic_has_publisher(ctx: ProbeContext, topic: str | None = None, component: str | None = None) -> ProbeResult:
        if topic is None and component is not None:
            comp = ctx.env.components.get(component)
            topic = comp.topics[0] if comp is not None and comp.topics else None
        if topic is None:
            return ProbeResult(UNKNOWN, "No topic to check")
        if ctx.graph is None:
            return ProbeResult(UNKNOWN, "ROS graph adapter unavailable")
        count = ctx.graph.publisher_count(topic)
        return ProbeResult(PASS if count > 0 else FAIL, f"{topic} has {count} publisher(s)", count)

    @registry.probe("device_present", "Named device exists (path in real, virtual table in sim)")
    def device_present(ctx: ProbeContext, device: str) -> ProbeResult:
        spec = ctx.env.devices.get(device)
        if spec is None:
            return ProbeResult(UNKNOWN, f"Device {device!r} not bound in env {ctx.env.kind}")
        if spec.get("kind") == "virtual":
            present = device not in ctx.virtual_devices_missing
            return ProbeResult(PASS if present else FAIL, f"virtual device {device} {'present' if present else 'missing'}")
        path = spec.get("path")
        if not path or ctx.system is None:
            return ProbeResult(UNKNOWN, f"Device {device!r} cannot be checked")
        present = ctx.system.path_exists(str(path))
        return ProbeResult(PASS if present else FAIL, f"{path} {'exists' if present else 'missing'}")

    @registry.probe("mode_is", "Robot mode item equals the given mode")
    def mode_is(ctx: ProbeContext, mode: str, item: str = "robot_mode") -> ProbeResult:
        entry = ctx.snapshot.item(item)
        if entry is None or entry.get("value") is None:
            return ProbeResult(UNKNOWN, f"{item} has no value")
        if classify_status(entry.get("status")) in ABNORMAL_CLASSES:
            return ProbeResult(UNKNOWN, f"{item} is {entry.get('status')}; value not trusted")
        value = str(entry.get("value")).strip().upper()
        return ProbeResult(PASS if value == mode.upper() else FAIL, f"{item}={value}", value)

    @registry.probe("twist_zero", "Fresh Twist item has all components below tolerance")
    def twist_zero(ctx: ProbeContext, item: str = "cmd_vel", tolerance: float = 1e-3, max_age_sec: float = 1.0) -> ProbeResult:
        entry = ctx.snapshot.item(item)
        if entry is None:
            return ProbeResult(UNKNOWN, f"{item} is not in the health snapshot")
        age = entry.get("receive_age_sec")
        value = entry.get("value")
        if age is None or float(age) > max_age_sec or not isinstance(value, dict):
            return ProbeResult(UNKNOWN, f"{item} not fresh (age={age})")
        numbers = [float(v) for v in value.values() if isinstance(v, (int, float))]
        if not numbers or any(not math.isfinite(v) for v in numbers):
            return ProbeResult(UNKNOWN, f"{item} has no finite values")
        zero = all(abs(v) <= tolerance for v in numbers)
        return ProbeResult(PASS if zero else FAIL, f"{item}={value}", value)

    @registry.probe("text_matches", "Item value (or summary) matches a regular expression")
    def text_matches(ctx: ProbeContext, item: str, pattern: str, field: str = "value") -> ProbeResult:
        entry = ctx.snapshot.item(item)
        value = None if entry is None else entry.get(field)
        if value is None:
            return ProbeResult(UNKNOWN, f"{item}.{field} has no value")
        matched = re.search(pattern, str(value)) is not None
        return ProbeResult(PASS if matched else FAIL, f"{item}.{field}={value!s:.100}", value)

    @registry.probe("never_observed", "Item has never received a single message since health_monitor started")
    def never_observed(ctx: ProbeContext, item: str) -> ProbeResult:
        entry = ctx.snapshot.item(item)
        if entry is None:
            return ProbeResult(UNKNOWN, f"{item} is not in the health snapshot")
        count = entry.get("message_count")
        never = (count == 0) if count is not None else (entry.get("receive_age_sec") is None and entry.get("value") is None)
        return ProbeResult(PASS if never else FAIL, f"{item} message_count={count} age={entry.get('receive_age_sec')}")

    @registry.probe("launch_profile_set", "A launcher mode (launch profile) was given to the recovery agent")
    def launch_profile_set(ctx: ProbeContext) -> ProbeResult:
        name = ctx.launch_profile_name
        return ProbeResult(PASS if name else FAIL, f"launch_profile={name or 'none'}", name)

    # Actions are declared with risk and env support so playbooks can be
    # validated and dry-run planned. v1 is read-only: no action is executed.
    registry.declare_action("restart_component", "A1", "Restart a software component (A2 when it drives actuators; set per playbook step)")
    registry.declare_action("lifecycle_reset", "A1", "Reset/activate a lifecycle node via its lifecycle manager")
    registry.declare_action("clear_costmap", "A1", "Clear Nav2 local/global costmap")
    registry.declare_action("cancel_motion", "A1", "Cancel active Nav2 goal / motion skill")
    registry.declare_action("publish_initialpose", "A1", "Publish a localization seed from a validated trusted pose or configured alignment tool")
    registry.declare_action("drive_alarm_reset", "A2", "Reset BLVD present alarm over Modbus", envs=("real",))
    registry.declare_action("dynamixel_reboot", "A2", "Send DYNAMIXEL Reboot instruction to one steering motor", envs=("real",))
    registry.declare_action("reduce_sim_load", "A1", "Reduce Isaac Sim load (disable non-essential sensors)", envs=("sim",))
    registry.declare_action("notify_mission", "A0", "Send capability_lost / capability_restored to the mission agent")
    return registry
