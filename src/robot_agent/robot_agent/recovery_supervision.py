"""Capability constraints from the independent Recovery Agent, shared by ROS/worker threads."""
from copy import deepcopy
import threading
import time


ALIASES = {"localization": {"auto_navigation"}, "navigation": {"auto_navigation"},
           "perception": {"stationary_perception"}, "arm": {"so101_arm"}}


class RecoverySupervision:
    def __init__(self):
        self.condition = threading.Condition()
        self._constraints = []
        self._received_at = None
        self._ready = False
        self._verified = set()

    def update(self, state, now=None):
        supervision = state.get("task_supervision")
        if not isinstance(supervision, dict):
            return  # Old/malformed messages cannot release an existing hold.
        constraints = supervision.get("constraints")
        verified = supervision.get("verified_capabilities")
        if not isinstance(verified, list) or not all(isinstance(cap, str) for cap in verified):
            return
        if not isinstance(constraints, list) or not all(
            isinstance(item, dict) and isinstance(item.get("capabilities"), list)
            and all(isinstance(cap, str) for cap in item["capabilities"])
            and isinstance(item.get("incident_id"), str)
            and item.get("action") == "pause_dependent" for item in constraints
        ):
            return
        with self.condition:
            self._received_at = time.monotonic() if now is None else now
            self._ready = supervision.get("ready") is True
            self._verified = set(verified) if self._ready else set()
            if self._ready:
                retained = [item for item in self._constraints
                            if not set(item["capabilities"]).issubset(verified)]
                by_id = {item["incident_id"]: item for item in retained + constraints}
                self._constraints = deepcopy(list(by_id.values()))
            else:
                # Environment suspension, stale Health or shutdown is not verification.
                by_id = {item["incident_id"]: item for item in self._constraints + constraints}
                self._constraints = deepcopy(list(by_id.values()))
            self.condition.notify_all()

    def blockers(self, required):
        required = {cap for name in required for cap in ALIASES.get(name, {name})}
        with self.condition:
            return deepcopy([item for item in self._constraints
                             if required.intersection(item["capabilities"])])

    def fresh(self, now=None):
        now = time.monotonic() if now is None else now
        with self.condition:
            return self._ready and self._received_at is not None and 0 <= now - self._received_at <= 5.0

    def cleared(self, required):
        with self.condition:
            return self.fresh() and not self.blockers(required)

    def resume_verified(self, required, after):
        """A new Recovery capability verdict, not merely an empty incident list."""
        caps = {cap for name in required for cap in ALIASES.get(name, {name})}
        with self.condition:
            return (bool(caps) and self.fresh() and self._received_at > after
                    and caps.issubset(self._verified) and not self.blockers(required))
