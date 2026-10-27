from __future__ import annotations

import json
import os
from datetime import datetime
from typing import Any
from pathlib import Path
from robot_agent.session import session_dir


class MissionLogger:
    def __init__(self, root: str = "debug-output/agent"):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = str(session_dir(Path(os.environ.get("ROBOT_WS", Path.cwd()))) / "agent") if root == "debug-output/agent" else os.path.abspath(os.path.join(root, stamp))
        os.makedirs(self.run_dir, exist_ok=True)
        self.events_path = os.path.join(self.run_dir, "skill_events.jsonl")

    def write_event(self, event: dict[str, Any]):
        if event.get("type") == "agent_heartbeat":
            return
        # Keep bounded debug history; detailed bags are explicitly recorded elsewhere.
        if os.path.exists(self.events_path) and os.path.getsize(self.events_path) > 10 * 1024 * 1024:
            for index in range(2, 0, -1):
                source = f"{self.events_path}.{index}"
                if os.path.exists(source):
                    os.replace(source, f"{self.events_path}.{index + 1}")
            os.replace(self.events_path, f"{self.events_path}.1")
        with open(self.events_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(event, ensure_ascii=False) + "\n")
