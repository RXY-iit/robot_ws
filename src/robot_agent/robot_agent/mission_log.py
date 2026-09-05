from __future__ import annotations

import json
import os
from datetime import datetime
from typing import Any


class MissionLogger:
    def __init__(self, root: str = "debug-output/agent"):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.abspath(os.path.join(root, stamp))
        os.makedirs(self.run_dir, exist_ok=True)
        self.events_path = os.path.join(self.run_dir, "skill_events.jsonl")

    def write_event(self, event: dict[str, Any]):
        with open(self.events_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(event, ensure_ascii=False) + "\n")
