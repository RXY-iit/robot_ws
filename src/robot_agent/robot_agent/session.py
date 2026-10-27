"""Shared launch identity; never reuse a previous process's dashboard state."""

import os
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4


def session_id() -> str:
    value = os.environ.get("ROBOT_AGENT_SESSION_ID")
    if not value:
        value = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid4().hex[:8]
        os.environ["ROBOT_AGENT_SESSION_ID"] = value
    return value


def session_dir(workspace: Path) -> Path:
    path = workspace / "debug-output" / "sessions" / session_id()
    path.mkdir(parents=True, exist_ok=True)
    marker = path / f".pid-{os.getpid()}"
    if not marker.exists():
        marker.touch()
    return path
