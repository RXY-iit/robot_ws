#!/usr/bin/env python3
"""Refuse a second dashboard stack, including orphaned Python entrypoints."""
import os
import fcntl
import hashlib
from pathlib import Path
import sys


NAMES = {"agent_node", "dashboard_node", "health_monitor", "recovery_agent"}


def find_existing(workspace, domain, proc=Path("/proc")):
    matches = []
    prefix = str(Path(workspace).resolve() / "install/robot_agent/lib/robot_agent") + "/"
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        name = None
        try:
            argv = (entry / "cmdline").read_bytes().decode(errors="replace").split("\0")
            name = next((arg[len(prefix):] for arg in argv
                         if arg.startswith(prefix) and arg[len(prefix):] in NAMES), None)
            if not name and any(argv[n:n + 3] == ["launch", "robot_agent", "agent_dashboard.launch.py"]
                                for n in range(len(argv) - 2)):
                if (entry / "cwd").resolve() == Path(workspace).resolve():
                    name = "agent_dashboard.launch.py (starting/running)"
            if not name:
                continue
            env = dict(item.split("=", 1) for item in (entry / "environ").read_bytes()
                       .decode(errors="replace").split("\0") if "=" in item)
            if env.get("ROS_DOMAIN_ID", "0") == str(domain):
                matches.append((int(entry.name), name))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            if name:
                matches.append((int(entry.name), name + " (domain unreadable)"))
    return sorted(matches)


def acquire_launch_lock(workspace, domain, directory=None):
    directory = Path(directory or f"/tmp/robot-agent-launch-{os.getuid()}")
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    identity = f"{Path(workspace).resolve()}:{domain}"
    path = directory / (hashlib.sha256(identity.encode()).hexdigest()[:24] + ".lock")
    stream = path.open("a+")
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        stream.close()
        raise RuntimeError("An Agent launcher is already starting/running in this ROS domain.") from None
    # ros2 launch must hold the lock until it exits, not just during the check.
    os.set_inheritable(stream.fileno(), True)
    return stream


def main():
    workspace = Path(__file__).resolve().parents[1]
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    command = sys.argv[2:] if sys.argv[1:2] == ["--run"] else []
    lock = None
    if sys.argv[1:2] == ["--run"]:
        if not command:
            print("--run requires a command", file=sys.stderr)
            return 2
        try:
            lock = acquire_launch_lock(workspace, domain)
        except RuntimeError as exc:
            print(f"[launcher] {exc}", file=sys.stderr)
            return 1
    matches = find_existing(workspace, domain)
    if matches:
        print("[launcher] Existing Agent stack detected. Stop it before launching another:", file=sys.stderr)
        for pid, name in matches:
            print(f"  PID {pid}: {name}", file=sys.stderr)
        if lock:
            lock.close()
        return 1
    if command:
        os.execvp(command[0], command)
    return 0


if __name__ == "__main__":
    sys.exit(main())
