#!/usr/bin/env python3
"""Own one recovery command, mirror output to its terminal/log, forward close."""
import argparse
import os
import shlex
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    parser.add_argument("--domain", required=True)
    parser.add_argument("--log", required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("missing command")
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=args.domain, RMW_IMPLEMENTATION="rmw_fastrtps_cpp", PYTHONNOUSERSITE="1")
    for key in ("CYCLONEDDS_URI", "PYTHONPATH", "OLD_PYTHONPATH", "LD_LIBRARY_PATH"):
        env.pop(key, None)
    setup = "\n".join(["set -e", "source /opt/ros/humble/setup.bash",
                       "source " + shlex.quote(os.path.join(args.workspace, "install/setup.bash")),
                       "exec " + shlex.join(command)])
    child = subprocess.Popen(["bash", "--noprofile", "--norc", "-c", setup], cwd=args.workspace,
                             env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, errors="replace", start_new_session=True)
    def stop(_sig, _frame):
        try:
            os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    with open(args.log, "a", buffering=1) as log:
        print(f"Recovery command (domain {args.domain}): {shlex.join(command)}", flush=True)
        for line in child.stdout:
            log.write(line)
            try:
                print(line, end="", flush=True)
            except OSError:
                stop(signal.SIGHUP, None)
        code = child.wait()
        log.write(f"\n[recovery terminal] exited {code}\n")
    if code:
        print(f"Command failed ({code}). Log: {args.log}", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
