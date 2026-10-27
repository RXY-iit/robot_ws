from __future__ import annotations

import argparse
import json

from robot_agent.agents.mission_agent import MissionAgent
from robot_agent.dry_run_skill_client import DryRunSkillClient


def main(args=None):
    parser = argparse.ArgumentParser(description="Run a robot agent command in dry-run mode.")
    parser.add_argument(
        "--agent-mode",
        choices=["classic", "jev_primary", "jev_shadow"],
        default="classic",
        help="Agent decision mode.",
    )
    parser.add_argument("command", help="Natural-language command.")
    parsed = parser.parse_args(args=args)

    skill_client = DryRunSkillClient()
    agent = MissionAgent(skill_client)
    agent.set_agent_mode(parsed.agent_mode)
    state = agent.run(parsed.command, mode="dry_run")
    print(json.dumps(state.to_dict(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
