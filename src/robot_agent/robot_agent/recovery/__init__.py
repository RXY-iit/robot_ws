"""Recovery Agent: health-driven diagnosis and playbook-based recovery.

Design: todo-my/recovery-agent/design.md (shared core) and sim-guideline.md.

The package is ROS-free so the diagnosis engine can be unit-tested with
synthetic health snapshots; ``robot_agent.nodes.recovery_agent`` wires it to
ROS topics and supplies the live probe adapters.
"""
