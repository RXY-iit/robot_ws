import re

from robot_agent.mission_test_commands import MISSION_TEST_COMMANDS


def test_all_mission_cases_and_variants_have_unique_labels():
    labels = [label for label, _ in MISSION_TEST_COMMANDS]
    assert len(labels) == len(set(labels))
    ids = {re.match(r"M\d+", label).group() for label in labels}
    assert ids == {f"M{i:02}" for i in [*range(1, 18), *range(23, 31)]}
    assert all(command.strip() for _, command in MISSION_TEST_COMMANDS)
