import pytest

from robot_agent.agents.perception_plan import compile_checks, target_config, parse_seed, questions_for


@pytest.mark.parametrize("source", ["red thing", "red objects", "something red"])
def test_primary_generic_reference_is_not_an_unsupported_category(source):
    unit = dict(source=source, kind="color", object="none", color="red")
    checks, gaps, proposals = compile_checks(
        [unit], target_config(), generic_color_reference=True)
    assert [c["query"] for c in checks] == ["color_red"]
    assert not gaps and not proposals


@pytest.mark.parametrize("source", ["red box", "light red thing", "red dangerous thing"])
def test_generic_reference_does_not_erase_specific_constraints(source):
    unit = dict(source=source, kind="color", object="none", color="red")
    _, gaps, _ = compile_checks([unit], target_config(), generic_color_reference=True)
    assert any(g["kind"] == "qualifier" for g in gaps)


def test_classic_compilation_keeps_existing_behavior():
    unit = dict(source="red thing", kind="color", object="none", color="red")
    _, gaps, _ = compile_checks([unit], target_config())
    assert any("thing" in g["reason"] for g in gaps)


def test_questions_anchor_each_fragment_instead_of_only_array_index():
    config = target_config()
    units, relation, _ = parse_seed("yellow car or red bottle", config)
    questions, _ = questions_for(units, config, relation)
    for i, source in enumerate(["yellow car", "red bottle"]):
        for key in ["kind", "object", "color"]:
            assert f"source fragment {source!r}" in questions[f"{key}_{i}"]["instructions"]
    assert "No color requirement" in questions["color_1"]["criteria"]["none"]
