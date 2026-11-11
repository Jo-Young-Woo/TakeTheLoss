import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("wingbot_mission_evaluation", ROOT / "scripts/evaluate_wingbot_missions.py")
evaluation = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(evaluation)


def test_final_accepted_counts_not_local_counts_and_overcounts_do_not_cancel_misses():
    expected = {"CM": {"TANK": 2, "DRUM": 1}}
    submitted = {"sectors": [{"sector": "CM", "objects": [{"type": "TANK", "count": 1}, {"type": "DRUM", "count": 2}]}]}
    result = evaluation.compare_m3(expected, submitted)
    assert result["reported_total"] == result["expected_total"] == 3
    assert not result["all_counts_equal"]
    assert result["sum_minimum_bin_counts"] == 2
    assert result["undercount_total"] == result["overcount_total"] == 1
    assert result["identity_level_recall_available"] is False


def test_blue_and_red_score_opposing_sectors_plus_common_from_actual_faces():
    spec = evaluation.load_spec(ROOT / "ros2/maicon_arena_gz/config/arena_spec.yaml")
    layout = evaluation.load_layout(ROOT / "ros2/maicon_arena_gz/config/layouts/layout_default.yaml")
    blue, _ = evaluation.aggregate_truth(spec, layout, "BLUE")
    red, _ = evaluation.aggregate_truth(spec, layout, "RED")
    assert set(blue) == {"R-S1", "R-S2", "R-S3", "CM"}
    assert set(red) == {"B-S1", "B-S2", "B-S3", "CM"}
    assert sum(sum(value.values()) for value in blue.values()) == 21
    assert blue["CM"]["MORTAR"] == 4
    assert blue["R-S1"]["TANK"] == 4


def test_local_entry_and_wrong_base_capture_are_not_m5_success():
    spec = evaluation.load_spec(ROOT / "ros2/maicon_arena_gz/config/arena_spec.yaml")
    layout = evaluation.load_layout(ROOT / "ros2/maicon_arena_gz/config/layouts/layout_default.yaml")
    config = {"config": {"team": "BLUE", "slot": 2}, "operator_fixture": {"targets": {"BLUE": {"objective": "RED_MAINTENANCE"}}}}
    report = {"submitted_reports": {"m1": {"target": "RED_RADAR"}, "m6": {"ranking": ["R-1", "R-2", "R-3"]}},
              "agent_log": [[250., "M5 local entry estimate in R-1"]],
              "referee_events": [{"type": "base_captured", "base": "R-1", "alliance": "BLUE"},
                                  {"type": "target_hit", "shooter": "B-2", "camp": "BLUE", "kind": "radar"}]}
    result = evaluation.evaluate(report, config, spec, layout)
    assert result["m1"]["local_api_accepted"]
    assert result["m6"]["local_api_accepted"]
    assert not result["m5"]["referee_confirmed_capture"]
    assert not result["m2"]["referee_confirmed_designated_hit"]
    report["referee_events"] += [{"type": "base_captured", "base": "R-2", "alliance": "BLUE"},
                                 {"type": "target_hit", "shooter": "B-2", "camp": "RED", "kind": "radar"}]
    result = evaluation.evaluate(report, config, spec, layout)
    assert result["m5"]["referee_confirmed_capture"]
    assert result["m2"]["referee_confirmed_designated_hit"]
