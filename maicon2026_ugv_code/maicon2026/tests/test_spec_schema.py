"""Wire rules from the local 2026-11-04 API snapshot; no external server access."""
import copy

import pytest

from maicon26.arena import ArenaGeometry
from maicon26.config import Config
from maicon26.link import schema
from maicon26.perception.object_map import ObjectMap
from maicon26.types import Detection, Frame, Pose2D
from maicon26.vocab import BASE_WIRE, FACILITY_WIRE_VALUES, OBJECT_WIRE, SECTOR_WIRE, TARGET_WIRE, WIRE_SECTORS


@pytest.mark.spec("S5.2", "S5.3", "S5.4", "S5.5", "SA.2")
def test_m1_partial_report_replaces_omitted_fields_with_null_and_normalises_case():
    assert schema.validate_m1({"base": "red_maintenance"}) == {
        "target": None, "search_sectors": None, "base": "RED_MAINTENANCE"}
    assert schema.validate_m1({"target": "red_radar", "search_sectors": ["r-s1", "cm"], "base": None}) == {
        "target": "RED_RADAR", "search_sectors": ["CM", "R-S1"], "base": None}


@pytest.mark.spec("S5.2", "S5.8", "SA.1", "SA.4")
@pytest.mark.parametrize("bad", [{}, {"target": None, "base": None}, {"target": "레이더시설"},
                                  {"base": "RED_RADAR"}, {"search_sectors": [1]},
                                  {"search_sectors": ["CM", "cm"]}, {"search_sectors": "R-S1"},
                                  {"target": "RED_RADAR", "sector": "R-S1"}])
def test_m1_invalid_wire_reports_have_structured_422_locations(bad):
    with pytest.raises(schema.ValidationError) as error:
        schema.validate_m1(bad)
    assert isinstance(error.value.detail, list)
    assert error.value.detail[0]["loc"][0] == "body"
    assert error.value.detail[0]["msg"]


@pytest.mark.spec("S5.3", "S5.5", "S5.9")
def test_m1_received_sector_order_and_case_are_cosmetic_but_values_are_not():
    sent = {"target": "red_radar", "search_sectors": ["R-S1", "CM"], "base": "red_supply_depot"}
    received = {"target": "RED_RADAR", "search_sectors": ["cm", "r-s1"], "base": "RED_SUPPLY_DEPOT"}
    assert schema.equal_received("m1", sent, received, "BLUE")
    assert not schema.equal_received("m1", sent, {**received, "target": "RED_COMMS"}, "BLUE")
    assert not schema.equal_received("m1", sent, {**received, "search_sectors": ["CM"]}, "BLUE")
    assert not schema.equal_received("m1", sent, {**received, "search_sectors": ["CM", "R-S1", "R-S2"]}, "BLUE")
    assert schema.equal_received("m1", {"base": "RED_MAINTENANCE"},
                                 {"target": None, "search_sectors": None, "base": "red_maintenance"}, "BLUE")


def m3_sample():
    return {"sectors": [{"sector": "R-S1", "objects": [{"type": "tank", "count": 2}, {"type": "drum", "count": 1}]},
                        {"sector": "CM", "objects": []}, {"sector": "R-S2", "objects": [{"type": "military_truck", "count": 0}]}]}


@pytest.mark.spec("S7.2", "S7.3", "S7.4", "S7.5", "S7.6")
def test_m3_received_sector_and_object_order_do_not_matter_but_omission_and_zero_do():
    sent = m3_sample()
    received = schema.validate_m3(sent)
    received["sectors"].reverse()
    for sector in received["sectors"]:
        sector["objects"].reverse()
    assert schema.equal_received("m3", sent, received, "BLUE")
    different = copy.deepcopy(received)
    different["sectors"].pop()
    assert not schema.equal_received("m3", sent, different, "BLUE")
    different = copy.deepcopy(received)
    next(s for s in different["sectors"] if s["sector"] == "R-S2")["objects"] = []
    assert not schema.equal_received("m3", sent, different, "BLUE")  # Missing explicit zero type differs.
    different = copy.deepcopy(received)
    next(s for s in different["sectors"] if s["sector"] == "R-S1")["objects"][0]["count"] += 1
    assert not schema.equal_received("m3", sent, different, "BLUE")


@pytest.mark.spec("S7.3", "S7.8", "SA.1", "SA.4")
@pytest.mark.parametrize("sectors", [[], [{}]*8, [{"sector": 1}], [{"sector": "COMMON"}],
                                      [{"sector": "CM"}, {"sector": "cm"}],
                                      [{"sector": "R-S1", "objects": None}]])
def test_m3_rejects_bad_sector_shapes_names_duplicates_and_cardinality(sectors):
    with pytest.raises(schema.ValidationError):
        schema.validate_m3({"sectors": sectors})


@pytest.mark.spec("S7.3", "S7.4", "S7.8", "SA.5")
@pytest.mark.parametrize("objects", [[{"type": "UGV", "count": 1}], [{"type": "TRUCK", "count": 1}],
                                      [{"type": "tank", "count": -1}], [{"type": "tank", "count": 100}],
                                      [{"type": "tank", "count": True}], [{"type": "tank", "count": 1.5}],
                                      [{"type": "tank", "count": "2"}],
                                      [{"type": "tank", "count": 1}, {"type": "TANK", "count": 2}]])
def test_m3_rejects_nonclasses_duplicate_types_and_invalid_integer_counts(objects):
    with pytest.raises(schema.ValidationError):
        schema.validate_m3({"sectors": [{"sector": "CM", "objects": objects}]})


@pytest.mark.spec("S7.3", "S7.4", "S7.6", "SA.5")
def test_m3_internal_snapshot_filters_nonpictures_clamps_counts_and_keeps_zero_sectors():
    payload = schema.m3_to_server({"sectors": {"COMMON": {"TANK": 120, "UGV": 2, "BARRIER": 3,
                                                          "DESTROYED_BUILDING": 4, "BLOCK": 5},
                                               "R-S1": {"DRUM": 2}, "B-S1": {"TANK": 8}}}, "BLUE")
    received = schema.validate_m3(payload)
    assert schema.equal_received("m3", payload, received, "BLUE")
    counted = {s["sector"]: s["objects"] for s in received["sectors"]}
    assert counted["CM"] == [{"type": "TANK", "count": 99}]
    assert counted["R-S2"] == counted["R-S3"] == [] and "B-S1" not in counted


@pytest.mark.spec("S7.4", "SA.5")
@pytest.mark.parametrize("kind,accepted", [(k, True) for k in OBJECT_WIRE] +
                         [(k, False) for k in ("UGV", "BUILDING", "BARRIER", "POSITION", "TREE", "DESTROYED_BUILDING", "BLOCK")])
def test_ground_object_map_distinguishes_tank_picture_from_ugv_and_other_obstacles(kind, accepted):
    objects = ObjectMap(Config())
    frame = Frame(None, 1.)
    objects.observe([Detection(kind, .9, (270., 180., 370., 300.), 1.)], Pose2D(3., 2., 0.), frame, None)
    assert bool(objects.faces) is accepted
    if accepted:
        assert objects.faces[0].kind == "object" and objects.faces[0].cls == kind


@pytest.mark.spec("S11.4", "S11.5", "S11.7")
def test_m6_received_case_is_cosmetic_but_rank_order_is_semantic():
    sent = {"ranking": ["r-2", "r-1", "r-3"]}
    assert schema.equal_received("m6", sent, {"ranking": ["R-2", "R-1", "R-3"]}, "BLUE")
    assert not schema.equal_received("m6", sent, {"ranking": ["R-1", "R-2", "R-3"]}, "BLUE")
    assert schema.validate_m6({"ranking": ["b-3"]}, "RED") == {"ranking": ["B-3"]}


@pytest.mark.spec("S11.4", "S11.5", "S11.7", "SA.1")
@pytest.mark.parametrize("rank", [None, [], ["R-1"]*4, ["B-2"], ["R-S2"], ["R-UGV-2"], ["R-1", "r-1"], "R-1"])
def test_m6_rejects_wrong_side_sector_names_duplicates_and_bad_lengths(rank):
    with pytest.raises(schema.ValidationError):
        schema.validate_m6({"ranking": rank}, "BLUE")


@pytest.mark.spec("S11.7", "SA.1")
def test_m6_wrong_camp_error_matches_complete_source_example_without_echoing_unvalidated_input():
    with pytest.raises(schema.ValidationError) as error:
        schema.validate_m6({"ranking": ["b-2"]}, "BLUE")
    assert error.value.detail == [{"loc": ["body", "ranking", 0],
                                  "msg": "위험도 순위는 상대 진영(R-1~R-3)만 적습니다: B-2"}]
    raw = "not-a-callsign-or-safe-echo"
    with pytest.raises(schema.ValidationError) as invalid:
        schema.validate_m6({"ranking": [raw]}, "BLUE")
    assert raw not in str(invalid.value)


@pytest.mark.spec("S2.6", "S9.5", "SR.4")
@pytest.mark.parametrize("text,category", [(schema.REASON_ORDER_NOT_YET, "NOT_YET"),
                                          (schema.REASON_TARGET_NOT_YET, "NOT_YET"),
                                          (schema.REASON_NOT_ASSIGNED, "NOT_ASSIGNED"),
                                          (schema.REASON_NO_POST, "NO_POST"),
                                          (schema.REASON_NO_REHEARSAL, "NO_REHEARSAL"),
                                          ("new server message", "UNKNOWN")])
def test_accepted_false_reason_classification_tolerates_spacing_and_punctuation(text, category):
    assert schema.classify_reason(text) == category
    assert schema.classify_reason(" [ " + text.replace(" ", "  \n") + "! ] ") == category


@pytest.mark.spec("S1.5", "S5.8", "S7.8", "S11.7")
@pytest.mark.parametrize("reason,category", [("phase", "PHASE_MISMATCH"), ("not_assigned", "NOT_ASSIGNED"),
                                            ("excluded", "EXCLUDED"), ("not_started", "UGV_NOT_STARTED")])
def test_submission_rejection_classifier_handles_structured_detail_and_known_prefix(reason, category):
    text = schema.submission_error("m3", reason, "reporting")
    assert schema.classify_409({"detail": [{"loc": ["body"], "msg": text + " — suffix"}]}) == category
    assert schema.classify_409("unknown message") == "UNKNOWN"


@pytest.mark.spec("S1.5", "S4.7", "S5.8", "S7.8", "S11.7", "SA.1")
def test_complete_error_strings_match_snapshot_and_missing_source_suffixes_are_marked():
    assert schema.ERR_INVALID_CODE == "임무코드가 올바르지 않습니다"
    assert schema.ready_error("unconfirmed", "ready") == "UGV 경기 설정이 아직 확인되지 않았습니다 — 잠시 뒤 다시 보내세요"
    assert schema.submission_error("m1", "phase", "ready") == "명령 해석 보고(M1)는 '진행' 단계에서만 받습니다 (지금은 '준비')"
    assert schema.submission_error("m3", "phase", "reporting") == "객체 인식 보고(M3)는 '진행' 단계에서만 받습니다 (지금은 '보고')"
    assert schema.submission_error("m6", "phase", "running") == "상황보고는 '보고' 단계에서만 받습니다 (지금은 '진행')"
    assert schema.drone_error("phase", "reporting", True) == "드론 화면은 경기 진행 중에만 받을 수 있습니다 (지금: 보고) — 지금은 테스트 모드가 아닙니다"
    assert "submission.excluded" in schema.ERROR_DETAILS_UNSPECIFIED
    assert schema.submission_error("m3", "excluded", "running") == "이번 경기에서 빠진 팀입니다"


@pytest.mark.spec("S0.7", "S0.15", "S5.4", "S7.4", "SA.2", "SA.4", "SA.5")
def test_central_wire_tables_match_snapshot_and_arena_rows():
    assert schema.SCHEMA_PROVISIONAL and schema.SCHEMA_SNAPSHOT == "2026-11-04"
    assert set(TARGET_WIRE.values()) == {"COMMS", "OUTPOST", "RADAR"}
    assert set(BASE_WIRE.values()) == {"COMMAND_POST", "MAINTENANCE", "SUPPLY_DEPOT"}
    assert len(FACILITY_WIRE_VALUES) == len(set(FACILITY_WIRE_VALUES)) == 12
    assert set(OBJECT_WIRE.values()) == {"military_truck", "mortar", "tank", "missile", "ammo_box", "drum"}
    assert WIRE_SECTORS == ("B-S1", "B-S2", "B-S3", "CM", "R-S1", "R-S2", "R-S3")
    assert SECTOR_WIRE["COMMON"] == "CM"
    arena = ArenaGeometry()
    for side in ("B", "R"):
        centres = [arena.sector_center(f"{side}-S{i}") for i in (1, 2, 3)]
        assert centres[0][1] > centres[1][1] > centres[2][1]
        assert all(arena.sector_of(*p) == f"{side}-S{i}" for i, p in enumerate(centres, 1))
    assert arena.sector_of(.1, 2.) is None and arena.sector_of(arena.W-.1, 2.) is None


@pytest.mark.spec("S5.4", "SA.2", "SA.4", "S11.4")
@pytest.mark.parametrize("team,prefix,sector", [("BLUE", "RED", "R-S1"), ("RED", "BLUE", "B-S1")])
def test_internal_encoders_keep_opponent_facility_prefix_and_scored_territory(team, prefix, sector):
    out = schema.m1_to_server({"target": "COMM_FACILITY", "goal_base": "SUPPLY_BASE", "sectors": [sector, "COMMON"]}, team)
    assert out["target"] == prefix+"_COMMS" and out["base"] == prefix+"_SUPPLY_DEPOT"
    assert schema.validate_m1(out)["target"] == out["target"]
    assert schema.scoring_sectors(team) == [f"{prefix[0]}-S{i}" for i in (1, 2, 3)]+["CM"]
