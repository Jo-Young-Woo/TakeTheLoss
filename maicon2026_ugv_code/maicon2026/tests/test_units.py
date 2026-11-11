import math
from maicon26.config import Config
from maicon26.mission.command import CommandParser, parse_base_command
from maicon26.util import wrap, clamp
from maicon26.arena import ArenaGeometry


def test_wrap_clamp():
    assert abs(wrap(3 * math.pi) - math.pi) < 1e-9 or abs(wrap(3 * math.pi) + math.pi) < 1e-9
    assert clamp(5, 0, 1) == 1


def test_official_example():
    c = CommandParser().parse("찰리와 델타 구역의 레이더 시설을 타격하고 군수보급소로 진입하라")
    assert c.to_payload() == {"target": "RADAR_FACILITY", "sectors": ["CHARLIE", "DELTA"], "goal_base": "SUPPLY_BASE"}


def test_parser_variants():
    p = CommandParser()
    c = p.parse("알파, 브라보 구역 정찰 후 통신 시설을 타격하라. 이후 정비소로 이동")
    assert c.target == "COMM_FACILITY"
    assert set(c.sectors) == {"ALPHA", "BRAVO"}
    assert c.goal_base == "MAINT_DEPOT"


def test_base_command():
    assert parse_base_command("작전지휘소로 진입하라") == "COMMAND_POST"


def test_config_roundtrip(tmp_path):
    import json
    cfg = Config()
    f = tmp_path / "c.json"
    f.write_text(json.dumps(cfg.to_dict()))
    assert Config.load(str(f)).to_dict() == cfg.to_dict()


def test_config_unknown_key(tmp_path):
    import json, pytest
    f = tmp_path / "c.json"
    f.write_text(json.dumps({"nope": 1}))
    with pytest.raises(KeyError):
        Config.load(str(f))


def test_arena_bases_distinct():
    g = ArenaGeometry()
    cs = [g.base_center(f"{s}-{i}") for s in "BR" for i in (1, 2, 3)]
    assert len(set(cs)) == 6
    assert g.base_center("B-1")[1] > g.base_center("B-3")[1]
