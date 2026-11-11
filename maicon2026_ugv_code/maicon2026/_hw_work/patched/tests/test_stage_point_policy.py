"""M5 대기점을 적 가운데 거점(BLUE→R-2, RED→B-2) 문 앞으로(mission.stage_policy). 옛 값 (W-2.0, H/2)=(4.12, 2.50)은
Gazebo 고정 배치의 R_middle_block(x 4.02–4.82, y 2.30–2.70) 안이었다. 시뮬을 쓰지 않는다."""
import math
from unittest.mock import MagicMock

import pytest

from hw_fakes import make_robot
from maicon26.arena import ArenaGeometry
from maicon26.config import Config
from maicon26.link.base import ServerLink
from maicon26.mission.manager import Agent
from maicon26.perception.detector import Detector
from maicon26.types import Pose2D
from maicon26.validate import sanitize_config

R_MIDDLE_BLOCK = (4.02, 2.30, 4.82, 2.70)          # sim/fixed_maicon.py 의 고정 구조물(중심 4.42, 2.50, 0.80×0.40)


class NullLink(ServerLink):
    def poll(self):
        return []

    def submit(self, kind, payload):
        return True


class NoDet(Detector):
    def detect(self, frame):
        return []


def make_agent(team="BLUE", cfg=None):
    cfg = cfg or Config()
    cfg.nav.backend = "builtin"
    robot, _, _ = make_robot("practice", "idle", cfg=cfg)
    agent = Agent(robot, NullLink(), cfg=cfg, detector=NoDet())
    agent.team, agent.slot = team, 2
    agent.stack = MagicMock()
    agent.stack.pose = Pose2D(1.0, 2.5, 0.0)
    return agent


def inside(pt, rect):
    return rect[0] <= pt[0] <= rect[2] and rect[1] <= pt[1] <= rect[3]


def test_default_is_the_door_front_of_the_enemy_middle_base():
    blue, red = make_agent("BLUE")._stage_point(), make_agent("RED")._stage_point()
    assert blue == pytest.approx((6.12 - 0.80 - 0.30, 2.50))           # R-2 경계 x=5.32 에서 0.30 m 안쪽
    assert red == pytest.approx((0.80 + 0.30, 2.50))                   # B-2 경계 x=0.80 에서 0.30 m 안쪽
    assert blue[0] + red[0] == pytest.approx(6.12) and blue[1] == red[1]    # 좌우 대칭


def test_the_old_point_was_inside_the_fixed_block_and_the_new_one_is_not():
    legacy = make_agent("BLUE", cfg=Config())
    legacy.cfg.mission.stage_policy = "legacy"
    assert legacy._stage_point() == pytest.approx((4.12, 2.50)) and inside(legacy._stage_point(), R_MIDDLE_BLOCK)
    new = make_agent("BLUE")._stage_point()
    assert not inside(new, R_MIDDLE_BLOCK)
    # 고정 배치에서 블록 오른쪽 면(4.82)과 거점 경계(5.32) 사이 가운데는 5.07 — 기본 0.30 은 블록에 0.20 m 로 붙는다(알려진 여유 부족)
    assert new[0] - R_MIDDLE_BLOCK[2] == pytest.approx(0.20)


def test_legacy_policy_reproduces_the_old_values_for_both_teams():
    for team, want in (("BLUE", (4.12, 2.50)), ("RED", (2.0, 2.50))):
        a = make_agent(team)
        a.cfg.mission.stage_policy = "legacy"
        assert a._stage_point() == pytest.approx(want)


def test_override_wins_over_the_policy():
    a = make_agent("BLUE")
    a.cfg.mission.stage_point_override = (3.60, 3.00)                   # 고정 Gazebo 배치용
    assert a._stage_point() == (3.60, 3.00)


def test_unknown_team_falls_back_to_the_old_point_instead_of_guessing_a_side():
    a = make_agent(None)
    assert a._stage_point() == pytest.approx((2.0, 2.50))               # 옛 코드와 같은 결과(BLUE 가 아니면 2.0)


def test_standoff_moves_the_point_and_is_clamped_inside_the_arena():
    a = make_agent("BLUE")
    a.cfg.mission.stage_standoff_m = 0.50
    assert a._stage_point()[0] == pytest.approx(6.12 - 0.80 - 0.50)
    a.cfg.mission.stage_standoff_m = 9.0
    assert a._stage_point()[0] == pytest.approx(0.3)                    # 벽에서 0.3 m 이상


def test_uses_the_measured_base_outline_when_the_geometry_has_one():
    cfg = Config()
    cfg.arena = ArenaGeometry(measured_bases={"R-2": (5.20, 2.05, 6.12, 3.05), "B-2": (0.0, 2.05, 0.92, 3.05)})
    assert make_agent("BLUE", cfg=cfg)._stage_point() == pytest.approx((5.20 - 0.30, 2.55))
    cfg2 = Config()
    cfg2.arena = cfg.arena
    assert make_agent("RED", cfg=cfg2)._stage_point() == pytest.approx((0.92 + 0.30, 2.55))


def test_point_is_in_the_arena_outside_every_base_and_in_the_enemy_middle_row():
    for team in ("BLUE", "RED"):
        a = make_agent(team)
        x, y = a._stage_point()
        geo = a.geo
        assert geo.inside(x, y, margin=0.3)
        assert not any(geo.in_base(x, y, n) for n in geo.bases("BLUE") + geo.bases("RED"))
        enemy = "R-2" if team == "BLUE" else "B-2"
        assert abs(y - geo.base_center(enemy)[1]) < 1e-9
        assert geo.sector_of(x, y) in ("R-S2", "B-S2")                  # 적 진영 가운데 섹터 안


def test_broken_geometry_does_not_raise_and_uses_the_old_point():
    a = make_agent("BLUE")
    a.geo = MagicMock()
    a.geo.base_rect.side_effect = KeyError("R-2")
    a.geo.W, a.geo.H = 6.12, 5.0
    assert a._stage_point() == pytest.approx((4.12, 2.50))


def test_sanitize_resets_a_bad_policy_or_standoff_and_leaves_defaults_alone():
    assert sanitize_config(Config()) == []
    cfg = Config()
    cfg.mission.stage_policy, cfg.mission.stage_standoff_m = "middle", 7.0
    changed = sanitize_config(cfg)
    assert set(changed) == {"mission.stage_policy", "mission.stage_standoff_m"}
    assert cfg.mission.stage_policy == "enemy_middle" and cfg.mission.stage_standoff_m == pytest.approx(0.30)
    cfg = Config()
    cfg.mission.stage_standoff_m = float("nan")
    assert sanitize_config(cfg) == ["mission.stage_standoff_m"] and cfg.mission.stage_standoff_m == pytest.approx(0.30)
