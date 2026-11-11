"""UAV 영상으로 처음 받은 배치의 모든 면을 기록부에 올리고, 식별한 면은 다시 보러 가지 않는다(explore.uav_face_registry / redetect_resolved).
시뮬을 쓰지 않는다."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import CONFIRMED, EMPTY, UNSEEN, CoverageMap

ROOT = Path(__file__).resolve().parents[1]


def fp(x=2.0, y=1.0, sx=0.4, sy=0.4):
    return SimpleNamespace(cx=x, cy=y, sx=sx, sy=sy, confidence=0.95)


def cov_on(**over):
    cfg = Config()
    cfg.explore.uav_face_registry = True
    for k, v in over.items():
        setattr(cfg.explore, k, v)
    return CoverageMap(cfg), OccupancyGrid(cfg.arena)


def accept(cov, *boxes, t=1.0):
    cov.ingest_uav_boxes(list(boxes), t)
    cov.ingest_uav_boxes(list(boxes), t + 1.0)                                       # 두 번 일관돼야 받아들인다


def resolve(f, cls="TANK"):
    f.det_n, f.det_votes, f.det_kind = 6, {cls: 6.0}, "object"


def mark_empty(f):
    f.views, f.view_poses = 3, [(1.0, 1.0, 0.0), (1.5, 1.0, 0.0), (2.0, 0.2, 0.0)]


# ====================================================================== 기본값
def test_default_keeps_the_old_behaviour():
    cfg = Config()
    assert cfg.explore.uav_face_registry is False and cfg.explore.redetect_resolved is True
    cov = CoverageMap(cfg)
    accept(cov, fp())
    assert cov.registry == {} and cov.summary()["face_registry"] is None
    f = next(iter(cov.boxes.values())).faces[0]
    resolve(f)
    f.suspect = True
    assert cov.face_state(f) == CONFIRMED and cov.face_weight(f) == pytest.approx(0.4)      # 옛 동작: 이동 의심이면 재검증 가치


def test_real_config_turns_the_registry_on_and_stops_redetecting():
    cfg = Config.load(str(ROOT / "config/waveshare_real.json"))
    assert cfg.explore.uav_face_registry is True and cfg.explore.redetect_resolved is False


# ====================================================================== 기록
def test_all_faces_of_every_accepted_uav_box_are_registered_at_once():
    cov, grid = cov_on()
    accept(cov, fp(2.0, 1.0), fp(3.5, 3.0))
    assert len(cov.boxes) == 2 and len(cov.registry) == 8                             # 상자 둘 × 4면
    assert {r.origin for r in cov.registry.values()} == {"uav"}
    assert {(r.idx) for r in cov.registry.values()} == {0, 1, 2, 3}
    c = cov.registry_counts()
    assert c["registered"] == 8 and c["pending"] == 8 and c["identified"] == 0
    assert all(f.reg is not None for b in cov.boxes.values() for f in b.faces)
    assert cov.registry_t0 == pytest.approx(2.0)


def test_single_unconfirmed_frame_registers_nothing():
    cov, grid = cov_on()
    cov.ingest_uav_boxes([fp()], 1.0)
    assert cov.registry == {}


def test_repeated_frames_do_not_duplicate_records():
    cov, grid = cov_on()
    accept(cov, fp())
    for t in (4.0, 5.0, 6.0):
        cov.ingest_uav_boxes([fp()], t)
    assert len(cov.registry) == 4


def test_resolved_faces_are_marked_identified_or_empty_with_their_class():
    cov, grid = cov_on()
    accept(cov, fp())
    b = next(iter(cov.boxes.values()))
    resolve(b.faces[0], "TRUCK")
    mark_empty(b.faces[1])
    cov.refresh(grid, None, 10.0, force=True)
    recs = {r.idx: r for r in cov.registry.values()}
    assert recs[0].status == "identified" and recs[0].cls == "TRUCK"
    assert recs[1].status == "empty" and recs[1].cls is None
    assert recs[2].status == "pending"
    c = cov.registry_counts()
    assert (c["identified"], c["empty"], c["pending"]) == (1, 1, 2)
    assert cov.summary()["face_registry"]["registered"] == 4
    json.dumps(cov.summary()["face_registry"])


def test_lidar_first_faces_enter_the_registry_when_they_are_resolved():
    cov, grid = cov_on()
    accept(cov, fp(2.0, 1.0))
    # LiDAR 로 먼저 본 상자(UAV 와 멀리 떨어진 곳)
    from maicon26.perception.coverage import BoxModel
    b = BoxModel(99, 4.0, 3.0, "full", [], 1.0, 1.0)
    cov._make_faces(b)
    cov.boxes[99] = b
    resolve(b.faces[2])
    cov.refresh(grid, None, 10.0, force=True)
    rec = cov.registry[b.faces[2].reg]
    assert rec.origin == "lidar" and rec.status == "identified"
    assert b.faces[0].reg is None                                                      # 해결되지 않은 면은 올리지 않는다


# ====================================================================== 다시 보지 않기
def test_resolved_faces_have_no_weight_and_suspect_does_not_revive_them_when_redetect_is_off():
    cov, grid = cov_on(redetect_resolved=False)
    accept(cov, fp())
    f = next(iter(cov.boxes.values())).faces[0]
    resolve(f)
    f.suspect = True
    assert cov.face_weight(f) == 0.0
    f.recheck = True
    assert cov.face_weight(f) == pytest.approx(0.4)                                   # 센서 간 불일치는 예외(확정을 의심)


def test_redetect_resolved_true_keeps_the_old_suspect_revival_even_with_the_registry():
    cov, grid = cov_on(redetect_resolved=True)
    accept(cov, fp())
    f = next(iter(cov.boxes.values())).faces[0]
    resolve(f)
    f.suspect = True
    assert cov.face_weight(f) == pytest.approx(0.4)


def test_registry_resolution_survives_a_vote_wobble():
    cov, grid = cov_on(redetect_resolved=False)
    accept(cov, fp())
    f = next(iter(cov.boxes.values())).faces[0]
    resolve(f)
    cov.refresh(grid, None, 10.0, force=True)
    f.det_n, f.det_votes = 2, {"TANK": 1.0, "TRUCK": 1.0}                             # 득표가 흔들려 UNCERTAIN 으로 내려갔다
    assert cov.face_state(f) != CONFIRMED
    cov.refresh(grid, None, 12.0, force=True)
    assert cov.registry[f.reg].status == "identified"                                 # 기록은 그대로
    assert cov.face_weight(f) == 0.0                                                  # 다시 보러 가지 않는다
    assert f not in cov.pending_faces()


def test_registry_resolved_faces_are_not_offered_for_oak_identification():
    cov, grid = cov_on()
    cov.cfg.oak_det.identify_fallback = True
    accept(cov, fp())
    f = next(iter(cov.boxes.values())).faces[0]
    f.exterior_open, f.arrive_attempts = True, 3
    f.det_n, f.det_votes = 2, {"TANK": 1.0, "TRUCK": 1.0}
    assert f in cov.identify_faces()                                                  # 아직 못 풀었다
    resolve(f)
    cov.refresh(grid, None, 10.0, force=True)
    f.det_n, f.det_votes = 2, {"TANK": 1.0, "TRUCK": 1.0}
    assert f not in cov.identify_faces()                                              # 기록부가 '식별함'이라고 안다


def test_unresolved_faces_keep_their_normal_weight():
    cov, grid = cov_on(redetect_resolved=False)
    accept(cov, fp())
    f = next(iter(cov.boxes.values())).faces[3]
    assert cov.face_state(f) == UNSEEN and cov.face_weight(f) == 1.0


# ====================================================================== 박스가 사라졌다 다시 생겨도
def test_a_box_that_disappears_and_returns_keeps_its_resolved_faces():
    cov, grid = cov_on(redetect_resolved=False)
    accept(cov, fp())
    b = next(iter(cov.boxes.values()))
    resolve(b.faces[0], "TANK")
    mark_empty(b.faces[1])
    b.faces[1].arrive_attempts = 1
    cov.refresh(grid, None, 10.0, force=True)
    rid0, rid1 = b.faces[0].reg, b.faces[1].reg
    del cov.boxes[b.bid]                                                                # UAV 상자가 사라졌다(영상 끊김 뒤 정리 등)
    assert rid0 in cov.registry
    accept(cov, fp(2.05, 1.0), t=20.0)                                                  # 같은 자리에 다시 받아들여진다(5 cm 어긋남)
    nb = next(iter(cov.boxes.values()))
    assert nb.bid != b.bid
    assert nb.faces[0].reg == rid0 and nb.faces[1].reg == rid1                          # 같은 기록을 이어 받는다
    assert nb.faces[1].views == 3 and nb.faces[1].arrive_attempts == 1                  # 관측 이력도 되돌려 준다
    assert cov.face_state(nb.faces[1]) == EMPTY                                         # 빈 면은 처음부터 다시 보지 않는다
    assert cov.face_weight(nb.faces[1]) == 0.0 and cov.face_weight(nb.faces[0]) == 0.0  # 식별 기록 때문에 가치 0
    assert nb.faces[2].reg not in (rid0, rid1) and cov.registry[nb.faces[2].reg].status == "pending"
    assert cov.registry_counts()["restored"] == 2
    assert len(cov.registry) == 4                                                       # 기록이 늘지 않았다


def test_a_box_that_returns_somewhere_else_does_not_inherit_the_old_record():
    cov, grid = cov_on()
    accept(cov, fp(2.0, 1.0))
    b = next(iter(cov.boxes.values()))
    resolve(b.faces[0])
    cov.refresh(grid, None, 10.0, force=True)
    del cov.boxes[b.bid]
    accept(cov, fp(3.5, 3.0), t=20.0)
    nb = next(iter(cov.boxes.values()))
    assert cov.face_state(nb.faces[0]) == UNSEEN and cov.registry_counts()["restored"] == 0


def test_a_record_is_only_inherited_by_the_face_with_the_same_direction():
    cov, grid = cov_on()
    accept(cov, fp(2.0, 1.0))
    b = next(iter(cov.boxes.values()))
    resolve(b.faces[3])                                                                 # S 면(2.0, 0.8)
    cov.refresh(grid, None, 10.0, force=True)
    rid3 = b.faces[3].reg
    del cov.boxes[b.bid]
    accept(cov, fp(1.85, 0.85), t=20.0)                                                 # 새 상자의 E 면(2.05, 0.85)이 옛 S 면과 0.07 m — 하지만 방향이 다르다
    nb = next(iter(cov.boxes.values()))
    assert nb.faces[0].reg != rid3 and nb.faces[3].reg == rid3                           # S 는 S 끼리만(1.85, 0.65 ↔ 2.0, 0.8: 0.21 m)


def test_proven_move_forgets_the_records_of_that_box():
    cov, grid = cov_on()
    from maicon26.perception.coverage import BoxModel
    b = BoxModel(7, 4.0, 3.0, "full", [], 1.0, 1.0, full_seen=3)
    cov._make_faces(b)
    cov.boxes[7] = b
    resolve(b.faces[0])
    cov.refresh(grid, None, 10.0, force=True)
    assert b.faces[0].reg in cov.registry
    omap = SimpleNamespace(drop_box_faces=lambda faces: len(faces), faces=[])
    cov._drop_box(b, omap)                                                              # 여러 번 완전히 보인 상자가 비어 있다고 확인됨 = 옮겨졌다
    assert not any(r.cx == pytest.approx(b.faces[0].cx) for r in cov.registry.values())
    assert cov.registry_stats["forgotten"] >= 1


def test_partial_hypothesis_drop_keeps_the_records():
    cov, grid = cov_on()
    accept(cov, fp())
    b = next(iter(cov.boxes.values()))
    resolve(b.faces[0])
    cov.refresh(grid, None, 10.0, force=True)
    n = len(cov.registry)
    cov._drop_box(b, None)                                                              # UAV 만 본 가설 — 이동을 입증하지 못한다
    assert len(cov.registry) == n


# ====================================================================== 안전
def test_registry_off_never_touches_face_fields():
    cfg = Config()
    cov = CoverageMap(cfg)
    accept(cov, fp())
    assert all(f.reg is None for b in cov.boxes.values() for f in b.faces)
    cov.refresh(OccupancyGrid(cfg.arena), None, 10.0, force=True)
    assert cov.registry == {} and cov.registry_counts()["registered"] == 0


def test_box_snapshots_stay_json_safe():
    """스냅샷/보고서 직렬화(compare_sim.clean(record(box)))는 set 을 못 쓴다 — BoxModel.internal 은 list 여야 한다."""
    import dataclasses
    cov, grid = cov_on()
    accept(cov, fp())

    def rec(v):
        if dataclasses.is_dataclass(v):
            return {f.name: rec(getattr(v, f.name)) for f in dataclasses.fields(v)}
        if isinstance(v, dict):
            return {str(k): rec(x) for k, x in v.items()}
        if isinstance(v, (list, tuple)):
            return [rec(x) for x in v]
        return v

    json.dumps([rec(b) for b in cov.boxes.values()], allow_nan=False)
    json.dumps([rec(r) for r in cov.registry.values()], allow_nan=False)


# ====================================================================== 매니저 보고
def test_manager_reports_the_registry_without_spamming_the_log():
    from unittest.mock import MagicMock
    from hw_fakes import make_robot
    from maicon26.link.base import ServerLink
    from maicon26.mission.manager import Agent
    from maicon26.perception.detector import Detector

    class NullLink(ServerLink):
        def poll(self):
            return []

        def submit(self, kind, payload):
            return True

    class NoDet(Detector):
        def detect(self, frame):
            return []

    cfg = Config()
    cfg.nav.backend = "builtin"
    cfg.explore.uav_face_registry = True
    robot, _, _ = make_robot("practice", "idle", cfg=cfg)
    agent = Agent(robot, NullLink(), cfg=cfg, detector=NoDet())
    agent.stack = MagicMock()
    logs = []
    agent._log = lambda now, msg: logs.append(msg)
    agent._registry_report(1.0)
    assert logs == []                                                                   # 기록이 없으면 말하지 않는다
    accept(agent.coverage, fp())
    agent._registry_report(3.0)
    assert len(logs) == 1 and "등록 4면" in logs[0]
    agent._registry_report(3.5)
    assert len(logs) == 1                                                               # 같은 수는 다시 쓰지 않는다
    f = next(iter(agent.coverage.boxes.values())).faces[0]
    resolve(f)
    agent.coverage.refresh(OccupancyGrid(cfg.arena), None, 4.0, force=True)
    agent._registry_report(4.0)
    assert len(logs) == 1                                                               # 5 s 안에 또 바뀌어도 로그는 늘리지 않는다
    agent._registry_report(9.0)
    assert len(logs) == 2 and "식별 1" in logs[1]
    assert agent.stats["face_registry"]["identified"] == 1
    json.dumps(agent.stats["face_registry"])
    agent.coverage.registry_counts = MagicMock(side_effect=RuntimeError("x"))
    agent._registry_report(20.0)                                                        # 예외는 삼킨다
    assert agent.stats["face_registry_errors"] == 1


def test_manager_registry_report_is_off_by_default():
    from unittest.mock import MagicMock
    from hw_fakes import make_robot
    from maicon26.link.base import ServerLink
    from maicon26.mission.manager import Agent
    from maicon26.perception.detector import Detector

    class NullLink(ServerLink):
        def poll(self):
            return []

        def submit(self, kind, payload):
            return True

    class NoDet(Detector):
        def detect(self, frame):
            return []

    cfg = Config()
    cfg.nav.backend = "builtin"
    robot, _, _ = make_robot("practice", "idle", cfg=cfg)
    agent = Agent(robot, NullLink(), cfg=cfg, detector=NoDet())
    agent._registry_report(1.0)
    assert "face_registry" not in agent.stats
