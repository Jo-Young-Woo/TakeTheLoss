"""UAV 로 받은 상자의 면을 섹터로 나눠 장부에 두고, M1 이 섹터를 지정하면 그 섹터의 면을 모두 확인·분류해 면 식별(기록부)에 올린다.

ROS·네트워크·시뮬 월드 없이 돈다(가짜 로봇·모의 서버·FakeClock). 객체 분류 득표는 YOLO 없이 장부에 직접 넣는다(분류 정확도는 이 시험의 대상이 아니다).
해석: M1 이 지정한 섹터 = '탐색 구역'. explore.m1_sector_sweep 가 켜지면 채점 구역(M3 제출)과 별개로 그 섹터의 면을 우선 방문한다.
"""
import json
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from maicon26.arena import ArenaGeometry
from maicon26.config import Config
from maicon26.link.schema import scoring_sectors, server_sector
from maicon26.mission.explore_online import OnlineExplorer
from maicon26.mission.manager import Agent
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import BoxModel, CONFIRMED, CoverageMap, EMPTY, UNSEEN
from maicon26.types import Command, Pose2D

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2/maicon_arena_gz"


def fp(x, y, sx=0.4, sy=0.4):
    return SimpleNamespace(cx=x, cy=y, sx=sx, sy=sy, confidence=0.95)


def cov_on(**over):
    cfg = Config()
    cfg.explore.uav_face_registry = True
    for key, value in over.items():
        setattr(cfg.explore, key, value)
    return CoverageMap(cfg), OccupancyGrid(cfg.arena)


def accept(cov, *boxes, t=1.0):
    cov.ingest_uav_boxes(list(boxes), t)
    cov.ingest_uav_boxes(list(boxes), t + 1.0)                                       # 두 번 일관돼야 받아들인다


def resolve(f, cls="TANK"):
    f.det_n, f.det_votes, f.det_kind = 6, {cls: 6.0}, "object"


def mark_empty(f):
    f.views, f.view_poses = 3, [(1.0, 1.0, 0.0), (1.5, 1.0, 0.0), (2.0, 0.2, 0.0)]


def box_in(cov, sector):
    return next(b for b in cov.boxes.values() if server_sector(cov.box_sector(b) or "") == sector)


def sector_point(cfg, sector, dx=0.0, dy=0.0):
    x, y = cfg.arena.sector_center(sector)
    return x + dx, y + dy


# ====================================================================== 장부: 섹터별 면
def test_ledger_splits_uav_boxes_by_sector_and_counts_four_faces_each():
    cov, grid = cov_on()
    cfg = cov.cfg
    pts = {"B-S1": sector_point(cfg, "B-S1"), "R-S2": sector_point(cfg, "R-S2"), "CM": sector_point(cfg, "COMMON")}
    accept(cov, *[fp(*p) for p in pts.values()], fp(*sector_point(cfg, "R-S2", 0.0, 0.8)))
    led = cov.sector_ledger()
    assert led["B-S1"]["boxes"] == 1 and led["B-S1"]["faces"] == 4
    assert led["R-S2"]["boxes"] == 2 and led["R-S2"]["faces"] == 8 and led["CM"]["boxes"] == 1
    assert led["R-S1"]["boxes"] == 0 and led["R-S3"]["boxes"] == 0 and led["none"]["boxes"] == 0
    assert led["R-S2"]["pending"] == 8 and len(led["R-S2"]["todo"]) == 8 and led["R-S2"]["complete"] is False
    assert all(isinstance(k, list) and len(k) == 2 for k in led["R-S2"]["todo"])
    reg = cov.registry_by_sector()
    assert reg["R-S2"]["registered"] == 8 and reg["R-S2"]["pending"] == 8 and reg["CM"]["registered"] == 4
    assert {r.sector for r in cov.registry.values()} == {"B-S1", "R-S2", "CM"}


def test_ledger_sector_follows_the_box_centre_not_the_face():
    cov, _ = cov_on()
    geo = cov.geo
    # 두 섹터 경계 가까이(박스 중심은 한쪽, 한 면의 중심은 반대쪽에 걸친다)
    border = next(x for x in np.arange(0.5, geo.W, 0.01) if geo.sector_of(x, 2.4) != geo.sector_of(x + 0.01, 2.4))
    x = border - 0.05
    accept(cov, fp(x, 2.4))
    b = next(iter(cov.boxes.values()))
    assert cov.box_sector(b) == geo.sector_of(x, 2.4)
    assert {r.sector for r in cov.registry.values()} == {server_sector(geo.sector_of(x, 2.4))}


def test_ledger_states_identified_empty_and_completion_per_sector():
    cov, grid = cov_on()
    p = sector_point(cov.cfg, "R-S2")
    accept(cov, fp(*p))
    b = next(iter(cov.boxes.values()))
    for f, how in zip(b.faces, ("id", "id", "empty", None)):
        if how == "id":
            resolve(f, "TRUCK")
        elif how == "empty":
            mark_empty(f)
    cov.refresh(grid, None, 10.0, force=True)
    row = cov.sector_ledger(["R-S2"])["R-S2"]
    assert (row["identified"], row["empty"], row["pending"]) == (2, 1, 1) and row["complete"] is False
    assert row["todo"] == [list(b.faces[3].key)]
    resolve(b.faces[3], "TANK")
    cov.refresh(grid, None, 11.0, force=True)
    row = cov.sector_ledger(["R-S2"])["R-S2"]
    assert (row["identified"], row["empty"], row["todo"], row["complete"]) == (3, 1, [], True)
    prog = cov.sector_progress(["R-S2"])
    assert prog["done"] == 4 and prog["todo"] == 0 and prog["complete"] is True and prog["boxes"] == 1
    assert {r.status for r in cov.registry.values()} == {"identified", "empty"}
    assert cov.registry_by_sector()["R-S2"]["identified"] == 3


def test_neighbouring_boxes_hide_their_facing_faces_and_hidden_faces_are_not_todo():
    cov, grid = cov_on()
    x, y = sector_point(cov.cfg, "R-S2")
    accept(cov, fp(x, y), fp(x + 0.4, y))                                            # 서로 붙은 상자
    cov.refresh(grid, None, 10.0, force=True)
    row = cov.sector_ledger(["R-S2"])["R-S2"]
    assert row["boxes"] == 2 and row["faces"] == 8
    assert row["hidden"] == 2 and row["pending"] == 6 and len(row["todo"]) == 6
    for b in cov.boxes.values():
        for f in b.faces:
            assert (cov.face_ledger_state(b, f) == "hidden") == (not f.exterior_open)
    # 보이는 면을 모두 풀면 가려진 면이 남아 있어도 섹터는 끝난 것이다
    for b in cov.boxes.values():
        for f in b.faces:
            if f.exterior_open:
                resolve(f)
    cov.refresh(grid, None, 11.0, force=True)
    prog = cov.sector_progress(["R-S2"])
    assert prog["complete"] is True and prog["hidden"] == 2 and prog["done"] == 6 and prog["todo"] == 0


def test_given_up_and_uncertain_faces_are_reported_apart_from_pending():
    cov, grid = cov_on()
    accept(cov, fp(*sector_point(cov.cfg, "B-S2")))
    b = next(iter(cov.boxes.values()))
    b.faces[0].arrive_attempts = cov.ec.attempts_giveup                                # 가 봤지만 못 푼 면
    b.faces[1].det_n, b.faces[1].det_votes, b.faces[1].det_kind = 1, {"TANK": 1.0}, "object"      # 득표 부족
    row = cov.sector_ledger()["B-S2"]
    assert row["unreachable"] == 1 and row["uncertain"] == 1 and row["pending"] == 2
    assert len(row["todo"]) == 3                                                       # 못 푼 면은 할 일이 아니다(더 시간을 쓰지 않는다)
    assert cov.sector_progress(["B-S2"])["unreachable"] == 1


def test_sector_without_boxes_is_not_complete_and_unknown_names_are_ignored():
    cov, _ = cov_on()
    accept(cov, fp(*sector_point(cov.cfg, "R-S2")))
    prog = cov.sector_progress(["R-S1"])
    assert prog["boxes"] == 0 and prog["complete"] is False                              # 상자를 못 봤다 ≠ 끝났다(영상이 없을 수도 있다)
    assert cov.sector_ledger(["ALPHA", "bogus"]) == {}
    assert set(cov.sector_ledger(["COMMON", "r-s2"])) == {"CM", "R-S2"}


def test_long_structures_and_suspended_image_only_boxes_are_not_counted():
    cov, _ = cov_on()
    accept(cov, fp(*sector_point(cov.cfg, "R-S2")))
    x, y = sector_point(cov.cfg, "R-S3")
    wall = BoxModel(90, x, y, "full", [], 1.0, 1.0, sx=2.0, sy=0.4)                  # 긴 구조물(M3 후보 아님)
    cov._make_faces(wall)
    cov.boxes[90] = wall
    led = cov.sector_ledger()
    assert led["R-S3"]["boxes"] == 0 and led["R-S2"]["boxes"] == 1
    cov.tick_uav_layout(200.0)                                                         # 영상이 끊긴 지 오래 → UAV 가설 상자는 쓸 수 없다
    assert cov.sector_ledger()["R-S2"]["boxes"] == 0


def test_summary_carries_the_ledger_only_when_the_registry_is_on_and_is_json_safe():
    cov, grid = cov_on()
    accept(cov, fp(*sector_point(cov.cfg, "R-S2")))
    cov.refresh(grid, None, 10.0, force=True)
    summary = cov.summary()
    assert summary["sector_ledger"]["R-S2"]["boxes"] == 1
    json.dumps(summary["sector_ledger"])
    off = CoverageMap(Config())
    assert off.summary()["sector_ledger"] is None


# ====================================================================== 탐색: M1 섹터를 우선 방문
def layout():
    """test_m3_scoring_exploration 과 같은 배치: 섹터 7개 중심에 알 수 없는 상자 하나씩."""
    cfg = Config()
    cfg.explore.m1_sector_sweep = True
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    names = {}
    for bid, sector in enumerate(cfg.arena.sector_names(), 1):
        cx, cy = cfg.arena.sector_center(sector)
        box = BoxModel(bid, cx, cy, "full", [], 1., 1., full_seen=3, last_full_t=1.)
        cov._make_faces(box)
        cov.boxes[bid] = box
        names[bid] = sector
        for iy in range(grid.ny):
            for ix in range(grid.nx):
                x, y = grid.to_world(iy, ix)
                if abs(x - cx) < .2 and abs(y - cy) < .2:
                    grid.L[iy, ix] = grid.L_MAX
    return cfg, cov, grid, names


def _built(cfg, cov, grid, m1=None, team="BLUE", pose=Pose2D(.4, 2.5, 0.)):
    explorer = OnlineExplorer(cfg, cov)
    if m1 is not None:
        explorer.m1_sectors = set(m1)
    blocked, _ = cov.planning_cost_maps(grid)
    return explorer, explorer.build(pose, grid, blocked, 10., team=team)


def _face_boxes(cands):
    return {bid for c in cands for bid, _ in c.faces}


def test_m1_sectors_extend_the_search_beyond_the_scoring_territory():
    cfg, cov, grid, names = layout()
    base_explorer, base = _built(cfg, cov, grid)
    scoring = {bid for bid, n in names.items() if server_sector(n) in scoring_sectors("BLUE")}
    assert _face_boxes(base) == scoring
    own = next(b for b, n in names.items() if server_sector(n) == "B-S2")
    assert own not in scoring
    explorer, cands = _built(cfg, cov, grid, m1={"B-S2"})
    assert _face_boxes(cands) == scoring | {own}
    assert explorer.diagnostics["last_build"]["m1_sweep_sectors"] == ["B-S2"]
    assert "m1_sweep_sectors" not in base_explorer.diagnostics["last_build"] or base_explorer.diagnostics["last_build"]["m1_sweep_sectors"] == []


def test_default_off_or_legacy_names_change_nothing():
    cfg, cov, grid, names = layout()
    _, base = _built(cfg, cov, grid)
    cfg.explore.m1_sector_sweep = False
    _, off = _built(cfg, cov, grid, m1={"B-S2"})
    assert _face_boxes(off) == _face_boxes(base)
    assert [(c.kind, c.vp, round(c.gain_f, 6)) for c in off] == [(c.kind, c.vp, round(c.gain_f, 6)) for c in base]
    cfg.explore.m1_sector_sweep = True
    _, legacy = _built(cfg, cov, grid, m1={"CHARLIE", "bogus"})                        # 서버에 없는 이름은 무시
    assert _face_boxes(legacy) == _face_boxes(base)
    _, unscoped = _built(cfg, cov, grid, m1={"B-S2"}, team=None)                       # 팀을 모르는 도구 호출은 기존 그대로
    assert unscoped is not None


def test_m1_sector_faces_get_the_configured_weight():
    cfg, cov, grid, names = layout()
    own = next(b for b, n in names.items() if server_sector(n) == "B-S2")
    gains = {}
    for weight in (1.0, 1.6):
        cfg.explore.m1_sector_weight = weight
        _, cands = _built(cfg, cov, grid, m1={"B-S2"})
        gains[weight] = {c.vp: c.gain_f for c in cands if c.kind == "face" and {bid for bid, _ in c.faces} == {own}}
    shared = set(gains[1.0]) & set(gains[1.6])
    assert shared
    for vp in shared:
        assert gains[1.6][vp] == pytest.approx(gains[1.0][vp] * 1.6, rel=1e-6)


def test_m1_sector_faces_leave_the_pending_list_once_identified():
    cfg, cov, grid, names = layout()
    own = next(b for b, n in names.items() if server_sector(n) == "B-S2")
    for f in cov.boxes[own].faces:
        resolve(f)
    _, cands = _built(cfg, cov, grid, m1={"B-S2"})
    assert own not in _face_boxes(cands)
    assert all(cov.face_state(f) == CONFIRMED for f in cov.boxes[own].faces)


# ====================================================================== manager: M1 섹터·진행 보고·임무 면
# 시뮬 월드는 쓰지 않는다: 가짜 로봇·모의 서버(FakeClock)·빈 항법 대역으로 Agent 를 만든다(tests/test_http_simulation_uav.py 와 같은 방식).
CODE = "UavE2E88"


def _measured_cfg():
    cfg = Config()
    cfg.arena = ArenaGeometry.from_spec(yaml.safe_load((PACKAGE / "config/arena_spec.yaml").read_text(encoding="utf-8")))
    cfg.team, cfg.slot = "BLUE", 2
    cfg.hal.restart_state_path = None
    cfg.explore.uav_face_registry = True
    cfg.explore.m1_sector_sweep = True
    return cfg


@pytest.fixture
def make_agent(monkeypatch):
    from maicon26.link.async_link import AsyncServerLink
    from maicon26.link.http import HttpServerLink
    from maicon26.sim.mockserver import FakeClock, MockServer
    monkeypatch.setattr(threading.Thread, "start", lambda _: None)            # 백그라운드 스레드는 시작하지 않는다
    made = []

    def build(cfg=None, detector=None):
        cfg = cfg or _measured_cfg()
        clock = FakeClock(10.)
        mock = MockServer({CODE: "B-2"}, clock=clock)
        mock.set_phase("running")
        extra = {} if detector is None else {"uav_layout_detector": detector, "uav_layout_source": "real_uav_image"}
        link = HttpServerLink("http://fixture.invalid", CODE, clock=clock, transport=mock, **extra)
        assert link.me()["phase"] == "running"
        link._started_bg = True
        transport = AsyncServerLink(link)
        robot = SimpleNamespace(get_scan=lambda: None, get_frame=lambda: None, get_depth=lambda: None,
                                get_odom=lambda: Pose2D(), now=lambda: clock() + 90.)
        grid = OccupancyGrid(cfg.arena)
        navigation = SimpleNamespace(initialized=True, pose=Pose2D(.34, 2.4), grid=grid,
                                     update=lambda *args: None, clear_goal=lambda: None)
        agent = Agent(robot, transport, cfg=cfg, stack=navigation)
        agent.tracker = SimpleNamespace(update_uav=lambda frame, pose, **kwargs: None)
        agent._test_logs = []
        agent._log = lambda now, msg: agent._test_logs.append((now, msg))
        made.append(agent)
        return SimpleNamespace(agent=agent, cfg=cfg, clock=clock, mock=mock, link=link, transport=transport, robot=robot, grid=grid)
    yield build
    for agent in made:
        try:
            agent.close_deadlock()
        except Exception:                                                        # noqa: BLE001
            pass


def logs(a):
    return [m for _, m in a._test_logs]


def test_manager_m1_sectors_use_server_names_and_are_empty_unless_enabled(make_agent):
    a = make_agent(Config()).agent
    a.cmd = Command(target="RADAR_FACILITY", sectors=["B-S2", "COMMON", "CHARLIE", "r-s1"])
    assert a.cfg.explore.m1_sector_sweep is False and a._m1_sectors() == set()        # 코드 기본값은 꺼짐
    a.cfg.explore.m1_sector_sweep = True
    assert a._m1_sectors() == {"B-S2", "CM", "R-S1"}                                   # 옛 NATO 이름은 서버에 없어 버린다
    a.cmd = None
    assert a._m1_sectors() == set()
    a.cmd = Command(sectors=[])
    assert a._m1_sectors() == set()


def test_scoring_faces_include_the_m1_sectors_so_progress_watches_them(make_agent):
    cfg = _measured_cfg()
    cfg.explore.m1_sector_sweep = False
    a = make_agent(cfg).agent
    own = BoxModel(1, 1.4, 2.0, "full", [], 0., 0.)
    a.coverage._make_faces(own)
    a.coverage.boxes = {1: own}
    assert cfg.arena.sector_of(1.4, 2.0) is not None and server_sector(cfg.arena.sector_of(1.4, 2.0)) not in scoring_sectors(a.team)
    assert a._scoring_faces() == []
    cfg.explore.m1_sector_sweep = True
    a.cmd = Command(sectors=[server_sector(cfg.arena.sector_of(1.4, 2.0))])
    assert {f.bid for f in a._scoring_faces()} == {1}


def _seed_ledger(a, sector, n_boxes=2):
    cov = a.coverage
    cov.boxes = {}
    cx, cy = a.cfg.arena.sector_center(sector)
    for i in range(n_boxes):
        b = BoxModel(i + 1, cx + 0.7 * i, cy, "full", [], 0., 0., full_seen=3, last_full_t=1.)
        cov._make_faces(b)
        cov.boxes[b.bid] = b
    return cov


def test_m1_sweep_report_tracks_progress_logs_changes_and_completion_once(make_agent):
    a = make_agent().agent
    sector = server_sector(a.cfg.arena.sector_names()[0])
    cov = _seed_ledger(a, a.cfg.arena.sector_names()[0])
    a.cmd = Command(sectors=[sector])
    a._m1_sweep_report(100.0)
    s = a.stats["m1_sweep"]
    assert s["sectors"] == [sector] and s["boxes"] == 2 and s["faces"] == 8 and s["done"] == 0 and s["todo"] == 8
    assert s["complete"] is False and s["per_sector"][sector]["pending"] == 8
    assert sum("M1 섹터 면 확인" in m for m in logs(a)) == 1
    a._m1_sweep_report(101.0)                                                          # 변화 없음 → 로그 중복 없음
    assert sum("M1 섹터 면 확인" in m for m in logs(a)) == 1
    first = cov.boxes[1]
    for f in first.faces:
        resolve(f)
    a._m1_sweep_report(102.0)                                                          # 변했지만 5 s 이내 → 로그는 아직
    assert a.stats["m1_sweep"]["done"] == 4 and sum("M1 섹터 면 확인" in m for m in logs(a)) == 1
    a._m1_sweep_report(110.0)
    assert sum("M1 섹터 면 확인" in m for m in logs(a)) == 2
    for f in cov.boxes[2].faces:
        resolve(f)
    a._m1_sweep_report(120.0)
    assert a.stats["m1_sweep"]["complete"] is True and a.stats["m1_sweep"]["done"] == 8
    assert sum("보이는 면을 모두 확인했다" in m for m in logs(a)) == 1
    a._m1_sweep_report(130.0)
    assert sum("보이는 면을 모두 확인했다" in m for m in logs(a)) == 1                  # 완료 로그는 한 번


def test_m1_sweep_report_is_silent_without_sectors_and_never_raises(make_agent):
    a = make_agent().agent
    a._m1_sweep_report(1.0)
    assert "m1_sweep" not in a.stats
    a.cfg.explore.m1_sector_sweep = True
    a.cmd = Command(sectors=["R-S1"])
    a.coverage.sector_progress = lambda sectors: (_ for _ in ()).throw(RuntimeError("boom"))
    a._m1_sweep_report(2.0)                                                            # 보조 기능이 제어 루프를 죽이지 않는다
    assert a.stats["m1_sweep_errors"] == 1 and any("M1 섹터 보고 실패" in m for m in logs(a))


# ====================================================================== 끝에서 끝까지: 영상 → 상자 → 섹터 → M1 → 식별
def test_real_detector_to_ledger_to_m1_identification(make_agent):
    pytest.importorskip("cv2")
    from maicon26.perception.uav_boxes_real import load_real_uav_detector
    from test_uav_boxes_real import render, jpeg

    cfg = _measured_cfg()
    detector = load_real_uav_detector(str(PACKAGE / "config/uav_grid_calibration.json"), cfg.arena, cfg.uav_boxes, camera="main")
    rsec = "R-S2"
    placed = [sector_point(cfg, rsec, -0.5, 0.0), sector_point(cfg, rsec, 0.5, 0.5), sector_point(cfg, "B-S1"), sector_point(cfg, "COMMON", 0.0, 0.6)]
    w = make_agent(cfg, detector)
    a, clock, mock, transport, robot, grid = w.agent, w.clock, w.mock, w.transport, w.robot, w.grid
    image = jpeg(render(placed))
    for index in range(2):
        if index:
            clock.advance(1.)
        mock.capture(image)
        transport._uav_once()
        assert transport.get_uav().layout_source == "real_uav_image"
        assert a._sense(robot.now(), .05)
    cov = a.coverage
    assert len(cov.boxes) == 4 and len(cov.registry) == 16
    ledger = cov.sector_ledger()
    assert ledger[server_sector(rsec)]["boxes"] == 2 and ledger["B-S1"]["boxes"] == 1 and ledger["CM"]["boxes"] == 1
    # M1 이 상대 진영 한 섹터를 지정했다
    a.cmd = Command(target="RADAR_FACILITY", sectors=[server_sector(rsec)])
    a._m1_sweep_report(100.0)
    s = a.stats["m1_sweep"]
    assert s["boxes"] == 2 and s["faces"] == 8 and s["todo"] == 8 and s["done"] == 0 and s["complete"] is False
    # 그 섹터의 면을 모두 확인·분류(여기서는 장부에 직접 득표를 넣는다) → 면 식별로 등록
    for b in cov.boxes.values():
        if server_sector(cov.box_sector(b)) == server_sector(rsec):
            for f in b.faces:
                resolve(f, "TANK")
    cov.refresh(grid, None, 110.0, force=True)
    a._m1_sweep_report(111.0)
    s = a.stats["m1_sweep"]
    assert s["done"] == 8 and s["todo"] == 0 and s["complete"] is True
    by = cov.registry_by_sector()
    assert by[server_sector(rsec)]["identified"] == 8 and by["B-S1"]["pending"] == 4 and by["CM"]["pending"] == 4
    assert any("보이는 면을 모두 확인했다" in m for m in logs(a))
    # 식별한 면은 다시 보러 가지 않는다 — 남은 면은 다른 섹터(지정하지 않은) 것뿐
    assert {server_sector(cov.box_sector(cov.boxes[f.bid])) for f in cov.pending_faces(None)} == {"B-S1", "CM"}


def test_m1_sector_faces_outside_the_scoring_territory_never_reach_the_m3_submission(make_agent):
    a = make_agent().agent
    cov = a.coverage
    own, opp = "B-S2", "R-S2"                                                          # 팀 BLUE: 채점 구역은 R-S1~3 + CM
    assert server_sector(own) not in scoring_sectors(a.team) and server_sector(opp) in scoring_sectors(a.team)
    cov.boxes = {}
    for bid, sector in ((1, own), (2, opp)):
        x, y = sector_point(a.cfg, sector)
        b = BoxModel(bid, x, y, "full", [], 0., 0., full_seen=3, last_full_t=1.)
        cov._make_faces(b)
        cov.boxes[bid] = b
        for f in b.faces:
            f.det_n, f.det_votes, f.det_kind = 6, {"TANK": 6.0}, "object"
    a.cmd = Command(sectors=[own, opp])
    counts = a._m3_counts()
    assert set(counts) == {"R-S1", "R-S2", "R-S3", "COMMON"}                          # 제출에는 채점 구역만
    assert counts["R-S2"]["TANK"] == 4 and sum(sum(v.values()) for v in counts.values()) == 4
    assert cov.sector_ledger([own])[own]["identified"] == 4                            # 장부·기록부에는 M1 섹터의 식별이 남는다
