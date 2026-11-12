"""All exposed scoring faces remain accountable; camera evidence is still required."""
import math
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.link.base import UavBox
from maicon26.mission.explore_online import Cand, OnlineExplorer
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import CoverageMap, EMPTY, CONFIRMED
from maicon26.types import Pose2D


def scene(rows=((4.2, 4.2),), team="BLUE", wingbot=False):
    cfg = Config()
    cfg.team = team
    cfg.explore.exhaustive_scoring_faces = True
    cfg.explore.uav_outer_faces = True
    if wingbot:
        cfg.robot.length, cfg.robot.width, cfg.robot.cam_height = .2598484, .2404398, .202726
        cfg.camera.K = (205.469637, 0., 320., 0., 205.469637, 240., 0., 0., 1.)
        cfg.camera.hfov_deg = 114.591559
    cov, grid = CoverageMap(cfg), OccupancyGrid(cfg.arena)
    for t in (1., 2., 3.):
        cov.ingest_uav_boxes([UavBox(x, y, .4, .4) for x, y in rows], t, grid)
    cov.refresh(grid, None, 4., force=True)
    return cfg, cov, grid


def test_wall_gap_is_not_contact_and_raw_geometry_survives_local_drift():
    _, cov, _ = scene()
    box = next(iter(cov.boxes.values()))
    north = box.faces[1]
    assert not cov._face_on_wall(box, 1)  # 0.4 m passage, not attached to wall.
    assert north in cov.pending_faces()
    box.cy += .7
    box.sy = 1.2
    assert not cov._face_on_wall(box, 1)
    box.arena_cy = cov.geo.H - .2
    assert cov._face_on_wall(box, 1)


def test_lidar_internal_cannot_erase_a_confirmed_aerial_outer_side():
    _, cov, _ = scene(((3., 2.2), (3., 2.6)))
    box = min(cov.boxes.values(), key=lambda b: b.cy)
    cov._apply_unit(box, SimpleNamespace(gid=1, closed=True, internal=[0, 1]))
    cov._update_uav_internal()
    assert box.lidar_internal == [0, 1]
    assert box.internal == [1]  # Real touching north side only.
    assert box.faces[0] in cov.pending_faces()
    assert not any(f.det_n for f in box.faces)


@pytest.mark.parametrize("team,x", [("BLUE", 4.2), ("RED", 1.8)])
def test_failed_face_remains_unfinished_and_retryable(team, x):
    _, cov, _ = scene(((x, 2.4),), team=team)
    box = next(iter(cov.boxes.values()))
    f = box.faces[0]
    f.arrive_attempts = 12
    f.exterior_open = False
    assert f in cov.pending_faces()
    assert cov.face_weight(f) >= .65
    row = cov.sector_ledger()[cov.box_sector(box)]
    assert list(f.key) in row["todo"] and not row["complete"]
    assert cov.face_state(f) not in (EMPTY, CONFIRMED)


def test_clipped_picture_cannot_accumulate_empty_evidence(monkeypatch):
    _, cov, grid = scene(((4.2, 2.4),))
    box = next(iter(cov.boxes.values()))
    # Isolate the important distinction: centre visible but image clipped.
    monkeypatch.setattr(cov, "face_visible", lambda *a, **kw: (True, "ok"))
    monkeypatch.setattr(cov, "picture_visible", lambda *a, **kw: (False, "fov"))
    for i in range(6):
        cov.record_view(Pose2D(4.8, 2.4 + .1*i, math.pi), 0., grid, 5.+i)
    assert all(f.views == 0 and cov.face_state(f) != EMPTY for f in box.faces)


@pytest.mark.parametrize("wingbot", [False, True])
def test_wall_side_has_oblique_collision_checked_viewpoints(wingbot):
    cfg, cov, grid = scene(((4.1984375, 4.1984375), (4.6, 4.1984375)), wingbot=wingbot)
    exp = OnlineExplorer(cfg, cov)
    box = next(iter(cov.boxes.values()))
    blocked, _ = cov.planning_cost_maps(grid)
    pose = Pose2D(3., 3.8, 0.)
    cands = exp.build(pose, grid, blocked, 5., team="BLUE")
    north = [c for c in cands if c.primary_face == box.faces[1].key]
    assert north, "A 40 cm wall corridor must be considered, with oblique camera views."
    occ = cov.visibility_occupancy(grid)
    for c in north:
        assert exp.view_body_heading(c.vp, c.heading, pose.th, occ, grid) is not None
        assert cov.picture_visible(*c.vp, c.heading, box.faces[1], occ, grid)[0]


def test_self_raster_suppression_does_not_remove_a_foreground_obstacle():
    from maicon26.perception.coverage import BoxModel
    cfg, cov, grid = scene(((4.1984375, 4.1984375),), wingbot=True)
    b = next(iter(cov.boxes.values()))
    f = b.faces[1]
    vp = (f.cx-.4, f.cy+.225)
    heading = math.atan2(f.cy-vp[1], f.cx-vp[0])
    assert cov.picture_visible(*vp, heading, f, cov.visibility_occupancy(grid), grid)[0]
    # A narrow foreground obstruction on the left picture edge remains blocking.
    px, py = f.cx-.18, f.cy+.002
    blocker = BoxModel(99, (vp[0]+px)/2, (vp[1]+py)/2, "full", [], 1., 4., sx=.06, sy=.06)
    cov.boxes[99] = blocker
    assert cov._picture_edge_occluded(*vp, px, py, f)
    assert cov.picture_visible(*vp, heading, f, cov.visibility_occupancy(grid), grid) == (False, "occluded")


def test_sector_commit_finishes_local_work_then_services_another_sector():
    cfg, cov, _ = scene(((4.2, 4.2), (3., 2.4)))
    exp = OnlineExplorer(cfg, cov)
    a, b = sorted(cov.boxes.values(), key=lambda b: -b.cx)
    # Viewpoint in CM still belongs to the observed enemy box's sector.
    first = Cand("face", (3., 3.8), 0., "look", primary_face=a.faces[0].key, utility=2.)
    other = Cand("face", (3., 1.6), 0., "look", primary_face=b.faces[0].key, utility=1.)
    assert exp.candidate_sector(first) == cov.box_sector(a)
    exp._now = 5.
    assert exp.choose([first, other], None) is first
    exp._now += cfg.explore.coverage_sector_commit_s + 1.
    assert exp.choose([first, other], first) is other


def test_m2_time_is_reserved_for_unresolved_faces_without_declaring_failure():
    from maicon26.mission.manager import Agent
    agent = Agent.__new__(Agent)
    agent.cfg, agent.coverage, _ = scene()
    agent.team = "BLUE"
    agent.target_failed = False
    agent.stats = {"mode_time": {"STRIKE": 29.}}
    assert not agent._coverage_defers_m2()
    agent.stats["mode_time"]["STRIKE"] = 31.
    assert agent._coverage_defers_m2()
    assert not agent.target_failed
    agent.coverage.sector_progress = lambda sectors: {"complete": True}
    assert not agent._coverage_defers_m2()


def test_unfinished_coverage_uses_match_time_but_preserves_goal_reserve():
    from maicon26.mission.manager import Agent
    a = Agent.__new__(Agent)
    a.cfg, a.coverage, _ = scene()
    a.team = "BLUE"
    assert a._continue_scoring_coverage(240.)
    assert a._continue_scoring_coverage(274.)
    assert not a._continue_scoring_coverage(275.)
    a.coverage.sector_progress = lambda sectors: {"complete": True}
    assert not a._continue_scoring_coverage(240.)


def test_excluded_faces_cannot_return_via_anchored_count_fallback():
    _, cov, _ = scene(((4.2, 2.4), (4.6, 2.4)))
    b = min(cov.boxes.values(), key=lambda b: b.cx)
    hidden = b.faces[0]
    assert hidden.idx in b.internal
    for box in cov.boxes.values():
        for f in box.faces:
            f.det_kind, f.det_n, f.det_votes = "object", 6, {"TANK": 6.}
    assert len(cov.m3_entries(None)) == 6
    old = SimpleNamespace(x=hidden.cx,y=hidden.cy,cx=b.cx,cy=b.cy,vx=1.,vy=0.,
                          geometry_key=hidden.key,n=6,votes={"TANK":6.},cls="TANK",kind="object")
    omap = SimpleNamespace(faces=[old],confirmed_faces=lambda: [old])
    cov._sync_detections(omap)
    assert hidden.det_n == 0
    assert cov.m3_entries(omap) == []


def test_aerial_cube_keeps_size_when_a_lidar_cluster_merges_edges(monkeypatch):
    _, cov, grid = scene()
    box = next(iter(cov.boxes.values()))
    monkeypatch.setattr(cov, "_clusters", lambda g: [("full",4.3,4.25,[],.6,.5)])
    cov._cl_units = []
    cov.refresh(grid,None,5.,force=True)
    assert (box.sx,box.sy) == pytest.approx((.4,.4))
    assert (box.cx,box.cy) == pytest.approx((box.arena_cx,box.arena_cy))


def test_partial_lidar_corner_is_not_an_extra_cube_when_aerial_map_is_available():
    from maicon26.perception.coverage import BoxModel
    _, cov, _ = scene()
    partial = BoxModel(99,4.6,3.8,"face",[],1.,4.)
    assert not cov.m3_candidate_box(partial)
    partial.kind = "full"
    assert not cov.m3_candidate_box(partial)
    _, no_uav, _ = scene(rows=())
    partial.kind = "face"
    assert no_uav.m3_candidate_box(partial), "No-UAV fallback retains unclosed hypotheses."
