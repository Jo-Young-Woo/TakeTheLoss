"""UAV 장애물 지도: ① 첫 영상 지도를 끝까지 쓴다(explore.uav_map_freeze) ② 맞닿은 면은 장부·기록부에 올리지 않는다(uav_outer_faces)
③ UGV 로 검출된 자리의 상자 후보는 지도에서 뺀다(uav_exclude_ugv). 시뮬을 쓰지 않는다."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.coverage import CoverageMap
from maicon26.validate import sanitize_config

ROOT = Path(__file__).resolve().parents[1]


def box(x, y, sx=0.4, sy=0.4):
    return SimpleNamespace(cx=x, cy=y, sx=sx, sy=sy, confidence=0.95)


def make(freeze=False, outer=False, ugv=False, **over):
    cfg = Config()
    cfg.explore.uav_face_registry = True
    cfg.explore.uav_census = True
    cfg.explore.uav_map_freeze, cfg.explore.uav_outer_faces, cfg.explore.uav_exclude_ugv = freeze, outer, ugv
    for k, v in over.items():
        setattr(cfg.explore, k, v)
    return cfg, CoverageMap(cfg), OccupancyGrid(cfg.arena)


def feed(cov, boxes, t0=1.0, n=3, step=1.0, grid=None, **kw):
    for i in range(n):
        cov.ingest_uav_boxes(boxes, t0 + i * step, grid, **kw)


def ledger_faces(cov):
    return sum(r["faces"] for r in cov.sector_ledger().values())


# ====================================================================== 기본값·설정
def test_defaults_are_off_and_real_config_turns_all_three_on():
    ec = Config().explore
    assert (ec.uav_map_freeze, ec.uav_outer_faces, ec.uav_exclude_ugv) == (False, False, False)
    real = Config.load(str(ROOT / "config/waveshare_real.json")).explore
    assert (real.uav_map_freeze, real.uav_outer_faces, real.uav_exclude_ugv) == (True, True, True)


def test_validate_rejects_out_of_range_values():
    cfg = Config()
    cfg.explore.uav_ugv_exclude_m = 0.40            # 맞닿은 상자 중심 간격(0.325 m)을 넘는다
    cfg.explore.uav_map_freeze_s = 0.0
    cfg.explore.uav_map_freeze_min_boxes = 0
    cfg.explore.uav_adjacent_tol_m = 0.5
    log = sanitize_config(cfg)
    ec = cfg.explore
    assert ec.uav_ugv_exclude_m == 0.25 and ec.uav_map_freeze_s == 4.0 and ec.uav_map_freeze_min_boxes == 1 and ec.uav_adjacent_tol_m == 0.14


# ====================================================================== ② 최외곽 면만
@pytest.mark.parametrize("name,boxes,old_faces,outer_faces", [
    ("단독", [(2.0, 2.0)], 4, 4),
    ("맞닿은 2개", [(2.0, 2.0), (2.4, 2.0)], 8, 6),
    ("L 3개", [(2.0, 2.0), (2.4, 2.0), (2.0, 2.4)], 12, 8),
    ("2x2", [(2.0, 2.0), (2.4, 2.0), (2.0, 2.4), (2.4, 2.4)], 16, 8),
    ("한 칸 띄운 2개", [(2.0, 2.0), (2.8, 2.0)], 8, 8),
])
def test_ledger_registry_and_census_hold_only_outer_faces(name, boxes, old_faces, outer_faces):
    for outer, want in ((False, old_faces), (True, outer_faces)):
        cfg, cov, grid = make(outer=outer)
        feed(cov, [box(*p) for p in boxes])
        cov.refresh(grid, None, 5.0, force=True)
        assert ledger_faces(cov) == want, (name, outer)
        assert cov.registry_counts()["registered"] == want, (name, outer)
        assert cov.uav_map_report()["undetected"] == want, (name, outer)


def test_outer_mode_marks_both_coincident_faces_internal_and_skips_them_in_planning():
    cfg, cov, grid = make(outer=True)
    feed(cov, [box(2.0, 2.0), box(2.4, 2.0)])
    cov.refresh(grid, None, 5.0, force=True)
    a, b = sorted(cov.boxes.values(), key=lambda x: x.cx)
    assert a.internal == [0] and b.internal == [2]                                 # 서로 맞닿은 E / W 면 — 둘 다 뺀다
    keys = {f.key for f in cov.pending_faces()}
    assert (a.bid, 0) not in keys and (b.bid, 2) not in keys and len(keys) == 6


def test_a_neighbour_that_appears_later_removes_the_unresolved_inner_face_but_keeps_resolved_ones():
    cfg, cov, grid = make(outer=True)
    feed(cov, [box(2.0, 2.0)], t0=1.0, n=2)
    cov.refresh(grid, None, 3.0, force=True)
    a = next(iter(cov.boxes.values()))
    a.faces[0].det_n, a.faces[0].det_votes, a.faces[0].det_kind = 6, {"TANK": 6.0}, "object"      # E 면을 이미 식별했다
    cov.refresh(grid, None, 4.0, force=True)
    assert cov.registry_counts()["identified"] == 1 and cov.registry_counts()["registered"] == 4
    feed(cov, [box(2.0, 2.0), box(2.4, 2.0)], t0=5.0, n=3)                         # 이웃이 나중에 보인다(E 면이 가려진다)
    cov.refresh(grid, None, 9.0, force=True)
    assert cov.registry_counts()["identified"] == 1                                # 식별한 면의 기록은 지우지 않는다
    b = next(x for x in cov.boxes.values() if x is not a)
    assert 2 in b.internal and cov.registry_stats["inner_dropped"] >= 1            # 이웃의 W 면(미해결)은 빠졌다


def test_old_behaviour_when_the_flag_is_off():
    cfg, cov, grid = make(outer=False)
    feed(cov, [box(2.0, 2.0), box(2.4, 2.0)])
    cov.refresh(grid, None, 5.0, force=True)
    assert cov.registry_counts()["registered"] == 8 and all(not b.uav_internal for b in cov.boxes.values())


# ====================================================================== ① 첫 지도를 끝까지
def test_without_freeze_uav_only_boxes_drop_out_after_the_video_stops():
    cfg, cov, grid = make(freeze=False)
    feed(cov, [box(2.0, 2.0), box(3.0, 2.5)])
    cov.refresh(grid, None, 3.0, force=True)
    assert len(cov.pending_faces()) == 8
    cov.refresh(grid, None, 30.0, force=True)                                      # 영상이 12 s 넘게 끊겼다
    assert cov.pending_faces() == [] and not cov.uav_map_ready()


def test_frozen_map_survives_a_long_video_outage():
    cfg, cov, grid = make(freeze=True)
    feed(cov, [box(2.0, 2.0), box(3.0, 2.5)], t0=1.0, n=3)                        # t=1,2,3 — 창(4 s)은 t=2 부터
    cov.refresh(grid, None, 3.5, force=True)
    assert not cov._map_frozen                                                      # 창이 아직 안 지났다
    cov.refresh(grid, None, 7.0, force=True)                                        # tick 이 창 종료를 본다(영상 없이도)
    assert cov._map_frozen and cov.map_stats["frozen_boxes"] == 2
    cov.refresh(grid, None, 300.0, force=True)                                      # 5 분 뒤
    assert len(cov.pending_faces()) == 8 and cov.uav_map_ready()
    assert all(cov.box_available(b) for b in cov.boxes.values())
    diag = cov.layout_diagnostics()
    assert diag["map_frozen"] is True and diag["frozen_boxes"] == 2


def test_frozen_map_ignores_later_frames_and_counts_them():
    cfg, cov, grid = make(freeze=True)
    feed(cov, [box(2.0, 2.0), box(3.0, 2.5)], t0=1.0, n=3)
    feed(cov, [box(2.05, 2.02), box(3.0, 3.4), box(4.0, 1.0)], t0=8.0, n=3, grid=grid)   # 창이 지난 뒤: 하나는 조금 이동, 둘은 새 자리
    assert cov._map_frozen and len(cov.boxes) == 2
    first = sorted((round(b.cx, 2), round(b.cy, 2)) for b in cov.boxes.values())
    assert first == [(2.0, 2.0), (3.0, 2.5)]                                        # 위치도 그대로
    assert cov.map_stats["late_boxes_ignored"] >= 2


def test_freeze_waits_for_the_minimum_box_count():
    cfg, cov, grid = make(freeze=True, uav_map_freeze_min_boxes=3)
    feed(cov, [box(2.0, 2.0), box(3.0, 2.5)], t0=1.0, n=2)
    cov.refresh(grid, None, 10.0, force=True)
    assert not cov._map_frozen                                                      # 상자 2 < 3 — 계속 모은다
    feed(cov, [box(2.0, 2.0), box(3.0, 2.5), box(4.0, 1.0)], t0=10.5, n=3)
    cov.refresh(grid, None, 20.0, force=True)
    assert cov._map_frozen and len(cov.boxes) == 3


def test_lidar_can_still_remove_a_frozen_box_it_sees_through():
    cfg, cov, grid = make(freeze=True)
    feed(cov, [box(2.0, 2.0), box(4.0, 1.0)], t0=1.0, n=3, grid=grid)
    cov.refresh(grid, None, 7.0, force=True)
    assert cov._map_frozen and len(cov.boxes) == 2
    iy0, ix0 = grid.to_cell(1.5, 1.5)
    iy1, ix1 = grid.to_cell(2.5, 2.5)
    grid.L[iy0:iy1 + 1, ix0:ix1 + 1] = -2.0                                         # LiDAR 가 그 자리를 반복해서 빈 공간으로 봤다
    for k in range(6):
        cov.refresh(grid, None, 20.0 + 1.0 * k, force=True)
    # 'full' 로 확인된 적 없는 가설 상자라 기록부 기록은 지우지 않지만 상자 자체는 지도에서 빠진다
    assert all(abs(b.cx - 2.0) > 0.3 for b in cov.boxes.values())
    assert any(abs(b.cx - 4.0) < 0.1 for b in cov.boxes.values())                   # 건드리지 않은 상자는 남는다


def test_freeze_off_keeps_updating_with_every_frame_like_before():
    cfg, cov, grid = make(freeze=False)
    feed(cov, [box(2.0, 2.0)], t0=1.0, n=3)
    feed(cov, [box(2.0, 2.0), box(4.0, 1.0)], t0=8.0, n=3)
    assert len(cov.boxes) == 2 and not cov._map_frozen


def test_fresh_frozen_rgb_footprint_survives_conflicting_lidar_then_can_be_removed():
    cfg, cov, grid = make(freeze=True)
    feed(cov, [box(2., 2.)], grid=grid)
    cov.refresh(grid, None, 7., force=True)
    bid = next(iter(cov.boxes))
    grid.L[:] = -2.
    for t in range(20, 26):
        cov.ingest_uav_boxes([box(2., 2.)], float(t), grid)
        cov.refresh(grid, None, float(t), force=True)
    assert bid in cov.boxes
    for t in range(40, 46):
        cov.ingest_uav_boxes([], float(t), grid)
        cov.refresh(grid, None, float(t), force=True)
    assert bid not in cov.boxes
    # Only a twice-reacquired original footprint returns. Unknown new footprints
    # stay excluded and no side label is manufactured from overhead geometry.
    cov.ingest_uav_boxes([box(2., 2.), box(4., 2.)], 50., grid)
    assert bid not in cov.boxes
    cov.ingest_uav_boxes([box(2., 2.), box(4., 2.)], 51., grid)
    assert set(cov.boxes) == {bid}
    assert all(f.det_n == 0 and not f.det_votes for f in cov.boxes[bid].faces)
    assert cov.map_stats['frozen_reacquired'] == 1


# ====================================================================== ③ UGV 제외
def test_ugv_detection_removes_box_candidates_on_top_of_it():
    cfg, cov, grid = make(ugv=True)
    boxes = [box(2.0, 2.0), box(3.0, 2.0)]
    feed(cov, boxes, ugv_xy=[(3.05, 2.05)])                                         # 두 번째 후보는 UGV 윗면이었다
    assert [round(b.cx, 1) for b in cov.boxes.values()] == [2.0] and cov.map_stats["ugv_excluded"] == 3


def test_a_box_touching_a_ugv_stays():
    cfg, cov, grid = make(ugv=True)
    feed(cov, [box(2.0, 2.0)], ugv_xy=[(2.325, 2.0)])                               # 맞닿은 UGV(중심 0.325 m)
    assert len(cov.boxes) == 1


def test_ugv_exclusion_is_off_by_default_and_ignores_empty_input():
    cfg, cov, grid = make(ugv=False)
    feed(cov, [box(2.0, 2.0), box(3.0, 2.0)], ugv_xy=[(3.0, 2.0)])
    assert len(cov.boxes) == 2
    cfg, cov, grid = make(ugv=True)
    feed(cov, [box(2.0, 2.0)], ugv_xy=None)
    feed(cov, [box(2.0, 2.0)], t0=10.0, ugv_xy=[])
    assert len(cov.boxes) == 1


def test_ugv_excluded_hint_does_not_enter_the_ledger_or_the_registry():
    cfg, cov, grid = make(ugv=True, outer=True)
    feed(cov, [box(2.0, 2.0), box(3.0, 2.0)], ugv_xy=[(3.0, 2.0)])
    cov.refresh(grid, None, 5.0, force=True)
    assert ledger_faces(cov) == 4 and cov.registry_counts()["registered"] == 4


# ====================================================================== 연결
def test_manager_passes_uav_ugv_positions_to_the_coverage_map():
    src = (ROOT / "maicon26/mission/manager.py").read_bytes().decode("utf-8-sig")
    assert "ugv_xy=ugv_xy" in src and "uav_exclude_ugv" in src
    ex = (ROOT / "maicon26/mission/explore_online.py").read_text(encoding="utf-8")
    assert ex.count("uav_map_ready()") == 2 and 'uav_layout_state != "available"' not in ex


# ====================================================================== ② 보강: 엇갈려 맞닿은 면·벽에 붙은 면·긴 블록
def _counts(boxes, **over):
    cfg, cov, grid = make(outer=True, **over)
    feed(cov, [box(*p) if len(p) == 2 else box(*p) for p in boxes])
    cov.refresh(grid, None, 5.0, force=True)
    return cov, ledger_faces(cov), cov.registry_counts()["registered"], cov.uav_map_report()["undetected"], len(cov.pending_faces())


def test_half_cell_staggered_neighbours_hide_each_other_partly():
    # (1.6,3.8) 의 E 면(y 3.6..4.0)은 (2.0,3.6)·(2.0,4.0) 두 칸이 각각 반씩 가린다 — 일부만 가려도 안쪽 면이다
    cov, led, reg, cen, pend = _counts([(1.6, 3.8), (2.0, 3.6), (2.0, 4.0)])
    assert led == reg == cen == pend
    a = next(b for b in cov.boxes.values() if abs(b.cx - 1.6) < 0.01)
    assert 0 in a.internal
    assert led == 7                                                                 # 12 면 중 5면(좌 칸 E, 우 두 칸 W, 우 두 칸끼리 N/S)이 빠진다


def test_a_face_only_grazed_by_less_than_the_cover_fraction_stays():
    cfg, cov, grid = make(outer=True, uav_cover_frac=0.2)
    feed(cov, [box(2.0, 2.0), box(2.4, 2.35)])                                      # 옆으로 0.35 어긋나 0.05/0.4 = 12 % 만 닿는다
    cov.refresh(grid, None, 5.0, force=True)
    a = next(b for b in cov.boxes.values() if abs(b.cy - 2.0) < 0.01)
    assert 0 not in a.internal


def test_wall_adjacent_faces_are_not_recorded_anywhere():
    cfg, cov, grid = make(outer=True)
    feed(cov, [(box(3.06, 4.40))])
    cov.refresh(grid, None, 5.0, force=True)
    b = next(iter(cov.boxes.values()))
    # 이 위치는 경기장 안쪽 끝에 붙어 있다 — N 면은 카메라가 들어갈 자리가 없다
    assert 1 in b.internal
    assert ledger_faces(cov) == cov.registry_counts()["registered"] == cov.uav_map_report()["undetected"] == len(cov.pending_faces()) == 3


def test_long_block_is_not_registered_but_still_hides_a_neighbours_face():
    cfg, cov, grid = make(outer=True)
    feed(cov, [box(2.0, 2.0, 0.4, 0.8), box(2.4, 1.8)])                              # 0.4×0.8 블록 + 그 옆 정육면체(블록 y 1.6..2.4 에 박스 y 1.6..2.0)
    cov.refresh(grid, None, 5.0, force=True)
    cube = next(b for b in cov.boxes.values() if b.sx < 0.5 and b.sy < 0.5)
    assert 2 in cube.internal                                                       # 정육면체의 W 면은 블록에 가린다
    assert cov.registry_counts()["registered"] == 3 == ledger_faces(cov) == len(cov.pending_faces())   # 블록 자신의 면은 M3 대상이 아니다
    assert cov.uav_map_report()["undetected"] == 3


def test_cover_fraction_config_is_validated():
    cfg = Config()
    cfg.explore.uav_cover_frac = 2.0
    sanitize_config(cfg)
    assert cfg.explore.uav_cover_frac == 0.2                                         # 범위 밖이면 기본값으로 되돌린다


# ====================================================================== ② 보강 2: 안쪽 면은 관측·득표·상태에서도 제외
def _two_touching(outer):
    cfg, cov, grid = make(outer=outer)
    feed(cov, [box(2.0, 2.0), box(2.4, 2.0)])
    cov.refresh(grid, None, 5.0, force=True)
    a, b = sorted(cov.boxes.values(), key=lambda x: x.cx)
    return cfg, cov, grid, a, b


def test_views_are_not_credited_to_internal_faces_and_cannot_make_them_empty():
    cfg, cov, grid, a, b = _two_touching(True)
    cov.face_visible = lambda *args, **kw: (True, "ok")                             # 가림 판정이 어긋나도(반쯤 가려진 면) 안쪽 면에는 쌓지 않는다
    from maicon26.types import Pose2D
    for i in range(8):
        cov.record_view(Pose2D(2.2 + 0.2 * i, 1.0, 1.57), 0.0, grid, 10.0 + 2.0 * i)
    assert a.faces[0].views == 0 and b.faces[2].views == 0                          # 서로 맞댄 E / W 면
    assert a.faces[1].views > 0                                                      # 바깥 면은 센다
    assert cov.face_state(a.faces[0]) != "empty" and cov.face_ledger_state(a, a.faces[0]) == "excluded"
    cfg2, cov2, grid2, a2, b2 = _two_touching(False)                                # 끄면 기존 동작: 센다
    cov2.face_visible = lambda *args, **kw: (True, "ok")
    for i in range(8):
        cov2.record_view(Pose2D(2.2 + 0.2 * i, 1.0, 1.57), 0.0, grid2, 10.0 + 2.0 * i)
    assert a2.faces[0].views > 0


def test_ledger_state_excluded_wins_over_identified_and_counts_in_ledger_row():
    cfg, cov, grid, a, b = _two_touching(True)
    a.faces[0].det_n, a.faces[0].det_votes, a.faces[0].det_kind = 6, {"TANK": 6.0}, "object"
    assert cov.face_ledger_state(a, a.faces[0]) == "excluded"                       # 식별로 세지 않는다
    row = cov.sector_ledger()
    assert sum(r["excluded"] for r in row.values()) == 2 and sum(r["faces"] for r in row.values()) == 6
    assert sum(r["identified"] for r in row.values()) == 0


def test_detection_votes_skip_internal_faces():
    cfg, cov, grid, a, b = _two_touching(True)
    of = SimpleNamespace(cx=2.2, cy=2.0, x=2.2, y=2.0, vx=-1.0, vy=0.0, n=6, votes={"TANK": 6.0}, kind="object", geometry_key=None)
    cov._sync_detections(SimpleNamespace(faces=[of]))
    assert a.faces[0].det_n == 0 and b.faces[2].det_n == 0                           # 맞닿은 자리의 표는 안쪽 면에 붙지 않는다
