"""2D LiDAR 면 지도(perception/lidar_faces.py). 시뮬 없이 numpy 광선 투사로 만든 장면을 실제 OccupancyGrid 에 먹여 검증한다.

장면 규칙: 장애물은 40 cm 정육면체(축 정렬)를 이어 붙인 것, UGV 는 LiDAR 평면에서 25 cm 상자 하나(윗부분만 보인다고 가정)."""
import math

import numpy as np
import pytest

from lidar_scene import C, around, cube, make_world, run
from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.lidar_faces import LidarFaceMap, ugv_points_from


# ---------------------------------------------------------------------------------------------- 단일 정육면체
def test_single_cube_four_faces_is_closed_and_one_unit():
    res, *_ = run([(3.0, 2.5)], around(3.0, 2.5))
    assert len(res.groups) == 1
    g = res.groups[0]
    assert g.closed is True
    assert len(res.units) == 1
    u = res.units[0]
    assert math.hypot(u.cx - 3.0, u.cy - 2.5) < 0.08
    assert set(u.faces) == {0, 1, 2, 3} and u.kind == "full"
    assert not u.internal
    assert all(0.36 <= f.length <= 0.50 for f in res.faces)


def test_single_cube_front_only_is_open_with_scout_behind():
    res, *_ = run([(3.0, 2.5)], [(1.6, 2.5, 0.0), (1.5, 2.5, 0.0), (1.7, 2.5, 0.0)])
    assert len(res.groups) == 1
    g = res.groups[0]
    assert g.closed is False
    assert len(res.faces) == 1 and res.faces[0].nx == -1
    assert g.scout_points, "뒷면을 못 봤으면 정찰 지점이 있어야 한다"
    sx, sy, dx, dy = g.scout_points[0]
    assert dx > 0.5 and sx > 3.0 - 0.1          # 가려진 쪽 = 동쪽(로봇 반대편)
    assert res.units[0].kind == "face"


def test_three_faces_seen_still_open_and_scout_points_to_missing_side():
    poses = [(3.0 + 1.0 * math.cos(a), 2.5 + 1.0 * math.sin(a), 0.0) for a in np.deg2rad([180, 135, 225, 90, 270, 120, 240])]
    res, *_ = run([(3.0, 2.5)], poses)
    g = res.groups[0]
    assert g.closed is False
    assert g.scout_points
    assert g.scout_points[0][2] > 0.5


def test_short_fragment_below_40cm_is_not_a_face():
    """25 cm 상자(UGV 크기)는 위치를 몰라도 면으로 기록되지 않는다."""
    res, *_ = run([cube(3.0, 2.5, 0.25)], around(3.0, 2.5, 0.9))
    assert res.faces == [] and res.units == []


def test_thin_wall_line_rejected_when_both_sides_free():
    thin = (2.0, 2.0, 2.04, 3.2)                  # 얇은 판(두 쪽 모두 광선이 지나간다)
    res, *_ = run([thin], [(1.2, 2.6, 0.0), (2.8, 2.6, 0.0), (1.4, 2.0, 0.0), (2.7, 3.0, 0.0)], scans=4)
    assert res.faces == []
    assert res.stats["thin_or_unknown_outside"] >= 1


# ---------------------------------------------------------------------------------------------- 이어 붙은 모양
def test_pair_2x1_makes_two_units_with_internal_faces():
    res, *_ = run([(3.0, 2.5), (3.4, 2.5)], around(3.2, 2.5, 1.2, 12))
    g = res.groups[0]
    assert g.closed is True and len(res.units) == 2
    left, right = sorted(res.units, key=lambda u: u.cx)
    assert abs(left.cx - 3.0) < 0.08 and abs(right.cx - 3.4) < 0.08
    assert 0 in left.internal and 2 in right.internal      # 맞닿은 면
    assert 0 not in left.faces and 2 not in right.faces    # 가려진 면은 식별 대상 아님
    assert {1, 2, 3} <= set(left.faces) and {0, 1, 3} <= set(right.faces)


def test_l_shape_three_cubes_closed_and_not_hexagon_required():
    cubes = [(3.0, 2.5), (3.4, 2.5), (3.4, 2.9)]
    res, *_ = run(cubes, around(3.3, 2.7, 1.3, 16))
    assert len(res.groups) == 1
    g = res.groups[0]
    assert g.closed is True
    assert len(res.units) == 3
    got = sorted((round(u.cx, 1), round(u.cy, 1)) for u in res.units)
    assert got == sorted((round(c[0], 1), round(c[1], 1)) for c in cubes)


def test_line_of_three_makes_three_units():
    cubes = [(2.6, 2.5), (3.0, 2.5), (3.4, 2.5)]
    res, *_ = run(cubes, around(3.0, 2.5, 1.3, 16))
    assert res.groups[0].closed is True
    assert len(res.units) == 3
    assert sorted(round(u.cx, 1) for u in res.units) == [2.6, 3.0, 3.4]


def test_each_cube_exterior_face_counted_once():
    """2×1 줄: 바깥 면 = 위 2 + 아래 2 + 양 끝 2 = 6 면(안쪽 맞닿은 면 2개는 제외)."""
    res, *_ = run([(3.0, 2.5), (3.4, 2.5)], around(3.2, 2.5, 1.2, 12))
    n = sum(len(u.faces) for u in res.units)
    assert n == 6


# ---------------------------------------------------------------------------------------------- 벽·UGV
def test_cube_against_wall_closes_with_wall_and_wall_face_internal():
    cfg = Config()
    W = cfg.arena.W
    cx = W - 0.2 - 0.02
    poses = [(cx - 1.0, 2.5, 0.0), (cx - 0.8, 2.0, 0.0), (cx - 0.8, 3.0, 0.0), (cx - 0.5, 1.9, 0.0), (cx - 0.5, 3.1, 0.0)]
    res, *_ = run([(cx, 2.5)], poses)
    assert len(res.units) == 1
    u = res.units[0]
    assert 0 in u.internal
    assert res.groups[0].closed is True and res.groups[0].touches_wall


def test_ugv_next_to_cube_is_excluded_from_faces_and_group_defers():
    ugv = [cube(3.45, 2.5, 0.25)]
    poses = around(3.0, 2.5, 1.1, 10)
    res_no, *_ = run([(3.0, 2.5)], poses, ugv=ugv)
    # UGV 위치를 알려 주면: UGV 셀은 제외되고, 가려진 쪽을 '열림'으로 오인하지 않도록 판정 보류
    res, *_ = run([(3.0, 2.5)], poses, ugv=ugv, ugv_points=[(3.45, 2.5, 0.30)])
    assert res.stats["ugv_cells_removed"] > 0
    assert all(math.hypot(u.cx - 3.45, u.cy - 2.5) > 0.3 for u in res.units)
    g = res.group(res.groups[0].gid)
    assert g.ugv_near is True and g.closed is None and g.scout_points == []
    assert res.stats["deferred"] == 1


def test_ugv_alone_is_never_a_face_even_if_position_unknown():
    res, *_ = run([], around(3.0, 2.5, 1.0), ugv=[cube(3.0, 2.5, 0.25)])
    assert res.faces == [] and res.units == []


def test_ugv_points_from_tracks_and_blobs():
    class T:
        def __init__(self, x, y, t):
            self.x, self.y, self.t = x, y, t

        def predicted(self, now):
            return (self.x + 0.1, self.y)

    class B:
        x, y = 1.0, 2.0

    pts = ugv_points_from([T(2.0, 2.0, 9.0), T(5.0, 5.0, 0.0)], [B()], 10.0, 0.3, 4.0, extra=[(0.5, 0.5)])
    assert (2.1, 2.0, 0.3) in [(round(x, 1), y, r) for x, y, r in pts]
    assert len(pts) == 3                       # 오래된 트랙(5,5)은 뺀다
    assert (1.0, 2.0, 0.3) in pts and (0.5, 0.5, 0.3) in pts


# ---------------------------------------------------------------------------------------------- 안정성
def test_group_ids_are_stable_across_updates_and_noise_tolerated():
    cfg, g = make_world([(3.0, 2.5), (3.4, 2.5)], around(3.2, 2.5, 1.2, 12), noise=0.01)
    m = LidarFaceMap(cfg)
    a = m.update(g, [], 0.0)
    b = m.update(g, [], 1.0)
    assert [x.gid for x in a.groups] == [x.gid for x in b.groups]
    assert a.groups[0].closed is True and len(a.units) == 2


def test_two_separate_cubes_are_two_groups():
    res, *_ = run([(2.0, 2.0), (4.0, 3.5)], around(2.0, 2.0, 1.0) + around(4.0, 3.5, 1.0))
    assert len(res.groups) == 2
    assert all(g.closed for g in res.groups)
    assert len(res.units) == 2


def test_empty_grid_returns_empty_result():
    cfg = Config()
    cfg.explore.lidar_faces = True
    g = OccupancyGrid(cfg.arena, res=0.05)
    res = LidarFaceMap(cfg).update(g, [], 0.0)
    assert res.groups == [] and res.faces == [] and res.units == []


def test_single_hit_cells_are_not_trusted():
    """한 번만 맞은 셀(L≈0.95 < lidar_face_min_L)은 점유로 안 센다."""
    cfg, g = make_world([(3.0, 2.5)], [(1.6, 2.5, 0.0)], scans=1)
    res = LidarFaceMap(cfg).update(g, [], 0.0)
    assert res.faces == []
