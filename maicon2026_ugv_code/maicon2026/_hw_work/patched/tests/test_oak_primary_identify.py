"""OAK(Depth 카메라) 우선 면 식별(oak_det.identify_primary): 코드는 들어 있지만 기본 OFF — 실기 설정에서도 꺼 두고 팬틸트가 식별한다.
여기서는 (1) 꺼져 있으면 기존 동작이 그대로인지, (2) 켰을 때 OAK 가 맡고 못 보는 면은 팬틸트로 넘기는지, (3) 전원 제어·화각·검증이 맞는지를 본다.
시뮬을 쓰지 않는다(가짜 로봇·가짜 검출기·합성 LiDAR 장면)."""
import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from hw_fakes import make_robot
from lidar_scene import around, make_world
from maicon26.config import Config
from maicon26.link.base import ServerLink
from maicon26.mission.explore_online import Cand, OnlineExplorer
from maicon26.mission.manager import Agent, Mode
from maicon26.perception.coverage import (BoxModel, CONFIRMED, CoverageMap, GIVEUP, UNSEEN, VIEWED_NO_DET, oak_fit_min_dist)
from maicon26.perception.detector import Detector
from maicon26.perception.object_map import Face, ObjectMap
from maicon26.perception.tracks import UgvTracker
from maicon26.types import Detection, Frame, Pose2D, Scan
from maicon26.validate import sanitize_config

K = (800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0)
W, H = 1280, 720


def primary(cfg):
    cfg.oak_det.enable = True
    cfg.oak_det.identify_primary = True


# ====================================================================== 장부(CoverageMap)
def ledger(mut=None, live=True):
    cfg = Config()
    if mut:
        mut(cfg)
    cov = CoverageMap(cfg)
    cov.oak_live = live
    box = BoxModel(7, 3.05, 2.525, "full", [], 1.0, 1.0)
    cov._make_faces(box)
    cov.boxes[box.bid] = box
    return cfg, cov, box


def test_defaults_keep_oak_off_and_pantilt_in_charge():
    cfg = Config()
    assert cfg.oak_det.enable is False and cfg.oak_det.identify_primary is False
    assert cfg.oak_det.identify_fallback is False and cfg.oak.power_gating is False
    cov = CoverageMap(cfg)
    cov.oak_live = True                                                          # 매니저가 True 로 알려도 설정이 꺼져 있으면 팬틸트가 맡는다
    assert cov.primary_oak() is False
    box = BoxModel(1, 3.0, 2.5, "full", [], 1.0, 1.0)
    cov._make_faces(box)
    cov.boxes[1] = box
    assert all(cov.pt_allowed(f) for f in box.faces) and cov.pt_gate(None) and cov.pt_gate((1, 0))
    assert len(cov.pending_faces()) == 4 and cov.identify_faces() == []         # 팬틸트 대상은 4면, OAK 식별 대상은 없다


def test_oak_fit_distance_follows_camera_height_and_fov():
    low = oak_fit_min_dist(0.10, 69.0, 42.0, 0.92)
    assert low == pytest.approx(0.80, abs=0.03)                                 # 낮은 카메라(0.10 m) — 세로가 먼저 걸려 0.8 m 쯤 떨어져야 한다
    assert oak_fit_min_dist(0.20, 69.0, 42.0, 0.92) < low                       # 그림 중심 높이에 가까울수록 가까이서도 온전히 보인다
    assert oak_fit_min_dist(0.10, 69.0, 54.0, 0.92) < low                       # 4:3 스트림(세로 화각이 큼)이면 더 가까이서도 된다
    assert oak_fit_min_dist(0.20, 69.0, 54.0, 0.92) >= 0.18 / math.tan(math.radians(34.5 * 0.92)) - 1e-9   # 가로 한계가 하한이다


def test_primary_mode_hands_unseen_faces_to_oak_not_pantilt():
    _, cov, box = ledger(primary)
    assert cov.primary_oak() is True
    assert cov.pending_faces() == []                                            # 팬틸트가 먼저 가 볼 면이 없다
    assert {f.key for f in cov.identify_faces()} == {f.key for f in box.faces}  # 팬틸트 시도 횟수와 무관하게 OAK 식별 대상
    assert all(not cov.pt_allowed(f) for f in box.faces)


def test_primary_mode_with_oak_not_live_looks_like_pantilt_only():
    _, cov, box = ledger(primary, live=False)
    assert cov.primary_oak() is False                                           # OAK 를 못 쓰면 모든 면을 팬틸트가 본다
    assert len(cov.pending_faces()) == 4 and all(cov.pt_allowed(f) for f in box.faces) and cov.pt_gate(None)


def test_a_face_handed_to_pantilt_returns_to_the_pantilt_queue_and_leaves_the_oak_queue():
    _, cov, box = ledger(primary)
    f = box.faces[2]
    cov.pt_fallback_face(f, "cropped", now=12.0)
    assert f.pt_fallback and f.oak_fail == "cropped"
    assert [p.key for p in cov.pending_faces()] == [f.key]
    assert f.key not in {i.key for i in cov.identify_faces()} and len(cov.identify_faces()) == 3
    cov.pt_fallback_face(f, "no_frames", now=13.0)                              # 두 번 넘겨도 처음 이유·횟수 그대로
    assert f.oak_fail == "cropped" and cov.fallback_stats["total"] == 1
    assert cov.fallback_stats["by_reason"] == {"cropped": 1}
    assert cov.pt_gate(f.key) is True and cov.pt_gate(box.faces[0].key) is False


def test_pt_gate_without_anchor_follows_the_pantilt_window():
    _, cov, _ = ledger(primary)
    assert cov.pt_gate(None) is False                                           # 위치 닻이 없는 팬틸트 득표는 평소엔 버린다
    cov.pt_window = True
    assert cov.pt_gate(None) is True                                            # 폴백 면을 보는 중에는 받는다


def test_pantilt_views_are_not_counted_for_faces_oak_owns():
    _, cov, box = ledger(primary)
    cfg = cov.cfg
    g = MagicMock()
    cov.visibility_occupancy = MagicMock(return_value=np.zeros((4, 4), bool))
    cov.face_visible = MagicMock(return_value=(True, "ok"))
    cov._independent = MagicMock(return_value=True)
    n = cov.record_view(Pose2D(2.0, 2.5, 0.0), 0.0, g, 20.0)
    assert n == 0 and all(f.views == 0 for f in box.faces)                      # OAK 가 맡은 면 — '봤는데 없었다'가 쌓이지 않는다
    cov.pt_fallback_face(box.faces[2], "cropped", now=20.0)
    n = cov.record_view(Pose2D(2.0, 2.5, 0.0), 0.0, g, 21.0)
    assert n == 1 and box.faces[2].views == 1 and sum(f.views for f in box.faces) == 1


@pytest.mark.parametrize("outcome", ["cropped", "no_frames"])
def test_cropped_or_missing_frames_send_the_face_to_pantilt_at_once(outcome):
    _, cov, box = ledger(primary)
    f = box.faces[2]
    cov.mark_identify([f.key], outcome=outcome, now=10.0)
    assert f.id_tries == 1 and f.pt_fallback and f.oak_fail == outcome and f.last_id_t == 10.0
    assert f.views == 0                                                         # 못 본 것은 독립 관측으로 세지 않는다


def test_seen_counts_one_view_and_none_hands_over_after_the_max_attempts():
    cfg, cov, box = ledger(primary)
    f, g = box.faces[2], box.faces[0]
    pose = Pose2D(2.0, 2.5, 0.0)
    cov.mark_identify([f.key], outcome="seen", pose=pose, now=10.0)
    assert f.views == 1 and f.view_poses == [(2.0, 2.5, 0.0)] and not f.pt_fallback and f.oak_none == 0
    need = cfg.oak_det.identify_max_attempts
    for i in range(need):
        assert not g.pt_fallback
        cov.mark_identify([g.key], outcome="none", pose=pose, now=10.0 + i)      # 제대로 보고도 검출이 없었다
    assert g.views == need and g.oak_none == need
    assert g.pt_fallback and g.oak_fail == "no_detection"


def test_unreached_only_counts_until_the_max_attempts_then_goes_to_pantilt():
    cfg, cov, box = ledger(primary)
    f = box.faces[1]
    need = cfg.oak_det.identify_max_attempts
    for i in range(need - 1):
        cov.mark_identify([f.key], outcome="unreached", now=10.0 + i)
    assert not f.pt_fallback and f.views == 0
    cov.mark_identify([f.key], outcome="unreached", now=20.0)
    assert f.pt_fallback and f.oak_fail == "unreached"


def test_outcome_none_only_raises_the_try_count():
    _, cov, box = ledger(primary)
    f = box.faces[3]
    cov.mark_identify([f.key], outcome=None, now=10.0)
    assert f.id_tries == 1 and not f.pt_fallback and f.views == 0


def test_without_primary_mark_identify_only_counts_tries():
    _, cov, box = ledger(lambda c: (setattr(c.oak_det, "enable", True), setattr(c.oak_det, "identify_fallback", True)))
    f = box.faces[2]
    cov.mark_identify([f.key], outcome="cropped", pose=Pose2D(1, 1, 0), now=10.0)
    assert f.id_tries == 1 and not f.pt_fallback and f.views == 0 and f.oak_none == 0   # 기존 동작 그대로


def test_unresolved_after_all_attempts_goes_to_pantilt_but_confirmed_faces_do_not():
    cfg, cov, box = ledger(primary)
    a, b = box.faces[2], box.faces[0]
    for f in (a, b):
        f.id_tries, f.last_id_t = cfg.oak_det.identify_max_attempts, 10.0
    b.det_n, b.det_votes, b.det_kind = 6, {"cls_a": 6.0}, "object"
    assert cov.face_state(b) == CONFIRMED
    cov.review_fallbacks(10.5)                                                  # 마지막 식별의 득표가 장부에 반영될 시간(1.5 s)을 둔다
    assert not a.pt_fallback
    cov.review_fallbacks(12.0)
    assert a.pt_fallback and a.oak_fail == "unresolved" and not b.pt_fallback


def test_identify_gain_weight_prefers_unseen_and_drops_with_each_try():
    _, cov, box = ledger(primary)
    f = box.faces[2]
    assert cov.face_state(f) == UNSEEN and cov.identify_gain_weight(f) == pytest.approx(1.0)
    f.id_tries = 1
    assert cov.identify_gain_weight(f) == pytest.approx(0.7)
    f.views = 3
    assert cov.face_state(f) in (VIEWED_NO_DET, UNSEEN)
    assert cov.identify_gain_weight(f) <= 0.7


def test_summary_reports_oak_identify_stats():
    _, cov, box = ledger(primary)
    cov.pt_fallback_face(box.faces[0], "cropped", now=5.0)
    s = cov.summary()["oak_identify"]
    assert s["primary"] is True and s["oak_live"] is True and s["total"] == 1 and s["by_reason"] == {"cropped": 1}
    assert s["events"][-1]["reason"] == "cropped" and s["events"][-1]["face"] == list(box.faces[0].key)
    cov.oak_live = False
    assert cov.summary()["oak_identify"]["primary"] is False


# ====================================================================== 물체 지도(ObjectMap)
def observed_box(bid=1, cx=2.0, cy=2.0):
    from maicon26.perception.coverage import FaceRec
    box = BoxModel(bid, cx, cy, "full", [(10, 10)], 0.0, 0.0, n_seen=3, full_seen=3)
    box.faces = [FaceRec(bid, i, nx, ny, cx + 0.2 * nx, cy + 0.2 * ny) for i, (nx, ny) in enumerate(((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)))]
    return box


def box_scan(box, pose):
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    dx, dy = np.cos(angles + pose.th), np.sin(angles + pose.th)
    with np.errstate(divide="ignore", invalid="ignore"):
        tx0, tx1 = (box.cx - 0.2 - pose.x) / dx, (box.cx + 0.2 - pose.x) / dx
        ty0, ty1 = (box.cy - 0.2 - pose.y) / dy, (box.cy + 0.2 - pose.y) / dy
    enter = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
    leave = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
    ranges = np.where((leave >= enter) & (enter > 0.05), enter, np.nan)
    return Scan(-math.pi, 2 * math.pi / 720, ranges, 1.0, 8.0)


def pantilt_det(omap, face, pose, cls="DRUM"):
    psi = math.atan2(face.cy - pose.y, face.cx - pose.x)
    distance = math.hypot(face.cx - pose.x, face.cy - pose.y)
    u = omap.cam.cx - omap.cam.f * math.tan(psi - pose.th)
    h = omap.cam.f * 0.4 / (distance * math.cos(psi - pose.th))
    return Detection(cls, 0.95, (u - 25, 240 - h / 2, u + 25, 240 + h / 2), 1.0)


def test_pantilt_votes_for_a_gated_face_are_dropped_and_the_others_are_kept():
    box = observed_box()
    pose = Pose2D(0.8, 2.0, 0.0)
    for allow, expect in ((False, 0), (True, 5), (None, 5)):                    # None = 게이트를 안 넘김(기존 동작)
        omap = ObjectMap(Config())
        det = pantilt_det(omap, box.faces[2], pose)
        gate = None if allow is None else (lambda key, a=allow: a)
        for _ in range(5):
            omap.observe([det], pose, Frame(None, 1.0), box_scan(box, pose), boxes=[box], pt_gate=gate)
        assert sum(f.n for f in omap.faces) == expect
        assert omap.association_stats["pt_gated"] == (5 if allow is False else 0)


def test_gate_is_not_applied_to_oak_votes():
    box = observed_box()
    pose = Pose2D(0.8, 2.0, 0.0)
    omap = ObjectMap(Config())
    det = pantilt_det(omap, box.faces[2], pose)
    for _ in range(5):
        omap.observe([det], pose, Frame(None, 1.0), box_scan(box, pose), boxes=[box], source="oak", pt_gate=lambda key: False)
    assert sum(f.n for f in omap.faces) == 5 and omap.association_stats["pt_gated"] == 0


def test_oak_id_votes_are_submittable_with_identify_primary_alone():
    cfg = Config()
    cfg.oak_det.enable = True
    om = ObjectMap(cfg)
    f = Face(2.0, 2.0, 1.8, 2.0, votes={"TANK": 5.0}, n=5)
    f.src_n = {"oak": 5.0, "oak_id": 5.0}
    assert om.submittable(f) is False                                           # 팬틸트 확인이 없고 identify_submit·primary 도 꺼져 있다
    cfg.oak_det.identify_primary = True
    assert om.submittable(f) is True
    f.src_n = {"oak": 5.0}                                                      # 식별 단계 밖의 OAK 득표는 여전히 안 된다
    assert om.submittable(f) is False


def test_is_object_cls_excludes_targets_and_base_marks():
    om = ObjectMap(Config())
    assert om.is_object_cls("TANK") is True
    assert om.is_object_cls("UGV") is False and om.is_object_cls("NOT_A_CLASS") is False


# ====================================================================== 탐색기(OnlineExplorer)
def explorer_for(cubes, poses, mut=None, live=True, **kw):
    cfg, g = make_world(cubes, poses, **kw)
    cfg.explore.lidar_faces = True
    if mut:
        mut(cfg)
    cov = CoverageMap(cfg)
    cov.oak_live = live
    cov.refresh(g, None, 10.0, force=True)
    return cfg, g, cov, OnlineExplorer(cfg, cov)


def ident_cands(cfg, g, cov, ex, pose=(2.0, 2.5, 0.0), now=11.0):
    blocked, _ = cov.planning_cost_maps(g)
    return [c for c in ex.build(Pose2D(*pose), g, blocked, now, team=None) if c.kind == "identify"]


def test_primary_offers_identify_stops_without_any_pantilt_attempt_at_the_fit_distance():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=primary)
    assert all(f.arrive_attempts == 0 for b in cov.boxes.values() for f in b.faces)
    d_stand = ex.identify_stand_off()
    assert d_stand > cfg.oak_det.identify_dist_m + 0.1                            # 면 그림이 잘리지 않는 거리 — 기본 0.8 m 보다 멀다
    cs = ident_cands(cfg, g, cov, ex)
    assert cs
    seen = set()
    for c in cs:
        f = cov.get_face(c.faces[0])
        d = math.hypot(c.vp[0] - f.cx, c.vp[1] - f.cy)
        along = (c.vp[0] - f.cx) * f.nx + (c.vp[1] - f.cy) * f.ny
        assert along == pytest.approx(d_stand, abs=1e-6) and d >= d_stand - 1e-6   # 옆으로 비켜도 면 법선 방향 거리는 줄지 않는다
        assert c.mode == "look" and c.primary_face == f.key and c.gain_f > 0
        seen.add(f.key)
    assert len(seen) >= 3                                                       # 열린 바깥 면이 여럿 후보가 된다


def test_default_config_offers_no_identify_stop_and_pantilt_pending_is_untouched():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5))
    assert not ident_cands(cfg, g, cov, ex)
    assert len(cov.pending_faces()) == 4 and ex.identify_stand_off() == cfg.oak_det.identify_dist_m


def test_identify_ready_false_silences_primary_candidates():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=primary)
    ex.identify_ready = False
    assert not ident_cands(cfg, g, cov, ex)


def test_wider_measured_fov_shortens_the_stand_off():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=primary)
    base = ex.identify_stand_off()
    ex.oak_fov = (69.0, 54.0)                                                   # 4:3 전체 스트림
    assert ex.identify_stand_off() < base
    ex.oak_fov = None
    assert ex.identify_stand_off() == pytest.approx(base)
    cfg.rig.oak_rgb_xyz = (0.30, 0.0, 0.10)                                     # 카메라가 몸 기준점보다 앞에 있는 만큼 몸은 더 떨어져 선다
    assert ex.identify_stand_off() > base


def test_cropped_stop_hands_the_face_to_pantilt_and_the_candidate_is_dropped():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=primary)
    c = ident_cands(cfg, g, cov, ex)[0]
    f = cov.get_face(c.faces[0])
    ex.note_observation(c, 12.0, outcome="cropped", pose=Pose2D(*c.vp, 0.0))
    assert f.pt_fallback and f.oak_fail == "cropped"
    assert f.key not in {cov.get_face(x.faces[0]).key for x in ident_cands(cfg, g, cov, ex, now=13.0)}
    assert f.key in {p.key for p in cov.pending_faces()}                         # 이제 팬틸트 방문 후보


def test_unreached_identify_stops_hand_over_after_the_max_attempts():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=primary)
    c = ident_cands(cfg, g, cov, ex)[0]
    key = c.faces[0]
    f = cov.get_face(key)
    ex.mark_bad(c.vp, 12.0)
    assert f.id_tries == 1 and not f.pt_fallback
    again = [x for x in ident_cands(cfg, g, cov, ex, now=13.0) if x.faces[0] == key]
    assert again
    ex.mark_bad(again[0].vp, 13.0)
    assert f.id_tries == 2 and f.pt_fallback and f.oak_fail == "unreached"


def test_face_with_no_reachable_oak_spot_goes_to_pantilt_after_the_noview_time():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=primary)
    ex._free = lambda grid, blocked, vp: False                                   # 어느 자리에도 설 수 없다(벽에 붙은 면 등)
    assert not ident_cands(cfg, g, cov, ex, now=20.0)
    assert not any(f.pt_fallback for b in cov.boxes.values() for f in b.faces)    # 아직 기다린다
    assert not ident_cands(cfg, g, cov, ex, now=20.0 + cfg.oak_det.identify_noview_s + 0.1)
    assert all(f.pt_fallback and f.oak_fail == "no_oak_view" for b in cov.boxes.values() for f in b.faces if f.exterior_open)
    assert len(cov.pending_faces()) == 4                                         # 모두 팬틸트가 맡는다


def test_gain_of_identify_stop_shrinks_for_a_face_already_tried():
    cfg, g, cov, ex = explorer_for([(3.0, 2.5)], around(3.0, 2.5), mut=primary)
    fresh = {c.faces[0]: c.gain_f for c in ident_cands(cfg, g, cov, ex)}
    key = next(iter(fresh))
    cov.get_face(key).id_tries = 1
    tried = {c.faces[0]: c.gain_f for c in ident_cands(cfg, g, cov, ex, now=12.0)}
    assert tried[key] < fresh[key] and tried[key] == pytest.approx(fresh[key] * 0.7)


# ====================================================================== 매니저(Agent)
class NullLink(ServerLink):
    def poll(self):
        return []

    def submit(self, kind, payload):
        return True


class OakDet(Detector):
    def __init__(self):
        self.oak, self.latest = [], ([], -1.0)

    def detect(self, frame):
        return []

    def submit_oak(self, frame):
        self.latest = (list(self.oak), frame.stamp)

    def poll_oak(self):
        return self.latest

    def set_strike(self, on):
        pass


def det(cls="TANK", u=640.0, conf=0.9, h=200.0, w=160.0):
    return Detection(cls, conf, (u - w / 2, 360.0 - h / 2, u + w / 2, 360.0 + h / 2), 0.0, None, {})


def make_agent(prim=True, **oak):
    cfg = Config()
    cfg.nav.backend = "builtin"
    cfg.oak_det.enable, cfg.oak_det.save_max, cfg.oak_det.on_demand = True, 0, True
    cfg.oak_det.identify_primary = prim
    for k, v in oak.items():
        setattr(cfg.oak_det, k, v)
    robot, tr, clock = make_robot("practice", "idle", cfg=cfg)
    agent = Agent(robot, NullLink(), cfg=cfg, detector=OakDet())
    agent.tracker = UgvTracker(cfg, "BLUE")
    agent.team, agent.slot = "BLUE", 2
    agent.stack = MagicMock()
    agent.stack.pose = Pose2D(2.0, 1.0, 0.0)
    agent._hist.append((0.0, 2.0, 1.0, 0.0))
    agent._hist.append((9.0, 2.0, 1.0, 0.0))
    robot.on_oak_info(K, W, H)
    return agent, robot, clock


def push(robot, hdr=0.0):
    robot.on_oak_frame(np.zeros((H, W, 3), np.uint8), hdr)


def go_explore(agent, cand, phase="travel"):
    agent.mode, agent._ex_cand, agent._ex_phase = Mode.EXPLORE, cand, phase


def add_box(agent, bid=7, cx=3.0, cy=1.0):
    box = BoxModel(bid, cx, cy, "full", [], 1.0, 1.0)
    agent.coverage._make_faces(box)
    agent.coverage.boxes[bid] = box
    return box


def ident(face_key, vp=(2.0, 1.0), heading=0.0):
    return Cand("identify", vp, heading, "look", faces=[face_key], primary_face=face_key)


def test_agent_with_primary_off_never_touches_the_primary_state():
    agent, robot, clock = make_agent(prim=False)
    agent._oak_primary_tick(5.0)
    cov = agent.coverage
    assert cov.oak_live is False and cov.pt_window is False and cov.primary_oak() is False
    agent._oak_power_gating = MagicMock(side_effect=AssertionError("전원 제어를 부르면 안 된다"))
    agent._oak_primary_tick(6.0)
    robot.oak_power = MagicMock()
    agent._oak_primary_tick(7.0)
    robot.oak_power.assert_not_called()


def test_oak_usable_needs_live_frames_or_confirmed_power_state():
    agent, robot, clock = make_agent()
    assert agent._oak_usable(1.0) is False                                       # 프레임이 없다
    clock.t = 2.0
    push(robot, 2.0)
    assert agent._oak_usable(3.0) is True
    agent._oak_broken = True
    assert agent._oak_usable(3.0) is False                                       # 포기한 뒤에는 쓰지 않는다
    agent._oak_broken = False
    agent2, robot2, _ = make_agent(prim=True)
    agent2.cfg.oak.power_gating = True
    agent2._oak_pwr_state = "off"
    assert agent2._oak_usable(1.0) is True                                       # 꺼져 있어도 필요할 때 켠다 — 프레임이 없어도 쓸 수 있다


def test_primary_tick_tells_the_ledger_whether_oak_is_usable():
    agent, robot, clock = make_agent()
    agent._oak_primary_tick(1.0)
    assert agent.coverage.oak_live is False and agent.coverage.primary_oak() is False
    clock.t = 2.0
    push(robot, 2.0)
    agent._oak_primary_tick(3.0)
    assert agent.coverage.oak_live is True and agent.coverage.primary_oak() is True


def test_pantilt_vote_window_opens_only_while_watching_a_handed_over_face():
    agent, robot, clock = make_agent()
    box = add_box(agent)
    clock.t = 2.0
    push(robot, 2.0)
    face = box.faces[2]
    go_explore(agent, Cand("face", (2.0, 1.0), 0.0, "look", faces=[face.key], primary_face=face.key), "observe")
    agent._oak_primary_tick(3.0)
    assert agent.coverage.pt_window is False                                     # 아직 OAK 가 맡은 면 — 팬틸트 득표를 받지 않는다
    agent.coverage.pt_fallback_face(face, "cropped", now=3.0)
    agent._oak_primary_tick(4.0)
    assert agent.coverage.pt_window is True
    go_explore(agent, ident(face.key), "observe")                                # 식별 후보를 보는 중이면 팬틸트 창은 닫힌다
    agent._oak_primary_tick(5.0)
    assert agent.coverage.pt_window is False
    agent.mode = Mode.WAIT
    agent._oak_primary_tick(6.0)
    assert agent.coverage.pt_window is False


def test_identify_outcome_matrix():
    agent, robot, clock = make_agent(identify_crop_frames=2)
    assert agent._identify_outcome() is None                                     # 정렬 전 — 횟수만 센다
    agent._ex_aligned_since, agent._ex_oak0 = 5.0, 0
    agent.stats["oak_det_consumed"] = 0
    assert agent._identify_outcome() == "no_frames"
    agent.stats["oak_det_consumed"] = 4
    agent._id_full = agent._id_crop = 0
    assert agent._identify_outcome() == "none"                                   # 프레임은 있었는데 표지 물체 검출이 없다
    agent._id_crop = 1
    assert agent._identify_outcome() == "none"
    agent._id_crop = 2
    assert agent._identify_outcome() == "cropped"
    agent._id_full = 1
    assert agent._identify_outcome() == "seen"                                   # 온전한 검출이 하나라도 있으면 본 것


def test_note_identify_dets_separates_whole_from_cropped_detections():
    agent, robot, clock = make_agent(identify_edge_margin_px=8)
    src = Frame(None, 1.0, 0.0, 0.0, W, H)
    agent._ex_aligned_since = None
    agent._note_identify_dets([det()], src)
    assert agent._id_full == 0 and agent._id_crop == 0                           # 정렬 전 프레임은 세지 않는다
    agent._ex_aligned_since = 5.0
    agent._note_identify_dets([det()], src)
    assert (agent._id_full, agent._id_crop) == (1, 0)
    agent._note_identify_dets([det(h=800.0)], src)                               # 위·아래가 영상 밖으로 나간다 — 가까워서 잘린 그림
    assert (agent._id_full, agent._id_crop) == (1, 1)
    agent._note_identify_dets([det(h=800.0), det()], src)                        # 온전한 검출이 하나라도 있으면 그 프레임은 '본 것'
    assert (agent._id_full, agent._id_crop) == (2, 1)
    agent._note_identify_dets([det("UGV")], src)                                 # 표지 물체가 아니면 세지 않는다
    assert (agent._id_full, agent._id_crop) == (2, 1)


def test_two_frames_without_oak_frames_give_up_oak_for_all_faces():
    agent, robot, clock = make_agent(identify_fail_limit=2)
    agent._oak_note_outcome(1.0, None)                                           # 정렬 전 끝남 — 세지 않는다
    agent._oak_note_outcome(2.0, "no_frames")
    assert agent._oak_broken is False and agent.stats["oak_identify_no_frames"] == 1
    agent._oak_note_outcome(3.0, "seen")                                         # 한 번 제대로 받으면 연속 횟수가 풀린다
    agent._oak_note_outcome(4.0, "no_frames")
    assert agent._oak_broken is False
    agent._oak_note_outcome(5.0, "no_frames")
    assert agent._oak_broken is True
    assert agent._oak_usable(6.0) is False
    agent._oak_primary_tick(6.0)
    assert agent.coverage.oak_live is False and agent.coverage.primary_oak() is False   # 이제 모든 면을 팬틸트가 본다


def test_oak_fov_comes_from_camera_info():
    agent, robot, clock = make_agent()
    hfov, vfov = agent._oak_fov()
    assert hfov == pytest.approx(math.degrees(2 * math.atan(W / (2 * 800.0))))
    assert vfov == pytest.approx(math.degrees(2 * math.atan(H / (2 * 800.0))))
    robot.on_oak_info((0.0,) * 9, W, H)                                          # 쓸 수 없는 K 는 None
    assert agent._oak_fov() is None


def test_identify_limit_includes_the_power_up_time_only_when_gating():
    agent, robot, clock = make_agent()
    base = agent.cfg.oak_det.identify_timeout_s
    assert agent._identify_limit() == base
    agent.cfg.oak.power_gating = True
    assert agent._identify_limit() == base + agent.cfg.oak.start_timeout_s
    agent2, *_ = make_agent(prim=False)
    agent2.cfg.oak.power_gating = True
    assert agent2._identify_limit() == agent2.cfg.oak_det.identify_timeout_s


def test_cropped_frames_end_the_stop_early_only_in_primary_mode():
    for prim, finished, outcome in ((True, True, "cropped"), (False, False, None)):
        agent, robot, clock = make_agent(prim=prim, identify_crop_frames=2)
        c = ident((0, 0))
        go_explore(agent, c, "observe")
        assert agent._identify_step(c, 10.0, 0.1, 0.5) is False                   # 정렬이 시작된다
        agent.stats["oak_det_consumed"] += 2
        agent._id_full, agent._id_crop = 0, 2
        assert agent._identify_step(c, 10.6, 0.1, 1.1) is finished
        assert agent._id_outcome == outcome


# ------------------------------------------------------------------------------------------------ 전원 제어
def power_agent(res="ok"):
    agent, robot, clock = make_agent()
    agent.cfg.oak.power_gating = True
    box = add_box(agent)
    robot.oak_power = MagicMock(return_value=res)
    go_explore(agent, ident(box.faces[2].key, vp=(2.0, 1.0)), "travel")
    agent._test_clock = clock
    return agent, robot


def test_power_is_requested_on_near_the_identify_stop_and_off_after_the_delay():
    agent, robot = power_agent("ok")
    agent.stack.pose = Pose2D(-0.5, 1.0, 0.0)                                     # 2.5 m 밖 — 아직 안 켠다
    agent._oak_want_t = -1e9
    agent._oak_primary_tick(10.0)
    robot.oak_power.assert_called_with(False)                                     # 기본 상태는 OFF — 시작하면 먼저 끈다
    assert agent._oak_pwr_state == "off" and agent.stats["oak_power_off"] == 1
    agent.stack.pose = Pose2D(1.0, 1.0, 0.0)                                      # wake_dist_m(2.0) 안 — 직전 명령이 성공했으므로 대기 없이 켠다
    agent._oak_primary_tick(11.0)
    robot.oak_power.assert_called_with(True)
    assert agent._oak_pwr_state == "on" and agent.stats["oak_power_on"] == 1
    agent.mode = Mode.WAIT                                                        # 식별이 끝났다
    agent._oak_primary_tick(11.0 + agent.cfg.oak.off_delay_s - 1.0)
    assert agent._oak_pwr_state == "on"                                           # 끄는 것은 off_delay_s 뒤
    agent._oak_primary_tick(11.0 + agent.cfg.oak.off_delay_s + 0.5)
    robot.oak_power.assert_called_with(False)
    assert agent._oak_pwr_state == "off" and agent.stats["oak_power_off"] == 2


def test_power_command_failure_is_retried_after_the_retry_interval():
    agent, robot = power_agent("fail")
    agent.stack.pose = Pose2D(1.0, 1.0, 0.0)
    agent._oak_primary_tick(10.0)
    agent._oak_primary_tick(10.5)                                                 # power_retry_s(2.0) 안 — 다시 보내지 않는다
    assert robot.oak_power.call_count == 1 and agent.stats["oak_power_fail"] == 1
    agent._oak_primary_tick(12.5)
    assert robot.oak_power.call_count == 2 and agent._oak_pwr_state != "on"


def test_pending_power_command_is_not_resent_every_tick():
    agent, robot = power_agent("pending")
    agent.stack.pose = Pose2D(1.0, 1.0, 0.0)
    for t in (10.0, 10.1, 10.2):
        agent._oak_primary_tick(t)
    assert robot.oak_power.call_count == 3                                        # 결과를 알려 줄 때까지 매 틱 확인한다(HAL 이 비동기로 처리)
    robot.oak_power.return_value = "ok"
    agent._oak_primary_tick(10.3)
    assert agent._oak_pwr_state == "on"


def test_missing_power_service_falls_back_to_keeping_oak_on_and_warns_once():
    agent, robot = power_agent("none")
    clock = agent._test_clock
    clock_logs = []
    agent._log = lambda now, msg: clock_logs.append(msg)
    agent.stack.pose = Pose2D(1.0, 1.0, 0.0)
    agent._oak_primary_tick(10.0)
    agent._oak_primary_tick(13.0)
    assert agent.stats["oak_power_unavailable"] >= 1 and agent._oak_pwr_state not in ("on", "off")
    assert sum("start/stop 서비스" in m for m in clock_logs) == 1                  # 경고는 한 번
    assert agent._oak_usable(14.0) is False                                       # 상태를 모르면 프레임이 실제로 들어올 때만 쓴다
    clock.t = 15.0
    push(robot, 15.0)
    assert agent._oak_usable(15.5) is True


def test_power_exception_does_not_kill_the_control_loop():
    agent, robot = power_agent("ok")
    robot.oak_power = MagicMock(side_effect=RuntimeError("usb"))
    agent.stack.pose = Pose2D(1.0, 1.0, 0.0)
    agent._oak_primary_tick(10.0)
    assert agent.stats["oak_power_fail"] == 1


# ------------------------------------------------------------------------------------------------ 관측 단계 배선
def drive_observe(agent, c, outcome_setup, now=20.0):
    go_explore(agent, c, "observe")
    agent._ex_obs_t0 = now - 0.2
    agent._mode_explore_online(now, 0.1, now)                                     # 정렬이 시작된다
    assert agent._ex_cand is c
    outcome_setup()
    agent._mode_explore_online(now + 0.5, 0.1, now + 0.5)


def test_cropped_observation_stop_marks_the_face_for_pantilt_through_the_explorer():
    agent, robot, clock = make_agent(identify_crop_frames=2)
    box = add_box(agent)
    face = box.faces[2]
    c = ident(face.key, vp=(2.0, 1.0))
    agent.coverage.oak_live = True

    def setup():
        agent.stats["oak_det_consumed"] += 3
        agent._id_full, agent._id_crop = 0, 2

    drive_observe(agent, c, setup)
    assert agent._ex_cand is None and agent._ex_phase == "travel"                 # 정차가 끝났다
    assert face.pt_fallback and face.oak_fail == "cropped" and face.id_tries == 1
    assert agent.stats["oak_identify_cropped"] >= 1


def test_seen_observation_stop_counts_a_view_and_keeps_the_face_with_oak():
    agent, robot, clock = make_agent(identify_dwell_s=0.3, identify_min_frames=2)
    box = add_box(agent)
    face = box.faces[2]
    c = ident(face.key, vp=(2.0, 1.0))
    agent.coverage.oak_live = True

    def setup():
        agent.stats["oak_det_consumed"] += 3
        agent._id_full, agent._id_crop = 2, 0

    drive_observe(agent, c, setup)
    assert agent._ex_cand is None
    assert face.views == 1 and face.id_tries == 1 and not face.pt_fallback and face.last_id_t > 0
    assert agent.stats["oak_identify_seen"] >= 1


def test_no_frames_at_the_stop_counts_toward_giving_up_oak():
    agent, robot, clock = make_agent(identify_fail_limit=1, identify_timeout_s=1.0)
    box = add_box(agent)
    face = box.faces[2]
    c = ident(face.key, vp=(2.0, 1.0))
    agent.coverage.oak_live = True
    go_explore(agent, c, "observe")
    agent._ex_obs_t0 = 20.0
    agent._mode_explore_online(20.1, 0.1, 20.1)                                    # 정렬 시작(프레임은 한 장도 안 들어온다)
    agent._mode_explore_online(22.5, 0.1, 22.5)                                    # identify_timeout_s 를 넘겼다
    assert face.pt_fallback and face.oak_fail == "no_frames"
    assert agent._oak_broken is True


def test_primary_off_observation_stop_keeps_the_old_marking():
    agent, robot, clock = make_agent(prim=False, identify_dwell_s=0.3, identify_min_frames=2)
    agent.cfg.oak_det.identify_fallback = True
    box = add_box(agent)
    face = box.faces[2]
    c = ident(face.key, vp=(2.0, 1.0))

    def setup():
        agent.stats["oak_det_consumed"] += 3

    drive_observe(agent, c, setup)
    assert face.id_tries == 1 and not face.pt_fallback and face.views == 0 and agent._oak_fail_n == 0


# ====================================================================== 설정 검증
def test_sanitize_turns_off_primary_and_power_gating_when_oak_det_is_disabled():
    cfg = Config()
    cfg.oak_det.enable = False
    cfg.oak_det.identify_primary = True
    cfg.oak.power_gating = True
    notes = sanitize_config(cfg)
    assert cfg.oak_det.identify_primary is False and cfg.oak.power_gating is False
    assert any("identify_primary" in n for n in notes) and any("power_gating" in n for n in notes)


def test_sanitize_keeps_a_valid_primary_setup():
    cfg = Config()
    primary(cfg)
    cfg.oak.power_gating = True
    sanitize_config(cfg)
    assert cfg.oak_det.identify_primary is True and cfg.oak.power_gating is True


@pytest.mark.parametrize("section,key,bad", [
    ("oak_det", "identify_crop_frames", 0),
    ("oak_det", "identify_fail_limit", 99),
    ("oak_det", "fit_hfov_deg", 5.0),
    ("oak_det", "fit_vfov_deg", 500.0),
    ("oak_det", "fit_usable", 0.1),
    ("oak_det", "identify_noview_s", -1.0),
    ("oak", "wake_dist_m", 0.0),
    ("oak", "off_delay_s", -3.0),
    ("oak", "start_timeout_s", 0.0),
    ("oak", "power_retry_s", float("nan")),
])
def test_out_of_range_values_fall_back_to_the_defaults(section, key, bad):
    cfg = Config()
    primary(cfg)
    default = getattr(getattr(Config(), section), key)
    setattr(getattr(cfg, section), key, bad)
    sanitize_config(cfg)
    assert getattr(getattr(cfg, section), key) == default
