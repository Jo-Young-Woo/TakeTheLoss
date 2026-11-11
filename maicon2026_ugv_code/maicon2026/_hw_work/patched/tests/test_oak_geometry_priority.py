"""6장: OAK 는 '위치 증거'만 보탠다 — 방위 기하, 가중치, M3 제출 조건, 엇갈림 처리, 그리고 OAK 검출로는 쏘지 않는다.
시뮬을 쓰지 않는다(가짜 로봇·가짜 검출기)."""
import inspect
import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from hw_fakes import make_robot
from maicon26.config import Config
from maicon26.link.base import ServerLink
from maicon26.mission import manager as manager_mod
from maicon26.mission.manager import Agent, Mode
from maicon26.perception.camera import oak_obs_camera
from maicon26.perception.coverage import CoverageMap
from maicon26.perception.detector import Detector
from maicon26.perception.object_map import ObjectMap
from maicon26.perception.tracks import UgvTracker
from maicon26.types import Detection, Frame, Pose2D

K = (800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0)          # 1280×720 가상 K (시험용 값)
W, H = 1280, 720


def oak_frame(stamp=1.0):
    return Frame(None, stamp, 0.0, 0.0, W, H, meta={"source": "oak"})


def pt_frame(stamp=1.0, pan=0.0):
    return Frame(None, stamp, pan, 0.0, 640, 480)


def det(cls="TANK", u=640.0, conf=0.9, h=200.0, w=160.0, team=None):
    cy = 360.0
    return Detection(cls, conf, (u - w / 2, cy - h / 2, u + w / 2, cy + h / 2), 0.0, team, {})


def cfg_oak(**over):
    cfg = Config()
    cfg.oak_det.enable = True
    cfg.oak_det.save_max = 0
    for k, v in over.items():
        setattr(cfg.oak_det, k, v)
    return cfg


# ====================================================================== 방위 기하
def test_world_bearing_is_heading_plus_mount_yaw_plus_pixel_bearing():
    cfg = cfg_oak()
    cfg.rig.oak_rgb_rpy = (0.0, 0.0, 0.3)
    cfg.rig.oak_rgb_xyz = (0.0, 0.0, 0.0)
    cam = oak_obs_camera(cfg.oak, cfg.rig, K, W, H)
    om = ObjectMap(cfg)
    pose = Pose2D(1.0, 1.0, 0.5)
    d = det(u=640.0 + 100.0)
    x, y, r, psi = om.locate(d, pose, oak_frame(), None, cam=cam)
    assert psi == pytest.approx(pose.th + 0.3 + math.atan2(640.0 - 740.0, 800.0))
    assert math.hypot(x - 1.0, y - 1.0) == pytest.approx(r, rel=0.05)        # 장착 위치 0 이면 차체 원점에서 쏜다


def test_oak_ignores_the_pan_of_the_frame_it_rides_on():
    """OAK 프레임의 pan 은 0 이다 — 방위에 팬틸트 값이 섞이면 안 된다."""
    cfg = cfg_oak()
    cam = oak_obs_camera(cfg.oak, cfg.rig, K, W, H)
    om = ObjectMap(cfg)
    pose = Pose2D(0.0, 0.0, 0.0)
    _, _, _, psi0 = om.locate(det(u=640.0), pose, oak_frame(), None, cam=cam)
    assert psi0 == pytest.approx(0.0)


def test_size_range_starts_at_the_camera_but_lidar_range_starts_at_the_body():
    cfg = cfg_oak()
    cfg.rig.oak_rgb_xyz = (0.10, 0.05, 0.2)
    cam = oak_obs_camera(cfg.oak, cfg.rig, K, W, H)
    pose = Pose2D(2.0, 3.0, math.pi / 2)
    ox, oy = cam.origin(pose, "size")
    assert (ox, oy) == pytest.approx((2.0 - 0.05, 3.0 + 0.10))               # 회전 후 장착 오프셋
    assert cam.origin(pose, "lidar") == (2.0, 3.0)


def test_no_camera_info_means_no_oak_camera_not_a_guessed_fov():
    cfg = cfg_oak()
    assert oak_obs_camera(cfg.oak, cfg.rig, None, W, H) is None
    assert oak_obs_camera(cfg.oak, cfg.rig, (0.0,) * 9, W, H) is None
    assert oak_obs_camera(cfg.oak, cfg.rig, K[:8], W, H) is None


# ====================================================================== 가중치·득표
def test_oak_vote_weight_defaults_to_half_and_pantilt_is_unchanged():
    cfg = cfg_oak()
    assert cfg.oak_det.weight == 0.5
    cam = oak_obs_camera(cfg.oak, cfg.rig, K, W, H)
    pose = Pose2D(0.0, 0.0, 0.0)
    om = ObjectMap(cfg)
    om.observe([det(conf=0.8)], pose, oak_frame(), None, cam=cam, weight=cfg.oak_det.weight, source="oak")
    f = om.faces[0]
    assert f.votes["TANK"] == pytest.approx(0.8 * 0.5)
    assert f.n == pytest.approx(0.5)
    assert f.src_n == {"oak": 1.0}
    om2 = ObjectMap(cfg)
    om2.observe([det(conf=0.8, u=320.0, h=150.0, w=100.0)], pose, pt_frame(), None)
    assert om2.faces[0].votes["TANK"] == pytest.approx(0.8) and om2.faces[0].n == 1


def test_pantilt_call_without_new_arguments_equals_explicit_defaults():
    cfg = cfg_oak()
    pose = Pose2D(0.0, 0.0, 0.0)
    a, b = ObjectMap(cfg), ObjectMap(cfg)
    for _ in range(5):
        a.observe([det(u=320.0, h=150.0, w=100.0)], pose, pt_frame(), None)
        b.observe([det(u=320.0, h=150.0, w=100.0)], pose, pt_frame(), None, cam=None, weight=1.0, source="pantilt")
    fa, fb = a.faces[0], b.faces[0]
    assert (fa.x, fa.y, fa.n, dict(fa.votes)) == (fb.x, fb.y, fb.n, dict(fb.votes))


def test_oak_never_counts_base_markers_or_led():
    cfg = cfg_oak()
    cam = oak_obs_camera(cfg.oak, cfg.rig, K, W, H)
    om = ObjectMap(cfg)
    om.observe([det("COMMAND_POST")], Pose2D(1.0, 1.0, 0.0), oak_frame(), None, cam=cam, weight=0.5, source="oak")
    assert not om.base_votes
    d = det("GUARD_POST")
    d.extra["led_on"] = True
    for _ in range(10):
        om.observe([d], Pose2D(1.0, 1.0, 0.0), oak_frame(), None, cam=cam, weight=0.5, source="oak")
    assert all(not f.led_on for f in om.faces)                               # OAK 영상으로 '타격 완료'를 만들지 않는다


# ====================================================================== M3 제출 조건
def _oak_only_map(cfg, n=12):
    cam = oak_obs_camera(cfg.oak, cfg.rig, K, W, H)
    om = ObjectMap(cfg)
    for i in range(n):
        om.observe([det("TANK")], Pose2D(0.0, 0.0, 0.0), oak_frame(1.0 + i), None, cam=cam, weight=0.5, source="oak")
    return om


def test_oak_only_face_is_not_submitted_by_default():
    cfg = cfg_oak()
    om = _oak_only_map(cfg)
    assert om.faces and om.faces[0].n >= cfg.perception.min_votes             # 득표는 충분하다
    assert om.confirmed_faces() == []                                         # 그래도 M3 로는 내지 않는다


def test_oak_only_face_is_submitted_when_submit_alone_is_on():
    cfg = cfg_oak(submit_alone=True)
    om = _oak_only_map(cfg)
    assert [f.src_n for f in om.confirmed_faces()] == [{"oak": 12.0}]


def test_one_pantilt_observation_lets_the_merged_face_be_submitted():
    cfg = cfg_oak()
    om = _face_with_votes(cfg, "TANK", "TANK", n_pt=1, n_oak=12)             # 팬틸트 1회 + OAK 12회가 같은 면에 합쳐진다
    assert len(om.faces) == 1 and om.faces[0].src_n == {"pantilt": 1.0, "oak": 12.0}
    assert len(om.confirmed_faces()) == 1


def test_coverage_ledger_skips_oak_only_faces():
    cfg = cfg_oak()
    cov = CoverageMap(cfg)
    om = _oak_only_map(cfg)
    cov._sync_detections(om)                                                  # 예외 없이, 장부에 아무것도 올리지 않는다
    assert all(f.det_n == 0 for b in cov.boxes.values() for f in b.faces)


# ====================================================================== 엇갈림
def _face_with_votes(cfg, pt_cls, oak_cls, n_pt=3, n_oak=3):
    cam = oak_obs_camera(cfg.oak, cfg.rig, K, W, H)
    om = ObjectMap(cfg)
    pose = Pose2D(0.0, 0.0, 0.0)
    for _ in range(n_pt):
        om.observe([det(pt_cls, u=320.0, h=150.0, w=100.0)], pose, pt_frame(), None)
    f0 = om.faces[0]
    for i in range(n_oak):
        # OAK 에서 팬틸트와 같은 월드 점을 겨냥하는 픽셀: 둘 다 정면(u=중심)이고 거리가 같도록 높이를 맞춘다
        r = f0.x
        h_oak = 800.0 * 0.40 / r
        om.observe([det(oak_cls, u=640.0, h=h_oak, w=0.8 * h_oak)], pose, oak_frame(10.0 + i), None, cam=cam, weight=0.5, source="oak")
    return om


def test_class_disagreement_is_recorded_once():
    cfg = cfg_oak()
    om = _face_with_votes(cfg, "TANK", "TRUCK")
    assert len(om.faces) == 1
    got = om.pop_disagreements()
    assert len(got) == 1 and got[0]["pantilt"] == "TANK" and got[0]["oak"] == "TRUCK"
    assert om.pop_disagreements() == []                                       # 한 번만
    assert om.association_stats["oak_pantilt_disagree"] == 1


def _strict_json(value):
    """최종 보고서 작성기(scripts/run_wingbot_autonomous_trial.json_value)와 같은 규칙: 데이터클래스·dict·list·tuple·원시형만 허용하고
    deque·set 같은 것은 거부한다. 그 스크립트를 가져오면 시뮬 모듈까지 딸려 오므로 같은 규칙을 여기에 둔다(실제 함수는 기존 시험
    test_wingbot_selfplay_coordinator 가 확인한다)."""
    import dataclasses
    if dataclasses.is_dataclass(value):
        return {f.name: _strict_json(getattr(value, f.name)) for f in dataclasses.fields(value)}
    if isinstance(value, dict):
        return {str(k): _strict_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_strict_json(v) for v in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    raise TypeError(f"JSON 으로 쓸 수 없는 타입: {type(value).__name__}")


def test_face_with_oak_votes_and_a_disagreement_can_be_written_to_the_final_report_and_restored():
    # 회귀: Face 에 deque/set 필드를 두면 최종 보고서 저장이 TypeError 로 죽는다. 저장 → 복원이 끝까지 되어야 한다.
    import json
    from maicon26.perception.object_map import Face
    cfg = cfg_oak(vote_window=2)                                              # 창이 있으면 oak_obs 가 채워진다
    om = _face_with_votes(cfg, "TANK", "TRUCK", n_pt=3, n_oak=4)
    face = om.faces[0]
    assert face.disagreed == ["TANK>TRUCK"] and len(face.oak_obs) == 2          # 엇갈림과 관측 창이 실제로 들어 있다
    saved = json.loads(json.dumps(_strict_json(face), ensure_ascii=False, allow_nan=False))   # 예외 없이 쓴다
    assert saved["src_n"]["oak"] >= 1.0 and saved["disagreed"] == ["TANK>TRUCK"] and len(saved["oak_obs"]) == 2
    back = Face(**saved)                                                      # scripts/audit_m3_anchor_filter.py 가 이렇게 되살린다
    assert back.disagreed == face.disagreed and back.src_n == face.src_n
    assert _strict_json(back)["disagreed"] == ["TANK>TRUCK"]
    old = {k: v for k, v in saved.items() if k not in ("src_n", "src_votes", "oak_obs", "disagreed")}
    assert Face(**old).disagreed == []                                        # 6장 이전에 저장된 보고서도 되살려진다


def test_strict_json_helper_rejects_the_types_that_broke_the_report():
    from collections import deque
    for bad in (deque([1]), {1, 2}):
        with pytest.raises(TypeError):
            _strict_json({"x": bad})


def test_agreeing_classes_are_not_flagged():
    cfg = cfg_oak()
    om = _face_with_votes(cfg, "TANK", "TANK")
    assert om.pop_disagreements() == []


def test_flag_recheck_makes_a_confirmed_face_worth_seeing_again():
    from maicon26.link.base import UavBox
    cfg = Config()
    cfg.explore.complete_m3_faces = True
    cov = CoverageMap(cfg)
    cov.ingest_uav_boxes([UavBox(2.0, 2.0, 0.4, 0.4)], 1.0)
    cov.ingest_uav_boxes([UavBox(2.0, 2.0, 0.4, 0.4)], 2.0)
    box = next(iter(cov.boxes.values()))
    for f in box.faces:
        f.det_kind, f.det_n, f.det_votes = "object", 5, {"DRUM": 4.5}
    f = box.faces[0]
    assert cov.face_weight(f) == 0.0
    key = cov.flag_recheck(f.cx, f.cy)
    assert key == f.key and cov.face_weight(f) == pytest.approx(0.4)
    assert cov.flag_recheck(9.0, 9.0) is None                                 # 가까운 면이 없으면 아무 일도 없다


# ====================================================================== Agent: 가짜 검출기·가짜 로봇
class NullLink(ServerLink):
    def poll(self):
        return []

    def submit(self, kind, payload):
        return True


class OakDet(Detector):
    """팬틸트 슬롯은 pt 를, OAK 슬롯은 oak 를 돌려준다. OAK 결과는 submit 한 프레임의 stamp 로 나온다."""

    def __init__(self):
        self.pt, self.oak = [], []
        self.latest = ([], -1.0)
        self.strike = None
        self.submitted = []

    def detect(self, frame):
        return list(self.pt)

    def submit_oak(self, frame):
        self.submitted.append(frame.stamp)
        self.latest = (list(self.oak), frame.stamp)

    def poll_oak(self):
        return self.latest

    def set_strike(self, on):
        self.strike = on


def make_agent(cfg=None, flags=()):
    cfg = cfg or cfg_oak()
    cfg.nav.backend = "builtin"
    robot, tr, clock = make_robot("practice", "idle", flags=flags, cfg=cfg)
    d = OakDet()
    agent = Agent(robot, NullLink(), cfg=cfg, detector=d)
    agent.tracker = UgvTracker(cfg, "BLUE")                                   # _sense 가 정체성을 정할 때 만드는 것
    agent.team, agent.slot = "BLUE", 2
    agent.stack = MagicMock()                                                 # 항법 스택은 이 시험의 관심사가 아니다
    agent.stack.pose = Pose2D(2.0, 1.0, 0.0)
    agent._hist.append((0.0, 2.0, 1.0, 0.0))
    agent._hist.append((9.0, 2.0, 1.0, 0.0))
    robot.on_oak_info(K, W, H)
    return agent, robot, tr, clock, d


def push_oak(robot, clock, hdr=0.0):
    robot.on_oak_frame(np.zeros((H, W, 3), np.uint8), hdr)


def test_default_config_leaves_oak_completely_off():
    cfg = Config()
    cfg.nav.backend = "builtin"
    robot, tr, clock = make_robot("practice", "idle", cfg=cfg)
    d = OakDet()
    agent = Agent(robot, NullLink(), cfg=cfg, detector=d)
    assert agent._oak_on is False and agent.ring is None
    push_oak(robot, clock)
    agent._ingest_oak(1.0)
    assert d.submitted == [] and agent.stats["oak_frames_seen"] == 0


def test_enabled_agent_submits_frame_and_adds_a_weighted_observation():
    agent, robot, tr, clock, d = make_agent()
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("TANK", conf=0.8, u=640.0, h=200.0, w=160.0)]
    agent._ingest_oak(1.0)
    assert d.submitted and agent.stats["oak_frames_seen"] == 1 and agent.stats["oak_det_consumed"] == 1
    assert agent.stats["oak_dets"] == 1
    f = agent.omap.faces[0]
    assert f.src_n == {"oak": 1.0} and f.votes["TANK"] == pytest.approx(0.8 * 0.5)
    agent._ingest_oak(1.1)                                                    # 같은 결과를 두 번 반영하지 않는다
    assert agent.stats["oak_det_consumed"] == 1 and agent.omap.faces[0].n == pytest.approx(0.5)


def test_detections_below_oak_conf_are_dropped():
    agent, robot, tr, clock, d = make_agent()
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("TANK", conf=0.30)]                                          # oak_det.conf 기본 0.35
    agent._ingest_oak(1.0)
    assert agent.stats["oak_dets"] == 0 and not agent.omap.faces


def test_oak_observation_needs_camera_info_k():
    agent, robot, tr, clock, d = make_agent()
    robot._oak_info = None
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("TANK")]
    agent._ingest_oak(1.0)
    assert agent.stats["oak_no_camera"] == 1 and not agent.omap.faces


def test_result_without_its_frame_is_dropped_not_mixed():
    agent, robot, tr, clock, d = make_agent()
    clock.t = 1.0
    d.latest = ([det("TANK")], 123.0)                                         # 우리가 낸 적 없는 프레임의 결과
    agent._ingest_oak(1.0)
    assert agent.stats["oak_det_dropped"] == 1 and not agent.omap.faces


def test_strike_and_hunt_tell_the_detector_to_pause_oak():
    agent, robot, tr, clock, d = make_agent()
    for mode, want in ((Mode.EXPLORE, False), (Mode.STRIKE, True), (Mode.HUNT, True), (Mode.EXPLORE, False)):
        agent.mode = mode
        agent._ingest_oak(1.0)
        assert d.strike is want


def test_no_vision_degraded_never_touches_the_oak_slot():
    agent, robot, tr, clock, d = make_agent(flags={"no_vision"})
    push_oak(robot, clock)
    agent._ingest_oak(1.0)
    assert d.submitted == []


def test_detector_without_oak_slot_warns_once_and_continues():
    agent, robot, tr, clock, d = make_agent()

    class Plain(Detector):
        def detect(self, frame):
            return []
    agent.detector = Plain()
    for t in (1.0, 2.0, 3.0):
        agent._ingest_oak(t)
    assert sum("OAK 슬롯" in m for _, m in agent.log) == 1


def test_an_oak_failure_never_escapes_the_control_loop():
    agent, robot, tr, clock, d = make_agent()

    def boom():
        raise RuntimeError("poll failed")
    d.poll_oak = boom
    agent._ingest_oak(1.0)                                                    # 예외 없음
    assert agent.stats["oak_errors"] == 1


def test_oak_disagreement_flags_the_face_for_a_pantilt_recheck():
    agent, robot, tr, clock, d = make_agent()
    calls = []
    agent.coverage.flag_recheck = lambda x, y, radius=0.45: calls.append((x, y))
    agent.omap.disagreements.append({"x": 1.0, "y": 2.0, "pantilt": "TANK", "oak": "TRUCK", "t": 0.0})
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("TANK")]
    agent._ingest_oak(1.0)
    assert calls == [(1.0, 2.0)] and agent.stats["oak_disagreements"] == 1


# ====================================================================== OAK 검출로는 쏘지 않는다 (6-4)
def test_oak_enemy_ugv_becomes_a_cue_but_never_a_fire_request():
    agent, robot, tr, clock, d = make_agent()
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("UGV", conf=0.9, u=640.0, h=220.0, w=160.0, team="RED")]      # 우리가 BLUE → RED 가 적
    agent._ingest_oak(1.0)
    assert agent._dets == []                                                  # 조준·사격이 보는 목록은 그대로 비어 있다
    cands = agent._hunt_candidates(1.1)
    assert len(cands) == 1                                                    # 기존 '관측 방위 후보'(tracker.enemies)에는 단서로 들어간다
    assert agent._best_det(lambda x: x.cls == "UGV") is None                  # 그러나 쏠 근거(팬틸트 검출)는 없다
    agent.mode = Mode.HUNT
    for i in range(60):                                                       # 적 쪽으로 접근·탐색만 한다
        clock.t += 0.05
        agent._mode_hunt(clock.t, 0.05, 1.0 + 0.05 * i)
        agent._ingest_oak(clock.t)
    assert tr.fire == [] and agent.laser.shots == 0
    assert agent.stats["hunt_shots"] == 0 and agent.stats["strike_shots"] == 0


def test_ingest_oak_source_never_touches_aim_laser_or_fire_paths():
    src = inspect.getsource(Agent._ingest_oak) + inspect.getsource(Agent._ingest_oak_inner)
    for forbidden in ("self._dets", "self.aim", "self.laser", "laser_fire", "robot.set_pan_tilt", "set_velocity", "_apply("):
        assert forbidden not in src, forbidden


def test_oak_module_state_is_not_exposed_to_the_pantilt_det_context():
    agent, robot, tr, clock, d = make_agent()
    before = (agent._det_ctx if hasattr(agent, "_det_ctx") else None, agent._det_consumed, agent._dets_t)
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("TANK")]
    agent._ingest_oak(1.0)
    assert (agent._det_ctx if hasattr(agent, "_det_ctx") else None, agent._det_consumed, agent._dets_t) == before
