"""LiDAR 가 못 본 면을 OAK 로 식별하는 단계의 매니저 쪽 동작: on_demand 게이트, 식별 단계 판정, 정렬·머묾·시간 초과, 식별 가중치와 제출 조건,
UGV 위치 전달, OAK 생존 판정. 시뮬을 쓰지 않는다(가짜 로봇·가짜 검출기)."""
import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from hw_fakes import make_robot
from maicon26.config import Config
from maicon26.link.base import ServerLink
from maicon26.mission.explore_online import Cand
from maicon26.mission.manager import Agent, Mode
from maicon26.perception.detector import Detector
from maicon26.perception.object_map import ObjectMap
from maicon26.perception.tracks import UgvTracker
from maicon26.types import Detection, Frame, Pose2D

K = (800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0)
W, H = 1280, 720


class NullLink(ServerLink):
    def poll(self):
        return []

    def submit(self, kind, payload):
        return True


class OakDet(Detector):
    def __init__(self):
        self.oak, self.latest, self.submitted = [], ([], -1.0), []

    def detect(self, frame):
        return []

    def submit_oak(self, frame):
        self.submitted.append(frame.stamp)
        self.latest = (list(self.oak), frame.stamp)

    def poll_oak(self):
        return self.latest

    def set_strike(self, on):
        pass


def det(cls="TANK", u=640.0, conf=0.9, h=200.0, w=160.0):
    return Detection(cls, conf, (u - w / 2, 360.0 - h / 2, u + w / 2, 360.0 + h / 2), 0.0, None, {})


def make_agent(on_demand=True, **oak):
    cfg = Config()
    cfg.nav.backend = "builtin"
    cfg.oak_det.enable, cfg.oak_det.save_max, cfg.oak_det.on_demand = True, 0, on_demand
    for k, v in oak.items():
        setattr(cfg.oak_det, k, v)
    robot, tr, clock = make_robot("practice", "idle", cfg=cfg)
    d = OakDet()
    agent = Agent(robot, NullLink(), cfg=cfg, detector=d)
    agent.tracker = UgvTracker(cfg, "BLUE")
    agent.team, agent.slot = "BLUE", 2
    agent.stack = MagicMock()
    agent.stack.pose = Pose2D(2.0, 1.0, 0.0)
    agent._hist.append((0.0, 2.0, 1.0, 0.0))
    agent._hist.append((9.0, 2.0, 1.0, 0.0))
    robot.on_oak_info(K, W, H)
    return agent, robot, clock, d


def push(robot, hdr=0.0):
    robot.on_oak_frame(np.zeros((H, W, 3), np.uint8), hdr)


def ident_cand(vp=(2.0, 1.0), heading=0.0):
    return Cand("identify", vp, heading, "look", faces=[(0, 0)], primary_face=(0, 0))


def go_explore(agent, cand, phase="travel"):
    agent.mode, agent._ex_cand, agent._ex_phase = Mode.EXPLORE, cand, phase


# ====================================================================== 식별 단계 판정
def test_identify_active_needs_explore_mode_and_an_identify_candidate():
    agent, *_ = make_agent()
    assert not agent._identify_active(1.0)                                    # 후보 없음
    go_explore(agent, Cand("face", (2.0, 1.0), 0.0, "look"), "observe")
    assert not agent._identify_active(1.0)                                    # 일반 면 후보는 OAK 식별 단계가 아니다
    go_explore(agent, ident_cand(), "observe")
    assert agent._identify_active(1.0)
    agent.mode = Mode.STAGE if hasattr(Mode, "STAGE") else Mode.IDLE
    assert not agent._identify_active(1.0)                                    # 탐색 모드가 아니면 아니다


def test_identify_active_while_travelling_only_within_one_meter_of_the_viewpoint():
    agent, *_ = make_agent()
    go_explore(agent, ident_cand(vp=(3.5, 1.0)), "travel")
    assert not agent._identify_active(1.0)                                    # 1.5 m 떨어져 있다
    agent.stack.pose = Pose2D(2.6, 1.0, 0.0)
    assert agent._identify_active(1.0)                                        # 0.9 m


# ====================================================================== on_demand 게이트
def test_on_demand_keeps_oak_detector_fed_only_during_the_identify_phase():
    agent, robot, clock, d = make_agent(on_demand=True)
    clock.t = 1.0
    push(robot)
    d.oak = [det()]
    agent._ingest_oak(1.0)                                                    # 평소: 프레임을 검출기에 넣지 않는다
    assert d.submitted == [] and agent.stats["oak_frames_seen"] == 0
    go_explore(agent, ident_cand(), "observe")
    agent._ingest_oak(1.1)
    assert d.submitted and agent.stats["oak_det_consumed"] == 1               # 식별 단계: 처리한다


def test_without_on_demand_oak_is_ingested_all_the_time():
    agent, robot, clock, d = make_agent(on_demand=False)
    clock.t = 1.0
    push(robot)
    d.oak = [det()]
    agent._ingest_oak(1.0)
    assert d.submitted and agent.stats["oak_det_consumed"] == 1


# ====================================================================== 식별 관측의 가중치
def test_identify_observation_uses_identify_weight_and_counts_as_oak_id():
    agent, robot, clock, d = make_agent(identify_weight=1.0, weight=0.5)
    clock.t = 1.0
    push(robot)
    d.oak = [det("TANK", conf=0.8)]
    go_explore(agent, ident_cand(), "observe")
    agent._ingest_oak(1.0)
    f = agent.omap.faces[0]
    assert f.votes["TANK"] == pytest.approx(0.8 * 1.0)                        # 보통 OAK 는 0.5 — 식별 단계는 identify_weight
    assert f.src_n.get("oak_id") == 1.0 and agent.stats["oak_identify_obs"] == 1
    assert agent.omap.identify_mode is False                                  # 관측이 끝나면 되돌려 둔다


def test_identify_observation_resets_the_flag_even_when_observe_raises():
    agent, robot, clock, d = make_agent()
    clock.t = 1.0
    push(robot)
    d.oak = [det()]
    go_explore(agent, ident_cand(), "observe")
    agent.omap.observe = MagicMock(side_effect=RuntimeError("boom"))
    agent._ingest_oak(1.0)                                                    # 예외는 _ingest_oak 가 삼킨다
    assert agent.omap.identify_mode is False and agent.stats["oak_errors"] == 1


def test_oak_id_votes_are_submittable_only_with_identify_submit():
    for allow in (False, True):
        cfg = Config()
        cfg.oak_det.enable = True
        cfg.oak_det.identify_submit = allow
        om = ObjectMap(cfg)
        om.identify_mode = True
        pose = Pose2D(0.0, 0.0, 0.0)
        from maicon26.perception.camera import oak_obs_camera
        cam = oak_obs_camera(cfg.oak, cfg.rig, K, W, H)
        for i in range(12):
            h = 800.0 * 0.40 / 1.0
            om.observe([det("TANK", u=640.0, h=h, w=0.8 * h)], pose, Frame(None, 1.0 + i, 0.0, 0.0, W, H, meta={"source": "oak"}),
                       None, cam=cam, weight=1.0, source="oak")
        assert om.faces and om.faces[0].src_n == {"oak": 12.0, "oak_id": 12.0}
        assert bool(om.confirmed_faces()) is allow


# ====================================================================== 정렬·머묾·시간 초과
def test_identify_step_waits_for_alignment_then_dwell_and_frames():
    agent, robot, clock, d = make_agent(identify_dwell_s=2.5, identify_min_frames=4)
    c = ident_cand(heading=math.pi / 2)                                       # 몸이 0 rad — 90° 어긋남
    go_explore(agent, c, "observe")
    assert agent._identify_step(c, 10.0, 0.1, 0.5) is False
    assert agent._ex_aligned_since is None                                    # 아직 정렬 전
    agent.stack.pose = Pose2D(2.0, 1.0, math.pi / 2)
    assert agent._identify_step(c, 11.0, 0.1, 1.0) is False
    assert agent._ex_aligned_since == 11.0
    agent.stats["oak_det_consumed"] += 4
    assert agent._identify_step(c, 12.0, 0.1, 2.0) is False                   # 머문 시간 1.0 < 2.5
    assert agent._identify_step(c, 13.6, 0.1, 3.6) is True                    # 2.6 s, 프레임 4장


def test_identify_step_does_not_finish_without_enough_oak_frames_until_timeout():
    agent, robot, clock, d = make_agent(identify_dwell_s=1.0, identify_min_frames=4, identify_timeout_s=7.0)
    c = ident_cand()
    go_explore(agent, c, "observe")
    assert agent._identify_step(c, 10.0, 0.1, 0.5) is False
    assert agent._identify_step(c, 12.0, 0.1, 2.5) is False                   # 시간은 찼지만 프레임 0장
    assert agent._identify_step(c, 17.6, 0.1, 7.1) is True                    # 시간 초과로 끝낸다
    assert agent.stats["oak_identify_timeouts"] == 1


def test_identify_step_times_out_when_never_aligned():
    agent, robot, clock, d = make_agent(identify_timeout_s=7.0)
    c = ident_cand(heading=math.pi)
    go_explore(agent, c, "observe")
    assert agent._identify_step(c, 10.0, 0.1, 3.0) is False
    assert agent._identify_step(c, 15.0, 0.1, 7.5) is True                    # 정렬이 안 돼도 끝없이 돌지 않는다


def test_identify_step_turns_the_body_toward_the_face_and_commands_no_translation():
    agent, robot, clock, d = make_agent()
    c = ident_cand(heading=0.0)
    go_explore(agent, c, "observe")
    f = MagicMock(cx=2.0, cy=3.0)                                             # 면은 +y 방향
    agent.coverage.get_face = MagicMock(return_value=f)
    c.primary_face = (1, 1)
    agent._apply = MagicMock()
    agent._identify_step(c, 10.0, 0.1, 0.1)
    vx, wz = agent._apply.call_args[0][0], agent._apply.call_args[0][1]
    assert vx == 0.0 and wz > 0.0                                             # 제자리 회전(왼쪽)
    assert c.heading == pytest.approx(math.pi / 2)


# ====================================================================== UGV 위치 전달
def test_feed_ugv_points_is_a_noop_when_lidar_faces_is_off():
    agent, *_ = make_agent()
    agent.coverage.set_ugv_points = MagicMock()
    agent._feed_ugv_points(1.0)
    agent.coverage.set_ugv_points.assert_not_called()


def test_feed_ugv_points_passes_tracker_positions_and_never_raises():
    agent, *_ = make_agent()
    agent.cfg.explore.lidar_faces = True
    agent.coverage.set_ugv_points = MagicMock()
    agent._feed_ugv_points(1.0)
    agent.coverage.set_ugv_points.assert_called_once()
    agent.coverage.set_ugv_points = MagicMock(side_effect=RuntimeError("x"))
    agent._feed_ugv_points(2.0)                                               # 보조 기능의 예외는 제어 루프를 죽이지 않는다
    assert agent.stats["lidar_ugv_feed_errors"] == 1


# ====================================================================== OAK 생존 판정
def test_oak_alive_follows_fresh_frames_within_three_seconds():
    agent, robot, clock, d = make_agent()
    assert agent._oak_alive(10.0) is False                                    # 프레임이 없다
    clock.t = 11.0
    push(robot, 11.0)
    assert agent._oak_alive(12.0) is True
    assert agent._oak_alive(12.5) is True                                     # 1 Hz 캐시
    assert agent._oak_alive(15.5) is False                                    # 같은 프레임이 3 s 넘게 — 죽었다


def test_oak_alive_is_false_for_a_robot_without_oak_frames():
    agent, robot, clock, d = make_agent()
    robot.get_oak_frame = MagicMock(side_effect=RuntimeError("usb"))
    assert agent._oak_alive(10.0) is False


def test_frames_consumed_before_alignment_do_not_count_toward_the_dwell():
    agent, robot, clock, d = make_agent(identify_dwell_s=1.0, identify_min_frames=4, identify_timeout_s=30.0)
    c = ident_cand(heading=math.pi / 2)
    go_explore(agent, c, "observe")
    agent.stats["oak_det_consumed"] = 10                                      # 정렬 전에 이미 10장이 처리됐다(다른 방향을 보던 프레임)
    agent._identify_step(c, 10.0, 0.1, 0.5)
    agent.stack.pose = Pose2D(2.0, 1.0, math.pi / 2)
    agent._identify_step(c, 11.0, 0.1, 1.0)                                   # 여기서 정렬 시작 — 기준선은 10
    assert agent._identify_step(c, 13.0, 0.1, 3.0) is False                   # 2 s 지났지만 정렬 뒤 프레임 0장
    agent.stats["oak_det_consumed"] = 14
    assert agent._identify_step(c, 13.1, 0.1, 3.1) is True


def test_observe_phase_is_identify_active_even_if_the_robot_stopped_far_from_the_viewpoint():
    agent, *_ = make_agent()
    go_explore(agent, ident_cand(vp=(5.0, 4.0)), "observe")                   # 도착 허용 오차로 먼 곳에서 관측을 시작할 수 있다
    assert agent._identify_active(1.0)
    agent._ex_phase = "travel"
    assert not agent._identify_active(1.0)
