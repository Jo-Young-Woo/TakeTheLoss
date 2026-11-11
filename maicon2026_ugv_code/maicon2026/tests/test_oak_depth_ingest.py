"""Agent._ingest_oak: OAK Depth 로 상대 UGV 거리·피아식별 연결(oak_det.depth_range / ugv_only). 가짜 로봇·가짜 검출기만 쓴다(시뮬·ROS 없음).
- 깊이 프레임이 있으면 UGV 트랙 거리는 깊이(how='depth'), 없으면 기존(크기) 경로 그대로.
- ugv_only 는 면(물체 지도) 득표를 내지 않는다. 어느 경우에도 조준·사격 목록은 비어 있다."""
import numpy as np
import pytest

from test_oak_geometry_priority import H, K, W, cfg_oak, det, make_agent, push_oak
from test_oak_ugv_range import FX, scene
from maicon26.types import DepthFrame

DEPTH_K = (FX, 0.0, W / 2, 0.0, FX, H / 2, 0.0, 0.0, 1.0)


def _ugv(z=1.2, team="RED"):
    d, box = scene(z)
    return d, box, _det_from_box(box, team)


def _det_from_box(box, team):
    from maicon26.types import Detection
    return Detection("UGV", 0.9, tuple(box), 0.0, team, {})


def _push_depth(robot, depth, stamp):
    robot._depth = DepthFrame(depth.astype(np.float32), stamp, K=DEPTH_K)


def _agent(**over):
    cfg = cfg_oak(**over)
    return make_agent(cfg)


def test_default_has_no_depth_ranger_and_no_depth_stats():
    agent, robot, tr, clock, d = make_agent()
    assert agent._depth_ranger is None
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("UGV", u=640.0, h=220.0, w=160.0, team="RED")]
    agent._ingest_oak(1.0)
    assert "oak_depth" not in agent.stats
    assert agent.tracker.depth_stats == {} or sum(agent.tracker.depth_stats.values()) == 0


def test_depth_frame_gives_the_ugv_track_a_depth_range_and_boosts_a_near_iff_vote():
    agent, robot, tr, clock, d = _agent(depth_range=True)
    clock.t = 1.0
    push_oak(robot, clock)
    depth, box, ugv = _ugv(1.2, "RED")
    _push_depth(robot, depth, 1.0)
    d.oak = [ugv]
    agent._ingest_oak(1.0)
    st = agent.stats["oak_depth"]
    assert st["ok"] == 1 and agent.tracker.depth_stats["iff_boost"] == 1
    assert agent.tracker.last_depth_range is not None and agent.tracker.last_depth_range[1] == pytest.approx(1.2, abs=0.05)
    assert agent.tracker.last_depth_range[2] == "RED"
    tracks = agent.tracker.enemies(1.1)
    assert len(tracks) == 1
    assert agent._dets == []                                                   # 조준·사격이 보는 목록에는 들어가지 않는다


def test_far_ugv_keeps_its_position_but_loses_the_unreliable_antenna_colour():
    agent, robot, tr, clock, d = _agent(depth_range=True, iff_max_range_m=1.0)
    clock.t = 1.0
    push_oak(robot, clock)
    depth, box, ugv = _ugv(1.6, "RED")
    _push_depth(robot, depth, 1.0)
    d.oak = [ugv]
    agent._ingest_oak(1.0)
    assert agent.tracker.depth_stats["iff_far"] == 1 and agent.tracker.last_depth_range[2] is None
    assert agent.tracker.tracks                                                # 위치 증거는 남는다


def test_no_depth_frame_falls_back_to_the_size_path_and_counts_it():
    agent, robot, tr, clock, d = _agent(depth_range=True)
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("UGV", u=640.0, h=220.0, w=160.0, team="RED")]
    agent._ingest_oak(1.0)                                                     # robot.get_depth() 는 None
    assert agent.stats["oak_depth"]["no_frame"] >= 1 and agent.stats["oak_depth"]["ok"] == 0
    assert agent.tracker.tracks and agent.tracker.last_depth_range is None


def test_stale_depth_frame_is_not_paired_with_the_detection():
    agent, robot, tr, clock, d = _agent(depth_range=True, depth_max_dt_s=0.35)
    clock.t = 1.0
    push_oak(robot, clock)
    depth, box, ugv = _ugv(1.2, "RED")
    _push_depth(robot, depth, 0.2)                                             # 0.8 s 전 영상
    d.oak = [ugv]
    agent._ingest_oak(1.0)
    assert agent.stats["oak_depth"]["no_frame"] >= 1 and agent.tracker.last_depth_range is None


def test_ugv_only_feeds_the_tracker_but_never_the_object_map():
    agent, robot, tr, clock, d = _agent(depth_range=True, ugv_only=True)
    clock.t = 1.0
    push_oak(robot, clock)
    depth, box, ugv = _ugv(1.2, "RED")
    _push_depth(robot, depth, 1.0)
    d.oak = [det("TANK", conf=0.9, u=300.0), ugv]
    agent._ingest_oak(1.0)
    assert agent.stats["oak_ugv_dets"] == 1 and agent.stats["oak_depth"]["ok"] == 1
    assert agent.omap.faces == [] and agent.stats.get("oak_identify_obs", 0) == 0     # 면 득표 0 — 면 식별은 팬틸트만
    assert agent.tracker.tracks


def test_ugv_only_without_a_ugv_detection_changes_nothing():
    agent, robot, tr, clock, d = _agent(depth_range=True, ugv_only=True)
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("TANK", conf=0.9, u=300.0)]
    agent._ingest_oak(1.0)
    assert agent.omap.faces == [] and agent.stats["oak_ugv_dets"] == 0 and not agent.tracker.tracks


def test_depth_without_ugv_only_still_votes_faces_as_before():
    agent, robot, tr, clock, d = _agent(depth_range=True, ugv_only=False)
    clock.t = 1.0
    push_oak(robot, clock)
    d.oak = [det("TANK", conf=0.8, u=640.0, h=200.0, w=160.0)]
    agent._ingest_oak(1.0)
    assert agent.omap.faces and agent.omap.faces[0].votes["TANK"] == pytest.approx(0.8 * 0.5)


def test_misaligned_depth_config_is_ignored_not_trusted():
    agent, robot, tr, clock, d = _agent(depth_range=True, depth_aligned_to_rgb=False)
    clock.t = 1.0
    push_oak(robot, clock)
    depth, box, ugv = _ugv(1.2, "RED")
    _push_depth(robot, depth, 1.0)
    d.oak = [ugv]
    agent._ingest_oak(1.0)
    assert agent.stats["oak_depth"]["not_aligned"] >= 1 and agent.tracker.last_depth_range is None and agent.tracker.tracks
