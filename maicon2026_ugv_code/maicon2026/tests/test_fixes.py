"""코드 검토에서 확인된 결함 3개의 수정 검증: ① M1/M3 제출 영구 포기 ② 카메라-레이저 시차 ③ 프레임 보관 메모리."""
import math

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.link.base import ServerLink
from maicon26.mission.aim import AimController
from maicon26.mission.submit import Submitter
from maicon26.sim.run import build_match
from maicon26.sim.subgame import run_try
from maicon26.types import Detection, Frame


# ====================================================================== ① 제출은 STOP 전까지 계속 시도한다
class _Srv(ServerLink):
    def __init__(self, fails):
        self.fails, self.calls = fails, []

    def poll(self):
        return []

    def submit(self, kind, payload):
        self.calls.append(kind)
        if self.fails > 0:
            self.fails -= 1
            return False
        return True


def _pump(sub, t0, t1, step=0.1):
    t = t0
    while t <= t1:
        sub.update(t)
        t += step


def test_unlimited_retries_survive_a_long_outage_with_backoff():
    srv = _Srv(fails=12)                                   # 예전 한도(6회)의 두 배 거부
    sub = Submitter(srv, retry_s=1.0, ack_timeout_s=3.0)   # max_tries 기본 = 무제한
    sub.request("m1", {"a": 1}, 0.0, backoff=True)
    _pump(sub, 0.0, 120.0)
    assert sub.acked("m1") and sub.tries("m1") == 13 and sub.rejections("m1") == 12


def test_backoff_gaps_grow_to_a_cap():
    srv = _Srv(fails=10 ** 6)
    sub = Submitter(srv, retry_s=1.0, ack_timeout_s=3.0)
    sub.request("m3", {"a": 1}, 0.0, backoff=True)
    sent = []
    t = 0.0
    last = 0
    while t < 60.0:
        sub.update(t)
        if len(srv.calls) > last:
            sent.append(round(t, 1)); last = len(srv.calls)
        t += 0.1
    gaps = [round(b - a, 1) for a, b in zip(sent, sent[1:])]
    assert all(abs(g - e) <= 0.25 for g, e in zip(gaps[:5], [1.0, 2.0, 4.0, 5.0, 5.0])), gaps    # 1, 2, 4 s 로 늘다가 5 s 에서 멈춘다
    assert max(gaps) <= 5.3                                # 상한 retry_s × 5 — 서버가 복구되면 몇 초 안에 접수된다


def test_m6_does_not_back_off_and_stop_cancels_pending():
    srv = _Srv(fails=10 ** 6)
    sub = Submitter(srv, retry_s=1.0)
    sub.request("m6", {"r": []}, 0.0, deadline=30.0, max_tries=10 ** 6)               # backoff 없음: 마감까지 매초
    _pump(sub, 0.0, 30.5)
    assert len(srv.calls) >= 25                            # 약 1.1 s 간격(0.1 s 구동 단위) × 30 s
    sub2 = Submitter(_Srv(10 ** 6), retry_s=1.0)
    sub2.request("m1", {}, 0.0, backoff=True)
    sub2.close_for_stop(("m6",))
    _pump(sub2, 0.0, 10.0)
    assert sub2.state("m1") == "cancelled" and sub2.server.calls == []                # STOP 뒤에는 더 보내지 않는다


def test_finite_limit_can_be_revived_until_stop():
    srv = _Srv(fails=10 ** 6)
    sub = Submitter(srv, retry_s=1.0, max_tries=3)
    sub.request("m1", {"a": 1}, 0.0)
    _pump(sub, 0.0, 10.0)
    assert sub.state("m1") == "gave_up"
    assert sub.revive("m1", 10.0) and sub.state("m1") == "pending"
    srv.fails = 0
    _pump(sub, 10.0, 15.0)
    assert sub.acked("m1")
    sub.close_for_stop(("m6",))
    assert not sub.revive("m1", 20.0)


def test_agent_gets_m1_and_m3_through_after_a_long_server_outage():
    from test_state_machine import Wrap, _advance
    cfg = Config()
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    m = build_match(seed=2, cfg=cfg, teams=["B2"])
    a = m.agents[0]
    w = Wrap(a.server)
    a.server = w
    a.submitter.server = w
    w.fail = {"m1": 12, "m3": 12}                           # 예전에는 6회째에 영구 포기
    _advance(m, 120.0)
    assert a.submitter.acked("m1") and a.submitter.tries("m1") == 13


# ====================================================================== ② 카메라-레이저 시차는 거리별로 보정한다
def _det_at_center(cls="RADAR_FACILITY"):
    d = Detection(cls, 0.9, (300.0, 200.0, 340.0, 280.0), 1.0)
    d.extra["sensors"] = [(320.0, 240.0, 6.0, 0.9)]         # 센서가 화면 정중앙
    return d


def test_parallax_compensation_shifts_the_desired_bearing_by_range():
    cfg = Config()
    cfg.aim.laser_lateral_m = 0.04                           # 레이저가 카메라보다 4 cm 왼쪽
    AimController(cfg).cam.f
    errs = {}
    for r in (1.0, 2.0):
        _, _, res = AimController(cfg).update(_det_at_center(), 0.0, 0.0, 0.0, r)
        errs[r] = res.err_pan
        # 센서가 정중앙인데 레이저가 왼쪽에 있으므로 '표적이 더 왼쪽에 보여야' 한다 → 오른쪽(−)으로 돌려야 함
        assert res.err_pan == pytest.approx(-math.degrees(math.atan2(0.04, r)), abs=0.05)
    assert abs(errs[1.0]) > abs(errs[2.0]) * 1.8             # 가까울수록 시차가 크다


def test_vertical_offset_and_zero_offset_are_neutral():
    cfg = Config()
    _, _, r0 = AimController(cfg).update(_det_at_center(), 0.0, 0.0, 0.0, 1.5)
    assert abs(r0.err_pan) < 1e-6 and abs(r0.err_tilt) < 1e-6          # 오프셋 0 이면 이전 동작과 같다
    cfg.aim.laser_vertical_m = 0.03                                     # 레이저가 3 cm 위 → 표적이 위쪽에 보여야 → 아래(−)로 틸트
    _, _, r1 = AimController(cfg).update(_det_at_center(), 0.0, 0.0, 0.0, 1.0)
    assert r1.err_tilt < -1.0


def _subgame_ok(lat, vert, comp, seeds=range(3), box="face"):
    cfg = Config()
    cfg.laser.sim_origin_lateral_m, cfg.laser.sim_origin_vertical_m = lat, vert        # 시뮬의 숨은 발광부 위치
    cfg.perception.sim_target_box = box
    if comp:
        cfg.aim.laser_lateral_m, cfg.aim.laser_vertical_m = lat, vert                  # Agent 가 아는 값
    return [run_try(s, cfg)["ok"] for s in seeds]


def test_sim_hits_with_compensation_and_misses_without_it():
    """레이저가 카메라에서 4 cm 옆·3 cm 위에 있을 때: 시차를 보정하면 맞고, 안 하면(예전 동작) 못 맞힌다."""
    assert all(_subgame_ok(0.04, 0.03, comp=True))
    assert not any(_subgame_ok(0.04, 0.03, comp=False))


def test_sim_neutral_when_there_is_no_offset():
    assert all(_subgame_ok(0.0, 0.0, comp=False))


# ====================================================================== ③ 프레임 보관에 이미지가 남지 않는다
def test_kept_frames_drop_the_image_but_keep_geometry():
    cfg = Config()
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    m = build_match(seed=1, cfg=cfg, teams=["B2"])
    a, r = m.agents[0], m.robots[0]
    big = np.zeros((1944, 2592, 3), np.uint8)                # 5MP BGR 한 장 ≈ 15 MB
    orig = r.get_frame

    def with_image():
        fr = orig()
        if fr is None:
            return None
        return Frame(image=big, stamp=fr.stamp, pan=fr.pan, tilt=fr.tilt, width=fr.width, height=fr.height, meta=fr.meta)

    r.get_frame = with_image
    for _ in range(int(6 / 0.05)):
        a.tick()
        m.world.step(0.05)
    assert len(a._frames) > 8
    assert all(f.image is None for f in a._frames)           # 16장을 쌓아도 이미지는 없다(≈ 240 MB 절약)
    assert all(f.width == 640 and f.stamp > 0 for f in a._frames)
    assert a._frame is not None and a._frame.image is big     # 가장 최근 한 장만 원본을 유지한다
    assert a.stats["det_dropped"] == 0                        # 짝짓기는 그대로 동작
