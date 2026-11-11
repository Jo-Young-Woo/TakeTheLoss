"""코드 검토 ④~⑦ 수정 검증: ④ LiDAR 자기 가림 ⑤ 후진 안전 제한 ⑥ 검출기 오류 노출 ⑦ 서버 연결 오류 노출."""
import math
import time

import numpy as np
import pytest

from maicon26.config import Config, LidarCfg
from maicon26.link.async_link import AsyncServerLink
from maicon26.link.base import ServerLink
from maicon26.link.http import HttpServerLink
from maicon26.mission.manager import Mode
from maicon26.nav.safety import SafetyLimiter
from maicon26.nav.selfmask import SelfMask
from maicon26.perception.detector import AsyncDetector, Detector
from maicon26.sim.run import build_match
from maicon26.types import Detection, Frame, Scan


def _scan(fn, n=360, rmax=8.0):
    ang = np.radians(np.arange(n) * 360.0 / n - 180.0)
    r = np.array([fn(float(np.degrees(a))) for a in ang], dtype=float)
    return Scan(float(ang[0]), float(ang[1] - ang[0]), r, 1.0, rmax)


def _solo(cfg=None, seed=1):
    cfg = cfg or Config()
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    m = build_match(seed=seed, cfg=cfg, teams=["B2"])
    return m, m.agents[0], m.robots[0]


def _run(m, seconds):
    for _ in range(int(seconds / 0.05)):
        m.agents[0].tick()
        m.world.step(0.05)


# ====================================================================== ④ LiDAR 자기 가림
def test_self_mask_removes_only_near_returns_inside_the_sectors():
    cfg = LidarCfg(self_mask_sectors=((0.0, 15.0),), self_mask_range=0.35)
    sc = _scan(lambda d: 0.10 if abs(d) <= 5 else (0.20 if abs(d - 90) <= 5 else (1.0 if abs(d) <= 10 else 3.0)))
    out = SelfMask(cfg).apply(sc)
    deg = np.degrees(np.angle(np.exp(1j * out.angles)))
    assert np.isnan(out.ranges[np.abs(deg) <= 4.5]).all()                     # 마스크 뒤 공간은 미관측
    assert np.isfinite(out.ranges[np.abs(deg - 90) <= 4.5]).all()             # 각도 밖의 가까운 반사(진짜 벽)는 남긴다
    assert np.isclose(out.ranges[(np.abs(deg) > 5.5) & (np.abs(deg) <= 9.5)], 1.0).all()   # 각도 안이어도 먼 반사는 장애물로 남긴다
    assert out.stamp == sc.stamp and (sc.ranges[np.abs(deg) <= 4.5] == 0.10).all()          # 원본은 건드리지 않는다


def test_min_range_and_default_noop():
    sc = _scan(lambda d: 0.08 if abs(d - 40) <= 3 else 2.0)
    assert SelfMask(LidarCfg()).apply(sc) is sc                                # 설정이 없으면 그대로(기존 동작)
    out = SelfMask(LidarCfg(min_range=0.12)).apply(sc)
    assert np.isnan(out.ranges[np.abs(np.degrees(np.angle(np.exp(1j * out.angles))) - 40) <= 3]).all()


def _fake_mast(front_deg=10.0, rng=0.10):
    def make(orig):
        def fn():
            sc = orig()
            if sc is None:
                return None
            r = sc.ranges.copy()
            r[np.abs(np.degrees(np.angle(np.exp(1j * sc.angles)))) <= front_deg] = rng
            return Scan(sc.angle_min, sc.angle_inc, r, sc.stamp, sc.range_max)
        return fn
    return make


def test_front_mast_return_pins_the_robot_without_a_mask_and_the_mask_frees_it():
    d = {}
    for mask in (False, True):
        cfg = Config()
        if mask:
            cfg.lidar.self_mask_sectors = ((0.0, 12.0),)
        m, a, r = _solo(cfg)
        r.get_scan = _fake_mast()(r.get_scan)
        _run(m, 60.0)
        d[mask] = (r.u.distance, a)
    assert d[False][0] < 1.5                                                   # 전방 17 cm 안의 '장애물' 때문에 제자리에서 맴돈다
    assert d[True][0] > 4.0 and d[True][0] > 4 * d[False][0]                   # 마스크로 풀린다
    assert not d[True][1].stats["lidar_suspects"]


def test_unmasked_body_part_is_reported_with_a_merged_candidate_sector():
    m, a, r = _solo()
    r.get_scan = _fake_mast()(r.get_scan)
    _run(m, 40.0)
    sus = a.stats["lidar_suspects"]
    assert len(sus) == 1, sus                                                  # 이웃한 칸은 한 건으로 합쳐 경고
    assert "self_mask_sectors" in sus[0] and "0.10 m" in sus[0]
    assert any("WARN LiDAR 자기 가림 의심" in msg for _, msg in a.log)


def test_world_walls_that_move_across_bins_are_not_flagged():
    sm = SelfMask(LidarCfg())
    heading, t, flagged = 0.0, 0.0, []
    for _ in range(200):                                                       # 20 s 동안 제자리 회전(0.15 rad/s... ≈ 3 rad)
        heading += 0.015
        t += 0.1
        wall_deg = 30.0 - math.degrees(heading)                               # 월드의 벽: 로봇이 돌면 차체 기준 방향이 밀린다
        flagged += sm.observe(_scan(lambda d: 0.12 if abs(((d - wall_deg + 180) % 360) - 180) <= 4 else 3.0), heading, t)
    assert flagged == []


# ====================================================================== ⑤ 후진 안전 제한
def test_reverse_is_stopped_and_slowed_by_rear_obstacles():
    cfg = Config()
    s = SafetyLimiter(cfg)
    near = _scan(lambda d: 0.10 if abs(abs(d) - 180) <= 20 else 3.0)
    mid = _scan(lambda d: 0.30 if abs(abs(d) - 180) <= 20 else 3.0)
    far = _scan(lambda d: 1.00 if abs(abs(d) - 180) <= 20 else 3.0)
    assert s.limit(near, -0.10, 0.4, 0.0) == (0.0, 0.4)                       # 정지(회전은 그대로)
    v, w = s.limit(mid, -0.10, 0.0, 0.0)
    stop, slow = cfg.nav.stop_dist, cfg.nav.slow_dist                     # 정지·감속 거리는 차체 외곽 예산에서 파생(footprint.py)
    assert -0.10 < v < 0.0 and v == pytest.approx(-0.10 * (0.30 - stop) / (slow - stop), rel=1e-6)   # 정지~감속 거리 사이: 비례 감속
    assert s.limit(far, -0.10, 0.0, 0.0) == (-0.10, 0.0)
    fwd = _scan(lambda d: 0.10 if abs(d) <= 20 else 3.0)
    assert s.limit(fwd, -0.10, 0.0, 0.0) == (-0.10, 0.0)                      # 앞의 장애물은 후진을 막지 않는다
    assert s.limit(fwd, 0.30, 0.0, 0.0)[0] == 0.0                             # 전진은 예전처럼 막는다


def test_recover_backing_up_is_checked_every_tick():
    m, a, r = _solo()
    _run(m, 6.0)
    now = a.robot.now()
    a._rec = {"from": Mode.EXPLORE, "t0": now, "turn": 1.0, "rear_ok": True}    # 진입 때는 후방이 비어 있었다
    a._set_mode(now, Mode.RECOVER, "test")
    a._v_cmd = a._w_cmd = 0.0
    orig = r.get_scan

    def rear_blocker():                                                          # 후진 중 뒤에 다른 UGV 가 끼어든다
        sc = orig()
        rg = sc.ranges.copy()
        rg[np.abs(np.abs(np.degrees(np.angle(np.exp(1j * sc.angles)))) - 180) <= 25] = 0.12
        return Scan(sc.angle_min, sc.angle_inc, rg, sc.stamp, sc.range_max)

    r.get_scan = rear_blocker
    for _ in range(8):                                                           # 후진 구간(0.7 s) 안
        a.tick()
        m.world.step(0.05)
        assert a.mode == Mode.RECOVER and r.u.cmd_v >= 0.0, f"뒤가 막혔는데 후진 명령({r.u.cmd_v})"


# ====================================================================== ⑥ 검출기 오류 노출
class _Raiser(Detector):
    def __init__(self):
        self.fail = True

    def detect(self, frame):
        if self.fail:
            raise RuntimeError("CUDA out of memory")
        return [Detection("DRUM", 0.9, (0, 0, 10, 10), frame.stamp)]


def test_async_detector_error_is_not_disguised_as_a_fresh_empty_result():
    inner = _Raiser()
    ad = AsyncDetector(inner)
    try:
        ad.detect_stamped(Frame(None, 1.0))
        time.sleep(0.4)
        dets, stamp = ad.detect_stamped(Frame(None, 1.0))
        ok, why = ad.health()
        assert stamp == -1.0 and dets == []                                      # 새 stamp 가 붙지 않는다 → 소비되지 않고 오래된 채로 남는다
        assert not ok and "CUDA out of memory" in why and ad.error_count >= 1
        inner.fail = False
        ad.detect_stamped(Frame(None, 2.0))
        time.sleep(0.4)
        dets, stamp = ad.detect_stamped(Frame(None, 2.0))
        assert stamp == 2.0 and len(dets) == 1 and ad.health() == (True, "")      # 회복하면 정상으로 돌아온다
    finally:
        ad.close()


def test_agent_survives_a_sync_detector_that_raises_and_blocks_firing():
    m, a, r = _solo()
    a.detector = _Raiser()                                                       # detect 가 예외를 던지는 동기 검출기
    a.detector.asynchronous = False
    _run(m, 12.0)                                                                # 예전에는 tick() 이 예외로 죽었다
    assert a.stats["detector_errors"] >= 1
    assert any("WARN detector 문제" in msg and "CUDA" in msg for _, msg in a.log)
    assert not a.vision_fresh(a.robot.now()) and not a._may_fire(a.robot.now())  # 영상 신선도 감시가 발사를 막는다
    assert sum(1 for _, msg in a.log if "WARN detector 문제" in msg) <= 2         # 같은 문구를 매 프레임 쏟아내지 않는다


def test_agent_reports_a_dead_async_detector():
    m, a, r = _solo()
    inner = _Raiser()
    a.detector = AsyncDetector(inner)
    try:
        _run(m, 10.0)
    finally:
        a.detector.close()
    assert a.stats["detector_errors"] >= 1 and a.detector.error_count >= 1
    assert any("WARN detector 문제" in msg for _, msg in a.log)
    assert not a.vision_fresh(a.robot.now())


# ====================================================================== ⑦ 서버 연결 오류 노출
class _BrokenPoll(ServerLink):
    def __init__(self):
        self.ok = False

    def poll(self):
        if not self.ok:
            raise ConnectionError("서버에 연결할 수 없음")
        return []

    def submit(self, kind, payload):
        return True


def _until(fn, limit=3.0):
    t = time.monotonic()
    while time.monotonic() - t < limit:
        if fn():
            return True
        time.sleep(0.02)
    return fn()


def test_async_link_health_reports_the_reason_and_recovers():
    inner = _BrokenPoll()
    link = AsyncServerLink(inner, poll_hz=100, uav_hz=10)
    try:
        assert _until(lambda: not link.health()[0])
        ok, why = link.health()
        assert not ok and "ConnectionError" in why and "서버에 연결할 수 없음" in why
        inner.ok = True
        assert _until(lambda: link.health()[0], 4.0)                              # poll 이 성공하면 정상으로 돌아온다
    finally:
        link.close()


def test_http_link_failure_is_visible_instead_of_silent():
    link = AsyncServerLink(HttpServerLink("http://127.0.0.1:9", "AbC12345", wait_s=0, me_period_s=0.05), poll_hz=100, uav_hz=10)
    try:
        assert _until(lambda: not link.health()[0])
        ok, why = link.health()
        assert "me " in why and "AbC12345" not in why                             # 서버에 못 붙으면 이유가 드러나고 임무코드는 가려진다
    finally:
        link.close()


def test_agent_logs_link_problems_once_and_logs_recovery():
    from test_state_machine import Wrap
    m, a, r = _solo()
    w = Wrap(a.server)
    a.server = w
    a.submitter.server = w
    state = {"ok": False, "why": "poll ConnectionError: 서버에 연결할 수 없음"}
    w.health = lambda: (state["ok"], "" if state["ok"] else state["why"])
    _run(m, 4.0)
    assert sum(1 for _, msg in a.log if "WARN link 문제" in msg) == 1               # 같은 문구는 매 틱 반복하지 않는다
    assert a.stats["link_errors"] == 1
    _run(m, 16.0)
    assert 2 <= sum(1 for _, msg in a.log if "WARN link 문제" in msg) <= 3          # 계속되면 10 s 마다 한 번 다시 알린다
    state["ok"] = True
    _run(m, 17.0)
    assert any(msg == "link 회복" for _, msg in a.log)
