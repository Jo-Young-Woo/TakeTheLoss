"""본선 규정의 이벤트 요소 처리 검증 (상태전이표 6장, ID 는 6.3 의 E1~E10 과 같다):
E1 서버 경기상태 이벤트 · E2 3색 타워 보조 정지 · E3 스모그 · E4 드론 영상 주기 · E5 표적 시그널 LED · E6 오타격 방지 ·
E7 스모그 중 발사 신뢰도 · E8 UGV 후면 LED(M6 증거) · E9 거점 진입 이벤트 · E10 비인식용 박스."""
import math

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.link.base import ServerMsg
from maicon26.link.schema import normalize_event
from maicon26.mission import threat as T
from maicon26.mission.env import SmogMonitor, TowerMonitor, UavMonitor, isolated_near_mask, speckle_filter
from maicon26.mission.manager import Mode
from maicon26.perception.detector import Detector
from maicon26.perception.led import classify_tower, find_signal_led, read_ugv_leds
from maicon26.perception.object_map import ObjectMap
from maicon26.perception.tracks import Track, UgvTracker
from maicon26.sim.run import build_match
from maicon26.types import Command, Detection, Frame, Pose2D, Scan

from test_fixes2 import _run, _solo                              # → (m, agent, robot)
from test_state_machine import Wrap, _advance
from test_state_machine import _solo as _solo_w                  # → (m, agent, Wrap 서버) — 메시지 주입·삭제용

cv2 = pytest.importorskip("cv2")


def _scan(rng_fn, n=360):
    ang = np.radians(np.arange(n) * 360.0 / n - 180.0)
    r = np.array([rng_fn(i) for i in range(n)], dtype=float)
    return Scan(float(ang[0]), float(ang[1] - ang[0]), r, 1.0, 8.0)


# ====================================================================== E3 스모그: 감지 부품
def _clean(i):
    return 0.5 if 100 <= i < 140 else 3.0                      # 벽 3 m + 0.5 m 앞의 박스(연속된 면: 40 빔)


def _smoky(seed, k=50):
    rng = np.random.default_rng(seed)
    free = [i for i in range(360) if not 100 <= i < 140]                       # 박스 면(100~139)에는 입자를 찍지 않는다
    idx = set(rng.choice(free, k, replace=False).tolist())
    vals = {i: float(rng.uniform(0.1, 1.0)) for i in idx}
    return _scan(lambda i: vals.get(i, _clean(i)))


def _feed(mon, make, t0, seconds, dt=0.1):
    t, ch = t0, []
    while t < t0 + seconds:
        c = mon.update(make(int(t * 10)), t)
        if c is not None:
            ch.append((round(t, 1), c))
        t += dt
    return ch


def test_coherent_surfaces_and_thin_poles_are_not_smog():
    mon = SmogMonitor(Config().event)
    assert _feed(mon, lambda k: _scan(_clean), 0.0, 6.0) == [] and not mon.active and mon.index < 0.01
    pole = _scan(lambda i: 0.4 if i in (200, 201) else _clean(i))              # 다른 UGV 안테나 같은 가는 막대(빔 2개)
    assert _feed(mon, lambda k: pole, 6.0, 6.0) == [] and not mon.active


def test_smog_turns_on_after_a_sustained_burst_and_off_only_after_clearing_with_hysteresis():
    mon = SmogMonitor(Config().event)
    ch = _feed(mon, lambda k: _smoky(k), 0.0, 3.0)
    assert ch and ch[0][1] is True and 0.7 <= ch[0][0] <= 1.1                    # ≈ smog_on_s(0.8 s) 뒤에 켠다
    assert mon.active and mon.index > 0.04
    ch2 = _feed(mon, lambda k: _scan(_clean), 3.0, 6.0)
    assert ch2 and ch2[0][1] is False and 2.9 <= ch2[0][0] - 3.0 <= 3.4         # ≈ smog_off_s(3 s) 뒤에 끈다


def test_a_single_noisy_scan_does_not_flip_the_state():
    mon = SmogMonitor(Config().event)
    mon.update(_smoky(1), 0.0)
    assert _feed(mon, lambda k: _scan(_clean), 0.1, 3.0) == [] and not mon.active


def test_server_event_overrides_and_returns_control_to_sensors():
    mon = SmogMonitor(Config().event)
    assert mon.force(True) is True and mon.active
    assert _feed(mon, lambda k: _scan(_clean), 0.0, 6.0) == [] and mon.active    # 서버가 켰으면 센서가 깨끗해 보여도 유지
    assert mon.force(False) is False and not mon.active
    mon.force(None)
    assert _feed(mon, lambda k: _smoky(k), 10.0, 3.0)[0][1] is True              # 다시 센서 판단


def test_low_image_contrast_adds_evidence_for_smog():
    mon = SmogMonitor(Config().event)
    weak = lambda k: _scan(lambda i: 0.3 if i in (50, 90, 130, 170, 210, 250, 290, 330, 20, 70) else _clean(i))   # 고립 10개 ≈ 0.028
    t = 0.0
    for _ in range(45):                                                          # 연기 전: 영상 대비 50 → 기준
        mon.update(_scan(_clean), t, 50.0)
        t += 0.1
    assert mon.update(weak(0), t, 50.0) is None and not mon.active              # 대비가 정상이면 약한 고립만으로는 안 켠다
    ch = []
    for _ in range(30):
        c = mon.update(weak(0), t, 20.0)                                         # 대비가 기준의 40 % 로 떨어짐 + 약한 고립 반사
        if c is not None:
            ch.append(c)
        t += 0.1
    assert ch == [True] and mon.contrast_low


def test_speckle_filter_removes_isolated_near_returns_but_keeps_surfaces():
    sc = _smoky(5)
    out = speckle_filter(sc, 1.2, 0.25)
    iso = isolated_near_mask(sc, 1.2, 0.25)
    assert iso.sum() >= 30 and np.isnan(out.ranges[iso]).all()
    assert np.isclose(out.ranges[100:140][~iso[100:140]], 0.5).all()           # 박스 면(연속 40 빔)은 남는다
    assert np.isfinite(out.ranges[~iso & (sc.ranges < 5)]).all()
    clean = _scan(_clean)
    assert speckle_filter(clean, 1.2, 0.25) is clean                            # 깨끗하면 그대로(복사 없음)


# ====================================================================== E3 스모그: 에이전트 시나리오
def _smog_run(handle, announce, seed=1, t_on=15.0, t_off=55.0, T=70.0):
    cfg = Config()
    cfg.event.smog_enable = handle
    m, a, r = _solo(cfg, seed)
    w = m.world
    w.smog_center, w.smog_radius = (2.0, 2.5), 4.5                              # 로봇이 다니는 구역에 연기
    d0 = d1 = None
    while w.t < T:
        if d0 is None and w.t >= t_on:
            w.set_smog(True, announce)
            d0 = r.u.distance
        if d1 is None and w.t >= t_off:
            w.set_smog(False, announce)
            d1 = r.u.distance
        a.tick()
        w.step(0.05)
    return a, d1 - d0


def test_smog_handling_keeps_the_robot_moving_while_phantom_returns_pin_it_otherwise():
    on, off = _smog_run(True, False)[1], _smog_run(False, False)[1]
    assert off < 2.0                                                            # 처리 없이는 가짜 반사(8~20 cm)가 전방 정지 규칙을 계속 건다
    assert on > 3 * off and on > 2.5                                            # 감지·필터가 있으면 계속 움직인다(40 s 구간)


def test_smog_is_detected_from_the_lidar_alone_and_released_after_it_clears():
    a, _ = _smog_run(True, False)
    msgs = [(t, m) for t, m in a.log if m.startswith("SMOG")]
    assert [m.split()[1] for _, m in msgs] == ["ON", "OFF"] and "src=sensor" in msgs[0][1]
    assert 15.0 <= msgs[0][0] <= 22.0 and 55.0 <= msgs[1][0] <= 63.0
    assert a.stats["smog_changes"] == 2 and 35.0 < a.stats["smog_s"] < 50.0


def test_smog_event_from_the_server_is_followed_immediately():
    a, dist = _smog_run(True, True)
    msgs = [(t, m) for t, m in a.log if m.startswith("SMOG")]
    assert "src=server" in msgs[0][1] and abs(msgs[0][0] - 15.0) < 0.2 and "src=server" in msgs[1][1]
    assert abs(msgs[1][0] - 55.0) < 0.3 and dist > 3.0


def test_smog_slows_navigation_and_raises_the_firing_confidence_bar():
    m, a, r = _solo()
    goals = []
    orig = a.stack.set_goal
    a.stack.set_goal = lambda goal, tol=None, speed_scale=1.0: (goals.append(speed_scale), orig(goal, tol, speed_scale))[1]
    a.smog.force(True)
    _run(m, 4.0)
    a.smog.force(False)
    _run(m, 8.0)
    assert goals and min(goals) == pytest.approx(0.8) and max(goals) == pytest.approx(1.0)
    weak, strong = Detection("RADAR_FACILITY", 0.45, (0, 0, 10, 10), 1.0), Detection("RADAR_FACILITY", 0.75, (0, 0, 10, 10), 1.0)
    a.smog.force(True)
    assert not a._fire_conf_ok(weak) and a._fire_conf_ok(strong)               # 스모그 중: 신뢰도 0.60 미만은 발사 보류
    a.smog.force(False)
    assert a._fire_conf_ok(weak)                                                # 평소에는 영향 없음


def test_objects_far_away_are_not_counted_during_smog_and_nonobjects_are_ignored():
    cfg = Config()
    om = ObjectMap(cfg)
    fr = Frame(None, 1.0, 0.0, 0.0, 640, 480)
    far = Detection("AMMO_BOX", 0.9, (300, 200, 340, 260), 1.0)                 # 높이 60 px → bbox 크기로 ≈ 1.5 m
    om.observe([far], Pose2D(1.0, 1.0, 0.0), fr, None, max_range=1.0)
    assert not om.faces                                                         # 스모그 중(집계 거리 1.0 m)에는 1.5 m 객체를 세지 않는다
    om.observe([far], Pose2D(1.0, 1.0, 0.0), fr, None)
    assert len(om.faces) == 1
    om2 = ObjectMap(cfg)
    om2.observe([Detection("TREE", 0.95, (300, 200, 360, 300), 1.0), Detection("BUILDING", 0.9, (100, 100, 200, 220), 1.0)],
                Pose2D(1.0, 1.0, 0.0), fr, None)
    assert not om2.faces and om2.nonobject_seen == 2                            # 비인식용 박스는 세지 않고 통계만


# ====================================================================== E1 서버 이벤트
def test_event_aliases_are_normalized():
    assert normalize_event({"type": "state_change", "state": "완료"}) == {"type": "game_state", "state": "finished"}
    assert normalize_event({"event": "match_state", "status": "RUNNING"})["state"] == "running"
    assert normalize_event({"type": "smoke", "on": "ON"})["on"] is True
    assert normalize_event({"type": "fog", "active": False})["on"] is False
    assert normalize_event({"type": "base_reached", "by": "B-UGV-1"})["type"] == "base_entered"
    assert normalize_event({"type": "ugv_hit", "by": "x"})["type"] == "ugv_hit"
    assert normalize_event({"type": "mystery"})["type"] == "mystery"


def test_game_state_finished_event_stops_the_robot_even_before_slam_is_ready():
    m, a, w = _solo_w(seed=2)
    a.robot.get_scan = lambda: None                                             # SLAM 불가 상태
    w.inject = [(0.2, ServerMsg("event", data={"type": "state_change", "state": "ready"})),
                (0.4, ServerMsg("event", data={"type": "state_change", "state": "종료"}))]
    _advance(m, 3.0)
    assert a.mode == Mode.DONE and a.stop_source == "server_event"
    assert a.stats["game_events"] == {"game_state": 2}
    assert a.robot.u.cmd_v == 0.0


def test_event_of_another_match_does_not_stop_this_one():
    m, a, w = _solo_w(seed=2, match_id="M-1")
    w.inject = [(3.0, ServerMsg("event", data={"type": "game_state", "state": "finished", "match_id": "M-9"}))]
    _advance(m, 6.0)
    assert a.mode not in (Mode.HALT, Mode.REPORT, Mode.DONE) and a.stats["msg_rejected"].get("other-match") == 1


def test_base_entered_event_is_recorded_and_own_entry_is_confirmed():
    m, a, w = _solo_w(seed=2)
    w.inject = [(2.0, ServerMsg("event", data={"type": "base_reached", "by": "R-UGV-1", "base": "B-2"})),
                (3.0, ServerMsg("event", data={"type": "base_reached", "by": "B-UGV-2", "base": "R-3"}))]
    _advance(m, 5.0)
    assert [e[1] for e in a.stats["base_entries"]] == ["R-UGV-1", "B-UGV-2"]
    assert a.m5_confirmed_event is not None and abs(a.m5_confirmed_event - 3.0) < 0.3


# ====================================================================== E5 표적 시그널 LED
def _target_img():
    im = np.full((480, 640, 3), (140, 150, 120), np.uint8)
    cv2.rectangle(im, (300, 200), (400, 330), (120, 160, 110), -1)
    return im, (300.0, 200.0, 400.0, 330.0)


def test_signal_led_lit_red_is_found_and_other_things_are_not():
    im, box = _target_img()
    assert not find_signal_led(im, box)                                         # 꺼져 있음
    lit = im.copy()
    cv2.circle(lit, (350, 192), 6, (0, 0, 255), -1)                             # 표적 상단 위쪽에 빨간 LED 점등
    assert find_signal_led(lit, box)
    green = im.copy()
    cv2.circle(green, (350, 192), 6, (0, 255, 0), -1)
    assert not find_signal_led(green, box)                                      # 빨간색이 아니면 아니다
    low = im.copy()
    cv2.circle(low, (350, 320), 6, (0, 0, 255), -1)                             # 사진 아래쪽의 빨간 물체(예: 트럭) — 탐색 영역 밖
    assert not find_signal_led(low, box)
    wide = im.copy()
    cv2.rectangle(wide, (300, 190), (400, 240), (0, 0, 255), -1)                # 큰 빨간 면은 LED 가 아니다
    assert not find_signal_led(wide, box) and not find_signal_led(None, box)


def test_led_on_needs_consecutive_confirmation_in_the_object_map():
    om = ObjectMap(Config())
    fr = Frame(None, 1.0, 0.0, 0.0, 640, 480)
    mk = lambda on: Detection("RADAR_FACILITY", 0.9, (300, 200, 340, 280), 1.0, extra={"led_on": on})
    pose = Pose2D(1.0, 1.0, 0.0)
    om.observe([mk(True)], pose, fr, None)
    assert not om.faces[0].led_on                                               # 한 프레임 오검출로 '타격 완료'가 되지 않는다
    om.observe([mk(False)], pose, fr, None)
    om.observe([mk(True)], pose, fr, None)
    om.observe([mk(True)], pose, fr, None)
    assert not om.faces[0].led_on and om.faces[0].led_votes == 2                # 중간에 꺼짐이 보이면 표가 깎인다
    om.observe([mk(True)], pose, fr, None)
    assert om.faces[0].led_on                                                   # 3 프레임 연속 확인 → 점등


def test_signal_led_confirms_m2_visually_without_a_server_event():
    m, a, r = _solo(seed=1)
    w = Wrap(a.server)
    w.drop = {"event"}                                                          # 서버의 target_hit 이벤트가 오지 않아도
    a.server = w
    a.submitter.server = w
    _run(m, 70.0)
    assert a.target_done and a.stats["led_confirmed"] >= 1                      # LED 점등 영상으로 M2 완료를 안다
    assert any("signal LED lit" in msg for _, msg in a.log)


# ====================================================================== E8 UGV 후면 LED → M6 증거
def _ugv_img(color_bgr, n, y=280, spacing=16):
    im = np.full((480, 640, 3), (50, 50, 55), np.uint8)
    cv2.rectangle(im, (200, 150), (360, 300), (30, 30, 32), -1)
    for i in range(n):
        cv2.circle(im, (225 + i * spacing, y), 4, color_bgr, -1)
    return im, (195.0, 145.0, 365.0, 305.0)


YEL, BLUE, RED, GREEN, ORANGE = (0, 255, 255), (255, 0, 0), (0, 0, 255), (0, 255, 0), (0, 128, 255)


def test_rear_leds_are_read_as_hits_by_color_and_ammo_by_count():
    for color, hits in ((BLUE, 0), (GREEN, 1), (YEL, 2), (ORANGE, 3), (RED, 4)):
        im, box = _ugv_img(color, 5)
        led = read_ugv_leds(im, box)
        assert led is not None and led["hits"] == hits and led["ammo"] == 5 and led["conf"] >= 0.7, (color, led)
    im, box = _ugv_img(BLUE, 8)
    assert read_ugv_leds(im, box)["ammo"] == 8
    im, box = _ugv_img(BLUE, 0)
    assert read_ugv_leds(im, box) is None and read_ugv_leds(None, box) is None  # 못 읽으면 증거 없음(틀린 증거보다 낫다)


def test_tower_colors_are_classified():
    for color, name in ((RED, "red"), (GREEN, "green"), (YEL, "yellow")):
        im = np.full((480, 640, 3), (40, 40, 40), np.uint8)
        cv2.circle(im, (120, 130), 9, color, -1)
        got = classify_tower(im, (100.0, 100.0, 140.0, 200.0))
        assert got is not None and got[0] == name and got[1] >= 0.5
    assert classify_tower(np.full((480, 640, 3), 40, np.uint8), (100.0, 100.0, 140.0, 200.0)) is None


def test_led_readings_are_median_filtered_and_monotone():
    tr = Track(1, 5.0, 1.0, 10.0, ident="R-UGV-1")
    for h, am in ((0, 8), (0, 8), (2, 3), (2, 3), (2, 3)):
        UgvTracker._add_led(tr, h, am, 10.0)
    assert (tr.led_hits, tr.led_ammo) == (2, 3)
    for _ in range(5):                                                          # 가려져서 '0 피격·8발'처럼 읽혀도 피격은 줄지 않고 탄은 늘지 않는다
        UgvTracker._add_led(tr, 0, 8, 20.0)
    assert (tr.led_hits, tr.led_ammo) == (2, 3)
    UgvTracker._add_led(tr, 9, -3, 30.0)                                        # 범위를 벗어난 값은 보정
    assert tr.led_hits <= 4 and tr.led_ammo >= 0


def test_m6_ranking_uses_led_hits_and_ammo_when_they_were_read():
    cfg = Config()
    tracker = UgvTracker(cfg, "BLUE")
    for ident, hits, ammo in (("R-UGV-1", 3, 1), ("R-UGV-2", 0, 8), ("R-UGV-3", 1, 4)):
        tr = Track(int(ident[-1]), 5.0, 1.0, 10.0, ident=ident, n_obs=10)
        tr.sectors = {"R-S1", "R-S2"}
        tr.travel = 6.0
        UgvTracker._add_led(tr, hits, ammo, 10.0)
        tracker.tracks.append(tr)
    evs = T.score(T.evidence_from_tracks(tracker, "BLUE"))
    assert [e.ident for e in evs] == ["R-UGV-2", "R-UGV-3", "R-UGV-1"]          # 안 맞고 탄이 많은 쪽이 가장 위험
    assert all(e.hits_known and e.ammo_known for e in evs)
    assert evs[0].ammo == pytest.approx(1.0) and evs[2].surv == pytest.approx(0.25)
    for tr in tracker.tracks:                                                   # LED 를 못 읽은 트랙은 예전처럼 중간값(0.6)
        tr.led_hits = tr.led_ammo = None
    ev2 = T.score(T.evidence_from_tracks(tracker, "BLUE"))
    assert all((not e.ammo_known) and e.ammo == pytest.approx(0.6) for e in ev2)


def test_sim_exposes_the_rear_led_only_from_behind_and_close():
    m = build_match(seed=1, cfg=Config(), teams=["B1", "R1"])
    w = m.world
    w.cubes = []
    me, other = w.ugvs[0], w.ugvs[1]
    other.hits_taken, other.ammo = 2, 5
    me.pose.x, me.pose.y, me.pose.th, me.pan = 2.0, 2.5, 0.0, 0.0
    other.pose.x, other.pose.y = 3.0, 2.5

    def led_for(th):
        other.pose.th = th
        for _ in range(40):                                                     # 시뮬은 검출을 5 % 확률로 버린다 → 여러 번 시도
            got = [d for d in w.camera_detections(me) if d.cls == "UGV"]
            if got:
                return got
        return []

    behind = led_for(0.0)                                                       # 상대가 나와 같은 방향을 봄 → 내 카메라는 상대의 뒤
    assert behind and behind[0].extra["led"]["hits"] == 2.0 and abs(behind[0].extra["led"]["ammo"] - 5) <= 1
    facing = led_for(math.pi)                                                   # 상대가 나를 마주 봄 → 앞쪽만 보인다
    assert facing and "led" not in facing[0].extra and "sensors" in facing[0].extra
    other.pose.x = 4.5
    far = led_for(0.0)
    assert far and "led" not in far[0].extra                                    # 1.8 m 보다 멀면 못 읽는다


# ====================================================================== E2 3색 타워 LED
class _TowerSeen(Detector):
    def __init__(self, colors, conf=0.9):
        self.colors, self.conf, self.i = colors, conf, 0

    def detect(self, frame):
        c = self.colors[self.i % len(self.colors)]
        self.i += 1
        return [Detection("TOWER_LED", self.conf, (100, 100, 140, 200), frame.stamp, extra={"tower_color": c})]


def _link_down(a, w):
    w.health = lambda: (False, "poll ConnectionError: 서버에 연결할 수 없음")


def test_tower_monitor_needs_enough_consistent_observations():
    tm = TowerMonitor(Config().event)
    assert tm.state_of("red") == "finished" and tm.state_of("green") == "running" and tm.state_of("yellow") == "ready"
    assert tm.state_of("purple") is None
    t = 0.0
    for i in range(30):                                                         # 15 Hz 로 2 s 동안 red
        tm.observe("red", t)
        t += 1 / 15
    assert tm.confirmed("finished", t) and not tm.confirmed("running", t)
    tm2 = TowerMonitor(Config().event)
    t = 0.0
    for i in range(30):                                                         # red/green 이 번갈아 → 50 % 라 확인 안 됨
        tm2.observe("red" if i % 2 else "green", t)
        t += 1 / 15
    assert not tm2.confirmed("finished", t)


def _tower_world(colors, link_down, start_seen=30.0, run_to=75.0, conf=0.9):
    m, a, r = _solo(seed=2)
    w = Wrap(a.server)
    a.server = w
    a.submitter.server = w
    w.drop = {"stop"}                                                           # 서버 STOP 이 오지 않는다(유실)
    if link_down:
        _link_down(a, w)
    a.detector = _TowerSeen(colors, conf)
    a.detector.asynchronous = False
    orig = r.get_frame
    r.get_frame = lambda: orig() if m.world.t >= start_seen else None          # 타워는 start_seen 이후부터 보인다
    _advance(m, run_to)
    return a, m


def test_tower_finished_halts_only_when_the_server_link_is_down_and_after_the_minimum_time():
    a, m = _tower_world(["red"], link_down=True, start_seen=30.0, run_to=55.0)
    assert a.mode not in (Mode.HALT, Mode.REPORT, Mode.DONE)                    # 시작 후 60 s 전에는 타워로 멈추지 않는다
    _advance(m, 68.0)
    assert a.mode in (Mode.REPORT, Mode.DONE) and a.stop_source == "tower"
    assert any("3색 타워" in msg for _, msg in a.log) and a.robot.u.cmd_v == 0.0


def test_tower_finished_is_ignored_while_the_server_link_is_healthy():
    a, m = _tower_world(["red"], link_down=False, start_seen=30.0, run_to=75.0)
    assert a.mode not in (Mode.HALT, Mode.REPORT, Mode.DONE) and a.stats["tower_obs"] > 100   # STOP 메시지를 기다린다


def test_tower_running_or_flickering_colors_never_halt():
    for colors in (["green"], ["red", "green"], ["yellow"], ["purple"]):
        a, m = _tower_world(colors, link_down=True, start_seen=30.0, run_to=75.0)
        assert a.mode not in (Mode.HALT, Mode.REPORT, Mode.DONE), colors


def test_low_confidence_tower_detection_is_not_used():
    a, m = _tower_world(["red"], link_down=True, start_seen=30.0, run_to=75.0, conf=0.4)
    assert a.mode not in (Mode.HALT, Mode.REPORT, Mode.DONE) and a.stats["tower_obs"] == 0


# ====================================================================== E4 드론 영상 주기
def test_uav_monitor_classifies_the_period_and_detects_a_cutoff():
    mon = UavMonitor(Config().event)
    t = 0.0
    for _ in range(6):                                                          # 1 s 간격
        mon.observe(t)
        t += 1.0
    assert mon.state == "normal" and mon.track_max_age() == 3.0
    for _ in range(5):                                                          # 2 s 간격(상대가 표적 1개 유효타격)
        mon.observe(t)
        t += 2.0
    assert mon.state == "slow2" and mon.track_max_age() == pytest.approx(3.2)
    for _ in range(5):                                                          # 5 s 간격
        mon.observe(t)
        t += 5.0
    assert mon.state == "slow5" and mon.track_max_age() == pytest.approx(8.0)
    mon.tick(t - 5.0 + 4.0)
    assert mon.state == "slow5"
    mon.tick(t - 5.0 + 9.0)                                                     # 새 프레임이 8 s 넘게 없다 → 송출 중단(3개 모두 유효타격)
    assert mon.state == "off" and mon.track_max_age() == 3.0
    assert [s for _, s in mon.changes] == ["normal", "slow2", "slow5", "off"]


def test_agent_follows_the_drone_stream_through_slowdown_and_cutoff():
    m, a, r = _solo(seed=2)
    w = m.world
    plan = [(25.0, 2.0), (55.0, 5.0), (95.0, math.inf)]
    seen = []
    while w.t < 125.0:
        if plan and w.t >= plan[0][0]:
            w.set_uav_period(plan.pop(0)[1])
        a.tick()
        w.step(0.05)
        if not seen or a.uavmon.state != seen[-1]:
            seen.append(a.uavmon.state)
    assert seen[0] == "unknown" and seen[1:] == ["normal", "slow2", "slow5", "off"], seen
    assert sum(1 for _, msg in a.log if msg.startswith("UAV 영상 상태")) == 4
    assert a.mode not in (Mode.SAFE_STOP,)                                      # 영상이 끊겨도 주행·안전은 영향이 없다(UAV 지연은 정지 사유가 아니다)


# ====================================================================== E6 오타격 방지
def test_other_teams_target_in_the_line_of_fire_inhibits_shooting():
    m, a, r = _solo()
    a.cmd = Command(target="RADAR_FACILITY")
    a._scan, a._dth = None, 0.0
    ours = Detection("RADAR_FACILITY", 0.9, (300.0, 200.0, 340.0, 280.0), 1.0)   # 높이 80 px → ≈ 1.1 m
    near = Detection("GUARD_POST", 0.9, (305.0, 160.0, 335.0, 320.0), 1.0)        # 같은 방향, 더 가깝다(≈ 0.56 m)
    far = Detection("COMM_FACILITY", 0.9, (305.0, 220.0, 335.0, 260.0), 1.0)      # 같은 방향이지만 훨씬 멀다(≈ 2.2 m)
    aside = Detection("GUARD_POST", 0.9, (580.0, 160.0, 620.0, 320.0), 1.0)       # 사선 밖
    same = Detection("RADAR_FACILITY", 0.9, (310.0, 150.0, 330.0, 310.0), 1.0)    # 같은 종류(우리 표적의 다른 면) — 오타격 아님
    now = 5.0
    a._dets = [ours, near]
    assert a._wrong_target_in_line(ours, now, 0.0) and a.stats["wrong_target_inhibit"] == 1
    a._dets = [ours, far]
    assert not a._wrong_target_in_line(ours, now, 0.0)
    a._dets = [ours, aside]
    assert not a._wrong_target_in_line(ours, now, 0.0)
    a._dets = [ours, same]
    assert not a._wrong_target_in_line(ours, now, 0.0)
    a.cfg.event.wrong_target_guard = False
    a._dets = [ours, near]
    assert not a._wrong_target_in_line(ours, now, 0.0)
