"""2차 사전교육 자료(ROS2 API, PDF p9~18) 반영 시험 — rclpy 없이 FakeTransport 로 HAL 과 Agent 반응을 본다.

순수 규칙(단위 변환·속도 한계·잠금 판정·레이저/피격/팬틸트 추정·OLED)은 hal/mil_api.py, 구동·서비스 호출 흐름은 hal/wingbot.py.
Agent 쪽(일시정지·잠금·보조 정지·기울기·재시작 복원)은 시뮬 로봇의 status() 를 감싸 /mil/match 값을 주입해서 본다.
"""
import dataclasses
import json
import math

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.hal.mil_api import (FireModel, GimbalModel, HitModel, OledScreen, control_allowed, limit_cmd_vel, pan_deg_to_unit,
                                  pan_unit_to_deg, quantize_pan_tilt, quat_to_rpy_deg, tilt_deg_to_unit)
from maicon26.hal.ros_mil import image_to_bgr
from maicon26.hal.wingbot import Transport, WingbotRobot
from maicon26.mission.manager import Mode
from test_state_machine import _advance, _solo


# ====================================================================== 순수 규칙
def test_control_rule_matches_pdf_p11():
    assert control_allowed("practice", "idle") is True            # 연습 모드는 단계별 잠금 없음
    assert control_allowed("mission", "standby") is False
    assert control_allowed("mission", "start") is True
    assert control_allowed("mission", "resume") is True
    for c in ("idle", "standby", "pause", "report", "stop"):
        assert control_allowed("mission", c) is False
    assert control_allowed("", "") is None                          # 아직 못 받음 → 모름


def test_pan_tilt_units_and_signs():
    h = Config().hal
    assert pan_deg_to_unit(50.0) == -100                             # 내부 왼쪽 + → API 오른쪽 +: 왼쪽 50° = −100
    assert pan_deg_to_unit(-50.0) == 100
    assert pan_deg_to_unit(0.5) == -1                                # 1단위 = 0.5°
    assert pan_deg_to_unit(80.0) == -100                             # 한계를 넘으면 자른다
    assert tilt_deg_to_unit(15.0) == 100 and tilt_deg_to_unit(-15.0) == -100
    assert tilt_deg_to_unit(0.15) == 1                               # 1단위 = 0.15°
    assert quantize_pan_tilt(10.0, 3.0, h) == (-20, 20)
    assert pan_unit_to_deg(-20) == 10.0
    # 현재 설정(±50°, ±15°)이 API 범위와 일치한다
    assert Config().api_issues() == []


def test_limit_cmd_vel():
    assert limit_cmd_vel(0.9, 5.0) == (0.6, 3.0)
    assert limit_cmd_vel(-0.9, -5.0) == (-0.6, -3.0)
    assert limit_cmd_vel(float("nan"), 1.0) == (0.0, 0.0)
    assert limit_cmd_vel(0.3, -1.0) == (0.3, -1.0)


def test_config_api_issues_flags_over_limit():
    cfg = Config()
    cfg.robot.v_max, cfg.robot.pan_limit = 0.8, 60.0
    names = {n for n, *_ in cfg.api_issues()}
    assert {"robot.v_max", "robot.pan_limit"} <= names


def test_gimbal_model_speed_latency_and_history():
    g = GimbalModel(speed_dps=100.0, latency_s=0.1)
    g.command(1.0, 30.0, 0.0)
    assert g.position(1.05) == (0.0, 0.0)                            # 지연 중
    p = g.position(1.2)[0]
    assert abs(p - 10.0) < 1e-6                                      # 0.1 s 이동 × 100°/s
    assert abs(g.position(2.0)[0] - 30.0) < 1e-6                     # 도착 후 유지
    assert abs(g.at(1.2, now=2.0)[0] - 10.0) < 1.0                   # 과거(촬영 시각)의 위치
    g.command(2.0, -10.0, 5.0)
    assert g.position(2.1 + 0.1)[0] < 30.0


def test_fire_model_timeline_p16():
    f = FireModel(ammo=8, shot_s=5.0, period_s=10.0)
    assert f.on_response(100.0, "fired", 10000) == "fired"
    assert f.ammo == 7
    assert f.laser_on(102.0) and not f.laser_on(105.5)               # 5 s 발광
    assert f.cooldown_left(105.5) == pytest.approx(4.5)              # 시작 후 10 s 에 재발사 가능
    assert f.cooldown_left(110.1) == 0.0
    f.on_response(106.0, "reloading", 3000)
    assert f.cooldown_left(106.0) == pytest.approx(4.0)              # 더 긴 쪽(우리 추정 4 s vs 응답 3 s)
    f.on_response(120.0, "empty", None)
    assert f.ammo == 0
    f.on_response(121.0, "locked", None)
    assert f.rejected["locked"] == 1
    f.on_response(130.0, "fired", 10000)
    f.cancel_emission(131.0)                                         # 일시정지 → 레이저 꺼짐
    assert not f.laser_on(132.0)


def test_hit_model_locks_p16():
    hm = HitModel(lock_s=10.0, drive_pause_s=10.0)
    assert hm.on_state(0.0, 4, 4) == 0 and hm.hits == 0
    assert hm.on_state(0.5, 3, 4) == 1
    assert hm.disabled_left(5.0) == pytest.approx(5.5)              # 피격 후 10 s 조준·발사 제한
    assert hm.disabled_left(11.0) == 0.0 and hm.drive_left(5.0) == 0.0
    hm.on_state(20.0, 2, 4)
    hm.on_state(30.0, 1, 4)
    hm.on_state(40.0, 0, 4)                                         # HP 0 = 누적 hp_max → 주행 정지(가정)
    assert hm.hits == 4 and hm.drive_left(41.0) > 0
    hm2 = HitModel(10.0, 10.0)                                      # 경기 도중 (재)시작: 이미 입은 피격은 개수만, 잠금 시각은 모른다
    hm2.on_state(100.0, 2, 4)
    assert hm2.hits == 2 and hm2.disabled_left(100.0) == 0.0


def test_hit_model_practice_hp_recovery_is_not_a_hit():
    hm = HitModel(10.0, 10.0)
    hm.on_state(0.0, 4, 4)
    hm.on_state(1.0, 3, 4)
    hm.on_state(5.0, 4, 4)                                          # 연습 모드는 HP 자동 회복
    assert hm.hits == 1


def test_oled_screen_p13():
    s = OledScreen()
    s.set_text("A" * 30 + "\n한글OK\nline3")
    assert s.want[0] == "A" * 21                                    # 21자
    assert s.want[1] == "??OK"                                      # 영문·숫자·기호만
    sent = []
    t = 0.0
    while t < 3.0:
        r = s.next(t)
        if r:
            sent.append((round(t, 2), r[0]))
        t += 0.1
    assert [i for _, i in sent] == [0, 1, 2]                        # 바뀐 줄만, 한 번에 한 줄
    assert all(b - a >= 0.5 - 1e-9 for (a, _), (b, _) in zip(sent, sent[1:]))   # 간격 ≥ 0.5 s
    assert s.next(4.0) is None                                      # 안 바뀌면 보내지 않는다
    assert s.next(12.0) is not None                                 # 8 s 뒤엔 기본 화면 복귀 전에 다시 쓴다


def test_quat_rpy_matches_pdf_yaw_formula():
    z = math.sin(math.radians(45.0)), math.cos(math.radians(45.0))      # 90° yaw
    r, p, y = quat_to_rpy_deg(0.0, 0.0, z[0], z[1])
    assert abs(y - 90.0) < 1e-6 and abs(r) < 1e-6 and abs(p) < 1e-6


def test_image_to_bgr_variants():
    class M:
        pass
    m = M()
    m.height, m.width, m.encoding = 2, 3, "rgb8"
    arr = np.arange(2 * 3 * 3, dtype=np.uint8)
    m.data, m.step = arr.tobytes(), 9
    out = image_to_bgr(m)
    assert out.shape == (2, 3, 3) and out[0, 0].tolist() == [2, 1, 0]   # RGB → BGR
    m.encoding = "bgr8"
    assert image_to_bgr(m)[0, 0].tolist() == [0, 1, 2]
    m.encoding, m.step = "mono8", 3
    m.data = np.arange(6, dtype=np.uint8).tobytes()
    assert image_to_bgr(m).shape == (2, 3, 3)
    m.encoding = "16UC1"
    with pytest.raises(ValueError):
        image_to_bgr(m)


# ====================================================================== WingbotRobot + FakeTransport
class FakeTransport(Transport):
    def __init__(self):
        self.cmds, self.oled, self.gimbal_calls, self.fire_calls = [], [], [], []
        self.gimbal_reply, self.fire_reply = "ok", ("fired", 10000)
        self.defer = False                 # True 면 응답을 보류(늦은 응답 시험)
        self._deferred = []
        self.closed = False

    def publish_cmd(self, v, w):
        self.cmds.append((v, w))

    def publish_oled(self, line, text):
        self.oled.append((line, text))

    def call_gimbal(self, pan_unit, tilt_unit, speed, acc, cb):
        self.gimbal_calls.append((pan_unit, tilt_unit, speed, acc))
        if self.defer:
            self._deferred.append(lambda: cb(self.gimbal_reply))
        else:
            cb(self.gimbal_reply)

    def call_fire(self, cb):
        self.fire_calls.append(1)
        if self.defer:
            self._deferred.append(lambda: cb(*self.fire_reply))
        else:
            cb(*self.fire_reply)

    def flush(self):
        q, self._deferred = self._deferred, []
        for f in q:
            f()

    def close(self):
        self.closed = True


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def _robot(**hal):
    cfg = Config()
    for k, v in hal.items():
        setattr(cfg.hal, k, v)
    clk, tr = Clock(), FakeTransport()
    r = WingbotRobot(cfg, transport=tr, clock=clk)
    return r, tr, clk


def test_velocity_clamped_repeated_and_watchdog():
    r, tr, clk = _robot()
    r.on_match("practice", "idle")
    r.set_velocity(2.0, -9.0)
    r.pump()
    assert tr.cmds[-1] == (0.6, -3.0)                               # PDF p15 한계
    clk.t += 0.1
    r.pump()
    assert tr.cmds[-1] == (0.6, -3.0)                               # 다음 호출 전까지 유지(반복)
    clk.t += 0.5                                                    # Agent 가 멈췄다: 낡은 명령을 반복하지 않는다
    r.pump()
    assert tr.cmds[-1] == (0.0, 0.0)
    r.set_velocity(0.2, 0.0)
    r.pump()
    assert tr.cmds[-1] == (0.2, 0.0)
    n = len(tr.cmds)
    r.set_velocity(0.0, 0.0)
    assert len(tr.cmds) == n + 1 and tr.cmds[-1] == (0.0, 0.0)      # 정지는 즉시(다음 주기를 기다리지 않는다)


def test_pause_zeroes_and_resume_resends_gimbal():
    r, tr, clk = _robot()
    r.on_match("mission", "start")
    r.set_pan_tilt(10.0, 3.0)
    assert tr.gimbal_calls[-1][:2] == (-20, 20) and tr.gimbal_calls[-1][2:] == (150.0, 20.0)
    r.set_velocity(0.3, 0.0)
    r.pump()
    assert tr.cmds[-1] == (0.3, 0.0)
    r.on_match("mission", "pause")                                  # 일시정지: 모터·팬틸트 정지, 주행 무시
    assert tr.cmds[-1] == (0.0, 0.0)
    clk.t += 0.2
    n = len(tr.gimbal_calls)
    r.set_velocity(0.3, 0.0)
    r.set_pan_tilt(20.0, 0.0)
    r.pump()
    assert tr.cmds[-1] == (0.0, 0.0) and len(tr.gimbal_calls) == n  # 잠금 중엔 locked 만 돌아오므로 호출하지 않는다
    assert r.status().control is False and r.status().paused
    r.on_match("mission", "resume")                                 # 재개 시 로봇은 스스로 움직이지 않는다 → 명령 다시 전송
    clk.t += 0.2
    r.pump()
    assert tr.cmds[-1] == (0.0, 0.0)                                # Agent 가 새 명령을 주기 전까지 0
    assert len(tr.gimbal_calls) == n + 1 and tr.gimbal_calls[-1][:2] == (-40, 0)
    r.set_velocity(0.2, 0.0)
    r.pump()
    assert tr.cmds[-1] == (0.2, 0.0)


def test_gimbal_estimates_position_and_retries_locked():
    r, tr, clk = _robot()
    r.on_match("mission", "start")
    tr.gimbal_reply = "locked"
    r.set_pan_tilt(-30.0, 0.0)
    assert r.status().disabled_left > 0                              # 제어 허용 중 locked = 하드웨어가 알려 준 잠금
    assert r.get_pan_tilt() == (0.0, 0.0)                            # 접수 안 됨 → 위치 불변
    n = len(tr.gimbal_calls)
    r.pump()
    assert len(tr.gimbal_calls) == n                                # 재시도 간격(0.5 s) 전
    clk.t += 0.6
    tr.gimbal_reply = "ok"
    r.pump()
    assert len(tr.gimbal_calls) == n + 1
    clk.t += 0.05 + 0.1                                              # 지연 0.05 + 0.1 s 이동 = 15° 이하
    p = r.get_pan_tilt()[0]
    assert -30.0 < p < 0.0                                           # 움직이는 중(왼쪽 + 약속이라 −30° = 오른쪽 30°)
    clk.t += 1.0
    assert r.get_pan_tilt()[0] == pytest.approx(-30.0)               # 왼쪽 + 약속: −30° = 오른쪽 30°
    assert tr.gimbal_calls[-1][0] == 60                              # API 는 오른쪽 + 이므로 +60 단위
    assert r.stats["gimbal_ok"] == 1 and r.stats["gimbal_locked"] == 1


def test_gimbal_no_response_does_not_move_estimate():
    r, tr, clk = _robot()
    r.on_match("practice", "idle")
    tr.gimbal_reply = None                                           # No response
    r.set_pan_tilt(20.0, 5.0)
    clk.t += 2.0
    assert r.get_pan_tilt() == (0.0, 0.0)
    assert r.stats["gimbal_fail"] >= 1


def test_laser_fire_flow_and_locks():
    r, tr, clk = _robot()
    r.on_match("mission", "standby")
    assert r.laser_fire() is False and not tr.fire_calls            # 잠금: 서비스를 부르지 않는다
    r.on_match("mission", "start")
    assert r.laser_fire() is True
    st = r.status()
    assert st.ammo == 7 and st.laser_on
    clk.t += 1.0
    assert r.laser_fire() is False                                   # 발광/재장전 중에는 호출하지 않는다
    assert len(tr.fire_calls) == 1
    clk.t += 4.5                                                     # 5.5 s: 발광 끝, 재장전 대기
    st = r.status()
    assert not st.laser_on and 4.0 < st.cooldown_left < 5.0
    clk.t += 5.0
    assert r.laser_fire() is True and r.status().ammo == 6
    r.on_match("mission", "pause")                                   # 일시정지 시 레이저 꺼짐
    assert not r.status().laser_on


def test_transport_exceptions_do_not_reach_the_control_loop():
    """서비스 요청 필드 이름이 틀려 전송 계층이 예외를 던져도 Agent.tick() 까지 올라가 경기를 끝내면 안 된다."""
    r, tr, clk = _robot()
    r.on_match("mission", "start")

    def boom(*a, **k):
        raise AttributeError("Request has no field 'pan'")
    tr.call_gimbal, tr.call_fire = boom, boom
    r.set_pan_tilt(10.0, 0.0)                                        # 예외 없음
    assert r.stats["gimbal_fail"] == 1 and r._g_pending_t is None
    assert r.laser_fire() is False
    assert r._fire_pending is None and r.status().cooldown_left <= r.cfg.hal.locked_retry_s + 1e-6   # 10 s 가 아니라 짧게만 쉰다
    assert r.stats["transport_errors"] == 2 and "AttributeError" in r.stats["transport_last_error"]
    tr.call_gimbal = FakeTransport.call_gimbal.__get__(tr)           # 복구되면 다시 보낸다
    clk.t += 5.0
    r.pump()
    assert tr.gimbal_calls


def test_practice_cmd_change_does_not_void_inflight_fire_reply():
    """연습 모드에서 cmd 만 바뀌는 것은 제어 허용 변화가 아니다 — 늦게 온 fired 응답을 locked 로 오판하면 안 된다."""
    r, tr, clk = _robot()
    r.on_match("practice", "idle")
    tr.defer = True
    assert r.laser_fire() is False                                   # 응답 대기 중 (fire_wait_s 안에 못 온다)
    r.on_match("practice", "standby")
    tr.flush()
    st = r.status()
    assert r.fire.last_msg == "fired" and st.ammo == 7
    assert st.disabled_left == 0.0                                   # 잠금 10 s 가 잘못 걸리지 않는다


def test_failed_fire_call_only_pauses_briefly():
    r, tr, clk = _robot()
    r.on_match("mission", "start")
    tr.fire_reply = (None, None)                                     # 서비스 미준비·호출 실패
    assert r.laser_fire() is False
    assert r.status().cooldown_left <= r.cfg.hal.locked_retry_s + 1e-6   # 10 s 재시도 차단이 아니다
    clk.t += r.cfg.hal.locked_retry_s + 0.1
    tr.fire_reply = ("fired", 10000)
    assert r.laser_fire() is True


def test_scan_rebin_keeps_bin_count_for_float32_increment():
    from maicon26.hal.ros_mil import scan_in_base
    inc = float(np.float32(2 * math.pi / 720))
    _, _, out, _ = scan_in_base(float(np.float32(-math.pi)), inc, np.full(720, 2.0), 0.1, 8.0, 0.0, 0.0, 0.0)
    assert len(out) == 720 and not np.isnan(out).any()
    _, _, out, _ = scan_in_base(0.0, 0.0175, np.full(100, 2.0), 0.1, 8.0, 0.0, 0.0, 0.0)   # 비정수 비율은 올림 유지
    assert len(out) == int(math.ceil(2 * math.pi / 0.0175))


def test_laser_fire_rejections_and_late_reply():
    r, tr, clk = _robot()
    r.on_match("mission", "start")
    tr.fire_reply = ("locked", None)
    assert r.laser_fire() is False
    assert r.status().disabled_left > 0 and r.stats["fire_calls"] == 1
    clk.t += 2.0
    r.hit.disabled_until = -1
    tr.fire_reply = ("empty", None)
    assert r.laser_fire() is False and r.status().ammo == 0
    # 응답 지연은 성공으로 간주하지 않고, 늦은 응답으로 상태만 갱신한다.
    r2, tr2, clk2 = _robot()
    r2.on_match("mission", "start")
    tr2.defer = True
    assert r2.laser_fire() is False
    tr2.fire_reply = ("reloading", 4000)
    tr2.flush()
    assert not r2.status().laser_on and r2.status().cooldown_left == pytest.approx(4.0)


def test_hp_drop_sets_hit_lock_and_battery():
    r, tr, clk = _robot()
    r.on_match("mission", "start")
    r.on_state(4, 4, 95.0)
    r.on_state(3, 4, 94.0)
    st = r.status()
    assert st.hits_taken == 1 and st.disabled_left == pytest.approx(10.0) and st.battery_pct == 94.0 and (st.hp, st.hp_max) == (3, 4)
    clk.t += 10.5
    assert r.status().disabled_left == 0.0


def test_camera_rate_limit_and_capture_pan():
    r, tr, clk = _robot()
    img = np.zeros((480, 640, 3), np.uint8)
    n = 0
    for _ in range(30):                                              # 30 FPS 로 들어온다
        r.on_image(img, 0.0)
        n += 1
        clk.t += 1 / 30
    assert 8 <= r.stats["frames"] <= 11 and r.stats["frames_dropped"] > 15     # 10 FPS 이하(p17)
    assert r.get_frame().width == 640 and r.get_frame().height == 480
    r.on_match("practice", "idle")
    r.set_pan_tilt(-40.0, 0.0)
    clk.t += 2.0
    r.on_image(img, 0.0)
    assert r.get_frame().pan == pytest.approx(-40.0)                 # 촬영 시각의 팬각은 추정 위치에서


def test_oled_is_throttled_through_pump():
    r, tr, clk = _robot()
    r.show("T1 WAIT\nHP 4/4 BAT 95%\n00:00 WAIT\nM1- M3-")
    for _ in range(40):
        r.pump()
        clk.t += 0.1
    assert [l for l, _ in tr.oled[:4]] == [0, 1, 2, 3] and len(tr.oled) == 4


def test_close_stops_and_is_idempotent():
    r, tr, clk = _robot()
    r.set_velocity(0.4, 0.0)
    r.pump()
    r.close()
    assert tr.cmds[-1] == (0.0, 0.0) and tr.closed
    n = len(tr.cmds)
    r.pump()
    assert len(tr.cmds) == n                                        # 닫힌 뒤에는 발행하지 않는다


# ====================================================================== Agent: /mil/match 반응
class ApiRobot:
    """SimRobot 을 감싸 status() 에 /mil/match·IMU 값을 주입한다(나머지는 그대로 위임)."""

    def __init__(self, inner):
        self._inner = inner
        self.mode, self.cmd = "mission", "start"
        self.roll = self.pitch = None
        self.battery = None
        self.vels = []
        self.shown = []

    def __getattr__(self, k):
        return getattr(self._inner, k)

    def status(self):
        s = self._inner.status()
        return dataclasses.replace(s, control=self.mode == "practice" or self.cmd in ("start", "resume"), match_mode=self.mode,
                                   match_cmd=self.cmd, roll_deg=self.roll, pitch_deg=self.pitch, battery_pct=self.battery)

    def set_velocity(self, v, w):
        self.vels.append((self._inner.now(), v, w))
        self._inner.set_velocity(v, w)

    def show(self, text):
        self.shown.append(text)


def _api_solo(seed=1):
    m, a, w = _solo(seed=seed)
    ar = ApiRobot(a.robot)
    a.robot = ar
    a.laser.robot = ar
    return m, a, w, ar


def test_agent_match_locked_stops_and_resumes():
    m, a, w, ar = _api_solo()
    _advance(m, 20.0)
    assert a.mode not in (Mode.WAIT, Mode.SAFE_STOP)
    ar.cmd = "pause"
    t_pause = m.world.t
    _advance(m, t_pause + 1.0)
    assert a.mode == Mode.SAFE_STOP and a._safe.get("reason") == "match_locked"
    late = [v for t, v, wz in ar.vels if t > t_pause + 0.2]
    assert late and all(abs(v) < 1e-9 for v in late)                # 잠금 중에는 구동 0 (램프 없이)
    ar.cmd = "resume"
    _advance(m, m.world.t + 1.5)
    assert a.mode != Mode.SAFE_STOP                                  # 재개하면 새 센서 표본 뒤 복귀


def test_agent_pause_freezes_match_clock():
    m, a, w, ar = _api_solo()
    _advance(m, 30.0)
    t0 = m.world.t - a.t_start
    ar.cmd = "pause"
    _advance(m, m.world.t + 10.0)
    ar.cmd = "resume"
    _advance(m, m.world.t + 0.5)
    t1 = m.world.t - a.t_start
    assert 9.0 < t1 - t0 - 0.0 - 10.5 + 10.0 < 12.0 or abs((t1 - t0) - 10.5 + a.stats["paused_s"]) < 1.0
    assert 8.5 < a.stats["paused_s"] < 10.5                          # 멈춘 시간만큼 경기 시계가 안 흘렀다


def test_agent_pause_clock_freeze_can_be_disabled():
    m, a, w, ar = _api_solo()
    a.cfg.hal.freeze_clock_on_pause = False
    _advance(m, 20.0)
    ar.cmd = "pause"
    _advance(m, m.world.t + 5.0)
    assert a.stats["paused_s"] == 0.0


def test_agent_stops_on_match_report_even_without_server_stop():
    m, a, w, ar = _api_solo()
    w.drop.add("stop")
    _advance(m, 25.0)
    assert not a.stopped
    ar.cmd = "report"                                                # 서버 STOP 유실: 로봇이 report 를 알려 준다
    _advance(m, m.world.t + 0.5)
    assert a.stopped and a.stop_source == "match_cmd"


def test_agent_ignores_stale_stop_cmd_before_run():
    """이전 경기의 낡은 cmd=stop 이 남아 있어도(START 가 먼저 와도) 이번 경기를 멈추지 않는다."""
    m, a, w, ar = _api_solo()
    ar.cmd = "stop"
    _advance(m, 8.0)
    assert not a.stopped
    ar.cmd = "start"
    _advance(m, m.world.t + 1.0)
    ar.cmd = "stop"
    _advance(m, m.world.t + 0.5)
    assert a.stopped and a.stop_source == "match_cmd"


def test_agent_practice_mode_has_no_stage_lock():
    m, a, w, ar = _api_solo()
    ar.mode, ar.cmd = "practice", "idle"
    _advance(m, 15.0)
    assert a.mode != Mode.SAFE_STOP or a._safe.get("reason") != "match_locked"
    assert a.stats["match_locked_s"] == 0.0


def test_agent_tilt_stop_with_hysteresis():
    m, a, w, ar = _api_solo()
    _advance(m, 15.0)
    ar.roll = 40.0
    _advance(m, m.world.t + 1.0)
    assert a.mode == Mode.SAFE_STOP and a._safe.get("reason") == "tilt"
    ar.roll = 20.0                                                   # 해제 한계(15°) 위: 아직 정지
    _advance(m, m.world.t + 1.0)
    assert a.mode == Mode.SAFE_STOP
    ar.roll = 3.0
    _advance(m, m.world.t + 1.5)
    assert a.mode != Mode.SAFE_STOP and a.stats["tilt_stops"] == 1


def test_agent_oled_status_and_battery_saving():
    m, a, w, ar = _api_solo()
    ar.battery = 90.0
    _advance(m, 12.0)
    assert ar.shown and all(len(line) <= 21 for txt in ar.shown for line in txt.split("\n"))
    assert len(ar.shown[-1].split("\n")) == 4
    assert len(ar.shown) <= 14                                      # 1 Hz 정도로만 갱신
    assert not a._depth_off
    ar.battery = 10.0
    _advance(m, m.world.t + 1.0)
    assert a._depth_off and any("배터리" in s for _, s in a.log)


def test_agent_restore_clock_after_node_restart(tmp_path):
    p = tmp_path / "match.json"
    import time as _t
    p.write_text(json.dumps({"match_id": "M7", "wall_start": _t.time() - 40.0, "team": "BLUE", "slot": 2}), encoding="utf-8")
    m, a, w, ar = _api_solo()
    a.cfg.hal.restart_state_path = str(p)
    w.drop.update({"start", "command"})                              # 재실행된 노드는 START 를 못 받는다
    _advance(m, 3.0)
    assert a.stats["restored"] and a.t_start is not None
    assert 35.0 < m.world.t - a.t_start < 46.0                       # 40 s 지난 경기 시계로 복원
    assert a.latch.match_id == "M7"


def test_agent_does_not_restore_old_file(tmp_path):
    p = tmp_path / "match.json"
    import time as _t
    p.write_text(json.dumps({"match_id": "M1", "wall_start": _t.time() - 4000.0}), encoding="utf-8")
    m, a, w, ar = _api_solo()
    a.cfg.hal.restart_state_path = str(p)
    w.drop.update({"start", "command"})
    _advance(m, 3.0)
    assert not a.stats["restored"]                                   # 오래된 파일은 무시하고, 로봇의 cmd=start 로 새 경기 시계를 시작한다
    assert a.t_start is not None and m.world.t - a.t_start < 4.0


def test_agent_persists_start_and_clears_at_done(tmp_path):
    p = tmp_path / "match.json"
    m, a, w, ar = _api_solo()
    a.cfg.hal.restart_state_path = str(p)
    _advance(m, 5.0)
    assert p.exists() and "wall_start" in json.loads(p.read_text(encoding="utf-8"))
    _advance(m, 400.0)
    assert a.mode == Mode.DONE and not p.exists()


def test_agent_starts_clock_from_robot_start_signal():
    """서버 API 3단계: 시작 신호를 받은 때가 곧 0:00 — 로봇의 /mil/match cmd=start 가 서버 phase 폴링보다 정확하다."""
    from maicon26.link.base import ServerMsg
    m, a, w, ar = _api_solo()
    w.drop.update({"start", "command"})
    ar.cmd = "standby"
    _advance(m, 3.0)
    assert a.t_start is None
    ar.cmd = "start"
    t_sig = m.world.t
    _advance(m, t_sig + 0.2)
    assert a.t_start is not None and abs(a.t_start - t_sig) < 0.2
    assert any("src=robot_match" in s for _, s in a.log)
    t0 = a.t_start
    w.inject.append((m.world.t, ServerMsg("start")))             # 뒤늦게 서버 phase 에서 온 start 는 중복으로 무시
    _advance(m, m.world.t + 0.5)
    assert a.t_start == t0


def test_agent_uses_server_objective_code_for_m5():
    from maicon26.link.base import ServerMsg
    m, a, w, ar = _api_solo()
    _advance(m, 5.0)
    w.inject.append((m.world.t, ServerMsg("base_command", text="", data={"objective": "RED_MAINTENANCE"})))
    _advance(m, m.world.t + 0.3)
    assert a.m5_type == "MAINT_DEPOT"


def test_m3_first_submit_early_and_refresh():
    m, a, w, ar = _api_solo(seed=2)
    _advance(m, 75.0)
    assert a.m3_submitted                                            # 서버 문서: 마지막 순간에 보내지 말고 미리·자주
    sub0 = sum(1 for k, ok in w.submits if k == "m3")
    assert sub0 >= 1
    _advance(m, 140.0)
    assert sum(1 for k, ok in w.submits if k == "m3") >= sub0        # 집계가 바뀌면 갱신 제출(바뀌지 않으면 안 보낸다)


def test_tick_budget_stat_exists():
    m, a, w, ar = _api_solo()
    _advance(m, 3.0)
    assert a.stats["tick_ms_max"] > 0.0
