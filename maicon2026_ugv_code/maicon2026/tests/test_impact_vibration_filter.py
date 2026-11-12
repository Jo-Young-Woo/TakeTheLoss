"""IMU 충돌 감지기의 진동 거르기(vib_filter) — Gazebo 에서 바퀴 진동(최대 0.98 g)이 충돌로 오인되어 주행이 반복 정지한 문제.

합성 신호(ROS·로봇 없음): 진동은 부호가 번갈아 바뀌는 수평 가속도, 충돌은 한쪽 방향 펄스(+ 잔향).
"""
import math

from maicon26.config import Config
from maicon26.nav.pose_guard import ImpactDetector
from maicon26.validate import sanitize_config

HZ = 100.0
DT = 1.0 / HZ


def _run(det, accel_xy, t_end, t0=0.0):
    """accel_xy(t) -> (ax, ay) [g] 를 100 Hz 로 먹인다. 반환: 낸 이벤트 목록."""
    out = []
    n = int(round((t_end - t0) * HZ))
    for i in range(n):
        t = t0 + i * DT
        ax, ay = accel_xy(t)
        g = math.sqrt(1.0 + ax * ax + ay * ay) if False else 1.0                # 수직(g)은 그대로 — 충돌은 수평 성분으로 본다
        ev = det.add(t, g, ax, ay, 0.0)
        if ev is not None:
            out.append(ev)
    return out + det.pop()[:0]


def _vib(amp=0.95, f=40.0, start=1.0, dur=1.0):
    def fn(t):
        if start <= t < start + dur:
            return (amp * math.sin(2 * math.pi * f * t), 0.15 * amp * math.sin(2 * math.pi * f * t + 1.0))
        return (0.0, 0.0)
    return fn


def _hit(peak=0.8, width=0.03, at=1.0, front=True):
    """앞에서 부딪히면 몸은 뒤로(−x) 밀리는 가속도를 받는다. 한쪽 펄스 + 약한 반대 방향 잔향."""
    sgn = -1.0 if front else 1.0

    def fn(t):
        if at <= t < at + width:
            return (sgn * peak, 0.0)
        if at + width <= t < at + 3 * width:
            return (-sgn * 0.25 * peak, 0.0)
        return (0.0, 0.0)
    return fn


def test_vibration_without_filter_is_reported_as_collisions_baseline():
    det = ImpactDetector()
    evs = _run(det, _vib(), 3.0)
    assert len(evs) >= 1                                    # 지금(꺼짐) 동작: 진동이 충돌로 보인다 — 문제 재현


def test_vibration_is_rejected_when_filter_is_on():
    det = ImpactDetector(vib_filter=True)
    evs = _run(det, _vib(), 3.0)
    assert evs == []
    assert det.vib_rejected >= 1 and det.vib_last_unipolar is not None and det.vib_last_unipolar < 0.45


def test_long_vibration_stays_rejected_across_sustain_resets():
    det = ImpactDetector(vib_filter=True)
    evs = _run(det, _vib(dur=6.0), 8.0)
    assert evs == [] and det.vib_rejected >= 3


def test_real_collision_is_still_detected_with_bearing_front():
    det = ImpactDetector(vib_filter=True)
    evs = _run(det, _hit(), 2.0)
    assert len(evs) == 1
    ev = evs[0]
    assert abs(ev.t - 1.0) < 0.02                           # 시각은 덩어리가 시작된 때(판정이 늦어도 사건 시각은 그대로)
    assert ev.sector == "front" and ev.severity == "minor"
    assert ev.peak_dg >= 0.7


def test_rear_hit_bearing_and_side_hit_are_kept():
    d = ImpactDetector(vib_filter=True)
    ev = _run(d, _hit(front=False), 2.0)[0]
    assert ev.sector == "rear"
    d2 = ImpactDetector(vib_filter=True)

    def side(t):
        a, _ = _hit()(t)
        return (0.0, -a)                                    # 몸이 −y 로 밀림 → 오른쪽에서 맞음이 아니라 왼쪽에서 맞음(부호 확인)
    ev2 = _run(d2, side, 2.0)[0]
    assert ev2.sector in ("left", "right")


def test_vibration_then_real_collision_in_the_same_run():
    def mix(t):
        a = _vib(start=0.6, dur=1.2)(t)
        b = _hit(at=2.5)(t)
        return (a[0] + b[0], a[1] + b[1])
    det = ImpactDetector(vib_filter=True)
    evs = _run(det, mix, 4.0)
    assert len(evs) == 1 and abs(evs[0].t - 2.5) < 0.03


def test_major_hit_is_not_delayed_by_the_window():
    det = ImpactDetector(vib_filter=True)
    first = None
    for i in range(300):
        t = i * DT
        ax = -1.6 if 1.0 <= t < 1.03 else 0.0
        ev = det.add(t, 1.0, ax, 0.0, 0.0)
        if ev is not None and first is None:
            first = (t, ev)
    assert first is not None and abs(first[0] - 1.0) < 1e-6 and first[1].severity == "major"


def test_gyro_trigger_is_never_filtered():
    det = ImpactDetector(vib_filter=True)
    got = None
    for i in range(300):
        t = i * DT
        gz = 250.0 if 1.0 <= t < 1.05 else 0.0
        ev = det.add(t, 1.0, 0.0, 0.0, gz)
        got = got or ev
    assert got is not None and got.src == "gyro"


def test_without_horizontal_axes_the_filter_cannot_judge_and_behaves_as_before():
    det = ImpactDetector(vib_filter=True)
    got = []
    for i in range(300):
        t = i * DT
        g = 1.0 + (0.9 * math.sin(2 * math.pi * 40 * t) if 1.0 <= t < 2.0 else 0.0)
        ev = det.add(t, g)
        if ev is not None:
            got.append(ev)
    assert len(got) >= 1


def test_slow_imu_with_too_few_samples_is_reported_not_guessed():
    det = ImpactDetector(vib_filter=True, vib_min_samples=4)
    got = []
    dt = 0.2                                                # 5 Hz IMU: 창 안에 표본이 2개뿐
    for i in range(40):
        t = i * dt
        ax = -0.9 if 2.0 <= t < 2.1 else 0.0
        ev = det.add(t, 1.0, ax, 0.0, 0.0)
        if ev is not None:
            got.append(ev)
    assert len(got) == 1


def test_default_is_off_and_matches_legacy_event_timing():
    legacy = ImpactDetector()
    off = ImpactDetector(vib_filter=False)
    a = _run(legacy, _hit(), 2.0)
    b = _run(off, _hit(), 2.0)
    assert [(e.t, round(e.peak_dg, 6), e.bearing) for e in a] == [(e.t, round(e.peak_dg, 6), e.bearing) for e in b]


def test_config_defaults_validation_and_hal_wiring():
    cfg = Config()
    assert cfg.impact.vib_filter is False and cfg.impact.vib_window_s == 0.12
    cfg.impact.vib_min_unipolar = 3.0                       # 범위 밖 → 검증이 기본값으로 되돌린다
    cfg.impact.vib_min_samples = 1
    fixes = sanitize_config(cfg)
    assert cfg.impact.vib_min_unipolar == 0.45 and cfg.impact.vib_min_samples == 4
    assert any("vib_min_unipolar" in str(f) for f in fixes)

    from hw_fakes import make_robot
    c2 = Config()
    c2.impact.enable = True
    c2.impact.vib_filter = True
    c2.impact.vib_window_s = 0.2
    robot, _, clock = make_robot(cfg=c2)
    det = robot._impact_det
    assert det.vib_filter is True and det.vib_window_s == 0.2 and det.vib_min_unipolar == 0.45
    # HAL 경로: 진동은 이벤트가 안 되고, 한쪽 펄스는 이벤트가 된다(IMU 100 Hz)
    for i in range(300):
        clock.t += 0.01
        a = 0.95 * math.sin(2 * math.pi * 40 * clock.t) if 1.0 <= i * 0.01 < 2.0 else 0.0
        robot.on_imu(0.0, 0.0, 0.0, 1.0, a, 0.1 * a, 0.0)
    assert robot.pop_impacts() == []
    for i in range(200):
        clock.t += 0.01
        a = -0.8 if i in (50, 51, 52) else 0.0
        robot.on_imu(0.0, 0.0, 0.0, 1.0, a, 0.0, 0.0)
    evs = robot.pop_impacts()
    assert len(evs) == 1 and evs[0].sector == "front"


def test_seeded_sweep_vibration_mostly_rejected_and_real_hits_kept():
    """무작위 진동(20..80 Hz, 0.5..1.15 g, 200 Hz IMU)·무작위 충돌 펄스. 합성이라 숫자는 참고용 — 거르기가 켜지면 진동 오검출 구간이 크게 줄고 충돌 검출은 유지된다."""
    import random
    rnd = random.Random(5)

    def run(fn, hz, T, filt):
        det = ImpactDetector(vib_filter=filt)
        n = 0
        for i in range(int(T * hz)):
            t = i / hz
            ax, ay = fn(t)
            if det.add(t, 1.0, ax, ay, 0.0):
                n += 1
        return n

    def mkvib():
        f, A, ph = rnd.uniform(20, 80), rnd.uniform(0.5, 1.15), rnd.uniform(0, 6.28)
        return lambda t: ((A * math.sin(2 * math.pi * f * t + ph) if 1.0 <= t < 4.0 else 0.0), 0.2 * A * math.sin(2 * math.pi * f * t + ph + 1) if 1.0 <= t < 4.0 else 0.0)

    def mkhit():
        pk, w, ring, fr, sg = rnd.uniform(0.5, 1.15), rnd.uniform(0.01, 0.08), rnd.uniform(0, 0.5), rnd.uniform(60, 150), rnd.choice([-1, 1])

        def fn(t):
            if 1.0 <= t < 1.0 + w:
                return (sg * pk * math.sin(math.pi * (t - 1.0) / w), 0.0)
            if 1.0 + w <= t < 1.0 + w + 0.1:
                return (-sg * ring * pk * math.sin(2 * math.pi * fr * (t - 1 - w)) * math.exp(-(t - 1 - w) * 30), 0.0)
            return (0.0, 0.0)
        return fn
    N = 100
    vib_on = sum(run(mkvib(), 200, 5.0, True) > 0 for _ in range(N))
    vib_off = sum(run(mkvib(), 200, 5.0, False) > 0 for _ in range(N))
    hit_on = sum(run(mkhit(), 200, 3.0, True) > 0 for _ in range(N))
    assert vib_off >= 90                      # 꺼져 있으면 거의 매번 오검출(문제 재현)
    assert vib_on <= 20                       # 켜면 구간 오검출 20 % 이하
    assert hit_on >= 90                       # 충돌은 90 % 이상 그대로 검출
