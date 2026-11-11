"""6장: 한 모델·한 스레드로 팬틸트와 OAK 를 번갈아 처리하는 SharedAsyncDetector 의 정책.
대부분은 가짜 시계로 한 스레드의 일정을 직접 굴린다(실제 시간·스레드에 기대지 않는다). 동시 실행 시험만 짧은 실제 스레드를 쓴다."""
import threading
import time

import pytest

from maicon26.config import OakDetCfg
from maicon26.perception.detector import AsyncDetector, Detector, SharedAsyncDetector
from maicon26.types import Detection, Frame


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class TimedInner(Detector):
    """검출 한 번이 가짜 시계로 dur 초 걸린다. OAK 프레임(meta source=oak)은 따로 센다."""

    def __init__(self, clk, dur_pt=0.03, dur_oak=None, fail_oak=False):
        self.clk, self.dur_pt, self.dur_oak = clk, dur_pt, dur_oak if dur_oak is not None else dur_pt
        self.fail_oak = fail_oak
        self.starts = []                                    # (시작 시각, "pantilt"|"oak")

    def detect(self, frame):
        src = frame.meta.get("source", "pantilt")
        self.starts.append((self.clk.t, src))
        if src == "oak":
            if self.fail_oak:
                raise RuntimeError("oak decode failed")
            self.clk.t += self.dur_oak
        else:
            self.clk.t += self.dur_pt
        return [Detection("TANK", 0.9, (1, 1, 2, 2), frame.stamp, None, {})]


def build(cfg, inner, clk):
    det = SharedAsyncDetector(inner, cfg, clock=clk)
    det.close()                                             # 진짜 스레드는 끈다 — 아래 simulate 가 그 한 스레드의 일을 대신 굴린다
    return det


def simulate(cfg, dur_pt, dur_oak=None, seconds=10.0, oak=True, strike=(), fail_oak=False, pt_period=0.1, oak_period=0.1):
    clk = FakeClock()
    inner = TimedInner(clk, dur_pt, dur_oak, fail_oak)
    det = build(cfg, inner, clk)
    free_at, next_pt, next_oak, step, i = 0.0, 0.0, 0.0, 0.002, 0
    t = 0.0
    while t < seconds:
        clk.t = t
        det.set_strike(any(a <= t < b for a, b in strike))
        if t >= next_pt - 1e-9:
            det.detect_stamped(Frame(None, t, 0.0, 0.0, 640, 480))
            next_pt += pt_period
        if oak and t >= next_oak - 1e-9:
            det.submit_oak(Frame(None, 1000.0 + t, 0.0, 0.0, 1280, 720, meta={"source": "oak"}))
            next_oak += oak_period
        if t >= free_at:                                    # 스레드가 놀고 있다 — _run 이 하는 일과 같은 순서
            try:
                fr = det._q.get_nowait()
            except Exception:
                fr = None
            if fr is not None:
                det._process(fr)
            else:
                det._maybe_run_oak()
            free_at = max(clk.t, t)
        i += 1
        t = i * step
    return det, inner


def cfg(**kw):
    c = OakDetCfg(enable=True)
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def test_slow_detector_keeps_pantilt_throughput_and_oak_starves():
    off, _ = simulate(cfg(), 0.080, oak=False)
    on, _ = simulate(cfg(), 0.080, oak=True)
    assert off.stats["pantilt_runs"] >= 80
    assert on.stats["pantilt_runs"] >= 0.9 * off.stats["pantilt_runs"]       # 팬틸트 처리량은 90% 이상 지킨다
    assert on.stats["oak_runs"] <= 1                                          # 틈이 없으면 OAK 는 굶는다(첫 한 번은 추정치가 없어 돌 수 있다)
    assert on.stats["oak_skipped_no_gap"] > 0


def test_fast_detector_gives_oak_its_share_without_exceeding_max_hz():
    on, inner = simulate(cfg(max_hz=3.0), 0.030, seconds=10.0)
    assert on.stats["oak_runs"] > 10                                          # 굶지 않는다
    assert on.stats["oak_runs"] <= 3.0 * 10.0 + 1                             # oak_det.max_hz 를 넘지 않는다
    off, _ = simulate(cfg(), 0.030, oak=False, seconds=10.0)
    assert on.stats["pantilt_runs"] >= 0.9 * off.stats["pantilt_runs"]


def test_oak_rate_follows_max_hz():
    slow, _ = simulate(cfg(max_hz=1.0), 0.020, seconds=10.0)
    fast, _ = simulate(cfg(max_hz=5.0), 0.020, seconds=10.0)
    assert slow.stats["oak_runs"] <= 11 < fast.stats["oak_runs"] <= 51


def test_pantilt_frame_is_never_made_to_wait_for_an_oak_run():
    """OAK 가 돌고 있는 동안 도착한 팬틸트 프레임의 대기는 (OAK 한 번의 길이) 이내다 — 틈 검사가 그 이상을 막는다."""
    on, inner = simulate(cfg(max_hz=5.0), 0.030, dur_oak=0.030, seconds=10.0)
    starts = [t for t, s in inner.starts if s == "pantilt"]
    gaps = [b - a for a, b in zip(starts, starts[1:])]
    assert max(gaps) <= 0.1 + 0.030 + 0.01                                    # 팬틸트 도착 간격 0.1 s + OAK 한 번 + 시험 단위(2 ms) 여유


def test_no_oak_runs_start_during_strike_and_stale_frames_are_dropped():
    on, inner = simulate(cfg(max_hz=5.0), 0.020, seconds=10.0, strike=[(3.0, 6.0)])
    inside = [t for t, s in inner.starts if s == "oak" and 3.0 <= t < 6.0]
    assert inside == []
    assert on.stats["oak_skipped_strike"] > 0
    outside = [t for t, s in inner.starts if s == "oak"]
    assert any(t < 3.0 for t in outside) and any(t >= 6.0 for t in outside)   # 끝나면 다시 돈다(사격 중 쌓인 프레임을 몰아서 돌리지 않는다)
    after = sorted(t for t in outside if t >= 6.0)
    assert after and after[0] - 6.0 < 0.5


def test_pause_in_strike_off_keeps_oak_running_during_strike():
    on, inner = simulate(cfg(max_hz=5.0, pause_in_strike=False), 0.020, seconds=6.0, strike=[(2.0, 4.0)])
    assert any(2.0 <= t < 4.0 for t, s in inner.starts if s == "oak")


def test_an_oak_failure_is_counted_and_does_not_touch_pantilt_health():
    on, _ = simulate(cfg(max_hz=5.0), 0.020, seconds=3.0, fail_oak=True)
    assert on.oak_error_count > 0 and "oak decode failed" in on.oak_last_error
    assert on.health() == (True, "")                                          # 팬틸트 건강 상태는 그대로
    assert on.stats["oak_runs"] == 0 and on.poll_oak() == ([], -1.0)


def test_oak_result_carries_its_own_frame_stamp():
    on, _ = simulate(cfg(max_hz=5.0), 0.020, seconds=2.0)
    dets, stamp = on.poll_oak()
    assert dets and stamp >= 1000.0                                           # OAK 프레임의 stamp (시험이 1000 + t 로 만들었다)
    pt_dets, pt_stamp = on.detect_stamped(Frame(None, 123.0, 0.0, 0.0, 640, 480))
    assert pt_stamp < 1000.0                                                  # 팬틸트 결과와 섞이지 않는다


def test_same_oak_frame_is_not_submitted_twice():
    clk = FakeClock()
    det = build(cfg(), TimedInner(clk), clk)
    f = Frame(None, 5.0, 0.0, 0.0, 1280, 720, meta={"source": "oak"})
    det.submit_oak(f)
    det._oak_in = None                                                        # 한 번 돌렸다고 치고 슬롯을 비운다
    det.submit_oak(f)
    assert det._oak_in is None


# ---------------------------------------------------------------------- 진짜 스레드: 동시 predict 없음
class CountingInner(Detector):
    def __init__(self):
        self.active, self.max_active, self.calls = 0, 0, 0
        self.lock = threading.Lock()

    def detect(self, frame):
        with self.lock:
            self.active += 1
            self.max_active = max(self.max_active, self.active)
            self.calls += 1
        time.sleep(0.005)
        with self.lock:
            self.active -= 1
        return []


def test_real_thread_never_runs_two_inferences_at_once():
    inner = CountingInner()
    det = SharedAsyncDetector(inner, cfg(max_hz=50.0))
    try:
        t_end = time.monotonic() + 0.8
        i = 0
        while time.monotonic() < t_end:
            i += 1
            det.detect_stamped(Frame(None, float(i), 0.0, 0.0, 640, 480))
            det.submit_oak(Frame(None, 5000.0 + i, 0.0, 0.0, 1280, 720, meta={"source": "oak"}))
            time.sleep(0.004)
    finally:
        det.close()
    assert inner.calls > 20 and inner.max_active == 1


def test_plain_async_detector_is_unchanged_when_oak_is_off():
    """oak_det.enable 이 꺼져 있으면 run_real 은 기존 AsyncDetector 를 쓴다 — 그 클래스는 OAK 슬롯이 없다."""
    inner = CountingInner()
    det = AsyncDetector(inner)
    try:
        assert not hasattr(det, "submit_oak") and not hasattr(det, "poll_oak")
    finally:
        det.close()
