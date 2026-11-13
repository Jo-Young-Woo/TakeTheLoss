"""상태 전이표(MAICON2026_상태전이표_경쟁상태반영.html) 구현 검증.

순수 부품은 단위 시험, 상태 전이는 시뮬 + 장애 주입(서버 메시지 순서 교란, LiDAR 정지, 피격 잠금, 제출 실패)으로 본다.
시나리오는 전이표 8장의 '검증 시나리오'와 7장의 경쟁 상태 ID(R01…)를 따른다. (이벤트 요소는 6장 — tests/test_events.py)
"""
import dataclasses
import math
import time

from maicon26.config import Config
from maicon26.link.async_link import AsyncServerLink
from maicon26.link.base import ServerLink, ServerMsg, UavFrame
from maicon26.mission.guard import Locks, MatchLatch, SensorWatch
from maicon26.mission.manager import Mode
from maicon26.mission.submit import Submitter
from maicon26.nav.cmd_cache import CmdCache
from maicon26.nav.stack import NavOut
from maicon26.perception.detector import AsyncDetector, Detector, SimDetector
from maicon26.sim.run import build_match
from maicon26.types import Detection, Frame, RobotStatus


# ====================================================================== 공통 도우미
def _cfg(**mission):
    cfg = Config()
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    for k, v in mission.items():
        setattr(cfg.mission, k, v)
    return cfg


class Wrap(ServerLink):
    """SimServer 를 감싸 메시지 주입·삭제·제출 실패를 흉내 낸다."""

    def __init__(self, inner, match_id=None):
        self.inner, self.world, self.match_id = inner, inner.world, match_id
        self.inject = []                 # [(월드 시각, ServerMsg)]
        self.drop = set()
        self.fail = {}                   # kind → 남은 실패 횟수(math.inf 가능)
        self.submits = []
        self.uav_override = None

    def poll(self):
        out = []
        for m in self.inner.poll():
            if m.kind in self.drop:
                continue
            if self.match_id and m.kind in ("start", "command", "base_command", "stop"):
                m.data = {**m.data, "match_id": self.match_id}
            out.append(m)
        while self.inject and self.inject[0][0] <= self.world.t:
            out.append(self.inject.pop(0)[1])
        return out

    def submit(self, kind, payload):
        n = self.fail.get(kind, 0)
        if n > 0:
            self.fail[kind] = n - 1
            self.submits.append((kind, False))
            return False
        # 주입한 STOP 은 월드를 끝내지 않으므로, SimServer 가 M6 를 '경기 중'이라며 거부하는 것은 시험 장치의 한계다 → 수락 처리
        ok = True if (kind == "m6" and not self.world.ended) else self.inner.submit(kind, payload)
        self.submits.append((kind, ok))
        return ok

    def get_uav(self):
        return self.uav_override if self.uav_override is not None else self.inner.get_uav()

    def identity(self):
        return self.inner.identity()


def _solo(seed=1, cfg=None, match_id=None, **kw):
    m = build_match(seed=seed, cfg=cfg or _cfg(), teams=["B2"], **kw)
    a = m.agents[0]
    w = Wrap(a.server, match_id)
    a.server = w
    a.submitter.server = w
    return m, a, w


def _advance(m, until, hook=None):
    wd = m.world
    while wd.t < until and not wd.finished:
        if hook:
            hook(m)
        for a in m.agents:
            a.tick()
        wd.step(0.05)


# ====================================================================== 단위: 래치 / 감시 / 잠금
def test_match_latch_start_once_stop_terminal_and_ids():
    L = MatchLatch()
    assert L.accept(ServerMsg("start", data={"match_id": "A", "seq": 1}))[0]
    assert L.accept(ServerMsg("start", data={"match_id": "A", "seq": 2})) == (False, "dup-start")
    assert L.accept(ServerMsg("event", data={"match_id": "B"})) == (False, "other-match")
    assert L.accept(ServerMsg("event", data={"seq": 5}))[0]
    assert L.accept(ServerMsg("event", data={"seq": 5})) == (False, "stale-seq")     # 순서 뒤바뀜/중복
    assert L.accept(ServerMsg("stop", data={"match_id": "B"})) == (False, "other-match")   # 다른 경기의 STOP 은 무시
    assert not L.stopped
    assert L.accept(ServerMsg("stop", data={"match_id": "A"}))[0] and L.stopped
    assert L.accept(ServerMsg("start")) == (False, "after-stop")                     # STOP 뒤 START 무시: 취소 불가
    assert L.accept(ServerMsg("base_command", text="x")) == (False, "after-stop")


def test_sensor_watch_uses_arrival_time_not_sensor_clock():
    s = SensorWatch(0.6)
    assert not s.fresh(0.0) and s.age(0.0) == math.inf
    s.on_scan(1000.0, 5.0)                      # 센서 시계가 로봇 시계와 전혀 달라도 된다
    assert s.fresh(5.5) and not s.fresh(5.7)
    s.on_scan(1000.0, 5.8)                      # 같은 stamp 의 재전달은 '새 표본'이 아니다
    assert s.n_new == 1 and not s.fresh(5.8)
    s.on_scan(1000.1, 5.9)
    assert s.n_new == 2 and s.fresh(5.9)


def test_locks_hardware_authority_and_hit_grace():
    L = Locks(grace_s=0.6)
    assert not L.weapon_locked(RobotStatus(), 0.0)
    L.update(RobotStatus(hits_taken=1), 10.0)              # 하드웨어 상태보다 먼저 이벤트/카운트가 오면 유예 동안 잠금
    assert L.weapon_locked(RobotStatus(hits_taken=1), 10.3)
    assert not L.drive_locked(RobotStatus(hits_taken=1), 10.3)       # 첫 3회는 주행을 잠그지 않는다
    assert not L.weapon_locked(RobotStatus(hits_taken=1), 10.7)      # 유예 뒤엔 하드웨어(0)가 권한
    assert L.weapon_locked(RobotStatus(hits_taken=1, disabled_left=4.0), 10.7)
    L.update(RobotStatus(hits_taken=4), 20.0)                         # 4번째 피격부터 주행도
    assert L.drive_locked(RobotStatus(hits_taken=4), 20.2)
    assert L.drive_locked(RobotStatus(hits_taken=4, drive_paused_left=8.0), 25.0)
    assert not L.drive_locked(RobotStatus(hits_taken=4), 25.0)


# ====================================================================== 단위: 제출 추적 (R10)
class _Srv(ServerLink):
    def __init__(self, results):
        self.results, self.calls = list(results), []

    def poll(self):
        return []

    def submit(self, kind, payload):
        self.calls.append((kind, dict(payload)))
        return self.results.pop(0) if self.results else True


def _pump(sub, srv, t0, t1, step=0.1):
    t = t0
    while t <= t1:
        sub.update(t)
        t += step


def test_submitter_rejected_is_not_done_and_retries_until_ack():
    srv = _Srv([False, False, True])
    sub = Submitter(srv, retry_s=1.0, ack_timeout_s=3.0, max_tries=6)
    assert sub.request("m1", {"a": 1}, 0.0)
    assert not sub.request("m1", {"a": 2}, 0.0)             # 진행 중이면 중복 요청 무시
    _pump(sub, srv, 0.0, 0.5)
    assert sub.state("m1") == "retry" and not sub.acked("m1")   # 거부를 '완료'로 기록하지 않는다
    _pump(sub, srv, 0.5, 6.0)
    assert sub.acked("m1") and sub.tries("m1") == 3 and sub.rejections("m1") == 2
    assert [c[1] for c in srv.calls] == [{"a": 1}] * 3


def test_submitter_lost_response_resends_same_token_and_deadline_gives_up():
    class Lost(_Srv):
        def submit_async(self, kind, payload, token):       # 응답이 영영 오지 않는 서버
            self.calls.append((kind, token))

    srv = Lost([])
    sub = Submitter(srv, retry_s=1.0, ack_timeout_s=2.0, max_tries=10)
    sub.request("m6", {"r": [1]}, 0.0, deadline=9.0, max_tries=10 ** 6)
    _pump(sub, srv, 0.0, 12.0)
    tokens = {c[1] for c in srv.calls}
    assert len(srv.calls) >= 3 and len(tokens) == 1          # 재전송은 같은 token(멱등 키)
    assert sub.state("m6") == "gave_up" and sub.rejections("m6") == 0     # 거부가 아니라 '미확인'


def test_submitter_close_for_stop_blocks_new_and_cancels_pending():
    srv = _Srv([])
    sub = Submitter(srv)
    sub.request("m3", {"c": 1}, 5.0)                          # 아직 보내기 전
    sub.close_for_stop(("m6",))
    sub.update(6.0)
    assert sub.state("m3") == "cancelled" and srv.calls == []
    assert not sub.request("m1", {}, 6.0)                     # STOP 뒤 M1~M5 신규 제출 금지
    assert sub.request("m6", {"r": []}, 6.0)
    sub.update(6.0)
    assert sub.state("m6") == "inflight"


# ====================================================================== 단위: Nav2 속도 게이트 (R06)
def test_cmd_cache_drops_old_goal_commands_until_new_goal_accepted():
    c = CmdCache(timeout_s=0.5)
    assert not c.put(0.3, 0.0, 1.0)                           # 목표 수락 전 → 버림
    c.open_gate()
    assert c.put(0.3, 0.1, 1.0) and c.get(1.2) == (0.3, 0.1)
    assert c.get(1.7) is None                                 # 오래되면 None → 호출자가 0 으로 해석
    c.invalidate()                                            # 취소/목표 교체
    assert c.get(1.1) is None
    assert not c.put(0.4, 0.0, 1.3) and c.dropped == 2        # 취소 직후 도착한 이전 목표의 속도는 버린다
    c.open_gate()
    assert c.put(0.1, 0.0, 1.4) and c.get(1.5) == (0.1, 0.0)


# ====================================================================== 단위: 비동기 링크 (R02)
class _SlowInner(ServerLink):
    def __init__(self):
        self.msgs = [ServerMsg("stop")]
        self.submitted = []
        self.uav_calls = 0

    def poll(self):
        out, self.msgs = self.msgs, []
        return out

    def submit(self, kind, payload):
        time.sleep(0.6)                                       # 느린 서버
        self.submitted.append(kind)
        return True

    def get_uav(self):
        self.uav_calls += 1
        if self.uav_calls < 3:
            raise RuntimeError("boom")                        # 예외가 스레드를 죽이면 안 된다
        return UavFrame(stamp=float(self.uav_calls))


def test_async_link_stop_not_blocked_by_slow_submit_and_survives_errors():
    inner = _SlowInner()
    link = AsyncServerLink(inner, poll_hz=100, uav_hz=50)
    try:
        link.submit_async("m6", {"r": []}, "tok")
        t0 = time.monotonic()
        got = []
        while time.monotonic() - t0 < 0.4 and not got:
            got = link.poll()
            time.sleep(0.005)
        assert [m.kind for m in got] == ["stop"] and time.monotonic() - t0 < 0.3   # 제출이 0.6s 걸려도 STOP 은 즉시
        assert link.submit_results() == []                                         # 아직 전송 중
        time.sleep(0.9)
        assert link.submit_results() == [("tok", True)]
        t1 = time.monotonic()
        while link.get_uav() is None and time.monotonic() - t1 < 3.0:              # 예외 두 번(백오프 0.5s) 뒤에도 회복
            time.sleep(0.05)
        assert link.get_uav() is not None and link.last_error is not None
    finally:
        link.close()


# ====================================================================== 단위: 검출-프레임 짝 (R03)
class _Delayed(Detector):
    """항상 'k 프레임 늦은' 결과를 준다(비동기 검출기가 이전 프레임 결과를 주는 상황)."""
    asynchronous = True

    def __init__(self, k=3):
        self.k, self.buf, self.calls = k, [], []

    def detect(self, frame):
        return self.detect_stamped(frame)[0]

    def detect_stamped(self, frame):
        if not self.buf or self.buf[-1][1] != frame.stamp:
            self.buf.append((SimDetector().detect(frame), frame.stamp))
        old = self.buf[-1 - self.k] if len(self.buf) > self.k else ([], -1.0)
        self.calls.append((frame.stamp, old[1]))
        return old


def test_delayed_detections_are_paired_with_their_own_frame_and_consumed_once():
    m, a, _ = _solo(seed=3)
    d = _Delayed(k=3)
    a.detector = d
    coverage_frames=[]
    original_record=a.coverage.record_view
    def recorded(*args, **kwargs):
        coverage_frames.append(args[3])
        return original_record(*args, **kwargs)
    a.coverage.record_view=recorded
    _advance(m, 8.0)
    assert a.stats["det_dropped"] == 0
    assert a.stats["det_consumed"] > 50
    # 소비된 결과는 항상 '현재 프레임보다 과거' 프레임의 것이었고 같은 결과를 두 번 소비하지 않았다
    olds = [o for (cur, o) in d.calls if o >= 0 and o < cur]
    assert olds and a._det_consumed <= max(c for c, _ in d.calls)
    assert a.stats["det_consumed"] == len(set(o for _, o in d.calls if o >= 0))      # 결과 하나당 정확히 한 번
    assert len(coverage_frames)==a.stats['det_consumed'], 'Unprocessed frames cannot establish EMPTY'


def test_stuck_detector_result_not_reapplied():
    class Stuck(Detector):
        asynchronous = True

        def detect(self, frame):
            return []

        def detect_stamped(self, frame):
            return [], 1.0                                    # 같은 (결과, stamp) 를 계속 돌려준다

    m, a, _ = _solo(seed=3)
    a.detector = Stuck()
    _advance(m, 3.0)
    assert a.stats["det_consumed"] <= 1                       # 있었다 해도 한 번만


def test_completed_coverage_result_keeps_its_original_rgb_pixels():
    import dataclasses
    import numpy as np
    m, a, _ = _solo(seed=3)
    a.cfg.explore.exhaustive_scoring_faces = True
    a.detector = _Delayed(k=3)
    originals = {}
    raw_frame = a.robot.get_frame
    def with_image():
        frame = raw_frame()
        if frame is None:
            return None
        pixels = originals.setdefault(frame.stamp, np.full((480,640,3), int(frame.stamp*10)%255, np.uint8))
        return dataclasses.replace(frame, image=pixels)
    a.robot.get_frame = with_image
    seen = []
    def record(*args, **kwargs):
        seen.append(a._det_consumed)
        assert kwargs['image'] is originals[a._det_consumed]
    a.coverage.record_view = record
    _advance(m, 3.)
    assert len(seen) >= 3
    assert len(a._frames) <= 16


def test_aiming_frames_are_classified_but_not_counted_as_blank_face_evidence():
    m, a, _ = _solo(seed=3)
    _advance(m, .3)
    class EmptyResult(Detector):
        def detect(self, frame):
            return []
    a.detector = EmptyResult()
    frames=[]
    a.coverage.record_view=lambda *args, **kwargs: frames.append(args[3])
    stamp=m.world.t+.1
    a.robot.get_frame=lambda: Frame(None,stamp,0.,0.,640,480)
    before=a.stats['det_consumed']
    for mode in (Mode.HUNT, Mode.STRIKE):
        a.mode=mode
        stamp+=.1
        a._ingest_vision(stamp)
    assert a.stats['det_consumed']==before+2 and not frames
    a.mode=Mode.EXPLORE
    stamp+=.1
    a._ingest_vision(stamp)
    assert len(frames)==1


def test_async_detector_runs_each_frame_once_and_reports_stamp():
    calls = []

    class Inner(Detector):
        def detect(self, frame):
            calls.append(frame.stamp)
            time.sleep(0.05)
            return [Detection("DRUM", 0.9, (0, 0, 10, 10), frame.stamp)]

    ad = AsyncDetector(Inner())
    try:
        f1, f2 = Frame(None, 1.0), Frame(None, 2.0)
        assert ad.detect_stamped(f1)[1] == -1.0
        for _ in range(30):                                   # 같은 프레임으로 여러 번 호출해도 한 번만 돈다
            r = ad.detect_stamped(f1)
            time.sleep(0.005)
        assert r[1] == 1.0 and r[0][0].stamp == 1.0 and calls == [1.0]
        ad.detect_stamped(f2)
        time.sleep(0.2)
        assert ad.detect_stamped(f2)[1] == 2.0 and calls == [1.0, 2.0]
    finally:
        ad.close()


# ====================================================================== 시나리오: 정지 우선 / 래치 (R01, R09, R12)
def test_stop_before_slam_ready_halts_immediately():
    """SLAM/LiDAR 가 준비되지 않아도 STOP 은 센서 처리보다 먼저 처리된다."""
    m, a, w = _solo(seed=2)
    a.robot.get_scan = lambda: None                           # LiDAR 없음 → stack 초기화 불가
    w.inject = [(0.2, ServerMsg("stop"))]
    _advance(m, 3.0)
    assert not a.stack.initialized
    assert a.mode == Mode.DONE and a.stop_source == "server"
    assert a.robot.u.cmd_v == 0.0 and a.robot.u.cmd_w == 0.0
    assert ("m6", True) in w.submits                          # 센서 없이도 M6(중간값 순위)가 제출된다
    assert a.m6_result == "acked"


def test_start_while_slam_not_ready_is_safe_stop_then_explore():
    m, a, _ = _solo(seed=2)
    orig = a.robot.get_scan
    a.robot.get_scan = lambda: None
    _advance(m, 1.0)
    assert a.t_start is not None and a.mode == Mode.SAFE_STOP and a._safe["reason"] == "pose_not_ready"
    assert a.robot.u.cmd_v == 0.0
    a.robot.get_scan = orig
    _advance(m, 3.0)
    assert a.mode != Mode.SAFE_STOP and a.stack.initialized


def test_duplicate_start_and_foreign_match_ignored_stop_of_own_match_halts():
    m, a, w = _solo(seed=2, match_id="M-1")
    w.inject = [(5.0, ServerMsg("start", data={"match_id": "M-1"})),          # 중복 START
                (6.0, ServerMsg("stop", data={"match_id": "M-OTHER"})),       # 다른 경기의 STOP
                (8.0, ServerMsg("event", data={"match_id": "M-OTHER", "type": "ugv_hit"}))]
    _advance(m, 7.0)
    t_start = a.t_start
    assert t_start is not None and t_start < 1.0               # 경기 시계가 5.0 으로 재설정되지 않았다
    assert a.stats["msg_rejected"].get("dup-start") == 1 and a.stats["msg_rejected"].get("other-match") == 1
    assert a.mode not in (Mode.HALT, Mode.REPORT, Mode.DONE)
    w.inject = [(7.5, ServerMsg("stop", data={"match_id": "M-1"})),
                (7.6, ServerMsg("start", data={"match_id": "M-1"}))]          # STOP 뒤 START 는 무시
    _advance(m, 9.0)
    assert a.mode == Mode.DONE and a.stop_source == "server" and a.t_start == t_start


def test_local_timeout_halts_when_stop_is_lost():
    cfg = _cfg(t_total=20.0, local_stop_grace_s=1.0)
    m, a, w = _solo(seed=2, cfg=cfg)
    w.drop = {"stop"}                                          # STOP 유실
    _advance(m, 19.5)
    assert a.mode not in (Mode.HALT, Mode.REPORT, Mode.DONE)
    _advance(m, 24.0)
    assert a.stop_source == "local_timeout" and a.mode == Mode.DONE
    assert 20.9 <= a.halt_t - a.t_start <= 21.2
    assert a.robot.u.cmd_v == 0.0


def test_stop_cancels_unsent_m3_and_blocks_new_m1_m5_submits():
    m, a, w = _solo(seed=2)
    w.inject = [(3.0, ServerMsg("stop"))]
    _advance(m, 5.0)
    assert a.mode == Mode.DONE
    kinds = [k for k, _ in w.submits]
    assert "m3" not in kinds                                   # 235 s 전에 끝나면 STOP 뒤에 M3 를 내지 않는다(문서 8장)
    assert a.submitter.state("m3") == "idle" and "m6" in kinds


def test_m6_rejected_until_deadline_is_reported_as_rejected_not_success():
    m, a, w = _solo(seed=2)
    w.fail = {"m6": math.inf}
    w.inject = [(2.0, ServerMsg("stop"))]
    _advance(m, 40.0)
    assert a.mode == Mode.DONE and a.m6_result == "rejected"
    t_done = next(t for t, msg in a.log if msg.startswith("M6 result"))
    assert 29.5 <= t_done - (a.halt_t - a.t_start) <= 31.0                 # STOP 으로부터 30 s 마감
    assert sum(1 for k, ok in w.submits if k == "m6" and not ok) >= 10     # 기한까지 재시도했다


def test_m1_rejection_is_retried():
    m, a, w = _solo(seed=2)
    w.fail = {"m1": 2}
    _advance(m, 8.0)
    assert a.submitter.acked("m1") and a.submitter.tries("m1") == 3
    assert [ok for k, ok in w.submits if k == "m1"] == [False, False, True]


def test_late_llm_result_for_old_command_is_dropped():
    m, a, _ = _solo(seed=2)
    _advance(m, 0.5)
    a._pending_llm = {"done": True, "res": {"target": None}, "t0": 0.0, "ver": a._cmd_ver - 1}
    a.submitter.jobs.pop("m1", None)
    cmd_before = a.cmd
    a._poll_llm(a.robot.now())
    assert a.cmd is cmd_before and a._pending_llm is None


def test_uav_frame_with_older_stamp_is_ignored():
    m, a, w = _solo(seed=2)
    _advance(m, 3.0)
    w.uav_override = UavFrame(stamp=100.0)
    _advance(m, 3.3)
    assert a._uav_stamp == 100.0
    w.uav_override = UavFrame(stamp=50.0)                      # 늦게 도착한 옛 프레임
    _advance(m, 3.6)
    assert a._uav_stamp == 100.0


def test_real_uav_frame_is_restamped_to_robot_clock_for_tracker():
    """실기 UAV 프레임의 stamp 는 서버 벽시계 epoch 다. 트래커에는 로봇 시계(now − age)로 바꿔 넣어야 트랙이 만료된다."""
    m, a, w = _solo(seed=2)
    _advance(m, 3.0)
    seen = []
    a.tracker.update_uav = lambda uav, pose, *, self_ident: seen.append((uav.stamp, self_ident))
    w.uav_override = UavFrame(stamp=1.78e9, age_s=1.5, recv_mono=None)
    _advance(m, 3.3)
    assert a._uav_stamp == 1.78e9                                 # 순서 판단은 원래 stamp 로
    assert seen and abs(seen[0][0] - (a.robot.now() - 1.5)) < 1.0    # 트래커에는 로봇 시계 시각
    assert all(ident == "B-UGV-2" for _, ident in seen)        # 자기 로봇을 적 추적으로 만들지 않도록 실제 호출부호 전달


# ====================================================================== 시나리오: SAFE_STOP / 잠금 (R04, R08)
def test_lidar_stall_enters_safe_stop_and_resumes_only_after_new_scans():
    m, a, _ = _solo(seed=4)
    state = {}

    def hook(mm):
        r, t = a.robot, mm.world.t
        if "orig" not in state:
            state["orig"] = r.get_scan
        if 6.0 <= t < 9.0:
            if "frozen" not in state:
                state["frozen"] = state["orig"]()
            r.get_scan = lambda: state["frozen"]               # LiDAR 정지: 같은 스캔만 계속 온다
        else:
            r.get_scan = state["orig"]
        if 6.8 <= t < 9.0:
            assert r.u.cmd_v == 0.0 and r.u.cmd_w == 0.0, f"t={t:.2f} 구동 명령이 남아 있다"

    _advance(m, 6.5, hook)
    assert a.mode != Mode.SAFE_STOP
    _advance(m, 8.0, hook)
    assert a.mode == Mode.SAFE_STOP and a._safe["reason"] == "lidar_stale"
    _advance(m, 9.0, hook)
    assert a.mode == Mode.SAFE_STOP                            # 센서가 돌아오기 전에는 복귀하지 않는다
    _advance(m, 9.1, hook)                                     # 새 표본 1개
    _advance(m, 10.0, hook)
    assert a.mode != Mode.SAFE_STOP
    assert any("SAFE_STOP resume" in msg for _, msg in a.log)


def test_camera_stall_does_not_stop_driving_but_blocks_firing():
    m, a, _ = _solo(seed=1)
    _advance(m, 4.0)
    a.robot.get_frame = lambda: None                           # 카메라 정지(LiDAR·SLAM 은 정상)
    d0 = a.robot.u.distance
    _advance(m, 12.0)
    assert a.mode != Mode.SAFE_STOP and a.robot.u.distance > d0 + 0.5    # 주행은 계속
    assert not a.vision_fresh(a.robot.now()) and not a._may_fire(a.robot.now())
    assert a._dets == []                                       # 오래된 검출은 폐기


def test_drive_lock_enters_safe_stop_and_returns_after_release():
    m, a, _ = _solo(seed=4)
    orig = a.robot.status

    def hook(mm):
        t = mm.world.t
        a.robot.status = (lambda: dataclasses.replace(orig(), drive_paused_left=5.0, disabled_left=5.0, hits_taken=4)) \
            if 7.0 <= t < 12.0 else orig
        if 7.2 <= t < 12.0:
            assert a.robot.u.cmd_v == 0.0

    _advance(m, 8.0, hook)
    assert a.mode == Mode.SAFE_STOP and a._safe["reason"] == "drive_locked"
    _advance(m, 13.5, hook)
    assert a.mode != Mode.SAFE_STOP


def test_weapon_lock_blocks_fire_then_allows_after_release():
    m, a, _ = _solo(seed=1)                                    # 지정표적 접근이 시작되는 시각은 탐색 방식에 따라 달라 고정하지 않는다
    orig = a.robot.status
    lock = {"t0": None}

    def hook(mm):
        t = mm.world.t
        if lock["t0"] is None and a.mode == Mode.STRIKE:
            lock["t0"] = t - 0.5                                # 접근 시작 직후부터 22 s 동안 발사 잠금(피격 직후 상태를 흉내)
        on = lock["t0"] is not None and lock["t0"] <= t < lock["t0"] + 22.0
        a.robot.status = (lambda: dataclasses.replace(orig(), disabled_left=10.0, hits_taken=1)) if on else orig

    wd = m.world
    while lock["t0"] is None and wd.t < 90.0:                   # 접근이 시작될 때까지
        _advance(m, wd.t + 1.0, hook)
    assert lock["t0"] is not None, "지정표적 접근이 시작되지 않았다"
    _advance(m, lock["t0"] + 21.0, hook)
    assert a.laser.shots == 0, "발사 잠금 중에 발사했다"
    assert a.mode != Mode.HUNT
    _advance(m, lock["t0"] + 90.0, hook)
    assert a.laser.shots >= 1, "잠금이 풀린 뒤에는 쏠 수 있어야 한다"


# ====================================================================== 시나리오: RECOVER (순위 13·14)
def test_blocked_navigation_recovers_then_safe_stops_when_exhausted():
    m, a, _ = _solo(seed=5)
    _advance(m, 6.0)
    a.stack.step = lambda scan, now, dt: NavOut(0.0, 0.0, "blocked", 1.0)   # 항법이 계속 막힘
    _advance(m, 70.0)
    assert a.stats["recoveries"] >= 3
    assert a.stats["safe_stops"].get("recover_exhausted", 0) >= 1           # 같은 막힘으로 무한 루프 금지
    transitions = [msg for _, msg in a.log if msg.startswith("mode ")]
    assert any("EXPLORE->RECOVER" in msg for msg in transitions)
    assert any("RECOVER->EXPLORE" in msg for msg in transitions)             # 복구 뒤 현재 단계로 재선택
    assert any("->SAFE_STOP" in msg for msg in transitions) and any("SAFE_STOP->EXPLORE" in msg for msg in transitions)


# ====================================================================== 시나리오: M5 / 기본 전략 (R14, 순위 06·15)
def test_m5_local_entry_holds_without_ending_match():
    m, a, w = _solo(seed=1, m5_ends_match=False)               # 월드가 M5 로 경기를 끝내지 않는다 → 서버 STOP 없음
    _advance(m, 285.0)
    assert a.mode == Mode.GOAL and a.m5_entered_local is not None
    assert a.m6_result is None and not a.submitter.started("m6")    # 자체 추정만으로 M6 를 시작하지 않는다
    assert a.robot.u.cmd_v == 0.0
    assert a.stats["multi_output"] == 0


def test_default_strategy_is_m2_first_hunt_off():
    assert Config().mission.engage_ugv is False
    m, a, _ = _solo(seed=1)
    _advance(m, 60.0)
    assert a.stats["hunt_shots"] == 0 and a.stats["multi_output"] == 0
    names = {n for n in a.stats["state_time"]}
    assert "HUNT" not in names and names <= {"EXPLORE", "STRIKE_APPROACH", "STRIKE_AIM", "STAGE", "GOAL", "RECOVER",
                                             "SAFE_STOP", "WAIT"}
