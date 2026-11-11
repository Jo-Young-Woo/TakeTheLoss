"""코드 검토 ⑧~⑩ 수정 검증: ⑧ 센서 stamp 시계 기준 ⑨ 경기별 재초기화(러너) ⑩ 조준 직전 겨냥점 초기화."""
import random

import pytest

from maicon26.config import Config
from maicon26.hal.clock import StampSync
from maicon26.link.base import ServerMsg
from maicon26.mission.aim import AimController
from maicon26.mission.manager import Agent, Mode
from maicon26.mission.runner import MatchRunner, ReplayLink
from maicon26.sim.run import build_match
from maicon26.types import Detection, Frame

from test_fixes2 import _run, _solo
from test_state_machine import Wrap, _advance


# ====================================================================== ⑧ 시계 기준
def test_same_clock_stamps_are_left_alone_even_with_a_latency_spike():
    s, now, out, stamps = StampSync(), 100.0, [], []
    for i in range(40):
        lat = 1.5 if i == 20 else 0.06 + 0.02 * (i % 4)                 # 한 번의 1.5 s 지연 스파이크
        stamp = now - lat
        stamps.append(stamp)
        out.append(s.to_robot(stamp, now))
        now += 0.1
    assert s.mode == "same" and out == stamps                            # 변환하지 않는다(스파이크로 모드가 뒤집히지 않는다)


def test_foreign_clock_is_converted_into_the_capture_to_arrival_interval():
    rng = random.Random(1)
    s, now = StampSync(), 12.0                                           # 로봇 시계(부팅 후 12 s)
    for i in range(80):
        lat = rng.uniform(0.05, 0.25)
        capture = now - lat
        stamp = 1.7e9 + capture                                          # 센서/ROS 는 에포크 시계
        conv = s.to_robot(stamp, now)
        assert capture - 1e-6 <= conv <= now + 1e-6, (i, capture, conv, now)   # 항상 [촬영, 도착] 사이
        now += 0.1
    ch = s.consume_change()                                              # 모드가 바뀐 순간의 (mode, offset)
    assert s.mode == "offset" and ch is not None and ch[0] == "offset" and abs(ch[1] + 1.7e9) < 1.0
    assert s.consume_change() is None                                    # 변경 통지는 한 번만


def test_known_minimum_latency_removes_the_residual_error_and_a_wrong_value_shows_up_as_exactly_that_error():
    def errors(floor):
        rng, s, now, out = random.Random(3), StampSync(floor_s=floor), 5.0, []
        for i in range(300):
            lat = rng.uniform(0.10, 0.25)                                # 실제 최소 지연 0.10 s
            capture = now - lat
            out.append(s.to_robot(1.7e9 + capture, now) - capture)
            now += 0.07
        return out[60:]                                                  # 추정이 안정된 뒤
    right = errors(0.10)
    assert max(abs(e) for e in right) < 0.02                              # 맞는 floor: 변환 시각 ≈ 촬영 시각
    wrong = errors(0.05)
    assert all(abs(e - 0.05) < 0.02 for e in wrong)                       # 틀린 floor: 틀린 만큼(+0.05 s) 일정하게 어긋난다
    assert min(errors(0.0)) >= 0.09                                       # 안 넣으면 최소 지연만큼 늦다


def _foreign_delayed_frames(r, seed=0):
    """카메라가 에포크 시계로 stamp 를 달고 50~250 ms 늦게 도착하는 상황을 흉내 낸다. 도착/촬영 시각을 기록한다."""
    rng = random.Random(seed)
    orig = r.get_frame
    pend, delivered, last = [], {}, [None]

    def get():
        now = r.now()
        fr = orig()
        if fr is not None and fr.stamp not in {p[1].stamp for p in pend} and fr.stamp not in {k - 1.7e9 for k in delivered}:
            pend.append((fr.stamp + rng.uniform(0.05, 0.25), fr))
        ready = [p for p in pend if p[0] <= now]
        for p in ready:
            pend.remove(p)
        if ready:
            _, f = max(ready, key=lambda p: p[1].stamp)
            shifted = Frame(f.image, f.stamp + 1.7e9, f.pan, f.tilt, f.width, f.height, f.meta)
            delivered[shifted.stamp] = (f.stamp, now)                    # (촬영 시각, 도착 시각)
            last[0] = shifted
        return last[0]

    r.get_frame = get
    return delivered


def test_agent_converts_a_foreign_camera_clock_and_never_falls_off_the_pose_history():
    m, a, r = _solo(seed=1)
    delivered = _foreign_delayed_frames(r)
    _run(m, 20.0)
    kept = [f for f in a._frames if f.stamp in delivered]
    assert len(kept) >= 10 and a.stats["det_consumed"] > 50
    for f in kept[3:]:                                                   # 판단 표본(5개)이 쌓인 뒤
        capture, arrival = delivered[f.stamp]
        assert capture - 1e-6 <= f.meta["t_robot"] <= arrival + 1e-6      # 자세는 촬영~도착 사이 시각으로 조회한다
    assert a.stats["pose_at_clamped"] == 0                               # 예전에는 에포크 stamp 라 이력 밖 → 최신 자세로 조용히 고정됐다
    warns = [msg for _, msg in a.log if msg.startswith("WARN frame stamp")]
    assert len(warns) == 1 and "mode=offset" in warns[0]                 # 시계가 다르다는 사실은 한 번 알린다


def test_pose_at_outside_the_history_is_reported_not_silent():
    m, a, r = _solo(seed=1)
    _run(m, 3.0)
    n0 = a.stats["pose_at_clamped"]
    a.pose_at(a.robot.now() + 5.0)                                       # 말이 안 되는 미래 시각
    a.pose_at(a.robot.now() - 1000.0)
    assert a.stats["pose_at_clamped"] == n0 + 2
    assert sum(1 for _, msg in a.log if msg.startswith("WARN pose_at")) == 1   # 로그는 한 번만 + 200회마다


def test_same_clock_sim_is_unchanged():
    m, a, r = _solo(seed=1)
    _run(m, 15.0)
    assert a.stats["pose_at_clamped"] == 0 and a._sync_frame.mode == "same" and a._sync_scan.mode == "same"
    assert not any("stamp 가 robot.now()" in msg for _, msg in a.log)


# ====================================================================== ⑨ 경기별 재초기화
def test_replay_link_hands_over_pending_messages_first():
    class Inner:
        def poll(self):
            return [ServerMsg("event")]

    inner = Inner()
    link = ReplayLink(inner, [ServerMsg("start"), ServerMsg("command")])    # type: ignore[arg-type]
    assert [x.kind for x in link.poll()] == ["start", "command", "event"]
    assert [x.kind for x in link.poll()] == ["event"]                        # 이월 메시지는 한 번만


def test_next_match_messages_during_report_are_carried_but_old_duplicates_are_not():
    m, a, w = None, None, None
    m = build_match(seed=2, cfg=Config(), teams=["B2"])
    a = m.agents[0]
    w = Wrap(a.server, match_id="M-1")
    a.server = w
    a.submitter.server = w
    w.fail = {"m6": 3}                                                       # 보고가 몇 초 걸리게
    w.inject = [(3.0, ServerMsg("stop", data={"match_id": "M-1"})),
                (3.4, ServerMsg("start", data={"match_id": "M-1"})),          # 이전 경기의 늦은 중복 START → 이월 아님
                (3.6, ServerMsg("start", data={"match_id": "M-2"})),          # 다음 경기 START → 이월
                (3.7, ServerMsg("command", text="레이더 시설을 타격하라", data={"match_id": "M-2"})),
                (3.8, ServerMsg("start"))]                                    # match_id 없는 메시지는 구분 불가 → 이월하지 않는다
    _advance(m, 8.0)
    assert a.mode == Mode.DONE
    assert [(x.kind, x.data["match_id"]) for x in a.carry_over] == [("start", "M-2"), ("command", "M-2")]
    assert a.stats["msg_rejected"].get("after-stop", 0) >= 2


def test_runner_builds_a_fresh_agent_per_match_and_hands_over_early_messages():
    cfg = Config()
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    m = build_match(seed=2, cfg=cfg, teams=["B2"])
    robot = m.robots[0]
    link = Wrap(m.servers[0], match_id="M-1")
    link.fail = {"m6": 2}                                                    # 첫 경기 보고가 STOP 뒤 몇 초 걸린다
    link.inject = [(3.0, ServerMsg("stop", data={"match_id": "M-1"})),
                   (3.5, ServerMsg("start", data={"match_id": "M-2"})),       # 보고 중에 다음 경기 시작이 먼저 도착
                   (3.6, ServerMsg("command", text="레이더 시설을 타격하라", data={"match_id": "M-2"})),
                   (14.0, ServerMsg("stop", data={"match_id": "M-2"}))]
    made, done = [], []

    def make(srv):
        ag = Agent(robot, srv, cfg=cfg, seed=len(made), name=f"match{len(made) + 1}")
        made.append(ag)
        return ag

    runner = MatchRunner(make, link, max_matches=2, sleep=lambda s: None, after_tick=lambda ag: m.world.step(0.05),
                         on_match_done=lambda s, ag: done.append((s, ag)))
    res = runner.run()
    assert len(res) == 2 and made[0] is not made[1]
    s1, s2 = res
    assert (s1.match_id, s1.stop_source, s1.m6_result, s1.carried) == ("M-1", "server", "acked", 2)
    assert (s2.match_id, s2.stop_source) == ("M-2", "server")
    a1, a2 = made
    assert a2.t_start is not None and a1.halt_t is not None and a2.t_start > a1.halt_t + 1.0   # 둘째 경기는 첫 경기 보고가 끝난 뒤 시작
    assert a2.cmd is not None and a2.submitter.acked("m1")                  # 이월된 명령도 처리했다(경기 시작 직후)
    mt = a2.stats["mode_time"]                                              # 새 Agent 가 실제로 일했다(탐색 후 이월된 명령의 표적 접근)
    assert mt.get("EXPLORE", 0.0) > 1.0 and sum(mt.values()) > 5.0 and "WAIT" not in mt
    assert a2.latch.match_id == "M-2" and a1.latch.match_id == "M-1"        # 래치·상태가 경기마다 따로다
    assert a2.omap is not a1.omap and a2.stack is not a1.stack


# ====================================================================== ⑩ 조준 직전 겨냥점 초기화
def _det(u, v=240.0, cls="RADAR_FACILITY"):
    d = Detection(cls, 0.9, (u - 30.0, v - 40.0, u + 30.0, v + 40.0), 1.0)
    d.extra["sensors"] = [(u, v, 6.0, 0.9)]
    return d


def test_aim_restarts_tracking_after_a_gap_instead_of_blending_with_the_old_target():
    ac = AimController(Config())
    ac.update(_det(200.0), 0.0, 0.0, 0.0, 1.5)
    ac.update(_det(300.0), 0.0, 0.0, 2.0, 1.5)                                # 2 s 공백(> lost_timeout 0.8 s), 이동 100 px(< 점프 한계)
    assert ac.auto_resets == 1
    assert ac._last_pt[0] == pytest.approx(300.0)                             # 옛 값과 섞이지 않는다(섞이면 0.55·300+0.45·200 = 255)


def test_aim_restarts_on_a_jump_but_not_on_ordinary_motion():
    ac = AimController(Config())
    t, u = 0.0, 200.0
    for _ in range(30):                                                       # 프레임마다 10 px 이동(= 정상 추적)
        ac.update(_det(u), 0.0, 0.0, t, 1.5)
        t, u = t + 0.05, u + 10.0
    assert ac.auto_resets == 0
    ac.update(_det(u + 300.0), 0.0, 0.0, t, 1.5)                              # 연속 검출 중 다른 대상으로 교체(공백 없음)
    assert ac.auto_resets == 1 and ac._last_pt[0] == pytest.approx(u + 300.0)


def test_after_a_reset_the_ugv_sensor_is_chosen_nearest_to_the_image_center():
    ac = AimController(Config())
    ac.update(_det(100.0, cls="UGV"), 0.0, 0.0, 0.0, 1.5)                     # 옛 대상은 왼쪽 가장자리
    d = Detection("UGV", 0.9, (250.0, 200.0, 560.0, 330.0), 1.0)
    d.extra["sensors"] = [(300.0, 290.0, 9.0, 0.9), (520.0, 290.0, 9.0, 0.9)]  # 새 UGV 의 두 돔
    ac.update(d, 0.0, 0.0, 3.0, 1.5)                                          # 공백 뒤: 옛 겨냥점(100) 이 아니라 화면 중앙(320) 기준
    assert ac._last_pt[0] == pytest.approx(300.0)


def test_manager_resets_the_aim_when_it_picks_a_new_target_face():
    m, a, r = _solo(seed=1)
    calls = []
    orig = a.aim.reset

    def spy():
        calls.append((a.mode, a._strike.get("phase")))
        orig()

    a.aim.reset = spy
    _run(m, 60.0)                                                              # seed 1 은 47 s 쯤 지정표적 면을 고른다
    assert (Mode.STRIKE, "nav") in calls                                      # 표적 면을 고른 직후(nav 단계)에 초기화했다
