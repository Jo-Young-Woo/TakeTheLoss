"""아군 UGV 추적 지도(avoid.friend_track): 출발 때 거점 위치로 신원, 프레임마다 이어 추적, 놓치면 '식별 안 됨'으로 남기고 다시 잇지 않는다. 시뮬을 쓰지 않는다."""
import json
import math
from unittest.mock import MagicMock

import pytest

from hw_fakes import make_robot
from maicon26.config import Config
from maicon26.link.base import ServerLink, UavDet, UavFrame
from maicon26.mission.manager import Agent
from maicon26.perception.detector import Detector
from maicon26.perception.threat_board import ThreatBoard
from maicon26.perception.tracks import UgvTracker
from maicon26.types import Detection, Frame, Pose2D

B1, B2, B3 = (0.4, 4.16), (0.4, 2.5), (0.4, 0.83)          # BLUE 거점 중심 1·2·3 (자기 = 슬롯 2)
SELF = "B-UGV-2"


def cfg_on(**over):
    cfg = Config()
    cfg.avoid.friend_track = True
    for k, v in over.items():
        setattr(cfg.avoid, k, v)
    return cfg


def frame(t, *pts, interval=None):
    """pts: (x, y) 또는 (x, y, team)."""
    dets = [UavDet(p[0], p[1], p[2] if len(p) > 2 else None) for p in pts]
    return UavFrame(stamp=t, detections=dets, interval_s=interval)


def feed(tr, t, *pts, me=B2, interval=None):
    tr.update_uav(frame(t, *pts, interval=interval), Pose2D(me[0], me[1], 0.0), self_ident=SELF)


def tracker(**over):
    return UgvTracker(cfg_on(**over), "BLUE")


def slots(tr, now):
    return {s["ident"]: s for s in tr.fmap.snapshot(now)}


def track_of(tr, ident):
    return next((t for t in tr.tracks if t.ident == ident), None)


# ====================================================================== 기본값
def test_default_is_off_and_old_birth_logic_applies():
    tr = UgvTracker(Config(), "BLUE")
    assert tr.fmap is None
    feed(tr, 0.0, B1, B2, B3)
    assert sorted(t.ident for t in tr.tracks) == ["B-UGV-1", "B-UGV-3"]            # 옛 방식: 거점 근접 ident
    assert tr.tick_friends(99.0) == []


def test_real_config_turns_it_on_and_code_default_is_off():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    assert Config().avoid.friend_track is False
    assert Config.load(str(root / "config/waveshare_real.json")).avoid.friend_track is True


# ====================================================================== 출생
def test_friends_are_born_at_their_own_bases_and_self_is_not_tracked():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    s = slots(tr, 0.0)
    assert set(s) == {"B-UGV-1", "B-UGV-3"}                                         # 자기 슬롯 제외
    assert all(v["state"] == "tracked" for v in s.values())
    assert s["B-UGV-1"]["x"] == pytest.approx(0.4) and s["B-UGV-1"]["y"] == pytest.approx(4.16)
    assert track_of(tr, "B-UGV-1") is not None and track_of(tr, "B-UGV-3") is not None
    assert track_of(tr, SELF) is None


def test_enemy_base_births_still_work_in_friend_mode():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3, (5.72, 2.5))                                          # 상대 거점의 UGV
    assert track_of(tr, "R-UGV-2") is not None


def test_friend_not_at_its_base_in_the_first_window_is_never_identified():
    tr = tracker()
    feed(tr, 0.0, B1, B2)                                                           # 3 번이 안 보인다
    feed(tr, 1.0, B1, B2, (1.5, 0.83))                                              # 이미 거점을 떠난 뒤에 나타남
    feed(tr, 5.0, (0.4, 0.83), B1, B2)                                              # 창이 닫힌 뒤 거점에 보여도 새로 붙이지 않는다
    s = slots(tr, 5.0)
    assert s["B-UGV-3"]["state"] == "unidentified" and s["B-UGV-3"]["reason"] == "not_seen_at_start"
    assert track_of(tr, "B-UGV-3") is None


def test_two_uncoloured_robots_at_one_base_cannot_be_told_apart_at_birth():
    tr = tracker()
    feed(tr, 0.0, B1, (0.5, 4.3), B2)
    s = slots(tr, 0.0)
    assert s["B-UGV-1"]["state"] == "unidentified" and s["B-UGV-1"]["reason"] == "birth_ambiguous"
    assert track_of(tr, "B-UGV-1") is None


def test_opponent_coloured_detection_at_a_friendly_base_is_not_born_as_a_friend():
    tr = tracker()
    feed(tr, 0.0, (B1[0], B1[1], "RED"), B2)
    assert slots(tr, 0.0)["B-UGV-1"]["state"] == "unidentified"


def test_birth_window_extends_for_slow_video():
    tr = tracker()
    feed(tr, 0.0, B2, interval=5.0)                                                 # 첫 프레임엔 아군이 안 보인다 — 5 s 주기 영상: 창 = max(3, 1.5×5)=7.5 s
    feed(tr, 4.0, B1, B2, interval=5.0)
    assert slots(tr, 4.0)["B-UGV-1"]["state"] == "tracked"


def test_uav_supplied_friend_ident_is_trusted():
    tr = tracker()
    tr.update_uav(UavFrame(stamp=0.0, detections=[UavDet(2.0, 2.0, "BLUE", "B-UGV-3")]), Pose2D(*B2, 0.0), self_ident=SELF)
    assert slots(tr, 0.0)["B-UGV-3"]["state"] == "tracked"
    assert slots(tr, 0.0)["B-UGV-3"]["x"] == pytest.approx(2.0)


# ====================================================================== 이어짐
def test_a_moving_friend_keeps_its_identity_and_track_across_frames():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    tid = track_of(tr, "B-UGV-1").tid
    for i in range(1, 8):
        feed(tr, float(i), (0.4 + 0.4 * i, 4.16 - 0.1 * i), B2, (0.4 + 0.3 * i, 0.83))
    s = slots(tr, 7.0)
    assert s["B-UGV-1"]["state"] == "tracked" and s["B-UGV-3"]["state"] == "tracked"
    assert s["B-UGV-1"]["x"] == pytest.approx(3.2) and s["B-UGV-1"]["tid"] == tid
    assert track_of(tr, "B-UGV-1").tid == tid                                       # 같은 트랙을 이어 갱신
    assert len([t for t in tr.tracks if t.ident == "B-UGV-1"]) == 1


def test_friend_missing_for_a_short_time_is_kept_then_resumed():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    feed(tr, 1.0, B3, B2)                                                           # 1 번이 안 보임(가려짐)
    feed(tr, 2.0, B3, B2)
    assert slots(tr, 2.0)["B-UGV-1"]["state"] == "tracked"                          # 4 s 미만
    feed(tr, 3.0, (0.7, 4.0), B3, B2)
    s = slots(tr, 3.0)["B-UGV-1"]
    assert s["state"] == "tracked" and s["x"] == pytest.approx(0.7)


# ====================================================================== 놓침 → 식별 안 됨
def test_an_uncoloured_robot_cutting_in_makes_the_friend_unidentified_and_never_steals_the_ident():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    feed(tr, 1.0, (0.55, 4.0), B2, B3)
    feed(tr, 2.0, (0.70, 3.9), (0.78, 3.7), B2, B3)                                 # 두 대가 0.2 m 안: 어느 쪽이 1 번인지 모른다
    s = slots(tr, 2.0)
    assert s["B-UGV-1"]["state"] == "unidentified" and s["B-UGV-1"]["reason"] == "ambiguous"
    assert all(t.ident != "B-UGV-1" for t in tr.tracks)                             # 어느 트랙도 1 번 신원을 갖지 않는다
    assert any(t.ident_lost == "B-UGV-1" for t in tr.tracks)
    assert tr.fmap.stats["lost_ambiguous"] == 1
    assert s["B-UGV-3"]["state"] == "tracked"                                       # 다른 아군은 영향 없다


def test_lost_friend_is_not_reattached_by_proximity_later():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    feed(tr, 1.0, (0.55, 4.0), B2, B3)
    feed(tr, 2.0, (0.70, 3.9), (0.78, 3.7), B2, B3)                                 # 놓침
    for i in range(3, 10):
        feed(tr, float(i), (0.75, 3.8), B2, B3)                                     # 그 뒤 같은 자리에 깨끗하게 한 대만 보여도
    assert slots(tr, 9.0)["B-UGV-1"]["state"] == "unidentified"
    assert all(t.ident != "B-UGV-1" for t in tr.tracks)


def test_friend_unseen_beyond_the_limit_is_lost_even_without_new_frames():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    assert tr.tick_friends(3.9) == []
    lost = tr.tick_friends(4.5)                                                     # 영상이 끊긴 채 4 s 넘게 확인 못 함
    assert sorted(lost) == ["B-UGV-1", "B-UGV-3"]
    assert slots(tr, 4.5)["B-UGV-1"]["reason"] == "timeout"
    assert all(t.ident is None for t in tr.tracks)
    assert tr.tick_friends(9.0) == []                                               # 두 번 세지 않는다


def test_loss_limit_scales_with_slow_uav_video():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3, interval=5.0)
    assert tr.tick_friends(11.0, 5.0) == []                                         # 2.5×5=12.5 s 이전
    assert len(tr.tick_friends(13.0, 5.0)) == 2


def test_friend_far_from_the_gate_is_not_chased_and_times_out():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    feed(tr, 1.0, (3.0, 4.16), B2, B3)                                              # 1 s 에 2.6 m — 불가능한 이동: 같은 로봇이 아니다
    assert slots(tr, 1.0)["B-UGV-1"]["x"] == pytest.approx(0.4)                     # 위치를 옮기지 않는다
    tr.tick_friends(5.0)
    assert slots(tr, 5.0)["B-UGV-1"]["state"] == "unidentified"
    assert track_of(tr, "B-UGV-1") is None


def test_two_friends_close_together_both_become_unidentified():
    tr = tracker()
    me = (3.0, 4.5)                                                                 # 자기는 멀리 있다
    feed(tr, 0.0, B1, B3, me=me)
    for t, ya, yb in ((1.0, 3.7, 1.3), (2.0, 3.25, 1.75), (3.0, 2.8, 2.2)):
        feed(tr, t, (0.4, ya), (0.4, yb), me=me)
        s = slots(tr, t)
        assert s["B-UGV-1"]["state"] == "tracked" and s["B-UGV-3"]["state"] == "tracked"    # 0.6 m 이상 떨어져 있는 동안은 구분된다
    feed(tr, 4.0, (0.4, 2.55), (0.4, 2.45), me=me)                                  # 두 아군이 0.1 m 안으로 붙는다
    s = slots(tr, 4.0)
    assert s["B-UGV-1"]["state"] == "unidentified" and s["B-UGV-3"]["state"] == "unidentified"


def _two_friends_approaching(tr, me):
    feed(tr, 0.0, B1, B3, me=me)
    for t, ya, yb in ((1.0, 3.7, 1.3), (2.0, 3.3, 1.75), (3.0, 3.0, 2.2), (4.0, 3.0, 2.6)):
        feed(tr, t, (0.4, ya), (0.4, yb), me=me)
        assert all(v["state"] == "tracked" for v in slots(tr, t).values())              # 0.4 m 떨어져 있는 동안은 구분된다


def test_one_detection_between_two_friends_is_not_given_to_either():
    tr = tracker()
    me = (3.0, 4.5)
    _two_friends_approaching(tr, me)
    feed(tr, 5.0, (0.4, 2.8), me=me)                                                # 한 대만 보이는데 두 아군 모두와 0.2 m 안: 누구인지 모른다
    s = slots(tr, 5.0)
    assert s["B-UGV-1"]["state"] == "unidentified" and s["B-UGV-3"]["state"] == "unidentified"


def test_a_detection_clearly_nearer_to_one_friend_never_moves_the_other_friend():
    tr = tracker()
    me = (3.0, 4.5)
    _two_friends_approaching(tr, me)
    feed(tr, 5.0, (0.4, 3.0), me=me)                                                # 1 번의 바로 위(0), 3 번과는 0.4 m — 3 번은 안 보인다
    s = slots(tr, 5.0)
    assert s["B-UGV-1"]["state"] == "tracked" and s["B-UGV-1"]["y"] == pytest.approx(3.0)
    assert s["B-UGV-3"]["state"] == "tracked" and s["B-UGV-3"]["y"] == pytest.approx(2.6)     # 안 보였을 뿐 — 위치도 1 번 쪽으로 끌려가지 않았다
    assert s["B-UGV-3"]["age_s"] == pytest.approx(1.0)
    assert tr.tick_friends(8.5) == ["B-UGV-3"] and slots(tr, 8.5)["B-UGV-3"]["reason"] == "timeout"    # 계속 안 보이면 시간 초과로 놓친다


def test_an_unfiltered_self_detection_is_never_taken_as_a_hidden_friend():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    # 1 번이 안 보이는 프레임에 우리 자신의 검출이 자기 자세 기대 위치(0.6, 3.9)에서 0.33 m 어긋나 필터를 못 통과했다.
    # 1 번 허용 거리 안(0.23 m)이지만 자기 자세와 0.35 m 안이라 아군 후보에서 뺀다
    tr.update_uav(frame(1.0, (0.62, 4.23), B3), Pose2D(0.6, 3.9, 0.0), self_ident=None)
    assert tr.self_filtered == 1                                                    # (첫 프레임의 자기 검출만 걸러졌다 — 이번 프레임은 못 찾았다)
    s = slots(tr, 1.0)["B-UGV-1"]
    assert s["state"] == "tracked" and s["y"] == pytest.approx(4.16) and s["n_match"] == 1


def test_a_friend_whose_detection_is_clearly_owned_by_a_nearer_friend_is_not_an_alternative():
    tr = tracker()
    me = (3.0, 4.5)
    feed(tr, 0.0, B1, B3, me=me)
    for t, ya, yb in ((1.0, 3.8, 1.4), (2.0, 3.5, 2.0), (3.0, 3.5, 2.6), (4.0, 3.5, 3.0)):
        feed(tr, t, (0.4, ya), (0.4, yb), me=me)
        assert all(v["state"] == "tracked" for v in slots(tr, t).values())
    feed(tr, 5.0, (0.4, 3.5), (0.4, 2.7), me=me)                                    # 3 번이 1 번에서 멀어지는 쪽으로 움직였다 — 1 번의 검출(0.5 m 위)은 1 번이 분명히 차지한다
    assert all(v["state"] == "tracked" for v in slots(tr, 5.0).values())


def test_opponent_coloured_detection_never_takes_a_friend_ident():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    feed(tr, 1.0, (0.6, 4.0, "RED"), B2, B3)                                        # 상대 색: 후보가 아니다
    t_enemy = next(t for t in tr.tracks if t.team == "RED")
    assert t_enemy.ident is None
    assert slots(tr, 1.0)["B-UGV-1"]["x"] == pytest.approx(0.4)                     # 위치 갱신 없음
    feed(tr, 2.0, (0.7, 3.9), B2, B3)                                               # 색 없는 검출은 이어진다(상대 색 검출은 후보에서 제외)
    assert slots(tr, 2.0)["B-UGV-1"]["state"] == "tracked"


def test_uncoloured_detection_near_a_friend_track_does_not_hijack_it_from_the_camera_side():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    t1 = track_of(tr, "B-UGV-1")
    # 카메라가 색 없이 근처를 봤다 — 예약된 트랙에 합쳐지면 안 된다
    tr._upsert(0.5, 4.1, 0.5, None, 0.9, "cam")
    assert track_of(tr, "B-UGV-1") is t1 and (t1.x, t1.y) == pytest.approx(B1)
    assert any(t.ident is None and abs(t.x - 0.5) < 0.01 for t in tr.tracks)        # 별도 '식별 안 됨' 트랙


# ====================================================================== 카메라 보강
def test_camera_reading_the_team_colour_keeps_the_friend_tracked_while_video_is_missing():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    for i in range(1, 8):                                                           # UAV 영상 없음, 카메라가 1 번의 아군 색을 계속 본다
        tr._upsert(0.4 + 0.05 * i, 4.16, float(i), "BLUE", 0.9, "cam")
        tr.tick_friends(float(i))
    s = slots(tr, 7.0)
    assert s["B-UGV-1"]["state"] == "tracked" and s["B-UGV-1"]["age_s"] == pytest.approx(0.0)
    assert s["B-UGV-3"]["state"] == "unidentified"                                  # 3 번은 못 봐서 놓침


def test_camera_colour_between_two_friends_is_not_credited_to_either():
    tr = tracker()
    me = (3.0, 4.5)
    feed(tr, 0.0, B1, B3, me=me)
    feed(tr, 1.0, (0.4, 3.7), (0.4, 1.3), me=me)
    feed(tr, 2.0, (0.4, 3.25), (0.4, 1.75), me=me)
    tr._upsert(0.4, 2.5, 2.5, "BLUE", 0.9, "cam")                                   # 두 아군의 한가운데(0.75 m 씩) — 누구인지 모른다
    s = slots(tr, 2.5)
    assert s["B-UGV-1"]["age_s"] == pytest.approx(0.5) and s["B-UGV-3"]["age_s"] == pytest.approx(0.5)   # 확인 시각이 갱신되지 않았다
    tr._upsert(0.4, 3.3, 2.6, "BLUE", 0.9, "cam")                                   # 1 번에 분명히 가까운 검출은 이어진다
    s = slots(tr, 2.6)
    assert s["B-UGV-1"]["age_s"] == pytest.approx(0.0) and s["B-UGV-3"]["age_s"] == pytest.approx(0.6)


# ====================================================================== 위협판(피아식별 결과)
def board_view(tr, now):
    board = ThreatBoard(tr.cfg, "BLUE")
    return {t.tid: t for t in board.update(tr, [], now, Pose2D(*B2, 0.0))}


def test_tracked_friend_is_friend_and_lost_friend_is_an_unknown_ugv():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    tid1 = track_of(tr, "B-UGV-1").tid
    v = board_view(tr, 0.5)
    assert v[tid1].friend is True and v[tid1].p_enemy < 0.1
    tr.tick_friends(4.5)                                                            # 놓침
    v = board_view(tr, 5.0)
    assert v[tid1].friend is False and v[tid1].p_enemy == pytest.approx(0.5)        # 아군도 적도 아님: 미확인 UGV 로 다룬다
    assert v[tid1].ident is None


def test_lost_friend_with_team_colour_votes_is_a_friend_without_a_number():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    t1 = track_of(tr, "B-UGV-1")
    tr.tick_friends(4.5)
    for i in range(4):
        tr._upsert(t1.x, t1.y, 5.0 + i * 0.1, "BLUE", 0.9, "cam")                   # 놓친 뒤 카메라가 아군 색을 읽는다
    v = board_view(tr, 5.5)
    assert v[t1.tid].friend is True and v[t1.tid].ident is None


# ====================================================================== 보고용 표현
def test_snapshot_and_summary_are_json_safe_and_hide_nothing_about_loss():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    tr.tick_friends(9.0)
    out = json.loads(json.dumps(tr.fmap.summary(9.0), allow_nan=False))
    assert {s["ident"]: s["state"] for s in out["slots"]} == {"B-UGV-1": "unidentified", "B-UGV-3": "unidentified"}
    assert [e["kind"] for e in out["events"]].count("lost") == 2
    assert out["stats"]["lost_timeout"] == 2


def test_red_team_uses_red_slots():
    tr = UgvTracker(cfg_on(), "RED")
    tr.update_uav(frame(0.0, (5.72, 4.16), (5.72, 2.5), (5.72, 0.83)), Pose2D(5.72, 0.83, 0.0), self_ident="R-UGV-3")
    assert {s["ident"] for s in tr.fmap.snapshot(0.0)} == {"R-UGV-1", "R-UGV-2"}
    assert all(s["state"] == "tracked" for s in tr.fmap.snapshot(0.0))


def test_self_detection_that_was_not_filtered_is_not_taken_as_a_friend():
    tr = tracker()
    feed(tr, 0.0, B1, B2, B3)
    # 자기 검출이 자기 자세와 0.3 m 어긋나 필터를 못 통과한 프레임 — 아군 1 번 근처가 아니라 자기 근처라 후보에서 빠진다
    tr.update_uav(frame(1.0, B1, (0.55, 2.6), B3), Pose2D(0.6, 2.7, 0.0), self_ident=None)
    s = slots(tr, 1.0)
    assert s["B-UGV-1"]["state"] == "tracked" and s["B-UGV-3"]["state"] == "tracked"


# ====================================================================== 매니저
class NullLink(ServerLink):
    def poll(self):
        return []

    def submit(self, kind, payload):
        return True


class NoDet(Detector):
    def detect(self, frame):
        return []


def make_agent(**over):
    cfg = cfg_on(**over)
    cfg.nav.backend = "builtin"
    robot, _, clock = make_robot("practice", "idle", cfg=cfg)
    agent = Agent(robot, NullLink(), cfg=cfg, detector=NoDet())
    agent.team, agent.slot = "BLUE", 2
    agent.tracker = UgvTracker(cfg, "BLUE")
    agent.stack = MagicMock()
    agent.stack.pose = Pose2D(*B2, 0.0)
    logs = []
    agent._log = lambda now, msg: logs.append(msg)
    return agent, logs


def test_manager_logs_birth_and_loss_once_and_publishes_the_map():
    agent, logs = make_agent()
    feed(agent.tracker, 0.0, B1, B2, B3)
    agent._friend_report(0.1)
    assert sum("추적 시작" in m for m in logs) == 2
    agent._friend_report(0.2)
    assert sum("추적 시작" in m for m in logs) == 2                                  # 같은 사건을 다시 쓰지 않는다
    assert agent.stats["friend_map"]["slots"][0]["state"] == "tracked"
    agent._friend_report(9.0)                                                       # 영상 없이 시간만 흘렀다
    assert sum("식별 안 됨" in m for m in logs) == 2 and any("다시 이어 붙이지 않는다" in m for m in logs)
    assert agent.stats["friend_map"]["slots"][0]["state"] == "unidentified"
    json.dumps(agent.stats["friend_map"], allow_nan=False)


def test_manager_report_is_a_noop_when_off_and_never_raises():
    agent, logs = make_agent()
    agent.cfg.avoid.friend_track = False
    agent.tracker = UgvTracker(agent.cfg, "BLUE")
    agent._friend_report(1.0)
    assert "friend_map" not in agent.stats and logs == []
    agent, logs = make_agent()
    agent.tracker.tick_friends = MagicMock(side_effect=RuntimeError("x"))
    agent._friend_report(1.0)                                                       # 예외는 삼킨다
    assert agent.stats["friend_map_errors"] == 1


# ====================================================================== 설정 검증
def test_sanitize_resets_bad_friend_keys_and_leaves_defaults_alone():
    from maicon26.validate import sanitize_config
    assert sanitize_config(Config()) == []
    cfg = Config()
    cfg.avoid.friend_lost_s, cfg.avoid.friend_gate_m, cfg.avoid.friend_max_speed = 0.0, float("nan"), 9.0
    assert set(sanitize_config(cfg)) == {"avoid.friend_lost_s", "avoid.friend_gate_m", "avoid.friend_max_speed"}
    assert (cfg.avoid.friend_lost_s, cfg.avoid.friend_gate_m, cfg.avoid.friend_max_speed) == (4.0, 0.5, 0.6)
