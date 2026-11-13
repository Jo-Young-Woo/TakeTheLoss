#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 없이 도는 단위 시험.  python3 tests/test_all.py  또는  pytest tests/"""
import contextlib
import io
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import bag_check  # noqa: E402
import extract_images  # noqa: E402
import scan_secret  # noqa: E402
from teleop_core import (DoublePress, Hold, KeyParser, MISSION_MODE, OTHER_PUBLISHER, Options, Teleop,  # noqa: E402
                         drive_allowed, ramp, target_velocity)

FAKE_CODE = "K7M2Q9XA"                                    # 문서·시험에 쓰는 가짜 임무코드


class FakeRos:
    def __init__(self, mode="practice", cmd="idle"):
        self.mode, self.cmd, self.others = mode, cmd, 0
        self.cmds, self.events, self.gimbal, self.fire = [], [], [], []
        self.has_gimbal = self.has_fire = True

    def other_cmd_publishers(self):
        return self.others

    def publish_cmd(self, v, w):
        self.cmds.append((v, w))

    def event(self, d):
        self.events.append(d)

    def discover(self):
        pass

    def call_gimbal(self, pan, tilt, speed, acc, cb):
        if self.has_gimbal:
            self.gimbal.append((pan, tilt, cb))
        return self.has_gimbal

    def call_fire(self, cb):
        if self.has_fire:
            self.fire.append(cb)
        return self.has_fire


def make(ros=None, **opt):
    ros = ros or FakeRos()
    said = []
    return Teleop(ros, Options(**opt), 0.0, say=said.append), ros, said


def run(t, t0, t1, key=None, every=0.05, dt=0.01):
    """t0~t1 동안 tick. key 가 있으면 every 간격으로 키 반복을 넣는다."""
    now, nxt = t0, t0
    while now < t1 - 1e-9:
        if key and now >= nxt - 1e-9:
            t.on_key(key, now)
            nxt += every
        t.tick(now)
        now += dt
    return now


# ---------------------------------------------------------------- 키 해석
def test_parser_arrows_both_forms():
    p = KeyParser()
    assert p.feed(b"\x1b[A\x1b[B\x1b[C\x1b[D") == ["UP", "DOWN", "RIGHT", "LEFT"]
    assert p.feed(b"\x1bOA\x1bOD") == ["UP", "LEFT"]


def test_parser_split_sequence_and_lone_esc():
    p = KeyParser()
    assert p.feed(b"\x1b") == [] and p.feed(b"[") == [] and p.feed(b"A") == ["UP"]
    assert p.feed(b"\x1b") == [] and p.flush() == ["ESC"]
    assert p.feed(b"\x1b[") == [] and p.flush() == [] and p.buf == b""        # 끊긴 제어열은 버린다


def test_parser_chars_and_unknown_sequences():
    p = KeyParser()
    assert p.feed(b"1wW m\x03") == ["1", "w", "W", "SPACE", "m", "CTRL_C"]
    assert p.feed(b"\x1b[1~q\x1b[15~") == ["q"]                               # Home, F5 는 무시
    assert p.feed(b"\x1bx") == ["ESC", "x"]


# ---------------------------------------------------------------- 누름 판정·속도
def test_hold_tap_then_release():
    h = Hold(0.6, 0.2)
    h.press("UP", 0.0)
    assert h.active(0.5) == "UP" and h.active(0.61) is None


def test_hold_repeat_then_release():
    h = Hold(0.6, 0.2)
    for t in (0.0, 0.5, 0.55, 0.60):
        h.press("UP", t)
    assert h.active(0.79) == "UP" and h.active(0.81) is None
    h.press("UP", 1.0)
    h.press("LEFT", 1.05)                                                     # 다른 키는 새로 시작
    assert h.active(1.5) == "LEFT" and h.repeats == 0


def test_ramp_and_caps():
    assert abs(ramp(0.0, 0.5, 1.0, 0.1) - 0.1) < 1e-9
    assert abs(ramp(0.5, 0.0, 1.0, 0.1) - 0.3) < 1e-9                         # 감속은 두 배
    assert ramp(0.05, 0.0, 1.0, 0.1) == 0.0
    assert target_velocity("UP", 1, 0.6, 3.0) == (0.12, 0.0)
    assert target_velocity("LEFT", 5, 9.0, 9.0) == (0.0, 3.0)                 # API 한도로 자른다
    assert target_velocity("RIGHT", 2, 0.6, 3.0)[1] < 0 and target_velocity(None, 3, 0.6, 3.0) == (0.0, 0.0)


def test_drive_allowed_table():
    assert drive_allowed("practice", "idle", 0)[0]
    assert not drive_allowed("mission", "start", 0)[0]
    assert not drive_allowed("", "", 0)[0] and drive_allowed("", "", 0, state_guard=False)[0]
    assert not drive_allowed("practice", "idle", 1)[0]


def test_double_press():
    d = DoublePress(1.0)
    assert not d.press(0.0) and d.press(0.8)
    assert not d.press(1.0) and not d.press(2.5) and d.press(3.0)


# ---------------------------------------------------------------- 텔레옵 본체
def test_unknown_mode_sends_only_zero():
    t, ros, _ = make(FakeRos(mode="", cmd=""))
    run(t, 0.0, 1.0, key="UP")
    assert ros.cmds and all(c == (0.0, 0.0) for c in ros.cmds)


def test_hold_drives_and_release_stops():
    t, ros, _ = make()
    end = run(t, 0.0, 1.5, key="UP")
    assert abs(ros.cmds[-1][0] - 0.12) < 1e-9 and ros.cmds[-1][1] == 0.0
    assert max(v for v, _ in ros.cmds) <= 0.12 + 1e-9
    n = len(ros.cmds)
    run(t, end, end + 0.6)                                                    # 손을 뗐다
    assert ros.cmds[-1] == (0.0, 0.0) and len(ros.cmds) > n
    stop_at = next(i for i in range(n, len(ros.cmds)) if ros.cmds[i] == (0.0, 0.0))
    assert (stop_at - n) * 0.1 <= 0.5                                         # 0.5초 안에 0


def test_cmd_rate_is_10hz():
    t, ros, _ = make()
    run(t, 0.0, 2.0)
    assert 19 <= len(ros.cmds) <= 21


def test_level5_capped_and_left_is_positive():
    t, ros, _ = make(max_lin=5.0, max_ang=50.0)
    t.on_key("5", 0.0)
    run(t, 0.0, 2.0, key="LEFT")
    assert abs(ros.cmds[-1][1] - 3.0) < 1e-9 and max(abs(w) for _, w in ros.cmds) <= 3.0
    run(t, 2.0, 4.0, key="UP")
    assert abs(ros.cmds[-1][0] - 0.6) < 1e-9


def test_space_stops_immediately():
    t, ros, _ = make()
    end = run(t, 0.0, 1.0, key="UP")
    t.on_key("SPACE", end)
    assert ros.cmds[-1] == (0.0, 0.0) and t.v == 0.0
    assert ros.events[-1]["type"] == "stop"


def test_other_publisher_stops_sending_then_quits():
    t, ros, said = make()
    end = run(t, 0.0, 1.0, key="UP")
    ros.others, n = 1, len(ros.cmds)
    end = run(t, end, end + 2.0, key="UP")
    assert len(ros.cmds) == n and t.quit is None                              # 0 도 보내지 않는다
    run(t, end, end + 1.5, key="UP")
    assert t.quit_kind == OTHER_PUBLISHER and not t.send_zero_on_exit() and len(ros.cmds) == n
    assert any("송신자" in s for s in said)


def test_other_publisher_gone_resumes():
    t, ros, _ = make()
    ros.others = 1
    end = run(t, 0.0, 1.0)
    ros.others = 0
    run(t, end, end + 1.0, key="UP")
    assert t.quit is None and ros.cmds[-1][0] > 0


def test_mission_mode_quits_without_sending():
    t, ros, _ = make(FakeRos(mode="mission", cmd="start"))
    run(t, 0.0, 0.5, key="UP")
    assert t.quit_kind == MISSION_MODE and ros.cmds == [] and not t.send_zero_on_exit()


def test_idle_exit_and_user_quit():
    t, ros, _ = make(idle_exit_s=120.0)
    run(t, 0.0, 119.0, dt=0.1)
    assert t.quit is None
    run(t, 119.0, 121.5, dt=0.1)
    assert t.quit and t.send_zero_on_exit()
    t2, _, _ = make()
    t2.on_key("q", 0.0)
    assert t2.quit and t2.send_zero_on_exit()


def test_gimbal_coalesces_and_clamps():
    t, ros, _ = make()
    t.on_key("d", 0.0)
    t.tick(0.0)
    assert [(p, tl) for p, tl, _ in ros.gimbal] == [(5, 0)]
    for k in ("d", "d", "w"):                                                 # 응답 전에 쌓인 입력
        t.on_key(k, 0.1)
        t.tick(0.1)
    assert len(ros.gimbal) == 1
    ros.gimbal[0][2]("ok")
    t.tick(0.2)
    assert [(p, tl) for p, tl, _ in ros.gimbal] == [(5, 0), (15, 5)]          # 마지막 목표 하나로 합친다
    assert ros.events[-1] == {"type": "gimbal", "pan": 5, "tilt": 0, "resp": "ok"}
    for _ in range(10):
        t.on_key("D", 0.3)
    assert t.pan == 100
    t.on_key("c", 0.3)
    assert (t.pan, t.tilt) == (0, 0)


def test_gimbal_timeout_and_missing_service():
    t, ros, said = make()
    t.on_key("a", 0.0)
    t.tick(0.0)
    t.on_key("a", 1.0)
    t.tick(1.0)
    assert len(ros.gimbal) == 1
    t.tick(3.5)                                                               # 3초 넘게 응답 없음 → 다음 호출 허용
    assert len(ros.gimbal) == 2 and ros.gimbal[1][0] == -10
    t2, ros2, said2 = make()
    ros2.has_gimbal = False
    t2.on_key("w", 0.0)
    t2.tick(0.0)
    assert t2.gimbal_resp == "서비스 없음" and any("gimbal" in s for s in said2)


def test_gimbal_blocked_when_not_allowed():
    t, ros, _ = make(FakeRos(mode="", cmd=""))
    t.on_key("d", 0.0)
    t.tick(0.0)
    assert ros.gimbal == []


def test_fire_needs_flag_and_double_press():
    t, ros, said = make()
    t.on_key("f", 0.0)
    t.on_key("f", 0.2)
    assert ros.fire == [] and any("--allow-fire" in s for s in said)
    t, ros, _ = make(allow_fire=True)
    t.on_key("f", 0.0)
    assert ros.fire == []
    t.on_key("f", 1.5)                                                        # 1초를 넘겼다 → 다시 준비
    assert ros.fire == []
    t.on_key("f", 2.0)
    assert len(ros.fire) == 1
    ros.fire[0]("reloading", 4200.0)
    assert "reloading" in t.fire_resp and ros.events[-1]["type"] == "fire"


def test_fire_blocked_in_unknown_mode():
    t, ros, _ = make(FakeRos(mode="", cmd=""), allow_fire=True)
    t.on_key("f", 0.0)
    t.on_key("f", 0.3)
    assert ros.fire == []


def test_marker_and_level_events():
    t, ros, _ = make()
    t.on_key("m", 0.0)
    t.on_key("M", 0.1)
    t.on_key("3", 0.2)
    assert [e for e in ros.events if e["type"] == "marker"] == [{"type": "marker", "n": 1}, {"type": "marker", "n": 2}]
    assert t.level == 3 and "L3" in t.status() and "M2" in t.status()


def test_bytes_path_with_lone_esc_quits():
    t, _, _ = make()
    t.on_bytes(b"\x1b", 0.0)
    t.tick(0.01)
    assert t.quit is None
    t.tick(0.1)
    assert t.quit


# ---------------------------------------------------------------- 영상·구간
def test_image_to_bgr():
    img = extract_images.image_to_bgr(1, 2, "rgb8", 8, bytes([1, 2, 3, 4, 5, 6, 0, 0]))     # step 에 여백 2바이트
    assert img.shape == (1, 2, 3) and img[0, 0].tolist() == [3, 2, 1] and img[0, 1].tolist() == [6, 5, 4]
    assert extract_images.image_to_bgr(1, 1, "bgr8", 0, bytes([7, 8, 9]))[0, 0].tolist() == [7, 8, 9]
    assert extract_images.image_to_bgr(1, 1, "mono8", 1, bytes([5]))[0, 0].tolist() == [5, 5, 5]
    for bad in ((2, 2, "bgr8", 6, bytes(6)), (1, 1, "yuv422", 2, bytes(2))):
        try:
            extract_images.image_to_bgr(*bad)
            raise AssertionError("예외가 나야 한다")
        except ValueError:
            pass


def test_segments_and_pick():
    assert extract_images.segments([5.0, 2.0, 99.0], 0.0, 10.0) == [(0, 0.0, 2.0), (1, 2.0, 5.0), (2, 5.0, 10.0)]
    assert extract_images.segments([], 0.0, 10.0) == [(0, 0.0, 10.0)]
    stamps = [i * 0.1 for i in range(30)]
    assert extract_images.pick(stamps, 2.0) == [0, 5, 10, 15, 20, 25] and len(extract_images.pick(stamps, 0)) == 30


# ---------------------------------------------------------------- bag 점검·비밀값 검사
META = """rosbag2_bagfile_information:
  storage_identifier: mcap
  duration:
    nanoseconds: 10000000000
  starting_time:
    nanoseconds_since_epoch: 1000000000000
  message_count: 700
  topics_with_message_count:
%s  relative_file_paths:
    - data_0.mcap
"""


def _session(topics):
    d = tempfile.mkdtemp()
    os.makedirs(os.path.join(d, "bag"))
    rows = "".join(f"    - topic_metadata:\n        name: {n}\n        type: x/msg/T\n      message_count: {c}\n" for n, c in topics.items())
    with open(os.path.join(d, "bag", "metadata.yaml"), "w") as f:
        f.write(META % rows)
    with open(os.path.join(d, "bag", "data_0.mcap"), "wb") as f:
        f.write(b"\0" * 5000)
    return d


def _quiet(fn, *a):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = fn(*a)
    return rc, buf.getvalue()


def test_bag_check_ok_and_missing():
    good = {"/odom": 200, "/image_raw": 100, "/scan": 100, "/cmd_vel": 100, "/imu/data": 500, "/tf": 300, "/tf_static": 1,
            "/mil/state": 20, "/teleop/events": 3}
    d = _session(good)
    with open(os.path.join(d, "teleop_events.jsonl"), "w") as f:
        f.write('{"t": 1003.5, "type": "marker", "n": 1}\n{"t": 1004, "type": "level", "level": 2}\n')
    rc, out = _quiet(bag_check.main, [d])
    assert rc == 0 and "정상" in out and "M1 +3.5초" in out
    rc, out = _quiet(bag_check.main, [_session({k: v for k, v in good.items() if k != "/scan"})])
    assert rc == 1 and "실패: /scan 없음" in out
    rc, out = _quiet(bag_check.main, [_session(dict(good, **{"/odom": 30}))])
    assert rc == 0 and "/odom 주기가 낮다" in out
    rc, out = _quiet(bag_check.main, [_session(dict({k: v for k, v in good.items() if k != "/scan"}, **{"/ldlidar/scan": 100})),
                                      "--scan-topic", "/ldlidar/scan"])
    assert rc == 0
    assert _quiet(bag_check.main, [tempfile.mkdtemp()])[0] == 2


def test_scan_secret_finds_without_printing_value():
    old_env, old_chunk = os.environ.get("MAICON_CODE"), scan_secret.CHUNK
    os.environ["MAICON_CODE"] = FAKE_CODE
    scan_secret.CHUNK = 16                                                    # 블록 경계에 걸치게
    try:
        d = _session({"/odom": 1})
        assert _quiet(scan_secret.main, [d])[0] == 0 and not os.path.exists(os.path.join(d, "DO_NOT_SHARE"))
        with open(os.path.join(d, "bag", "data_0.mcap"), "wb") as f:
            f.write(b"\0" * 12 + FAKE_CODE.encode() + b"\0" * 40)             # 12~20 바이트: 경계에 걸친다
        with open(os.path.join(d, "notes.txt"), "wb") as f:
            f.write(b"abc" + FAKE_CODE.encode("utf-16-le"))                   # 파일 끝, UTF-16
        rc, out = _quiet(scan_secret.main, [d])
        assert rc == 1 and FAKE_CODE not in out and FAKE_CODE.lower() not in out.lower()
        assert "data_0.mcap (1곳)" in out and "notes.txt (1곳)" in out
        assert os.path.exists(os.path.join(d, "DO_NOT_SHARE"))
        del os.environ["MAICON_CODE"]
        assert _quiet(scan_secret.main, [d])[0] == 2
    finally:
        scan_secret.CHUNK = old_chunk
        if old_env is None:
            os.environ.pop("MAICON_CODE", None)
        else:
            os.environ["MAICON_CODE"] = old_env


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print("통과", name)
        except Exception as e:                                                # noqa: BLE001
            failed += 1
            print("실패", name, type(e).__name__, e)
    print(f"{len(tests) - failed}/{len(tests)} 통과")
    sys.exit(1 if failed else 0)
