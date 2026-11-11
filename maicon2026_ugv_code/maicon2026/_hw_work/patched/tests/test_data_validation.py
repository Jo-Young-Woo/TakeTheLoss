"""9장 데이터 유효성 검사(V1–V8)의 합격 기준. 시뮬·ROS·Gazebo 를 쓰지 않는다 — 가짜 시계와 transport 만 쓴다."""
import json
import math
import time
from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.hal.mil_api import CMD_STATES, FireModel, control_allowed
from maicon26.hal.ros_mil import RosMilTransport, image_message_problem
from maicon26.link.http import HttpServerLink
from maicon26.mission.manager import Agent
from maicon26.perception.camera import CameraModel
from maicon26.types import Detection
from maicon26.validate import (FrameWatch, OdomJumpDetector, ValidationLog, check_aligned_size, check_camera_matrix,
                               check_fisheye_D, check_left_ms, clean_issued_at, sanitize_command_text, sanitize_config,
                               sanitize_detections, unknown_match_fields, unknown_response)
from hw_fakes import FakeTransport, make_robot

FAKE_CODE = "K7M2Q9XA"                              # 가짜 임무코드(실제 코드는 어디에도 두지 않는다)


def _msg(h=480, w=640, enc="bgr8", step=None, n=None, color=None):
    ch = {"bgr8": 3, "mono8": 1, "bgra8": 4}[enc]
    step = w * ch if step is None else step
    n = h * step if n is None else n
    return SimpleNamespace(height=h, width=w, encoding=enc, step=step, data=bytes(n),
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)))


def _transport(robot):
    """RosMilTransport 의 수신 콜백만 시험한다 — rclpy 없이."""
    t = RosMilTransport.__new__(RosMilTransport)
    t.robot, t.cfg = robot, robot.cfg
    t._camera_calibration = None
    t._pending_camera = deque(maxlen=4)
    t._oak_last_decode = -1e9
    t._oak_compressed = False
    return t


def _scene(seed=1, h=480, w=640):
    return np.random.default_rng(seed).integers(0, 256, (h, w, 3), dtype=np.uint8)


# ====================================================================== 공통: 기록
def test_each_reason_warns_once_per_five_seconds_and_counts_every_drop():
    lines, clock = [], {"t": 0.0}
    log = ValidationLog({}, clock=lambda: clock["t"], warn=lines.append)
    for _ in range(1000):
        log.bad("fire", "left_ms_nonfinite", "x")
    assert len(lines) == 1 and log.counts["fire"]["left_ms_nonfinite"] == 1000
    log.bad("fire", "left_ms_negative")                                   # 다른 이유는 따로 센다
    assert len(lines) == 2
    clock["t"] = 5.1
    log.bad("fire", "left_ms_nonfinite")
    assert len(lines) == 3 and "999건 생략" in lines[-1]


def test_reasons_per_source_are_bounded_so_raw_text_cannot_grow_the_table():
    log = ValidationLog({}, warn=lambda m: None)
    for i in range(500):
        log.bad("fire", f"unknown:{i}")
    assert len(log.counts["fire"]) <= 33 and log.total("fire") == 500


def test_status_file_has_the_documented_shape_and_is_written_from_its_own_thread(tmp_path):
    path = tmp_path / "logs" / "validation_status.json"
    log = ValidationLog({}, warn=lambda m: None, status_path=path, status_interval_s=0.05)
    log.bad("image", "camera_frozen", "20 frames")
    log.good("odom")
    log.start_writer()
    try:
        deadline = time.time() + 3.0
        while not path.exists() and time.time() < deadline:
            time.sleep(0.02)
        assert path.exists()
    finally:
        log.stop_writer()
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["schema"] == 1 and set(data) >= {"written_wall", "uptime_s", "warnings_emitted", "sources"}
    img = data["sources"]["image"]
    assert img["dropped"] == 1 and img["reasons"] == {"camera_frozen": 1} and img["last_reason"] == "camera_frozen"
    assert img["last_detail"] == "20 frames" and img["last_invalid_wall"] > 0 and img["last_valid_wall"] is None
    odom = data["sources"]["odom"]
    assert odom["dropped"] == 0 and odom["valid_count"] == 1 and odom["last_valid_age_s"] >= 0.0
    assert not list(path.parent.glob("*.tmp"))                             # 임시 파일을 남기지 않는다


def test_status_file_failure_is_a_warning_not_an_exception(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("x")
    log = ValidationLog({}, warn=lambda m: None, status_path=blocker / "sub" / "s.json")   # 폴더를 만들 수 없는 경로
    assert log.write_status() is False and log.write_errors == 1


def test_absorb_merges_counts_from_a_pre_robot_log():
    a, b = ValidationLog({}, warn=lambda m: None), ValidationLog({}, warn=lambda m: None)
    a.bad("config", "camera.model", "x")
    b.bad("config", "camera.model", "y")
    b.bad("config", "aim.gimbal_settle_s")
    a.absorb(b)
    assert a.counts["config"] == {"camera.model": 2, "aim.gimbal_settle_s": 1}


# ====================================================================== V1 — left_ms
@pytest.mark.parametrize("bad,reason", [(float("nan"), "left_ms_nonfinite"), (float("inf"), "left_ms_nonfinite"),
                                        (-1, "left_ms_negative"), (1e9, "left_ms_too_large"), (15000.5, "left_ms_too_large"),
                                        ("12", "left_ms_type"), (True, "left_ms_type")])
def test_v1_bad_left_ms_is_dropped_and_the_default_period_is_used(bad, reason):
    log = ValidationLog({}, warn=lambda m: None)
    fire = FireModel(10, 5.0, 10.0, vlog=log)
    assert fire.on_response(100.0, "fired", bad) == "fired"
    assert fire.ready_at == pytest.approx(110.0) and math.isfinite(fire.cooldown_left(101.0))
    assert log.counts["fire"] == {reason: 1}


@pytest.mark.parametrize("good,expect", [(1500, 1.5), (7.0, 0.007), (15000, 15.0), (0, 10.0), (None, 10.0)])
def test_v1_normal_left_ms_behaves_exactly_as_before(good, expect):
    log = ValidationLog({}, warn=lambda m: None)
    fire = FireModel(10, 5.0, 10.0, vlog=log)
    fire.on_response(100.0, "fired", good)
    assert fire.ready_at == pytest.approx(100.0 + expect) and log.total() == 0


def test_v1_reloading_with_nan_does_not_poison_ready_at_and_the_robot_path_is_the_same():
    robot, tr, clk = make_robot("mission", "start", transport=FakeTransport([("reloading", float("nan")), ("fired", 1e9)]))
    robot.on_state(4, 4, 90.0)
    assert robot.laser_fire() is False                                    # reloading
    assert math.isfinite(robot.fire.ready_at)
    clk.t += 1.0
    assert robot.laser_fire() is True                                     # 1e9 는 버려지고 period_s 로
    assert robot.fire.cooldown_left(clk.t) == pytest.approx(robot.fire.period_s)
    assert robot.stats["invalid"]["fire"] == {"left_ms_nonfinite": 1, "left_ms_too_large": 1}


def test_v1_limit_comes_from_the_config():
    cfg = Config()
    cfg.hal.fire_left_ms_max = 2000.0
    robot, tr, clk = make_robot("mission", "start", cfg=cfg, transport=FakeTransport([("fired", 3000)]))
    robot.laser_fire()
    assert robot.stats["invalid"]["fire"] == {"left_ms_too_large": 1}


# ====================================================================== V2 — 응답 문자열
def test_v2_unknown_fire_and_gimbal_strings_are_counted_with_the_original_text_and_still_rejected():
    robot, tr, clk = make_robot("mission", "start", transport=FakeTransport([("FIRED-ish" + "x" * 60, None)], gimbal_reply="BUSY"))
    robot.on_state(4, 4, 90.0)
    assert robot.laser_fire() is False
    assert robot.fire.rejected["none"] == 1                               # 동작은 지금처럼 거절
    key = [k for k in robot.stats["invalid"]["fire"] if k.startswith("unknown:")]
    assert key == ["unknown:" + ("FIRED-ish" + "x" * 60)[:32]]             # 원문(대소문자 그대로) 앞 32자
    robot.set_pan_tilt(10.0, 0.0)
    assert robot.stats["gimbal_fail"] == 1 and robot.stats["invalid"]["gimbal"] == {"unknown:BUSY": 1}


def test_v2_known_strings_and_failed_calls_are_not_unknown():
    for kind, ok in (("fire", ("fired", "empty", "locked", "reloading")), ("gimbal", ("ok", "locked"))):
        for m in ok:
            assert unknown_response(kind, m) is None and unknown_response(kind, m.upper()) is None
        assert unknown_response(kind, None) is None                       # 호출 자체가 실패한 경우는 응답 문자열이 아니다
    assert unknown_response("fire", "ok") == "unknown:ok"                 # 짐벌의 'ok' 는 발사에서는 모르는 값


# ====================================================================== V3 — /mil/match
def test_v3_unknown_mode_is_not_practice_and_warns_once_in_five_seconds():
    robot, tr, clk = make_robot("practice", "idle")
    assert robot.fire.practice and robot.hit.practice
    lines = []
    robot.vlog._warn_fn = lines.append
    for _ in range(300):
        robot.on_match("practise", "idle")                                # 오타
    assert not robot.fire.practice and not robot.hit.practice             # 연습 모드로 보지 않는다
    assert robot.stats["invalid"]["match"] == {"unknown_mode:practise": 300}
    assert len([l for l in lines if "unknown_mode" in l]) == 1            # 5 s 안 반복 억제


def test_v3_control_decision_is_unchanged_for_unknown_values():
    for mode, cmd in (("weird", "start"), ("mission", "dance"), ("practice", "xyz"), ("", "")):
        robot, _, _ = make_robot("mission", "idle")
        robot.on_match(mode, cmd)
        assert robot.status().control == control_allowed(mode.strip().lower(), cmd.strip().lower())


def test_v3_known_values_and_empty_strings_are_silent():
    assert unknown_match_fields("practice", "idle", CMD_STATES) == []
    assert unknown_match_fields("MISSION", " Start ", CMD_STATES) == []
    assert unknown_match_fields("", "", CMD_STATES) == [] and unknown_match_fields(None, None, CMD_STATES) == []
    assert unknown_match_fields("race", "boost", CMD_STATES) == ["unknown_mode:race", "unknown_cmd:boost"]
    for cmd in ("idle", "standby", "start", "pause", "resume", "report", "stop"):
        assert cmd in CMD_STATES


# ====================================================================== V4 — /image_raw
@pytest.mark.parametrize("kw,reason", [
    (dict(w=320, h=240), "size_mismatch"),
    (dict(n=480 * 640 * 3 - 1), "short_buffer"),
    (dict(step=640 * 3 - 1), "bad_step"),
    (dict(h=0), "bad_dims"),
])
def test_v4_wrong_size_short_buffer_bad_step_and_dims_are_dropped_before_conversion(kw, reason):
    robot, tr, clk = make_robot()
    t = _transport(robot)
    assert image_message_problem(_msg(**kw), 640, 480) == reason
    t._on_image(_msg(**kw))
    assert robot.get_frame() is None and robot.stats["invalid"]["image"] == {reason: 1}
    assert "image" not in robot._received                                 # 버린 영상으로는 수신 시각을 갱신하지 않는다


def test_v4_a_well_formed_message_passes_and_reaches_the_robot():
    robot, tr, clk = make_robot()
    t = _transport(robot)
    for enc in ("bgr8", "mono8", "bgra8"):
        assert image_message_problem(_msg(enc=enc), 640, 480) is None
    t._on_image(_msg())
    assert robot.get_frame() is not None and robot.get_frame().width == 640 and not robot.stats.get("invalid")


def test_v4_message_check_never_raises_on_garbage():
    for m in (None, object(), SimpleNamespace(), SimpleNamespace(height="a", width=1, encoding="bgr8", step=1, data=b""),
              SimpleNamespace(height=1, width=1, encoding="bgr8", step=1, data=None)):
        assert image_message_problem(m, 640, 480) in ("bad_message", None)


def _feed(robot, clk, frames, hdr=lambda i: 0.0, dt=0.1):
    for i, img in enumerate(frames):
        robot.on_image(img, hdr(i))
        clk.t += dt


def test_v4_twenty_identical_frames_freeze_the_camera_and_stop_the_receive_time():
    robot, tr, clk = make_robot("mission", "standby")
    robot.on_state(4, 4, 90.0)
    robot.on_odom(0, 0, 0)
    robot.on_scan(-math.pi, math.pi / 2, [1, 1, 1, 1], 0, 8)
    robot.on_imu(0, 0, 0, 1)
    img = _scene()
    _feed(robot, clk, [img.copy() for _ in range(19)])
    assert robot.stats.get("invalid", {}).get("image", {}).get("camera_frozen", 0) == 0
    last_ok = robot._received["image"]
    _feed(robot, clk, [img.copy() for _ in range(20)])
    assert robot.stats["invalid"]["image"]["camera_frozen"] >= 19
    assert robot._received["image"] == last_ok                             # 갱신 없음
    assert "image" in robot.readiness()[1]                                 # 1장 준비 확인에서 '없음'
    _feed(robot, clk, [_scene(2)])                                         # 다른 영상이 오면 바로 회복
    assert robot._received["image"] > last_ok and "image" not in robot.readiness()[1]


def test_v4_a_repeated_header_stamp_freezes_too_even_when_pixels_differ():
    robot, tr, clk = make_robot()
    _feed(robot, clk, [_scene(i) for i in range(25)], hdr=lambda i: 7.0)
    assert robot.stats["invalid"]["image"]["camera_frozen"] >= 5


def test_v4_a_still_scene_with_sensor_noise_never_freezes_in_200_frames():
    robot, tr, clk = make_robot()
    rng = np.random.default_rng(7)
    base = _scene(3).astype(np.int16)
    frames = [np.clip(base + rng.integers(-2, 3, base.shape), 0, 255).astype(np.uint8) for _ in range(200)]
    _feed(robot, clk, frames)
    assert "camera_frozen" not in robot.stats.get("invalid", {}).get("image", {})
    assert robot.stats["frames"] >= 190                                    # 10 FPS 이하 제한은 그대로 지킨다(시험은 10 FPS 로 넣는다)


def test_v4_dark_or_white_frames_only_warn_and_are_kept():
    robot, tr, clk = make_robot()
    rng = np.random.default_rng(11)
    frames = [rng.integers(0, 3, (480, 640, 3), dtype=np.uint8) for _ in range(40)]     # 평균 ≈ 1, 잡음 있음(멈춤 아님)
    _feed(robot, clk, frames)
    assert robot.stats["invalid"]["image"]["camera_dark"] >= 1
    assert "camera_frozen" not in robot.stats["invalid"]["image"]
    assert robot.stats["frames"] == 40 and robot.get_frame() is not None   # 버리지 않는다
    white = [255 - f for f in frames[:35]]
    robot2, _, clk2 = make_robot()
    _feed(robot2, clk2, white)
    assert robot2.stats["invalid"]["image"]["camera_dark"] >= 1


def test_v4_frame_watch_costs_less_than_one_millisecond_per_frame():
    watch = FrameWatch()
    imgs = [_scene(i) for i in range(5)]
    times = []
    for i in range(200):
        t0 = time.perf_counter()
        watch.update(imgs[i % 5], float(i))
        times.append(time.perf_counter() - t0)
    assert float(np.median(times)) < 1e-3 and float(np.percentile(times, 95)) < 2e-3


def test_v4_frame_watch_needs_the_configured_number_of_frames_and_never_raises():
    w = FrameWatch(frozen_frames=5)
    img = _scene()
    flags = [w.update(img.copy(), None)[0] for _ in range(6)]
    assert flags == [False, False, False, False, True, True]
    assert w.update(None, None) == (False, False) and w.update("x", 1.0) == (False, False)


# ====================================================================== V5 — 검출 상자
def _d(box, conf=0.9, cls="BOX"):
    return Detection(cls, conf, box)


def test_v5_bad_boxes_are_dropped_and_straddling_boxes_are_clipped():
    good = _d((100.0, 100.0, 200.0, 220.0))
    kept, why = sanitize_detections([
        good,
        _d((float("nan"), 0, 10, 10)),                                    # 유한하지 않음
        _d((50, 50, 20, 80)),                                             # x 뒤집힘
        _d((50, 80, 90, 80)),                                             # 높이 0
        _d((700, 10, 760, 60)),                                           # 완전히 화면 밖
        _d((-30, 10, 100, 60)),                                           # 걸침 → 자름
        _d((10, 10, 12, 100)),                                            # 너비 2 < 4
        _d((10, 10, 60, 60), conf=1.5), _d((10, 10, 60, 60), conf=float("nan")), _d((10, 10, 60, 60), conf=-0.1),
    ], 640, 480)
    assert why == ["box_nonfinite", "box_inverted", "box_inverted", "box_outside", "box_too_small",
                   "conf_range", "conf_range", "conf_range"]
    assert kept[0] is good                                                # 정상 검출은 같은 객체
    assert kept[1].box == (0.0, 10.0, 100.0, 60.0) and len(kept) == 2


def test_v5_clipping_that_leaves_less_than_min_box_px_is_dropped():
    kept, why = sanitize_detections([_d((-50, 10, 2, 60))], 640, 480, 4.0)
    assert kept == [] and why == ["box_too_small"]
    kept, why = sanitize_detections([_d((-50, 10, 2, 60))], 640, 480, 1.0)
    assert len(kept) == 1 and kept[0].box == (0.0, 10.0, 2.0, 60.0)


def test_v5_never_raises_on_malformed_detections():
    kept, why = sanitize_detections([None, object(), SimpleNamespace(box=(1, 2, 3), conf=1), _d("abcd"), _d(None)], 640, 480)
    assert kept == [] and len(why) == 5


def test_v5_agent_applies_the_same_rule_to_pantilt_and_oak_and_counts_by_source():
    robot, _, _ = make_robot()
    me = SimpleNamespace(cfg=robot.cfg, robot=robot, stats={}, _log=lambda *a, **k: None)
    src = SimpleNamespace(width=640, height=480)
    dets = [_d((10, 10, 60, 60)), _d((float("nan"), 1, 2, 3)), _d((-10, 20, 70, 90))]
    for source in ("detections", "oak_detections"):
        out = Agent._clean_dets(me, dets, src, source)
        assert [d.box for d in out] == [(10, 10, 60, 60), (0.0, 20.0, 70.0, 90.0)]
        assert robot.stats["invalid"][source] == {"box_nonfinite": 1}
    me.cfg = None                                                         # 검사 자체가 깨져도 예외 없이 비운다(나쁜 상자를 넣지 않는다)
    assert Agent._clean_dets(me, dets, src, "detections") == [] and me.stats["det_sanitize_errors"] == 1


# ====================================================================== V6 — 서버 /order·/target
def _link(**kw):
    link = HttpServerLink("http://127.0.0.1:9", FAKE_CODE, "BLUE", 1, transport=lambda *a, **k: None, **kw)
    got = []
    link._emit = lambda name, msg: got.append((name, msg))
    return link, got


def test_v6_long_command_is_cut_and_control_characters_are_removed():
    link, got = _link(max_command_chars=500)
    text = "A\x00B\x07C\nD\tE" + "가" * 2000
    link._get_json = lambda kind, params=None, timeout=None: (200, {"accepted": True, "order": {
        "tactical_command": text, "issued_at": "2026-06-01 10:00:00"}})
    link._wait_loop("order")
    (name, msg), = got
    assert name == "order" and len(msg.text) == 500 and msg.text.startswith("ABC D E가")
    assert not any(ord(c) < 32 or ord(c) == 127 for c in msg.text)
    assert link.vlog.counts["server_order"] == {"command_truncated": 1, "command_control_chars": 1}
    assert msg.data["issued_at"] == "2026-06-01 10:00:00" and link._order_issued_at == "2026-06-01 10:00:00"


def test_v6_normal_command_and_issued_at_are_untouched():
    link, got = _link()
    link._get_json = lambda kind, params=None, timeout=None: (200, {"accepted": True, "order": {
        "tactical_command": "A 구역 먼저 정찰하고 M2 를 수행하라", "issued_at": "2026-06-01 10:00:00"}})
    link._wait_loop("order")
    assert got[0][1].text == "A 구역 먼저 정찰하고 M2 를 수행하라" and link.vlog.total() == 0


@pytest.mark.parametrize("junk", ["garbage", "2026-13-40 99:99:99", "", float("nan"), float("inf"), -5, 0, True, [], {}])
def test_v6_a_junk_issued_at_is_treated_as_missing(junk):
    assert clean_issued_at(junk) is None
    link, got = _link()
    link._get_json = lambda kind, params=None, timeout=None: (200, {"accepted": True, "order": {
        "tactical_command": "go", "issued_at": junk}})
    link._wait_loop("order")
    assert got[0][1].data["issued_at"] is None and link._order_issued_at is None
    assert link.vlog.counts["server_order"] == {"issued_at_invalid": 1}


def test_v6_numbers_and_the_documented_string_format_are_accepted_and_target_gets_the_same_rule():
    assert clean_issued_at("2026-06-01 10:04:00") == "2026-06-01 10:04:00" and clean_issued_at(1.7e9) == 1.7e9
    assert clean_issued_at(None) is None
    link, got = _link()
    link._get_json = lambda kind, params=None, timeout=None: (200, {"accepted": True, "order": {
        "objective_label": "A", "objective": {"x": 1}, "issued_at": "bad"}})
    link._wait_loop("target")
    assert got[0][1].data["issued_at"] is None and link._target_issued_at is None
    assert link.vlog.counts["server_target"] == {"issued_at_invalid": 1}


def test_v6_sanitize_command_text_contract():
    assert sanitize_command_text("a\r\nb", 10) == ("a  b", False, 0)
    assert sanitize_command_text("a\x1bb", 10) == ("ab", False, 1)
    assert sanitize_command_text("abcdef", 3) == ("abc", True, 0)
    assert sanitize_command_text(None, 3) == ("", False, 0)
    assert sanitize_command_text("x" * 5, 5) == ("xxxxx", False, 0)       # 길이가 딱 맞으면 자르지 않는다


# ====================================================================== V7 — odom 연속성
def test_v7_a_teleport_to_zero_is_one_odom_jump_and_the_value_is_passed_through():
    robot, tr, clk = make_robot()
    x = 0.0
    for _ in range(20):                                                   # 0.3 m/s 로 달린다 — 정상
        robot.on_odom(3.0 + x, 2.0, 0.5)
        x += 0.03
        clk.t += 0.1
    assert "odom" not in robot.stats.get("invalid", {})
    robot.on_odom(0.0, 0.0, 0.0)                                          # 드라이버 재시작
    assert robot.stats["invalid"]["odom"] == {"odom_jump": 1}
    assert (robot.get_odom().x, robot.get_odom().y, robot.get_odom().th) == (0.0, 0.0, 0.0)   # 값은 그대로
    clk.t += 0.1
    robot.on_odom(0.02, 0.0, 0.0)                                         # 튄 뒤 새 자리 기준으로 다시 정상
    assert robot.stats["invalid"]["odom"] == {"odom_jump": 1}


def test_v7_rotation_jump_and_the_exact_threshold():
    det = OdomJumpDetector(v_cap=0.6, w_cap=3.0)
    assert det.update(0.0, 0, 0, 0) is None
    assert det.update(0.1, 0.6 * 0.1 * 3 + 0.04, 0, 0) is None            # 상한 안
    assert det.update(0.2, 0.6 * 0.1 * 3 + 0.04 + 0.6 * 0.1 * 3 + 0.06, 0, 0) == "odom_jump"
    assert det.update(0.3, 0.0, 0.0, 0.0) == "odom_jump"                  # 이동 쪽
    d2 = OdomJumpDetector()
    d2.update(0.0, 0, 0, 0)
    assert d2.update(0.1, 0, 0, 3.0 * 0.1 * 3 + 0.2) == "odom_jump"       # 회전 쪽
    assert d2.update(0.2, 0, 0, 3.0 * 0.1 * 3 + 0.2 + 0.1) is None
    assert det.update(float("nan"), 0, 0, 0) is None                      # 유한하지 않은 입력은 기준으로 삼지 않는다


def test_v7_non_finite_odom_is_dropped_counted_and_keeps_the_last_value():
    robot, tr, clk = make_robot()
    robot.on_odom(1.0, 2.0, 0.3)
    t0 = robot._received["odom"]
    clk.t += 1.0
    for bad in ((float("nan"), 0, 0), (0, float("inf"), 0), (None, 0, 0), ("a", 0, 0)):
        robot.on_odom(*bad)
    assert robot.get_odom().x == 1.0 and robot._received["odom"] == t0
    assert robot.stats["invalid"]["odom"] == {"nonfinite": 4}


# ====================================================================== V8 — OAK·어안·설정
K_OK = (400.0, 0.0, 640.0, 0.0, 400.0, 360.0, 0.0, 0.0, 1.0)


def test_v8_oak_camera_info_with_a_bad_k_is_dropped_and_turns_the_feature_off():
    robot, tr, clk = make_robot()
    robot.on_oak_info(K_OK, 1280, 720)
    assert robot.get_oak_info() is not None
    for k, w, h, why in [((float("nan"),) + K_OK[1:], 1280, 720, "K_nonfinite"),
                         ((0.0,) + K_OK[1:], 1280, 720, "K_focal"),
                         ((400.0, 0.0, 2000.0, 0.0, 400.0, 360.0, 0.0, 0.0, 1.0), 1280, 720, "K_principal_x"),
                         ((400.0, 0.0, 640.0, 0.0, 400.0, -5.0, 0.0, 0.0, 1.0), 1280, 720, "K_principal_y"),
                         (K_OK[:8], 1280, 720, "K_malformed")]:
        robot.on_oak_info(K_OK, 1280, 720)
        robot.on_oak_info(k, w, h)
        assert robot.get_oak_info() is None, why                          # 이전 값도 지운다 → manager 가 OAK 관측을 쓰지 않는다
        assert robot.stats["invalid"]["oak_info"][why] >= 1


def test_v8_aligned_profile_needs_a_multiple_of_16_and_the_default_profile_does_not():
    cfg = Config()
    cfg.oak.profile = "rgbd"
    robot, _, _ = make_robot(cfg=cfg)
    robot.on_oak_info(K_OK, 1280, 721)
    assert robot.get_oak_info() is None and robot.stats["invalid"]["oak_info"] == {"size_not_multiple_of_16": 1}
    robot.on_oak_info(K_OK, 1280, 720)
    assert robot.get_oak_info() is not None
    plain, _, _ = make_robot()
    plain.on_oak_info(K_OK, 1281, 721)                                    # rgb_lowpower 는 정렬이 아니다
    assert plain.get_oak_info() is not None
    assert check_aligned_size(1280, 720) is None and check_aligned_size(1280, 721) == "size_not_multiple_of_16"
    assert check_aligned_size("a", 720) == "size_malformed"


def test_v8_oak_frames_follow_the_v4_rules_and_must_match_camera_info():
    robot, tr, clk = make_robot()
    robot.on_oak_info((300.0, 0.0, 160.0, 0.0, 300.0, 120.0, 0.0, 0.0, 1.0), 320, 240)
    robot.on_oak_frame(np.zeros((480, 640, 3), np.uint8), 0.0)            # CameraInfo 와 크기가 다르다
    assert robot.get_oak_frame() is None and robot.stats["invalid"]["oak_rgb"] == {"size_mismatch_info": 1}
    robot.on_oak_frame(np.zeros((240, 320), np.uint8), 0.0)               # 채널 없음
    assert robot.stats["invalid"]["oak_rgb"]["bad_dims"] == 1 and robot.stats["oak_frame_invalid"] == 1
    rng = np.random.default_rng(5)
    for i in range(30):
        robot.on_oak_frame(rng.integers(0, 256, (240, 320, 3), dtype=np.uint8), float(i + 1))
    assert robot.get_oak_frame() is not None
    same = rng.integers(0, 256, (240, 320, 3), dtype=np.uint8)
    for _ in range(25):
        robot.on_oak_frame(same.copy(), 0.0)
    assert robot.stats["invalid"]["oak_rgb"]["camera_frozen"] >= 5


def test_v8_oak_message_checks_run_before_decoding_and_a_bad_camera_info_message_is_survivable():
    robot, tr, clk = make_robot()
    t = _transport(robot)
    t._on_oak_rgb(_msg(h=720, w=1280, n=100))                             # 짧은 버퍼
    assert robot.stats["invalid"]["oak_rgb"] == {"short_buffer": 1}
    robot.on_oak_info(K_OK, 1280, 720)
    t._on_oak_info(SimpleNamespace(width=0, height=0, p=[], k=[]))        # 쓸 수 없는 CameraInfo
    assert robot.get_oak_info() is None and robot.stats["invalid"]["oak_info"] == {"camera_info_unusable": 1}


def test_v8_fisheye_calibration_checks_use_the_shared_functions():
    assert check_camera_matrix(K_OK, 1280, 720) is None and check_camera_matrix(list(K_OK)) is None
    assert check_camera_matrix(None) == "K_malformed" and check_camera_matrix("abc") == "K_malformed"
    assert check_fisheye_D((0.1, 0.0, 0.0, 0.0)) is None and check_fisheye_D((0.1, 0.0)) == "D_count"
    assert check_fisheye_D((0.1, float("nan"), 0.0, 0.0)) == "D_nonfinite" and check_fisheye_D(5) == "D_count"
    cfg = Config().camera
    cfg.model, cfg.D = "fisheye", (0.0, 0.0, 0.0, 0.0)
    cfg.K = (300.0, 0.0, 900.0, 0.0, 300.0, 240.0, 0.0, 0.0, 1.0)         # 주점이 영상(640 폭) 밖
    cm = CameraModel(cfg)
    assert cm.model == "pinhole" and "K_principal_x" in cm.fallback_reason


def test_v8_default_config_is_not_changed_by_the_sanitizer():
    cfg = Config()
    before = json.dumps({k: getattr(cfg, k).__dict__ for k in ("camera", "oak_det", "aim", "oak")}, default=str, sort_keys=True)
    assert sanitize_config(cfg, root=".") == []
    assert before == json.dumps({k: getattr(cfg, k).__dict__ for k in ("camera", "oak_det", "aim", "oak")}, default=str, sort_keys=True)


@pytest.mark.parametrize("section,name,bad,expect", [
    ("camera", "model", "plumb_bob", "pinhole"),
    ("camera", "model", 3, "pinhole"),
    ("camera", "usable_hfov_deg", 0.0, None),
    ("camera", "usable_hfov_deg", 181.0, None),
    ("camera", "usable_hfov_deg", float("nan"), None),
    ("oak_det", "max_hz", 0.0, 3.0),
    ("oak_det", "max_hz", -2.0, 3.0),
    ("oak_det", "max_hz", float("nan"), 3.0),
    ("oak_det", "vote_window", 0, None),
    ("oak_det", "vote_window", 2.5, None),
    ("oak_det", "save_max", 99, 10),
    ("oak_det", "save_max", -4, 0),
    ("oak_det", "save_max", "x", 8),
    ("aim", "gimbal_settle_s", -0.1, 0.0),
    ("aim", "gimbal_settle_s", float("nan"), 0.0),
])
def test_v8_bad_new_config_keys_warn_and_fall_back(section, name, bad, expect):
    cfg = Config()
    setattr(getattr(cfg, section), name, bad)
    log = ValidationLog({}, warn=lambda m: None)
    changed = sanitize_config(cfg, log)
    assert changed == [f"{section}.{name}"] and getattr(getattr(cfg, section), name) == expect
    assert log.counts["config"] == {f"{section}.{name}": 1}


@pytest.mark.parametrize("section,name,good", [("camera", "model", "fisheye"), ("camera", "usable_hfov_deg", 180.0),
                                               ("camera", "usable_hfov_deg", 95.5), ("oak_det", "max_hz", 0.5),
                                               ("oak_det", "vote_window", 1), ("oak_det", "vote_window", 16),
                                               ("oak_det", "save_max", 0), ("oak_det", "save_max", 10),
                                               ("aim", "gimbal_settle_s", 0.0), ("aim", "gimbal_settle_s", 0.7)])
def test_v8_good_new_config_values_are_left_alone(section, name, good):
    cfg = Config()
    setattr(getattr(cfg, section), name, good)
    assert sanitize_config(cfg) == [] and getattr(getattr(cfg, section), name) == good


def test_v8_missing_oak_parameter_file_falls_back_to_the_default_profile(tmp_path):
    cfg = Config()
    cfg.oak.profile = "nope"
    root = tmp_path / "repo"
    (root / "config" / "oak").mkdir(parents=True)
    (root / "config" / "oak" / "rgb_lowpower.yaml").write_text("a: 1\n")
    log = ValidationLog({}, warn=lambda m: None)
    assert sanitize_config(cfg, log, root=root) == ["oak.profile"] and cfg.oak.profile == "rgb_lowpower"
    cfg2 = Config()
    cfg2.oak.profile = "nope"
    assert sanitize_config(cfg2, ValidationLog({}, warn=lambda m: None), root=tmp_path / "empty") == ["oak.profile"]
    assert cfg2.oak.profile == "nope"                                     # 바꿀 기본 파일도 없으면 경고만(OAK 기동만 건너뛴다)


def test_v8_sanitizer_never_raises_even_for_a_fake_config():
    assert sanitize_config(SimpleNamespace()) == []
    assert sanitize_config(None) == []
    assert sanitize_config(SimpleNamespace(camera=SimpleNamespace(model="x"))) == ["camera.model"]


# ====================================================================== 공통 — 콜백 1000번
def test_a_thousand_bad_inputs_into_every_callback_raise_nothing_and_warn_at_most_once_per_reason():
    robot, tr, clk = make_robot()
    t = _transport(robot)
    lines = []
    robot.vlog._warn_fn = lines.append
    rng = np.random.default_rng(3)
    junk = [None, "x", float("nan"), float("inf"), -1, 1e30, [], {}, object(), b"\x00", (1, 2), True]
    for i in range(1000):
        a, b, c = (junk[int(i + k) % len(junk)] for k in (0, 5, 7))
        robot.on_match(a, b)
        robot.on_state(a, b, c)
        robot.on_odom(a, b, c)
        robot.on_imu(a, b, c, a)
        robot.on_scan(a, b, [a, b], c, a)
        robot.on_image(a if i % 3 else rng.integers(0, 255, (4, 4), dtype=np.uint8), c)
        robot.on_oak_frame(a, b)
        robot.on_oak_info(a, b, c)
        t._on_image(a if i % 2 else _msg(w=int(i % 7)))
        t._on_oak_rgb(a if i % 2 else _msg(w=int(i % 5)))
        robot.fire.on_response(1.0, a if isinstance(a, str) else None, b)
        clk.t += 0.001
    per_reason = {}
    for line in lines:
        per_reason[line] = per_reason.get(line, 0) + 1
    assert robot.vlog.total() > 1000                                       # 많이 들어왔고 모두 세었다
    keys = {(s, r) for s, per in robot.stats["invalid"].items() for r in per}
    assert len(lines) <= len(keys) and all(n == 1 for n in per_reason.values())   # 이유당 5 s 에 1줄 이하(시험은 1 s 안에 끝난다)


def test_stats_invalid_is_json_friendly_so_it_can_go_into_the_final_report():
    robot, _, _ = make_robot()
    robot.on_match("race", "boost")
    robot.fire.on_response(1.0, "fired", float("nan"))
    json.dumps(robot.stats["invalid"], allow_nan=False)
