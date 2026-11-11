"""6장 6-1·6-2: OAK 드라이버 기동·파라미터 파일·토픽 확인. ROS·depthai·시뮬 없이 가짜 프로세스·가짜 그래프로 본다."""
import importlib.util
import json
import os
import signal
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from maicon26 import oak_launch
from maicon26.config import Config, OakCfg
from maicon26.hal.ros_mil import RosMilTransport
from maicon26.hal.wingbot import WingbotRobot
from hw_fakes import FakeTransport, make_robot

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import capture_hw_baseline as cap  # noqa: E402

FIX = ROOT / "tests" / "fixtures" / "hw_baseline"


# ====================================================================== team_node
def _load_team_node():
    spec = importlib.util.spec_from_file_location("oak_team_node_under_test", ROOT / "team" / "team_node.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _run_team(env_pairs, popen=None, argv_extra=()):
    mod = _load_team_node()
    saved_env, saved_argv = dict(os.environ), list(sys.argv)
    # mod.subprocess / mod.runpy 는 표준 라이브러리 모듈 그 자체다 — 대입하면 프로세스 전체에 남아 뒤따르는 시험을 깨뜨린다. 끝나면 되돌린다.
    real_popen, real_run_path = mod.subprocess.Popen, mod.runpy.run_path
    stops = []
    try:
        for k in list(os.environ):
            if k.startswith("MAICON_") or k == "TEAM_LAUNCH_NAV":
                del os.environ[k]
        os.environ.update({"MAICON_SERVER_URL": "http://127.0.0.1:9", **env_pairs})
        captured = []
        mod.runpy.run_path = lambda *a, **k: captured.append(list(sys.argv))
        if popen is not None:
            mod.subprocess.Popen = popen
        sys.argv = ["team_node.py", *argv_extra]
        real_stop, real_watch = oak_launch.stop_process_group, oak_launch.watch_early_exit
        oak_launch.stop_process_group = lambda proc, **k: stops.append(proc)
        oak_launch.watch_early_exit = lambda *a, **k: False                 # 감시 스레드는 아래에서 따로 시험한다
        try:
            rc = mod.main()
        finally:
            oak_launch.stop_process_group, oak_launch.watch_early_exit = real_stop, real_watch
        return rc, (captured[-1] if captured else None), stops
    finally:
        mod.subprocess.Popen, mod.runpy.run_path = real_popen, real_run_path
        os.environ.clear()
        os.environ.update(saved_env)
        sys.argv = saved_argv


class FakeProc:
    pid = 4242

    def __init__(self, rc=None):
        self.rc = rc

    def poll(self):
        return self.rc


def test_without_maicon_oak_nothing_is_launched_and_argv_equals_the_saved_baseline(monkeypatch):
    import subprocess

    def no_popen(*a, **k):
        raise AssertionError("MAICON_OAK 가 없는데 프로세스를 띄웠다")
    monkeypatch.setattr(subprocess, "Popen", no_popen)
    now = json.loads(json.dumps(cap._jsonable(cap.collect_team_argv())))
    saved = json.loads((FIX / "team_argv.json").read_text(encoding="utf-8"))
    assert now == saved


@pytest.mark.parametrize("value", ["", "0", "true", "yes"])
def test_only_the_literal_one_turns_the_oak_launch_on(value):
    calls = []
    rc, argv, stops = _run_team({"MAICON_OAK": value} if value else {}, popen=lambda *a, **k: calls.append((a, k)) or FakeProc())
    assert rc == 0 and calls == [] and stops == []


def test_maicon_oak_1_launches_driver_with_params_file_and_telemetry_off():
    calls = []

    def popen(cmd, **kw):
        calls.append((cmd, kw))
        return FakeProc()
    rc, argv, stops = _run_team({"MAICON_OAK": "1"}, popen=popen)
    assert rc == 0 and argv is not None                                        # run_real 은 그대로 이어서 불린다
    (cmd, kw), = calls
    assert cmd[:4] == ["ros2", "launch", "depthai_ros_driver", "camera.launch.py"]
    assert cmd[-1] == f"params_file:={(ROOT / 'config' / 'oak' / 'rgb_lowpower.yaml').resolve()}"
    assert kw["env"]["DEPTHAI_TELEMETRY"] == "0"
    assert kw["start_new_session"] is (os.name == "posix")                      # 새 프로세스 그룹
    assert len(stops) == 1                                                      # 끝날 때 정리한다


def test_launch_failure_is_only_a_warning_and_the_run_continues(capsys):
    def popen(*a, **k):
        raise FileNotFoundError("ros2")
    rc, argv, stops = _run_team({"MAICON_OAK": "1"}, popen=popen)
    assert rc == 0 and argv is not None and stops == []
    assert "WARN OAK 드라이버 기동 실패" in capsys.readouterr().out


def test_bad_profile_is_only_a_warning(monkeypatch, capsys):
    from maicon26 import launch_profile
    monkeypatch.setattr(launch_profile, "oak_launch_spec", lambda *a, **k: (_ for _ in ()).throw(ValueError("oak.profile 은 ...")))
    calls = []
    rc, argv, stops = _run_team({"MAICON_OAK": "1"}, popen=lambda *a, **k: calls.append(1) or FakeProc())
    assert rc == 0 and calls == []
    assert "OAK 드라이버 명령을 만들지 못했다" in capsys.readouterr().out


# ====================================================================== 조기 종료 감시·정리
class Clk:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


def test_early_exit_is_a_warning_and_a_running_process_is_not():
    clk, msgs = Clk(), []
    assert oak_launch.watch_early_exit(FakeProc(rc=3), ["ros2", "launch"], msgs.append, 5.0, clk.sleep, clk) is True
    assert msgs and "WARN" in msgs[0] and "종료 코드 3" in msgs[0]
    clk2, msgs2 = Clk(), []
    assert oak_launch.watch_early_exit(FakeProc(rc=None), ["x"], msgs2.append, 5.0, clk2.sleep, clk2) is False
    assert msgs2 == [] and clk2.t >= 5.0


def test_watcher_never_raises():
    class Broken:
        def poll(self):
            raise RuntimeError("boom")
    clk, msgs = Clk(), []
    assert oak_launch.watch_early_exit(Broken(), ["x"], msgs.append, 5.0, clk.sleep, clk) is False
    assert any("WARN" in m for m in msgs)


class KillProc:
    pid = 999

    def __init__(self, dies_on_int=True, alive=True):
        self.alive, self.dies_on_int, self.signals = alive, dies_on_int, []

    def poll(self):
        return None if self.alive else 0

    def wait(self, timeout=None):
        if self.alive:
            raise TimeoutError("still running")
        return 0


@pytest.mark.skipif(os.name != "posix", reason="프로세스 그룹은 POSIX")
def test_stop_sends_sigint_to_the_group_first_then_sigkill_only_if_needed(monkeypatch):
    sent = []
    monkeypatch.setattr(os, "getpgid", lambda pid: 777)

    def killpg(g, sig):
        sent.append((g, sig))
        if sig == signal.SIGINT and p.dies_on_int:
            p.alive = False
        if sig == signal.SIGKILL:
            p.alive = False
    monkeypatch.setattr(os, "killpg", killpg)
    p = KillProc(dies_on_int=True)
    oak_launch.stop_process_group(p, 0.01, 0.01, lambda m: None)
    assert sent == [(777, signal.SIGINT)]
    sent.clear()
    p = KillProc(dies_on_int=False)
    oak_launch.stop_process_group(p, 0.01, 0.01, lambda m: None)
    assert sent == [(777, signal.SIGINT), (777, signal.SIGKILL)]


def test_stop_does_nothing_for_none_or_finished_process(monkeypatch):
    monkeypatch.setattr(os, "killpg", lambda *a: pytest.fail("신호를 보내면 안 된다"), raising=False)
    oak_launch.stop_process_group(None)
    oak_launch.stop_process_group(KillProc(alive=False))


# ====================================================================== 명령·환경
def test_launch_command_parts_and_profile_names():
    oc = OakCfg(launch_args=("camera_model:=OAK-D-LITE",), profile="rgbd", params_file_arg="params_file")
    cmd = oak_launch.launch_command(oc, ROOT)
    assert cmd[:4] == ["ros2", "launch", "depthai_ros_driver", "camera.launch.py"] and cmd[4] == "camera_model:=OAK-D-LITE"
    assert cmd[5] == f"params_file:={(ROOT / 'config' / 'oak' / 'rgbd.yaml').resolve()}"
    with pytest.raises(ValueError):
        oak_launch.params_path(OakCfg(profile="nope"), ROOT)
    with pytest.raises(FileNotFoundError):
        oak_launch.params_path(OakCfg(profile="rgbd"), ROOT / "does_not_exist")


def test_child_env_adds_telemetry_off_without_touching_the_parent_env():
    base = {"A": "1"}
    env = oak_launch.child_env(base)
    assert env == {"A": "1", "DEPTHAI_TELEMETRY": "0"} and base == {"A": "1"}


def test_default_oak_profile_is_the_low_power_one():
    assert Config().oak.profile == "rgb_lowpower"


# ====================================================================== 파라미터 파일
def _params(name):
    doc = yaml.safe_load((ROOT / "config" / "oak" / f"{name}.yaml").read_text(encoding="utf-8"))
    return doc["/oak"]["ros__parameters"]


def _out_size(p):
    base = {"1080": (1920, 1080), "720": (1280, 720), "800": (1280, 800), "4000x3000": (4056, 3040)}[str(p["rgb"]["i_resolution"])]
    if p["rgb"].get("i_set_isp_scale"):
        n, d = p["rgb"]["i_isp_num"], p["rgb"]["i_isp_den"]
        return base[0] * n // d, base[1] * n // d
    return base


@pytest.mark.parametrize("name", ["rgb_lowpower", "rgbd"])
def test_param_files_are_low_power_and_leave_the_robot_imu_alone(name):
    p = _params(name)
    assert p["rgb"]["i_fps"] <= 10.0
    assert p["camera"]["i_enable_imu"] is False                                 # OAK IMU 는 끈다(로봇 IMU 를 쓴다)
    assert p["camera"]["i_nn_type"] == "none"                                   # 장치 위 신경망은 쓰지 않는다(탐지는 Jetson)
    if "stereo" in p:
        assert p["stereo"]["i_fps"] <= 10.0


def test_rgb_profile_has_no_stereo_and_rgbd_profile_does():
    assert "stereo" not in _params("rgb_lowpower") and _params("rgb_lowpower")["camera"]["i_pipeline_type"] == "RGB"
    p = _params("rgbd")
    assert p["camera"]["i_pipeline_type"] == "RGBD" and p["stereo"]["i_align_depth"] is True


def test_aligned_depth_output_size_is_a_multiple_of_16():
    w, h = _out_size(_params("rgbd"))
    assert (w, h) == (1280, 720) and w % 16 == 0 and h % 16 == 0
    assert (640 % 16, 360 % 16) != (0, 0)                                       # 640×360 은 안 된다 — 그래서 이 값을 쓰지 않는다
    assert _out_size(_params("rgb_lowpower")) == (1280, 720)


def test_param_files_state_the_name_map_and_what_to_confirm():
    for name in ("rgb_lowpower", "rgbd"):
        head = "\n".join((ROOT / "config" / "oak" / f"{name}.yaml").read_text(encoding="utf-8").splitlines()[:12])
        assert "[실기 확인]" in head and "ros2 param list /oak" in head


# ====================================================================== 토픽 확인
def test_wanted_topics_follow_config_and_depth_enable():
    cfg = Config()
    cfg.depth.enable = False
    rgb = [cfg.oak.rgb_topic, cfg.oak.rgb_info_topic]
    assert oak_launch.wanted_topics(cfg) == rgb
    cfg.depth.enable = True
    got = oak_launch.wanted_topics(cfg)
    assert got[:2] == rgb and cfg.depth.topic in got and cfg.depth.info_topic in got
    assert oak_launch.wanted_topics(cfg, include_rgb=False) == [t for t in got if t not in rgb]
    cfg.depth.topic = cfg.oak.rgb_topic                                          # 겹치면 한 번만
    dup = oak_launch.wanted_topics(cfg)
    assert len(dup) == len(set(dup))


def test_missing_topics_are_reported_with_every_oak_topic_and_its_type():
    cfg = Config()
    cfg.depth.enable = False
    graph = [("/oak/rgb/image", ["sensor_msgs/msg/Image"]), ("/oak/rgb/camera_info", ["sensor_msgs/msg/CameraInfo"]),
             ("/OAK/stereo/image_raw", ["sensor_msgs/msg/Image"]), ("/scan", ["sensor_msgs/msg/LaserScan"])]
    msgs = []
    missing = oak_launch.check_topics(graph, oak_launch.wanted_topics(cfg), msgs.append)
    assert missing == ["/oak/rgb/image_raw"]
    (m,) = msgs
    assert "WARN" in m and "/oak/rgb/image_raw" in m
    assert "/oak/rgb/image [sensor_msgs/msg/Image]" in m and "/OAK/stereo/image_raw" in m
    assert "/scan" not in m.split("이름에 oak 이 든 토픽:")[1]                  # oak 이 안 든 것은 목록에 없다


def test_no_warning_when_all_topics_exist_and_leading_slash_is_tolerated():
    cfg = Config()
    cfg.depth.enable = False
    cfg.oak.rgb_topic, cfg.oak.rgb_info_topic = "oak/rgb/image_raw", "oak/rgb/camera_info"
    graph = [("/oak/rgb/image_raw", ["sensor_msgs/msg/Image"]), ("/oak/rgb/camera_info", ["sensor_msgs/msg/CameraInfo"])]
    msgs = []
    assert oak_launch.check_topics(graph, oak_launch.wanted_topics(cfg), msgs.append) == [] and msgs == []


def test_driver_not_running_says_so():
    msgs = []
    oak_launch.check_topics([("/scan", ["x"])], ["/oak/rgb/image_raw"], msgs.append)
    assert "ros2 topic list | grep oak" in msgs[0]


# ====================================================================== ros_mil: OAK 구독 콜백·토픽 점검(ROS 없이)
def _transport(cfg=None):
    robot, tr, clock = make_robot("practice", "idle", cfg=cfg or Config())
    t = object.__new__(RosMilTransport)
    t.robot, t.cfg = robot, robot.cfg
    t._oak_last_decode, t._oak_compressed, t._oak_topic_timer = -1e9, False, None
    return t, robot, clock


def _img_msg(w=64, h=48, enc="bgr8", stamp=(5, 0)):
    data = np.zeros((h, w, 3), np.uint8)
    data[:, :, 0], data[:, :, 1], data[:, :, 2] = 10, 120, 240
    return NS(height=h, width=w, encoding=enc, step=w * 3, data=data.tobytes(), header=NS(stamp=NS(sec=stamp[0], nanosec=stamp[1])))


def test_oak_rgb_callback_stores_a_bgr_frame_without_pan_tilt():
    t, robot, clock = _transport()
    clock.t = 5.0
    t._on_oak_rgb(_img_msg())
    fr = robot.get_oak_frame()
    assert fr is not None and fr.pan == 0.0 and fr.tilt == 0.0 and fr.meta["source"] == "oak"
    assert tuple(fr.image[0, 0]) == (10, 120, 240) and (fr.width, fr.height) == (64, 48)
    assert robot.get_frame() is None                                             # 팬틸트 영상 슬롯은 건드리지 않는다
    assert "image" not in robot._received                                        # /ready 준비 확인에도 쓰이지 않는다


def test_oak_rgb_callback_flips_rgb8_to_bgr():
    t, robot, clock = _transport()
    clock.t = 5.0
    msg = _img_msg(enc="rgb8")
    t._on_oak_rgb(msg)
    assert tuple(robot.get_oak_frame().image[0, 0]) == (240, 120, 10)


def test_oak_rgb_decoding_is_throttled_to_twice_max_hz():
    t, robot, clock = _transport()
    clock.t = 5.0
    t._on_oak_rgb(_img_msg())
    first = robot.get_oak_frame()
    clock.t = 5.05                                                               # 1/(2·3) = 0.167 s 안
    t._on_oak_rgb(_img_msg(stamp=(5, 50_000_000)))
    assert robot.get_oak_frame() is first
    clock.t = 5.2
    t._on_oak_rgb(_img_msg(stamp=(5, 200_000_000)))
    assert robot.get_oak_frame() is not first


def test_undecodable_oak_frame_is_counted_not_raised():
    t, robot, clock = _transport()
    t._on_oak_rgb(_img_msg(enc="16uc1"))
    assert robot.stats["oak_decode_error"] == 1 and robot.get_oak_frame() is None


def test_oak_camera_info_callback_keeps_k_and_size():
    t, robot, clock = _transport()
    info = NS(k=[800.0, 0, 640.0, 0, 800.0, 360.0, 0, 0, 1.0], p=[0.0] * 12, width=1280, height=720)
    t._on_oak_info(info)
    K, w, h = robot.get_oak_info()
    assert K[0] == 800.0 and (w, h) == (1280, 720)


def test_topic_check_reports_into_stats_and_prints_the_warning(capsys):
    cfg = Config()
    cfg.depth.enable = False
    t, robot, clock = _transport(cfg)
    cancelled = []
    t._oak_topic_timer = NS(cancel=lambda: cancelled.append(1))
    t.node = NS(get_topic_names_and_types=lambda: [("/oak/rgb/image", ["sensor_msgs/msg/Image"])])
    t._check_oak_topics()
    assert cancelled == [1] and robot.stats["oak_topics_missing"] == ["/oak/rgb/image_raw", "/oak/rgb/camera_info"]
    assert "WARN" in capsys.readouterr().out


def test_topic_check_graph_failure_is_not_fatal():
    t, robot, clock = _transport()
    t._oak_topic_timer = None

    def boom():
        raise RuntimeError("graph unavailable")
    t.node = NS(get_topic_names_and_types=boom)
    t._check_oak_topics()
    assert "graph unavailable" in robot.stats["oak_topic_check_error"]


def test_run_team_helper_does_not_leak_patches_into_the_stdlib():
    import runpy
    import subprocess
    real_popen, real_run_path = subprocess.Popen, runpy.run_path
    _run_team({"MAICON_OAK": "1"}, popen=lambda *a, **k: FakeProc())
    assert subprocess.Popen is real_popen and runpy.run_path is real_run_path
