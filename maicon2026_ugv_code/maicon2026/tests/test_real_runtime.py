"""Real entry-point lifecycle tests without ROS, hardware, models or network."""
import importlib.util
from pathlib import Path
import signal
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]


def load_script(relative, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def runtime(monkeypatch, tmp_path):
    mod = load_script("scripts/run_real.py", "test_run_real_entry")
    events, handlers = [], {}
    cfg = SimpleNamespace(team="BLUE", slot=1, loop_hz=20.0,
                          hal=SimpleNamespace(restart_state_path=""),
                          nav=SimpleNamespace(backend="nav2"))
    monkeypatch.setattr(mod, "load_run_config", lambda *args: (cfg, []))
    monkeypatch.setattr(mod, "validate_real_config", lambda _: None)
    monkeypatch.setattr(mod, "validate_model_labels", lambda *args: events.append("labels"))
    monkeypatch.setattr(mod, "clear_standby_restore", lambda *args: events.append("restore-check"))

    class Resource:
        def __init__(self, name):
            self.name = name
        def close(self):
            events.append("close-" + self.name)

    class Robot(Resource):
        def __init__(self, _cfg):
            super().__init__("robot")
            events.append("robot")
        def get_frame(self):
            return object()
        def status(self):
            return SimpleNamespace(match_mode="mission", match_cmd="stop")
        def set_velocity(self, v, w):
            events.append(("velocity", v, w))

    class Yolo:
        def __init__(self, *args, **kwargs):
            events.append("model")
        def detect(self, frame):
            events.append("warmup")
            return []

    class Link(Resource):
        def __init__(self, *args, **kwargs):
            super().__init__("link")
            events.append("link")
        def me(self):
            events.append("me")
            return {"phase": "running"}
        def ready_handshake(self, init_ok, should_quit):
            events.append("handshake")
            return init_ok()
        def identity(self):
            return "RED", 2

    class Runner:
        def __init__(self, factory, link, **kwargs):
            events.append("runner")
            self.factory = factory
            self.link = link
        def run(self, should_quit):
            self.factory(self.link)
            events.append("run")

    def fake_signal(sig, callback):
        prior = handlers.get(sig, signal.SIG_DFL)
        handlers[sig] = callback
        return prior

    monkeypatch.setattr(mod.signal, "signal", fake_signal)
    monkeypatch.setattr(mod, "RosWingbot", Robot)
    monkeypatch.setattr(mod, "YoloDetector", Yolo)
    monkeypatch.setattr(mod, "AsyncDetector", lambda _: Resource("detector"))
    monkeypatch.setattr(mod, "make_stack", lambda *args: Resource("stack"))
    monkeypatch.setattr(mod, "wait_ready", lambda *args, **kwargs: events.append("ready"))
    monkeypatch.setattr(mod, "HttpServerLink", Link)
    monkeypatch.setattr(mod, "AsyncServerLink", lambda link: events.append("async-link") or link)
    monkeypatch.setenv("MAICON_CODE", "A1b2C3d4")  # Synthetic fixture only; never read an operator's credentials.
    monkeypatch.delenv("MAICON_LLM_FACTORY", raising=False)
    monkeypatch.setattr(mod, "MatchRunner", Runner)
    monkeypatch.setattr(mod, "Agent", lambda *args, **kwargs: events.append(("agent", kwargs["detector"].name, kwargs["cfg"].team)))
    import maicon26.nav.ros_runtime as ros_runtime
    monkeypatch.setattr(ros_runtime, "shutdown", lambda: events.append("ros-shutdown"))
    model = tmp_path / "model.pt"
    model.write_bytes(b"not-loaded-by-fake")
    argv = ["run_real.py", "--server", "http://localhost", "--detector", str(model)]
    monkeypatch.setattr(sys, "argv", argv)
    return SimpleNamespace(mod=mod, events=events, handlers=handlers, argv=argv, cfg=cfg,
                           Robot=Robot, Yolo=Yolo, Link=Link, Runner=Runner)


@pytest.mark.spec("S1.1", "S1.7", "S2.4")
def test_success_infers_before_ready_and_closes_all(runtime):
    runtime.mod.main()
    e = runtime.events
    assert e.index("warmup") < e.index("handshake") < e.index("runner")
    assert e.index("handshake") < e.index("async-link") < e.index("runner")
    assert ("agent", "detector", "RED") in e
    for name in ("stack", "robot", "detector", "link"):
        assert "close-" + name in e
    assert e[-1] == "ros-shutdown"
    assert runtime.handlers[signal.SIGINT] == signal.SIG_DFL
    assert runtime.handlers[signal.SIGTERM] == signal.SIG_DFL


@pytest.mark.spec("S0.1")
def test_real_launcher_accepts_explicit_server_environment_without_guessing(runtime, monkeypatch):
    address = "http://localhost:8000"
    monkeypatch.setenv("MAICON_SERVER_URL", address)
    runtime.argv[1:3] = []
    supplied = []
    original = runtime.Link.__init__
    def init(self, *args, **kwargs):
        supplied.append((args, kwargs))
        original(self, *args, **kwargs)
    monkeypatch.setattr(runtime.Link, "__init__", init)
    runtime.mod.main()
    assert supplied[0][0][0] == address and supplied[0][1]["strict_code"] is False


@pytest.mark.spec("S0.2", "S0.4")
@pytest.mark.parametrize("equals", [False, True])
def test_removed_command_line_credential_never_echoes_the_supplied_value(runtime, capsys, equals):
    value = "Z9y8X7w6"
    runtime.argv.extend(["--mission-code="+value] if equals else ["--mission-code", value])
    with pytest.raises(SystemExit) as error:
        runtime.mod.main()
    output = capsys.readouterr()
    assert error.value.code == 2
    assert value.casefold() not in (output.out+output.err).casefold()
    assert not runtime.events


@pytest.mark.spec("S2.4")
def test_real_launcher_cannot_enable_network_calls_in_control_loop(runtime):
    runtime.argv.append("--sync-link")
    with pytest.raises(SystemExit) as error:
        runtime.mod.main()
    assert error.value.code == 2 and not runtime.events


def test_abbreviated_config_cannot_diverge_from_managed_launch(runtime):
    runtime.argv += ["--conf", "unshared-profile.json"]
    with pytest.raises(SystemExit) as exc:
        runtime.mod.main()
    assert exc.value.code == 2
    assert "robot" not in runtime.events


def test_team_node_accepts_standard_equals_server_option(monkeypatch):
    team = load_script("team/team_node.py", "test_team_equals_entry")
    monkeypatch.delenv("MAICON_SERVER", raising=False)
    monkeypatch.delenv("TEAM_LAUNCH_NAV", raising=False)
    monkeypatch.setattr(sys, "argv", ["team_node.py", "--server=http://localhost:8000"])
    called = []
    monkeypatch.setattr(team.runpy, "run_path", lambda *args, **kwargs: called.append(list(sys.argv)))
    assert team.main() == 0
    assert "--server=http://localhost:8000" in called[0]


@pytest.mark.spec("S0.1")
def test_team_node_supports_new_server_environment_with_legacy_fallback(monkeypatch):
    team = load_script("team/team_node.py", "test_team_server_env")
    monkeypatch.setenv("MAICON_SERVER_URL", "http://localhost:8001")
    monkeypatch.setenv("MAICON_SERVER", "http://localhost:8002")
    monkeypatch.delenv("TEAM_LAUNCH_NAV", raising=False)
    monkeypatch.setattr(sys, "argv", ["team_node.py"])
    called = []
    monkeypatch.setattr(team.runpy, "run_path", lambda *args, **kwargs: called.append(list(sys.argv)))
    assert team.main() == 0
    assert called[-1][called[-1].index("--server")+1] == "http://localhost:8001"
    monkeypatch.delenv("MAICON_SERVER_URL")
    assert team.main() == 0
    assert called[-1][called[-1].index("--server")+1] == "http://localhost:8002"


@pytest.mark.spec("S1.6", "S2.4")
def test_mid_match_restart_cannot_guess_starting_base(runtime, monkeypatch):
    monkeypatch.setattr(runtime.Robot, "status", lambda _: SimpleNamespace(match_mode="mission", match_cmd="resume"))
    with pytest.raises(RuntimeError, match="앵커"):
        runtime.mod.main()
    assert "handshake" not in runtime.events and "runner" not in runtime.events
    assert runtime.events.index("warmup") < runtime.events.index("me")
    assert "close-link" in runtime.events
    assert "close-robot" in runtime.events


@pytest.mark.spec("S1.6", "S10.2", "S11.3")
def test_reporting_restart_latches_stop_before_first_agent_tick_even_with_stale_robot_start(runtime, monkeypatch):
    from maicon26.mission.guard import MatchLatch
    state = {"cmd": "resume"}
    monkeypatch.setattr(runtime.Robot, "status", lambda _: SimpleNamespace(match_mode="mission", match_cmd=state["cmd"]))
    monkeypatch.setattr(runtime.Link, "me", lambda _: {"phase": "reporting"})
    agent = SimpleNamespace(latch=MatchLatch())
    monkeypatch.setattr(runtime.mod, "Agent", lambda *args, **kwargs: agent)
    def run(runner, should_quit):
        made = runner.factory(runner.link)
        assert made.latch.stopped and not made.latch.started
        state["cmd"] = "stop"
    monkeypatch.setattr(runtime.Runner, "run", run)
    runtime.mod.main()
    assert agent.latch.stopped and "close-link" in runtime.events


@pytest.mark.spec("S0.16", "S10.2")
def test_finished_restart_does_not_ready_or_report(runtime, monkeypatch):
    monkeypatch.setattr(runtime.Robot, "status", lambda _: SimpleNamespace(match_mode="mission", match_cmd="resume"))
    monkeypatch.setattr(runtime.Link, "me", lambda _: {"phase": "finished"})
    runtime.mod.main()
    assert "handshake" not in runtime.events and "runner" not in runtime.events
    assert ("velocity", 0., 0.) in runtime.events


def test_missing_detector_is_rejected_before_hardware(runtime, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["run_real.py", "--server", "http://localhost"])
    with pytest.raises(SystemExit) as exc:
        runtime.mod.main()
    assert exc.value.code == 2
    assert not runtime.events


@pytest.mark.spec("S1.1", "S1.7")
def test_model_load_failure_does_not_open_hardware(runtime, monkeypatch):
    def fail(*args, **kwargs):
        raise RuntimeError("model cannot load")
    monkeypatch.setattr(runtime.mod, "YoloDetector", fail)
    with pytest.raises(RuntimeError, match="model cannot load"):
        runtime.mod.main()
    assert "robot" not in runtime.events and "link" not in runtime.events
    assert runtime.events[-1] == "ros-shutdown"
    assert runtime.handlers[signal.SIGTERM] == signal.SIG_DFL


@pytest.mark.spec("S1.1", "S1.7")
def test_readiness_timeout_closes_robot_stack_without_reporting_ready(runtime, monkeypatch):
    # 규칙 변경(프롬프트 1장): 예전에는 'scan stale' 이 TimeoutError 였다. 지금은 scan 이 없어도 저하 모드로 /ready 를 보내므로
    # 이 시험은 '필수 항목(state·match)이 끝까지 없으면 준비완료를 보내지 않고 자원을 정리한다'를 확인한다.
    def timeout(*args, **kwargs):
        raise TimeoutError("준비완료 미전송: state; match")
    monkeypatch.setattr(runtime.mod, "wait_ready", timeout)
    with pytest.raises(TimeoutError, match="state"):
        runtime.mod.main()
    assert "close-robot" in runtime.events and "close-stack" in runtime.events
    assert "handshake" not in runtime.events and "link" not in runtime.events


def test_warmup_error_closes_hardware_before_ready(runtime, monkeypatch):
    def fail(*args):
        raise RuntimeError("inference failed")
    monkeypatch.setattr(runtime.Yolo, "detect", fail)
    with pytest.raises(RuntimeError, match="inference failed"):
        runtime.mod.main()
    assert "close-robot" in runtime.events and "close-stack" in runtime.events
    assert "handshake" not in runtime.events


@pytest.mark.parametrize("failure", ["exception", "false"])
def test_handshake_failure_cleans_every_open_resource(runtime, monkeypatch, failure):
    def handshake(*args, **kwargs):
        if failure == "exception":
            raise TimeoutError("readiness lost")
        return False
    monkeypatch.setattr(runtime.Link, "ready_handshake", handshake)
    if failure == "exception":
        with pytest.raises(TimeoutError):
            runtime.mod.main()
    else:
        runtime.mod.main()
    assert "runner" not in runtime.events
    for name in ("stack", "robot", "detector", "link"):
        assert "close-" + name in runtime.events


def test_sigterm_requests_runner_stop_and_closes_hardware(runtime, monkeypatch):
    def run(_self, should_quit):
        assert not should_quit()
        runtime.handlers[signal.SIGTERM](signal.SIGTERM, None)
        assert should_quit()
    monkeypatch.setattr(runtime.Runner, "run", run)
    runtime.mod.main()
    assert "close-robot" in runtime.events and "close-stack" in runtime.events
    assert not any(isinstance(e, tuple) and e[0] == "velocity" for e in runtime.events)


def test_preflight_only_never_opens_server_or_uses_implicit_detector(runtime, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["run_real.py", "--preflight-only"])
    runtime.mod.main()
    assert "model" not in runtime.events and "link" not in runtime.events
    assert "runner" not in runtime.events and "ready" in runtime.events
    assert "close-robot" in runtime.events


def test_team_node_closes_launched_child_when_entry_fails(monkeypatch):
    team = load_script("team/team_node.py", "test_team_entry")
    events = []
    class Child:
        pid = 123
        def poll(self): return None
        def wait(self, timeout): events.append("wait")
        def terminate(self): events.append("terminate")
        def send_signal(self, sig):
            assert sig == signal.SIGINT
            events.append("signal-child")
    monkeypatch.setenv("MAICON_SERVER", "http://localhost")
    monkeypatch.setenv("TEAM_LAUNCH_NAV", "1")
    monkeypatch.setattr(sys, "argv", ["team_node.py"])
    saved_argv = sys.argv
    monkeypatch.setattr(team, "_launch_nav", lambda argv: Child())
    monkeypatch.setattr(team.os, "getpgid", lambda pid: pid, raising=False)
    monkeypatch.setattr(team.os, "killpg", lambda *args: events.append("signal-child"), raising=False)
    def fail(*args, **kwargs):
        raise SystemExit(2)
    monkeypatch.setattr(team.runpy, "run_path", fail)
    assert team.main() == 2
    assert sys.argv is saved_argv
    assert events == ["signal-child", "wait"]  # The owned fake supports graceful ros2-launch SIGINT on every platform.


def test_http_start_before_match_callback_restores_clock_without_overwriting(tmp_path, monkeypatch):
    import json
    from test_mil_hal import _api_solo
    import maicon26.mission.manager as manager
    _match, agent, _world, robot = _api_solo()
    agent.team, agent.slot = "BLUE", 2  # run_real resolves /me before constructing Agent.
    restore = tmp_path / "match.json"
    restore.write_text(json.dumps({"match_id": "existing", "wall_start": 960.0,
                                   "team": "BLUE", "slot": 2}), encoding="utf-8")
    before = restore.read_bytes()
    agent.cfg.hal.restart_state_path = str(restore)
    monkeypatch.setattr(manager.time, "time", lambda: 1000.0)
    agent.latch.started = True  # HTTP _serve has accepted start before _match_api runs.
    agent._on_start(5.0, "server_phase")
    assert agent.t_start == -35.0
    assert agent.stats["restored"]
    assert agent.latch.match_id == "existing"
    assert restore.read_bytes() == before


@pytest.mark.parametrize("team,slot", [("RED", 2), ("BLUE", 3)])
def test_restart_does_not_restore_another_assignment(tmp_path, monkeypatch, team, slot):
    import json
    from test_mil_hal import _api_solo
    import maicon26.mission.manager as manager
    _match, agent, _world, robot = _api_solo()
    agent.team, agent.slot = "BLUE", 2
    restore = tmp_path / "match.json"
    restore.write_text(json.dumps({"wall_start": 960.0, "team": team, "slot": slot}), encoding="utf-8")
    agent.cfg.hal.restart_state_path = str(restore)
    monkeypatch.setattr(manager.time, "time", lambda: 1000.0)
    agent._try_restore(5.0)
    assert not agent.stats["restored"] and agent.t_start is None


@pytest.mark.parametrize("available,pose", [(False, (0, 0, 0)), (True, None)])
def test_preflight_waits_for_both_nav2_and_transform(available, pose):
    from maicon26.preflight import wait_ready
    robot = SimpleNamespace(readiness=lambda: (True, []))
    client = SimpleNamespace(available=lambda: available, lookup_map_to_base=lambda: pose)
    ticks = iter([0.0, 0.0, 2.0])
    # 규칙 변경(프롬프트 1장): Nav2 액션 서버·map→base TF 는 여전히 둘 다 확인하지만, 제한시간 뒤 TimeoutError 대신
    # 저하 모드(no_drive)로 /ready 를 보낸다. 확인 횟수(시계 호출 3번)는 기존과 같다.
    state = wait_ready(robot, SimpleNamespace(client=client), 1.0,
                       clock=lambda: next(ticks), sleep=lambda _: None)
    assert state.degraded and state.flags == {"no_drive"}
    assert any(("Nav2" in r) if not available else ("TF" in r) for r in state.reasons)


def test_standby_clears_old_restore_but_running_preserves_it(tmp_path):
    from maicon26.preflight import clear_standby_restore
    restore = tmp_path / "match.json"
    restore.write_text("{}", encoding="utf-8")
    state = SimpleNamespace(match_mode="mission", match_cmd="start")
    robot = SimpleNamespace(status=lambda: state)
    clear_standby_restore(robot, str(restore))
    assert restore.exists()
    state.match_cmd = "standby"
    clear_standby_restore(robot, str(restore))
    assert not restore.exists()


def test_model_without_base_signs_is_rejected():
    from maicon26.config import Config
    from maicon26.preflight import validate_model_labels
    cfg = Config()
    labels = list(cfg.perception.object_classes) + list(cfg.perception.target_classes) + ["UGV"]
    detector = SimpleNamespace(model=SimpleNamespace(names=labels), names_map={})
    with pytest.raises(ValueError, match="COMMAND_POST"):
        validate_model_labels(detector, cfg)


def test_simulation_uav_option_requires_explicit_measured_arena_before_hardware(runtime, monkeypatch):
    monkeypatch.setattr(sys, "argv", runtime.argv + ["--simulation-uav-calibration", "unused.json"])
    with pytest.raises(SystemExit):
        runtime.mod.main()
    assert not runtime.events


def test_simulation_uav_option_passes_measured_detector_only_when_explicit(runtime, monkeypatch):
    package = ROOT/"ros2/maicon_arena_gz"
    captured = {}
    original = runtime.Link
    class CapturedLink(original):
        def __init__(self, *args, **kwargs):
            captured.update(kwargs)
            super().__init__(*args, **kwargs)
    monkeypatch.setattr(runtime.mod, "HttpServerLink", CapturedLink)
    runtime.cfg.nav2 = SimpleNamespace(start_pose_override=None)
    runtime.cfg.explore = SimpleNamespace(uav_layout_timeout_s=12.)
    monkeypatch.setattr(sys, "argv", runtime.argv + [
        "--arena-spec", str(package/"config/arena_spec.yaml"),
        "--simulation-uav-calibration", str(package/"config/uav_grid_calibration.json")])
    runtime.mod.main()
    assert callable(captured["uav_layout_detector"])
    assert captured["uav_layout_max_age_s"] == 12.
    assert captured["uav_layout_detector"].__self__.calibration.arena_width == 6.12
