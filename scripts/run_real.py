#!/usr/bin/env python3
"""실기 실행 진입점. 로봇 ROS2 API(2차 사전교육 PDF p9~18)는 hal/wingbot.py + hal/ros_mil.py 로 구현했다(로봇에서는 미검증).
경기 서버(HTTP) 쪽 link/http.py 는 「참가팀 단계별 API 안내」(2026-11-04 스냅숏)로 구현했다(모의 서버로만 시험).
실기 기본 설정: config/waveshare_real.json. 임무 실행에는 --server 및 --detector가 필요하다.
항법: 기본은 SLAM + Nav2 (--nav nav2). `ros2 launch ros2/launch/maicon_waveshare.launch.py` 를 사용한다.
--preflight-only는 서버 제출·이동·발사 없이 센서·TF·서비스 준비를 확인한다.
      ROS 노드가 불안하면 --nav builtin 으로 내장 SLAM+A* 폴백.
대회 규정(PDF p18): 이 프로그램은 ~/team/team_node.py(= team/team_node.py)가 실행한다 — standby 때 자동 시작, 한 경기 뒤 report→stop 이면 종료,
비정상 종료는 경기당 최대 3회 자동 재실행된다. --restore 는 시계만 보존하므로 현재 위치·앵커가 복원되지 않는 경기 도중 시작은 차단한다."""
import argparse, os, sys, signal, threading
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.credentials import read_mission_code
from maicon26.arena import ArenaGeometry
from maicon26.launch_profile import load_run_config
from maicon26.hal.wingbot import RosWingbot
from maicon26.link.async_link import AsyncServerLink
from maicon26.link.base import ServerMsg
from maicon26.link.http import HttpServerLink
from maicon26.link.secrets import register_secret, redacted_output
from maicon26.mission.manager import Agent
from maicon26.mission.runner import MatchRunner
from maicon26.perception.detector import AsyncDetector, YoloDetector
from maicon26.nav.factory import make_stack
from maicon26.preflight import validate_model_labels, validate_real_config, wait_ready, clear_standby_restore, load_llm_factory

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


@redacted_output
def main():
    ap = argparse.ArgumentParser(allow_abbrev=False)
    ap.add_argument("--server", default=os.environ.get("MAICON_SERVER_URL"),
                    help="경기 서버 주소 http://<서버IP>:8000 (현장 안내값; MAICON_SERVER_URL 사용 가능)")
    ap.add_argument("--code-file", help="임무코드 파일; MAICON_CODE/MAICON_MISSION_CODE 환경변수가 우선, 없으면 숨김 입력")
    ap.add_argument("--rehearsal", action="store_true", help="리허설 경기 창구(/api/rehearsal/)")
    ap.add_argument("--ready-timeout", type=float, default=60.0, help="센서·TF·Nav2 준비 제한시간(s). 만료 시 준비완료 미전송")
    ap.add_argument("--team", choices=["BLUE", "RED"], help="보통 필요 없다 — 서버 /me 의 slot 으로 정해진다")
    ap.add_argument("--slot", type=int, choices=[1, 2, 3])
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "waveshare_real.json"))
    ap.add_argument("--arena-spec", help="측정 경기장 YAML; 중심 world 좌표를 항법 격자 좌표로 명시적으로 변환")
    ap.add_argument("--simulation-uav-calibration", help="시뮬 전용: 실측 상공 보정 JSON으로 박스 형상 후보 추출. 같은 치수의 --arena-spec 필요")
    ap.add_argument("--preflight-only", action="store_true", help="센서·TF·서비스 준비만 확인. 서버 요청과 이동·발사 없음")
    ap.add_argument("--names-map", help="YOLO 클래스명 → 내부 클래스명 JSON 매핑")
    ap.add_argument("--llm-factory", help="팀 LLM 어댑터 module:function; complete(prompt, timeout) 구현")
    ap.add_argument("--drone-camera", choices=["main", "test1", "test2"], help="리허설 진단용 UAV 카메라")
    ap.add_argument("--overrides", default=None, nargs="?", const="config/field_overrides.json",
                    help="현장 실험으로 갱신한 값(config/field_overrides.json)을 얹는다. 값 없이 쓰면 기본 경로. python scripts/field_exp.py status 로 확인")
    ap.add_argument("--detector", default=None, help="YOLO 가중치(.pt/.engine)")
    ap.add_argument("--nav", choices=["nav2", "builtin"], default="nav2",
                    help="항법 백엔드: nav2(ROS2 SLAM+Nav2, 기본) | builtin(내장 SLAM+A* 폴백)")
    ap.add_argument("--matches", type=int, choices=[1], default=1, help="공식 team_node 수명: 프로세스당 한 경기")
    ap.add_argument("--restore", nargs="?", const="~/team/.maicon26_match.json", default=None,
                    help="경기 시계 저장 경로. 지도·앵커 복원 기능은 아니며 실기 진입점은 경기 도중 새 프로세스 임무 시작을 차단")
    if any(item == "--mission-code" or item.startswith("--mission-code=") for item in sys.argv[1:]):
        # argparse normally echoes unrecognized arguments, including their
        # values. Reject this removed secret-bearing option without that echo.
        ap.error("임무코드는 명령행 인자로 받지 않습니다. MAICON_CODE 또는 --code-file을 사용하세요")
    a = ap.parse_args()
    if not a.preflight_only and (not a.server or not a.detector):
        ap.error("실기 임무에는 --server와 --detector가 필요합니다. 센서만 확인하려면 --preflight-only")
    if a.detector and not os.path.isfile(a.detector):
        ap.error("--detector는 존재하는 로컬 가중치 파일이어야 합니다")
    if a.drone_camera and not a.rehearsal:
        ap.error("--drone-camera는 --rehearsal 진단에만 사용합니다")
    if a.simulation_uav_calibration and not a.arena_spec:
        ap.error("--simulation-uav-calibration은 측정된 --arena-spec이 필요합니다")
    cfg, applied = load_run_config(a.config, a.overrides, a.nav)
    if a.arena_spec:
        import yaml
        with open(a.arena_spec, encoding="utf-8") as arena_file:
            cfg.arena = ArenaGeometry.from_spec(yaml.safe_load(arena_file))
    uav_layout_detector = None
    if a.simulation_uav_calibration:
        from maicon26.perception.simulation_uav import load_simulation_uav_detector
        try:
            uav_layout_detector = load_simulation_uav_detector(a.simulation_uav_calibration, cfg.arena).detect
        except (ValueError, OSError, TypeError, KeyError):
            ap.error("시뮬 UAV 보정/치수/독립 검증이 유효하지 않습니다")
    if a.restore:
        cfg.hal.restart_state_path = os.path.expanduser(a.restore)
    if a.overrides:
        print(f"[run_real] 현장 오버라이드 {len(applied)}개 적용: {', '.join(applied) or '(없음)'}", flush=True)
    cfg.team, cfg.slot = a.team or cfg.team, a.slot or cfg.slot
    validate_real_config(cfg)
    try:
        code = "" if a.preflight_only else read_mission_code(a.code_file)
    except ValueError as error:
        ap.error(str(error))
    llm_cfg = getattr(cfg, "llm", None)
    key_env = getattr(llm_cfg, "api_key_env", "MAICON_LLM_API_KEY")
    register_secret(os.environ.get(key_env, ""))
    llm_factory = a.llm_factory or os.environ.get("MAICON_LLM_FACTORY") or getattr(llm_cfg, "adapter_factory", None)
    if llm_cfg is not None:
        llm_cfg.endpoint = os.environ.get("MAICON_LLM_API_URL") or llm_cfg.endpoint
    robot = stack = link = det = llm = None
    quit_event = threading.Event()
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda *_: quit_event.set())
    try:
        if llm_factory:
            llm = load_llm_factory(llm_factory)
        elif not a.preflight_only:
            print("[run_real] WARN LLM 미연결: M1/M6 규칙 기반 대체만 사용. PDF의 LLM 수행 검증은 별도 필요", flush=True)
        inner = None
        if a.detector:
            import json
            names_map = None
            if a.names_map:
                with open(a.names_map, encoding="utf-8") as f:
                    names_map = json.load(f)
            inner = YoloDetector(a.detector, names_map=names_map)
            validate_model_labels(inner, cfg)
        robot = RosWingbot(cfg)
        stack = make_stack(cfg, robot)
        wait_ready(robot, stack, a.ready_timeout, should_quit=quit_event.is_set)
        if inner is not None:
            inner.detect(robot.get_frame())  # First real-image inference before /ready.
            det = AsyncDetector(inner)
        if a.preflight_only:
            print("[run_real] 사전 점검 통과: 센서·경기 상태·서비스·항법 연결. 구동/발사 시험은 하지 않음", flush=True)
            return
        link = HttpServerLink(a.server, code, cfg.team, cfg.slot, rehearsal=a.rehearsal,
                              drone_camera=a.drone_camera, strict_code=False,
                              **({"uav_layout_detector": uav_layout_detector,
                                  "uav_layout_max_age_s": cfg.explore.uav_layout_timeout_s}
                                 if uav_layout_detector is not None else {}))
        startup_phase = None
        st = robot.status()
        if st.match_mode == "mission" and st.match_cmd in ("start", "resume", "pause"):
            robot.set_velocity(0.0, 0.0)
            state = link.me()  # Read-only phase check after all local initialization; never /ready or motion.
            startup_phase = state.get("phase") if isinstance(state, dict) else None
            if startup_phase == "finished":
                robot.set_velocity(0.0, 0.0)
                return
            if startup_phase != "reporting":
                raise RuntimeError("경기 도중 새 프로세스 시작: 시계만으로 SLAM 앵커를 복원할 수 없습니다. "
                                   "현재 위치 재초기화 없이 출발 거점으로 가정하지 않습니다")
        clear_standby_restore(robot, cfg.hal.restart_state_path)
        def init_ok():
            wait_ready(robot, stack, a.ready_timeout, should_quit=quit_event.is_set)
            return True
        if not link.ready_handshake(init_ok, should_quit=quit_event.is_set):
            return
        ident = link.identity()
        if ident is None:
            raise RuntimeError("서버 팀·슬롯 확인 실패 — 임의 BLUE/2 출발 금지")
        cfg.team, cfg.slot = ident
        if getattr(getattr(cfg, "arena", None), "measured_starts", None):
            cfg.nav2.start_pose_override = cfg.arena.measured_starts[f"{cfg.team[0]}-{cfg.slot}"]
        startup_phase = getattr(link, "phase", None) or startup_phase
        link = AsyncServerLink(link)  # Real control loops must never wait on network requests.
        def make_agent(srv):
            agent = Agent(robot, srv, cfg=cfg, detector=det, stack=stack, llm=llm,
                          name=f"{cfg.team}-{cfg.slot}")
            if startup_phase == "reporting":
                # Seed the authoritative phase before the first control tick;
                # stale robot start/resume metadata cannot authorize motion.
                agent.latch.accept(ServerMsg("stop", data={"src": "startup_server_phase"}))
            return agent
        runner = MatchRunner(make_agent, link,
                             tick_period=1.0 / cfg.loop_hz, max_matches=1)
        runner.run(should_quit=quit_event.is_set)
        # PDF2 p18: stay stopped after reporting until operator sends stop.
        while not quit_event.is_set():
            st = robot.status()
            if st.match_mode != "mission" or st.match_cmd == "stop":
                break
            robot.set_velocity(0.0, 0.0)
            quit_event.wait(0.1)
    except InterruptedError:
        pass
    finally:
        for resource in (stack, robot, det, link, llm):
            if resource is not None:
                try:
                    getattr(resource, "close", lambda: None)()
                except Exception as exc:
                    print(f"[run_real] 종료 정리 실패: {type(exc).__name__}", flush=True)
        from maicon26.nav.ros_runtime import shutdown as ros_shutdown
        ros_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
