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
from maicon26.perception.detector import AsyncDetector, SharedAsyncDetector, YoloDetector
from maicon26.nav.factory import make_stack
from maicon26.preflight import (validate_model_labels, validate_real_config, wait_ready, clear_standby_restore, load_llm_factory,
                                FLAG_NO_DRIVE, FLAG_NO_VISION)
from maicon26.degrade import DegradeController
from maicon26.validate import ValidationLog, run_log_dir, sanitize_config
from maicon26.nav.degraded import NoDriveStack

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def link_validation_kwargs(cfg, robot):
    """9장 V6: 서버 링크에 명령문 길이 제한과 검증 기록을 넘긴다. 설정·로봇에 해당 값이 없으면(시험용 가짜 객체) 아무것도 넘기지 않는다."""
    out = {}
    link_cfg = getattr(cfg, "link", None)
    if link_cfg is not None and hasattr(link_cfg, "max_command_chars"):
        out["max_command_chars"] = link_cfg.max_command_chars
    vlog = getattr(robot, "vlog", None)
    if vlog is not None:
        out["vlog"] = vlog
    return out


@redacted_output
def main():
    ap = argparse.ArgumentParser(allow_abbrev=False)
    ap.add_argument("--server", default=os.environ.get("MAICON_SERVER_URL"),
                    help="경기 서버 주소 http://<서버IP>:8000 (현장 안내값; MAICON_SERVER_URL 사용 가능)")
    ap.add_argument("--code-file", help="임무코드 파일; MAICON_CODE/MAICON_MISSION_CODE 환경변수가 우선, 없으면 숨김 입력")
    ap.add_argument("--rehearsal", action="store_true", help="리허설 경기 창구(/api/rehearsal/)")
    ap.add_argument("--ready-timeout", type=float, default=60.0,
                    help="센서·TF·Nav2 준비 제한시간(s). 만료 시 필수 항목(state·match)이 없으면 준비완료 미전송, "
                         "LiDAR·Nav2·깊이 등이 없으면 저하 모드로 /ready 를 보낸다(LiDAR 기동 시간을 재서 정한다)")
    ap.add_argument("--team", choices=["BLUE", "RED"], help="보통 필요 없다 — 서버 /me 의 slot 으로 정해진다")
    ap.add_argument("--slot", type=int, choices=[1, 2, 3])
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "waveshare_real.json"))
    ap.add_argument("--arena-spec", help="측정 경기장 YAML; 중심 world 좌표를 항법 격자 좌표로 명시적으로 변환")
    ap.add_argument("--simulation-uav-calibration", help="시뮬 전용: 실측 상공 보정 JSON으로 박스 형상 후보 추출. 같은 치수의 --arena-spec 필요")
    ap.add_argument("--uav-calibration", help="실기 UAV(상공 카메라) 보정 JSON — 이 보정으로 영상의 상자 윗면을 찾아 면 장부에 올린다. 없으면 config uav_boxes.calibration 을 본다")
    ap.add_argument("--uav-camera", choices=["main", "test1", "test2"], help="보정이 맞는 UAV 카메라(기본: config uav_boxes.camera = main). 서버 X-Camera 가 다른 프레임은 쓰지 않는다 — 연습 카메라 test1/test2 는 그 카메라의 보정과 함께 지정")
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
    if a.simulation_uav_calibration and a.uav_calibration:
        ap.error("--simulation-uav-calibration(시뮬)과 --uav-calibration(실기)은 함께 쓸 수 없습니다")
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
    uav_layout_source = "gazebo_uav_image"
    ub = getattr(cfg, "uav_boxes", None)
    if uav_layout_detector is None and ub is not None and (a.uav_calibration or ub.enable):
        # 실기 UAV 상자 검출기. 인자로 준 보정이 틀리면 시작 전에 알리고, 설정 파일의 보정은 없거나 틀려도 경고만 낸다(/ready 전 단계에 새 예외를 넣지 않는다).
        uav_path = a.uav_calibration or ub.calibration
        if uav_path and not os.path.isabs(uav_path):
            uav_path = os.path.join(ROOT, uav_path)
        if not uav_path:
            print("[run_real] WARN UAV 상자 검출 꺼짐: uav_boxes.calibration 도 --uav-calibration 도 없습니다", flush=True)
        else:
            from maicon26.perception.uav_boxes_real import load_real_uav_detector
            try:
                uav_layout_detector = load_real_uav_detector(uav_path, cfg.arena, ub, camera=a.uav_camera or ub.camera)
                uav_layout_source = "real_uav_image"
                status = uav_layout_detector.cal.validation_report()["status"]
                print(f"[run_real] UAV 상자 검출 켬: 보정 {os.path.basename(uav_path)} 카메라 {uav_layout_detector.camera} 검증 {status}"
                      + ("" if status == "within_tolerance" else " — 별도 점검점 없이 만든 보정입니다(연습 때 확인)"), flush=True)
            except (ValueError, OSError, TypeError, KeyError) as error:
                if a.uav_calibration:
                    ap.error(f"--uav-calibration을 쓸 수 없습니다: {error}")
                print(f"[run_real] WARN UAV 상자 검출 꺼짐: 보정 파일을 쓸 수 없습니다({type(error).__name__}: {error})", flush=True)
    if a.restore:
        cfg.hal.restart_state_path = os.path.expanduser(a.restore)
    if a.overrides:
        print(f"[run_real] 현장 오버라이드 {len(applied)}개 적용: {', '.join(applied) or '(없음)'}", flush=True)
    cfg.team, cfg.slot = a.team or cfg.team, a.slot or cfg.slot
    start_vlog = ValidationLog(warn=lambda m: print(m, flush=True))     # 9장 V8: 새 설정 키가 이상하면 경고 후 기본값. /ready 전 단계에 새 예외를 넣지 않는다
    try:
        sanitize_config(cfg, start_vlog, root=ROOT)
    except Exception as exc:                                            # noqa: BLE001 — sanitize_config 는 던지지 않지만 시작을 막지 않는다
        print(f"[run_real] 설정 점검 건너뜀: {type(exc).__name__}", flush=True)
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
    agents_made = []                                            # 6장: 경기마다 만든 Agent — 종료 때 OAK 표본 쓰기 스레드를 닫는다
    degrade = None                                           # 저하 모드 제어기(1장) — 저하일 때만 만든다
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
        robot_vlog = getattr(robot, "vlog", None)                       # 시험용 가짜 로봇에는 없다
        if robot_vlog is not None:
            try:                                                        # 9장: 버린 데이터 상태 파일(2 s 마다, 별도 스레드). 실패해도 경기는 계속한다
                robot_vlog.absorb(start_vlog)
                robot_vlog.status_path = run_log_dir() / "validation_status.json"
                robot_vlog.start_writer()
            except Exception as exc:                                    # noqa: BLE001
                print(f"[run_real] validation_status.json 쓰기 시작 실패: {type(exc).__name__}", flush=True)
        stack = make_stack(cfg, robot)

        def say(message):
            print(message, flush=True)

        def adopt(state):
            """wait_ready 결과를 반영한다. 저하(LiDAR·Nav2·깊이 등 누락)면 HAL 가드·항법 대체물·재확인 제어기를 건다. 정상이면 아무것도 안 한다."""
            nonlocal stack, degrade
            if not getattr(state, "degraded", False) and degrade is None:
                return
            if degrade is None:
                if FLAG_NO_DRIVE in state.flags and not isinstance(stack, NoDriveStack):
                    stack = NoDriveStack(stack)              # 주행 목표를 받지 않고 '도달 불가'를 즉시 돌려준다
                degrade = DegradeController(robot, stack, state, report=say)
            else:
                degrade.update(state)
            if state.degraded:
                say(f"[run_real] 저하 모드로 /ready 를 보냅니다: 플래그 {sorted(state.flags) or '없음(선택 항목만)'} | 누락: {state.summary()}")

        state = wait_ready(robot, stack, a.ready_timeout, should_quit=quit_event.is_set, report=say)
        adopt(state)
        if inner is not None:
            if not (getattr(state, "degraded", False) and FLAG_NO_VISION in state.flags):
                inner.detect(robot.get_frame())  # First real-image inference before /ready. (영상이 없으면 건너뛴다 — 저하 no_vision)
            # 6장: oak_det.enable 이면 같은 모델·같은 스레드로 팬틸트 슬롯과 OAK 슬롯을 번갈아 처리한다(꺼져 있으면 기존 AsyncDetector 그대로)
            oak_det = getattr(cfg, "oak_det", None)                   # 시험용 단순 cfg 에는 없을 수 있다
            det = SharedAsyncDetector(inner, oak_det) if getattr(oak_det, "enable", False) else AsyncDetector(inner)
        if a.preflight_only:
            if getattr(state, "degraded", False):
                print(f"[run_real] 사전 점검 저하: 누락 항목 = {state.summary()} | 플래그 {sorted(state.flags) or '없음(선택 항목만)'}", flush=True)
                return 2                                    # 운영자가 알아채도록 0 이 아닌 종료 코드
            print("[run_real] 사전 점검 통과: 센서·경기 상태·서비스·항법 연결. 구동/발사 시험은 하지 않음", flush=True)
            return
        link = HttpServerLink(a.server, code, cfg.team, cfg.slot, rehearsal=a.rehearsal,
                              drone_camera=a.drone_camera, strict_code=False,
                              **link_validation_kwargs(cfg, robot),
                              **({"uav_layout_detector": uav_layout_detector,
                                  "uav_layout_max_age_s": cfg.explore.uav_layout_timeout_s,
                                  "uav_layout_source": uav_layout_source}
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
        last = {"state": state}

        def init_ok():
            # 첫 wait_ready 가 제한시간을 이미 썼다면 이후 호출(핸드셰이크가 2 s 마다 부른다)은 짧게 다시 분류만 한다.
            prior = getattr(last["state"], "degraded", False)
            last["state"] = wait_ready(robot, stack, min(a.ready_timeout, 1.0) if prior else a.ready_timeout,
                                       should_quit=quit_event.is_set, report=say)
            adopt(last["state"])
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
            extra = {} if degrade is None else {"degrade": degrade}      # 정상 경로의 Agent 호출은 기존과 같다
            agent = Agent(robot, srv, cfg=cfg, detector=det, stack=stack, llm=llm,
                          name=f"{cfg.team}-{cfg.slot}", **extra)
            if startup_phase == "reporting":
                # Seed the authoritative phase before the first control tick;
                # stale robot start/resume metadata cannot authorize motion.
                agent.latch.accept(ServerMsg("stop", data={"src": "startup_server_phase"}))
            agents_made.append(agent)
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
        for made in agents_made:
            try:
                made.close_oak()
            except Exception as exc:
                print(f"[run_real] OAK 표본 저장 정리 실패: {type(exc).__name__}", flush=True)
        try:
            if getattr(robot, "vlog", None) is not None:
                robot.vlog.stop_writer()                                # 마지막 상태를 한 번 더 쓰고 멈춘다
        except Exception as exc:                                        # noqa: BLE001
            print(f"[run_real] validation_status.json 정리 실패: {type(exc).__name__}", flush=True)
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
    raise SystemExit(main() or 0)
