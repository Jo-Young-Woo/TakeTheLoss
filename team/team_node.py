#!/usr/bin/env python3
"""대회 규정 진입점 ~/team/team_node.py (2차 사전교육 PDF p18).

  - 운영 프로그램이 경기 시작 대기(standby) 때 이 파일을 자동 실행한다. 경기 종료 report 뒤 stop 을 받으면 노드가 끝나야 한다 → 한 경기만 치르고 종료(--matches 1).
  - 비정상 종료는 경기당 최대 3회 자동 재실행된다. --restore 는 시계만 저장하므로 현재 실기 진입점은 경기 도중 새 프로세스 임무 시작을 차단한다(지도·앵커·장부 복원이 추가로 필요).
  - step() 은 0.1 s 이내, AI 추론은 별도 스레드: Agent.tick() 이 그 step 이고 검출은 AsyncDetector 스레드다. 느려지면 stats['tick_over_budget'] 가 센다.
  - 하위 프로그램(SLAM/Nav2 launch)은 팀 노드가 실행·관리한다 → TEAM_LAUNCH_NAV=1 이면 ros2 launch 를 자식 프로세스로 올리고 종료 때 끈다.

설정은 환경변수(운영 환경에서 인자를 못 줄 때)나 인자로:
  MAICON_SERVER_URL 경기 서버 주소(legacy MAICON_SERVER도 지원; 현장 안내)  MAICON_CODE 임무코드(legacy MAICON_MISSION_CODE도 지원; 로그에 남기지 않는다)
  MAICON_TEAM / MAICON_SLOT  보통 불필요 — 서버 /me 의 slot 으로 정해진다   MAICON_REHEARSAL=1 이면 리허설 창구(/api/rehearsal/)
  MAICON_CONFIG  설정 파일(기본 <프로젝트>/config/waveshare_real.json)     MAICON_DETECTOR  YOLO 가중치(.pt/.engine)
  MAICON_NAV     nav2 | builtin (기본 nav2)                              TEAM_LAUNCH_NAV  1 이면 Nav2/SLAM launch 를 같이 띄운다
추가 인자는 scripts/run_real.py 로 그대로 넘어간다.
"""
import os
import runpy
import signal
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))


def _launch_nav(argv):
    """SLAM + Nav2를 관리한다. 기동 실패 시 사전 점검이 준비완료를 차단한다."""
    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
    from maicon26.launch_profile import nav_launch_command
    command = nav_launch_command(argv, os.environ, root=ROOT)
    if command is None:
        return None
    try:
        return subprocess.Popen(command, start_new_session=(os.name == "posix"))
    except OSError as e:
        print(f"[team_node] WARN Nav2 launch 실패: {e}", flush=True)
        return None


def main() -> int:
    env = os.environ.get
    argv = [os.path.join(ROOT, "scripts", "run_real.py")]
    if env("MAICON_SERVER_URL") or env("MAICON_SERVER"):
        argv += ["--server", env("MAICON_SERVER_URL") or env("MAICON_SERVER")]
    elif not any(arg == "--server" or arg.startswith("--server=") for arg in sys.argv[1:]) and "--preflight-only" not in sys.argv:
        print("[team_node] MAICON_SERVER_URL(legacy MAICON_SERVER) 환경변수 또는 --server 가 필요하다", flush=True)
        return 2
    cfgp = env("MAICON_CONFIG") or os.path.join(ROOT, "config", "waveshare_real.json")
    argv += ["--config", cfgp, "--nav", env("MAICON_NAV", "nav2"), "--matches", "1", "--restore"]
    for flag, key in (("--team", "MAICON_TEAM"), ("--slot", "MAICON_SLOT"), ("--detector", "MAICON_DETECTOR"),
                      ("--names-map", "MAICON_NAMES_MAP"), ("--llm-factory", "MAICON_LLM_FACTORY")):
        if env(key):
            argv += [flag, env(key)]
    if env("MAICON_REHEARSAL") == "1":
        argv += ["--rehearsal"]
    ov = os.path.join(ROOT, "config", "field_overrides.json")
    if os.path.exists(ov):
        argv += ["--overrides", ov]
    argv += sys.argv[1:]
    child = _launch_nav(argv[1:]) if env("TEAM_LAUNCH_NAV") == "1" else None
    old_argv = sys.argv
    sys.argv = argv
    try:
        runpy.run_path(argv[0], run_name="__main__")
        return 0
    except SystemExit as e:
        return int(e.code or 0)
    finally:
        sys.argv = old_argv
        if child is not None and child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)  # ros2 launch shuts down its children in order
            except (OSError, AttributeError):
                child.terminate()
            try:
                child.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                if os.name == "posix":
                    os.killpg(os.getpgid(child.pid), signal.SIGKILL)
                else:
                    child.kill()
                child.wait(timeout=3.0)


if __name__ == "__main__":
    sys.exit(main())
