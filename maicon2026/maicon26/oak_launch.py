"""OAK-D Lite 드라이버 기동·정리·토픽 확인 (6장) — ROS·depthai 를 import 하지 않는 순수 도우미. team/team_node.py 와 hal/ros_mil.py 가 쓴다.

  - 기동은 환경변수 MAICON_OAK=1 일 때만이다(기본 꺼짐). 명령은 oak.launch_cmd + oak.launch_args + '<params_file_arg>:=<파라미터 파일>'.
  - 실패하거나 5 s 안에 끝나 버려도 경고만 한다 — depth 와 OAK 컬러는 /ready 를 막지 않는 선택 항목이다(1장).
  - 자식은 새 프로세스 그룹으로 띄우고 끝날 때 그룹째 정리한다(좀비를 남기지 않는다).
  - 파라미터 이름은 depthai_ros_driver 판마다 다르다. 파일은 구 문서 이름으로 쓰고 위쪽 주석에 v3 대응표를 둔다 [실기 확인].
"""
from __future__ import annotations

import os
import signal
import time
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

PROFILES = ("rgb_lowpower", "rgbd")
EARLY_EXIT_S = 5.0


def oak_enabled(env) -> bool:
    return str(env.get("MAICON_OAK", "")).strip() == "1"


def params_path(oak_cfg, root) -> Path:
    """oak.profile 이 가리키는 파라미터 파일. 프로파일 이름은 파일 이름이다(rgb_lowpower | rgbd)."""
    name = str(oak_cfg.profile)
    if name not in PROFILES:
        raise ValueError(f"oak.profile 은 {' | '.join(PROFILES)} 중 하나 (받은 값: {name!r})")
    p = Path(root) / "config" / "oak" / f"{name}.yaml"
    if not p.is_file():
        raise FileNotFoundError(f"OAK 파라미터 파일이 없다: {p}")
    return p


def launch_command(oak_cfg, root) -> List[str]:
    """ros2 launch depthai_ros_driver camera.launch.py <launch_args…> params_file:=<파일>"""
    cmd = [str(c) for c in oak_cfg.launch_cmd] + [str(a) for a in oak_cfg.launch_args]
    return cmd + [f"{oak_cfg.params_file_arg}:={params_path(oak_cfg, root).resolve()}"]


def child_env(base_env) -> Dict[str, str]:
    """자식 프로세스 환경: 부모 환경 + DEPTHAI_TELEMETRY=0(경기장 네트워크로 텔레메트리를 내보내지 않는다)."""
    env = dict(base_env)
    env["DEPTHAI_TELEMETRY"] = "0"
    return env


def watch_early_exit(proc, cmd: Sequence[str], report: Callable[[str], None] = print, seconds: float = EARLY_EXIT_S,
                     sleep: Callable[[float], None] = time.sleep, clock: Callable[[], float] = time.monotonic) -> bool:
    """프로세스가 seconds 안에 끝나 버리면 경고하고 True. 계속 떠 있으면 False. 어떤 경우에도 예외를 올리지 않는다."""
    try:
        t0 = clock()
        while clock() - t0 < seconds:
            rc = proc.poll()
            if rc is not None:
                report(f"[oak] WARN OAK 드라이버가 {clock() - t0:.1f}s 만에 끝났다(종료 코드 {rc}): {' '.join(cmd)} — "
                       f"USB/Docker 장치 접근과 launch 인자를 확인. 계속 진행한다(OAK 는 선택 항목)")
                return True
            sleep(0.2)
    except Exception as e:                                            # pragma: no cover
        report(f"[oak] WARN OAK 드라이버 감시 실패: {type(e).__name__}")
    return False


def stop_process_group(proc, sig_wait_s: float = 8.0, kill_wait_s: float = 3.0, report: Callable[[str], None] = print) -> None:
    """프로세스 그룹째 정리: SIGINT(정상 종료) → 시간이 지나면 SIGKILL. 이미 끝났으면 아무것도 안 한다."""
    if proc is None or proc.poll() is not None:
        return
    group = None
    try:
        if os.name == "posix":
            group = os.getpgid(proc.pid)
    except (OSError, ProcessLookupError):
        group = None

    def send(sig):
        try:
            if group is not None:
                os.killpg(group, sig)
            elif sig == signal.SIGINT:
                proc.terminate()
            else:
                proc.kill()
        except (OSError, ProcessLookupError):
            pass
    send(signal.SIGINT)
    try:
        proc.wait(timeout=sig_wait_s)
        return
    except Exception:
        pass
    send(signal.SIGKILL)
    try:
        proc.wait(timeout=kill_wait_s)
    except Exception:
        report("[oak] WARN OAK 드라이버 프로세스를 정리하지 못했다")


# ---------------------------------------------------------------------------------------------- 토픽 확인
def wanted_topics(cfg, include_rgb: bool = True) -> List[str]:
    """설정에서 읽는 OAK 토픽: 컬러(oak.rgb_topic, oak.rgb_info_topic)와, 깊이를 쓰면 depth.topic·depth.info_topic."""
    out: List[str] = []
    if include_rgb:
        out += [cfg.oak.rgb_topic, cfg.oak.rgb_info_topic]
    if cfg.depth.enable:
        out += [cfg.depth.topic, cfg.depth.info_topic]
    return [t for t in dict.fromkeys(out) if t]


def check_topics(graph: Iterable[Tuple[str, Sequence[str]]], wanted: Sequence[str],
                 report: Callable[[str], None] = print) -> List[str]:
    """ROS 그래프(이름, 타입들)에 설정한 토픽이 없으면, 이름에 'oak' 이 든 토픽과 타입을 모두 경고로 낸다
    (`ros2 topic list -t | grep oak` 와 같은 정보). 막지 않는다. 없는 토픽 목록을 돌려준다."""
    names = {n: list(t) for n, t in graph}
    missing = [w for w in wanted if w not in names and ("/" + w.lstrip("/")) not in names]
    if missing:
        oaks = sorted((n, t) for n, t in names.items() if "oak" in n.lower())
        listing = "; ".join(f"{n} [{', '.join(t) or '?'}]" for n, t in oaks) if oaks else "없음 — 드라이버가 떠 있는지 확인(ros2 topic list | grep oak)"
        report(f"[oak] WARN 설정한 토픽이 ROS 그래프에 없다: {', '.join(missing)} | 이름에 oak 이 든 토픽: {listing} "
               f"→ oak.rgb_topic / oak.rgb_info_topic / depth.topic 설정을 실제 이름으로 고칠 것")
    return missing
