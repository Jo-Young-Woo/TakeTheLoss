"""Real-robot startup checks. No ROS imports and no motion commands."""
from __future__ import annotations

import time
import importlib
import logging
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, FrozenSet, Iterable, List, Optional, Tuple
from .link.secrets import redacted_output

log = logging.getLogger("maicon26.preflight")

# ---- 저하 플래그(1장). 플래그마다 지키는 곳은 한 군데다: no_drive=hal/wingbot._publish_cmd(+nav/degraded.NoDriveStack),
#      no_vision=mission/manager._ingest_vision(+run_real 첫 추론), no_fire=hal/wingbot 의 짐벌·발사 서비스 호출 두 곳.
FLAG_NO_DRIVE, FLAG_NO_VISION, FLAG_NO_FIRE = "no_drive", "no_vision", "no_fire"
ALL_FLAGS = (FLAG_NO_DRIVE, FLAG_NO_VISION, FLAG_NO_FIRE)

# 준비 항목 갈래. 이름은 hal/wingbot.py WingbotRobot.readiness() 와 hal/ros_mil.py RosMilTransport.readiness() 가 돌려주는 것이다.
REQUIRED_NAMES = frozenset({"state", "match", "match_mode"})                  # 대회 API 자체 — 없으면 경기가 안 된다
VISION_NAMES = frozenset({"image", "camera_info", "camera_calibration"})      # → no_vision
FIRE_NAMES = frozenset({"service:gimbal", "service:fire", "gimbal_feedback"})  # → no_fire
OPTIONAL_NAMES = frozenset({"depth", "topic:oled"})                           # 경고만(OLED 는 운영자 표시일 뿐)
# 그 밖의 이름(scan, odom, imu, Nav2 액션 서버, map→base TF, multiple_publishers:*, transport, 처음 보는 이름)은
# 보수적으로 '주행' 갈래다 → no_drive.


@redacted_output
def load_llm_factory(spec: str):
    """Load the team's local adapter without inventing an organizer LLM API.

    Factory takes no arguments, returns complete(prompt, timeout)->str.
    Credentials and model deployment belong to the adapter/environment.
    """
    module, sep, name = spec.partition(":")
    if not sep or not module or not name:
        raise ValueError("LLM adapter must be module:function")
    client = getattr(importlib.import_module(module), name)()
    if not callable(getattr(client, "complete", None)):
        raise TypeError("LLM adapter requires complete(prompt, timeout)")
    return client


def validate_model_labels(detector, cfg) -> None:
    """Reject a generic COCO model: these labels are required by mission logic."""
    names = detector.model.names
    values = names.values() if isinstance(names, dict) else names
    available = {detector.names_map.get(str(n), str(n)) for n in values}
    required = set(cfg.perception.object_classes) | set(cfg.perception.target_classes) | {
        "UGV", "COMMAND_POST", "MAINT_DEPOT", "SUPPLY_BASE"}
    missing = sorted(required - available)
    if missing:
        raise ValueError("YOLO 모델의 필수 클래스 누락: " + ", ".join(missing))


def validate_real_config(cfg) -> None:
    if cfg.nav2.use_sim_time:
        raise ValueError("실기에서는 nav2.use_sim_time=false 이어야 합니다")
    topics = (cfg.hal.scan_topic, cfg.hal.odom_topic, cfg.hal.cmd_topic, cfg.nav2.cmd_topic)
    if any(not t.startswith("/") or t == "/" or "//" in t or any(c.isspace() for c in t) for t in topics):
        raise ValueError("실기 센서·주행 토픽은 절대 이름이어야 합니다(예: /scan, /cmd_vel_nav2)")
    if cfg.nav.backend == "nav2" and cfg.nav2.cmd_topic == cfg.hal.cmd_topic:
        raise ValueError("Nav2 출력과 로봇 /cmd_vel은 분리해야 합니다")
    if cfg.api_issues():
        raise ValueError("대회 API 한계를 넘는 설정: " + repr(cfg.api_issues()))
    if cfg.depth.enable:
        # 깊이(OAK)는 선택 항목이다 — 설정이 덜 됐다고 시작 자체를 막지 않는다(1장). 경고를 남기고 끈다.
        d = cfg.depth
        problem = None
        if not d.require_calibration or not d.calibration_verified or not d.optical_frame.strip():
            problem = "실기 깊이 영상은 require_calibration, calibration_verified와 실제 optical_frame 설정이 필요합니다"
        elif cfg.nav2.base_frame not in ("base_link", "base_footprint"):
            problem = "실기 깊이 TF 검증은 base_link 또는 base_footprint 프레임이 필요합니다"
        elif not all(math.isfinite(v) and v > 0 for v in (d.tf_tolerance_m, d.tf_tolerance_deg)):
            problem = "깊이 TF 오차 허용값은 유한한 양수여야 합니다"
        if problem:
            log.warning("depth 설정 문제로 depth.enable=False 로 바꿉니다: %s", problem)
            d.enable = False
            if hasattr(cfg.nav2, "publish_depth_obstacles"):
                cfg.nav2.publish_depth_obstacles = False       # 깊이를 끄면 Nav2 장애물 입력에서도 뺀다
    lock_depth_navigation(cfg)
    oak = getattr(cfg, "oak", None)
    if oak is not None and getattr(oak, "profile", None) == "rgbd" and not cfg.depth.enable:
        log.warning("oak.profile=rgbd 인데 깊이를 쓰지 않으므로(depth.enable=False) rgb_lowpower 로 바꿉니다 — 스테레오 연산·USB 대역을 아낀다")
        oak.profile = "rgb_lowpower"


def lock_depth_navigation(cfg) -> List[str]:
    """실기에서는 OAK 깊이를 장애물 항법에 쓰지 않는다 — 항법 장애물 입력은 2D LiDAR(점유격자)뿐이다.
    꺼 두는 것: Nav2 로 보내는 깊이 장애물(nav2.publish_depth_obstacles), 항법 층에 얹는 3D 장애물(map3d.use_for_nav), 전방 깊이 정지(map3d.guard_enable).
    깊이 영상의 거리 측정(map3d.use_range)·발사 전 3D 확인은 항법이 아니라 그대로 둔다. depth.nav_allowed=True 로 명시하면 건드리지 않는다.
    바꾼 키 이름 목록을 돌려준다(예외를 던지지 않는다)."""
    changed: List[str] = []
    if getattr(cfg.depth, "nav_allowed", False):
        return changed
    for sec_name, key in (("map3d", "use_for_nav"), ("map3d", "guard_enable"), ("nav2", "publish_depth_obstacles")):
        sec = getattr(cfg, sec_name, None)
        if sec is not None and getattr(sec, key, False):
            log.warning("실기에서는 OAK 깊이를 장애물 항법에 쓰지 않습니다 — %s.%s=True 를 False 로 바꿉니다(허용하려면 depth.nav_allowed=true)", sec_name, key)
            try:
                setattr(sec, key, False)
                changed.append(f"{sec_name}.{key}")
            except Exception:
                pass
    return changed


def classify_reason(reason: str) -> str:
    """준비 누락 항목 하나의 갈래: required | vision | fire | optional | drive. 모르는 이름은 drive(보수적)."""
    r = str(reason).strip()
    if r in REQUIRED_NAMES:
        return "required"
    if r in VISION_NAMES:
        return "vision"
    if r in FIRE_NAMES:
        return "fire"
    if r in OPTIONAL_NAMES or r.startswith("depth:") or r.startswith("oak"):
        return "optional"
    return "drive"


@dataclass(frozen=True)
class ReadyState:
    """wait_ready 의 결과. degraded=False 면 모든 항목이 제한시간 안에 갖춰진 것(기존과 같다)."""
    degraded: bool = False
    flags: FrozenSet[str] = frozenset()                  # no_drive / no_vision / no_fire
    reasons: Tuple[str, ...] = ()                        # 누락 항목 전체
    required_missing: Tuple[str, ...] = ()               # state·match·match_mode — 있으면 /ready 를 보낼 수 없다
    optional_missing: Tuple[str, ...] = ()

    def summary(self) -> str:
        return "; ".join(self.reasons) if self.reasons else "(없음)"


def classify_readiness(reasons: Iterable[str]) -> ReadyState:
    """누락 항목들을 필수/주행/시각·사격/선택 네 갈래로 나눈다. 비어 있으면 degraded=False."""
    rs: List[str] = [str(r) for r in reasons]
    required, optional, flags = [], [], set()
    for r in rs:
        kind = classify_reason(r)
        if kind == "required":
            required.append(r)
        elif kind == "optional":
            optional.append(r)
        elif kind == "vision":
            flags.add(FLAG_NO_VISION)
        elif kind == "fire":
            flags.add(FLAG_NO_FIRE)
        else:
            flags.add(FLAG_NO_DRIVE)
    return ReadyState(degraded=bool(rs), flags=frozenset(flags), reasons=tuple(rs),
                      required_missing=tuple(required), optional_missing=tuple(optional))


def degraded_oled_text(flags: Iterable[str]) -> str:
    """OLED 한 줄(21자). 'DEGRADED NO_DRIVE'. 플래그가 여럿이면 'DEG DRIVE/VISION/FIRE'. 없으면 빈 문자열."""
    fl = [f for f in ALL_FLAGS if f in set(flags)]
    if not fl:
        return ""
    if len(fl) == 1:
        return "DEGRADED " + fl[0].upper()
    return "DEG " + "/".join(f.split("_", 1)[1].upper() for f in fl)


def not_ready_reasons(robot, stack) -> List[str]:
    """robot.readiness() 와 Nav2 상태에서 누락 항목 목록을 만든다(wait_ready 와 저하 재확인이 같이 쓴다)."""
    ok, reasons = robot.readiness()
    reasons = list(reasons)
    if "depth" in reasons:
        detail = getattr(robot, "stats", {}).get("depth_error")
        if detail:
            reasons.append("depth: " + str(detail))
    client = getattr(stack, "client", None)
    if client is not None:
        if not client.available():
            reasons.append("Nav2 액션 서버 미준비")
        if client.lookup_map_to_base() is None:
            reasons.append("map→base TF 미수신")
    if not ok and not reasons:
        reasons.append("센서 미준비")
    return reasons


def wait_ready(robot, stack, timeout: float, *, clock: Callable = time.monotonic,
               sleep: Callable = time.sleep, should_quit: Callable = lambda: False,
               report: Optional[Callable[[str], None]] = None, report_every_s: float = 5.0) -> ReadyState:
    """준비를 기다린다.

    모두 갖춰지면 즉시 ReadyState(degraded=False). 제한시간을 넘기면:
      - 필수 항목(state·match·match_mode, 대회 API)이 빠졌으면 TimeoutError — 필수 항목은 시간이 지났다고 ready 가 되지 않는다.
      - 필수 항목이 다 있으면 ReadyState(degraded=True, flags=…, reasons=…) 를 돌려준다 → /ready 를 보낸다.
        (LiDAR·Nav2·깊이 하나가 늦어 그 경기가 0점이 되지 않게 한다. M1·M6 는 주행 없이도 점수가 난다.)
    필수 항목이 빠진 동안은 report_every_s 마다 누락 항목을 report(없으면 logging)로 남긴다.
    """
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("ready timeout must be finite and positive")
    emit = report or log.warning
    deadline = clock() + timeout
    next_report = -math.inf
    while not should_quit():
        reasons = not_ready_reasons(robot, stack)
        if not reasons:
            return ReadyState()
        state = classify_readiness(reasons)
        now = clock()
        if state.required_missing and now >= next_report:
            next_report = now + report_every_s
            try:
                emit("[준비 대기] 필수 누락: " + ", ".join(state.required_missing) + " | 전체: " + state.summary())
            except Exception:
                pass
        if now >= deadline:
            if state.required_missing:
                raise TimeoutError("준비완료 미전송: " + "; ".join(reasons))
            return state
        sleep(0.1)
    raise InterruptedError("준비 확인 중 종료 요청")


def clear_standby_restore(robot, path: str) -> None:
    """A new standby must not reuse a previous match's recent start timestamp."""
    st = robot.status()
    if path and st.match_mode == "mission" and st.match_cmd in ("idle", "standby"):
        Path(path).unlink(missing_ok=True)
