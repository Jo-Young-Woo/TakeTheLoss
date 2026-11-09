"""Real-robot startup checks. No ROS imports and no motion commands."""
from __future__ import annotations

import time
import importlib
import math
from pathlib import Path
from typing import Callable
from .link.secrets import redacted_output


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
        d = cfg.depth
        if not d.require_calibration or not d.calibration_verified or not d.optical_frame.strip():
            raise ValueError("실기 깊이 영상은 require_calibration, calibration_verified와 실제 optical_frame 설정이 필요합니다")
        if cfg.nav2.base_frame not in ("base_link", "base_footprint"):
            raise ValueError("실기 깊이 TF 검증은 base_link 또는 base_footprint 프레임이 필요합니다")
        if not all(math.isfinite(v) and v > 0 for v in (d.tf_tolerance_m, d.tf_tolerance_deg)):
            raise ValueError("깊이 TF 오차 허용값은 유한한 양수여야 합니다")


def wait_ready(robot, stack, timeout: float, *, clock: Callable = time.monotonic,
               sleep: Callable = time.sleep, should_quit: Callable = lambda: False) -> None:
    """Missing inputs never turn into ready merely because time elapsed."""
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("ready timeout must be finite and positive")
    deadline = clock() + timeout
    while not should_quit():
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
        if ok and not reasons:
            return
        if clock() >= deadline:
            raise TimeoutError("준비완료 미전송: " + "; ".join(reasons or ["센서 미준비"]))
        sleep(0.1)
    raise InterruptedError("준비 확인 중 종료 요청")


def clear_standby_restore(robot, path: str) -> None:
    """A new standby must not reuse a previous match's recent start timestamp."""
    st = robot.status()
    if path and st.match_mode == "mission" and st.match_cmd in ("idle", "standby"):
        Path(path).unlink(missing_ok=True)
