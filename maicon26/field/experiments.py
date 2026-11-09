"""현장 실험 F01~F10. 각 실험 = 데이터 수집(run) + 분석(analyze: 지표·갱신 후보·판정).

!! 실제 로봇에서 실행해 본 적이 없다. 시뮬 로봇(field/runner.py make_sim_ctx)에서 '코드가 도는지'만 확인했고, 그 수치는 현장 값이 아니다.
시뮬에서는 환경 배치를 훅이 자동으로 하고(ctx.confirm), 현장에서는 사람에게 안내문을 보여 주고 Enter 를 기다린다.
결과는 JSON 기록(field/records/<날짜>/F0x_<시각>.json)과 갱신 후보(suggestions)로 남고, 사람이 확인한 뒤 `field_exp.py apply` 로 반영한다.

공통 약속
  - 좌표: 월드(경기장) 좌표. 로봇이 시작한 자세를 start_x/start_y/start_th 로 입력받아 오도메트리를 얹는다(SLAM 없이 짧은 시험용).
  - 시간: robot.now() 시계. 센서 stamp 도 같은 시계라고 본다(hal/base.py 계약). F01 이 그것을 확인한다.
  - 안전: 구동 명령은 느리게(≤ 0.2 m/s), 끝나면 반드시 정지. 레이저는 --allow-laser 없이는 절대 발사하지 않는다.
"""
from __future__ import annotations

import datetime as _dt
import json
import math
import os
import statistics
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from ..calib.depth import calibrate_mount_from_floor, median_depth
from ..config import Config
from ..geom3d import Intrinsics
from ..hal.base import Robot
from ..nav.stall import StallDetector
from ..perception.camera import lidar_range_at
from ..perception.depth_fusion import DepthFusion
from ..types import Pose2D
from . import tunables as tun


# ====================================================================== 공통 부품
@dataclass
class Suggestion:
    path: str
    value: Any
    note: str = ""


@dataclass
class Analysis:
    verdict: str = "INFO"                    # PASS | FAIL | CHECK | INFO
    metrics: Dict[str, Any] = field(default_factory=dict)
    suggestions: List[Suggestion] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


def jsonable(x: Any) -> Any:
    if isinstance(x, dict):
        return {str(k): jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return jsonable(x.tolist())
    if isinstance(x, (np.floating, float)):
        f = float(x)
        return None if (math.isnan(f) or math.isinf(f)) else f
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    return x


def pstats(a: List[float]) -> Dict[str, Optional[float]]:
    if not a:
        return {"n": 0, "min": None, "p50": None, "p95": None, "max": None}
    s = sorted(a)
    return {"n": len(s), "min": s[0], "p50": s[len(s) // 2], "p95": s[min(int(0.95 * len(s)), len(s) - 1)], "max": s[-1]}


def rel_pose(o0: Pose2D, od: Pose2D) -> Pose2D:
    """오도메트리 od 를 o0 시점의 로봇 좌표로 환산(o0⁻¹ ⊕ od). o0 가 (0,0,0) 이 아니어도 맞다."""
    c, s = math.cos(o0.th), math.sin(o0.th)
    dx, dy = od.x - o0.x, od.y - o0.y
    d = od.th - o0.th
    return Pose2D(c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(d), math.cos(d)))


class NeedInput(KeyError):
    """대화형이 아닌데 답이 필요한 질문이 있다."""


class Ctx:
    """실험이 쓰는 환경. 시뮬/실기 차이는 여기서 흡수한다."""

    def __init__(self, robot: Robot, cfg: Config, out_dir: str, interactive: bool = False,
                 answers: Optional[Dict[str, Any]] = None, hooks: Optional[Dict[str, Callable[["Ctx"], None]]] = None,
                 advance: Optional[Callable[[float], None]] = None, allow_laser: bool = False,
                 say: Callable[[str], None] = print, ask_fn: Callable[[str], str] = input, robot_kind: str = "real") -> None:
        self.robot, self.cfg, self.out_dir = robot, cfg, out_dir
        self.interactive, self.answers, self.hooks = interactive, answers or {}, hooks or {}
        self.advance, self.allow_laser, self._say, self._ask_fn = advance, allow_laser, say, ask_fn
        self.robot_kind = robot_kind
        self.given: Dict[str, Any] = {}              # 지금까지 받은 답(훅이 읽는다)
        self.unsupported: List[str] = []             # 스텁이라 못 읽은 센서

    # ---- 출력·입력
    def say(self, msg: str) -> None:
        self._say(msg)

    def ask(self, key: str, prompt: str, default: Any = None, cast: Callable[[str], Any] = float) -> Any:
        """미리 준 답(answers) > 대화형 입력 > default. default 가 None 이고 답이 없으면 NeedInput."""
        if key in self.answers:
            v = self.answers[key]
            v = v(self) if callable(v) else v
            self.given[key] = cast(v) if isinstance(v, str) else v
            return self.given[key]
        if self.interactive:
            raw = self._ask_fn(f"  ? {prompt}" + (f" [{default}]" if default is not None else "") + ": ").strip()
            if raw == "":
                if default is None:
                    raise NeedInput(key)
                self.given[key] = default
                return default
            self.given[key] = cast(raw)
            return self.given[key]
        if default is None:
            raise NeedInput(key)
        self.given[key] = default
        return default

    def confirm(self, key: str, text: str) -> None:
        """현장: 사람에게 배치/조작 안내를 보이고 Enter 를 기다린다. 시뮬: 훅이 같은 일을 자동으로 한다."""
        self.say(f"  ▶ {text}")
        hook = self.hooks.get(key)
        if hook is not None:
            hook(self)
            self.say("    (시뮬 자동 배치)")
        elif self.interactive:
            self._ask_fn("    준비되면 Enter ")

    # ---- 시간·구동
    def now(self) -> float:
        return self.robot.now()

    def pump(self, secs: float, on_tick: Optional[Callable[[float], Optional[bool]]] = None, dt: float = 0.05) -> None:
        """secs 동안 dt 간격으로 진행하며 on_tick(now) 을 부른다. on_tick 이 True 를 돌려주면 중단."""
        t_end = self.now() + secs
        while self.now() < t_end:
            if self.advance is not None:
                self.advance(dt)
            else:
                time.sleep(dt)
            if on_tick is not None and on_tick(self.now()):
                break

    def drive(self, v: float, w: float = 0.0) -> None:
        self.robot.set_velocity(float(v), float(w))

    def stop(self) -> None:
        self.robot.set_velocity(0.0, 0.0)

    def safe(self, getter: Callable[[], Any], name: str) -> Any:
        try:
            return getter()
        except NotImplementedError:
            if name not in self.unsupported:
                self.unsupported.append(name)
            return None

    def start_pose(self, default: Tuple[float, float, float] = (1.0, 2.5, 0.0)) -> Tuple[float, float, float]:
        x = self.ask("start_x", "시작 x (m, 경기장 좌표: 왼쪽 아래 모서리가 원점)", default[0])
        y = self.ask("start_y", "시작 y (m)", default[1])
        th = self.ask("start_th_deg", "시작 헤딩 (deg, +x 방향 0, 반시계 +)", math.degrees(default[2]))
        return float(x), float(y), math.radians(float(th))

    def world_pose(self, start: Tuple[float, float, float], odom: Pose2D) -> Pose2D:
        """시작 자세(월드) ⊕ 오도메트리((0,0,0) 시작) = 월드 자세."""
        x0, y0, t0 = start
        c, s = math.cos(t0), math.sin(t0)
        return Pose2D(x0 + c * odom.x - s * odom.y, y0 + s * odom.x + c * odom.y, t0 + odom.th)


class Experiment:
    id = ""
    title = ""
    minutes = 5
    needs: List[str] = []
    safety = ""
    procedure: List[str] = []

    def run(self, ctx: Ctx) -> Dict[str, Any]:                       # pragma: no cover - 추상
        raise NotImplementedError

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:   # pragma: no cover - 추상
        raise NotImplementedError


def _depth_ticker(ctx: Ctx, sink: Callable[[Any, Any, float], None]) -> Callable[[float], Optional[bool]]:
    """새 깊이 프레임이 올 때마다 sink(depth_frame, latest_scan, now) 를 부르는 on_tick."""
    last = {"stamp": None}

    def tick(now: float) -> Optional[bool]:
        df = ctx.safe(ctx.robot.get_depth, "depth")
        if df is not None and df.stamp != last["stamp"]:
            last["stamp"] = df.stamp
            sink(df, ctx.safe(ctx.robot.get_scan, "scan"), now)
        return None
    return tick


# ====================================================================== F01 센서 스트림 상태
class F01_SensorStreams(Experiment):
    id, title, minutes = "F01", "센서 주기·지연·시계 확인 (LiDAR/카메라/깊이)", 3
    needs = ["robot 정지", "센서 드라이버 실행"]
    procedure = ["로봇을 정지시킨 채 센서 드라이버만 켠다(Agent 불필요).", "측정 시간 동안 움직이지 않는다."]

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        dur = float(ctx.ask("duration_s", "측정 시간 (s)", 20.0))
        arr: Dict[str, List[Tuple[float, float]]] = {"scan": [], "frame": [], "depth": []}
        last: Dict[str, Any] = {}
        getters = {"scan": ctx.robot.get_scan, "frame": ctx.robot.get_frame, "depth": ctx.robot.get_depth}

        def tick(now: float) -> None:
            for name, g in getters.items():
                s = ctx.safe(g, name)
                if s is not None and s.stamp != last.get(name):
                    last[name] = s.stamp
                    arr[name].append((now, float(s.stamp)))
        ctx.pump(dur, tick, dt=0.02)                       # 센서 주기보다 훨씬 촘촘히 읽어야 도착 간격이 왜곡되지 않는다(50 Hz)
        return {"duration_s": dur, "arrivals": arr, "unsupported": list(ctx.unsupported)}

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis()
        per: Dict[str, Any] = {}
        for name, rows in data["arrivals"].items():
            if len(rows) < 2:
                per[name] = {"n": len(rows)}
                continue
            ts = [r[0] for r in rows]
            iv = [b - a_ for a_, b in zip(ts[:-1], ts[1:])]
            d = [r[0] - r[1] for r in rows]
            med = statistics.median(d)
            per[name] = {"n": len(rows), "hz": round((len(rows) - 1) / max(ts[-1] - ts[0], 1e-6), 2), "interval": pstats(iv),
                         "now_minus_stamp": pstats(d), "clock": "same" if abs(med) <= 1.0 else "offset",
                         "offset_s": None if abs(med) <= 1.0 else round(min(d), 3)}
        a.metrics = per
        miss = [n for n in ("scan", "frame", "depth") if per.get(n, {}).get("n", 0) < 2]
        if data.get("unsupported"):
            a.notes.append(f"읽지 못한 센서(스텁/미연결): {data['unsupported']}")
        for n, m in per.items():
            if m.get("clock") == "offset":
                a.notes.append(f"{n}: stamp 가 robot.now() 시계가 아니다(offset≈{m['offset_s']} s). Agent 가 변환하지만 HAL 이 now() 시계로 채우는 것이 맞다(hal/clock.py).")
        sc, fr, dp = per.get("scan", {}), per.get("frame", {}), per.get("depth", {})
        if sc.get("n", 0) >= 20:
            p95 = sc["interval"]["p95"]
            a.suggestions.append(Suggestion("mission.lidar_stale_s", round(min(max(4 * p95, 0.3), 1.5), 2),
                                            f"LiDAR 간격 p95={p95:.3f}s 의 4배(0.3~1.5 s). 현재 {cfg.mission.lidar_stale_s}"))
        if fr.get("n", 0) >= 20:
            p95 = fr["interval"]["p95"]
            a.suggestions.append(Suggestion("mission.vision_stale_s", round(min(max(4 * p95, 0.5), 2.0), 2),
                                            f"카메라 간격 p95={p95:.3f}s 의 4배. 검출기(YOLO) 지연은 별도로 더해야 한다. 현재 {cfg.mission.vision_stale_s}"))
        if fr.get("n", 0) >= 20 and dp.get("n", 0) >= 20:
            m = max(fr["interval"]["p95"], dp["interval"]["p95"])
            a.suggestions.append(Suggestion("map3d.sync_max_dt", round(min(max(2 * m, 0.1), 0.5), 2), "RGB↔깊이 짝짓기 허용 시각차 = 두 스트림 간격 p95 의 2배"))
        if sc.get("hz") is not None and sc["hz"] < 5:
            a.notes.append(f"LiDAR {sc['hz']} Hz — 기대(10 Hz)보다 많이 낮다")
        a.verdict = "PASS" if not miss and (sc.get("hz") or 0) >= 5 else "CHECK"
        if miss:
            a.notes.append(f"신호가 없는 센서: {miss}")
        return a


# ====================================================================== F02 깊이 카메라 설치 각도(바닥 평면)
class F02_DepthFloorMount(Experiment):
    id, title, minutes = "F02", "깊이 카메라 설치 roll·pitch·높이 (바닥 평면 맞춤)", 5
    needs = ["평평한 빈 바닥(앞 1.5 m 안에 박스·벽 없음)", "로봇 정지"]
    procedure = ["로봇을 평평한 빈 바닥에 둔다. 앞 1.5 m 안에 박스·사람·벽이 없어야 한다(있으면 roll 이 2~4° 틀어진다).",
                 "16장 이상을 모아 중앙값으로 푼다."]

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        ctx.confirm("empty_floor", "로봇을 평평한 빈 바닥에 정지시킨다(앞 1.5 m 비움)")
        ctx.pump(1.5)                                       # 정착 대기
        n = int(ctx.ask("frames", "사용할 깊이 프레임 수", 16, int))
        frames: List[np.ndarray] = []
        K: List[Any] = [None]

        def sink(df, scan, now):
            frames.append(np.asarray(df.depth, dtype=np.float32))
            K[0] = df.K
        ctx.pump(max(30.0, n * 0.6), self._stop_when(ctx, n, frames, sink))
        if len(frames) < 4:
            return {"error": "깊이 프레임이 부족하다(센서·토픽 확인)", "frames": len(frames)}
        med = median_depth(frames)
        intr = Intrinsics.from_K(K[0], med.shape[1], med.shape[0]) if K[0] else ctx.cfg.depth.intrinsics(med.shape[1], med.shape[0])
        max_r = float(ctx.ask("max_range", "바닥 점으로 쓸 최대 거리 (m) — 카메라가 낮으면 1.2 권장", 1.2))
        res = calibrate_mount_from_floor(med, intr, max_range=max_r, tol=0.008)
        os.makedirs(ctx.out_dir, exist_ok=True)
        npy = os.path.join(ctx.out_dir, f"F02_median_depth_{_dt.datetime.now().strftime('%H%M%S')}.npy")
        np.save(npy, med)
        return {"frames_used": len(frames), "shape": list(med.shape), "intrinsics": [intr.fx, intr.fy, intr.cx, intr.cy], "fit": res, "median_depth_npy": npy}

    @staticmethod
    def _stop_when(ctx: Ctx, n: int, frames: List[Any], sink: Callable) -> Callable[[float], Optional[bool]]:
        inner = _depth_ticker(ctx, sink)

        def tick(now: float) -> Optional[bool]:
            inner(now)
            return len(frames) >= n
        return tick

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis()
        if "fit" not in data:
            a.verdict, a.notes = "FAIL", [data.get("error", "데이터 없음")]
            return a
        f = data["fit"]
        ratio = f["inliers"] / max(f["n_points"], 1)
        a.metrics = {"roll_deg": round(f["roll_deg"], 2), "pitch_deg": round(f["pitch_deg"], 2), "height_m": round(f["height"], 3),
                     "inlier_ratio": round(ratio, 2), "rms_mm": round(f["rms_m"] * 1000, 1)}
        suspicious = abs(f["pitch_deg"]) > 15 or abs(f["roll_deg"]) > 10
        a.verdict = "PASS" if ratio >= 0.4 and f["rms_m"] < 0.01 and not suspicious else "CHECK"
        if suspicious:
            a.notes.append("기울기가 비정상적으로 크다 — 바닥이 아니라 벽·박스를 잡았을 수 있다. 빈 바닥에서 다시.")
        if ratio < 0.4:
            a.notes.append("바닥 인라이어 비율이 낮다 — 앞이 비었는지, max_range 를 줄여 보라")
        x, y, z = cfg.rig.depth_xyz
        a.suggestions += [Suggestion("rig.depth_rpy", [round(f["roll"], 5), round(f["pitch"], 5), cfg.rig.depth_rpy[2]],
                                     "roll, pitch 만 바닥에서 얻는다. yaw 는 그대로(벽 정면 영상 좌우 대칭으로 눈 확인)"),
                          Suggestion("rig.depth_xyz", [x, y, round(f["height"] - cfg.rig.base_z, 4)],
                                     "z 만 바닥에서(바닥 위 높이 − base_z). x, y 는 F09 줄자값·F04 로")]
        return a


# ====================================================================== F03 깊이 거리 정확도
class F03_DepthRangeAccuracy(Experiment):
    id, title, minutes = "F03", "깊이 거리 정확도·유효율 (평평한 벽, 줄자)", 10
    needs = ["평평한 벽 하나", "줄자", "로봇 정지"]
    procedure = ["평평한 벽 정면에 로봇을 정렬해 세운다(깊이 렌즈 → 벽 수직거리를 줄자로 잰다).",
                 "안내하는 거리마다 로봇을 옮기고 줄자 값을 입력한다. 거리마다 10프레임."]

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        raw = ctx.ask("distances", "벽까지 거리 목록 (m, 쉼표)", "0.5,1.0,1.5,2.0,3.0", str)
        ds = [float(x) for x in str(raw).split(",") if x.strip()]
        rows = []
        for d in ds:
            ctx.given["wall_d"] = d
            ctx.confirm("wall_at", f"깊이 렌즈에서 벽까지 약 {d:.2f} m 에 정지(벽에 정면)")
            tape = float(ctx.ask(f"tape_{d:.2f}", "줄자로 잰 실제 거리 (m)", d))
            ctx.pump(1.5)                                   # 정착 대기: 옮긴 직후의 진동·직전 위치의 낡은 프레임을 흘려보낸다
            zs: List[float] = []
            valid: List[float] = []

            def sink(df, scan, now, zs=zs, valid=valid):
                h, w = df.depth.shape
                roi = df.depth[int(h * 0.45):int(h * 0.55), int(w * 0.45):int(w * 0.55)]
                ok = np.isfinite(roi) & (roi > 0)
                valid.append(float(ok.mean()))
                if ok.any():
                    zs.append(float(np.median(roi[ok])))
            tk = _depth_ticker(ctx, sink)
            ctx.pump(8.0, lambda now: (tk(now), len(valid) >= 10)[1])
            rows.append({"nominal": d, "tape": tape, "n": len(zs), "z_median": float(np.median(zs)) if zs else None,
                         "z_std": float(np.std(zs)) if len(zs) > 1 else None, "valid_ratio": float(np.mean(valid)) if valid else 0.0})
        return {"rows": rows}

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis()
        rows = [r for r in data["rows"] if r["z_median"] is not None]
        per, ok_t, ks, ratios = [], [], [], []
        for r in rows:
            bias = r["z_median"] - r["tape"]
            std = r["z_std"] or 0.0
            good = abs(bias) < 0.03 and std < 0.02 and r["valid_ratio"] >= 0.8
            per.append({"tape": r["tape"], "bias_m": round(bias, 4), "std_m": round(std, 4), "valid": round(r["valid_ratio"], 2), "ok": good})
            ratios.append(r["z_median"] / r["tape"])
            if good:
                ok_t.append(r["tape"])
            if r["z_std"] is not None and r["tape"] > 0.3:
                ks.append(std / (r["tape"] ** 2))
        a.metrics = {"per_distance": per}
        if not rows:
            a.verdict, a.notes = "FAIL", ["유효한 측정이 없다(센서·토픽·벽 정렬 확인)"]
            return a
        med_ratio = statistics.median(ratios)
        if abs(math.log10(max(med_ratio, 1e-9))) > 0.5:
            a.notes.append(f"깊이/줄자 비 중앙값 {med_ratio:.3g} — 단위가 다르다. depth.unit_scale 점검")
            a.suggestions.append(Suggestion("depth.unit_scale", cfg.depth.unit_scale / med_ratio, "비율로 환산(10의 거듭제곱에 가까운지 확인)"))
        if ok_t:
            a.suggestions.append(Suggestion("depth.max_range", round(max(ok_t), 2), "편향<3 cm·표준편차<2 cm·유효율≥80 % 인 시험 거리 중 최대. 더 먼 거리는 시험하지 않았다"))
            if min(ok_t) <= 0.6:
                a.suggestions.append(Suggestion("depth.min_range", round(min(ok_t), 2), "같은 기준을 만족한 최소 시험 거리. 더 가까운 곳은 시험하지 않았다"))
        if ks:
            a.suggestions.append(Suggestion("depth.sim_noise_k", round(statistics.median(ks), 5), "표준편차/거리² 중앙값 — 시뮬 잡음 모델(σ=k·z²) 충실도용"))
        a.verdict = "PASS" if len(ok_t) >= 3 else "CHECK"
        return a


# ====================================================================== F04 깊이↔LiDAR 정합
class F04_DepthLidarAlignment(Experiment):
    id, title, minutes = "F04", "깊이↔LiDAR 정합(앞뒤 설치 오차) — 제자리 회전", 4
    needs = ["벽·박스가 3 m 안에 있는 곳(경기장 시작 거점)", "로봇 정지 가능한 자리(제자리 회전)"]
    procedure = ["경기장 시작 거점에 로봇을 두고 시작 자세를 입력한다.", "로봇이 제자리에서 약 0.5 rad/s 로 한 바퀴 돈다.",
                 "깊이 점의 LiDAR 평면 높이 부분을 같은 방위의 LiDAR 거리와 비교한다. rig.depth_xyz 의 x 를 바꾼 뒤 한 번 더 해서 잔차가 줄었는지 본다."]
    safety = "제자리 회전: 로봇 둘레 0.5 m 비우기"

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        start = ctx.start_pose()
        fu = DepthFusion(ctx.cfg)
        o0 = ctx.robot.get_odom()
        rate = float(ctx.ask("w_rad_s", "회전 속도 (rad/s)", 0.5))
        n_in = {"n": 0}

        def sink(df, scan, now):
            pose = ctx.world_pose(start, rel_pose(o0, ctx.robot.get_odom()))
            fu.ingest(df, df.stamp, pose, scan, pose)
            n_in["n"] += 1
        ctx.drive(0.0, rate)
        try:
            ctx.pump(2 * math.pi / abs(rate) + 1.0, _depth_ticker(ctx, sink))
        finally:
            ctx.stop()
        rep = fu.report()
        return {"start": list(start), "w": rate, "frames": n_in["n"], "report": rep, "rig_depth_xyz": list(ctx.cfg.rig.depth_xyz)}

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis()
        r = data["report"]
        resid, checks = r.get("lidar_resid_m"), r.get("lidar_checks", 0)
        a.metrics = {"resid_median_m": resid, "checks": checks, "frames": data["frames"], "occupied_voxels": r.get("occupied_voxels"), "bias_alarm": r.get("lidar_bias_alarm")}
        if resid is None or checks < 10:
            a.verdict = "CHECK"
            a.notes.append("정합 표본이 부족하다 — 3 m 안에 벽/박스가 있고 깊이·LiDAR 가 둘 다 들어오는지(F01)")
            return a
        a.notes.append("앞뒤(x) 어긋남만 본다. yaw·좌우(y) 오차는 거리 잔차로 안 나온다(평평한 벽에서는 yaw 가 거리를 안 바꾼다) → 벽 정면 영상 좌우 대칭·줄자로.")
        if abs(resid) < 0.02:
            a.verdict = "PASS"
        else:
            a.verdict = "CHECK"
            x, y, z = data["rig_depth_xyz"]
            a.suggestions.append(Suggestion("rig.depth_xyz", [round(x - resid, 4), y, z],
                                            f"잔차 {resid:+.3f} m: 실제 카메라가 모델보다 {-resid:+.3f} m 앞(+)에 있다는 뜻 → x 를 {-resid:+.3f} 보정. 적용 후 F04 를 다시 해 잔차가 0.02 아래로 줄었는지 확인"))
        return a


# ====================================================================== 낮은 물체·허상 공통 주행
def _log_layer(fu: DepthFusion, pose: Pose2D, scan, now: float, travel: float, log: List[Dict[str, Any]], region=None) -> Optional[float]:
    """3D 층(LiDAR 가 못 보는 장애물) 셀 수를 region 안/밖으로 나눠 한 줄 기록. 반환: LiDAR 정면 거리."""
    layer = fu.nav_layer()
    g = fu.map
    cells = outside = 0
    if layer is not None and layer.any():
        ys, xs = np.nonzero(layer)
        cx, cy = g.ox + (xs + 0.5) * g.res, g.oy + (ys + 0.5) * g.res
        if region is not None:
            inr = (cx >= region[0]) & (cx <= region[1]) & (cy >= region[2]) & (cy <= region[3])
            cells, outside = int(inr.sum()), int((~inr).sum())
        else:
            outside = int(len(xs))
    lf = None if scan is None else float(scan.min_in_sector(0.0, math.radians(10)))
    log.append({"t": now, "travel": travel, "layer_in_region": cells, "layer_outside": outside, "lidar_front": lf,
                "front_clearance": fu.front_clearance(pose, 2.0), "x": pose.x, "y": pose.y})
    return lf


def _drive_and_map(ctx: Ctx, start, fu: DepthFusion, v: float, w: float, secs: float, log: List[Dict[str, Any]], region=None,
                   travel_stop: Optional[float] = None, lidar_abort_m: float = 0.35) -> Dict[str, Any]:
    """(v, w) 로 secs 동안 구동하며 깊이를 3D 지도에 쌓고 0.5 s 마다 층 셀 수를 log 에 남긴다. 오도메트리로 자세를 얹는다(짧은 시험용)."""
    o0 = ctx.robot.get_odom()
    state = {"travel": 0.0, "t_last": -1.0, "aborted": None}

    def sink(df, scan, now):
        rel = rel_pose(o0, ctx.robot.get_odom())
        pose = ctx.world_pose(start, rel)
        state["travel"] = math.hypot(rel.x, rel.y)
        fu.ingest(df, df.stamp, pose, scan, pose)
        if now - state["t_last"] >= 0.5:
            state["t_last"] = now
            lf = _log_layer(fu, pose, scan, now, state["travel"], log, region)
            if lf is not None and lf < lidar_abort_m and v > 0:
                state["aborted"] = f"LiDAR 앞 {lf:.2f} m → 중단"
    tk = _depth_ticker(ctx, sink)

    def tick(now: float) -> Optional[bool]:
        tk(now)
        return state["aborted"] is not None or (travel_stop is not None and state["travel"] >= travel_stop)
    ctx.drive(v, w)
    try:
        ctx.pump(secs, tick)
    finally:
        ctx.stop()
    return state


# ====================================================================== F05 낮은 물체 감지 거리
class F05_LowObjectDetection(Experiment):
    id, title, minutes = "F05", "LiDAR 평면 아래 낮은 물체: 깊이 3D 층이 몇 m 에서 잡나", 8
    needs = ["낮은 블록(높이 5~12 cm, 폭 ≥ 20 cm)", "앞 4 m 직선", "줄자"]
    procedure = ["로봇 정면 약 3 m 앞 바닥에 낮은 블록을 놓는다(LiDAR 평면 0.24 m 아래).", "시작 자세와 로봇 중심→블록 앞면 거리를 줄자로 입력한다.",
                 "로봇이 0.15 m/s 로 접근하다 블록 앞 0.45 m 에서 멈춘다. 감지된 최초 거리·허상 수를 기록한다."]
    safety = "블록에 부딪히기 전에 멈춘다(앞 0.45 m). LiDAR 앞이 0.35 m 안이면 중단."

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        start = ctx.start_pose()
        dist = float(ctx.ask("block_dist", "로봇 중심 → 블록 앞면 거리 (m)", 3.0))
        height_cm = float(ctx.ask("block_h_cm", "블록 높이 (cm)", 12.0))
        stop_gap = float(ctx.ask("stop_gap", "블록 앞에서 멈출 거리 (m)", 0.45))
        v = float(ctx.ask("v", "접근 속도 (m/s, ≤0.2 권장)", 0.15))
        ctx.confirm("place_block", f"로봇 정면 {dist:.2f} m 에 높이 {height_cm:.0f} cm 블록을 놓는다")
        fu = DepthFusion(ctx.cfg)
        c, s = math.cos(start[2]), math.sin(start[2])
        bx, by = start[0] + c * (dist + 0.1), start[1] + s * (dist + 0.1)
        region = (bx - 0.4, bx + 0.4, by - 0.4, by + 0.4)
        log: List[Dict[str, Any]] = []
        st = _drive_and_map(ctx, start, fu, v, 0.0, (dist - stop_gap) / v + 8.0, log, region, travel_stop=dist - stop_gap)
        return {"start": list(start), "block_dist": dist, "block_h_cm": height_cm, "stop_gap": stop_gap, "v": v, "region": list(region),
                "log": log, "aborted": st["aborted"], "report": fu.report()}

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis()
        log, d0 = data["log"], data["block_dist"]
        first = next((d0 - r["travel"] for r in log if r["layer_in_region"] >= 3), None)
        max_cells = max([r["layer_in_region"] for r in log], default=0)
        phantoms = max([r["layer_outside"] for r in log], default=0)
        lidar_min = min([r["lidar_front"] for r in log if r["lidar_front"] is not None], default=None)
        a.metrics = {"first_detect_range_m": None if first is None else round(first, 2), "max_cells_on_block": max_cells,
                     "max_phantom_cells": phantoms, "lidar_front_min_m": lidar_min, "aborted": data.get("aborted")}
        a.notes.append("LiDAR 가 블록을 보면(lidar_front_min 이 블록 거리 근처) 이 시험이 의미 없다 — 블록이 LiDAR 평면보다 낮은지 확인")
        if first is None:
            a.verdict = "FAIL"
            a.notes.append("3D 층이 블록을 못 잡았다 — 임계가 너무 엄격하거나(nav_thr·nav_min_neighbors) 블록이 너무 낮고 얇다. 표 값을 낮춰 재시험")
            a.suggestions += [Suggestion("map3d.nav_thr", round(max(cfg.map3d.nav_thr - 0.6, 0.8), 2), "한 단계 낮춤(F06 으로 허상이 안 늘었는지 함께 확인)"),
                              Suggestion("map3d.nav_min_neighbors", max(cfg.map3d.nav_min_neighbors - 1, 1), "한 단계 낮춤")]
            return a
        gs = cfg.map3d.guard_slow
        if first < gs + 0.2:
            a.suggestions.append(Suggestion("map3d.guard_slow", round(max(first - 0.2, 0.3), 2),
                                            f"최초 감지 {first:.2f} m 가 guard_slow({gs})+0.2 보다 가깝다 → 감속 시작 거리를 감지 거리보다 안쪽으로"))
        a.verdict = "PASS" if first >= 0.8 and phantoms <= 3 else "CHECK"
        if phantoms > 3:
            a.notes.append(f"블록 밖 허상 셀 {phantoms} — F06 도 해 볼 것")
        return a


# ====================================================================== F06 허상(빈 환경)
class F06_DepthPhantoms(Experiment):
    id, title, minutes = "F06", "깊이 3D 층 허상(낮은 물체 없는 환경에서 층이 비어야 한다)", 5
    needs = ["낮은 물체가 없는 경기장(박스·벽만)", "앞 1.5 m 직선"]
    procedure = ["경기장에 낮은 물체가 없는지 확인하고(바닥 케이블·테이프 단차 포함) 시작 자세를 입력한다.", "제자리 한 바퀴 + 1 m 전진 동안 3D 층(LiDAR 가 못 보는 장애물)의 셀 수를 센다. 0 이어야 한다."]
    safety = "앞이 1.8 m 이상 비어야 전진한다. 아니면 회전만 한다."

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        start = ctx.start_pose()
        fu = DepthFusion(ctx.cfg)
        log: List[Dict[str, Any]] = []
        o0 = ctx.robot.get_odom()
        _drive_and_map(ctx, start, fu, 0.0, 0.5, 2 * math.pi / 0.5 + 1.0, log)         # 제자리 한 바퀴
        th_after = start[2] + rel_pose(o0, ctx.robot.get_odom()).th
        sc = ctx.safe(ctx.robot.get_scan, "scan")
        free_ahead = None if sc is None else float(sc.min_in_sector(0.0, math.radians(10)))
        moved = False
        if free_ahead is not None and free_ahead > 1.8:
            moved = True
            _drive_and_map(ctx, (start[0], start[1], th_after), fu, 0.15, 0.0, 10.0, log, travel_stop=1.0)
        layer = fu.nav_layer()
        pts = []
        if layer is not None and layer.any():
            ys, xs = np.nonzero(layer)
            pts = [[round(fu.map.ox + (x + .5) * fu.map.res, 2), round(fu.map.oy + (y + .5) * fu.map.res, 2)] for y, x in list(zip(ys, xs))[:40]]
        return {"start": list(start), "moved_forward": moved, "front_free_m": free_ahead, "layer_cells_final": int(layer.sum()) if layer is not None else 0,
                "layer_cells_max": max([r["layer_outside"] for r in log], default=0), "cell_xy_sample": pts, "report": fu.report()}

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis()
        final, mx = data["layer_cells_final"], max(data["layer_cells_max"], data["layer_cells_final"])
        a.metrics = {"layer_cells_final": final, "layer_cells_max": mx, "moved_forward": data["moved_forward"], "occupied_voxels": data["report"].get("occupied_voxels")}
        if mx <= 3:
            a.verdict = "PASS"
            return a
        a.verdict = "FAIL"
        a.notes.append(f"허상 셀 {mx}개(최종 {final}). 좌표 샘플은 기록에 있다 — 실제 바닥 물체(케이블·단차)가 아닌지 눈으로 먼저 확인")
        a.suggestions += [Suggestion("map3d.nav_thr", round(min(cfg.map3d.nav_thr + 0.6, 3.0), 2), "한 단계 올림 → F05 로 낮은 블록이 여전히 잡히는지 확인"),
                          Suggestion("map3d.blind_grow_cells", min(cfg.map3d.blind_grow_cells + 1, 5), "한 단계 올림(박스 둘레 번짐 제거)")]
        return a


# ====================================================================== F07 접촉(정체) 감지
class F07_BumpDetection(Experiment):
    id, title, minutes = "F07", "접촉(정체) 감지: 자유 주행 vs 보이지 않는 장애물에 막힘", 8
    needs = ["앞 2 m 이상 LiDAR 가 보는 벽/물체가 정면에", "LiDAR·깊이에 안 보이는 낮은 장애물(≤12 cm)", "로봇이 밀어도 되는 상황"]
    procedure = ["앞 2 m 이상 열린 곳에서 0.2 m/s 로 2.5 s 전진(자유 주행 기준선).", "낮은 장애물을 범퍼 바로 앞(5~10 cm)에 놓고 3.5 s 전진 명령(바퀴가 헛돈다).",
                 "SLAM 대신 LiDAR 정면 거리 변화를 '실제 이동'으로 쓴다. 자유/막힘 이동비를 비교해 bump_ratio 를 제안한다."]
    safety = "막힘 구간은 모터가 구속된다 — 3.5 s 이내, 사람이 정지 버튼 근처에 있을 것. 자유 주행은 앞 0.8 m 에서 자동 중단."

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        ctx.confirm("front_clear", "앞 2 m 이상 비워 두고, 정면에 LiDAR 가 보는 벽이 있게 한다")
        v = float(ctx.ask("v", "전진 속도 (m/s)", 0.2))
        phases: Dict[str, List[Dict[str, float]]] = {}

        def phase(name: str, secs: float) -> None:
            rows: List[Dict[str, float]] = []
            o0 = ctx.robot.get_odom()
            t0 = ctx.now()

            def tick(now: float) -> Optional[bool]:
                sc = ctx.safe(ctx.robot.get_scan, "scan")
                od = ctx.robot.get_odom()
                fr = None if sc is None else lidar_range_at(sc, 0.0)
                rows.append({"t": now - t0, "front": fr if fr is not None else float("nan"), "odom_travel": math.hypot(od.x - o0.x, od.y - o0.y)})
                return fr is not None and fr < 0.8 and name == "free"
            ctx.drive(v, 0.0)
            try:
                ctx.pump(secs, tick)
            finally:
                ctx.stop()
            phases[name] = rows
        phase("free", 3.2)
        ctx.pump(1.0)
        ctx.confirm("block_bumper", "낮은 장애물을 범퍼 바로 앞 5~10 cm 에 놓는다(LiDAR 에 안 보이는 높이)")
        phase("blocked", 4.0)
        return {"v": v, "phases": phases, "cfg_bump": {"window_s": ctx.cfg.map3d.bump_window_s, "v_min": ctx.cfg.map3d.bump_v_min, "ratio": ctx.cfg.map3d.bump_ratio}}

    @staticmethod
    def _ratios(rows: List[Dict[str, float]], v: float, window: float, v_min: float, ratio: float) -> Tuple[List[float], Optional[float]]:
        """StallDetector(Agent 와 같은 로직)에 'LiDAR 정면 거리로 환산한 위치'를 먹여 창별 이동비와 최초 정체 판정 시각을 구한다."""
        det = StallDetector(window, v_min, ratio)
        rs, first = [], None
        prev_t = None
        for r in rows:
            if math.isnan(r["front"]) or r["t"] < 0.6:        # 가속 구간(0.6 s)은 뺀다 — 명령 속도에 못 미쳐 이동비가 낮게 나온다
                continue
            dt = 0.05 if prev_t is None else max(r["t"] - prev_t, 1e-3)
            prev_t = r["t"]
            out = det.update(r["t"], dt, -r["front"], 0.0, v)
            if det.last_ratio is not None:
                rs.append(det.last_ratio)
            if out is not None and first is None:
                first = r["t"]
        return rs, first

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis()
        c, v = data["cfg_bump"], data["v"]
        free_r, free_first = self._ratios(data["phases"].get("free", []), v, c["window_s"], c["v_min"], 0.0)
        blk_r, blk_first = self._ratios(data["phases"].get("blocked", []), v, c["window_s"], c["v_min"], c["ratio"])
        blk_rows = [r for r in data["phases"].get("blocked", []) if not math.isnan(r["front"])]
        odom_claim = data["phases"].get("blocked", [{}])[-1].get("odom_travel") if data["phases"].get("blocked") else None
        lidar_moved = (blk_rows[0]["front"] - blk_rows[-1]["front"]) if len(blk_rows) > 1 else None
        a.metrics = {"free_ratio_min": None if not free_r else round(min(free_r), 3), "blocked_ratio_max": None if not blk_r else round(max(blk_r), 3),
                     "detect_time_s": blk_first, "false_alarm_in_free": free_first is not None,
                     "odom_claimed_m_blocked": odom_claim, "lidar_moved_m_blocked": None if lidar_moved is None else round(lidar_moved, 3)}
        if not free_r or not blk_r:
            a.verdict = "CHECK"
            a.notes.append("창 하나도 못 채웠다(LiDAR 정면 거리·속도 확인; 자유 주행이 2.5 s 안에 0.8 m 에서 끊겼을 수 있다)")
            return a
        fmin, bmax = min(free_r), max(blk_r)
        if odom_claim is not None and lidar_moved is not None and odom_claim > 0.1 and lidar_moved < 0.5 * odom_claim:
            a.notes.append("바퀴 오도메트리는 막혀도 이동으로 쟀다(헛도는 바퀴) → Agent 가 오도메트리가 아닌 SLAM/스캔 정합 자세를 쓰는 것이 맞다")
        if fmin > 2 * bmax:
            a.suggestions.append(Suggestion("map3d.bump_ratio", round(min(max(math.sqrt(fmin * max(bmax, 0.02)), 0.1), 0.6), 2),
                                            f"자유 최솟값 {fmin:.2f} 와 막힘 최댓값 {bmax:.2f} 의 기하평균(분리 마진 {fmin / max(bmax, 1e-3):.1f}배)"))
            a.verdict = "PASS" if blk_first is not None and free_first is None else "CHECK"
        else:
            a.verdict = "FAIL"
            a.notes.append("자유/막힘 이동비가 충분히 분리되지 않는다 — 속도를 올리거나 장애물을 더 단단히, 또는 SLAM 자세 정밀도 확인")
        return a


# ====================================================================== F08 데이터 저장(오프라인 튜닝용)
class F08_DataCapture(Experiment):
    id, title, minutes = "F08", "센서 데이터 저장(RGB·깊이·LiDAR·오도메트리) — 오프라인 임계값 튜닝용", 5
    needs = ["저장 공간(깊이 프레임당 약 1 MB)"]
    procedure = ["찍고 싶은 상황(스모그 가동, 3색 타워, 후면 LED, 낮은 블록, 빈 바닥 등)을 만들고 태그 이름을 정한다.",
                 "로봇을 사람이 천천히 밀거나 주행시키며 저장한다. 나중에 `field_exp.py replay-depth <폴더>` 로 임계값을 바꿔 가며 다시 돌려 본다."]

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        tag = str(ctx.ask("tag", "태그(예: smog_on, empty_floor, low_block)", "capture", str))
        dur = float(ctx.ask("duration_s", "저장 시간 (s)", 10.0))
        max_depth = int(ctx.ask("max_depth", "깊이 최대 장수", 80, int))
        d = os.path.join(ctx.out_dir, f"capture_{tag}_{_dt.datetime.now().strftime('%H%M%S')}")
        os.makedirs(d, exist_ok=True)
        idx: List[Dict[str, Any]] = []
        last: Dict[str, Any] = {}
        counts = {"scan": 0, "frame": 0, "depth": 0}
        try:
            import cv2  # type: ignore
        except Exception:
            cv2 = None

        def tick(now: float) -> None:
            od = ctx.robot.get_odom()
            for name, g in (("scan", ctx.robot.get_scan), ("frame", ctx.robot.get_frame), ("depth", ctx.robot.get_depth)):
                s = ctx.safe(g, name)
                if s is None or s.stamp == last.get(name):
                    continue
                last[name] = s.stamp
                ent: Dict[str, Any] = {"kind": name, "now": now, "stamp": float(s.stamp), "odom": [od.x, od.y, od.th]}
                if name == "scan":
                    fn = f"scan_{counts['scan']:04d}.npy"
                    np.save(os.path.join(d, fn), np.asarray(s.ranges, dtype=np.float32))
                    ent.update(file=fn, angle_min=float(s.angle_min), angle_inc=float(s.angle_inc), range_max=float(s.range_max))
                elif name == "depth":
                    if counts["depth"] >= max_depth:
                        continue
                    fn = f"depth_{counts['depth']:04d}.npy"
                    np.save(os.path.join(d, fn), np.asarray(s.depth, dtype=np.float32))
                    ent.update(file=fn, pan=float(s.pan), tilt=float(s.tilt), K=list(s.K) if s.K else None)
                else:
                    ent.update(pan=float(s.pan), tilt=float(s.tilt))
                    if s.image is not None and cv2 is not None and counts["frame"] < 60:
                        fn = f"frame_{counts['frame']:04d}.png"
                        cv2.imwrite(os.path.join(d, fn), s.image)
                        ent["file"] = fn
                counts[name] += 1
                idx.append(ent)
        ctx.pump(dur, tick)
        with open(os.path.join(d, "index.json"), "w", encoding="utf-8") as f:
            json.dump(jsonable({"tag": tag, "config_depth": {"hfov_deg": ctx.cfg.depth.hfov_deg, "K": ctx.cfg.depth.K}, "entries": idx}), f, ensure_ascii=False, indent=1)
        return {"dir": d, "counts": counts, "tag": tag, "unsupported": list(ctx.unsupported)}

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis(verdict="INFO", metrics=data["counts"])
        a.notes.append(f"저장 폴더: {data['dir']} — 오프라인 재생: python scripts/field_exp.py replay-depth \"{data['dir']}\" --start 1.0 2.5 0")
        if sum(data["counts"].values()) == 0:
            a.verdict = "CHECK"
        return a


def replay_depth(capture_dir: str, cfg: Config, start: Tuple[float, float, float] = (1.0, 2.5, 0.0), overrides: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """F08 로 저장한 깊이·LiDAR·오도메트리를 현재(또는 일부 바꾼) 설정으로 다시 3D 지도에 넣어 본다 — 임계값 튜닝을 현장 밖에서."""
    from .overrides import set_by_path
    cfg = Config.load(None) if cfg is None else cfg
    for p, v in (overrides or {}).items():
        set_by_path(cfg, p, v)
    with open(os.path.join(capture_dir, "index.json"), "r", encoding="utf-8") as f:
        idx = json.load(f)["entries"]
    scans = [e for e in idx if e["kind"] == "scan"]
    from ..types import DepthFrame, Scan
    fu = DepthFusion(cfg)
    first = idx[0]["odom"] if idx else [0, 0, 0]
    ctx = Ctx(None, cfg, "")                                    # world_pose 만 쓴다
    cells_max = 0
    for e in idx:
        if e["kind"] != "depth":
            continue
        od = e["odom"]
        pose = ctx.world_pose(start, rel_pose(Pose2D(*first), Pose2D(*od)))
        sc = None
        if scans:
            se = min(scans, key=lambda s: abs(s["stamp"] - e["stamp"]))
            sc = Scan(se["angle_min"], se["angle_inc"], np.load(os.path.join(capture_dir, se["file"])), se["stamp"], se["range_max"])
        df = DepthFrame(np.load(os.path.join(capture_dir, e["file"])), e["stamp"], e.get("pan", 0.0), e.get("tilt", 0.0), tuple(e["K"]) if e.get("K") else None)
        fu.ingest(df, e["stamp"], pose, sc, pose)
        layer = fu.nav_layer()
        cells_max = max(cells_max, int(layer.sum()) if layer is not None else 0)
    rep = fu.report()
    rep["layer_cells_max"] = cells_max
    layer = fu.nav_layer()
    rep["layer_cells_final"] = int(layer.sum()) if layer is not None else 0
    return rep


# ====================================================================== F09 줄자 측정값 입력
class F09_ManualMeasurements(Experiment):
    id, title, minutes = "F09", "줄자·눈으로 잰 값 입력(설치 위치·치수·경기장) — 빈 칸은 건너뜀", 15
    needs = ["줄자", "경기장·로봇 접근"]
    procedure = ["각 항목을 줄자로 재서 입력한다. 모르면 Enter(건너뜀).", "base_link = 차체 중심, 높이는 차축(바퀴 중심) 높이. x 앞 +, y 왼쪽 +, z 위 +."]

    # (config 경로, 안내, 종류) — 종류: f=실수, v3=x y z, v2=두 값(창), arena_cy=세 값
    ITEMS: List[Tuple[str, str, str]] = [
        ("rig.depth_xyz", "깊이 렌즈 위치: base_link 기준 x y z (m)", "v3"),
        ("rig.camera_xyz", "RGB 렌즈 위치: 팬/틸트 축 기준 x y z (m)", "v3"),
        ("rig.pan_xyz", "팬 축 위치: base_link 기준 x y z (m)", "v3"),
        ("rig.lidar_xyz", "LiDAR 중심: base_link 기준 x y z (m)", "v3"),
        ("rig.base_z", "바닥 → base_link(차축) 높이 (m) = 바퀴 반지름", "f"),
        ("robot.wheel_radius", "바퀴 반지름 (m)", "f"),
        ("robot.track_width", "좌우 바퀴 중심 간격 (m)", "f"),
        ("robot.radius", "로봇이 들어가는 최소 원 반지름 (m) — 중심에서 가장 먼 모서리", "f"),
        ("robot.cam_height", "바닥 → RGB 렌즈 높이 (m)", "f"),
        ("aim.laser_lateral_m", "레이저 발광부가 RGB 렌즈보다 왼쪽으로 몇 m (오른쪽이면 음수)", "f"),
        ("aim.laser_vertical_m", "레이저 발광부가 RGB 렌즈보다 위로 몇 m (아래면 음수)", "f"),
        ("map3d.target_window_z", "표적 면 흰 원판 중심 높이 (m) → ±0.08 허용 창으로 환산", "win"),
        ("map3d.ugv_receiver_z", "상대 UGV 수광 돔 중심 높이 (m) → ±0.12 허용 창으로 환산", "win_ugv"),
        ("arena.W", "경기장 가로 안쪽 길이 (m)", "f"),
        ("arena.H", "경기장 세로 안쪽 길이 (m)", "f"),
        ("arena.base_depth", "거점 영역 깊이(벽에서 안쪽, m)", "f"),
        ("arena.base_height", "거점 영역 폭 (m)", "f"),
        ("arena.common_half", "COMMON 띠 반폭 (m)", "f"),
        ("arena.box_size", "박스 한 변 (m)", "f"),
        ("arena.wall_h", "외벽 높이 (m)", "f"),
        ("arena.base_cy", "거점 1·2·3 중심 y (m) — 위에서부터", "v3"),
    ]

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for path, prompt, kind in self.ITEMS:
            raw = str(ctx.ask("m_" + path, f"{path}: {prompt}", "", str)).strip()
            if raw:
                out[path] = {"raw": raw, "kind": kind}
        return {"entries": out}

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis(verdict="INFO")
        for path, e in data["entries"].items():
            try:
                nums = [float(x) for x in e["raw"].replace(",", " ").split()]
                k = e["kind"]
                if k == "f":
                    val: Any = nums[0]
                elif k == "v3":
                    if len(nums) != 3:
                        raise ValueError("값 3개 필요")
                    val = nums
                elif k in ("win", "win_ugv"):
                    m = 0.08 if k == "win" else 0.12
                    val = [round(max(nums[0] - m, 0.0), 3), round(nums[0] + m, 3)]
                else:
                    raise ValueError(k)
                a.suggestions.append(Suggestion(path, val, f"입력: {e['raw']}"))
            except (ValueError, IndexError) as ex:
                a.notes.append(f"{path}: '{e['raw']}' 해석 실패({ex}) — 건너뜀")
        if "rig.base_z" in data["entries"] and "robot.wheel_radius" not in data["entries"]:
            a.notes.append("바퀴 반지름도 같은 값이면 robot.wheel_radius 를 따로 입력하라(두 값은 별개 설정)")
        a.metrics = {"entered": len(a.suggestions)}
        return a


# ====================================================================== F10 레이저 보어사이트
class F10_LaserBoresight(Experiment):
    id, title, minutes = "F10", "레이저↔카메라 축 오프셋(옆·위 평행 오프셋 + 축 각도) — 두 거리에서 점 위치", 10
    needs = ["평평한 표적면 + 십자 표시", "줄자", "레이저 보호 안경"]
    procedure = ["팬/틸트 0°, 표적면에 십자 표시. 카메라 영상 중심(십자선)을 표시에 맞춘다.", "레이저 점이 표시에서 왼쪽/위로 몇 cm 벗어나는지 잰다(왼쪽·위가 +).",
                 "거리 r1(예 1.0 m)와 r2(예 2.5 m) 두 곳에서 반복. 평행 오프셋은 거리와 무관, 축 각도 오차는 거리에 비례한다."]
    safety = "레이저는 --allow-laser 를 줬을 때만 자동 발사한다. 아니면 사람이 따로 발사한다. 눈 방향 금지."

    def run(self, ctx: Ctx) -> Dict[str, Any]:
        pts = []
        for i, dflt in ((1, 1.0), (2, 2.5)):
            r = float(ctx.ask(f"r{i}", f"거리 r{i} (m): 카메라 렌즈 → 표적면", dflt))
            if ctx.allow_laser and ctx.robot_kind != "sim":
                ctx.confirm(f"fire_{i}", f"r{i} 에서 레이저를 자동 발사한다(보호 안경·표적 방향 확인)")
                ctx.robot.laser_fire()
            else:
                ctx.say("  (레이저 자동 발사 안 함 — 사람이 직접 발사해 점 위치를 읽는다)")
            left = float(ctx.ask(f"left_cm{i}", f"r{i} 에서 레이저 점이 십자 표시보다 왼쪽으로 몇 cm (오른쪽이면 음수)", 0.0))
            up = float(ctx.ask(f"up_cm{i}", f"r{i} 에서 레이저 점이 십자 표시보다 위로 몇 cm (아래면 음수)", 0.0))
            pts.append({"r": r, "left_cm": left, "up_cm": up})
        return {"points": pts}

    def analyze(self, data: Dict[str, Any], cfg: Config) -> Analysis:
        a = Analysis()
        (p1, p2) = data["points"]
        dr = p2["r"] - p1["r"]
        if abs(dr) < 0.5:
            a.verdict, a.notes = "CHECK", ["두 거리 차이가 0.5 m 미만이면 평행 오프셋과 축 각도를 분리할 수 없다"]
            return a
        res = {}
        for name, key in (("lateral(left)", "left_cm"), ("vertical(up)", "up_cm")):
            o1, o2 = p1[key] / 100.0, p2[key] / 100.0
            tan_a = (o2 - o1) / dr
            off = o1 - p1["r"] * tan_a
            res[name] = {"offset_m": off, "axis_deg": math.degrees(math.atan(tan_a))}
        a.metrics = {k: {kk: round(vv, 4) for kk, vv in v.items()} for k, v in res.items()}
        a.suggestions += [Suggestion("aim.laser_lateral_m", round(res["lateral(left)"]["offset_m"], 4), "점 위치 o(r)=오프셋+r·tan(축각) 의 두 점 연립해(왼쪽 +)"),
                          Suggestion("aim.laser_offset_pan_deg", round(res["lateral(left)"]["axis_deg"], 3), "축 각도 오차(왼쪽 +)"),
                          Suggestion("aim.laser_vertical_m", round(res["vertical(up)"]["offset_m"], 4), "(위 +)"),
                          Suggestion("aim.laser_offset_tilt_deg", round(res["vertical(up)"]["axis_deg"], 3), "축 각도 오차(위 +)")]
        big = max(abs(res["lateral(left)"]["offset_m"]), abs(res["vertical(up)"]["offset_m"]))
        a.verdict = "CHECK" if big > 0.15 else "PASS"
        if big > 0.15:
            a.notes.append("평행 오프셋이 15 cm 를 넘는다 — 부호·거리 입력을 다시 확인")
        a.notes.append("점 위치는 십자 표시(카메라 광축 위의 점) 기준. 적용 후 같은 방법으로 한 번 더 확인해 점이 십자 근처로 오는지 본다.")
        return a


EXPERIMENTS: Dict[str, Experiment] = {e.id: e for e in (F01_SensorStreams(), F02_DepthFloorMount(), F03_DepthRangeAccuracy(), F04_DepthLidarAlignment(),
                                                        F05_LowObjectDetection(), F06_DepthPhantoms(), F07_BumpDetection(), F08_DataCapture(),
                                                        F09_ManualMeasurements(), F10_LaserBoresight())}


# ====================================================================== 실행·기록
def execute(exp: Experiment, ctx: Ctx, note: str = "") -> Tuple[str, Dict[str, Any]]:
    """실험을 돌리고 분석해 기록 파일에 쓴다. (기록 경로, 기록 dict)."""
    ctx.say(f"\n=== {exp.id} {exp.title} (약 {exp.minutes}분) ===")
    for s in exp.procedure:
        ctx.say(f"  · {s}")
    if exp.safety:
        ctx.say(f"  ! 안전: {exp.safety}")
    t0 = _dt.datetime.now()
    try:
        data = exp.run(ctx)
    finally:
        try:
            ctx.stop()
        except Exception:
            pass
    an = exp.analyze(data, ctx.cfg)
    rel = [t for t in tun.TUNABLES if t.exp == exp.id]
    rec = {"id": exp.id, "title": exp.title, "date": t0.date().isoformat(), "time": t0.strftime("%H:%M:%S"), "robot": ctx.robot_kind, "note": note,
           "data": data,
           "analysis": {"verdict": an.verdict, "metrics": an.metrics, "notes": an.notes,
                        "suggestions": [{"path": s.path, "value": s.value, "note": s.note} for s in an.suggestions]},
           "related_tunables": {t.path: jsonable(tun.resolve(ctx.cfg, t.path)) for t in rel}}
    if ctx.robot_kind == "sim":
        rec["WARNING"] = "시뮬 로봇에서 돌린 코드 확인용 기록이다 — 이 수치를 설정에 적용하지 말 것"
    d = os.path.join(ctx.out_dir)
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, f"{exp.id}_{t0.strftime('%H%M%S')}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(jsonable(rec), f, ensure_ascii=False, indent=2)
    ctx.say(f"\n  판정: {an.verdict}")
    for k, v in an.metrics.items():
        ctx.say(f"  {k}: {v}")
    for n in an.notes:
        ctx.say(f"  ※ {n}")
    for s in an.suggestions:
        ctx.say(f"  → 제안 {s.path} = {s.value}   ({s.note})")
    ctx.say(f"  기록: {path}")
    return path, rec
