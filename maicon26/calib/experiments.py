"""보정용 측정 시험. 로봇 종류(RobotIO)와 무관하게 같은 절차를 돌리고 Trace 를 남긴다.

시험              | 알아내는 값
static            | 오도메트리 잡음, 오도메트리/IMU 자이로 편향
speed_steps       | 최대 속도(전/후진), 가속도, 명령 지연, 상승 시간, 센서(오도메트리) 지연
turn_steps        | 최대 각속도, 각가속도, IMU 지연
straight          | 직진 좌우 편차(rad/m), 오도메트리 선형 스케일, 주행면별 슬립
rotate            | 제자리 회전 효율, 오도메트리 회전 스케일, 회전 중 위치 이동, 정지 후 오버슈트
brake             | 제동 감속도, 속도별 정지거리, 제동 지연, 마찰계수 하한
lidar_square      | LiDAR 장착 위치·방향 (벽에 정렬해 놓고 두 벽까지 거리)
avoid             | 경로 위 장애물 회피: 성공 여부, 소요 시간, 최소 이격, 복구 횟수

필요한 공간: 직선 3.5 m x 폭 1 m (speed_steps/straight/brake), 회전 1 m x 1 m, 벽 모서리 (lidar_square).
"""
from __future__ import annotations

import math
import time
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

from .records import RosbagRecorder, Trace, write_manifest
from .robot_io import NAN, RobotIO

START = (0.8, 2.5, 0.0)          # 시뮬 기본 시작 자세(경기장 안 넓은 곳)


def _drive(io: RobotIO, tr: Trace, v: float, w: float, seconds: float,
           stop_when: Optional[Callable[[Dict[str, float]], bool]] = None) -> bool:
    for _ in range(int(round(seconds / io.dt))):
        io.set_cmd(v, w)
        s = io.tick()
        s["cmd_v"], s["cmd_w"] = v, w
        tr.add(s)
        if stop_when is not None and stop_when(s):
            return True
    return False


def _odom_dist(tr: Trace) -> float:
    r = tr.rows                                  # (tr.a 는 매번 배열을 다시 만들어 느리다 — 행 리스트를 직접 본다)
    if len(r) < 2:
        return 0.0
    return float(math.hypot(r[-1][3] - r[0][3], r[-1][4] - r[0][4]))     # COLS: odom_x=3, odom_y=4


def _settle(io: RobotIO, tr: Trace, max_s: float = 2.5, min_s: float = 0.8) -> None:
    """명령 0 으로 두고 odom 속도가 ~0 이 될 때까지(최소 min_s)."""
    t0 = None
    prev = None
    quiet = 0

    def done(s: Dict[str, float]) -> bool:
        nonlocal prev, quiet, t0
        t0 = s["t"] if t0 is None else t0
        if prev is not None and s["t"] > prev["t"]:
            sp = math.hypot(s.get("odom_x", 0) - prev.get("odom_x", 0), s.get("odom_y", 0) - prev.get("odom_y", 0)) / (s["t"] - prev["t"])
            quiet = quiet + 1 if sp < 0.01 else 0
        prev = s
        return (s["t"] - t0) >= min_s and quiet >= int(0.3 / io.dt)
    _drive(io, tr, 0.0, 0.0, max_s, done)


def _new(name: str, kind: str, surface: str, **meta: Any) -> Trace:
    return Trace(name, {"kind": kind, "surface": surface, **meta})


def _finish(io: RobotIO, tr: Trace, kind: str) -> None:
    et = io.end_truth(kind)
    if et:
        tr.meta["truth_end"] = et


# ====================================================================== 시험들
def exp_static(io: RobotIO, surface: str = "default", seconds: float = 20.0) -> List[Trace]:
    io.place(3.0, 2.5, 0.0, "평평한 곳에 정지시킨다(20 s 동안 건드리지 말 것)")
    tr = _new(f"static__{surface}", "static", surface)
    _drive(io, tr, 0.0, 0.0, seconds)
    return [tr]


def exp_speed_steps(io: RobotIO, surface: str = "default", speeds: Sequence[float] = (0.15, 0.30, 0.45, 0.80, -0.30, -0.80),
                    hold: float = 2.0) -> List[Trace]:
    out = []
    for v in speeds:
        io.place(START[0] if v > 0 else START[0] + 2.4, START[1], START[2], f"직선 3 m 확보, 시작 위치에 정지 (속도 요청 {v:+.2f} m/s)")
        tr = _new(f"speed_step__{surface}__{v:+.2f}", "speed_step", surface, cmd=v, hold=hold)
        _drive(io, tr, 0.0, 0.0, 0.5)                                       # 정지 기준선
        cut = _drive(io, tr, v, 0.0, hold, lambda s: _odom_dist(tr) > 3.0)   # 공간 보호
        tr.meta["t_cmd_on"] = float(tr.a[int(0.5 / io.dt), 0]) if len(tr.a) > int(0.5 / io.dt) else None
        tr.meta["t_cmd_off"] = float(tr.a[-1, 0])
        tr.meta["cut_short"] = bool(cut)
        _settle(io, tr)
        _finish(io, tr, tr.name)
        out.append(tr)
    return out


def exp_turn_steps(io: RobotIO, surface: str = "default", ws: Sequence[float] = (0.8, 1.6, 3.2), hold: float = 1.5) -> List[Trace]:
    out = []
    for w in ws:
        io.place(3.0, 2.5, 0.0, f"1 m x 1 m 빈 곳에서 제자리 회전 (각속도 요청 {w:.1f} rad/s)")
        tr = _new(f"turn_step__{surface}__{w:+.2f}", "turn_step", surface, cmd=w, hold=hold)
        _drive(io, tr, 0.0, 0.0, 0.5)
        tr.meta["t_cmd_on"] = float(tr.a[-1, 0])
        _drive(io, tr, 0.0, w, hold)
        tr.meta["t_cmd_off"] = float(tr.a[-1, 0])
        _settle(io, tr)
        _finish(io, tr, tr.name)
        out.append(tr)
    return out


def exp_straight(io: RobotIO, surface: str = "default", speeds: Sequence[float] = (0.20, 0.40), dist: float = 2.5) -> List[Trace]:
    out = []
    for v in speeds:
        io.place(START[0], START[1], START[2], f"직선 {dist + 0.5:.1f} m 확보, 바닥에 시작선/끝선 표시 (v={v:.2f})")
        tr = _new(f"straight__{surface}__{v:+.2f}", "straight", surface, cmd=v, dist=dist)
        _drive(io, tr, 0.0, 0.0, 0.5)
        _drive(io, tr, v, 0.0, (dist / v) * 2.0, lambda s: _odom_dist(tr) >= dist)
        _settle(io, tr)
        _finish(io, tr, tr.name)
        out.append(tr)
    return out


def exp_rotate(io: RobotIO, surface: str = "default", ws: Sequence[float] = (1.0, 1.8), turns: float = 2.0) -> List[Trace]:
    out = []
    for w in ws:
        io.place(3.0, 2.5, 0.0, f"제자리 {turns:.0f}바퀴 회전 (w={w:.1f}), 바닥에 바퀴 중심 표시")
        tr = _new(f"rotate__{surface}__{w:+.2f}", "rotate", surface, cmd=w, turns=turns)
        _drive(io, tr, 0.0, 0.0, 0.5)
        tr.meta["t_cmd_on"] = float(tr.a[-1, 0])
        _drive(io, tr, 0.0, w, turns * 2 * math.pi / w)
        tr.meta["t_cmd_off"] = float(tr.a[-1, 0])
        _settle(io, tr, max_s=3.0, min_s=1.2)                               # 정지 후 오버슈트 관찰
        _finish(io, tr, tr.name)
        out.append(tr)
    return out


def exp_brake(io: RobotIO, surface: str = "default", speeds: Sequence[float] = (0.15, 0.30, 0.45), repeats: int = 2) -> List[Trace]:
    out = []
    for v in speeds:
        for k in range(repeats):
            io.place(START[0], START[1], START[2], f"급정지 시험: 직선 3 m 확보 (v0={v:.2f}, {k + 1}/{repeats})")
            tr = _new(f"brake__{surface}__{v:+.2f}__{k}", "brake", surface, cmd=v)
            _drive(io, tr, 0.0, 0.0, 0.3)
            _drive(io, tr, v, 0.0, 4.0, lambda s: _odom_dist(tr) > 1.6 and s.get("t", 0) - tr.rows[0][0] > 1.8)
            tr.meta["t_cmd_off"] = float(tr.rows[-1][0])
            _settle(io, tr, max_s=3.0, min_s=0.8)
            _finish(io, tr, tr.name)
            out.append(tr)
    return out


def exp_lidar_square(io: RobotIO, surface: str = "default", n: int = 40, d_front: Optional[float] = None,
                     d_left: Optional[float] = None) -> List[Trace]:
    """로봇을 정면 벽에 '직각'으로 정렬해 놓고 정면 벽·왼쪽 벽까지 거리(차체 중심 기준, 줄자)를 입력한다."""
    io.place(5.0, 4.0, 0.0, "정면 벽에 직각으로 정렬(자/레이저 수평계), 왼쪽에도 벽이 보이는 모서리")
    if d_front is None or d_left is None:
        if io.name != "sim":
            raise ValueError("lidar_square: 정면/왼쪽 벽까지 거리(차체 중심 기준, m)를 --d-front, --d-left 로 줄자 측정해 넣는다")
        d_front, d_left = 6.12 - 5.0, 5.0 - 4.0                           # 시뮬: 경기장 외벽까지
    scs = io.scans(n)
    tr = _new(f"lidar_square__{surface}", "lidar_square", surface, d_front=d_front, d_left=d_left, n_scans=len(scs))
    if scs:
        R = np.vstack([s.ranges for s in scs])
        R[~np.isfinite(R)] = np.nan
        with np.errstate(all="ignore"):
            med = np.nanmedian(R, axis=0)
        tr.meta["angle_min"], tr.meta["angle_inc"] = float(scs[0].angle_min), float(scs[0].angle_inc)
        tr.meta["ranges"] = [None if not np.isfinite(x) else float(x) for x in med]
    tr.add({"t": 0.0})
    return [tr]


def exp_avoid(io: RobotIO, surface: str = "default", forward: float = 2.4, lateral: float = 0.0, timeout: float = 45.0) -> List[Trace]:
    if not io.avoid_supported():
        return []
    io.place(START[0], START[1], START[2], f"전방 {forward:.1f} m 직선 경로 확보 (중간에 박스를 놓는다)")
    io.avoid_setup(forward, lateral)
    tr = _new(f"avoid__{surface}", "avoid", surface, forward=forward, lateral=lateral)
    io.avoid_start(forward, lateral)
    t_start, status, rec = None, "running", 0
    while True:
        status, rec = io.avoid_poll()
        s = io.tick()
        t_start = s["t"] if t_start is None else t_start
        s["cmd_v"], s["cmd_w"] = NAN, NAN
        tr.add(s)
        if status != "running" or s["t"] - t_start > timeout:
            break
    io.avoid_stop()
    tr.meta.update(status=status if status != "running" else "timeout", recoveries=rec, duration=float(tr.a[-1, 0] - tr.a[0, 0]))
    _settle(io, tr, max_s=2.0, min_s=0.5)
    _finish(io, tr, tr.name)
    return [tr]


EXPERIMENTS = {
    "static": exp_static, "speed_steps": exp_speed_steps, "turn_steps": exp_turn_steps, "straight": exp_straight,
    "rotate": exp_rotate, "brake": exp_brake, "lidar_square": exp_lidar_square, "avoid": exp_avoid,
}


def run_experiments(io: RobotIO, out_dir: str, which: Sequence[str], surface: str = "default",
                    recorder: Optional[RosbagRecorder] = None, kwargs: Optional[Dict[str, Dict[str, Any]]] = None,
                    info: Optional[Dict[str, Any]] = None, log: Callable[[str], None] = print) -> List[Trace]:
    kwargs = kwargs or {}
    all_tr: List[Trace] = []
    for name in which:
        fn = EXPERIMENTS[name]
        log(f"[calib] {name} (surface={surface}) ...")
        if recorder:
            recorder.start(f"{name}__{surface}")
        t0 = time.time()
        trs = fn(io, surface=surface, **kwargs.get(name, {}))
        if recorder:
            recorder.stop()
        for t in trs:
            t.meta.setdefault("io", io.name)
            t.save(out_dir)
        all_tr += trs
        log(f"[calib]   → {len(trs)}개 기록, {time.time() - t0:.0f}s")
    write_manifest(out_dir, {**(info or {}), **io.info(), "surface": surface, "experiments": list(which)}, all_tr)
    return all_tr
