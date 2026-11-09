"""측정 기록(Trace) → 보정값. 입력은 CSV 시계열뿐이라 시뮬/실기/rosbag 변환물 모두에 같은 코드를 쓴다.

참값 모드 (시험 하나마다 자동 판별)
  continuous  truth_* 열이 있음(모캡/브리지 /ground_truth/시뮬) → 가장 정확
  end         시작·끝에서 줄자/각도기로 잰 수기값(meta.truth_end: distance_m, lateral_m, heading_deg)
              → 오도메트리 시계열에 시험 전체 배율을 곱해 '참 속도'로 쓴다 (근사)
  none        참값 없음 → 오도메트리를 그대로 믿는다 (스케일/슬립은 못 구함, 응답 형태만 얻음)

모든 결과는 profile 의 경로에 provenance='measured' 로 들어가고, 근거 시험 목록이 노트에 남는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .profile import Profile
from .records import Trace

G = 9.81


# ====================================================================== 시계열 도우미
def _ma(x: np.ndarray, n: int = 5) -> np.ndarray:
    if n <= 1 or len(x) < n:
        return x
    k = np.ones(n) / n
    pad = n // 2
    xp = np.pad(x, (pad, n - 1 - pad), mode="edge")
    return np.convolve(xp, k, mode="valid")


def _deriv(t: np.ndarray, x: np.ndarray, n: int = 3) -> np.ndarray:
    return _ma(np.gradient(x, t), n)


@dataclass
class Series:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    th: np.ndarray            # unwrapped
    v: np.ndarray             # 전진 속도 (부호 있음)
    w: np.ndarray
    mode: str                 # continuous | end | none
    k_lin: float = 1.0
    k_ang: float = 1.0


def _regularize(t: np.ndarray, *cols: np.ndarray) -> Tuple[np.ndarray, ...]:
    """샘플-앤-홀드 제거. ROS 에서 50 Hz 로 읽어도 토픽 발행 지터 때문에 같은 메시지가 두 번 읽히거나(값 중복) 건너뛰는 일이
    생긴다(실측: 참값 표본의 ~40 % 가 중복) → 그대로 미분하면 0 과 2배 스파이크가 번갈아 나온다.
    값이 바뀐 표본만 남기고 그 사이를 선형 보간해 원래 시각 격자에 다시 놓는다. 중복이 없으면(시뮬 직접) 그대로다."""
    ok = np.ones(len(t), dtype=bool)
    ok[1:] = np.any([np.abs(np.diff(c)) > 1e-12 for c in cols], axis=0)
    if ok.all() or ok.sum() < 4:
        return cols
    return tuple(np.interp(t, t[ok], c[ok]) for c in cols)


def _on_stamps(tr: Trace, stamp_col: str, t: np.ndarray, *cols: np.ndarray) -> Tuple[np.ndarray, ...]:
    """각 채널을 자기 메시지 header 시각(stamp_col) 기준으로 재구성해 표본 시각 격자 t 로 보간한다.
    중복(같은 메시지를 두 번 읽음)·건너뜀(두 개가 한 틱에 옴)이 있어도 시간축이 바르다. stamp 가 없으면 값 변화 기준(_regularize)."""
    st = tr.col(stamp_col)
    if np.isfinite(st).sum() > 8 and np.isfinite(st).all():
        keep = np.ones(len(st), dtype=bool)
        keep[1:] = np.diff(st) > 1e-9
        if keep.sum() >= 4:
            return tuple(np.interp(t, st[keep], c[keep]) for c in cols)
    return _regularize(t, *cols)


def _pose_series(t: np.ndarray, x: np.ndarray, y: np.ndarray, th: np.ndarray) -> Tuple[np.ndarray, ...]:
    th = np.unwrap(th)
    vx, vy = _deriv(t, x), _deriv(t, y)
    v = vx * np.cos(th) + vy * np.sin(th)
    return th, v, _deriv(t, th)


def odom_series(tr: Trace) -> Series:
    t = tr.col("t")
    x, y, th0 = _on_stamps(tr, "odom_t", t, tr.col("odom_x"), tr.col("odom_y"), np.unwrap(tr.col("odom_th")))   # 각도는 보간 전에 unwrap(±π 경계 보간 방지)
    th, v, w = _pose_series(t, x, y, th0)
    return Series(t, x, y, th, v, w, "none")


def gyro_series(tr: Trace) -> np.ndarray:
    g = tr.col("gyro_z")
    if not np.isfinite(g).all():
        return g
    return _on_stamps(tr, "imu_t", tr.col("t"), g)[0]


def true_series(tr: Trace) -> Series:
    """참 운동. continuous → 참값 미분, end → 오도메트리×(수기 참값/오도메트리) 배율, none → 오도메트리."""
    t = tr.col("t")
    if tr.has_truth():
        x, y, th0 = _on_stamps(tr, "truth_t", t, tr.col("truth_x"), tr.col("truth_y"), np.unwrap(tr.col("truth_th")))
        th, v, w = _pose_series(t, x, y, th0)
        return Series(t, x, y, th, v, w, "continuous")
    o = odom_series(tr)
    et = tr.meta.get("truth_end") or {}
    k_lin = k_ang = 1.0
    mode = "none"
    od = float(np.hypot(o.x[-1] - o.x[0], o.y[-1] - o.y[0]))
    if "distance_m" in et and od > 0.05:
        k_lin, mode = float(et["distance_m"]) / od, "end"
    dth = float(o.th[-1] - o.th[0])
    if "heading_deg" in et and abs(dth) > 0.5:
        k_ang, mode = math.radians(float(et["heading_deg"])) / dth, "end"
    return Series(o.t, o.x * k_lin, o.y * k_lin, o.th * k_ang, o.v * k_lin, o.w * k_ang, mode, k_lin, k_ang)


def _first_cross(t: np.ndarray, y: np.ndarray, level: float, after: float) -> Optional[float]:
    idx = np.nonzero((t >= after) & (y >= level))[0]
    return float(t[idx[0]]) if len(idx) else None


def _ramp_fit(t: np.ndarray, y: np.ndarray, t_on: float, plateau: float, t_end: float) -> Tuple[float, float]:
    """스텝 응답을 `y = clip(a·(t − t_on − L), 0, plateau)` 모델에 맞춘다 → (가속 a, 지연 L).
    모든 표본을 쓰는 격자 탐색이라 미분 잡음·발행 지터에 20~80 % 구간 직선 맞춤보다 강하다."""
    m = (t >= t_on - 0.1) & (t <= t_end)
    tt, yy = t[m] - t_on, y[m]
    best, ba, bL = float("inf"), float("nan"), float("nan")
    Ls = np.arange(0.0, 0.45, 0.005)
    for a in np.geomspace(0.2, 40.0, 140):
        pred = np.clip(a * (tt[None, :] - Ls[:, None]), 0.0, plateau)         # (L, 표본)
        e = np.sum((yy[None, :] - pred) ** 2, axis=1)
        j = int(np.argmin(e))
        if e[j] < best:
            best, ba, bL = float(e[j]), float(a), float(Ls[j])
    return ba, bL


def _brake_fit(t: np.ndarray, v: np.ndarray, t_off: float, v0: float, t_end: float) -> Tuple[float, float]:
    """정속 v0 → (지연 L 후) 등감속 a 로 정지하는 모델 `v = v0 (t < L), max(v0 − a(t − L), 0)` 에 맞춘다 → (a, L)."""
    m = (t >= t_off - 0.15) & (t <= t_end)
    tt, vv = t[m] - t_off, np.abs(v[m])
    v0 = abs(v0)
    best, ba, bL = float("inf"), float("nan"), float("nan")
    Ls = np.arange(0.0, 0.4, 0.005)
    for a in np.arange(0.3, 6.0, 0.02):
        pred = np.where(tt[None, :] < Ls[:, None], v0, np.clip(v0 - a * (tt[None, :] - Ls[:, None]), 0.0, v0))
        e = np.sum((vv[None, :] - pred) ** 2, axis=1)
        j = int(np.argmin(e))
        if e[j] < best:
            best, ba, bL = float(e[j]), float(a), float(Ls[j])
    return ba, bL


def step_response(t: np.ndarray, y: np.ndarray, t_on: float, t_off: float) -> Optional[Dict[str, float]]:
    """y(양수 방향으로 정규화) 의 스텝 응답: 평탄값, 기울기(가속), 지연, 10→90 % 상승시간."""
    hold = (t >= t_on + 0.6 * (t_off - t_on)) & (t <= t_off)
    if hold.sum() < 3:
        return None
    ys = np.sort(y[hold])
    k = int(0.1 * len(ys))
    plateau = float(np.mean(ys[k: len(ys) - k] if len(ys) - 2 * k >= 3 else ys))    # 10 % 절사 평균(중앙값은 지터 스파이크에 치우친다)
    if plateau < 1e-3:
        return None
    if not (y[t > t_on] >= 0.8 * plateau).any():
        return None
    a, L = _ramp_fit(t, y, t_on, plateau, t_off)
    t10, t90 = _first_cross(t, y, 0.1 * plateau, t_on), _first_cross(t, y, 0.9 * plateau, t_on)
    return {"plateau": plateau, "a": float(a), "L": float(L),
            "rise": float(t90 - t10) if (t10 is not None and t90 is not None) else float("nan")}


def _med(xs: Sequence[float]) -> float:
    xs = [x for x in xs if x is not None and np.isfinite(x)]
    return float(np.median(xs)) if xs else float("nan")


def _by_kind(traces: Sequence[Trace], kind: str) -> List[Trace]:
    return [t for t in traces if t.meta.get("kind") == kind and len(t.rows) > 5]


def _group_surface(trs: Sequence[Trace]) -> Dict[str, List[Trace]]:
    g: Dict[str, List[Trace]] = {}
    for t in trs:
        g.setdefault(t.meta.get("surface", "default"), []).append(t)
    return g


# ====================================================================== 시험별 분석
def fit_static(traces: Sequence[Trace]) -> Dict[str, float]:
    trs = _by_kind(traces, "static")
    if not trs:
        return {}
    ov, ow, gz = [], [], []
    for tr in trs:
        t = tr.col("t")
        o = odom_series(tr)
        dt = np.diff(t)
        ov.append(np.hypot(np.diff(o.x), np.diff(o.y)) / dt)
        ow.append(np.diff(o.th) / dt)
        gz.append(gyro_series(tr))
    v = np.concatenate(ov)
    w = np.concatenate(ow)
    g = np.concatenate(gz)
    g = g[np.isfinite(g)]
    # 정지 중 오도메트리 속도는 부호 없는 크기라 평균이 잡음 크기를 반영 → 표준편차 대신 RMS 로
    out = {"odom_noise_v": float(np.sqrt(np.mean(v ** 2))), "odom_gyro_bias_z": float(np.mean(w)),
           "odom_noise_w": float(np.std(w))}
    if len(g):
        out.update(imu_bias_z=float(np.mean(g)), imu_noise_z=float(np.std(g)))
    return out


def fit_speed_steps(traces: Sequence[Trace]) -> Dict[str, Any]:
    trs = _by_kind(traces, "speed_step")
    if not trs:
        return {}
    fw, rv, accs, lats, rises, odl, notes = [], [], [], [], [], [], []
    for tr in trs:
        cmd = float(tr.meta["cmd"])
        t_on, t_off = tr.meta.get("t_cmd_on"), tr.meta.get("t_cmd_off")
        if t_on is None or t_off is None:
            continue
        S, O = true_series(tr), odom_series(tr)
        sgn = 1.0 if cmd > 0 else -1.0
        r = step_response(S.t, S.v * sgn, t_on, t_off)
        ro = step_response(O.t, O.v * sgn * (S.k_lin if S.mode == "end" else 1.0), t_on, t_off)
        if r is None:
            continue
        sat = r["plateau"] < 0.97 * abs(cmd)
        rec = {"cmd": cmd, "plateau": r["plateau"], "saturated": sat, "mode": S.mode}
        notes.append(rec)
        if cmd > 0:
            fw.append(rec)
        else:
            rv.append(rec)
        if abs(cmd) >= 0.25:
            accs.append(r["a"]); lats.append(r["L"]); rises.append(r["rise"])
            if ro is not None:
                odl.append(ro["L"] - r["L"])
    out: Dict[str, Any] = {"records": notes}
    for name, lst in (("v_max", fw), ("v_max_reverse", rv)):
        if lst:
            top = max(lst, key=lambda r: abs(r["cmd"]))
            out[name] = float(top["plateau"])
            out[name + "_is_lower_bound"] = not top["saturated"]       # 요청 속도까지 못 올라갔어야 '최대'가 확정
    out.update(a_accel=_med(accs), cmd_latency_s=max(0.0, _med(lats)), rise_time_s=_med(rises),
               odom_latency_s=max(0.0, _med(odl)))
    return out


def _xcorr_lag(t: np.ndarray, a: np.ndarray, b: np.ndarray, max_lag: float = 0.4) -> float:
    """b 가 a 보다 얼마나 늦은가(s). 같은 균일 격자에 보간 후 상호상관, 포물선 보간."""
    dt = float(np.median(np.diff(t)))
    n_max = int(max_lag / dt)
    a0, b0 = a - np.mean(a), b - np.mean(b)
    lags = list(range(-n_max, n_max + 1))
    cs = []
    for k in lags:
        if k >= 0:
            x, y = a0[: len(a0) - k], b0[k:]
        else:
            x, y = a0[-k:], b0[: len(b0) + k]
        cs.append(float(np.dot(x, y)) / max(len(x), 1))
    i = int(np.argmax(cs))
    if 0 < i < len(cs) - 1:
        d = cs[i - 1] - 2 * cs[i] + cs[i + 1]
        frac = 0.5 * (cs[i - 1] - cs[i + 1]) / d if abs(d) > 1e-12 else 0.0
    else:
        frac = 0.0
    return (lags[i] + frac) * dt


def fit_turn_steps(traces: Sequence[Trace]) -> Dict[str, Any]:
    trs = _by_kind(traces, "turn_step")
    if not trs:
        return {}
    recs, accs, lats, imu_lags, imu_scales, odl = [], [], [], [], [], []
    top = None
    for tr in trs:
        cmd = float(tr.meta["cmd"])
        t_on, t_off = tr.meta.get("t_cmd_on"), tr.meta.get("t_cmd_off")
        if t_on is None or t_off is None:
            continue
        S, O = true_series(tr), odom_series(tr)
        sgn = 1.0 if cmd > 0 else -1.0
        r = step_response(S.t, S.w * sgn, t_on, t_off)
        if r is None:
            continue
        recs.append({"cmd": cmd, "plateau": r["plateau"], "saturated": r["plateau"] < 0.97 * abs(cmd), "mode": S.mode})
        if top is None or abs(cmd) > abs(top["cmd"]):
            top = recs[-1]
        if abs(cmd) >= 0.7:
            accs.append(r["a"]); lats.append(r["L"])
            ro = step_response(O.t, O.w * sgn * (S.k_ang if S.mode == "end" else 1.0), t_on, t_off)
            if ro is not None:
                odl.append(ro["L"] - r["L"])
        g = gyro_series(tr)
        if np.isfinite(g).all() and S.mode == "continuous":
            m = (S.t >= t_on - 0.3) & (S.t <= t_off + 1.0)
            imu_lags.append(_xcorr_lag(S.t[m], S.w[m], g[m]))
            ss = (S.t >= t_on + 0.6 * (t_off - t_on)) & (S.t <= t_off)
            bias = float(np.median(g[S.t <= t_on])) if (S.t <= t_on).sum() > 3 else 0.0
            wm = float(np.median(S.w[ss]))
            if abs(wm) > 0.3:
                imu_scales.append((float(np.median(g[ss])) - bias) / wm)
    out: Dict[str, Any] = {"records": recs}
    if top:
        out["w_max"] = float(top["plateau"])
        out["w_max_is_lower_bound"] = not top["saturated"]
    out.update(alpha_max=_med(accs), cmd_latency_w_s=max(0.0, _med(lats)), imu_latency_s=max(0.0, _med(imu_lags)),
               imu_scale=_med(imu_scales), odom_latency_w_s=max(0.0, _med(odl)))
    return out


def fit_straight(traces: Sequence[Trace]) -> Dict[str, Any]:
    """주행면별: 선형 스케일(오도메트리/참), 직진 편차(rad/m), 끝 횡편차. 슬립은 fit_run 이 기준 주행면과 비교해 계산."""
    out: Dict[str, Any] = {}
    for surf, trs in _group_surface(_by_kind(traces, "straight")).items():
        scales, lr, lat, dists = [], [], [], []
        for tr in trs:
            o = odom_series(tr)
            d_odom = float(np.hypot(o.x[-1] - o.x[0], o.y[-1] - o.y[0]))
            if tr.has_truth():
                x, y, th = tr.col("truth_x"), tr.col("truth_y"), np.unwrap(tr.col("truth_th"))
                # 시작 방향 기준 전진/횡 변위
                c, s = math.cos(th[0]), math.sin(th[0])
                dx, dy = x[-1] - x[0], y[-1] - y[0]
                fwd, lft = c * dx + s * dy, -s * dx + c * dy
                dhead = float(th[-1] - th[0])
            else:
                et = tr.meta.get("truth_end") or {}
                if "distance_m" not in et:
                    continue
                fwd, lft = float(et["distance_m"]), float(et.get("lateral_m", float("nan")))
                dhead = math.radians(float(et["heading_deg"])) if "heading_deg" in et else float("nan")
            if fwd < 0.2:
                continue
            scales.append(d_odom / fwd)
            lr.append(dhead / fwd)
            lat.append(lft)
            dists.append(fwd)
        if scales:
            out[surf] = {"lin_scale": _med(scales), "lr_bias": _med(lr), "lateral_end_m": _med(lat), "n": len(scales)}
    return out


def _fit_circle(x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    return float(cx), float(cy), float(math.sqrt(max(c + cx * cx + cy * cy, 0.0)))


def fit_rotate(traces: Sequence[Trace]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    eff, ysc, drift, over = [], [], [], []
    for tr in _by_kind(traces, "rotate"):
        cmd = float(tr.meta["cmd"])
        t_on, t_off = tr.meta.get("t_cmd_on"), tr.meta.get("t_cmd_off")
        if t_on is None or t_off is None:
            continue
        S, O = true_series(tr), odom_series(tr)
        ss = (S.t >= t_on + 0.4) & (S.t <= t_off)
        sl_t = np.polyfit(S.t[ss], S.th[ss], 1)[0]
        sl_o = np.polyfit(O.t[ss], O.th[ss], 1)[0]
        if S.mode == "continuous":
            eff.append(sl_t / cmd)
            ysc.append(sl_o / sl_t)
            # 회전 중 위치 이동 = 원 궤적 반경 (스키드 조향의 밀림). 중심은 평균 위치
            cx, cy, r = _fit_circle(S.x[ss], S.y[ss])
            drift.append(r)                                   # 원 궤적 반경 [m] = 회전 1 rad 당 밀림 [m/rad]
            over.append(math.degrees(float(S.th[-1] - S.th[np.nonzero(S.t <= t_off)[0][-1]])) * (1 if cmd > 0 else -1))
        elif S.mode == "end":
            float(O.th[-1] - O.th[0])
            ysc.append(1.0 / S.k_ang if S.k_ang else float("nan"))
            eff.append(abs(math.radians(float((tr.meta.get("truth_end") or {}).get("heading_deg", float("nan"))))) / max(abs(cmd) * (t_off - t_on), 1e-6))
    if eff:
        out = {"yaw_efficiency": _med(eff), "odom_yaw_scale": _med(ysc), "drift_m_per_rad": _med(drift),
               "overshoot_deg": _med(over), "n": len(eff)}
    elif ysc:
        out = {"odom_yaw_scale": _med(ysc), "n": len(ysc)}
    return out


def fit_brake(traces: Sequence[Trace]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for surf, trs in _group_surface(_by_kind(traces, "brake")).items():
        v0s, ds, accs, lats, cmds = [], [], [], [], []
        for tr in trs:
            t_off = tr.meta.get("t_cmd_off")
            if t_off is None:
                continue
            S = true_series(tr)
            pre = (S.t >= t_off - 0.5) & (S.t <= t_off)
            if pre.sum() < 3:
                continue
            v0 = float(np.median(S.v[pre]))
            if abs(v0) < 0.05:
                continue
            post = np.nonzero(S.t >= t_off)[0]
            # 정지: 속도가 5 % v0 아래로 내려간 첫 시점
            i_stop = next((i for i in post if abs(S.v[i]) < 0.05 * abs(v0)), post[-1])
            i0 = post[0]
            if S.mode == "continuous":
                c, s = math.cos(S.th[i0]), math.sin(S.th[i0])
                d = c * (S.x[i_stop] - S.x[i0]) + s * (S.y[i_stop] - S.y[i0])
            else:
                d = float(np.hypot(S.x[i_stop] - S.x[i0], S.y[i_stop] - S.y[i0]))
            a, lat = _brake_fit(S.t, S.v, t_off, v0, float(S.t[min(i_stop + 5, len(S.t) - 1)]))   # 정지 직후까지의 전 구간을 모델에 맞춤
            v0s.append(abs(v0)); ds.append(abs(d)); accs.append(a); cmds.append(round(abs(float(tr.meta.get("cmd", v0))), 2))
            lats.append(lat)
        if len(v0s) >= 2:
            v0a, da = np.array(v0s), np.array(ds)
            # d = L*v0 + v0²/(2a)
            coef, *_ = np.linalg.lstsq(np.c_[v0a, v0a ** 2 / 2], da, rcond=None)
            a_fit = 1.0 / coef[1] if coef[1] > 1e-6 else float("nan")
            table = {}
            for c in sorted(set(cmds)):                                       # 표의 키는 요청 속도, 값은 그 반복들의 평균 정지거리
                table[f"{c:.2f}"] = float(np.mean([d for cc, d in zip(cmds, ds) if cc == c]))
            a_med = _med(accs)
            out[surf] = {"a_brake": a_med, "a_brake_fit": float(a_fit), "latency_s": float(coef[0]),
                         "latency_onset_s": _med(lats), "stop_distance": table, "mu_lower_bound": a_med / G if np.isfinite(a_med) else None,
                         "n": len(v0s)}
    return out


def _tls_line(P: np.ndarray) -> Tuple[float, float, float]:
    """점들의 전체 최소제곱 직선 → (법선각 ψ, 센서→직선 수직거리 d, RMS). 법선은 센서(원점)→벽 방향."""
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c)
    d = vt[0]
    n = np.array([-d[1], d[0]])
    if n @ c < 0:
        n = -n
    rms = float(np.sqrt(np.mean(((P - c) @ n) ** 2)))
    return float(math.atan2(n[1], n[0])), float(n @ c), rms


def fit_lidar_square(traces: Sequence[Trace]) -> Dict[str, Any]:
    trs = _by_kind(traces, "lidar_square") or [t for t in traces if t.meta.get("kind") == "lidar_square"]
    if not trs:
        return {}
    res = []
    for tr in trs:
        r = np.array([np.nan if x is None else x for x in tr.meta["ranges"]], dtype=float)
        ang = tr.meta["angle_min"] + tr.meta["angle_inc"] * np.arange(len(r))
        pts = {}
        for name, center in (("A", 0.0), ("B", math.pi / 2)):
            d = np.angle(np.exp(1j * (ang - center)))
            m = (np.abs(d) < math.radians(40)) & np.isfinite(r) & (r > 0.1)
            if m.sum() < 15:
                break
            P = np.c_[r[m] * np.cos(ang[m]), r[m] * np.sin(ang[m])]
            pts[name] = _tls_line(P)
        if len(pts) < 2:
            continue
        (psiA, dA, rmsA), (psiB, dB, rmsB) = pts["A"], pts["B"]
        yawA = -psiA                                          # 정면 벽: 기준 방향 0
        yawB = -(psiB - math.pi / 2)                          # 왼쪽 벽: 기준 방향 90°
        yawB = (yawB + math.pi) % (2 * math.pi) - math.pi
        res.append({"yaw": 0.5 * (yawA + yawB), "yaw_consistency": abs(yawA - yawB),
                    "x": float(tr.meta["d_front"]) - dA, "y": float(tr.meta["d_left"]) - dB, "rms": max(rmsA, rmsB)})
    if not res:
        return {}
    return {k: _med([r[k] for r in res]) for k in ("yaw", "yaw_consistency", "x", "y", "rms")}


def fit_avoid(traces: Sequence[Trace], inscribed: float) -> Dict[str, Any]:
    out = {}
    for tr in _by_kind(traces, "avoid"):
        sm = tr.col("scan_min")
        sm = sm[np.isfinite(sm)]
        o = odom_series(tr)
        path = float(np.sum(np.hypot(np.diff(o.x), np.diff(o.y))))
        out[tr.name] = {"status": tr.meta.get("status"), "duration_s": tr.meta.get("duration"),
                        "recoveries": tr.meta.get("recoveries"), "path_len_m": path,
                        "min_lidar_range_m": float(sm.min()) if len(sm) else None,
                        "min_clearance_m": float(sm.min() - inscribed) if len(sm) else None}
    return out


# ====================================================================== 종합 → 프로파일
def fit_run(profile: Profile, traces: Sequence[Trace], ref_surface: str = "default",
            source: str = "") -> Tuple[Profile, List[str]]:
    P = profile.copy()
    L: List[str] = []
    note = f"측정: {source}" if source else "측정"

    def put(path: str, val: Any, fmt: str = "{:.4g}") -> None:
        if val is None or (isinstance(val, float) and not np.isfinite(val)):
            return
        P.set(path, val, "measured", note)
        L.append(f"- `{path}` = {fmt.format(val) if isinstance(val, float) else val}")

    # 주행면에 의존하는 값(최고속도 등)의 기준은 ref_surface 시험만 쓴다. 주행면별 값은 아래에서 따로 구한다.
    ref_traces = [t for t in traces if t.meta.get("surface", "default") == ref_surface] or list(traces)
    st = fit_static(ref_traces)
    if st:
        L.append("### static (정지)")
        put("sensors.odom.noise_v", st["odom_noise_v"]); put("sensors.odom.noise_w", st["odom_noise_w"])
        put("sensors.odom.gyro_bias_z", st["odom_gyro_bias_z"])
        if "imu_bias_z" in st:
            put("sensors.imu.gyro_bias_z", st["imu_bias_z"]); put("sensors.imu.gyro_noise_z", st["imu_noise_z"])
    sp = fit_speed_steps(ref_traces)
    if sp:
        L.append("### speed_steps (직선 속도 스텝)")
        for k, path in (("v_max", "drive.v_max"), ("v_max_reverse", "drive.v_max_reverse"), ("a_accel", "drive.a_accel"),
                        ("cmd_latency_s", "drive.cmd_latency_s"), ("rise_time_s", "drive.rise_time_s"),
                        ("odom_latency_s", "sensors.odom.latency_s")):
            put(path, sp.get(k))
        for k in ("v_max", "v_max_reverse"):
            if sp.get(k + "_is_lower_bound"):
                L.append(f"  - ⚠ `{k}`: 가장 큰 요청 속도에서도 포화되지 않았다 → 이 값은 **하한값**. 더 큰 속도를 요청해 다시 측정")
    tn = fit_turn_steps(ref_traces)
    if tn:
        L.append("### turn_steps (제자리 각속도 스텝)")
        for k, path in (("w_max", "drive.w_max"), ("alpha_max", "drive.alpha_max"), ("imu_latency_s", "sensors.imu.latency_s"),
                        ("imu_scale", "sensors.imu.gyro_scale")):
            put(path, tn.get(k))
        if tn.get("w_max_is_lower_bound"):
            L.append("  - ⚠ `w_max` 하한값 — 더 큰 각속도 요청으로 다시 측정")
    stt = fit_straight(traces)
    br = fit_brake(traces)
    rt = fit_rotate(traces)
    if stt:
        L.append("### straight (직진)")
        ref = stt.get(ref_surface) or next(iter(stt.values()))
        S_enc = ref["lin_scale"]                                   # 기준 주행면은 슬립 0 으로 가정 → 오도메트리 선형 스케일
        put("sensors.odom.lin_scale", float(S_enc))
        put("drive.lr_bias_rad_per_m", ref["lr_bias"])
        for surf, d in stt.items():
            slip = max(0.0, 1.0 - S_enc / d["lin_scale"])
            P.set(f"drive.surfaces.{surf}.slip", float(slip), "measured", f"{note} (기준면 {ref_surface} 슬립 0 가정)")
            L.append(f"- `drive.surfaces.{surf}.slip` = {slip:.4g}  (오도메트리/참 = {d['lin_scale']:.4g}, 직진편차 {d['lr_bias']:+.4f} rad/m, 끝 횡편차 {d['lateral_end_m']:+.3f} m, n={d['n']})")
    if br:
        L.append("### brake (급정지)")
        d0 = br.get(ref_surface) or next(iter(br.values()))
        put("drive.a_brake", d0["a_brake"])
        P.set("drive.stop_distance", d0["stop_distance"], "measured", note)
        L.append(f"- `drive.stop_distance` = {d0['stop_distance']}  (제동 시작 지연 {d0['latency_s']:.3f} s, 감속 fit {d0['a_brake_fit']:.3g} m/s²)")
        for surf, d in br.items():
            P.set(f"drive.surfaces.{surf}.a_brake", d["a_brake"], "measured", note)
            P.set(f"drive.surfaces.{surf}.mu_lower_bound", d["mu_lower_bound"], "measured", f"{note} (제동 감속 ÷ g — 하한값, 마찰계수 직접 측정은 견인 시험)")
            L.append(f"- `drive.surfaces.{surf}`: a_brake {d['a_brake']:.3g} m/s² → μ ≥ {d['mu_lower_bound']:.2f}")
    if rt:
        L.append("### rotate (제자리 회전)")
        put("sensors.odom.yaw_scale", rt.get("odom_yaw_scale"))
        if "yaw_efficiency" in rt:
            put("drive.pivot.yaw_efficiency", rt["yaw_efficiency"]); put("drive.pivot.drift_m_per_rad", rt["drift_m_per_rad"])
            put("drive.pivot.overshoot_deg", rt["overshoot_deg"])
    # 주행면별 가속도: speed_steps 가 surface 별로 나뉘어 있으면 표에 채운다
    for surf, trs in _group_surface(_by_kind(traces, "speed_step")).items():
        r = fit_speed_steps(trs)
        if r and np.isfinite(r.get("a_accel", float("nan"))):
            P.set(f"drive.surfaces.{surf}.a_accel", r["a_accel"], "measured", note)
    ls = fit_lidar_square(ref_traces)
    if ls:
        L.append("### lidar_square (LiDAR 장착)")
        xyz = list(P.get("sensors.lidar.xyz"))
        rpy = list(P.get("sensors.lidar.rpy"))
        xyz[0], xyz[1], rpy[2] = float(ls["x"]), float(ls["y"]), float(ls["yaw"])
        P.set("sensors.lidar.xyz", xyz, "measured", f"{note} (z 는 줄자로 직접)")
        P.set("sensors.lidar.rpy", rpy, "measured", note)
        L.append(f"- `sensors.lidar.xyz[0:2]` = ({ls['x']:+.3f}, {ls['y']:+.3f}) m, `rpy[2]` = {ls['yaw']:+.4f} rad "
                 f"(두 벽 yaw 불일치 {ls['yaw_consistency']:.4f} rad, 직선 RMS {ls['rms']*1000:.1f} mm)")
    lat = [t.col("scan_lat") for t in traces if np.isfinite(t.col("scan_lat")).any()]
    if lat:
        v = np.concatenate([x[np.isfinite(x)] for x in lat])
        if len(v) > 10:
            put("sensors.lidar.latency_s", max(0.0, float(np.median(v))))
    av = fit_avoid(traces, profile.inscribed_radius())
    if av:
        L.append("### avoid (장애물 회피)")
        for k, d in av.items():
            L.append(f"- {k}: {d}")
    # 파생: 정지거리·속도 한계를 항법 설정에 반영하기 위한 기록
    P.add_measurement({"source": source, "traces": [t.name for t in traces], "n_traces": len(traces)})
    return P, L
