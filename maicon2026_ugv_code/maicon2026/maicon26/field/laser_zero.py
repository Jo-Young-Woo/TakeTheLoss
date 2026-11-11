"""텔레옵 영점 결과(laser_zero.yaml)를 조준 설정 4개(aim.laser_*)로 바꾼다.

배경. 영점 도구는 거리 d 에서 레이저 점이 화면 (u, v) 어디에 찍히는지 재서 `u(d)=u0+au/d, v(d)=v0+av/d` 로 맞춘다.
조준 코드(mission/aim.py)는 이 식을 읽지 않고, 거리 rr 의 표적이 화면에서 있어야 할 방향을 이렇게 계산한다.
    b_des = laser_offset_pan_deg  + atan2(laser_lateral_m,  rr)      (왼쪽 +)
    e_des = laser_offset_tilt_deg + atan2(laser_vertical_m, rr)      (위 +)
Gazebo 시뮬(hal/wingbot_calibration.configure_wingbot_simulation)은 같은 모델에 '레이저 − 카메라 영점 위치' 의 옆(+Y 왼쪽)·위(+Z)
성분을 laser_lateral_m / laser_vertical_m 로 넣고 축 각도(laser_offset_*_deg)는 0 으로 둔다(발광부가 카메라와 평행하게 쏜다는 모델).
이 모듈은 그 모델을 그대로 이어받는다 — 영점 점 위치 → 각도(런타임 CameraModel 로) → β_i = 축각 + atan(옆거리/d_i) 를 최소제곱으로 맞춘다.

- 픽셀 → 각도는 **런타임 CameraModel** 로 한다(설정의 K 또는 화각). 영점 도구의 화각 가정(114.6°)과 다를 수 있으므로 거리·화각에 영향받는 것은
  각도 쪽뿐이고, 영점 식 자체는 픽셀 값이라 화각 가정과 무관하다.
- 축각을 자유롭게 맞추려면 서로 다른 거리 3곳 이상, 거리 폭 0.5 m 이상이 필요하다. 모자라면 축각 0(평행) 으로 두고 옆·위 거리만 맞춘다(Gazebo 와 같은 모델).
- 이 모듈은 파일·설정을 쓰지 않는다. 쓰기는 scripts/field_exp.py zero-import 가 provenance 와 함께 한다."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..config import Config
from ..perception.camera import CameraModel

PATHS = ("aim.laser_lateral_m", "aim.laser_vertical_m", "aim.laser_offset_pan_deg", "aim.laser_offset_tilt_deg")
DEFAULT_DISTANCES = (0.6, 1.0, 1.5, 2.0)             # 영점 도구 안내 거리(0.6 · 1.0 · 1.5 · 2.0 m)
MIN_RANGE = 0.3                                       # aim.py 가 거리를 max(r, 0.3) 으로 쓴다 — 이보다 가까운 샘플은 맞추지 않는다
FREE_MIN_POINTS, FREE_MIN_SPAN_M = 3, 0.5            # F10 과 같은 기준: 거리 차가 작으면 평행 오프셋과 축각을 못 가른다
MAX_OFFSET_M = 0.15                                   # F10 과 같은 판정선
RMS_PX_GOOD = 2.0                                     # 영점 도구 안내: rms 2 px 이하면 양호
RMS_PX_BAD = 5.0

_ALIASES = {
    "d": ("d", "dist", "distance", "distance_m", "range", "range_m", "r"),
    "u": ("u", "px_u", "x", "col"),
    "v": ("v", "px_v", "y", "row"),
    "u0": ("u0", "u_inf", "u_infinity"),
    "au": ("au", "a_u", "ku", "u_slope"),
    "v0": ("v0", "v_inf", "v_infinity"),
    "av": ("av", "a_v", "kv", "v_slope"),
}


class ZeroFormatError(ValueError):
    """영점 파일에서 필요한 값을 찾지 못했을 때(고치는 법을 메시지에 담는다)."""


def _pick(d: Dict[str, Any], key: str, required: bool = True) -> Optional[float]:
    for name in _ALIASES[key]:
        if name in d and d[name] is not None:
            try:
                return float(d[name])
            except (TypeError, ValueError):
                raise ZeroFormatError(f"'{name}' 값이 숫자가 아니다: {d[name]!r}")
    if required:
        raise ZeroFormatError(f"필요한 키 {_ALIASES[key]} 중 하나가 없다: {sorted(d)}")
    return None


@dataclass
class ZeroData:
    """영점 파일에서 읽은 값. samples = [(d, u, v)], fit = (u0, au, v0, av) — 둘 중 하나는 있어야 한다."""
    samples: List[Tuple[float, float, float]] = field(default_factory=list)
    fit: Optional[Tuple[float, float, float, float]] = None
    image_size: Optional[Tuple[int, int]] = None
    assumed_hfov_deg: Optional[float] = None
    rms_px: Optional[float] = None
    source: str = ""


def parse_zero(doc: Dict[str, Any], source: str = "") -> ZeroData:
    """laser_zero.yaml 을 읽은 dict → ZeroData. 모르는 키는 무시하고, 필요한 값이 없으면 ZeroFormatError."""
    if not isinstance(doc, dict):
        raise ZeroFormatError("영점 파일의 최상위가 매핑(dict)이 아니다")
    out = ZeroData(source=source)
    fit = doc.get("fit")
    if isinstance(fit, dict):
        out.fit = tuple(_pick(fit, k) for k in ("u0", "au", "v0", "av"))             # type: ignore[assignment]
        if fit.get("rms_px") is not None:
            out.rms_px = float(fit["rms_px"])
    if doc.get("rms_px") is not None:
        out.rms_px = float(doc["rms_px"])
    for item in doc.get("samples") or []:
        if not isinstance(item, dict):
            continue
        d, u, v = _pick(item, "d"), _pick(item, "u"), _pick(item, "v")
        if d is None or d <= 0 or not all(math.isfinite(x) for x in (d, u, v)):
            continue
        out.samples.append((d, u, v))
    if out.fit is None and not out.samples:
        raise ZeroFormatError("fit(u0, au, v0, av) 도 samples(d, u, v) 도 없다 — 영점 도구에서 `p` 로 저장한 파일인지 확인")
    for holder in (doc, doc.get("camera") or {}, doc.get("image") or {}):
        if isinstance(holder, dict) and holder.get("width") and holder.get("height"):
            out.image_size = (int(holder["width"]), int(holder["height"]))
        if isinstance(holder, dict) and holder.get("hfov_deg"):
            out.assumed_hfov_deg = float(holder["hfov_deg"])
    return out


def load_zero(path: str) -> ZeroData:
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    try:
        import yaml
        doc = yaml.safe_load(text)
    except ImportError:                                  # PyYAML 이 없으면 JSON 만
        doc = json.loads(text)
    return parse_zero(doc, source=path)


# ---------------------------------------------------------------------------------------------- 맞추기
def _fit_axis(d: Sequence[float], beta: Sequence[float], free: bool) -> Tuple[float, float, List[float]]:
    """β_i = a + atan(l/d_i) 를 최소제곱으로 맞춘다. (옆/위 거리 l [m], 축각 a [rad], 잔차 [rad]). free=False 면 a=0."""
    n = len(d)
    inv = [1.0 / x for x in d]
    # 초기값: 작은 각 근사 β ≈ a + l/d 의 선형 최소제곱
    if free and n >= 2:
        mx, my = sum(inv) / n, sum(beta) / n
        sxx = sum((x - mx) ** 2 for x in inv)
        l = sum((x - mx) * (y - my) for x, y in zip(inv, beta)) / sxx if sxx > 1e-12 else 0.0
        a = my - l * mx
    else:
        a = 0.0
        l = sum(x * math.tan(b) for x, b in zip(d, beta)) / n
    lam = 1e-9
    for _ in range(60):                                  # 가우스-뉴턴(+아주 작은 감쇠): 변수 2개라 몇 번이면 수렴한다
        res = [b - (a + math.atan(l / x)) for x, b in zip(d, beta)]
        jl = [x / (x * x + l * l) for x in d]
        if free:
            s11, s12, s22 = float(n), sum(jl), sum(j * j for j in jl)
            g1, g2 = sum(res), sum(r * j for r, j in zip(res, jl))
            det = (s11 + lam) * (s22 + lam) - s12 * s12
            if abs(det) < 1e-18:
                break
            da = (g1 * (s22 + lam) - s12 * g2) / det
            dl = ((s11 + lam) * g2 - s12 * g1) / det
        else:
            s22 = sum(j * j for j in jl) + lam
            da, dl = 0.0, sum(r * j for r, j in zip(res, jl)) / s22
        a, l = a + da, l + dl
        if abs(da) < 1e-12 and abs(dl) < 1e-12:
            break
    res = [b - (a + math.atan(l / x)) for x, b in zip(d, beta)]
    return l, a, res


@dataclass
class ZeroResult:
    """변환 결과. values 는 설정 경로 → 값(4개), 나머지는 사람이 판단할 근거."""
    values: Dict[str, float]
    rms_px: float
    max_px: float
    n_samples: int
    d_range: Tuple[float, float]
    axis_mode: str                                       # "free" | "parallel"
    source: str                                          # "samples" | "fit"
    warnings: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    per_sample: List[Dict[str, float]] = field(default_factory=list)

    @property
    def verdict(self) -> str:
        if self.rms_px > RMS_PX_BAD or any(w.startswith("!") for w in self.warnings):
            return "CHECK"
        return "PASS" if self.rms_px <= RMS_PX_GOOD else "CHECK"


def camera_for(cfg: Config) -> CameraModel:
    return CameraModel(cfg.camera)


def samples_from_fit(fit: Tuple[float, float, float, float], distances: Sequence[float]) -> List[Tuple[float, float, float]]:
    u0, au, v0, av = fit
    return [(d, u0 + au / d, v0 + av / d) for d in distances]


def convert(data: ZeroData, cam: CameraModel, distances: Optional[Sequence[float]] = None, axis: str = "auto",
            runtime_hfov_deg: Optional[float] = None) -> ZeroResult:
    """영점 데이터 → aim.laser_* 4개. axis: auto | free | parallel. 필요한 입력이 모자라면 ZeroFormatError."""
    warn: List[str] = []
    notes: List[str] = []
    if axis not in ("auto", "free", "parallel"):
        raise ValueError("axis 는 auto | free | parallel")
    # 1) 샘플 정하기 — 실제 샘플이 있으면 그것을, 없으면 맞춘 식을 잰 거리 범위에서 다시 읽는다
    if data.samples:
        samples, src = list(data.samples), "samples"
    else:
        assert data.fit is not None
        if distances:
            ds = list(distances)
        else:
            ds = list(DEFAULT_DISTANCES)
            warn.append(f"식(fit)만 있고 잰 거리가 없어 기본 거리 {DEFAULT_DISTANCES} m 에서 읽었다 — 실제로 잰 거리를 --distances 로 주면 더 정확하다")
        samples, src = samples_from_fit(data.fit, ds), "fit"
    # 2) 해상도 차이 보정(영점 화면과 런타임 화면의 크기가 다르면 비례해 옮긴다)
    if data.image_size and (data.image_size[0] != cam.w or data.image_size[1] != cam.h):
        sx, sy = cam.w / data.image_size[0], cam.h / data.image_size[1]
        if abs(sx / sy - 1.0) > 0.01:
            raise ZeroFormatError(f"영점 화면 {data.image_size} 와 런타임 화면 {(cam.w, cam.h)} 의 가로세로 비율이 다르다 — 같은 해상도로 다시 잰다")
        samples = [(d, u * sx, v * sy) for d, u, v in samples]
        warn.append(f"영점 화면 {data.image_size} → 런타임 {(cam.w, cam.h)} 로 픽셀을 비례 환산했다")
    near = [s for s in samples if s[0] < MIN_RANGE]
    if near:
        warn.append(f"거리 {MIN_RANGE} m 미만 샘플 {len(near)}개는 뺐다(조준 코드가 거리를 {MIN_RANGE} m 이상으로 쓴다)")
        samples = [s for s in samples if s[0] >= MIN_RANGE]
    if not samples:
        raise ZeroFormatError("쓸 수 있는 샘플이 없다(거리 0.3 m 이상, 숫자 값)")
    ds = [s[0] for s in samples]
    span = max(ds) - min(ds)
    distinct = len({round(x, 3) for x in ds})
    # 3) 축각을 자유롭게 둘지
    if axis == "free":
        free = True
        if distinct < 2:
            raise ZeroFormatError("축각을 자유롭게 맞추려면 서로 다른 거리 2곳 이상이 필요하다")
    elif axis == "parallel":
        free = False
    else:
        free = distinct >= FREE_MIN_POINTS and span >= FREE_MIN_SPAN_M
    if not free and axis == "auto":
        notes.append(f"거리 {distinct}곳·폭 {span:.2f} m — 옆·위 거리와 축각을 가르기에 모자라(3곳·0.5 m 이상 필요) 축각을 0 으로 두고 옆·위 거리만 맞췄다(Gazebo 와 같은 모델)")
    if distinct == 1:
        warn.append("거리 1곳뿐이라 점 위치 전부를 평행 오프셋으로 해석했다(축각 0 가정) — 옆·위 거리와 축각을 가르려면 거리 2곳 이상(권장 3곳, 폭 0.5 m 이상) 다시 잰다")
    if free and distinct == 2:
        warn.append("거리 2곳만으로 축각까지 맞췄다 — 잡음에 민감하다. 3곳 이상 권장")
    # 4) 각도로 바꿔 맞추기
    beta = [cam.bearing(u, v) for _, u, v in samples]
    elev = [cam.elevation(v, u) for _, u, v in samples]
    lat, a_pan, r_u = _fit_axis(ds, beta, free)
    ver, a_tilt, r_v = _fit_axis(ds, elev, free)
    # 5) 픽셀 잔차(맞춘 값을 조준 코드 모델에 넣었을 때 점이 어디로 가는지)
    per: List[Dict[str, float]] = []
    err_u, err_v = [], []
    for (d, u, v) in samples:
        b = a_pan + math.atan(lat / d)
        e = a_tilt + math.atan(ver / d)
        pr = cam.project(b, e)
        if pr is None:
            warn.append(f"! 거리 {d} m 의 예측 점이 카메라 시야 밖이다 — 입력 부호·거리를 확인")
            continue
        eu, ev = pr[0] - u, pr[1] - v
        err_u.append(eu)
        err_v.append(ev)
        per.append({"d": d, "u": u, "v": v, "u_pred": pr[0], "v_pred": pr[1], "err_u": eu, "err_v": ev})
    n = max(len(err_u), 1)
    rms = math.sqrt(sum(x * x + y * y for x, y in zip(err_u, err_v)) / n)
    mx = max([math.hypot(x, y) for x, y in zip(err_u, err_v)] or [0.0])
    # 6) 판정·경고
    if max(abs(lat), abs(ver)) > MAX_OFFSET_M:
        warn.append(f"! 옆/위 오프셋이 {MAX_OFFSET_M * 100:.0f} cm 를 넘는다(옆 {lat * 100:.1f} cm, 위 {ver * 100:.1f} cm) — 부호·거리 입력을 확인")
    if abs(ver - 0.0465) > 0.03 and src:
        notes.append(f"참고: 사진 역산 추정(레이저가 카메라 4.65 cm 위)과 {abs(ver - 0.0465) * 100:.1f} cm 차이 — 실측이 우선이지만 부호가 맞는지 한 번 확인")
    if free and (abs(math.degrees(a_pan)) > 5 or abs(math.degrees(a_tilt)) > 5):
        warn.append(f"! 축각 오차가 5° 를 넘는다(팬 {math.degrees(a_pan):.2f}°, 틸트 {math.degrees(a_tilt):.2f}°) — 레이저 고정 상태·영점 데이터를 확인")
    if rms > RMS_PX_GOOD:
        warn.append(f"맞춘 모델이 점 위치와 rms {rms:.2f} px 어긋난다(양호 기준 {RMS_PX_GOOD} px)" + (" — 거리를 더 다양하게 재거나 샘플을 확인" if rms > RMS_PX_BAD else ""))
    if data.rms_px is not None and data.rms_px > RMS_PX_GOOD:
        warn.append(f"영점 도구가 낸 rms_px 가 {data.rms_px:.2f} px 로 이미 크다 — 영점 데이터부터 다시 잰다")
    if data.assumed_hfov_deg and runtime_hfov_deg and abs(data.assumed_hfov_deg - runtime_hfov_deg) > 2.0:
        notes.append(f"영점 도구의 화각 가정 {data.assumed_hfov_deg:.1f}° 와 런타임 화각 {runtime_hfov_deg:.1f}° 가 다르다 — 점 위치(픽셀)는 영향이 없고, 각도는 런타임 카메라로 다시 계산했다")
    vals = {PATHS[0]: round(lat, 4), PATHS[1]: round(ver, 4),
            PATHS[2]: round(math.degrees(a_pan), 3), PATHS[3]: round(math.degrees(a_tilt), 3)}
    return ZeroResult(values=vals, rms_px=rms, max_px=mx, n_samples=len(samples), d_range=(min(ds), max(ds)),
                      axis_mode="free" if free else "parallel", source=src, warnings=warn, notes=notes, per_sample=per)


# ---------------------------------------------------------------------------------------------- 기록(필드 실험과 같은 모양)
def to_record(res: ZeroResult, cfg: Config, source_path: str, robot: str = "real", note: str = "", now=None) -> Dict[str, Any]:
    """field/records 에 두는 기록 dict — `field_exp.py suggest/apply` 가 F01~F10 과 똑같이 읽는다."""
    import datetime as _dt
    t = now or _dt.datetime.now()
    cam = cfg.camera
    k = getattr(cam, "K", None)
    sug = [{"path": p, "value": res.values[p],
            "note": f"영점 변환({res.source}, 축각 {res.axis_mode}, 거리 {res.d_range[0]:.2f}–{res.d_range[1]:.2f} m, rms {res.rms_px:.2f} px)"} for p in PATHS]
    meta = {"카메라 K": list(k) if k else None, "카메라 해상도": [cam.width, cam.height], "화각(K 없을 때)": cam.hfov_deg}
    rec = {"id": "ZERO", "title": "텔레옵 영점 → aim.laser_* 변환", "date": t.date().isoformat(), "time": t.strftime("%H:%M:%S"), "robot": robot,
           "note": note, "data": {"source_file": source_path, "samples": res.per_sample, "camera_used": meta},
           "analysis": {"verdict": res.verdict, "metrics": {"rms_px": round(res.rms_px, 3), "max_px": round(res.max_px, 3), "n": res.n_samples,
                                                              "d_min": res.d_range[0], "d_max": res.d_range[1], "axis": res.axis_mode},
                        "notes": res.notes + res.warnings, "suggestions": sug},
           "related_tunables": {p: None for p in PATHS}}
    if not k:
        rec["analysis"]["notes"].append("카메라 K 가 설정에 없어 화각으로 근사한 초점거리를 썼다 — 실제 K 가 정해지면 같은 영점 파일로 다시 변환할 것")
    return rec


def predict_spot_pixels(cam: CameraModel, lateral_m: float, vertical_m: float, pan_deg: float, tilt_deg: float,
                        distances: Sequence[float]) -> List[Tuple[float, float, float]]:
    """물리 모델(시험·점검용): 카메라에서 옆 lateral·위 vertical 에 있고, 카메라 축에서 (pan, tilt)° 틀어 쏘는 레이저가
    거리 d(카메라 축 방향 깊이) 의 평면에 맺는 점의 화면 위치. Gazebo 와 같은 약속(왼쪽 +, 위 +)."""
    out = []
    for d in distances:
        x_left = lateral_m + d * math.tan(math.radians(pan_deg))
        y_up = vertical_m + d * math.tan(math.radians(tilt_deg))
        pr = cam.project(math.atan2(x_left, d), math.atan2(y_up, d))
        if pr is None:
            raise ValueError(f"d={d} 의 점이 시야 밖")
        out.append((d, pr[0], pr[1]))
    return out
