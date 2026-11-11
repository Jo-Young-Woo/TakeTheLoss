"""들어오는 데이터의 공통 유효성 검사 (프롬프트 9장). rclpy·cv2 를 import 하지 않는다 — 순수 파이썬 + numpy.

규칙 (모든 검사 함수 공통)
  - 나쁜 값은 **버리거나 기록만** 한다. 고쳐서 쓰지 않는다(상자 자르기 V5·문장 자르기 V6 제외).
  - 버린 데이터로는 '수신 시각'을 갱신하지 않는다 → 계속 나쁘면 준비 확인·신선도 검사가 자연히 '없음'으로 잡는다.
  - 예외를 밖으로 던지지 않는다. 검사 함수는 이유 문자열(또는 None=정상)을 돌려주고, 기록은 ValidationLog 가 한다.
  - 버린 이유는 robot.stats["invalid"][출처][이유] 로 센다. 같은 이유의 경고는 5 s 에 한 줄만 낸다.
  - 2 s 마다 validation_status.json 을 별도 스레드에서 쓴다(노트북 tools/notebooks/oak_field_check.ipynb 가 읽는다).

이 모듈은 hal/·perception/·link/ 어디에서도 import 해도 순환하지 않도록 다른 maicon26 모듈을 import 하지 않는다
(API 문서의 cmd 목록처럼 다른 모듈이 가진 사실은 인자로 받는다).
"""
from __future__ import annotations

import datetime
import json
import logging
import math
import os
import threading
import time
import unicodedata
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

log = logging.getLogger("maicon26.validate")

WARN_INTERVAL_S = 5.0          # 같은 (출처, 이유) 경고 최소 간격
STATUS_INTERVAL_S = 2.0        # validation_status.json 쓰기 간격
MAX_REASONS_PER_SOURCE = 32    # 한 출처에서 서로 다른 이유 키가 무한히 늘지 않게(원문을 이유에 넣는 V2·V3 때문)
STATUS_SCHEMA = 1

MATCH_MODES = ("practice", "mission")                       # /mil/match mode — 2차 API 문서 p11
FIRE_RESPONSES = ("fired", "empty", "locked", "reloading")  # /mil/fire 응답 — p16
GIMBAL_RESPONSES = ("ok", "locked")                         # /mil/gimbal 응답 — p14 (ok 는 접수)
ISSUED_AT_FORMAT = "%Y-%m-%d %H:%M:%S"                      # 서버 /order·/target issued_at (KST) — link/clock_sync.py 와 같다


def run_log_dir() -> Path:
    """실행 로그 폴더. 저장소 밖이다. MAICON_LOG_DIR 로 바꾼다. 만들지는 않는다."""
    env = os.environ.get("MAICON_LOG_DIR", "").strip()
    return Path(env).expanduser() if env else Path.home() / "maicon_run_logs"


def finite(x: Any) -> bool:
    """bool 이 아닌 수이고 유한하다."""
    if isinstance(x, bool):
        return False
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError, OverflowError):
        return False


def raw_label(x: Any, n: int = 32) -> str:
    """모르는 문자열을 로그·이유 키에 넣을 때 쓰는 원문 앞 n 자(제어문자는 '?')."""
    s = str(x)[:n]
    return "".join(c if c.isprintable() else "?" for c in s)


# ============================================================================ 기록
class ValidationLog:
    """버린 데이터를 세고, 경고를 5 s 에 한 번으로 줄이고, 상태 파일을 쓴다. 모든 메서드는 예외를 던지지 않는다."""

    def __init__(self, stats: Optional[Dict[str, Any]] = None, *, clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time, warn: Optional[Callable[[str], None]] = None,
                 warn_interval_s: float = WARN_INTERVAL_S, status_path: Optional[os.PathLike] = None,
                 status_interval_s: float = STATUS_INTERVAL_S) -> None:
        self.stats: Dict[str, Any] = stats if stats is not None else {}
        self.counts: Dict[str, Dict[str, int]] = self.stats.setdefault("invalid", {})
        self._clock, self._wall = clock, wall
        self._warn_fn = warn or log.warning
        self.warn_interval_s = max(float(warn_interval_s), 0.0)
        self.status_path = Path(status_path) if status_path else None
        self.status_interval_s = max(float(status_interval_s), 0.05)
        self._lock = threading.Lock()
        self._last_warn: Dict[Tuple[str, str], float] = {}
        self._suppressed: Dict[Tuple[str, str], int] = {}
        self._last_reason: Dict[str, str] = {}
        self._last_detail: Dict[str, str] = {}
        self._last_bad_wall: Dict[str, float] = {}
        self._last_good_wall: Dict[str, float] = {}
        self._good_count: Dict[str, int] = {}
        self._t0, self._wall0 = clock(), wall()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.write_errors = 0
        self.warnings_emitted = 0

    # ------------------------------------------------------------ 기록
    def bad(self, source: str, reason: str, detail: str = "") -> None:
        """source 의 데이터 한 건을 reason 으로 버렸다(또는 의심 기록만 했다)."""
        try:
            with self._lock:
                per = self.counts.setdefault(source, {})
                if reason not in per and len(per) >= MAX_REASONS_PER_SOURCE:
                    reason = "other"
                per[reason] = per.get(reason, 0) + 1
                self._last_reason[source] = reason
                self._last_detail[source] = detail
                self._last_bad_wall[source] = self._wall()
                key = (source, reason)
                now = self._clock()
                last = self._last_warn.get(key)
                if last is not None and now - last < self.warn_interval_s:
                    self._suppressed[key] = self._suppressed.get(key, 0) + 1
                    return
                self._last_warn[key] = now
                skipped = self._suppressed.pop(key, 0)
                self.warnings_emitted += 1
            msg = f"[검증] {source}: {reason}" + (f" ({detail})" if detail else "") + (f" [이전 {skipped}건 생략]" if skipped else "")
            self._warn_fn(msg)
        except Exception:                                   # 기록 실패가 제어 루프를 죽이면 안 된다
            pass

    def good(self, source: str) -> None:
        """source 의 정상 데이터를 받았다(마지막 유효 시각 갱신)."""
        try:
            with self._lock:
                self._last_good_wall[source] = self._wall()
                self._good_count[source] = self._good_count.get(source, 0) + 1
        except Exception:
            pass

    def absorb(self, other: "ValidationLog") -> None:
        """다른 기록(예: robot 을 만들기 전에 설정 점검이 쓴 기록)의 개수와 마지막 이유를 이쪽에 합친다. 예외를 던지지 않는다."""
        try:
            snap = other.snapshot()["sources"]
            with self._lock:
                for source, info in snap.items():
                    per = self.counts.setdefault(source, {})
                    for reason, n in info["reasons"].items():
                        per[reason] = per.get(reason, 0) + int(n)
                    if info["last_reason"] is not None:
                        self._last_reason[source] = info["last_reason"]
                        self._last_detail[source] = info["last_detail"] or ""
                        self._last_bad_wall[source] = info["last_invalid_wall"] or self._wall()
        except Exception:
            pass

    def total(self, source: Optional[str] = None) -> int:
        with self._lock:
            if source is not None:
                return sum(self.counts.get(source, {}).values())
            return sum(sum(v.values()) for v in self.counts.values())

    # ------------------------------------------------------------ 상태 파일
    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            wall = self._wall()
            sources: Dict[str, Any] = {}
            for s in sorted(set(self.counts) | set(self._last_good_wall)):
                per = dict(self.counts.get(s, {}))
                good = self._last_good_wall.get(s)
                bad = self._last_bad_wall.get(s)
                sources[s] = {
                    "dropped": int(sum(per.values())),
                    "reasons": per,
                    "last_reason": self._last_reason.get(s),
                    "last_detail": self._last_detail.get(s) or None,
                    "last_invalid_wall": bad,
                    "last_valid_wall": good,
                    "last_valid_age_s": None if good is None else max(wall - good, 0.0),
                    "valid_count": self._good_count.get(s, 0),
                }
            return {"schema": STATUS_SCHEMA, "written_wall": wall, "uptime_s": max(self._clock() - self._t0, 0.0),
                    "warnings_emitted": self.warnings_emitted, "sources": sources}

    def write_status(self) -> bool:
        """validation_status.json 을 임시 파일로 쓴 뒤 바꿔치기한다. 실패는 경고만."""
        if self.status_path is None:
            return False
        try:
            data = json.dumps(self.snapshot(), ensure_ascii=False, indent=1, default=str)
            self.status_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.status_path.with_name(self.status_path.name + ".tmp")
            tmp.write_text(data, encoding="utf-8")
            os.replace(tmp, self.status_path)
            return True
        except Exception as e:
            self.write_errors += 1
            if self.write_errors == 1 or self.write_errors % 30 == 0:
                try:
                    self._warn_fn(f"[검증] 상태 파일을 쓰지 못했다: {type(e).__name__}: {e}")
                except Exception:
                    pass
            return False

    def start_writer(self) -> None:
        """상태 파일을 status_interval_s 마다 쓰는 데몬 스레드를 시작한다(경로가 없으면 아무것도 안 한다)."""
        if self.status_path is None or (self._thread is not None and self._thread.is_alive()):
            return
        self._stop.clear()

        def run() -> None:
            while not self._stop.wait(self.status_interval_s):
                self.write_status()
            self.write_status()

        self._thread = threading.Thread(target=run, name="validation-status", daemon=True)
        self._thread.start()

    def stop_writer(self, timeout: float = 1.0) -> None:
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout)
        self._thread = None


def guarded(vlog: Optional[ValidationLog], source: str, fn: Callable, *, default: Any = None) -> Callable:
    """콜백을 감싼다: 예외는 세고 기록한 뒤 default 를 돌려준다(예외를 밖으로 던지지 않는다)."""
    def wrapped(*a, **k):
        try:
            return fn(*a, **k)
        except Exception as e:                              # noqa: BLE001 — 콜백 경계
            if vlog is not None:
                vlog.bad(source, "exception", f"{type(e).__name__}: {e}"[:80])
            return default
    wrapped.__name__ = getattr(fn, "__name__", "guarded")
    return wrapped


# ============================================================================ V1 — /mil/fire left_ms
def check_left_ms(left_ms: Any, max_ms: float = 15000.0) -> Tuple[Optional[float], Optional[str]]:
    """(쓸 값, 버린 이유). 없으면(None) (None, None). 유한하고 0 ≤ left_ms ≤ max_ms 일 때만 쓴다."""
    if left_ms is None:
        return None, None
    if isinstance(left_ms, bool) or not isinstance(left_ms, (int, float, np.integer, np.floating)):
        return None, "left_ms_type"
    v = float(left_ms)
    if not math.isfinite(v):
        return None, "left_ms_nonfinite"
    if v < 0:
        return None, "left_ms_negative"
    if v > max_ms:
        return None, "left_ms_too_large"
    return v, None


# ============================================================================ V2 — 서비스 응답 문자열
def unknown_response(kind: str, msg: Optional[str]) -> Optional[str]:
    """kind 'fire'|'gimbal'. 알려진 응답이면 None, 모르는 문자열이면 '<kind>:<원문 앞 32자>'. msg 가 None(호출 실패)이면 None."""
    if msg is None:
        return None
    known = FIRE_RESPONSES if kind == "fire" else GIMBAL_RESPONSES
    m = str(msg).strip().lower()
    if m in known:
        return None
    return f"unknown:{raw_label(msg)}"


# ============================================================================ V3 — /mil/match
def unknown_match_fields(mode: Any, cmd: Any, known_cmds: Iterable[str]) -> List[str]:
    """모르는 값마다 이유 문자열('unknown_mode:<원문>' / 'unknown_cmd:<원문>'). 빈 문자열은 '없음'이지 모르는 값이 아니다."""
    out: List[str] = []
    m, c = str(mode or "").strip().lower(), str(cmd or "").strip().lower()
    if m and m not in MATCH_MODES:
        out.append("unknown_mode:" + raw_label(mode))
    if c and c not in tuple(known_cmds):
        out.append("unknown_cmd:" + raw_label(cmd))
    return out


def is_practice(mode: Any) -> bool:
    """연습 모드로 볼 수 있는가 — 정확히 'practice' 일 때만. 모르는 값은 연습 모드가 아니다(2장)."""
    return str(mode or "").strip().lower() == "practice"


# ============================================================================ V4 — /image_raw
def check_image_message(height: Any, width: Any, step: Any, channels: int, buf_len: Any,
                        expect_w: Optional[int] = None, expect_h: Optional[int] = None) -> Optional[str]:
    """영상 메시지의 모양을 본다. 정상이면 None, 아니면 이유.

    h·w > 0, step ≥ w·채널 수, 버퍼 길이 ≥ h·step, 그리고 설정(camera.width×height)과 같은 크기."""
    try:
        h, w, st, n = int(height), int(width), int(step), int(buf_len)
    except (TypeError, ValueError, OverflowError):
        return "bad_dims"
    if h <= 0 or w <= 0:
        return "bad_dims"
    if st < w * channels:
        return "bad_step"
    if n < h * st:
        return "short_buffer"
    if expect_w is not None and expect_h is not None and (w != int(expect_w) or h != int(expect_h)):
        return "size_mismatch"
    return None


class FrameWatch:
    """카메라 멈춤·암흑 감시. 프레임당 비용이 1 ms 미만이어야 한다.

    멈춤: header stamp 가 직전과 같은 값이거나, 고정 위치 원본 화소(기본 2000개 × 3채널)가 직전 프레임과 완전히 같은 상태가
    frozen_frames 장 이어지면 frozen. 축소 영상의 해시는 쓰지 않는다 — 정지한 장면은 축소하면 센서 잡음이 평균돼 사라져
    멀쩡한 카메라를 멈춤으로 오판한다.
    암흑: 평균 < 5 또는 > 250 이 연속 dark_frames 장이면 dark(경고만 — 실제로 어두운 곳일 수 있다).
    """

    def __init__(self, frozen_frames: int = 20, dark_frames: int = 30, n_pixels: int = 2000, seed: int = 20260531) -> None:
        self.frozen_frames = max(int(frozen_frames), 2)
        self.dark_frames = max(int(dark_frames), 1)
        self.n_pixels = max(int(n_pixels), 16)
        self._rng = np.random.default_rng(seed)
        self._idx: Dict[Tuple[int, int], Tuple[np.ndarray, np.ndarray]] = {}
        self._prev_px: Optional[np.ndarray] = None
        self._prev_shape: Optional[Tuple[int, ...]] = None
        self._prev_stamp: Optional[float] = None
        self._same_run = 0
        self._dark_run = 0
        self.frozen = False
        self.dark = False

    def reset(self) -> None:
        self._prev_px = self._prev_shape = self._prev_stamp = None
        self._same_run = self._dark_run = 0
        self.frozen = self.dark = False

    def _positions(self, h: int, w: int) -> Tuple[np.ndarray, np.ndarray]:
        key = (h, w)
        pos = self._idx.get(key)
        if pos is None:
            pos = (self._rng.integers(0, h, self.n_pixels), self._rng.integers(0, w, self.n_pixels))
            self._idx[key] = pos
        return pos

    def update(self, image: np.ndarray, stamp: Optional[float] = None) -> Tuple[bool, bool]:
        """(frozen, dark). 예외를 던지지 않는다(이상한 입력은 '정상'으로 본다)."""
        try:
            h, w = int(image.shape[0]), int(image.shape[1])
            ys, xs = self._positions(h, w)
            px = image[ys, xs]
            same_stamp = (stamp is not None and finite(stamp) and stamp > 0 and self._prev_stamp is not None
                          and stamp == self._prev_stamp)
            same_px = (self._prev_px is not None and self._prev_shape == image.shape and bool(np.array_equal(px, self._prev_px)))
            self._same_run = self._same_run + 1 if (same_stamp or same_px) else 1
            self._prev_px, self._prev_shape = px, image.shape
            self._prev_stamp = float(stamp) if stamp is not None and finite(stamp) else None
            self.frozen = self._same_run >= self.frozen_frames
            mean = float(image[::4, ::4].mean())
            self._dark_run = self._dark_run + 1 if (mean < 5.0 or mean > 250.0) else 0
            self.dark = self._dark_run >= self.dark_frames
        except Exception:
            self.frozen = self.dark = False
        return self.frozen, self.dark


# ============================================================================ V5 — 검출 상자
def sanitize_detections(dets: Iterable[Any], width: float, height: float, min_box_px: float = 4.0
                        ) -> Tuple[List[Any], List[str]]:
    """(쓸 검출, 버린 이유 목록). 팬틸트와 OAK 검출에 똑같이 적용한다.

    버림: 좌표가 유한하지 않다 / x1 ≥ x2 또는 y1 ≥ y2 / 완전히 화면 밖 / 너비·높이 < min_box_px / conf 가 0–1 밖(NaN 포함).
    자름: 화면 밖으로 나간 부분만 잘라 낸다. 정상 검출은 같은 객체를 그대로 돌려준다(출력이 바뀌지 않는다).
    """
    kept: List[Any] = []
    why: List[str] = []
    W, H = float(width), float(height)
    for d in dets or ():
        try:
            box = tuple(d.box)
            if len(box) != 4 or not all(finite(v) for v in box):
                why.append("box_nonfinite")
                continue
            x1, y1, x2, y2 = (float(v) for v in box)
            if x1 >= x2 or y1 >= y2:
                why.append("box_inverted")
                continue
            if not finite(d.conf) or not 0.0 <= float(d.conf) <= 1.0:
                why.append("conf_range")
                continue
            if x2 <= 0 or y2 <= 0 or x1 >= W or y1 >= H:
                why.append("box_outside")
                continue
            cx1, cy1, cx2, cy2 = max(x1, 0.0), max(y1, 0.0), min(x2, W), min(y2, H)
            if cx2 - cx1 < min_box_px or cy2 - cy1 < min_box_px:
                why.append("box_too_small")
                continue
            if (cx1, cy1, cx2, cy2) != (x1, y1, x2, y2):
                d = replace(d, box=(cx1, cy1, cx2, cy2))
            kept.append(d)
        except Exception:
            why.append("detection_malformed")
    return kept, why


# ============================================================================ V6 — 서버 /order·/target
def clean_issued_at(v: Any) -> Optional[Any]:
    """issued_at 이 문서 형식('YYYY-MM-DD HH:MM:SS') 문자열이거나 유한한 양수일 때만 그대로 돌려준다. 아니면 None(= 없는 것)."""
    if v is None or isinstance(v, bool):
        return None
    if isinstance(v, (int, float, np.integer, np.floating)):
        return v if math.isfinite(float(v)) and float(v) > 0 else None
    if isinstance(v, str):
        try:
            datetime.datetime.strptime(v, ISSUED_AT_FORMAT)
        except ValueError:
            return None
        return v
    return None


def sanitize_command_text(text: Any, max_chars: int = 500) -> Tuple[str, bool, int]:
    """(정리된 문장, 잘렸는가, 제거한 제어문자 수). 줄바꿈·탭은 공백 하나로, 그 밖의 제어문자는 뺀다. 길이는 max_chars 로 자른다."""
    s = "" if text is None else str(text)
    out: List[str] = []
    removed = 0
    for ch in s:
        if ch in "\r\n\t":
            out.append(" ")
        elif unicodedata.category(ch) == "Cc":
            removed += 1
        else:
            out.append(ch)
    clean = "".join(out)
    cut = len(clean) > max(int(max_chars), 0)
    return (clean[: max(int(max_chars), 0)] if cut else clean), cut, removed


# ============================================================================ V7 — odom 연속성
def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class OdomJumpDetector:
    """연속한 odom 사이 이동이 v_cap·dt·3 + 0.05 m, 회전이 w_cap·dt·3 + 0.1 rad 를 넘으면 odom_jump. **값은 바꾸지 않는다.**"""

    def __init__(self, v_cap: float = 0.6, w_cap: float = 3.0, factor: float = 3.0,
                 margin_m: float = 0.05, margin_rad: float = 0.1) -> None:
        self.v_cap, self.w_cap, self.factor = float(v_cap), float(w_cap), float(factor)
        self.margin_m, self.margin_rad = float(margin_m), float(margin_rad)
        self._prev: Optional[Tuple[float, float, float, float]] = None
        self.jumps = 0

    def update(self, t: float, x: float, y: float, th: float) -> Optional[str]:
        """정상이면 None, 튀었으면 'odom_jump'. 유한하지 않은 입력은 기준으로 삼지 않고 None."""
        if not all(finite(v) for v in (t, x, y, th)):
            return None
        prev, self._prev = self._prev, (float(t), float(x), float(y), float(th))
        if prev is None:
            return None
        dt = max(float(t) - prev[0], 0.0)
        dist = math.hypot(x - prev[1], y - prev[2])
        rot = abs(wrap_pi(th - prev[3]))
        if dist > self.v_cap * dt * self.factor + self.margin_m or rot > self.w_cap * dt * self.factor + self.margin_rad:
            self.jumps += 1
            return "odom_jump"
        return None


# ============================================================================ V8 — 새 데이터(OAK·어안·설정)
def check_camera_matrix(K: Any, width: Optional[float] = None, height: Optional[float] = None) -> Optional[str]:
    """K (3×3 또는 길이 9): 유한, fx·fy > 0, 주점이 영상 안. 정상이면 None."""
    try:
        k = np.asarray(K, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return "K_malformed"
    if k.size != 9:
        return "K_malformed"
    if not np.all(np.isfinite(k)):
        return "K_nonfinite"
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if fx <= 0 or fy <= 0:
        return "K_focal"
    if width is not None and not 0 < cx < width:
        return "K_principal_x"
    if height is not None and not 0 < cy < height:
        return "K_principal_y"
    return None


def check_fisheye_D(D: Any) -> Optional[str]:
    """어안 왜곡 계수 k1..k4: 4개이고 모두 유한. 정상이면 None."""
    try:
        d = np.asarray(D, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return "D_malformed"
    if d.size != 4:
        return "D_count"
    if not np.all(np.isfinite(d)):
        return "D_nonfinite"
    return None


def check_aligned_size(width: Any, height: Any, multiple: int = 16) -> Optional[str]:
    """깊이를 컬러에 정렬한 프로파일은 폭·높이가 16 의 배수여야 한다."""
    try:
        w, h = int(width), int(height)
    except (TypeError, ValueError):
        return "size_malformed"
    return None if (w > 0 and h > 0 and w % multiple == 0 and h % multiple == 0) else "size_not_multiple_of_16"


# ============================================================================ V8 — 설정 키 점검
# (섹션, [(키, 하한, 상한, 정수?)]) — impact · pose_guard 수치 범위. 값은 모두 [가정]이라 넓게 잡고, 벗어나면 기본값으로 되돌린다.
_GUARD_RULES = {
    "impact": [("trigger_dg", 0.05, 5.0, False), ("major_dg", 0.1, 10.0, False), ("gyro_dps", 20.0, 2000.0, False),
               ("major_gyro_dps", 20.0, 4000.0, False), ("noise_k", 1.0, 20.0, False), ("refractory_s", 0.05, 3.0, False),
               ("hold_s", 0.0, 5.0, False), ("hold_major_s", 0.0, 10.0, False), ("verify_timeout_s", 0.2, 10.0, False),
               ("contact_lidar_m", 0.05, 1.0, False), ("ugv_contact_m", 0.2, 2.0, False), ("storm_n", 2, 20, True),
               ("storm_window_s", 1.0, 120.0, False), ("storm_hold_s", 0.0, 10.0, False), ("scan_skip_s", 0.0, 3.0, False),
               ("tilt_scan_skip_deg", 1.0, 30.0, False)],
    "pose_guard": [("shift_detect_deg", 0.5, 20.0, False), ("shift_confirm_scans", 3, 100, True), ("shift_max_deg", 5.0, 90.0, False),
                   ("fallback_deg", 1.0, 40.0, False), ("pre_window_s", 1.0, 60.0, False), ("pre_std_deg", 0.2, 10.0, False),
                   ("post_std_deg", 0.2, 10.0, False), ("settle_s", 0.0, 3.0, False), ("quiet_rot_dps", 1.0, 90.0, False),
                   ("quiet_lin", 0.01, 0.5, False), ("verify_timeout_s", 0.5, 10.0, False), ("step_trigger_deg", 1.0, 40.0, False),
                   ("dead_reckon_s", 0.2, 20.0, False), ("jump_m", 0.1, 3.0, False), ("jump_deg", 5.0, 90.0, False),
                   ("jump_confirm_n", 1, 50, True), ("link_period_s", 0.2, 30.0, False), ("link_rot_dps", 1.0, 180.0, False),
                   ("link_lin", 0.02, 1.0, False), ("link_search_xy", 0.02, 0.6, False), ("link_search_deg", 1.0, 20.0, False),
                   ("link_baseline_min", 0.05, 1.0, False), ("link_lost_ratio", 0.01, 1.0, False), ("link_drop", 0.05, 1.0, False),
                   ("link_trim_gain", 0.02, 1.0, False), ("link_trim_max_m", 0.0, 0.5, False), ("link_trim_max_deg", 0.0, 20.0, False),
                   ("degraded_speed_scale", 0.2, 1.0, False),
                   ("reanchor_link_n", 1, 20, True), ("reanchor_plan_spread_m", 0.0, 0.5, False),
                   ("reanchor_ugv_radius_m", 0.05, 1.0, False), ("reanchor_ugv_margin_m", 0.0, 1.0, False), ("reanchor_search_xy", 0.1, 1.0, False), ("reanchor_search_deg", 3.0, 40.0, False),
                   ("reanchor_min_ratio", 0.05, 1.0, False), ("reanchor_min_gain", 0.0, 1.0, False), ("reanchor_ambiguity", 0.0, 0.5, False),
                   ("reanchor_min_move_m", 0.0, 0.3, False), ("reanchor_min_move_deg", 0.0, 10.0, False), ("reanchor_max_m", 0.05, 2.0, False),
                   ("reanchor_max_deg", 1.0, 60.0, False), ("reanchor_total_m", 0.05, 4.0, False), ("reanchor_total_deg", 1.0, 90.0, False),
                   ("reanchor_confirm_n", 1, 20, True), ("reanchor_ptp_m", 0.005, 0.5, False), ("reanchor_ptp_deg", 0.2, 10.0, False),
                   ("reanchor_window_s", 1.0, 120.0, False), ("reanchor_hold_s", 0.0, 15.0, False), ("reanchor_cooldown_s", 0.0, 300.0, False),
                   ("structures_min_boxes", 0, 40, True), ("reanchor_max_count", 0, 50, True), ("reanchor_verify_n", 1, 20, True), ("reanchor_verify_drop", 0.0, 0.5, False)],
}


def _default_of(section: Any, key: str) -> Any:
    """같은 dataclass 의 기본값(새 인스턴스에서 읽는다). 못 읽으면 None."""
    try:
        return getattr(type(section)(), key)
    except Exception:
        return None


def sanitize_config(cfg: Any, vlog: Optional[ValidationLog] = None, root: Optional[os.PathLike] = None) -> List[str]:
    """새 설정 키의 값이 이상하면 경고하고 기본값으로 되돌린다. 바꾼 키 이름('camera.model' …)의 목록을 돌려준다.

    예외를 던지지 않는다(/ready 전 단계에 새 예외를 넣지 않는다). 기본 설정에서는 아무것도 바꾸지 않는다.
      camera.model            pinhole | fisheye
      camera.usable_hfov_deg  None 또는 (0, 180]
      oak_det.max_hz          유한한 양수
      oak_det.vote_window     None 또는 1 이상의 정수
      oak_det.save_max        0..10 으로 자른다
      aim.gimbal_settle_s     유한한 0 이상
      mission.stage_policy    enemy_middle | legacy
      mission.stage_standoff_m  유한한 0.05..1.5 m
      explore.m1_sector_weight  M1 지정 구역 가중이 0.5..5 안이어야 한다
      uav_boxes.*             실기 UAV 상자 검출기 수치(HSV 문턱·ppm·채움률·최소/최대 크기·여유 거리)가 범위 안이어야 하고,
                              top_color 는 black|grey|both, ignore_rects 는 (x0,y0,x1,y1) 묶음이어야 한다
      explore.lidar_*         LiDAR 면 지도 수치(면 최소 길이·L 하한·UGV 제외 반경·닫힘 칸 수·정찰 거리·시도 수)가 범위 안이어야 한다
      oak_det.identify_*      식별 단계 수치가 범위 안이어야 하고, identify_fallback 은 oak_det.enable 이 켜져 있어야 의미가 있다(아니면 끈다)
      oak_det.identify_primary  oak_det.enable 이 켜져 있어야 의미가 있다(아니면 끈다). identify_edge_margin_px·identify_crop_frames·identify_fail_limit·
                              identify_noview_s·fit_hfov_deg·fit_vfov_deg·fit_usable 은 범위 안이어야 한다
      oak_det.depth_range     oak_det.enable 이 켜져 있어야 의미가 있다(아니면 끈다). depth_* · iff_* · ugv_width_m 은 범위 안이어야 한다(상대 UGV 거리·피아식별)
      oak_det.ugv_only        oak_det.enable 이 켜져 있어야 의미가 있다(아니면 끈다)
      explore.m1_enemy_weight · m1_cm_weight · m1_cm_defer · m1_enemy_budget_s · uav_census_window_s   범위 안이어야 한다(M1 지정 구역 우선순위·UAV 초기 영상 장부)
      impact.*                IMU 충돌 감지·대응 수치가 범위 안이어야 하고 major_dg > trigger_dg, hold_major_s ≥ hold_s 여야 한다
      pose_guard.*            자세 감시 수치가 범위 안이어야 하고 link_trim 은 link_check 가 켜져 있어야 의미가 있다(아니면 끈다)
      oak.power_gating        oak_det.enable 이 켜져 있어야 의미가 있다(아니면 끈다). wake_dist_m·off_delay_s·start_timeout_s·power_retry_s 는 범위 안이어야 한다
      oak.profile             config/oak/<profile>.yaml 이 있어야 한다(root 를 줬을 때만 본다)
    """
    changed: List[str] = []

    def report(key: str, old: Any, new: Any, why: str) -> None:
        changed.append(key)
        text = f"{key}={raw_label(old, 40)} {why} → {raw_label(new, 40)} 로 바꾼다"
        if vlog is not None:
            vlog.bad("config", key, text)
        else:
            log.warning("[검증] 설정 %s", text)

    def put(section: Any, name: str, key: str, value: Any, why: str, new: Any) -> None:
        report(key, value, new, why)
        try:
            setattr(section, name, new)
        except Exception:
            pass

    def number(v: Any) -> bool:
        return finite(v)

    try:
        cam = getattr(cfg, "camera", None)
        if cam is not None:
            v = getattr(cam, "model", "pinhole")
            if not isinstance(v, str) or v not in ("pinhole", "fisheye"):
                put(cam, "model", "camera.model", v, "은 pinhole|fisheye 가 아니다", "pinhole")
            v = getattr(cam, "usable_hfov_deg", None)
            if v is not None and not (number(v) and 0.0 < float(v) <= 180.0):
                put(cam, "usable_hfov_deg", "camera.usable_hfov_deg", v, "은 (0, 180] 밖이다", None)
        od = getattr(cfg, "oak_det", None)
        if od is not None:
            v = getattr(od, "max_hz", 3.0)
            if not (number(v) and float(v) > 0.0):
                put(od, "max_hz", "oak_det.max_hz", v, "은 양수가 아니다", _default_of(od, "max_hz"))
            v = getattr(od, "vote_window", None)
            if v is not None and not (number(v) and float(v) >= 1.0 and float(v) == int(float(v))):
                put(od, "vote_window", "oak_det.vote_window", v, "은 1 이상의 정수가 아니다", None)
            v = getattr(od, "save_max", 8)
            if not number(v):
                put(od, "save_max", "oak_det.save_max", v, "은 수가 아니다", _default_of(od, "save_max"))
            elif not 0 <= int(float(v)) <= 10 or int(float(v)) != float(v):
                put(od, "save_max", "oak_det.save_max", v, "은 0..10 밖이다", int(min(max(float(v), 0.0), 10.0)))
            def ranged(sec: Any, name: str, key: str, lo: float, hi: float, why: str, integer: bool = False) -> None:
                v = getattr(sec, name, None)
                if v is None:
                    return
                if not (number(v) and lo <= float(v) <= hi) or (integer and float(v) != int(float(v))):
                    put(sec, name, key, v, why, _default_of(sec, name))
            ranged(od, "identify_min_attempts", "oak_det.identify_min_attempts", 0, 10, "은 0..10 의 정수가 아니다", True)
            ranged(od, "identify_max_attempts", "oak_det.identify_max_attempts", 0, 10, "은 0..10 의 정수가 아니다", True)
            ranged(od, "identify_dist_m", "oak_det.identify_dist_m", 0.4, 1.5, "은 0.4..1.5 m 밖이다")
            ranged(od, "identify_max_range_m", "oak_det.identify_max_range_m", 0.5, 3.0, "은 0.5..3.0 m 밖이다")
            ranged(od, "identify_dwell_s", "oak_det.identify_dwell_s", 0.5, 10.0, "은 0.5..10 s 밖이다")
            ranged(od, "identify_min_frames", "oak_det.identify_min_frames", 1, 50, "은 1..50 의 정수가 아니다", True)
            ranged(od, "identify_timeout_s", "oak_det.identify_timeout_s", 1.0, 30.0, "은 1..30 s 밖이다")
            ranged(od, "identify_weight", "oak_det.identify_weight", 0.05, 1.5, "은 0.05..1.5 밖이다")
            ranged(od, "identify_gain", "oak_det.identify_gain", 0.0, 5.0, "은 0..5 밖이다")
            if getattr(od, "identify_fallback", False) and not getattr(od, "enable", False):
                put(od, "identify_fallback", "oak_det.identify_fallback", True, "는 oak_det.enable 이 꺼져 있어 쓸 수 없다", False)
            if getattr(od, "on_demand", False) and not getattr(od, "enable", False):
                put(od, "on_demand", "oak_det.on_demand", True, "는 oak_det.enable 이 꺼져 있어 의미가 없다", False)
            ranged(od, "identify_edge_margin_px", "oak_det.identify_edge_margin_px", 0, 100, "은 0..100 의 정수가 아니다", True)
            ranged(od, "identify_crop_frames", "oak_det.identify_crop_frames", 1, 20, "은 1..20 의 정수가 아니다", True)
            ranged(od, "identify_fail_limit", "oak_det.identify_fail_limit", 1, 10, "은 1..10 의 정수가 아니다", True)
            ranged(od, "identify_noview_s", "oak_det.identify_noview_s", 0.0, 120.0, "은 0..120 s 밖이다")
            ranged(od, "fit_hfov_deg", "oak_det.fit_hfov_deg", 30.0, 120.0, "은 30..120° 밖이다")
            ranged(od, "fit_vfov_deg", "oak_det.fit_vfov_deg", 20.0, 100.0, "은 20..100° 밖이다")
            ranged(od, "fit_usable", "oak_det.fit_usable", 0.5, 1.0, "은 0.5..1.0 밖이다")
            if getattr(od, "identify_primary", False) and not getattr(od, "enable", False):
                put(od, "identify_primary", "oak_det.identify_primary", True, "는 oak_det.enable 이 꺼져 있어 쓸 수 없다", False)
            ranged(od, "depth_max_dt_s", "oak_det.depth_max_dt_s", 0.05, 2.0, "은 0.05..2 s 밖이다")
            ranged(od, "depth_roi_frac", "oak_det.depth_roi_frac", 0.1, 1.0, "은 0.1..1 밖이다")
            ranged(od, "depth_min_pixels", "oak_det.depth_min_pixels", 1, 2000, "은 1..2000 의 정수가 아니다", True)
            ranged(od, "depth_fg_mass", "oak_det.depth_fg_mass", 0.05, 0.9, "은 0.05..0.9 밖이다")
            ranged(od, "depth_bin_m", "oak_det.depth_bin_m", 0.01, 0.3, "은 0.01..0.3 m 밖이다")
            ranged(od, "depth_agree_tol", "oak_det.depth_agree_tol", 0.1, 2.0, "은 0.1..2 밖이다")
            ranged(od, "iff_max_range_m", "oak_det.iff_max_range_m", 0.5, 5.0, "은 0.5..5 m 밖이다")
            ranged(od, "iff_weight_boost", "oak_det.iff_weight_boost", 1.0, 3.0, "은 1..3 밖이다")
            v = getattr(od, "ugv_width_m", (0.08, 0.70))
            okw = isinstance(v, (list, tuple)) and len(v) == 2 and all(number(x) for x in v) and 0.02 <= float(v[0]) < float(v[1]) <= 2.0
            if not okw:
                put(od, "ugv_width_m", "oak_det.ugv_width_m", v, "은 0.02 ≤ 최소 < 최대 ≤ 2 m 의 두 수가 아니다", _default_of(od, "ugv_width_m"))
            for flag in ("depth_range", "ugv_only"):
                if getattr(od, flag, False) and not getattr(od, "enable", False):
                    put(od, flag, f"oak_det.{flag}", True, "는 oak_det.enable 이 꺼져 있어 의미가 없다", False)
        ex = getattr(cfg, "explore", None)
        if ex is not None:
            def exr(name: str, lo: float, hi: float, why: str, integer: bool = False) -> None:
                v = getattr(ex, name, None)
                if v is None:
                    return
                if not (number(v) and lo <= float(v) <= hi) or (integer and float(v) != int(float(v))):
                    put(ex, name, f"explore.{name}", v, why, _default_of(ex, name))
            exr("lidar_face_min_len_m", 0.2, 0.8, "은 0.2..0.8 m 밖이다")
            exr("lidar_face_min_L", 0.8, 6.0, "은 0.8..6 밖이다")
            exr("lidar_ugv_exclude_m", 0.1, 0.8, "은 0.1..0.8 m 밖이다")
            exr("lidar_closure_min_cells", 1, 200, "은 1..200 의 정수가 아니다", True)
            exr("lidar_scout_standoff_m", 0.4, 1.5, "은 0.4..1.5 m 밖이다")
            exr("lidar_open_bonus", 0.0, 3.0, "은 0..3 밖이다")
            exr("lidar_scout_max_attempts", 0, 10, "은 0..10 의 정수가 아니다", True)
            exr("m1_sector_weight", 0.5, 5.0, "은 0.5..5 밖이다")
            exr("m1_enemy_weight", 0.5, 5.0, "은 0.5..5 밖이다")
            exr("m1_cm_weight", 0.5, 5.0, "은 0.5..5 밖이다")
            exr("m1_cm_defer", 0.0, 1.0, "은 0..1 밖이다")
            exr("m1_enemy_budget_s", 0.0, 1800.0, "은 0..1800 s 밖이다")
            exr("uav_census_window_s", 0.5, 60.0, "은 0.5..60 s 밖이다")
        for secname, rules in _GUARD_RULES.items():
            sec = getattr(cfg, secname, None)
            if sec is None:
                continue
            for name, lo, hi, integer in rules:
                v = getattr(sec, name, None)
                if v is None:
                    continue
                if isinstance(v, bool) or not (number(v) and lo <= float(v) <= hi) or (integer and float(v) != int(float(v))):
                    put(sec, name, f"{secname}.{name}", v, f"은 {lo:g}..{hi:g}{' 의 정수' if integer else ''} 밖이다", _default_of(sec, name))
        im = getattr(cfg, "impact", None)
        if im is not None:
            if float(getattr(im, "major_dg", 1.2)) <= float(getattr(im, "trigger_dg", 0.45)):
                put(im, "trigger_dg", "impact.trigger_dg", getattr(im, "trigger_dg", None), "가 major_dg 이상이다", _default_of(im, "trigger_dg"))
                if float(getattr(im, "major_dg", 1.2)) <= float(getattr(im, "trigger_dg", 0.45)):
                    put(im, "major_dg", "impact.major_dg", getattr(im, "major_dg", None), "가 trigger_dg 이하다", _default_of(im, "major_dg"))
            if float(getattr(im, "hold_major_s", 0.8)) < float(getattr(im, "hold_s", 0.4)):
                put(im, "hold_major_s", "impact.hold_major_s", getattr(im, "hold_major_s", None), "가 hold_s 보다 짧다", _default_of(im, "hold_major_s"))
        pg = getattr(cfg, "pose_guard", None)
        if pg is not None:
            if getattr(pg, "link_trim", False) and not getattr(pg, "link_check", True):
                put(pg, "link_trim", "pose_guard.link_trim", True, "는 link_check 가 꺼져 있어 의미가 없다", False)
            if str(getattr(pg, "structures", "fixed")) not in ("fixed", "uav"):
                put(pg, "structures", "pose_guard.structures", getattr(pg, "structures", None), "는 \"fixed\" 또는 \"uav\" 여야 한다", _default_of(pg, "structures"))
            if getattr(pg, "reanchor", False) and not getattr(pg, "link_check", True):
                put(pg, "reanchor", "pose_guard.reanchor", True, "는 link_check 가 꺼져 있어 의미가 없다(기준선을 못 만든다)", False)
            if float(getattr(pg, "reanchor_max_m", 0.6)) > float(getattr(pg, "reanchor_search_xy", 0.5)) * 2.0 + 0.2:
                put(pg, "reanchor_max_m", "pose_guard.reanchor_max_m", getattr(pg, "reanchor_max_m", None),
                    "가 탐색 반경(reanchor_search_xy)의 두 배보다 훨씬 크다 — 탐색 밖의 보정은 나올 수 없다", _default_of(pg, "reanchor_max_m"))
            if float(getattr(pg, "fallback_deg", 5.0)) > float(getattr(pg, "shift_max_deg", 40.0)):
                put(pg, "fallback_deg", "pose_guard.fallback_deg", getattr(pg, "fallback_deg", None), "가 shift_max_deg 보다 크다", _default_of(pg, "fallback_deg"))
        ub = getattr(cfg, "uav_boxes", None)
        if ub is not None:
            def ubr(name: str, lo: float, hi: float, why: str, integer: bool = False) -> None:
                v = getattr(ub, name, None)
                if v is None:
                    return
                if isinstance(v, bool) or not (number(v) and lo <= float(v) <= hi) or (integer and float(v) != int(float(v))):
                    put(ub, name, f"uav_boxes.{name}", v, why, _default_of(ub, name))
            v = getattr(ub, "top_color", "black")
            if not isinstance(v, str) or v not in ("black", "grey", "both"):
                put(ub, "top_color", "uav_boxes.top_color", v, "은 black|grey|both 가 아니다", _default_of(ub, "top_color"))
            v = getattr(ub, "camera", "main")
            if not isinstance(v, str) or v not in ("main", "test1", "test2"):
                put(ub, "camera", "uav_boxes.camera", v, "은 main|test1|test2 가 아니다", _default_of(ub, "camera"))
            v = getattr(ub, "calibration", None)
            if v is not None and not (isinstance(v, str) and v.strip()):
                put(ub, "calibration", "uav_boxes.calibration", v, "은 비어 있지 않은 경로 문자열이어야 한다", None)
            ubr("v_black_max", 1, 200, "은 1..200 밖이다")
            ubr("v_black_rel", 0.1, 0.95, "은 0.1..0.95 밖이다")
            ubr("v_black_min", 0, 100, "은 0..100 밖이다")
            ubr("min_floor_contrast", 1.0, 10.0, "은 1..10 밖이다")
            ubr("grey_s_max", 0, 255, "은 0..255 밖이다")
            ubr("grey_v_min", 0, 255, "은 0..255 밖이다")
            ubr("grey_v_max", 0, 255, "은 0..255 밖이다")
            ubr("ppm", 48, 256, "은 48..256 px/m 의 정수가 아니다", True)
            ubr("fill_min", 0.5, 0.98, "은 0.5..0.98 밖이다")
            ubr("min_conf", 0.0, 1.0, "은 0..1 밖이다")
            ubr("min_top_m", 0.1, 0.4, "은 0.1..0.4 m 밖이다")
            ubr("max_extent_m", 0.4, 6.0, "은 0.4..6 m 밖이다")
            ubr("tile_slack", 0.0, 0.4, "은 0..0.4 m 밖이다")
            ubr("extra_free", 0.0, 1.0, "은 0..1 밖이다")
            ubr("edge_margin_m", 0.0, 1.0, "은 0..1 m 밖이다")
            ubr("wall_band_extra_m", 0.0, 0.5, "은 0..0.5 m 밖이다")
            ubr("min_mean_v", 0, 100, "은 0..100 밖이다")
            ubr("max_mean_v", 150, 255, "은 150..255 밖이다")
            ubr("max_boxes", 1, 200, "은 1..200 의 정수가 아니다", True)
            ubr("ignore_margin_m", 0.0, 0.5, "은 0..0.5 m 밖이다")
            if float(getattr(ub, "grey_v_min", 0)) >= float(getattr(ub, "grey_v_max", 255)):
                put(ub, "grey_v_min", "uav_boxes.grey_v_min", getattr(ub, "grey_v_min", None), "이 grey_v_max 이상이다", _default_of(ub, "grey_v_min"))
                put(ub, "grey_v_max", "uav_boxes.grey_v_max", getattr(ub, "grey_v_max", None), "가 grey_v_min 이하다", _default_of(ub, "grey_v_max"))
            v = getattr(ub, "ignore_rects", ())
            ok = isinstance(v, (list, tuple)) and len(v) % 4 == 0 and all(not isinstance(x, bool) and number(x) for x in v)
            if ok:
                ok = all(float(v[i + 2]) > float(v[i]) and float(v[i + 3]) > float(v[i + 1]) for i in range(0, len(v), 4))
            if not ok:
                put(ub, "ignore_rects", "uav_boxes.ignore_rects", v, "은 (x0,y0,x1,y1) 4개씩 묶음(x1>x0, y1>y0)이어야 한다", ())
            if getattr(ub, "enable", False) and not getattr(ub, "calibration", None):
                changed.append("uav_boxes.calibration")
                msg = "uav_boxes.enable 이 켜졌지만 uav_boxes.calibration 이 비어 있다 — 실기 UAV 검출기는 꺼진 채로 시작한다(--uav-calibration 으로 줄 수 있다)"
                if vlog is not None:
                    vlog.bad("config", "uav_boxes.calibration", msg)
                else:
                    log.warning("[검증] 설정 %s", msg)
        av = getattr(cfg, "avoid", None)
        if av is not None:
            def avr(name: str, lo: float, hi: float, why: str) -> None:
                v = getattr(av, name, None)
                if v is not None and not (number(v) and lo <= float(v) <= hi):
                    put(av, name, f"avoid.{name}", v, why, _default_of(av, name))
            avr("friend_birth_s", 0.0, 30.0, "은 0..30 s 밖이다")
            avr("friend_birth_radius_m", 0.2, 2.0, "은 0.2..2.0 m 밖이다")
            avr("friend_lost_s", 1.0, 60.0, "은 1..60 s 밖이다")
            avr("friend_gate_m", 0.1, 2.0, "은 0.1..2.0 m 밖이다")
            avr("friend_max_speed", 0.1, 2.0, "은 0.1..2.0 m/s 밖이다")
            avr("friend_ambiguity_m", 0.05, 2.0, "은 0.05..2.0 m 밖이다")
        aim = getattr(cfg, "aim", None)
        if aim is not None:
            v = getattr(aim, "gimbal_settle_s", 0.0)
            if not (number(v) and float(v) >= 0.0):
                put(aim, "gimbal_settle_s", "aim.gimbal_settle_s", v, "은 0 이상의 수가 아니다", _default_of(aim, "gimbal_settle_s"))
        mis = getattr(cfg, "mission", None)
        if mis is not None:
            v = getattr(mis, "stage_policy", "enemy_middle")
            if not isinstance(v, str) or v not in ("enemy_middle", "legacy"):
                put(mis, "stage_policy", "mission.stage_policy", v, "은 enemy_middle|legacy 가 아니다", "enemy_middle")
            v = getattr(mis, "stage_standoff_m", 0.30)
            if not (number(v) and 0.05 <= float(v) <= 1.5):
                put(mis, "stage_standoff_m", "mission.stage_standoff_m", v, "은 0.05..1.5 m 밖이다", _default_of(mis, "stage_standoff_m"))
        oak = getattr(cfg, "oak", None)
        if oak is not None:
            def oakr(name: str, lo: float, hi: float, why: str) -> None:
                v = getattr(oak, name, None)
                if v is None:
                    return
                if not (number(v) and lo <= float(v) <= hi):
                    put(oak, name, f"oak.{name}", v, why, _default_of(oak, name))
            oakr("wake_dist_m", 0.5, 6.0, "은 0.5..6 m 밖이다")
            oakr("off_delay_s", 0.0, 60.0, "은 0..60 s 밖이다")
            oakr("start_timeout_s", 1.0, 60.0, "은 1..60 s 밖이다")
            oakr("power_retry_s", 0.5, 30.0, "은 0.5..30 s 밖이다")
            if getattr(oak, "power_gating", False) and not getattr(getattr(cfg, "oak_det", None), "enable", False):
                put(oak, "power_gating", "oak.power_gating", True, "는 oak_det.enable 이 꺼져 있어 의미가 없다", False)
        if oak is not None and root is not None:
            profile = getattr(oak, "profile", "")
            ok = isinstance(profile, str) and (Path(root) / "config" / "oak" / f"{profile}.yaml").is_file()
            if not ok:
                default = _default_of(oak, "profile")
                if default is not None and default != profile and (Path(root) / "config" / "oak" / f"{default}.yaml").is_file():
                    put(oak, "profile", "oak.profile", profile, "의 파라미터 파일(config/oak/<profile>.yaml)이 없다", default)
                else:
                    changed.append("oak.profile")
                    msg = f"oak.profile={raw_label(profile, 40)} 의 파라미터 파일이 없다 — 바꿀 기본 파일도 없다(OAK 드라이버 기동만 건너뛴다)"
                    if vlog is not None:
                        vlog.bad("config", "oak.profile", msg)
                    else:
                        log.warning("[검증] 설정 %s", msg)
    except Exception as e:                                  # 점검 실패가 시작을 막으면 안 된다
        try:
            log.warning("[검증] 설정 점검 중 오류 %s: %s", type(e).__name__, e)
        except Exception:
            pass
    return changed
