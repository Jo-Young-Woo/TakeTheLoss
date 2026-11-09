"""LED 읽기. 규정의 이벤트 요소 중 영상으로 읽는 것들(PDF p14·p25, 멘토링 11/4).

  find_signal_led   지정표적 상단 시그널 LED — 유효 레이저 피격 시 **빨간색 점등**. M2 성공의 시각 확인.
  read_ugv_leds     상대 UGV 뒤쪽 LED — 색으로 유효피격 수(파 정상/초 1/노 2/주 3/빨 4), 8칸 LED 로 남은 탄. M6 의 생존·잔탄 증거.
                    (멘토링: 생존 점수와 잔여 탄약은 대시보드가 안 주지만 시각적으로 일부 알 수 있다)
  classify_tower    3색 타워 LED(경기 준비·진행·완료)의 점등 색. 서버 STOP 유실 때의 보조.

**실물 사진 없이 규정 문구와 합성 이미지로 만들었다.** LED 의 정확한 위치·크기·색 임계는 현장 사진으로 교정해야 한다. 특히
UGV 뒤쪽 LED 배치(한 줄 8칸인지, 색이 같은 줄에 있는지)와 타워의 색-상태 대응은 규정에 구체적으로 없어 가정이다.
반환은 모두 '못 읽으면 None/False' — 호출자는 증거가 없는 것으로 다룬다(틀린 증거보다 낫다).
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

HIT_COLORS = ("blue", "green", "yellow", "orange", "red")        # 피격 0~4 (파·초·노·주·빨)
LIT_S, LIT_V = 110, 150                                          # 켜진 LED: 채도·밝기 하한


def hue_name(h: float) -> str:
    """OpenCV HSV 색상(0~179) → 색 이름."""
    if h < 6 or h >= 170:
        return "red"
    if h < 19:
        return "orange"
    if h < 36:
        return "yellow"
    if h < 88:
        return "green"
    if h < 132:
        return "blue"
    return "purple"


def _crop(image: np.ndarray, box: Sequence[float], pad: float = 0.0):
    H, W = image.shape[:2]
    w, h = box[2] - box[0], box[3] - box[1]
    x1, y1 = max(int(math.floor(box[0] - pad * w)), 0), max(int(math.floor(box[1] - pad * h)), 0)
    x2, y2 = min(int(math.ceil(box[2] + pad * w)), W), min(int(math.ceil(box[3] + pad * h)), H)
    if x2 - x1 < 8 or y2 - y1 < 8:
        return None, 0, 0
    return image[y1:y2, x1:x2], x1, y1


def _lit_blobs(crop: np.ndarray, min_area: float, max_area: float, min_solidity: float = 0.6):
    import cv2
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 1] >= LIT_S) & (hsv[..., 2] >= LIT_V)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cs:
        a = cv2.contourArea(c)
        if a < min_area or a > max_area:
            continue
        hull = cv2.contourArea(cv2.convexHull(c))
        if a / (hull + 1e-9) < min_solidity:
            continue
        m = cv2.moments(c)
        if m["m00"] <= 0:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        cm = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(cm, [c], -1, 255, -1)
        h = float(np.median(hsv[..., 0][cm > 0]))
        out.append((cx, cy, math.sqrt(a / math.pi), h))
    return out


# ====================================================================== E5 지정표적 시그널 LED
def find_signal_led(image: Optional[np.ndarray], box: Sequence[float]) -> bool:
    """표적 bbox 의 위쪽(위로 bbox 높이의 15 % 까지 확장)에서 **빨간색으로 켜진** 작고 둥근 점을 찾는다.
    한 프레임 판정이라 오검출이 있을 수 있다 — Agent/ObjectMap 이 연속 프레임으로 확인한다(event.led_confirm_frames)."""
    if image is None:
        return False
    try:
        __import__("cv2")  # availability probe; image helpers import it themselves
    except ImportError:
        return False
    h = box[3] - box[1]
    top = (box[0], box[1] - 0.15 * h, box[2], box[1] + 0.40 * h)
    crop, _, _ = _crop(image, top)
    if crop is None:
        return False
    area = crop.shape[0] * crop.shape[1]
    for (_, _, r, hue) in _lit_blobs(crop, math.pi * 1.5 ** 2, 0.08 * area, 0.75):
        if hue_name(hue) == "red":
            return True
    return False


# ====================================================================== E8 UGV 후면 LED
def read_ugv_leds(image: Optional[np.ndarray], box: Sequence[float]) -> Optional[Dict[str, float]]:
    """상대 UGV bbox 안의 켜진 LED 칸들을 모아 {"hits": 0~4, "ammo": 0~8, "conf": 0~1} 로. 못 읽으면 None.
    가정: 같은 높이에 가로로 늘어선 켜진 칸들이 탄창 LED(켜진 칸 수 = 남은 탄)이고, 그 색이 유효피격 수를 나타낸다."""
    if image is None:
        return None
    try:
        __import__("cv2")  # availability probe; image helpers import it themselves
    except ImportError:
        return None
    crop, _, _ = _crop(image, box)
    if crop is None:
        return None
    H, W = crop.shape[:2]
    blobs = _lit_blobs(crop, 3.0, 0.01 * H * W, 0.6)
    blobs = [b for b in blobs if hue_name(b[3]) in HIT_COLORS]
    if len(blobs) < 1:
        return None
    # 같은 줄(높이가 비슷한 것) 중 가장 많은 묶음을 탄창 줄로 본다
    best: List = []
    for b in blobs:
        row = [c for c in blobs if abs(c[1] - b[1]) <= 0.06 * H + 1.5 * b[2]]
        if len(row) > len(best):
            best = row
    if len(best) > 8:
        best = sorted(best, key=lambda c: -c[2])[:8]
    names = [hue_name(c[3]) for c in best]
    top = max(set(names), key=names.count)
    consistency = names.count(top) / len(names)
    size_ok = float(np.std([c[2] for c in best]) / (np.mean([c[2] for c in best]) + 1e-9)) < 0.5
    conf = 0.5 * consistency + 0.25 * (1.0 if len(best) >= 2 else 0.5) + 0.25 * (1.0 if size_ok else 0.4)
    return {"hits": float(HIT_COLORS.index(top)), "ammo": float(len(best)), "conf": float(conf)}


# ====================================================================== E2 3색 타워 LED
def classify_tower(image: Optional[np.ndarray], box: Sequence[float]) -> Optional[Tuple[str, float]]:
    """타워 bbox 안에서 가장 밝고 채도 높은 켜진 점의 색 이름과 신뢰도. 켜진 점이 없으면 None."""
    if image is None:
        return None
    try:
        __import__("cv2")  # availability probe; image helpers import it themselves
    except ImportError:
        return None
    crop, _, _ = _crop(image, box)
    if crop is None:
        return None
    H, W = crop.shape[:2]
    blobs = _lit_blobs(crop, 4.0, 0.5 * H * W, 0.5)
    if not blobs:
        return None
    big = max(blobs, key=lambda b: b[2])
    names = [hue_name(b[3]) for b in blobs]
    name = hue_name(big[3])
    return name, float(0.5 + 0.5 * names.count(name) / len(names))
