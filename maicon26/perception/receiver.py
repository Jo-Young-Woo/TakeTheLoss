"""수광센서 찾기. 표적 시설의 흰 원판, 상대 UGV 의 좌/우 흰 돔을 영상에서 직접 찾는다.

검출기(YOLO)가 주는 건 bbox 뿐이라, 센서가 bbox 안 어디인지는 이 모듈이 찾는다.
모양은 사전 PDF 사진으로 정했다(실물이 아니라 예시 이미지다):
  kind="target"  지정표적 면(p12): 사진 윗부분 **중앙(가로 50 %, 세로 약 16 %)에 순백색 원판**(면 폭의 약 15 %).
                 순백색(밝기·채도 임계) 덩어리 중 원형도·채움비가 높고 안이 균일하며 둘레와 대비가 있는 것을 고른다
                 (흰 구름은 가장자리가 흐리고 불규칙해서 걸러진다). 위치 사전분포(상단 중앙)를 점수에 더한다.
  kind="ugv"     상대 UGV(p21): 검은 차체 위 **좌/우 앞 모서리의 하얀 반투명 돔 두 개**. 흰 덩어리 중 크기·솔리디티가
                 맞는 것을 모아 높이가 비슷하고 좌우로 떨어진 한 쌍을 고른다. 정면의 작은 흰 램프(작다)와 은색 명판
                 (가로로 길다)은 크기·종횡비로 거른다.
  kind="auto"    위 둘을 모를 때: ROI 를 Otsu 이진화해 양 극성(어두운 원/밝은 원)의 원형 덩어리를 찾는 일반 방법.
target 이 아무것도 못 찾으면 auto 로 한 번 더 시도한다.
반환: [(u, v, r_px, conf), ...] (영상 픽셀 좌표, 좌→우). 못 찾으면 [].

한계: 실제 대회 사진·조명으로 확인하지 못했다. 임계값(DISC_*, DOME_*, MIN_*)은 현장 사진으로 교정할 것.
학습된 `RECEIVER` 클래스가 있으면 attach_learned_receivers 가 그 박스를 우선 쓴다(perception/detector.py).
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

WHITE_V = 170          # 흰색: 밝기 이상
WHITE_S = 70           # 흰색: 채도 이하
MIN_CIRC = 0.72        # 원형도 4πA/P² 하한
MIN_FILL = 0.70        # 외접원 대비 면적비 하한
MIN_R_PX = 2.0         # 최소 반경(너무 작으면 잡음)

Sensor = Tuple[float, float, float, float]    # (u, v, r_px, conf)


def _white_roi(crop: np.ndarray) -> Tuple[int, int, int, int]:
    import cv2
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    m = ((hsv[..., 2] >= WHITE_V) & (hsv[..., 1] <= WHITE_S)).astype(np.uint8) * 255
    k = max(3, int(min(crop.shape[:2]) * 0.06) | 1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(m)
    best, area = None, 0.08 * crop.shape[0] * crop.shape[1]
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= area:
            area, best = stats[i, cv2.CC_STAT_AREA], i
    if best is None:
        return 0, 0, crop.shape[1], crop.shape[0]
    x, y, w, h = (int(stats[best, c]) for c in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    pad = int(0.04 * max(w, h)) + 1
    return max(x - pad, 0), max(y - pad, 0), min(x + w + pad, crop.shape[1]), min(y + h + pad, crop.shape[0])


def _circles(gray: np.ndarray) -> List[Sensor]:
    """gray ROI 안의 원형 덩어리(양 극성). 좌표는 ROI 기준."""
    import cv2
    g = cv2.GaussianBlur(gray, (3, 3), 0)
    _, th = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    H, W = gray.shape
    out: List[Sensor] = []
    for mask in (th, 255 - th):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        # RETR_LIST: 흰 판 안의 어두운 원은 '구멍'(안쪽 윤곽)이라 바깥 윤곽만 보면 놓친다
        cs, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for c in cs:
            a = cv2.contourArea(c)
            p = cv2.arcLength(c, True)
            if a < math.pi * MIN_R_PX ** 2 or a > 0.5 * H * W or p <= 0:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if x <= 0 or y <= 0 or x + w >= W or y + h >= H:        # 경계에 닿는 건 배경/판 자체지 센서가 아니다
                continue
            circ = 4 * math.pi * a / (p * p)
            (cx, cy), r = cv2.minEnclosingCircle(c)
            fill = a / (math.pi * r * r + 1e-9)
            ar = w / max(h, 1)
            if circ < MIN_CIRC or fill < MIN_FILL or not 0.7 <= ar <= 1.4 or r < MIN_R_PX:
                continue
            ring = cv2.dilate(mask, np.ones((5, 5), np.uint8)) - mask
            inside = float(gray[mask > 0].mean()) if (mask > 0).any() else 0.0
            around = float(gray[(ring > 0) & (mask == 0)].mean()) if ((ring > 0) & (mask == 0)).any() else inside
            contrast = min(abs(inside - around) / 80.0, 1.0)
            conf = 0.45 * min((circ - MIN_CIRC) / (1.0 - MIN_CIRC) + 0.3, 1.0) + 0.25 * fill + 0.30 * contrast
            out.append((cx + 0.0, cy + 0.0, float(r), float(conf)))
    return out


DISC_V, DISC_S = 235, 40         # 표적 원판: 순백색(밝기 이상, 채도 이하)
DISC_MIN_CIRC, DISC_MIN_FILL, DISC_MAX_STD, DISC_MIN_CONTRAST = 0.78, 0.75, 14.0, 12.0
DOME_V, DOME_S = 185, 70         # UGV 돔: 음영이 있는 흰색
DOME_MIN_SOLIDITY, DOME_MAX_FRAC, DOME_MIN_R_FRAC = 0.78, 0.10, 0.045


def _white_disc(crop: np.ndarray) -> List[Sensor]:
    """지정표적 면: 순백색 원판 후보(crop 좌표). 상단 중앙 사전분포를 점수에 더한다."""
    import cv2
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    mask = ((hsv[..., 2] >= DISC_V) & (hsv[..., 1] <= DISC_S)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    H, W = gray.shape
    out: List[Sensor] = []
    for c in cs:
        a, p = cv2.contourArea(c), cv2.arcLength(c, True)
        if a < math.pi * MIN_R_PX ** 2 or a > 0.12 * H * W or p <= 0:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if x <= 0 or y <= 0 or x + w >= W or y + h >= H:
            continue
        circ = 4 * math.pi * a / (p * p)
        (cx, cy), r = cv2.minEnclosingCircle(c)
        fill = a / (math.pi * r * r + 1e-9)
        if circ < DISC_MIN_CIRC or fill < DISC_MIN_FILL or not 0.75 <= w / max(h, 1) <= 1.33:
            continue
        m = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(m, [c], -1, 255, -1)
        ring = cv2.dilate(m, np.ones((7, 7), np.uint8)) - m
        inside = gray[m > 0]
        around = gray[ring > 0]
        contrast = float(inside.mean() - around.mean()) if around.size else 0.0
        if float(inside.std()) > DISC_MAX_STD or contrast < DISC_MIN_CONTRAST:      # 구름: 안이 얼룩지고 둘레와 구분이 약하다
            continue
        pos = math.exp(-(((cx / W) - 0.5) / 0.3) ** 2) * (1.0 if cy / H < 0.45 else 0.6)
        conf = 0.30 * min((circ - DISC_MIN_CIRC) / (1 - DISC_MIN_CIRC) + 0.3, 1.0) + 0.20 * fill \
            + 0.25 * min(contrast / 60.0, 1.0) + 0.25 * pos
        out.append((cx, cy, float(r), float(conf)))
    return out


def _white_domes(crop: np.ndarray) -> List[Sensor]:
    """상대 UGV: 하얀 돔 후보(crop 좌표) 중 높이가 비슷하고 좌우로 떨어진 한 쌍(없으면 가장 큰 하나)."""
    import cv2
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 2] >= DOME_V) & (hsv[..., 1] <= DOME_S)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    H, W = mask.shape
    blobs: List[Sensor] = []
    r_min = max(2.5, DOME_MIN_R_FRAC * W)             # 돔은 bbox 폭의 약 8 %, 정면 램프는 약 2 % — 크기 하한을 bbox 에 비례시켜 램프를 거른다
    for c in cs:
        a = cv2.contourArea(c)
        if a < math.pi * r_min ** 2 or a > DOME_MAX_FRAC * H * W:
            continue
        x, y, w, h = cv2.boundingRect(c)
        hull = cv2.contourArea(cv2.convexHull(c))
        sol = a / (hull + 1e-9)
        if sol < DOME_MIN_SOLIDITY or not 0.45 <= w / max(h, 1) <= 2.2:          # 명판(가로로 길다)·불규칙한 반사 제외
            continue
        m = cv2.moments(c)
        if m["m00"] <= 0:
            continue
        blobs.append((m["m10"] / m["m00"], m["m01"] / m["m00"], math.sqrt(a / math.pi), float(min(0.5 + 0.5 * sol, 1.0))))
    if not blobs:
        return []
    best, score = None, -1e9
    for i in range(len(blobs)):
        for j in range(i + 1, len(blobs)):
            a, b = blobs[i], blobs[j]
            if abs(a[1] - b[1]) > 0.25 * H or abs(a[0] - b[0]) < 0.20 * W:
                continue
            sc = a[2] ** 2 + b[2] ** 2 - 0.5 * abs(a[2] ** 2 - b[2] ** 2) - 20.0 * abs(a[1] - b[1]) / H * (a[2] ** 2 + b[2] ** 2) / 2
            if sc > score:
                best, score = (a, b), sc
    if best is not None:
        return list(best)
    return [max(blobs, key=lambda s: s[2])]


def find_receivers(image: Optional[np.ndarray], box: Sequence[float], max_n: int = 1, kind: str = "auto") -> List[Sensor]:
    """bbox 안의 수광센서 후보 최대 max_n 개(좌→우). kind: 'target'(흰 원판) | 'ugv'(흰 돔 한 쌍) | 'auto'(일반 원형 덩어리)."""
    if image is None:
        return []
    try:
        import cv2  # noqa: F401
    except ImportError:
        return []
    H, W = image.shape[:2]
    x1, y1 = max(int(math.floor(box[0])), 0), max(int(math.floor(box[1])), 0)
    x2, y2 = min(int(math.ceil(box[2])), W), min(int(math.ceil(box[3])), H)
    if x2 - x1 < 12 or y2 - y1 < 12:
        return []
    crop = image[y1:y2, x1:x2]
    if kind in ("target", "ugv"):
        res = _white_disc(crop) if kind == "target" else _white_domes(crop)
        if kind == "target":
            res = sorted(res, key=lambda s: -s[3])[:max_n]
        if res:
            return sorted([(u + x1, v + y1, r, c) for (u, v, r, c) in res][:max_n], key=lambda s: s[0])
    rx1, ry1, rx2, ry2 = _white_roi(crop)
    gray = cv2.cvtColor(crop[ry1:ry2, rx1:rx2], cv2.COLOR_BGR2GRAY)
    cands = _circles(gray)
    cands = [(u + x1 + rx1, v + y1 + ry1, r, c) for (u, v, r, c) in cands]
    # 같은 자리에서 두 극성이 겹쳐 잡힌 중복 제거
    cands.sort(key=lambda s: -s[3])
    uniq: List[Sensor] = []
    for s in cands:
        if all(math.hypot(s[0] - t[0], s[1] - t[1]) > 0.6 * max(s[2], t[2]) for t in uniq):
            uniq.append(s)
    if max_n >= 2 and len(uniq) > 2:
        h = max(y2 - y1, 1)
        best, score = None, -1e9
        for i in range(len(uniq)):
            for j in range(i + 1, len(uniq)):
                a, b = uniq[i], uniq[j]
                sc = a[3] + b[3] - abs(a[1] - b[1]) / h - abs(a[2] - b[2]) / max(a[2], b[2])
                if sc > score:
                    best, score = (a, b), sc
        uniq = list(best) if best else uniq[:2]
    return sorted(uniq[:max_n], key=lambda s: s[0])


def attach_learned_receivers(dets: list, class_name: str = "RECEIVER") -> list:
    """YOLO 가 `RECEIVER` 박스를 따로 학습했다면, 그 박스 중심을 포함하는 부모 검출의 extra['sensors'] 로 붙이고 목록에서 뺀다."""
    parents = [d for d in dets if d.cls != class_name]
    for d in [d for d in dets if d.cls == class_name]:
        cx, cy = d.cx, d.cy
        cand = [p for p in parents if p.box[0] <= cx <= p.box[2] and p.box[1] <= cy <= p.box[3]]
        if not cand:
            continue
        p = min(cand, key=lambda q: (q.box[2] - q.box[0]) * (q.box[3] - q.box[1]))
        p.extra.setdefault("sensors", []).append((cx, cy, 0.25 * (d.w + d.h), d.conf))
    for p in parents:
        if "sensors" in p.extra:
            p.extra["sensors"].sort(key=lambda s: s[0])
    return parents
