"""카메라 내부 파라미터 보정 (체커보드). OpenCV 필요(`pip install opencv-python`).

촬영 요령(160° 광각): 보드(예 9x6, 칸 25 mm)를 화면 가장자리·모서리까지 포함해 다양한 거리/기울기로 25~40장.
같은 해상도·같은 설정(실제 임무에서 쓸 스트림)으로 찍을 것. 결과는 profile.sensors.camera.K/D 에 들어가고
ros2/config/camera_info.yaml 로 내보낸다.
"""
from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np


def board_points(cols: int, rows: int, square: float) -> np.ndarray:
    pts = np.zeros((rows * cols, 3), dtype=np.float64)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return pts


def detect_corners(image_paths: Sequence[str], board: Tuple[int, int] = (9, 6), square: float = 0.025,
                   log=print) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    import cv2  # type: ignore
    objp = board_points(board[0], board[1], square)
    objs, imgs, size = [], [], None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
    for p in image_paths:
        im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if im is None:
            continue
        size = (im.shape[1], im.shape[0])
        ok, c = cv2.findChessboardCorners(im, board, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            log(f"[camera] 코너 검출 실패: {os.path.basename(p)}")
            continue
        c = cv2.cornerSubPix(im, c, (7, 7), (-1, -1), crit)
        objs.append(objp.copy())
        imgs.append(c.reshape(-1, 2).astype(np.float64))
    if size is None:
        raise ValueError("읽을 수 있는 이미지가 없다")
    return objs, imgs, size


def calibrate(objs: List[np.ndarray], imgs: List[np.ndarray], size: Tuple[int, int], model: str = "fisheye") -> Dict[str, Any]:
    """model: 'fisheye'(등거리, D 4개) | 'plumb_bob'(D 5개). 반환: K(9), D, rms(px), per_view_rms."""
    import cv2  # type: ignore
    if len(objs) < 8:
        raise ValueError(f"유효한 보드 이미지가 {len(objs)}장뿐이다 — 최소 8장(권장 25장 이상)")
    if model == "fisheye":
        if cv2.__version__.startswith("5."):
            import warnings
            warnings.warn("OpenCV 5.0 의 어안 보정(cv2.fisheye.calibrate)이 수렴하지 않는 것을 확인했다 — OpenCV 4.x(ROS Humble/Jetson 기본)에서 실행할 것")
        O = [o.reshape(1, -1, 3) for o in objs]                    # (1,N,3): 4.x/5.x 모두 받는 형태
        I = [i.reshape(1, -1, 2) for i in imgs]
        K = np.array([[size[0] / 2.0, 0, size[0] / 2.0], [0, size[0] / 2.0, size[1] / 2.0], [0, 0, 1]], dtype=np.float64)
        D = np.zeros((4, 1), dtype=np.float64)
        # 상수 위치가 OpenCV 4.x(cv2.fisheye.*)와 5.x 에서 다르다 → 값(2, 8)으로 대체
        flags = getattr(cv2.fisheye, "CALIB_RECOMPUTE_EXTRINSIC", 2) + getattr(cv2.fisheye, "CALIB_FIX_SKEW", 8)
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-8)
        rms, K, D, rv, tv = cv2.fisheye.calibrate(O, I, size, K, D, None, None, flags, crit)
    else:
        rms, K, D, rv, tv = cv2.calibrateCamera([o.astype(np.float32) for o in objs], [i.astype(np.float32) for i in imgs], size, None, None)
    per = []
    for o, i, r, t in zip(objs, imgs, rv, tv):
        if model == "fisheye":
            pr, _ = cv2.fisheye.projectPoints(o.reshape(1, -1, 3), r, t, K, D)
        else:
            pr, _ = cv2.projectPoints(o.astype(np.float32), r, t, K, D)
        per.append(float(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - i) ** 2, axis=1)))))
    return {"model": model, "K": [float(x) for x in np.asarray(K).reshape(-1)], "D": [float(x) for x in np.asarray(D).reshape(-1)],
            "rms_px": float(rms), "per_view_rms_px": per, "size": list(size), "n_views": len(objs)}


def apply_to_profile(profile, res: Dict[str, Any], note: str = "") -> List[str]:
    """보정 결과를 프로파일에 넣는다. 화각(hfov_deg)은 plumb_bob 일 때만 K 로 갱신 — 어안은 보정(언디스토션) 후 쓰는 유효 화각을 따로 정한다."""
    lines = []
    profile.set("sensors.camera.K", res["K"], "measured", note)
    profile.set("sensors.camera.D", res["D"], "measured", note)
    profile.set("sensors.camera.model", "fisheye" if res["model"] == "fisheye" else "plumb_bob", "measured", note)
    profile.set("sensors.camera.width", int(res["size"][0]), "measured", note)
    profile.set("sensors.camera.height", int(res["size"][1]), "measured", note)
    profile.set("sensors.camera.calibrated", True, "measured", note)
    lines.append(f"- 카메라 {res['model']}: fx={res['K'][0]:.1f}, fy={res['K'][4]:.1f}, cx={res['K'][2]:.1f}, cy={res['K'][5]:.1f}, "
                 f"RMS {res['rms_px']:.3f}px ({res['n_views']}장)")
    if res["model"] != "fisheye":
        hfov = math.degrees(2 * math.atan(res["size"][0] / 2 / res["K"][0]))
        profile.set("sensors.camera.hfov_deg", hfov, "measured", note)
        lines.append(f"- hfov_deg = {hfov:.1f}")
    else:
        lines.append(f"- 어안 등거리 모델: 센서 전체 화각 ≈ {math.degrees(res['size'][0] / res['K'][0]):.0f}° (언디스토션 후 유효 화각은 별도로 `sensors.camera.hfov_deg` 에)")
    return lines
