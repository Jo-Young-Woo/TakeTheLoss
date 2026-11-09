"""검출기 인터페이스와 구현.

  SimDetector   : 시뮬레이터가 만든 검출을 그대로 전달(임무 로직 시험용)
  YoloDetector  : ultralytics YOLO (RTX 5070 PC 학습 → .pt 또는 .engine/.onnx 배포)
  AsyncDetector : 느린 검출기를 별도 스레드로 돌려 제어 루프가 기다리지 않게 하는 래퍼

클래스 이름 규약(학습 시 data.yaml 의 names 와 일치시킬 것):
  객체 6종  : AMMO_BOX DRUM MORTAR TRUCK TANK MISSILE
  지정표적 3: GUARD_POST RADAR_FACILITY COMM_FACILITY
  거점 표지 3: COMMAND_POST MAINT_DEPOT SUPPLY_BASE
  로봇      : UGV  (팀은 안테나 색으로 아래 team_from_antenna 가 판정)
"""
from __future__ import annotations

import queue
import threading
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..types import Detection, Frame


class Detector(ABC):
    asynchronous = False          # True면 detect_stamped 가 가벼워서 매 틱 호출해도 된다(AsyncDetector)

    @abstractmethod
    def detect(self, frame: Frame) -> List[Detection]:
        ...

    def detect_stamped(self, frame: Frame) -> Tuple[List[Detection], float]:
        """(검출, 그 검출이 나온 프레임의 stamp). 동기 검출기는 입력 프레임의 stamp 를 그대로 돌려준다.
        비동기 검출기는 이전 프레임의 결과를 줄 수 있으므로, 호출자는 이 stamp 의 프레임·자세와 결합해야 한다(R03)."""
        return self.detect(frame), frame.stamp

    def health(self) -> Tuple[bool, str]:
        """(정상 여부, 문제 설명). 검출기가 조용히 죽어도 Agent 가 알 수 있게 한다. 기본은 항상 정상."""
        return True, ""


class SimDetector(Detector):
    def detect(self, frame: Frame) -> List[Detection]:
        out = []
        for d in frame.meta.get("sim_detections", []):
            ex = {k: v for k, v in d.extra.items() if not k.startswith("_")}   # 정답(_gt)은 숨김
            out.append(Detection(d.cls, d.conf, d.box, d.stamp, d.team, ex))
        return out


def team_from_antenna(image: np.ndarray, box, min_frac: float = 0.01) -> Optional[str]:
    """UGV bbox 상단 40%에서 빨강/파랑 픽셀 비율로 팀 판정(안테나 구조물 색). 불확실하면 None.
    실제 안테나 색 범위는 현장 사진으로 HSV를 교정할 것."""
    import cv2  # 지연 import — 시뮬/서버 쪽에서는 필요 없음

    x1, y1, x2, y2 = [int(round(v)) for v in box]
    h = max(y2 - y1, 1)
    crop = image[max(y1, 0):max(y1, 0) + int(0.4 * h), max(x1, 0):max(x2, 1)]
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    red = (((hsv[..., 0] < 8) | (hsv[..., 0] > 172)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 70)).mean()
    blue = ((hsv[..., 0] > 100) & (hsv[..., 0] < 130) & (hsv[..., 1] > 120) & (hsv[..., 2] > 50)).mean()
    if max(red, blue) < min_frac or abs(red - blue) < 0.5 * max(red, blue):
        return None
    return "RED" if red > blue else "BLUE"


class YoloDetector(Detector):
    """ultralytics YOLO 래퍼. weights: .pt / .engine(TensorRT) / .onnx."""

    SENSOR_CLASSES = ("GUARD_POST", "RADAR_FACILITY", "COMM_FACILITY", "UGV")     # 수광센서를 겨냥해야 하는 클래스
    TARGET_CLASSES = ("GUARD_POST", "RADAR_FACILITY", "COMM_FACILITY")            # 시그널 LED 가 달린 지정표적

    def __init__(self, weights: str, conf: float = 0.35, imgsz: int = 640, device: Optional[str] = None,
                 names_map: Optional[Dict[str, str]] = None, locate_sensors: bool = True) -> None:
        from ultralytics import YOLO  # type: ignore

        self.model = YOLO(weights)
        self.conf, self.imgsz, self.device = conf, imgsz, device
        self.names_map = names_map or {}
        self.locate_sensors = locate_sensors

    def detect(self, frame: Frame) -> List[Detection]:
        if frame.image is None:
            return []
        res = self.model.predict(frame.image, conf=self.conf, imgsz=self.imgsz, device=self.device,
                                 verbose=False)[0]
        out: List[Detection] = []
        for b in res.boxes:
            name = res.names[int(b.cls)]
            name = self.names_map.get(name, name)
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            d = Detection(name, float(b.conf), (x1, y1, x2, y2), frame.stamp)
            if name == "UGV":
                d.team = team_from_antenna(frame.image, d.box)
            out.append(d)
        # 이벤트 LED(규정 이벤트 요소): 지정표적 시그널 LED 점등 / 상대 UGV 뒤쪽 LED(피격색·탄창) / 3색 타워 LED 색
        if self.locate_sensors:
            from .led import classify_tower, find_signal_led, read_ugv_leds
            for d in out:
                if d.cls in self.TARGET_CLASSES:
                    d.extra["led_on"] = find_signal_led(frame.image, d.box)
                elif d.cls == "UGV":
                    led = read_ugv_leds(frame.image, d.box)
                    if led:
                        d.extra["led"] = led
                elif d.cls == "TOWER_LED":
                    c = classify_tower(frame.image, d.box)
                    if c:
                        d.extra["tower_color"], d.extra["tower_conf"] = c
        # 수광센서: 학습한 RECEIVER 박스가 있으면 그것을, 없으면 영상에서 흰 영역 안의 원을 찾아 붙인다 (조준은 센서 중심을 겨냥)
        from .receiver import attach_learned_receivers, find_receivers
        out = attach_learned_receivers(out)
        if self.locate_sensors:
            for d in out:
                if d.cls in self.SENSOR_CLASSES and not d.extra.get("sensors"):
                    s = find_receivers(frame.image, d.box, max_n=2 if d.cls == "UGV" else 1,
                                       kind="ugv" if d.cls == "UGV" else "target")
                    if s:
                        d.extra["sensors"] = s
        return out


class AsyncDetector(Detector):
    """submit(frame)로 최신 프레임만 넣고, detect()는 가장 최근 완료된 결과를 즉시 돌려준다.
    큐 길이 1: 밀리면 옛 프레임을 버린다(실시간성 > 완전성)."""

    asynchronous = True

    def __init__(self, inner: Detector) -> None:
        self.inner = inner
        self._q: "queue.Queue[Frame]" = queue.Queue(maxsize=1)
        self._lock = threading.Lock()
        self._latest: List[Detection] = []
        self._latest_stamp = -1.0
        self._submitted_stamp = -1.0         # 마지막으로 큐에 넣은 프레임(같은 프레임을 두 번 돌리지 않는다)
        self.error_count = 0                 # 누적 예외 수
        self.consecutive_errors = 0          # 마지막 성공 이후 연속 예외 수(0 이면 정상)
        self.last_error: Optional[str] = None
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                fr = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                dets = self.inner.detect(fr)
            except Exception as e:  # 검출기 예외가 제어 루프를 죽이면 안 된다
                # 예외를 '빈 검출'로 바꿔 새 stamp 를 붙이면 모델이 죽어도 신선한 정상 결과처럼 보인다. 결과를 갱신하지 않아
                # 오래된 채로 두면 Agent 의 영상 신선도 감시(vision_stale_s)가 발사를 막고, health() 가 이유를 알려준다.
                with self._lock:
                    self.error_count += 1
                    self.consecutive_errors += 1
                    self.last_error = f"{type(e).__name__}: {e}"
                continue
            with self._lock:
                self._latest, self._latest_stamp = dets, fr.stamp
                self.consecutive_errors = 0

    def health(self) -> Tuple[bool, str]:
        with self._lock:
            return self.consecutive_errors == 0, (self.last_error or "") if self.consecutive_errors else ""

    def detect_stamped(self, frame: Frame) -> Tuple[List[Detection], float]:
        """가장 최근 '완료된' 결과와 그 프레임 stamp. 새 프레임이면 큐에 넣는다(밀린 옛 프레임은 버린다)."""
        if frame.stamp != self._submitted_stamp:
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(frame)
                self._submitted_stamp = frame.stamp
            except queue.Full:
                pass
        with self._lock:
            return list(self._latest), self._latest_stamp

    def detect(self, frame: Frame) -> List[Detection]:
        return self.detect_stamped(frame)[0]

    def close(self) -> None:
        self._stop.set()
        # Native OpenCV inference must leave its worker before interpreter teardown.
        # Merely setting the daemon's flag can abort the process after reports save.
        if threading.current_thread() is not self._th:
            self._th.join(timeout=15.0)
            if self._th.is_alive():
                raise RuntimeError("Detector did not finish its in-flight inference during shutdown")
