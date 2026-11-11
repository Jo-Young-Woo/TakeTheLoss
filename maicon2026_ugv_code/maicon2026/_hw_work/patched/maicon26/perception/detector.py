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
import time
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
        # 입력 채널 순서: frame.image 는 BGR (H,W,3) uint8 이고(대회 API /image_raw 640×480 BGR, hal/ros_mil.image_to_bgr),
        # ultralytics YOLO 의 predict 는 numpy 배열을 BGR 로 받는다. 그래서 여기서는 바꾸지 않고 그대로 넘긴다.
        # 하드웨어 PDF p74 의 'BGR→RGB 변환' 팁은 PIL/OpenCV 이미지를 직접 다루는 경우의 이야기라 우리 코드에는 맞지 않는다 — RGB 변환을 넣지 말 것.
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
            self._process(fr)

    def _process(self, fr: Frame) -> None:
        """큐에서 꺼낸 팬틸트 프레임 하나를 검출하고 결과를 갱신한다."""
        try:
            dets = self.inner.detect(fr)
        except Exception as e:  # 검출기 예외가 제어 루프를 죽이면 안 된다
            # 예외를 '빈 검출'로 바꿔 새 stamp 를 붙이면 모델이 죽어도 신선한 정상 결과처럼 보인다. 결과를 갱신하지 않아
            # 오래된 채로 두면 Agent 의 영상 신선도 감시(vision_stale_s)가 발사를 막고, health() 가 이유를 알려준다.
            with self._lock:
                self.error_count += 1
                self.consecutive_errors += 1
                self.last_error = f"{type(e).__name__}: {e}"
            return
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


class SharedAsyncDetector(AsyncDetector):
    """AsyncDetector + OAK 슬롯 (6장). **스레드 하나, 모델 하나**: 같은 YOLO 모델을 AsyncDetector 두 개가 나눠 쓰면 predict 가 두 스레드에서
    동시에 불리므로, 슬롯 두 개(팬틸트, OAK)를 한 스레드가 번갈아 처리한다.

      - 팬틸트 슬롯을 항상 먼저 본다. OAK 는 팬틸트 슬롯이 비어 있을 때만, 최대 oak_det.max_hz 로, 그리고 **다음 팬틸트 프레임이 올 때까지
        끝날 틈이 있을 때만** 돌린다(검출 시간이 길어 틈이 없으면 OAK 는 굶는다 — 팬틸트 처리량·지연이 우선이다).
      - STRIKE(조준·사격) 중(set_strike(True))에는 oak_det.pause_in_strike 가 켜져 있으면 OAK 를 처리하지 않는다.
      - 팬틸트 쪽 인터페이스(detect_stamped / health / close)는 AsyncDetector 와 같다. OAK 쪽은 submit_oak / poll_oak 이다.
    """

    def __init__(self, inner: Detector, oak_cfg, clock=time.monotonic) -> None:
        self._clock = clock
        self._oak_cfg = oak_cfg
        self._oak_lock = threading.Lock()
        self._oak_in: Optional[Frame] = None            # OAK 슬롯: 최신 프레임 하나만
        self._oak_submitted = -1.0
        self._oak_latest: List[Detection] = []
        self._oak_latest_stamp = -1.0
        self._strike = False
        self._pt_last_submit: Optional[float] = None    # 마지막 새 팬틸트 프레임이 들어온 시각
        self._pt_period = 0.1                           # 팬틸트 프레임 간격(지수평균, 처음엔 10 fps 가정)
        self._oak_dur: Optional[float] = None           # OAK 검출 한 번의 길이(지수평균)
        self._pt_dur: Optional[float] = None
        self._oak_last_start = -1e9
        self.oak_error_count = 0
        self.oak_last_error: Optional[str] = None
        self.stats = {"pantilt_runs": 0, "oak_runs": 0, "oak_skipped_strike": 0, "oak_skipped_no_gap": 0, "oak_skipped_rate": 0}
        super().__init__(inner)

    # ------------------------------------------------------------------ 팬틸트 쪽
    def detect_stamped(self, frame: Frame) -> Tuple[List[Detection], float]:
        if frame.stamp != self._submitted_stamp:
            now = self._clock()
            if self._pt_last_submit is not None:
                dt = now - self._pt_last_submit
                if 0.0 < dt < 1.0:
                    self._pt_period += 0.2 * (dt - self._pt_period)
            self._pt_last_submit = now
        return super().detect_stamped(frame)

    def _process(self, fr: Frame) -> None:
        t0 = self._clock()
        super()._process(fr)
        dur = self._clock() - t0
        self._pt_dur = dur if self._pt_dur is None else self._pt_dur + 0.3 * (dur - self._pt_dur)
        self.stats["pantilt_runs"] += 1

    # ------------------------------------------------------------------ OAK 쪽
    def set_strike(self, on: bool) -> None:
        self._strike = bool(on)

    def submit_oak(self, frame: Frame) -> None:
        """OAK 슬롯에 최신 프레임을 넣는다(옛 프레임은 버린다)."""
        if frame.stamp == self._oak_submitted:
            return
        with self._oak_lock:
            self._oak_in = frame
            self._oak_submitted = frame.stamp

    def poll_oak(self) -> Tuple[List[Detection], float]:
        with self._oak_lock:
            return list(self._oak_latest), self._oak_latest_stamp

    def _oak_fits_before_next_pantilt(self, now: float) -> bool:
        if self._pt_last_submit is None or now - self._pt_last_submit > 3.0 * max(self._pt_period, 0.05):
            return True                                   # 팬틸트 프레임이 한동안 안 온다 — 틈이 얼마든 있다
        nxt = self._pt_last_submit + self._pt_period
        while nxt < now:                                  # 다음 팬틸트 프레임이 올 것으로 예상되는 시각
            nxt += self._pt_period
        est = self._oak_dur if self._oak_dur is not None else (self._pt_dur if self._pt_dur is not None else 0.05)
        return now + 1.2 * est <= nxt

    def _maybe_run_oak(self) -> None:
        with self._oak_lock:
            fr = self._oak_in
        if fr is None:
            return
        cfg = self._oak_cfg
        now = self._clock()
        if self._strike and cfg.pause_in_strike:
            with self._oak_lock:
                self._oak_in = None                       # 사격 중 쌓인 프레임은 낡는다 — 버린다
            self.stats["oak_skipped_strike"] += 1
            return
        if now - self._oak_last_start < 1.0 / max(float(cfg.max_hz), 1e-6):
            self.stats["oak_skipped_rate"] += 1
            return
        if not self._oak_fits_before_next_pantilt(now):
            self.stats["oak_skipped_no_gap"] += 1
            return
        with self._oak_lock:
            self._oak_in = None
        self._oak_last_start = now
        try:
            dets = self.inner.detect(fr)
        except Exception as e:                            # OAK 의 실패가 팬틸트 건강 상태를 흐리면 안 된다
            self.oak_error_count += 1
            self.oak_last_error = f"{type(e).__name__}: {e}"
            return
        dur = self._clock() - now
        self._oak_dur = dur if self._oak_dur is None else self._oak_dur + 0.3 * (dur - self._oak_dur)
        self.stats["oak_runs"] += 1
        with self._oak_lock:
            self._oak_latest, self._oak_latest_stamp = dets, fr.stamp

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._oak_lock:
                waiting = self._oak_in is not None
            try:
                fr = self._q.get(timeout=0.01 if waiting else 0.1)
            except queue.Empty:
                self._maybe_run_oak()                     # 팬틸트 슬롯이 비어 있을 때만 OAK 를 본다
                continue
            self._process(fr)
