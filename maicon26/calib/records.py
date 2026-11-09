"""측정 기록: 시험 한 번 = Trace(시계열 CSV) + 메타(JSON). rosbag 은 별도 디렉터리로 같이 보관한다.

records/<run>/
   manifest.json            시험 목록, 로봇/주행면/날짜/도구 버전
   <trace>.csv              t, cmd_v, cmd_w, odom_x/y/th, gyro_z, truth_x/y/th(없으면 nan), scan_min, scan_lat
   <trace>.json             그 시험의 메타(주행면, 요청 속도, 수기 측정 진실값 등)
   bag_<experiment>/        ros2 bag (ROS 모드)
"""
from __future__ import annotations

import datetime as _dt
import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

COLS = ["t", "cmd_v", "cmd_w", "odom_x", "odom_y", "odom_th", "gyro_z",
        "truth_x", "truth_y", "truth_th", "scan_min", "scan_lat",
        "odom_t", "truth_t", "imu_t"]          # 마지막 셋: 각 토픽 메시지의 header 시각(표본 시각 t 와 다르다 — 발행 지터 보정용)


@dataclass
class Trace:
    name: str
    meta: Dict[str, Any] = field(default_factory=dict)
    rows: List[List[float]] = field(default_factory=list)
    _arr: Optional[np.ndarray] = None

    def add(self, s: Dict[str, float]) -> None:
        self.rows.append([float(s.get(c, float("nan"))) for c in COLS])
        self._arr = None

    @property
    def a(self) -> np.ndarray:
        if self._arr is None:
            self._arr = np.asarray(self.rows, dtype=float).reshape(-1, len(COLS))
        return self._arr

    def col(self, name: str) -> np.ndarray:
        return self.a[:, COLS.index(name)]

    def has_truth(self) -> bool:
        return bool(np.isfinite(self.col("truth_x")).sum() > 5)

    def save(self, directory: str) -> str:
        os.makedirs(directory, exist_ok=True)
        p = os.path.join(directory, f"{self.name}.csv")
        np.savetxt(p, self.a, delimiter=",", header=",".join(COLS), comments="", fmt="%.6f")
        with open(os.path.join(directory, f"{self.name}.json"), "w", encoding="utf-8") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)
        return p

    @classmethod
    def load(cls, csv_path: str) -> "Trace":
        base = os.path.splitext(csv_path)[0]
        arr = np.loadtxt(csv_path, delimiter=",", skiprows=1, ndmin=2)
        meta: Dict[str, Any] = {}
        if os.path.exists(base + ".json"):
            with open(base + ".json", "r", encoding="utf-8") as f:
                meta = json.load(f)
        t = cls(os.path.basename(base), meta)
        t.rows = arr.tolist()
        return t


def load_run(directory: str) -> List[Trace]:
    out = []
    for fn in sorted(os.listdir(directory)):
        if fn.endswith(".csv"):
            out.append(Trace.load(os.path.join(directory, fn)))
    return out


def write_manifest(directory: str, info: Dict[str, Any], traces: List[Trace]) -> None:
    man = dict(info)
    man["created"] = _dt.datetime.now().isoformat(timespec="seconds")
    man["traces"] = [{"name": t.name, "kind": t.meta.get("kind"), "surface": t.meta.get("surface"),
                      "samples": len(t.rows), "truth": t.has_truth() if t.rows else False} for t in traces]
    with open(os.path.join(directory, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(man, f, ensure_ascii=False, indent=2)


class RosbagRecorder:
    """`ros2 bag record` 를 부분 프로세스로. 시험마다 새 디렉터리. ROS 환경이 없으면 enabled=False 로 조용히 건너뛴다."""

    DEFAULT_TOPICS = ["/cmd_vel", "/cmd_vel_nav2", "/odom", "/imu", "/scan", "/tf", "/tf_static", "/clock", "/ground_truth"]

    def __init__(self, directory: str, topics: Optional[List[str]] = None, enabled: bool = True) -> None:
        self.dir, self.topics, self.enabled = directory, topics or self.DEFAULT_TOPICS, enabled
        self._p: Optional[subprocess.Popen] = None

    def start(self, name: str) -> None:
        if not self.enabled:
            return
        out = os.path.join(self.dir, f"bag_{name}")
        os.makedirs(self.dir, exist_ok=True)
        try:
            self._p = subprocess.Popen(["ros2", "bag", "record", "-o", out] + self.topics,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            time.sleep(1.5)                                     # 구독 연결 대기
        except FileNotFoundError:
            self.enabled = False

    def stop(self) -> None:
        if self._p is None:
            return
        try:
            os.killpg(os.getpgid(self._p.pid), signal.SIGINT)    # 정상 종료(메타 파일 기록)
            self._p.wait(timeout=15)
        except Exception:
            self._p.kill()
        self._p = None
