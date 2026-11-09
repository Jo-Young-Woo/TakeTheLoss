"""Bounded-memory recording of real ROS camera frames on a simulation timeline."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from .gazebo_uav import decode_bgr as decode_bgr  # public compatibility export

__all__ = ["decode_bgr", "SimulationVideoWriter"]


class SimulationVideoWriter:
    """Resample received frames to simulation time, documenting every held frame.

    Camera silence is never replaced by generated views: gaps beyond max_hold_s
    become explicitly marked missing-frame cards. Only the last image is kept.
    """

    def __init__(self, output: Path, fps: float = 10.0, max_hold_s: float = 1.0,
                 label: str = "", writer_factory: Optional[Callable] = None):
        import cv2
        if not math.isfinite(fps) or not 1 <= fps <= 60:
            raise ValueError("fps must be finite and between 1 and 60")
        if not math.isfinite(max_hold_s) or max_hold_s < 0:
            raise ValueError("max_hold_s must be finite and nonnegative")
        self.output = Path(output)
        self.timestamps = self.output.with_suffix(".frames.jsonl")
        self.metadata = self.output.with_suffix(".recording.json")
        self.first_preview = self.output.with_suffix(".first.png")
        self.last_preview = self.output.with_suffix(".last.png")
        for path in (self.output, self.timestamps, self.metadata, self.first_preview, self.last_preview):
            if path.exists():
                raise FileExistsError(f"Refusing to overwrite recording artifact: {path}")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.fps, self.max_hold_s, self.label = fps, max_hold_s, label
        self._factory = writer_factory or cv2.VideoWriter
        self._writer = self._log = self._previous = None
        self.first_stamp = self.last_stamp = None
        self.size = None
        self.source_frames = self.encoded_frames = self.held_frames = self.missing_frames = 0
        self.duplicate_stamps = self.same_slot_frames = 0
        self.max_source_gap_s = 0.0
        self.closed = False

    def add(self, bgr: np.ndarray, stamp: float) -> bool:
        import cv2
        if self.closed:
            raise RuntimeError("Recorder is already closed")
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError("Camera timestamp must be finite and nonnegative")
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError("Expected a uint8 BGR image")
        size = (bgr.shape[1], bgr.shape[0])
        if min(size) < 2 or size[0] % 2 or size[1] % 2:
            raise ValueError("H.264 recording requires positive even image dimensions")
        if self.size is not None and size != self.size:
            raise ValueError("Camera resolution changed during recording")
        if self.last_stamp is not None and stamp < self.last_stamp - 1e-6:
            raise ValueError("Camera clock moved backward; refusing a misleading video timeline")
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.duplicate_stamps += 1
            return False
        gap = None if self.last_stamp is None else stamp - self.last_stamp
        if gap is not None and gap > 60:
            raise ValueError("Camera gap exceeded 60 simulation seconds")
        if self._writer is None:
            self.size = size
            self._writer = self._factory(str(self.output), cv2.VideoWriter_fourcc(*"avc1"), self.fps, size)
            if not self._writer.isOpened():
                raise RuntimeError("OpenCV H.264 encoder is unavailable")
            self._log = self.timestamps.open("x", encoding="utf-8")
            self.first_stamp = stamp
            if not cv2.imwrite(str(self.first_preview), bgr):
                raise OSError("Could not write first camera preview")
        self.source_frames += 1
        target = int(math.floor((stamp - self.first_stamp) * self.fps + .5))
        if gap is not None:
            self.max_source_gap_s = max(self.max_source_gap_s, gap)
        if target < self.encoded_frames:
            self.same_slot_frames += 1
        for index in range(self.encoded_frames, target + 1):
            current = index == target
            frame = bgr if current else self._previous
            image_stamp = stamp if current else self.last_stamp
            timeline_s = index / self.fps
            held = not current
            missing = held and self.first_stamp + timeline_s - image_stamp > self.max_hold_s + 1e-6
            if missing:
                frame = np.zeros_like(bgr)
                cv2.putText(frame, "CAMERA FRAME MISSING", (20, max(35, size[1] // 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, .6, (200, 200, 255), 1, cv2.LINE_AA)
                self.missing_frames += 1
            elif held:
                self.held_frames += 1
            if self.label:
                frame = frame.copy()
                cv2.rectangle(frame, (0, 0), (size[0], 28), (25, 25, 25), -1)
                cv2.putText(frame, f"{self.label} | simulation +{timeline_s:.1f}s", (8, 19),
                            cv2.FONT_HERSHEY_SIMPLEX, .45, (245, 245, 245), 1, cv2.LINE_AA)
            self._writer.write(np.ascontiguousarray(frame))
            self._log.write(json.dumps({"frame": index, "video_s": round(timeline_s, 6),
                                        "image_sim_s": image_stamp, "held": held,
                                        "missing": missing}) + "\n")
            self.encoded_frames += 1
        self._log.flush()
        self._previous = bgr.copy()
        self.last_stamp = stamp
        return True

    def close(self) -> dict:
        import cv2
        if not self.closed:
            if self._writer is not None:
                self._writer.release()
            if self._log is not None:
                self._log.close()
            if self._previous is not None:
                cv2.imwrite(str(self.last_preview), self._previous)
            self.closed = True
        span = 0.0 if self.first_stamp is None or self.last_stamp is None else self.last_stamp - self.first_stamp
        return {"output": str(self.output), "timestamps": str(self.timestamps),
                "camera_image_only": True, "timeline": "simulation_time",
                "codec": "H264", "fps": self.fps, "size": self.size,
                "first_image_sim_s": self.first_stamp, "last_image_sim_s": self.last_stamp,
                "source_span_s": span, "source_frames": self.source_frames,
                "source_fps": (self.source_frames - 1) / span if span > 0 else 0.0,
                "encoded_frames": self.encoded_frames, "video_duration_s": self.encoded_frames / self.fps,
                "held_frames": self.held_frames, "missing_frames": self.missing_frames,
                "duplicate_stamps": self.duplicate_stamps, "same_slot_frames": self.same_slot_frames,
                "max_source_gap_s": self.max_source_gap_s, "max_hold_s": self.max_hold_s,
                "first_preview": str(self.first_preview), "last_preview": str(self.last_preview)}
