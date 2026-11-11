#!/usr/bin/env python3
"""Decode a complete Gazebo recording and make a six-frame visual contact sheet."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    args = parser.parse_args()
    video = args.video.resolve()
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {video}")
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    declared = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if fps <= 0 or declared <= 0 or width <= 0 or height <= 0:
        raise RuntimeError("Video metadata is incomplete")
    if declared < 6:
        raise RuntimeError("Recording is too short for six visual samples")
    targets = {int(i): int(i) / fps for i in np.linspace(0, declared - 1, 6).round()}
    samples = {}
    decoded = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if decoded in targets:
            samples[decoded] = frame.copy()
        decoded += 1
    cap.release()
    if decoded != declared or len(samples) != len(targets):
        raise RuntimeError(f"Incomplete decode: {decoded}/{declared}, samples {len(samples)}/{len(targets)}")
    tiles = []
    fingerprints = []
    low_contrast = []
    for index, frame in sorted(samples.items()):
        if float(frame.std()) < 10:
            # A robot-mounted camera can legitimately face a wall at goal entry.
            # Preserve the sample and report it for visual review, never hide it.
            low_contrast.append(round(index / fps, 1))
        fingerprints.append(hashlib.sha256(frame.tobytes()).hexdigest())
        tile = cv2.resize(frame, (640, round(640 * height / width)), interpolation=cv2.INTER_AREA)
        cv2.putText(tile, f"{index / fps:.1f} s", (12, 35), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
        tiles.append(tile)
    if len(set(fingerprints)) < 3:
        raise RuntimeError("Video samples do not show enough scene changes")
    if len(low_contrast) > len(samples) // 2:
        raise RuntimeError(f"Most video samples have little visible detail: {low_contrast}")
    sheet = np.vstack([np.hstack(tiles[:3]), np.hstack(tiles[3:])])
    sheet_path = video.with_name(video.stem + "_contact_sheet.jpg")
    check_path = video.with_name(video.stem + "_video_check.json")
    encoded, jpg = cv2.imencode(".jpg", sheet)
    if not encoded:
        raise RuntimeError(f"Cannot write {sheet_path}")
    sheet_path.write_bytes(jpg.tobytes())
    check = {"ok": True, "video": str(video), "fps": fps,
             "width": width, "height": height, "frames": decoded,
             "duration_s": round(decoded / fps, 2),
             "sample_times_s": [round(i / fps, 1) for i in sorted(samples)],
             "unique_samples": len(set(fingerprints)), "contact_sheet": str(sheet_path),
             "low_contrast_sample_times_s": low_contrast}
    check_path.write_text(json.dumps(check, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(check, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
