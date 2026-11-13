#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bag 에서 카메라 raw 영상을 뽑는다. 체스판 구간만 뽑아 캘리브레이션에 다시 쓰는 용도다.

  python3 extract_images.py ~/bags/<세션> --out cal_images                  # /image_raw 를 초당 2장
  python3 extract_images.py ~/bags/<세션> --segment 1 --hz 1 --out cal_images   # 마커 M1 ~ M2 사이만
  python3 extract_images.py ~/bags/<세션> --list                             # 마커와 구간 번호만 보기

구간 번호: 0 = 시작 ~ M1, 1 = M1 ~ M2, … 마지막 = 마지막 마커 ~ 끝. 마커는 텔레옵의 M 키(/teleop/events)다.
PNG(무손실)로 저장한다. 파일 이름은 <촬영 시각 ns>.png 이고, 같은 폴더의 frames.csv 에 목록을 남긴다.
로봇의 ROS 2 환경(rosbag2_py, rclpy)과 OpenCV·numpy 가 필요하다.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys


# ---------------------------------------------------------------- ROS 없이 시험되는 부분
def image_to_bgr(height: int, width: int, encoding: str, step: int, data):
    """sensor_msgs/Image → BGR 배열(numpy). 지원: bgr8, rgb8, mono8, bgra8, rgba8."""
    import numpy as np
    enc = (encoding or "").lower()
    ch = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}.get(enc)
    if ch is None:
        raise ValueError(f"지원하지 않는 encoding: {encoding}")
    step = int(step) or width * ch
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if height <= 0 or width <= 0 or step < width * ch or buf.size < height * step:
        raise ValueError("영상 크기와 버퍼 길이가 맞지 않는다")
    img = buf[: height * step].reshape(height, step)[:, : width * ch].reshape(height, width, ch)
    if enc == "mono8":
        return np.repeat(img, 3, axis=2)
    if enc in ("rgb8", "rgba8"):
        return img[:, :, [2, 1, 0]].copy()
    return img[:, :, :3].copy()


def segments(marker_times, t_start: float, t_end: float):
    """마커 시각들로 구간 [(번호, 시작, 끝)] 을 만든다. 번호 0 은 시작 ~ 첫 마커."""
    cuts = [t_start] + sorted(t for t in marker_times if t_start <= t <= t_end) + [t_end]
    return [(i, cuts[i], cuts[i + 1]) for i in range(len(cuts) - 1)]


def pick(stamps, hz: float):
    """시각 목록에서 1/hz 간격으로 고른 인덱스. hz <= 0 이면 전부."""
    if hz <= 0:
        return list(range(len(stamps)))
    out, nxt = [], None
    for i, t in enumerate(stamps):
        if nxt is None or t >= nxt - 1e-9:
            out.append(i)
            nxt = t + 1.0 / hz
    return out


def markers_from_jsonl(path: str):
    out = []
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            for line in f:
                try:
                    e = json.loads(line)
                except ValueError:
                    continue
                if e.get("type") == "marker":
                    out.append(float(e.get("t", 0)))
    return out


# ---------------------------------------------------------------- bag 읽기
def find_bag(path: str):
    for p in (path, os.path.join(path, "bag")):
        if os.path.isfile(os.path.join(p, "metadata.yaml")):
            return p
    return None


def open_reader(bag: str, topics):
    import rosbag2_py
    import yaml
    with open(os.path.join(bag, "metadata.yaml"), encoding="utf-8") as f:
        info = (yaml.safe_load(f) or {}).get("rosbag2_bagfile_information", {})
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=info.get("storage_identifier", "sqlite3")),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    start = float(info.get("starting_time", {}).get("nanoseconds_since_epoch", 0)) / 1e9
    dur = float(info.get("duration", {}).get("nanoseconds", 0)) / 1e9
    return reader, types, start, start + dur


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="bag 에서 raw 영상 뽑기")
    ap.add_argument("path", help="세션 폴더 또는 bag 폴더")
    ap.add_argument("--topic", default="/image_raw")
    ap.add_argument("--hz", type=float, default=2.0, help="초당 몇 장 (0 = 전부)")
    ap.add_argument("--segment", type=int, default=None, help="이 구간만 (0 = 시작~M1, 1 = M1~M2, …)")
    ap.add_argument("--out", default="", help="저장 폴더")
    ap.add_argument("--list", action="store_true", help="마커와 구간만 보여 주고 끝낸다")
    a = ap.parse_args(argv)
    bag = find_bag(a.path)
    if bag is None:
        print(f"metadata.yaml 이 없다: {a.path}")
        return 2
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        reader, types, t0, t1 = open_reader(bag, [a.topic, "/teleop/events"])
    except ImportError as e:
        print(f"ROS 2 환경이 필요하다(rosbag2_py, rclpy): {e}")
        return 2
    if a.topic not in types:
        print(f"{a.topic} 이 bag 에 없다. 있는 영상 토픽: " + ", ".join(n for n, t in types.items() if t.endswith("/Image")))
        return 2
    msg_cls = get_message(types[a.topic])
    evt_cls = get_message(types["/teleop/events"]) if "/teleop/events" in types else None
    marks, frames = [], []                                   # frames: (받은 시각 s, 직렬화 바이트)
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == "/teleop/events" and evt_cls is not None:
            try:
                if json.loads(deserialize_message(data, evt_cls).data).get("type") == "marker":
                    marks.append(t_ns / 1e9)
            except ValueError:
                pass
        elif topic == a.topic:
            frames.append((t_ns / 1e9, data))
    if not marks:                                            # bag 안에 이벤트가 없으면 세션 폴더의 파일을 본다
        session = os.path.dirname(bag.rstrip("/")) if os.path.basename(bag.rstrip("/")) == "bag" else bag
        marks = markers_from_jsonl(os.path.join(session, "teleop_events.jsonl"))
    if frames:
        t0, t1 = min(t0, frames[0][0]), max(t1, frames[-1][0])
    segs = segments(marks, t0, t1)
    print(f"영상 {len(frames)}장, 마커 {len(marks)}개")
    for i, s, e in segs:
        n = sum(1 for t, _ in frames if s <= t <= e)
        print(f"  구간 {i}: +{s - t0:.1f}초 ~ +{e - t0:.1f}초  영상 {n}장")
    if a.list:
        return 0
    if not a.out:
        print("--out 폴더를 지정한다")
        return 2
    if a.segment is not None:
        if not 0 <= a.segment < len(segs):
            print(f"구간 번호는 0 ~ {len(segs) - 1}")
            return 2
        _, s, e = segs[a.segment]
        frames = [(t, d) for t, d in frames if s <= t <= e]
    import cv2
    os.makedirs(a.out, exist_ok=True)
    keep = pick([t for t, _ in frames], a.hz)
    saved = 0
    with open(os.path.join(a.out, "frames.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["file", "stamp_ns", "bag_time_s", "width", "height", "encoding"])
        for i in keep:
            t, data = frames[i]
            m = deserialize_message(data, msg_cls)
            try:
                img = image_to_bgr(m.height, m.width, m.encoding, m.step, m.data)
            except ValueError as err:
                print(f"건너뜀: {err}")
                continue
            stamp = int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec)
            name = f"{stamp or int(t * 1e9)}.png"
            cv2.imwrite(os.path.join(a.out, name), img)
            w.writerow([name, stamp, f"{t - t0:.3f}", m.width, m.height, m.encoding])
            saved += 1
    print(f"저장 {saved}장 → {a.out}  (해상도는 frames.csv 에 있다. 보정에는 경기 때와 같은 640×480 이어야 한다)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
