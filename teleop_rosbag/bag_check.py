#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""기록한 bag 을 바로 점검한다. metadata.yaml 만 읽으므로 ROS 없이도 돈다.

  python3 bag_check.py ~/bags/20261118_101500_cal_cam_jetson        # 세션 폴더 또는 bag 폴더
  python3 bag_check.py <폴더> --scan-topic /scan --expect /oak/rgb/image_raw

토픽별 메시지 수와 평균 주기를 보여 주고, 꼭 있어야 하는 토픽이 빠졌으면 종료 코드 1 을 낸다.
세션 폴더에 teleop_events.jsonl 이 있으면 마커(M 키) 시각도 함께 보여 준다.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

REQUIRED = {"/odom": 10.0, "/image_raw": 5.0}          # 이름: 기대 최소 Hz (낮으면 경고, 없으면 실패)
WANTED = {"/cmd_vel": 5.0, "/imu/data": 0.0, "/tf": 0.0, "/tf_static": 0.0, "/mil/state": 0.0, "/teleop/events": 0.0}


def find_metadata(path: str):
    for p in (path, os.path.join(path, "bag")):
        m = os.path.join(p, "metadata.yaml")
        if os.path.isfile(m):
            return p, m
    return None, None


def load_info(meta_path: str) -> dict:
    import yaml
    with open(meta_path, encoding="utf-8") as f:
        info = (yaml.safe_load(f) or {}).get("rosbag2_bagfile_information") or {}
    dur = float((info.get("duration") or {}).get("nanoseconds", 0)) / 1e9
    start = float((info.get("starting_time") or {}).get("nanoseconds_since_epoch", 0)) / 1e9
    topics = {}
    for t in info.get("topics_with_message_count") or []:
        md = t.get("topic_metadata") or {}
        topics[md.get("name", "?")] = {"type": md.get("type", "?"), "count": int(t.get("message_count", 0))}
    return {"storage": info.get("storage_identifier", "?"), "duration": dur, "start": start,
            "count": int(info.get("message_count", 0)), "topics": topics,
            "files": list(info.get("relative_file_paths") or [])}


def check(info: dict, required: dict, wanted: dict):
    """(실패 목록, 경고 목록). 실패 = 꼭 있어야 하는 토픽이 없거나 메시지가 0개."""
    fails, warns = [], []
    dur = max(info["duration"], 1e-6)
    if info["duration"] < 1.0:
        warns.append(f"기록 길이가 {info['duration']:.1f}초로 짧다")
    for group, out in ((required, fails), (wanted, warns)):
        for name, min_hz in group.items():
            t = info["topics"].get(name)
            if t is None or t["count"] == 0:
                out.append(f"{name} 없음")
            elif min_hz > 0 and t["count"] / dur < min_hz:
                warns.append(f"{name} 주기가 낮다: {t['count'] / dur:.1f} Hz (기대 {min_hz:g} Hz 이상)")
    return fails, warns


def load_markers(session: str, start: float):
    p = os.path.join(session, "teleop_events.jsonl")
    out = []
    if not os.path.isfile(p):
        return out
    with open(p, encoding="utf-8") as f:
        for line in f:
            try:
                e = json.loads(line)
            except ValueError:
                continue
            if e.get("type") == "marker":
                out.append((int(e.get("n", 0)), float(e.get("t", 0)) - start))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="bag 점검(토픽 수·주기·빠진 토픽)")
    ap.add_argument("path", help="세션 폴더 또는 bag 폴더")
    ap.add_argument("--scan-topic", default="/scan", help="LiDAR 토픽 이름(자료에 없어 /scan 으로 가정)")
    ap.add_argument("--expect", action="append", default=[], help="꼭 있어야 하는 토픽 추가")
    a = ap.parse_args(argv)
    bag, meta = find_metadata(a.path)
    if meta is None:
        print(f"metadata.yaml 이 없다: {a.path}  (기록이 비정상 종료됐으면 ros2 bag reindex 로 복구)")
        return 2
    info = load_info(meta)
    required = dict(REQUIRED, **{a.scan_topic: 5.0}, **{t: 0.0 for t in a.expect})
    fails, warns = check(info, required, WANTED)
    size = sum(os.path.getsize(os.path.join(bag, f)) for f in os.listdir(bag) if os.path.isfile(os.path.join(bag, f)))
    dur = max(info["duration"], 1e-6)
    print(f"bag: {bag}")
    print(f"저장 {info['storage']}  길이 {info['duration']:.1f}초  메시지 {info['count']}개  토픽 {len(info['topics'])}개  "
          f"크기 {size / 1e6:.0f}MB (분당 {size / 1e6 / dur * 60:.0f}MB)")
    print(f"{'토픽':40s} {'개수':>8s} {'Hz':>7s}  타입")
    for name in sorted(info["topics"]):
        t = info["topics"][name]
        print(f"{name:40s} {t['count']:8d} {t['count'] / dur:7.1f}  {t['type']}")
    session = os.path.dirname(bag.rstrip("/")) if os.path.basename(bag.rstrip("/")) == "bag" else bag
    marks = load_markers(session, info["start"])
    if marks:
        print("마커: " + ", ".join(f"M{n} +{t:.1f}초" for n, t in marks))
    for w in warns:
        print("경고: " + w)
    for f in fails:
        print("실패: " + f)
    print("결과: " + ("다시 찍어야 한다" if fails else "경고 있음" if warns else "정상"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
