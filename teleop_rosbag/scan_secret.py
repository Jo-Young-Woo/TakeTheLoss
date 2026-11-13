#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""기록 폴더에 임무코드가 남았는지 검사한다. 값은 절대 출력하지 않는다.

  MAICON_CODE=<임무코드> python3 scan_secret.py ~/bags/<세션 폴더>

환경변수 MAICON_CODE(또는 MAICON_MISSION_CODE)의 값을 폴더 안 모든 파일의 바이트에서 찾고,
rosbag2_py 가 있으면(로봇의 ROS 환경) bag 의 메시지도 하나씩 풀어서 찾는다.
발견되면 DO_NOT_SHARE 파일을 만들고 종료 코드 1 을 낸다. 없으면 0, 검사할 값이 없으면 2.
"""
from __future__ import annotations

import os
import sys
import urllib.parse

CHUNK = 1 << 20


def variants(code: str):
    forms = {code, code.strip(), code.lower(), code.upper(), urllib.parse.quote(code, safe=""), urllib.parse.quote_plus(code)}
    out = set()
    for s in forms:
        if len(s) < 4:
            continue                                         # 너무 짧으면 우연히 맞는다
        for enc in ("utf-8", "utf-16-le", "utf-16-be"):
            out.add(s.encode(enc))
    return sorted(out, key=len, reverse=True)


def file_hits(path: str, needles) -> int:
    keep = max(len(n) for n in needles) - 1
    hits, tail = 0, b""
    with open(path, "rb") as f:
        while True:
            block = f.read(CHUNK)
            if not block:
                break
            data = tail + block                              # 앞 블록의 끝을 붙여 경계에 걸친 것도 찾는다
            hits += sum(data.count(n) - tail.count(n) for n in needles)   # tail 안에서 이미 센 것은 뺀다
            tail = data[-keep:] if keep > 0 else b""
    return hits


def scan_files(root: str, needles):
    found = {}
    for d, _, files in os.walk(root):
        for name in files:
            if name == "DO_NOT_SHARE":
                continue
            p = os.path.join(d, name)
            try:
                n = file_hits(p, needles)
            except OSError:
                continue
            if n:
                found[os.path.relpath(p, root)] = n
    return found


def scan_messages(root: str, needles):
    """메시지 단위 검사. (토픽별 개수, 검사했는가). 압축된 mcap 은 파일 바이트 검사로는 못 찾으므로 이 경로가 필요하다."""
    bag = None
    for p in (root, os.path.join(root, "bag")):
        if os.path.isfile(os.path.join(p, "metadata.yaml")):
            bag = p
    if bag is None:
        return {}, False
    try:
        import rosbag2_py
        import yaml
        with open(os.path.join(bag, "metadata.yaml"), encoding="utf-8") as f:
            storage = (yaml.safe_load(f) or {}).get("rosbag2_bagfile_information", {}).get("storage_identifier", "sqlite3")
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage), rosbag2_py.ConverterOptions("", ""))
    except Exception:
        return {}, False
    found = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if any(n in data for n in needles):
            found[topic] = found.get(topic, 0) + 1
    return found, True


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1 or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 2
    root = argv[0]
    code = os.environ.get("MAICON_CODE") or os.environ.get("MAICON_MISSION_CODE") or ""
    needles = variants(code) if code else []
    if not needles:
        print("검사할 값이 없다. MAICON_CODE 환경변수를 넣고 실행한다(4자 이상).")
        return 2
    if not os.path.isdir(root):
        print(f"폴더가 없다: {root}")
        return 2
    files = scan_files(root, needles)
    topics, did_messages = scan_messages(root, needles)
    for name, n in sorted(files.items()):
        print(f"발견: 파일 {name} ({n}곳)")
    for name, n in sorted(topics.items()):
        print(f"발견: 토픽 {name} (메시지 {n}개)")
    if not did_messages:
        print("참고: 메시지 단위 검사는 하지 못했다(rosbag2_py 없음 또는 metadata 없음). 압축된 bag 은 로봇의 ROS 환경에서 다시 검사한다.")
    if files or topics:
        with open(os.path.join(root, "DO_NOT_SHARE"), "w", encoding="utf-8") as f:
            f.write("임무코드가 이 기록에 들어 있다. 팀 밖으로도, 팀 공유 폴더로도 옮기지 않는다.\n")
        print("결과: 임무코드가 들어 있다. DO_NOT_SHARE 를 남겼다.")
        return 1
    print("결과: 임무코드를 찾지 못했다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
