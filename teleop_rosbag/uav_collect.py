#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UAV(상공 카메라) 영상을 일정 간격으로 받아 저장한다. HTTP 로 오는 영상이라 rosbag 에는 들어가지 않는다.

  MAICON_CODE=<임무코드> python3 uav_collect.py --server http://<서버> --camera main --label layout1
  python3 uav_collect.py --server http://<서버> --count 1                 # 한 장만

저장: ~/bags/uav/<날짜_시각>_<label>/f_0001.jpg … 와 frames.csv(파일·받은 시각·X-Camera·X-Captured-At).
같은 촬영 시각의 프레임은 다시 저장하지 않는다. Ctrl-C 로 끝낸다.
임무코드는 MAICON_CODE 환경변수나 숨김 입력으로만 받고, 화면·파일·오류 메시지에 남기지 않는다.
요청 방식은 maicon2026/scripts/uav_boxes_tool.py 의 fetch 와 같다(/api/report/drone).
"""
from __future__ import annotations

import argparse
import csv
import getpass
import os
import sys
import time
import urllib.parse
import urllib.request

HEADERS = ("X-Camera", "X-Captured-At", "X-Frame-Interval-S", "X-Test-Mode")


def read_code(code_file: str = "") -> str:
    code = os.environ.get("MAICON_CODE") or os.environ.get("MAICON_MISSION_CODE") or ""
    if not code and code_file:
        with open(os.path.expanduser(code_file), encoding="utf-8") as f:
            code = f.read().strip()
    if not code:
        code = getpass.getpass("임무코드(화면에 보이지 않는다): ").strip()
    if not code:
        raise SystemExit("임무코드가 없다")
    return code


def fetch(server: str, code: str, camera: str, timeout: float):
    """(JPEG 바이트, 헤더 dict). 실패하면 RuntimeError(주소와 코드가 담기지 않은 짧은 이유)."""
    params = {"mission_code": code}
    if camera:
        params["camera"] = camera
    url = f"{server.rstrip('/')}/api/report/drone?{urllib.parse.urlencode(params)}"
    try:
        with urllib.request.urlopen(urllib.request.Request(url, method="GET"), timeout=timeout) as resp:
            body, hdr = resp.read(), resp.headers
    except Exception as error:                               # 메시지에 주소·코드가 섞이지 않게 형식만 남긴다
        raise RuntimeError(type(error).__name__ + (f" {error.code}" if hasattr(error, "code") else "")) from None
    ctype = str(hdr.get("Content-Type", ""))
    if not ctype.lower().startswith("image/jpeg") or not body:
        raise RuntimeError(f"JPEG 가 아니다(Content-Type={ctype[:40]!r}, {len(body)} bytes)")
    return body, {k: hdr.get(k) for k in HEADERS}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="UAV 영상 반복 수집")
    ap.add_argument("--server", default=os.environ.get("MAICON_SERVER_URL") or os.environ.get("MAICON_SERVER") or os.environ.get("MAICON_BASE"),
                    help="서버 주소. 없으면 MAICON_SERVER_URL 환경변수")
    ap.add_argument("--camera", choices=["main", "test1", "test2"], default="main", help="연습 카메라(test1·test2)는 보정이 다르다")
    ap.add_argument("--interval", type=float, default=2.0, help="받는 간격 [s]")
    ap.add_argument("--count", type=int, default=0, help="저장할 장 수 (0 = Ctrl-C 까지)")
    ap.add_argument("--label", default="uav", help="폴더 이름에 붙일 말(예: layout1). 상자 배치를 바꿀 때마다 새로")
    ap.add_argument("--out", default="", help="저장 폴더(기본 ~/bags/uav/<날짜_시각>_<label>)")
    ap.add_argument("--code-file", default="", help="임무코드 파일. 환경변수가 우선")
    ap.add_argument("--timeout", type=float, default=5.0)
    a = ap.parse_args(argv)
    if not a.server:
        print("--server 또는 MAICON_SERVER_URL 이 필요하다")
        return 2
    if any(c not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-" for c in a.label):
        print("label 은 영문·숫자·_·- 만 쓴다")
        return 2
    code = read_code(a.code_file)
    out = a.out or os.path.join(os.path.expanduser("~/bags/uav"), time.strftime("%Y%m%d_%H%M%S") + "_" + a.label)
    os.makedirs(out, exist_ok=True)
    log_path = os.path.join(out, "frames.csv")
    new = not os.path.exists(log_path)
    saved = fails = 0
    last_stamp = None
    print(f"저장 폴더: {out}  (카메라 {a.camera}, 간격 {a.interval:g}초, Ctrl-C 로 끝)")
    with open(log_path, "a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["file", "received_at", "bytes"] + list(HEADERS))
        try:
            while a.count <= 0 or saved < a.count:
                t = time.time()
                try:
                    body, hdr = fetch(a.server, code, a.camera, a.timeout)
                except RuntimeError as e:
                    fails += 1
                    print(f"받기 실패({fails}): {e}")
                    if fails >= 5 and saved == 0:
                        print("다섯 번 연속 실패했다. 서버 주소·임무코드·카메라 이름을 확인한다.")
                        return 1
                else:
                    stamp = hdr.get("X-Captured-At")
                    if stamp is not None and stamp == last_stamp:
                        pass                                 # 서버가 아직 새 프레임을 안 냈다
                    else:
                        last_stamp = stamp
                        saved += 1
                        name = f"f_{saved:04d}.jpg"
                        with open(os.path.join(out, name), "wb") as img:
                            img.write(body)
                        w.writerow([name, f"{t:.3f}", len(body)] + [hdr.get(k) or "" for k in HEADERS])
                        f.flush()
                        cam = hdr.get("X-Camera")
                        print(f"{name}  {len(body)} bytes  X-Camera={cam}  X-Captured-At={stamp}"
                              + ("" if cam in (None, a.camera) else "  ← 요청한 카메라와 다르다"))
                if a.count > 0 and saved >= a.count:
                    break
                time.sleep(max(0.0, a.interval - (time.time() - t)))
        except KeyboardInterrupt:
            print()
    print(f"끝: {saved}장 저장, 실패 {fails}번 → {out}")
    print("상자 배치를 줄자로 잰 좌표를 같은 폴더에 boxes.txt 로 적어 둔다(검출 오차를 재는 기준).")
    return 0 if saved else 1


if __name__ == "__main__":
    sys.exit(main())
