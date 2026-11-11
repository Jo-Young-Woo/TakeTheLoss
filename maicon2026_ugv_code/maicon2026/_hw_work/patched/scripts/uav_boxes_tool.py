#!/usr/bin/env python3
"""실기 UAV 상자 검출기 연습·점검 도구. 로봇·ROS 없이 노트북에서 돈다.

  fetch   서버(연습/경기)에서 UAV 프레임 한 장을 받아 저장한다(헤더 X-Camera · X-Captured-At 도 출력).
  detect  저장한 프레임에 검출기를 돌려 상자 칸·기각 이유를 표로 보고, 원본 위에 겹쳐 그린 그림을 쓴다. --sectors 는 섹터별 상자·면 수.
  probe   상자 윗면 점(--box X Y)과 바닥 점(--floor X Y)의 밝기·채도를 재서 uav_boxes.* 문턱 값을 제안한다.

연습 순서(보정은 scripts/calibrate_uav_grid.py 또는 calibrate_uav.py 로 먼저 만든다)
  1) python scripts/uav_boxes_tool.py fetch --server http://<서버>:8000 --camera test1 --out frame_test1.jpg
  2) python scripts/uav_boxes_tool.py detect --image frame_test1.jpg --calibration calib/uav_test1.json --out out/t1 --sectors
     → out/t1_overlay.png 에서 초록(상자)·빨강(기각)·하늘(무시 구역)을 눈으로 확인. 놓친 상자는 기각 이유(fill·small·edge…)를 본다.
  3) 상자가 어둡게 안 잡히면: 상자 윗면 한 칸의 중심 좌표와 빈 바닥 점 몇 개를 줄자로 재서
     python scripts/uav_boxes_tool.py probe --image frame_test1.jpg --calibration calib/uav_test1.json --box 1.8 1.0 --floor 3.0 2.5 --floor 1.0 4.0
     → 제안된 uav_boxes.* 값을 config/waveshare_real.json 의 "uav_boxes" 에 넣거나 detect 의 --set 으로 먼저 써 본다.
  4) 확정한 보정은 calib/uav_main_calibration.json 로 두면(config uav_boxes.calibration) run_real 이 자동으로 켠다.

이 도구는 실영상으로 검증한 정확도를 보장하지 않는다 — 눈으로 겹침을 확인하는 것이 검증이다.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import os
import sys
import urllib.parse
import urllib.request

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def _read_image(path):
    import cv2
    import numpy as np
    data = np.fromfile(path, dtype=np.uint8)               # 한글 경로에서도 읽힌다
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit(f"영상을 읽을 수 없습니다: {path}")
    return img


def _write_image(path, img):
    import cv2
    ok, buf = cv2.imencode(os.path.splitext(path)[1] or ".png", img)
    if not ok:
        raise SystemExit(f"영상을 쓸 수 없습니다: {path}")
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    buf.tofile(path)


def _cfg_with_overrides(sets):
    from maicon26.config import UavBoxesCfg
    cfg = UavBoxesCfg()
    for item in sets or []:
        if "=" not in item:
            raise SystemExit(f"--set 은 키=값 형식입니다: {item}")
        key, raw = item.split("=", 1)
        if not hasattr(cfg, key):
            raise SystemExit(f"알 수 없는 uav_boxes 키: {key}")
        cur = getattr(cfg, key)
        try:
            if isinstance(cur, bool):
                val = raw.strip().lower() in ("1", "true", "yes", "on")
            elif isinstance(cur, int):
                val = int(raw)
            elif isinstance(cur, float):
                val = float(raw)
            elif isinstance(cur, tuple):
                val = tuple(float(x) for x in raw.replace(",", " ").split())
            else:
                val = raw
        except ValueError:
            raise SystemExit(f"값을 읽을 수 없습니다: {item}")
        setattr(cfg, key, val)
    return cfg


def _load_detector(args):
    from maicon26.perception.uav_boxes_real import RealUavBoxDetector
    from maicon26.perception.uav_calibration import load_calibration
    cfg = _cfg_with_overrides(getattr(args, "set", None))
    try:
        cal = load_calibration(args.calibration)
    except (ValueError, OSError, KeyError) as error:
        raise SystemExit(f"보정을 쓸 수 없습니다: {error}")
    return RealUavBoxDetector(cal, cfg, camera=getattr(args, "camera", None)), cal


def cmd_fetch(args):
    from maicon26.credentials import read_mission_code
    from maicon26.link.secrets import redacted_output  # noqa: F401  (임무코드가 출력에 섞이지 않게 비밀로 등록되는 것만 쓴다)
    code = read_mission_code(args.code_file)
    params = {"mission_code": code}
    if args.camera:
        params["camera"] = args.camera
    url = f"{args.server.rstrip('/')}/api/report/drone?{urllib.parse.urlencode(params)}"
    try:
        with urllib.request.urlopen(urllib.request.Request(url, method="GET"), timeout=args.timeout) as resp:
            body, hdr = resp.read(), resp.headers
    except Exception as error:                              # noqa: BLE001 — 주소·코드가 담긴 메시지는 형식만 보인다
        raise SystemExit(f"받기 실패: {type(error).__name__}")
    ctype = str(hdr.get("Content-Type", ""))
    if not ctype.lower().startswith("image/jpeg") or not body:
        raise SystemExit(f"JPEG 가 아닙니다(Content-Type={ctype!r}, {len(body)} bytes)")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "wb") as f:
        f.write(body)
    keys = ("X-Camera", "X-Captured-At", "X-Frame-Interval-S", "X-Test-Mode", "X-Camp", "X-Destroyed")
    print(f"저장 {args.out} ({len(body)} bytes)  " + "  ".join(f"{k}={hdr.get(k)}" for k in keys if hdr.get(k) is not None))


def cmd_detect(args):
    det, cal = _load_detector(args)
    img = _read_image(args.image)
    rep = det.analyze(img, debug=True)
    print(f"프레임: {'사용 가능' if rep.ok else '사용 불가 — ' + rep.reason}  바닥 밝기 {rep.floor_v:.0f}  윗면 문턱 {rep.thr_v:.0f}  "
          f"윗면색 화소 {rep.fg_fraction:.1%}  덩어리 {rep.components}  {rep.elapsed_ms:.0f} ms")
    print(f"상자 {len(rep.boxes)}개, 기각 {sum(rep.rejects.values())}건 {rep.rejects}")
    for c in sorted(rep.cells, key=lambda c: (c.status != "accepted", c.cy, c.cx)):
        if c.status == "accepted" or args.all:
            print(f"  {c.status:9s} ({c.cx:5.2f},{c.cy:5.2f}) 채움 {c.fill:4.2f} 신뢰 {c.conf:4.2f} {c.reason}")
    if args.sectors:
        from maicon26.arena import ArenaGeometry
        arena = ArenaGeometry()
        if args.arena_spec:
            import yaml
            with open(args.arena_spec, encoding="utf-8") as f:
                arena = ArenaGeometry.from_spec(yaml.safe_load(f))
        from maicon26.link.schema import server_sector
        per = {}
        for b in rep.boxes:
            name = server_sector(arena.sector_of(b.cx, b.cy) or "") or "섹터 밖"
            per.setdefault(name, []).append(b)
        print("섹터별(상자 중심 기준, 면 = 상자 × 4 — 이웃 상자·벽에 가린 면은 지상에서 가려짐으로 센다):")
        for name in sorted(per):
            print(f"  {name:8s} 상자 {len(per[name])}개 / 면 최대 {4 * len(per[name])}")
    if args.out:
        _write_image(args.out + "_overlay.png", det.draw(img, rep))
        _write_image(args.out + "_top.png", det.rectified(img))
        mask = rep.debug.get("mask")
        if mask is not None:
            _write_image(args.out + "_mask.png", mask)
        with open(args.out + ".json", "w", encoding="utf-8") as f:
            json.dump({"summary": rep.summary(), "cells": [c.to_dict() for c in rep.cells],
                       "boxes": [dataclasses.asdict(b) for b in rep.boxes]}, f, ensure_ascii=False, indent=1)
        print(f"쓴 파일: {args.out}_overlay.png(겹침) _top.png(위에서 본 모양) _mask.png(윗면색 마스크) .json")
    return 0 if rep.ok else 2


def cmd_probe(args):
    from maicon26.perception.uav_boxes_real import probe_points, suggest_thresholds
    det, cal = _load_detector(args)
    img = _read_image(args.image)
    boxes = probe_points(det, img, [tuple(p) for p in args.box or []], None)
    floors = probe_points(det, img, [tuple(p) for p in args.floor or []], cal.reference_plane_z)
    for kind, rows in (("상자 윗면", boxes), ("바닥", floors)):
        for r in rows:
            if r.get("n"):
                print(f"{kind} ({r['x']:.2f},{r['y']:.2f}): V 5%={r['v_p05']:.0f} 중앙={r['v_p50']:.0f} 95%={r['v_p95']:.0f} 최대={r['v_max']:.0f}  S 중앙={r['s_p50']:.0f} 95%={r['s_p95']:.0f}")
            else:
                print(f"{kind} ({r['x']:.2f},{r['y']:.2f}): 영상 밖")
    sug = suggest_thresholds(boxes, floors, min_floor_contrast=float(det.cfg.min_floor_contrast))
    print(json.dumps(sug, ensure_ascii=False, indent=1))
    if sug.get("ok"):
        keep = {k: v for k, v in sug.items() if k in ("top_color", "v_black_max", "v_black_rel", "grey_v_min", "grey_v_max", "grey_s_max")}
        print("config/waveshare_real.json 의 uav_boxes 에 넣을 값:\n" + json.dumps({"uav_boxes": keep}, ensure_ascii=False, indent=1))
        return 0
    return 2


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fetch", help="서버에서 UAV 프레임 한 장 받기")
    f.add_argument("--server", required=True)
    f.add_argument("--camera", choices=["main", "test1", "test2"])
    f.add_argument("--code-file")
    f.add_argument("--out", required=True)
    f.add_argument("--timeout", type=float, default=5.0)
    f.set_defaults(fn=cmd_fetch)
    for name, fn, help_ in (("detect", cmd_detect, "검출기 실행·겹쳐 그리기"), ("probe", cmd_probe, "색 표본 → 문턱 제안")):
        p = sub.add_parser(name, help=help_)
        p.add_argument("--image", required=True)
        p.add_argument("--calibration", required=True)
        p.add_argument("--camera", choices=["main", "test1", "test2"])
        p.add_argument("--set", action="append", metavar="키=값", help="uav_boxes 설정을 잠깐 바꿔 본다(예: --set v_black_max=60)")
        if name == "detect":
            p.add_argument("--out", help="출력 접두어(_overlay.png · _top.png · _mask.png · .json)")
            p.add_argument("--all", action="store_true", help="기각한 칸도 표에 나열")
            p.add_argument("--sectors", action="store_true", help="섹터별 상자·면 수")
            p.add_argument("--arena-spec", help="측정 경기장 YAML(섹터 경계)")
        else:
            p.add_argument("--box", nargs=2, type=float, action="append", metavar=("X", "Y"), help="상자 윗면 중심(아레나 좌표 m)")
            p.add_argument("--floor", nargs=2, type=float, action="append", metavar=("X", "Y"), help="빈 바닥 점(아레나 좌표 m)")
        p.set_defaults(fn=fn)
    args = ap.parse_args(argv)
    return args.fn(args) or 0


if __name__ == "__main__":
    sys.exit(main())
