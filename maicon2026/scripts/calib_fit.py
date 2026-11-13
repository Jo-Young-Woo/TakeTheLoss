#!/usr/bin/env python3
"""기록 → 분석 → 프로파일 갱신(+보고서). 카메라 내부 파라미터는 체커보드 이미지로.

  python scripts/calib_fit.py --records calib/records/20260820_101500_ros [더 많은 폴더...]
  python scripts/calib_fit.py --camera-images calib/camera/*.png --board 9x6 --square 0.025 --camera-model fisheye
기본으로 calib/robot_profile.yaml 을 읽어 갱신한다(원본은 .bak 으로 보관). 보고서: calib/reports/<시각>.md
"""
import argparse, datetime, glob, os, shutil, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.calib.analysis import fit_run
from maicon26.calib.profile import Profile
from maicon26.calib.records import load_run

ROOT = os.path.join(os.path.dirname(__file__), "..")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--records", nargs="*", default=[])
    ap.add_argument("--profile", default=os.path.join(ROOT, "calib", "robot_profile.yaml"))
    ap.add_argument("--ref-surface", default="default", help="슬립 0 으로 가정하는 기준 주행면")
    ap.add_argument("--camera-images", nargs="*", default=None)
    ap.add_argument("--board", default="9x6", help="체커보드 내부 코너 수 열x행")
    ap.add_argument("--square", type=float, default=0.025, help="칸 한 변 [m]")
    ap.add_argument("--camera-model", choices=["fisheye", "plumb_bob"], default="fisheye")
    ap.add_argument("--mu-pull", nargs="*", default=[], help="견인 시험 힘[N]: 주행면=힘 (예: tile=6.1 carpet=9.4)")
    a = ap.parse_args()

    prof = Profile.load(a.profile) if os.path.exists(a.profile) else Profile()
    lines = []
    if a.records:
        traces = []
        for d in a.records:
            traces += load_run(d)
        print(f"[fit] 시계열 {len(traces)}개 로드")
        prof, lines = fit_run(prof, traces, ref_surface=a.ref_surface, source=", ".join(os.path.basename(os.path.abspath(d)) for d in a.records))
    if a.camera_images:
        from maicon26.calib import camera
        files = sorted(f for pat in a.camera_images for f in glob.glob(pat))
        cols, rows = (int(x) for x in a.board.split("x"))
        objs, imgs, size = camera.detect_corners(files, (cols, rows), a.square)
        res = camera.calibrate(objs, imgs, size, a.camera_model)
        lines += ["### camera (내부 파라미터)"] + camera.apply_to_profile(prof, res, f"체커보드 {len(objs)}장")
    for kv in a.mu_pull:
        surf, n = kv.split("=")
        prof.set(f"drive.surfaces.{surf}.mu_pull_test_N", float(n), "measured", "견인 시험(스프링저울)")
        lines.append(f"- `drive.surfaces.{surf}.mu_pull_test_N` = {n}")
    if os.path.exists(a.profile):
        shutil.copyfile(a.profile, a.profile + ".bak")
    os.makedirs(os.path.dirname(a.profile), exist_ok=True)
    prof.save(a.profile)
    rep_dir = os.path.join(ROOT, "calib", "reports")
    os.makedirs(rep_dir, exist_ok=True)
    rep = os.path.join(rep_dir, datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + "_fit.md")
    with open(rep, "w", encoding="utf-8") as f:
        f.write("# 보정 분석 보고서\n\n입력: " + ", ".join(a.records) + "\n\n" + "\n".join(lines) + "\n")
    print("\n".join(lines))
    left = len(prof.assumed())
    print(f"\n프로파일 갱신: {a.profile}\n보고서: {rep}\n남은 가정 값: {left}개 (scripts/calib_report.py)")


if __name__ == "__main__":
    main()
