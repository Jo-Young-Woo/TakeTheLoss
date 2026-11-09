#!/usr/bin/env python3
"""Create an explicit UAV four-corner calibration; no pixel coordinates are guessed.

Example:
  python scripts/calibrate_uav.py --image-width 1280 --image-height 720 \
    --corners u0 v0 u1 v1 u2 v2 u3 v3 --camera-center 3.06 2.5 2.465 \
    --reference-height 0 --object-height .4 --output uav_calibration.json

Corners correspond in order to arena (0,0), (W,0), (W,H), (0,H).
--checkpoints accepts a JSON list of separate measurements, for example:
  [{"pixel": [640,360], "arena": [3.06,2.5], "plane_z": 0.0}]
Four fitted corners are not independent validation; corner-fit RMS is not an
accuracy estimate. Input pixels must already be lens-undistorted. Camera XYZ
is the optical centre, in the same arena frame as the known corner coordinates.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from maicon26.perception.uav_calibration import FourCornerCalibration


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image-width", type=int, required=True)
    parser.add_argument("--image-height", type=int, required=True)
    parser.add_argument("--corners", nargs=8, type=float, required=True, metavar="PIXEL")
    parser.add_argument("--arena-size", nargs=2, type=float, default=(6.12, 5.0), metavar=("WIDTH_M", "HEIGHT_M"))
    parser.add_argument("--camera-center", nargs=3, type=float, required=True, metavar=("X_M", "Y_M", "Z_M"))
    parser.add_argument("--reference-height", type=float, default=0.0,
                        help="Height of the four selected corners: floor=0, NOT wall-top height")
    parser.add_argument("--object-height", type=float, default=.4,
                        help="Box top plane, used to remove height-dependent parallax")
    parser.add_argument("--checkpoints", type=Path, help="Independent pixel/arena/plane_z measurement list")
    parser.add_argument("--checkpoint-tolerance", type=float, default=.10)
    parser.add_argument("--source", default="manual_labelled_arena_corners")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)
    if args.output.exists() and not args.overwrite:
        parser.error("Output already exists; use another path or --overwrite")
    try:
        checkpoints = json.loads(args.checkpoints.read_text(encoding="utf-8")) if args.checkpoints else []
        if not isinstance(checkpoints, list):
            raise ValueError("Checkpoint JSON must contain a list")
        calibration = FourCornerCalibration(
            (args.image_width, args.image_height), tuple(zip(args.corners[::2], args.corners[1::2])),
            tuple(args.camera_center), *args.arena_size, args.reference_height, args.object_height,
            args.source, tuple(checkpoints), args.checkpoint_tolerance)
        report = calibration.validation_report()
        if report["status"] == "outside_tolerance":
            print(json.dumps(report, indent=2))
            parser.error("Independent checkpoints exceed tolerance; calibration was not saved")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        calibration.save(args.output)
    except (ValueError, TypeError, OSError) as error:
        parser.error(str(error))
    print(json.dumps({"output": str(args.output.resolve()), **report}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
