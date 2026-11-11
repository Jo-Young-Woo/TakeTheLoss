#!/usr/bin/env python3
"""Refine manually labelled visible floor-grid crossings in an actual image.

Seeds identify physical grid intersections with known metric coordinates.
Final pixels come from image intensity ridges, never a fitted homography.
Occluded or weak crossings are rejected rather than synthesized.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def refine_cross(image, seed):
    x, y = map(lambda v: int(round(v)), seed)
    radius = 6
    if x < radius or y < radius or x + radius >= image.shape[1] or y + radius >= image.shape[0]:
        raise ValueError("seed outside measurable image area")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(float)
    patch = gray[y-radius:y+radius+1, x-radius:x+radius+1]
    positions = []
    contrasts = []
    for profile in (np.median(patch, axis=0), np.median(patch, axis=1)):
        background = float(np.percentile(profile, 25))
        peak = int(np.argmax(profile[2:-2])) + 2
        contrast = float(profile[peak] - background)
        if contrast < 6.:
            raise ValueError("crossing hidden or insufficient grid contrast")
        indices = np.arange(max(0, peak-1), min(len(profile), peak+2))
        weights = np.maximum(profile[indices] - background, 0.)
        positions.append(float(np.sum(indices*weights)/np.sum(weights))-radius)
        contrasts.append(contrast)
    return [x+positions[0], y+positions[1]], contrasts


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--seeds", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error("image could not be decoded")
    seeds = json.loads(args.seeds.read_text(encoding="utf-8"))
    measured, rejected = [], []
    for item in seeds:
        try:
            pixel, contrast = refine_cross(image, item["pixel_seed"])
            measured.append({**item, "pixel": pixel, "contrast_gray": contrast,
                             "source": "image_intensity_ridge_of_manually_identified_grid_crossing"})
        except ValueError as error:
            rejected.append({**item, "rejection": str(error)})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(measured, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    args.output.with_suffix(".rejected.json").write_text(json.dumps(rejected, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"measured": len(measured), "rejected": len(rejected), "output": str(args.output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
