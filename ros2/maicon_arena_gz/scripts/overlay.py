#!/usr/bin/env python3
"""Draw SSOT geometry on the supplied floor image; this is not a Gazebo render."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from arena_common import (FACE_DIRS, accessible_faces, expand_boxes, fixed_geometry,
                          load_layout, load_spec, parse_face)

PACKAGE = Path(__file__).resolve().parents[1]


def world_to_pixel(spec: dict, image_size, x: float, y: float, padding=0):
    width, height = spec["floor"]["size"]
    return padding + (x + width / 2) * image_size[0] / width, padding + (height / 2 - y) * image_size[1] / height


def draw_overlay(spec: dict, layout: dict, floor_image: Path, *, show_access=False):
    with Image.open(floor_image) as original:
        floor = original.convert("RGB")
    width, height = floor.size
    pad = max(70, round(width / 20))
    image = Image.new("RGB", (width + pad * 2, height + pad * 2), (235, 235, 235))
    image.paste(floor, (pad, pad))
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", max(16, round(width / 85)))
    except OSError:
        font = ImageFont.load_default()
    thickness = max(2, round(width / 600))
    def point(x, y):
        return world_to_pixel(spec, floor.size, x, y, pad)
    def rectangle(bounds, **kwargs):
        xmin, xmax, ymin, ymax = bounds[:4]
        draw.rectangle([point(xmin, ymax), point(xmax, ymin)], **kwargs)
    for name, bounds in spec["sectors"].items():
        rectangle(bounds, outline=(0, 255, 255), width=thickness)
        draw.text(point(bounds[0] + .03, bounds[3] - .03), name, fill=(0, 255, 255), font=font)
    for geometry in fixed_geometry(spec):
        if geometry["kind"] == "floor":
            continue
        (x, y, _), (sx, sy, _) = geometry["center"], geometry["size"]
        color = (255, 165, 0) if geometry["kind"] == "sensor_box" else (160, 160, 160)
        rectangle((x-sx/2, x+sx/2, y-sy/2, y+sy/2), outline=color, width=thickness)
    for bounds in spec["bases"]["printed_outline"].values():
        rectangle(bounds, outline=(255, 0, 255), width=thickness)
    for x, ya, yb in spec["bases"]["photoelectric_beam"]["segments"].values():
        draw.line([point(x, ya), point(x, yb)], fill=(255, 40, 40), width=2*thickness)
    for name, (x, y, yaw) in spec["bases"]["start_pose"].items():
        start, end = point(x, y), point(x + .2*math.cos(yaw), y + .2*math.sin(yaw))
        draw.line([start, end], fill=(255, 255, 0), width=2*thickness)
        draw.text(start, name, fill=(255, 255, 0), font=font)
    colors = {"m3": (60, 220, 60), "target": (255, 30, 30), "deco": (90, 140, 255),
              "blank": (200, 200, 200), "hidden": None}
    boxes = expand_boxes(spec, layout)
    highest = {}
    for box in boxes:
        key = tuple(box["center"][:2])
        if key not in highest or highest[key]["level"] < box["level"]:
            highest[key] = box
    for box in highest.values():
        x, y, _ = box["center"]
        half = box["size"][0]/2
        rectangle((x-half, x+half, y-half, y+half), fill=(25, 25, 25), outline=(255, 255, 255), width=thickness)
        for face, value in box["faces"].items():
            color = colors[parse_face(value)[0]]
            if color is None:
                continue
            dx, dy = FACE_DIRS[face]
            a = point(x + dx*half - dy*half, y + dy*half + dx*half)
            b = point(x + dx*half + dy*half, y + dy*half - dx*half)
            draw.line([a, b], fill=color, width=3*thickness)
        if box["level"]:
            draw.text(point(x-half/2, y+half/2), f"z{box['level']+1}", fill="white", font=font)
    if show_access:
        access = accessible_faces(spec, layout)
        for box in boxes:
            for face, value in box["faces"].items():
                if parse_face(value)[0] not in ("m3", "target"):
                    continue
                view = access["views"].get(box["name"] + ":" + face)
                if view is None:
                    continue
                vx, vy = point(*view[:2])
                draw.ellipse((vx-thickness, vy-thickness, vx+thickness, vy+thickness), fill=(0, 255, 150))
                draw.line([point(*box["center"][:2]), (vx, vy)], fill=(0, 255, 150), width=max(1, thickness//2))
    draw.text((pad, pad//3), "+X: RED/right    +Y: row 1/up    grid: %.2f m    YAML overlay (not rendered evidence)" % spec["floor"]["grid_cell"], fill="black", font=font)
    legend = "cyan: sector | magenta: base | red: beam/target | green: M3 | blue: decoration | yellow: spawn"
    draw.text((pad, height + pad + pad//3), legend, fill="black", font=font)
    return image


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=PACKAGE / "config/arena_spec.yaml")
    parser.add_argument("--layout", type=Path, default=PACKAGE / "config/layouts/layout_default.yaml")
    parser.add_argument("--floor-image", type=Path)
    parser.add_argument("--output", "-o", type=Path, required=True)
    parser.add_argument("--show-access", action="store_true")
    args = parser.parse_args(argv)
    spec, layout = load_spec(args.spec), load_layout(args.layout)
    floor_image = args.floor_image or PACKAGE / "media" / spec["floor"]["texture"]
    image = draw_overlay(spec, layout, floor_image, show_access=args.show_access)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)
    print(str(args.output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
