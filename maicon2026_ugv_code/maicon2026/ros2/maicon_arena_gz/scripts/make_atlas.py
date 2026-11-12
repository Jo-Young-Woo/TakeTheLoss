#!/usr/bin/env python3
"""Bake supplied face photographs into shared NESW+black cube atlases.

Images are copied without flips/rotations. UV coordinates in cube.obj make each
side upright when viewed from outside, with top = +Z. No external image service
or network is used. Source image bytes participate in the content hash, so
replacing a provisional photograph invalidates the generated atlas cache.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image


FACES = ("N", "E", "S", "W")
NORMALS = {"N": (0, 1, 0), "E": (1, 0, 0), "S": (0, -1, 0), "W": (-1, 0, 0)}
# Bottom-left, bottom-right, top-right, top-left AS SEEN FROM OUTSIDE.
FACE_VERTICES = {
    "N": ((.5, .5, -.5), (-.5, .5, -.5), (-.5, .5, .5), (.5, .5, .5)),
    "E": ((.5, -.5, -.5), (.5, .5, -.5), (.5, .5, .5), (.5, -.5, .5)),
    "S": ((-.5, -.5, -.5), (.5, -.5, -.5), (.5, -.5, .5), (-.5, -.5, .5)),
    "W": ((-.5, .5, -.5), (-.5, -.5, -.5), (-.5, -.5, .5), (-.5, .5, .5)),
    "top": ((-.5, -.5, .5), (.5, -.5, .5), (.5, .5, .5), (-.5, .5, .5)),
    "bottom": ((.5, -.5, -.5), (-.5, -.5, -.5), (-.5, .5, -.5), (.5, .5, -.5)),
}


def _local_image(texture_root, value):
    root = Path(texture_root).resolve()
    path = (root / value).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError("Face image must be an existing package-local file")
    return path


def image_for_face(value, texture_root):
    if value in (None, "hidden", "blank", "black"):
        return None
    if value == "hazard":
        return _local_image(texture_root, "hazard_stripe_256.png")
    if not isinstance(value, str) or ":" not in value:
        raise ValueError("Face must be blank/hidden or a m3/target/deco image identifier")
    label, _, variant = value.partition("@")
    category, kind = label.split(":", 1)
    if category not in ("m3", "target", "deco") or not kind or "/" in kind or "\\" in kind:
        raise ValueError("Unknown face image category or class")
    relative = f"faces/{category}_{kind}/{variant}" if variant else f"faces/{category}_{kind}.jpg"
    return _local_image(texture_root, relative)


def make_atlas(faces, texture_root, output_dir, tile_px=512):
    """Return one content-addressed PNG path for the effective four side images."""
    if not isinstance(tile_px, int) or tile_px <= 0 or tile_px * 5 > 2560:
        raise ValueError("Atlas tile size must be positive and total width <= 2560")
    atlas = Image.new("RGB", (tile_px * 5, tile_px), (0, 0, 0))
    for index, face in enumerate(FACES):
        image_path = image_for_face(faces.get(face, "blank"), texture_root)
        if image_path is not None:
            with Image.open(image_path) as source:
                image = source.convert("RGB").resize((tile_px, tile_px), getattr(Image, "Resampling", Image).LANCZOS)
            atlas.paste(image, (index * tile_px, 0))
    digest = hashlib.sha256(str(atlas.size).encode("ascii") + atlas.tobytes()).hexdigest()[:24]
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    destination = output_dir / f"atlas_{digest}.png"
    if not destination.exists():
        atlas.save(destination, optimize=True)
    return destination


def cube_obj():
    """A unit cube: one mesh, six outward faces, UV slots [N E S W black]."""
    lines = ["# Unit cube; side images upright from outside; scale comes from arena YAML.", "o cube_atlas"]
    all_normals = {**NORMALS, "top": (0, 0, 1), "bottom": (0, 0, -1)}
    # Fixed fractional gutter keeps bilinear filtering away from adjacent tiles.
    # It is a UV parameter, not a physical dimension: half a pixel at lite size.
    gutter = .5 / 256
    for face_index, (face, vertices) in enumerate(FACE_VERTICES.items()):
        slot = face_index if face_index < 4 else 4
        u0, u1 = (slot + gutter)/5, (slot + 1 - gutter)/5
        for vertex in vertices:
            lines.append("v " + " ".join(map(str, vertex)))
        for uv in ((u0, gutter), (u1, gutter), (u1, 1-gutter), (u0, 1-gutter)):
            lines.append("vt %.9f %.9f" % uv)
        lines.append("vn " + " ".join(map(str, all_normals[face])))
        first = face_index * 4 + 1
        normal = face_index + 1
        for indices in ((first, first+1, first+2), (first, first+2, first+3)):
            lines.append("f " + " ".join(f"{i}/{i}/{normal}" for i in indices))
    return "\n".join(lines) + "\n"


def floor_obj(print_fraction=(1.0, 1.0)):
    # Pixel (0,0) -> world(-width/2,+height/2); OBJ v=1 is image top.
    fx, fy = map(float, print_fraction)
    if not (0 < fx <= 1 and 0 < fy <= 1):
        raise ValueError("floor print must fit inside the floor")
    if (fx, fy) != (1., 1.):
        # Keep the original 20 cm grid and base outlines at their metric size.
        # Extend only edge texels into the new margins; still one floor visual.
        xs, ys = (-.5, -fx/2, fx/2, .5), (-.5, -fy/2, fy/2, .5)
        uv = (1e-6, 1e-6, 1-1e-6, 1-1e-6)
        lines = ["# Unit floor with a centred unscaled print and edge margins", "o floor", "vn 0 0 1"]
        for j, y in enumerate(ys):
            for i, x in enumerate(xs):
                lines += [f"v {x:.12g} {y:.12g} 0", f"vt {uv[i]} {uv[j]}"]
        for j in range(3):
            for i in range(3):
                if xs[i] == xs[i+1] or ys[j] == ys[j+1]:
                    continue
                a = 1+j*4+i
                for triangle in ((a, a+1, a+5), (a, a+5, a+4)):
                    lines.append("f " + " ".join(f"{k}/{k}/1" for k in triangle))
        return "\n".join(lines)+"\n"
    return ("# Unit floor rectangle; scale comes from arena YAML.\no floor\n"
            "v -0.5 -0.5 0\nv 0.5 -0.5 0\nv 0.5 0.5 0\nv -0.5 0.5 0\n"
            "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n"
            "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")


def write_shared_meshes(package_root, print_fraction=(1.0, 1.0)):
    package_root = Path(package_root)
    paths = {}
    floor_text = floor_obj(print_fraction)
    floor_name = "floor" if tuple(print_fraction) == (1., 1.) else "floor_"+hashlib.sha256(floor_text.encode()).hexdigest()[:12]
    for key, relative, text in (("cube", "models/cube_atlas/meshes/cube.obj", cube_obj()),
                                ("floor", f"models/floor/meshes/{floor_name}.obj", floor_text)):
        path = package_root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        paths[key] = path
    return paths


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--faces-json", required=True, help='JSON object containing N/E/S/W face labels')
    parser.add_argument("--texture-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--tile-px", type=int, default=512)
    args = parser.parse_args(argv)
    print(make_atlas(json.loads(args.faces_json), args.texture_root, args.output_dir, args.tile_px))


if __name__ == "__main__":
    main()
