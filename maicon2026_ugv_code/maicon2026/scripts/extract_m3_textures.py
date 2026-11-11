"""Extract the six supplied PDF p13 example photos for M3 box side panels.

These are practice examples, not a claim that final competition photos match.
Uses Poppler pdfimages and Pillow. Keeps the source document unchanged.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import tempfile

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
CLASSES = ("AMMO_BOX", "DRUM", "MORTAR", "TRUCK", "TANK", "MISSILE")


def extract(source: Path, output: Path):
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="maicon_m3_") as directory:
        prefix = Path(directory) / "picture"
        subprocess.run(["pdfimages", "-f", "13", "-l", "13", "-j", str(source), str(prefix)], check=True)
        rows = []
        for path in sorted(Path(directory).glob("*.jpg")):
            im = Image.open(path).convert("RGB")
            if 900 <= im.width <= 930 and 280 <= im.height <= 310:
                rows.append(im)
        if len(rows) != 2:
            raise ValueError("Expected the two 3-photo rows on first-training PDF p13")
        for index, cls in enumerate(CLASSES):
            row = rows[index // 3]
            # Embedded rows contain white separators around x=300 and x=610.
            x0, x1 = ((0, 294), (310, 604), (620, row.width))[index % 3]
            photo = row.crop((x0, 1, x1, row.height)).resize(
                (512, 512), getattr(Image, "Resampling", Image).LANCZOS)
            photo.save(output / f"{cls}.png")
            (output / f"{cls}.mtl").write_text(
                f"newmtl photo\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 1\nmap_Kd {cls}.png\n", encoding="utf-8")
            # Normal +X; rotate around Z for north/west/south. One visual per side.
            (output / f"{cls}.obj").write_text(
                f"mtllib {cls}.mtl\no {cls}\n"
                "v 0 -0.18 -0.18\nv 0 0.18 -0.18\nv 0 0.18 0.18\nv 0 -0.18 0.18\n"
                "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 1 0 0\nusemtl photo\n"
                "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n", encoding="utf-8")
    (output / "source.json").write_text(json.dumps({
        "source": source.name, "page": 13, "classes": CLASSES,
        "purpose": "M3 practice side panels extracted from the user-supplied first-training PDF",
        "notice": "The PDF says actual competition images can differ; these are not a training dataset or unseen-test claim.",
        "geometry_m": [0.4, 0.4, 0.4], "panel_m": [0.36, 0.36],
    }, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pdf", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "ros2/gazebo/assets/m3_photos")
    args = parser.parse_args()
    extract(args.pdf, args.output)
