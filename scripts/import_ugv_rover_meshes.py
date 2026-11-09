"""Copy the selected UGV Rover STL meshes from the user-supplied ROS archive."""
from __future__ import annotations

import argparse
import hashlib
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MESH_NAMES = (
    "base_link.stl",
    "left_up_wheel_link.stl",
    "left_down_wheel_link.stl",
    "right_up_wheel_link.stl",
    "right_down_wheel_link.stl",
    "3d_camera_link.stl",
    "base_lidar_link.stl",
    "pt_base_link.stl",
    "pt_link1.stl",
    "pt_link2.stl",
    "pt_camera_link.stl",
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "ros2" / "gazebo" / "assets" / "ugv_rover")
    args = parser.parse_args()
    with zipfile.ZipFile(args.archive) as archive:
        members = archive.namelist()
        for name in MESH_NAMES:
            suffix = "/ugv_description/meshes/ugv_rover/" + name
            matches = [member for member in members if member.endswith(suffix)]
            if len(matches) != 1:
                raise ValueError(f"Expected one {name} in archive, found {len(matches)}")
            data = archive.read(matches[0])
            if len(data) < 84 or data[80:84] == b"\0\0\0\0":
                raise ValueError(f"Invalid or empty STL: {name}")
            args.output.mkdir(parents=True, exist_ok=True)
            target = args.output / name
            target.write_bytes(data)
            print(f"{target.name}: {len(data)} bytes sha256={hashlib.sha256(data).hexdigest()[:16]}")


if __name__ == "__main__":
    main()
