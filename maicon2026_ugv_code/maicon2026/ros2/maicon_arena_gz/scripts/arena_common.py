"""Shared, ROS-free arena geometry. All world coordinates are floor-centred metres.

``stack`` is a positive layer count; ``level`` is the zero-based bottom layer.
Omitting both means one floor-supported cube. The same side assignments repeat
on each layer; touching side faces are always rendered black by ``expand_boxes``.
This module never imports the legacy positive-coordinate simulation world.
"""
from __future__ import annotations

from collections import Counter, defaultdict, deque
import math
from pathlib import Path
import re
from typing import Any

import yaml

FACE_DIRS = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}
M3_TYPES = ("military_truck", "mortar", "tank", "missile", "ammo_box", "drum")
TARGET_TYPES = ("radar", "comms", "outpost")
DECO_TYPES = ("city_tree", "barrier_wall", "field_bunker", "ruined_building")
EPS = 1e-8


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8-sig") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"YAML root must be a mapping: {Path(path).name}")
    return value


def load_spec(path: str | Path) -> dict[str, Any]:
    spec = load_yaml(path)
    for key in ("floor", "walls", "sectors", "cubes", "bases", "sensor_boxes", "center_pads", "ugv"):
        if key not in spec:
            raise ValueError(f"arena specification is missing {key}")
    return spec


def load_layout(path: str | Path) -> dict[str, Any]:
    layout = load_yaml(path)
    if not isinstance(layout.get("boxes"), list):
        raise ValueError("layout.boxes must be a list")
    return layout


def sector_of(spec: dict, x: float, y: float) -> str | None:
    """Unique interior sector. Shared boundary uses YAML order deterministically.

    Generators require a whole cube inside its labelled sector, so a generated
    cube centre can never depend on the boundary tie convention.
    """
    for name, (xmin, xmax, ymin, ymax) in spec["sectors"].items():
        if xmin - EPS <= x <= xmax + EPS and ymin - EPS <= y <= ymax + EPS:
            return name
    return None


def parse_face(value: str) -> tuple[str, str | None, str | None]:
    if value in ("hidden", "blank"):
        return value, None, None
    head, separator, variant = str(value).partition("@")
    kind, colon, label = head.partition(":")
    choices = {"m3": M3_TYPES, "target": TARGET_TYPES, "deco": DECO_TYPES}
    if not colon or kind not in choices or label not in choices[kind]:
        raise ValueError(f"unsupported face assignment: {value!r}")
    if separator and (not variant or Path(variant).name != variant or "\\" in variant):
        raise ValueError("face variant must be a filename without directories")
    return kind, label, variant if separator else None


def cube_extent(spec: dict, box: dict) -> tuple[float, ...]:
    """Return (xmin,xmax,ymin,ymax,zmin,zmax), raw or expanded cube accepted."""
    s = float(spec["cubes"]["size"])
    center = box["center"]
    if len(center) == 3:
        sx, sy, sz = box.get("size", (s, s, s))
        x, y, z = map(float, center)
    else:
        x, y = map(float, center)
        sx = sy = s
        sz = s * int(box.get("stack", 1))
        z = s * int(box.get("level", 0)) + sz / 2
    return (x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2,
            z - sz / 2, z + sz / 2)


def _positive_integer(value: Any, field: str, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{field} must be an integer >= {minimum}")
    return value


def _raw_expanded(spec: dict, layout: dict) -> list[dict]:
    s = float(spec["cubes"]["size"])
    result = []
    for index, original in enumerate(layout["boxes"]):
        count = _positive_integer(original.get("stack", 1), "stack", 1)
        base_level = _positive_integer(original.get("level", 0), "level", 0)
        x, y = map(float, original["center"])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("cube coordinates must be finite")
        faces = {face: original.get("faces", {}).get(face, "blank") for face in FACE_DIRS}
        for value in faces.values():
            parse_face(value)
        for level in range(base_level, base_level + count):
            result.append({"index": index, "level": level,
                           "name": f"cube_{index}_{level}",
                           "center": [x, y, s * (level + .5)], "size": [s, s, s],
                           "cluster": str(original["cluster"]),
                           "sector": original.get("sector"), "faces": dict(faces)})
    return result


def face_hidden(spec: dict, box: dict, face: str, boxes: list[dict]) -> bool:
    s = float(spec["cubes"]["size"])
    x, y, z = box["center"]
    dx, dy = FACE_DIRS[face]
    for other in boxes:
        ox, oy, oz = other["center"]
        if (abs(ox - x - s * dx) < EPS and abs(oy - y - s * dy) < EPS
                and abs(oz - z) < EPS):
            return True
    width, height = spec["walls"]["inner_size"]
    return abs(x + dx * s / 2) >= width / 2 - EPS or abs(y + dy * s / 2) >= height / 2 - EPS


def expand_boxes(spec: dict, layout: dict) -> list[dict]:
    """Physical layers, with adjacent side faces forced to hidden/black."""
    boxes = _raw_expanded(spec, layout)
    for box in boxes:
        for face in FACE_DIRS:
            if face_hidden(spec, box, face, boxes):
                box["faces"][face] = "hidden"
    return boxes


def rect_gap(a: tuple | list, b: tuple | list) -> float:
    return math.hypot(max(b[0] - a[1], a[0] - b[1], 0.0),
                      max(b[2] - a[3], a[2] - b[3], 0.0))


def fixed_geometry(spec: dict, floor_thickness: float = .02) -> list[dict]:
    """Box geometry with stable names used as the actual SDF collision names.

    Floor thickness .02 m is the user's modelling requirement, not a measured
    arena dimension. The generator passes its explicit profile value here.
    """
    w, h = map(float, spec["floor"]["size"])
    iw, ih = map(float, spec["walls"]["inner_size"])
    t, wh = float(spec["walls"]["thickness"]), float(spec["walls"]["height"])
    result = [{"name": "floor", "kind": "floor", "center": [0., 0., -floor_thickness / 2],
               "size": [w, h, floor_thickness]}]
    for name, center, size in (
        ("wall_north", [0, (ih + t) / 2, wh / 2], [iw + 2 * t, t, wh]),
        ("wall_south", [0, -(ih + t) / 2, wh / 2], [iw + 2 * t, t, wh]),
        ("wall_east", [(iw + t) / 2, 0, wh / 2], [t, ih, wh]),
        ("wall_west", [-(iw + t) / 2, 0, wh / 2], [t, ih, wh]),
    ):
        result.append({"name": name, "kind": "wall", "center": center, "size": size})
    sx, sy, sz = map(float, spec["sensor_boxes"]["size"])
    for side, centers in spec["sensor_boxes"]["centers"].items():
        for index, (x, y) in enumerate(centers):
            result.append({"name": f"sensor_{side}_{index}", "kind": "sensor_box",
                           "center": [x, y, sz / 2], "size": [sx, sy, sz]})
    s = float(spec["cubes"]["size"])
    for index, (x, y) in enumerate(spec["center_pads"]["centers"]):
        result.append({"name": f"center_pad_{index}", "kind": "center_pad",
                       "center": [x, y, s / 2], "size": [s, s, s]})
    return result


def geometry_extent(geometry: dict) -> tuple[float, ...]:
    (x, y, z), (sx, sy, sz) = geometry["center"], geometry["size"]
    return x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2, z - sz / 2, z + sz / 2


def merged_collisions(spec: dict, layout: dict) -> list[dict]:
    """Exact union of cubes; straight runs merge once, an L shape twice.

    Greedy rectangles never fill an L's empty corner or create a mesh collision.
    Clusters are small grid polycubes, so simple deterministic packing is enough.
    """
    s = float(spec["cubes"]["size"])
    groups = defaultdict(list)
    for box in expand_boxes(spec, layout):
        groups[box["cluster"]].append(box)
    result = []
    for name, boxes in groups.items():
        origin = [min(b["center"][i] for b in boxes) for i in range(3)]
        cells = set()
        for box in boxes:
            coordinates = [(box["center"][i] - origin[i]) / s for i in range(3)]
            if any(abs(v - round(v)) > EPS for v in coordinates):
                raise ValueError(f"cluster {name} is not a contiguous cube lattice")
            cells.add(tuple(round(v) for v in coordinates))
        ordinal = 0
        while cells:
            x, y, z = min(cells)
            nx = ny = nz = 1
            while (x + nx, y, z) in cells:
                nx += 1
            while all((xx, y + ny, z) in cells for xx in range(x, x + nx)):
                ny += 1
            while all((xx, yy, z + nz) in cells for xx in range(x, x + nx)
                      for yy in range(y, y + ny)):
                nz += 1
            used = {(xx, yy, zz) for xx in range(x, x + nx) for yy in range(y, y + ny)
                    for zz in range(z, z + nz)}
            cells.difference_update(used)
            safe = re.sub(r"[^a-zA-Z0-9_]", "_", name)
            result.append({"name": f"{safe}_collision_{ordinal}", "cluster": name,
                           "center": [origin[0] + s * (x + (nx - 1) / 2),
                                      origin[1] + s * (y + (ny - 1) / 2),
                                      origin[2] + s * (z + (nz - 1) / 2)],
                           "size": [s * nx, s * ny, s * nz]})
            ordinal += 1
    return result


def recompute_truth(spec: dict, layout: dict) -> dict[str, dict[str, int]]:
    counts = {name: Counter() for name in spec["sectors"]}
    for box in expand_boxes(spec, layout):
        sector = sector_of(spec, *box["center"][:2])
        for value in box["faces"].values():
            kind, label, _ = parse_face(value)
            if kind == "m3" and sector in counts:
                counts[sector][label] += 1
    return {sector: dict(sorted(count.items())) for sector, count in counts.items()}


def target_metadata(spec: dict, layout: dict) -> list[dict]:
    result = []
    s = float(spec["cubes"]["size"])
    receiver = spec["cubes"]["target_receiver"]
    led_size = spec["cubes"]["target_signal_led"]["size"]
    for box in expand_boxes(spec, layout):
        x, y, z = box["center"]
        for face, value in box["faces"].items():
            kind, label, _ = parse_face(value)
            if kind != "target":
                continue
            dx, dy = FACE_DIRS[face]
            receiver_center = [x + dx * s / 2, y + dy * s / 2,
                               z - s / 2 + float(receiver["z"])]
            result.append({"camp": "B" if x < 0 else "R", "center": [x, y],
                           "face": face, "kind": label, "box_id": box["name"],
                           "receiver": receiver_center, "diameter": receiver["diameter"],
                           "led_center": [x, y, z + s / 2 + led_size[2] / 2],
                           "led_size": list(led_size)})
    return result


def targets_summary(spec: dict, layout: dict) -> list[dict]:
    return [{key: target[key] for key in ("camp", "center", "face", "kind")}
            for target in target_metadata(spec, layout)]


def _point_rect_distance(x: float, y: float, rect: tuple) -> float:
    return math.hypot(max(rect[0] - x, x - rect[1], 0), max(rect[2] - y, y - rect[3], 0))


def accessible_faces(spec: dict, layout: dict) -> dict[str, Any]:
    """Conservative planar reachability, not a promise of optical recognition.

    A circumscribed UGV disc plus 1/20 grid-cell clearance must reach a viewing
    pose through the same free-space component as an actual spawn. Grid spacing
    is 1/8 the official floor cell. Faces may be anywhere in their side's clear
    half-space; a short centre-normal ray must remain unobstructed. Camera height
    / pitch visibility of elevated stacks is reported separately by callers.
    """
    w, h = map(float, spec["walls"]["inner_size"])
    grid = float(spec["floor"]["grid_cell"])
    resolution = grid / 8
    length, width = map(float, spec["ugv"]["footprint_lwh"][:2])
    radius = math.hypot(length, width) / 2 + grid / 20
    boxes = expand_boxes(spec, layout)
    fixtures = [geometry_extent(g) for g in fixed_geometry(spec)
                if g["kind"] in ("sensor_box", "center_pad")]
    obstacles = [cube_extent(spec, b) for b in boxes if b["level"] == 0] + fixtures
    nx, ny = round(w / resolution) + 1, round(h / resolution) + 1
    blocked = bytearray(nx * ny)
    # Mark only cells inside each inflated bounding rectangle, with exact disc
    # distance at corners. This avoids an O(cells * obstacles) full arena pass.
    for iy in range(ny):
        y = -h / 2 + iy * resolution
        for ix in range(nx):
            x = -w / 2 + ix * resolution
            if abs(x) > w / 2 - radius + EPS or abs(y) > h / 2 - radius + EPS:
                blocked[iy * nx + ix] = 1
    for rect in obstacles:
        ix0 = max(0, math.floor((rect[0] - radius + w / 2) / resolution))
        ix1 = min(nx - 1, math.ceil((rect[1] + radius + w / 2) / resolution))
        iy0 = max(0, math.floor((rect[2] - radius + h / 2) / resolution))
        iy1 = min(ny - 1, math.ceil((rect[3] + radius + h / 2) / resolution))
        for iy in range(iy0, iy1 + 1):
            for ix in range(ix0, ix1 + 1):
                if _point_rect_distance(-w / 2 + ix * resolution, -h / 2 + iy * resolution, rect) < radius - EPS:
                    blocked[iy * nx + ix] = 1
    def cell(x, y):
        ix, iy = round((x + w / 2) / resolution), round((y + h / 2) / resolution)
        return iy * nx + ix if 0 <= ix < nx and 0 <= iy < ny else None
    reached = bytearray(nx * ny)
    queue = deque()
    blocked_spawns = []
    for name, (x, y, _) in spec["bases"]["start_pose"].items():
        index = cell(x, y)
        if index is None or blocked[index]:
            blocked_spawns.append(name)
        elif not reached[index]:
            reached[index] = 1
            queue.append(index)
    while queue:
        index = queue.popleft()
        ix, iy = index % nx, index // nx
        for j in ((index - 1 if ix else -1), (index + 1 if ix + 1 < nx else -1),
                  (index - nx if iy else -1), (index + nx if iy + 1 < ny else -1)):
            if j >= 0 and not blocked[j] and not reached[j]:
                reached[j] = 1
                queue.append(j)
    views = {}
    for box in boxes:
        x, y, _ = box["center"]
        s = box["size"][0]
        for face, (dx, dy) in FACE_DIRS.items():
            key = f"{box['name']}:{face}"
            if face_hidden(spec, box, face, boxes):
                views[key] = None
                continue
            # Small tangential alternatives avoid a raster-cell coincidence.
            found = None
            for distance in (s / 2 + radius + resolution, s / 2 + radius + 2 * resolution):
                for offset in (0., -s / 4, s / 4):
                    vx, vy = x + dx * distance - dy * offset, y + dy * distance + dx * offset
                    index = cell(vx, vy)
                    if index is not None and reached[index] and all(
                            _point_rect_distance(vx, vy, r) >= radius - EPS for r in obstacles):
                        found = [round(vx, 6), round(vy, 6), math.atan2(-dy, -dx)]
                        break
                if found:
                    break
            views[key] = found
    return {"views": views, "blocked_spawns": blocked_spawns,
            "robot_radius": radius, "resolution": resolution,
            "reachable_cells": sum(reached)}


def validate_layout(spec: dict, layout: dict, *, require_all_sectors: bool = True,
                    require_targets: bool = True, check_truth: bool = True,
                    check_accessibility: bool = True) -> dict[str, Any]:
    errors, warnings = [], []
    try:
        raw = _raw_expanded(spec, layout)
        boxes = expand_boxes(spec, layout)
    except (KeyError, ValueError, TypeError) as exc:
        return {"ok": False, "errors": [str(exc)], "warnings": [], "metrics": {}}
    s = float(spec["cubes"]["size"])
    grid = float(spec["floor"]["grid_cell"])
    clearance = float(spec["cubes"]["min_clearance"])
    w, h = spec["walls"]["inner_size"]
    extents = [cube_extent(spec, b) for b in boxes]
    fixtures = [g for g in fixed_geometry(spec) if g["kind"] in ("sensor_box", "center_pad")]
    present = set()
    for index, (box, extent) in enumerate(zip(boxes, extents)):
        x, y, _ = box["center"]
        present.add(box["sector"])
        if any(abs(v / grid - round(v / grid)) > EPS for v in (x, y)):
            errors.append(f"{box['name']}: centre is not on the {grid:g} m grid")
        sector = sector_of(spec, x, y)
        if sector != box["sector"]:
            errors.append(f"{box['name']}: sector {box['sector']} differs from coordinate sector {sector}")
        if sector is not None:
            a, b, c, d = spec["sectors"][sector]
            if extent[0] < a - EPS or extent[1] > b + EPS or extent[2] < c - EPS or extent[3] > d + EPS:
                errors.append(f"{box['name']}: cube footprint crosses its sector")
        wall_gap = min(extent[0] + w / 2, w / 2 - extent[1], extent[2] + h / 2, h / 2 - extent[3])
        if wall_gap < clearance - EPS:
            errors.append(f"{box['name']}: wall clearance {wall_gap:.6f} < {clearance:g}")
        for fixture in fixtures:
            gap = rect_gap(extent, geometry_extent(fixture))
            if gap < clearance - EPS:
                errors.append(f"{box['name']}: {fixture['name']} clearance {gap:.6f} < {clearance:g}")
        if box["level"] and not any(abs(o["center"][0] - x) < EPS and abs(o["center"][1] - y) < EPS
                                     and o["level"] == box["level"] - 1 for o in boxes):
            errors.append(f"{box['name']}: unsupported stack level")
        for other, other_extent in zip(boxes[index + 1:], extents[index + 1:]):
            vertical_overlap = min(extent[5], other_extent[5]) - max(extent[4], other_extent[4]) > EPS
            xy_overlap = min(extent[1], other_extent[1]) - max(extent[0], other_extent[0]) > EPS and min(extent[3], other_extent[3]) - max(extent[2], other_extent[2]) > EPS
            if vertical_overlap and xy_overlap:
                errors.append(f"{box['name']} overlaps {other['name']}")
            if box["cluster"] != other["cluster"] and rect_gap(extent, other_extent) < clearance - EPS:
                errors.append(f"{box['name']}~{other['name']}: inter-cluster clearance < {clearance:g}")
        for face, value in raw[index]["faces"].items():
            hidden = face_hidden(spec, box, face, boxes)
            if hidden and value not in ("hidden", "blank"):
                errors.append(f"{box['name']}:{face}: nonblank image on a hidden face")
            if not hidden and value == "hidden":
                errors.append(f"{box['name']}:{face}: hidden label on exposed face")
    # A cluster name cannot exempt separated blocks from the clearance rule.
    groups = defaultdict(list)
    for b in boxes:
        groups[b["cluster"]].append(b)
    for name, members in groups.items():
        reached = {0}
        pending = [0]
        while pending:
            a = members[pending.pop()]
            for j, b in enumerate(members):
                delta = [abs(a["center"][k] - b["center"][k]) for k in range(3)]
                if j not in reached and sum(abs(v - s) < EPS for v in delta) == 1 and sum(v < EPS for v in delta) == 2:
                    reached.add(j)
                    pending.append(j)
        if len(reached) != len(members):
            errors.append(f"cluster {name}: disconnected cubes cannot share one cluster name")
    if require_all_sectors and present != set(spec["sectors"]):
        errors.append(f"layout must populate all seven sectors; missing {sorted(set(spec['sectors']) - present)}")
    truth = recompute_truth(spec, layout)
    if check_truth and layout.get("m3_ground_truth") != truth:
        errors.append("m3_ground_truth differs from actual exposed side images")
    targets = targets_summary(spec, layout)
    if require_targets:
        for camp in ("B", "R"):
            actual = [t for t in targets if t["camp"] == camp]
            if len(actual) != 3 or Counter(t["kind"] for t in actual) != Counter(TARGET_TYPES):
                errors.append(f"camp {camp} must contain exactly three distinct targets")
        mirror_face = {"N": "N", "S": "S", "E": "W", "W": "E"}
        for target in (t for t in targets if t["camp"] == "B"):
            if not any(t["camp"] == "R" and t["kind"] == target["kind"] and t["face"] == mirror_face[target["face"]]
                       and abs(t["center"][0] + target["center"][0]) < EPS
                       and abs(t["center"][1] - target["center"][1]) < EPS for t in targets):
                errors.append("target positions/faces/types must be mirrored across X=0")
        def target_key(t):
            return (t["camp"], tuple(t["center"]), t["face"], t["kind"])
        if sorted(map(target_key, layout.get("targets_summary", []))) != sorted(map(target_key, targets)):
            errors.append("targets_summary differs from actual side images")
    access = None
    if check_accessibility:
        access = accessible_faces(spec, layout)
        errors.extend(f"spawn {name}: no free robot footprint" for name in access["blocked_spawns"])
        for box in boxes:
            for face, value in box["faces"].items():
                if parse_face(value)[0] in ("m3", "target") and access["views"][f"{box['name']}:{face}"] is None:
                    errors.append(f"{box['name']}:{face}: image has no reachable robot viewing pose")
        if any(b["level"] for b in boxes):
            warnings.append("Stacked side images need camera-height/pitch validation; planar reachability alone does not prove optical visibility.")
    return {"ok": not errors, "errors": errors, "warnings": warnings,
            "metrics": {"physical_cubes": len(boxes), "clusters": len(groups),
                        "targets": len(targets), "m3_faces": sum(sum(v.values()) for v in truth.values())},
            "accessibility": access}
