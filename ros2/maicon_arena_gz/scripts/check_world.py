#!/usr/bin/env python3
"""Check actual SDF geometry/resources against YAML, without starting Gazebo.

This is acceptance A, not a replacement for rendered-image or real-time-factor
measurements. A metadata JSON supplied by the generator is deliberately unused.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from PIL import Image

from arena_common import (expand_boxes, fixed_geometry, load_layout, load_spec,
                          merged_collisions, target_metadata, validate_layout)

PACKAGE = Path(__file__).resolve().parents[1]
IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


def _numbers(text, count, default=None):
    values = list(default) if text is None and default is not None else [float(v) for v in (text or "").split()]
    if len(values) != count or any(not math.isfinite(v) for v in values):
        raise ValueError(f"expected {count} finite numbers, got {text!r}")
    return values


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _matvec(a, b):
    return [sum(a[i][k] * b[k] for k in range(3)) for i in range(3)]


def _pose(node):
    pose = node.find("pose")
    if pose is not None and pose.get("relative_to"):
        raise ValueError("relative_to poses require frame resolution; this checker refuses an unchecked frame")
    x, y, z, roll, pitch, yaw = _numbers(node.findtext("pose"), 6, (0,) * 6)
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return ([x, y, z], ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                       (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                       (-sp, cp * sr, cp * cr)))


def _compose(parent, local):
    position, rotation = parent
    moved = _matvec(rotation, local[0])
    return [position[i] + moved[i] for i in range(3)], _matmul(rotation, local[1])


def resolve_resource(uri, package_root=PACKAGE):
    """Resolve only the local package. No remote requests or Fuel lookups."""
    root = Path(package_root).resolve()
    prefix = "model://maicon_arena_gz/"
    package_prefix = "package://maicon_arena_gz/"
    if uri.startswith(prefix):
        path = root / uri[len(prefix):]
    elif uri.startswith(package_prefix):
        path = root / uri[len(package_prefix):]
    elif uri.startswith("file://"):
        path = Path(uri[len("file://"):])
    elif "://" in uri:
        raise ValueError(f"nonlocal resource is prohibited: {uri}")
    else:
        path = root / uri
    path = path.resolve()
    if not path.is_relative_to(root) or not path.exists():
        raise ValueError(f"missing or external package resource: {uri}")
    return path


def _mesh_bounds(mesh, package_root):
    path = resolve_resource(mesh.findtext("uri", ""), package_root)
    if path.suffix.lower() != ".obj":
        raise ValueError("static visual geometry must use the inspectable shared OBJ mesh")
    points = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("v "):
            points.append(_numbers(line[2:], 3))
    if not points:
        raise ValueError(f"mesh has no vertices: {path.name}")
    scale = _numbers(mesh.findtext("scale"), 3, (1, 1, 1))
    if any(v <= 0 for v in scale):
        raise ValueError("mesh scale must be positive (mirroring is not permitted)")
    low = [min(v[i] for v in points) * scale[i] for i in range(3)]
    high = [max(v[i] for v in points) * scale[i] for i in range(3)]
    return [(low[i] + high[i]) / 2 for i in range(3)], [high[i] - low[i] for i in range(3)]


def _geometry_record(node, parent_pose, package_root):
    world_pose = _compose(parent_pose, _pose(node))
    geometry = node.find("geometry")
    if geometry is None:
        raise ValueError("geometry is absent")
    box, mesh = geometry.find("box"), geometry.find("mesh")
    if box is not None:
        center, size = [0, 0, 0], _numbers(box.findtext("size"), 3)
        kind = "box"
    elif mesh is not None:
        center, size = _mesh_bounds(mesh, package_root)
        kind = "mesh"
    else:
        return {"name": node.get("name"), "center": world_pose[0], "size": None,
                "rotation": world_pose[1], "kind": "other", "node": node}
    offset = _matvec(world_pose[1], center)
    return {"name": node.get("name"), "center": [world_pose[0][i] + offset[i] for i in range(3)],
            "size": size, "rotation": world_pose[1], "kind": kind, "node": node}


def check_world(spec: dict, layout: dict, sdf, *, package_root=PACKAGE,
                profile: dict | None = None, tolerance: float = .001) -> dict:
    """Check XML/file/ElementTree. Report failures instead of trusting sidecars."""
    errors, warnings = [], []
    layout_report = validate_layout(spec, layout)
    errors.extend(layout_report["errors"])
    warnings.extend(layout_report["warnings"])
    if isinstance(sdf, ET.ElementTree):
        root = sdf.getroot()
    elif isinstance(sdf, ET.Element):
        root = sdf
    else:
        root = ET.parse(sdf).getroot()
    world = root.find("world")
    if world is None:
        return {"ok": False, "errors": ["SDF has no world"], "warnings": warnings, "counts": {}}
    records = {"collision": [], "visual": []}
    static_models = []
    texture_paths = set()
    for model in world.findall("model"):
        if model.findtext("static", "false").strip().lower() != "true":
            if model.get("name") in ("floor", "arena", "layout", "uav_camera", "validation_camera"):
                errors.append(f"model {model.get('name')} must be static")
            continue  # UGV collision/visual budgets are explicitly excluded.
        static_models.append(model)
        if model.find("model") is not None:
            errors.append(f"model {model.get('name')}: nested static model requires explicit verification")
        try:
            model_pose = _pose(model)
            for link in model.findall("link"):
                link_pose = _compose(model_pose, _pose(link))
                for tag in records:
                    for node in link.findall(tag):
                        try:
                            record = _geometry_record(node, link_pose, package_root)
                            record["model"] = model.get("name")
                            records[tag].append(record)
                            if tag == "collision" and record["kind"] != "box":
                                errors.append(f"collision {record['name']}: only primitive boxes are allowed")
                        except (ValueError, OSError) as exc:
                            errors.append(f"{tag} {node.get('name')}: {exc}")
        except ValueError as exc:
            errors.append(f"model {model.get('name')}: {exc}")
        for node in model.findall(".//uri"):
            try:
                resolve_resource(node.text or "", package_root)
            except ValueError as exc:
                errors.append(str(exc))
        for tag in ("albedo_map", "normal_map", "roughness_map", "metalness_map", "diffuse_map"):
            for node in model.findall(".//" + tag):
                try:
                    texture_paths.add(resolve_resource(node.text or "", package_root))
                except ValueError as exc:
                    errors.append(str(exc))
        # Ogre 1 consumes material scripts while Ogre 2 consumes albedo_map.
        # Inspect both paths so a modified fallback material is not unchecked.
        for material in model.findall(".//visual/material"):
            script = material.find("script")
            if script is None:
                continue
            try:
                directories = [resolve_resource(n.text or "", package_root) for n in script.findall("uri")]
                name = script.findtext("name", "")
                sources = [p / (name + ".material") for p in directories if p.is_dir() and (p / (name + ".material")).is_file()]
                if len(sources) != 1:
                    raise ValueError(f"material {name}: expected one inspectable local material script")
                texture_names = re.findall(r"^\s*texture\s+(\S+)\s*$", sources[0].read_text(encoding="utf-8"), re.MULTILINE)
                if not texture_names:
                    raise ValueError(f"material {name}: texture assignment missing")
                selected = set()
                for texture_name in texture_names:
                    paths = [p / texture_name for p in directories if p.is_dir() and (p / texture_name).is_file()]
                    if len(paths) != 1:
                        raise ValueError(f"material {name}: texture cannot be resolved unambiguously")
                    selected.add(resolve_resource(str(paths[0].resolve()), package_root))
                texture_paths.update(selected)
                albedo = material.findtext("pbr/metal/albedo_map")
                if albedo and selected != {resolve_resource(albedo, package_root)}:
                    errors.append(f"material {name}: Ogre 1 texture differs from Ogre 2 albedo")
            except (ValueError, OSError) as exc:
                errors.append(str(exc))
    # Count XML nodes, including malformed records, so malformed content cannot
    # accidentally make a world appear to meet a complexity budget.
    counts = {tag + "s": sum(len(m.findall(".//" + tag)) for m in static_models) for tag in records}
    counts["textures"] = len(texture_paths)
    counts["lights"] = len(world.findall(".//light"))
    counts["sensors"] = dict(Counter(n.get("type") for n in world.findall(".//sensor")))
    for key, limit in (("collisions", 40), ("visuals", 80), ("textures", 30)):
        if counts[key] > limit:
            errors.append(f"{key} budget exceeded: {counts[key]} > {limit}")
    if counts["lights"] != 1 or any(n.get("type") != "directional" for n in world.findall(".//light")):
        errors.append("exactly one directional light is required")
    for texture in sorted(texture_paths):
        try:
            with Image.open(texture) as image:
                if max(image.size) > 2560:
                    errors.append(f"texture {texture.name}: longest side exceeds 2560 px")
        except OSError:
            errors.append(f"texture {texture.name}: cannot read image")
    thickness = .02 if profile is None else profile["geometry"]["floor_thickness"]
    fixed = fixed_geometry(spec, floor_thickness=thickness)
    expected_collisions = fixed + merged_collisions(spec, layout)
    expected_visuals = [{**g, "name": g["name"] + "_visual"} for g in fixed if g["kind"] != "floor"] + expand_boxes(spec, layout)
    expected_visuals.append({"name": "floor_print", "center": [0, 0, 0], "size": [*spec["floor"]["size"], 0]})
    for target in target_metadata(spec, layout):
        expected_visuals.append({"name": target["box_id"] + "_signal", "center": target["led_center"], "size": target["led_size"]})
    def compare(tag, expected, exact_names=False):
        actual = {}
        for record in records[tag]:
            if record["name"] in actual:
                errors.append(f"{tag} name is not unique: {record['name']}")
            actual[record["name"]] = record
        names = {g["name"] for g in expected}
        if exact_names:
            errors.extend(f"unexpected {tag}: {name}" for name in sorted(set(actual) - names))
        for geometry in expected:
            name = geometry["name"]
            observed = actual.get(name)
            if observed is None:
                errors.append(f"missing {tag}: {name}")
                continue
            for field in ("center", "size"):
                if observed[field] is None or any(abs(a - b) > tolerance + 1e-9 for a, b in zip(observed[field], geometry[field])):
                    errors.append(f"{tag} {name}: {field} differs from YAML by more than {tolerance:g} m")
            if any(abs(observed["rotation"][i][j] - IDENTITY[i][j]) > 1e-7 for i in range(3) for j in range(3)):
                errors.append(f"{tag} {name}: rotated geometry/face orientation differs from YAML")
    compare("collision", expected_collisions, exact_names=True)
    compare("visual", expected_visuals)
    if world.find("include") is not None:
        errors.append("world includes are not allowed: resources must be generated locally")
    if world.find("scene/sky") is not None or world.find("scene/fog") is not None:
        errors.append("default sky/fog must be absent")
    return {"ok": not errors, "errors": errors, "warnings": warnings,
            "counts": counts, "layout": layout_report["metrics"],
            "tolerance_m": tolerance, "checked": "actual SDF XML + actual mesh/image resources; no metadata sidecar"}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=PACKAGE / "config/arena_spec.yaml")
    parser.add_argument("--layout", type=Path, default=PACKAGE / "config/layouts/layout_default.yaml")
    parser.add_argument("--world", type=Path, default=PACKAGE / "worlds/maicon_arena.sdf")
    parser.add_argument("--profile", default="standard")
    parser.add_argument("--json", type=Path, help="optional machine-readable acceptance report")
    args = parser.parse_args(argv)
    from gen_world import load_profile
    report = check_world(load_spec(args.spec), load_layout(args.layout), args.world,
                         profile=load_profile(args.profile))
    result = json.dumps(report, ensure_ascii=False, indent=2)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(result + "\n", encoding="utf-8")
    print(result)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
