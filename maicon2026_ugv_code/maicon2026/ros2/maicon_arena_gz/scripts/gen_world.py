#!/usr/bin/env python3
"""Generate the measured MAICON arena from the exact supplied YAML inputs.

No arena positions live in this module. Physical dimensions are read from the
specification; additional rendering/physics assumptions are explicit profiles.
All resources use package-local model:// URIs and require no Fuel connection.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from arena_common import expand_boxes, fixed_geometry, load_layout, load_spec, merged_collisions, validate_layout
from make_atlas import FACES, NORMALS, make_atlas, write_shared_meshes


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
# Every Gazebo system filename/class is centralized for Fortress -> Harmonic.
_SYSTEM_SLUGS = {
    "Physics": "physics", "UserCommands": "user-commands", "SceneBroadcaster": "scene-broadcaster",
    "Sensors": "sensors", "Imu": "imu", "DiffDrive": "diff-drive",
    "JointPositionController": "joint-position-controller", "VelocityControl": "velocity-control",
    "PosePublisher": "pose-publisher", "JointStatePublisher": "joint-state-publisher",
}
SYSTEMS = {
    simulator: {name: (f"{prefix}-{slug}-system", f"{namespace}::{name}") for name, slug in _SYSTEM_SLUGS.items()}
    for simulator, prefix, namespace in (("fortress", "ignition-gazebo", "ignition::gazebo::systems"),
                                         ("harmonic", "gz-sim", "gz::sim::systems"))
}
for _systems in SYSTEMS.values():
    _systems["OpaqueMaterial"] = ("maicon_opaque_material_system", "maicon::OpaqueMaterialSystem")
PHYSICS_ENGINE = {"fortress": "libignition-physics-dartsim-plugin.so", "harmonic": "libgz-physics-dartsim-plugin.so"}


def sub(parent, tag, value=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if value is not None:
        node.text = str(value).lower() if isinstance(value, bool) else str(value)
    return node


def vector(values):
    return " ".join(format(float(value), ".10g") for value in values)


def add_system(parent, key, simulator="fortress"):
    filename, name = SYSTEMS[simulator][key]
    return sub(parent, "plugin", filename=filename, name=name)


def resource_uri(path, package_root):
    relative = Path(path).resolve().relative_to(Path(package_root).resolve()).as_posix()
    return "model://maicon_arena_gz/" + relative


def static_model(world, name):
    model = sub(world, "model", name=name)
    sub(model, "static", True)
    return model


def material(visual, color, texture=None, package_root=None, emissive=False):
    mat = sub(visual, "material")
    sub(mat, "ambient", vector(color))
    sub(mat, "diffuse", vector(color))
    sub(mat, "specular", "0 0 0 1")
    if emissive:
        sub(mat, "emissive", vector(color))
    if texture is not None:
        pbr = sub(mat, "pbr")
        metal = sub(pbr, "metal")
        sub(metal, "albedo_map", resource_uri(texture, package_root))
        sub(metal, "metalness", 0)
        sub(metal, "roughness", 1)
        # Ogre 1 uses the material script; Ogre 2 uses the PBR albedo above.
        # Each script references the identical texture, not a duplicate image.
        texture = Path(texture)
        key = "maicon_" + hashlib.sha256(texture.relative_to(package_root).as_posix().encode()).hexdigest()[:20]
        script_dir = Path(package_root) / "media/materials/scripts"
        script_dir.mkdir(parents=True, exist_ok=True)
        script_file = script_dir / (key + ".material")
        script_file.write_text(
            f"material {key}\n{{\n technique\n {{\n  pass\n  {{\n   ambient 1 1 1\n   diffuse 1 1 1\n"
            f"   specular 0 0 0 0\n   texture_unit\n   {{\n    texture {texture.name}\n    filtering bilinear\n   }}\n  }}\n }}\n}}\n",
            encoding="utf-8")
        script = sub(mat, "script")
        sub(script, "uri", resource_uri(script_dir, package_root))
        sub(script, "uri", resource_uri(texture.parent, package_root))
        sub(script, "name", key)
    return mat


def box(link, name, center, size, *, collide=True, color=None, mesh=None, texture=None, package_root=None, emissive=False):
    if collide:
        collision = sub(link, "collision", name=name)
        sub(collision, "pose", vector([*center, 0, 0, 0]))
        sub(sub(sub(collision, "geometry"), "box"), "size", vector(size))
    if color is None:
        return None
    visual = sub(link, "visual", name=name + "_visual" if collide else name)
    sub(visual, "pose", vector([*center, 0, 0, 0]))
    geometry = sub(visual, "geometry")
    if mesh is not None:
        geom = sub(geometry, "mesh")
        sub(geom, "uri", resource_uri(mesh, package_root))
        sub(geom, "scale", vector(size))
    else:
        sub(sub(geometry, "box"), "size", vector(size))
    material(visual, color, texture, package_root, emissive)
    return visual


def cylinder_visual(link, name, center, radius, length, color, emissive=False):
    visual = sub(link, "visual", name=name)
    sub(visual, "pose", vector([*center, 0, 0, 0]))
    cylinder = sub(sub(visual, "geometry"), "cylinder")
    sub(cylinder, "radius", radius)
    sub(cylinder, "length", length)
    material(visual, color, emissive=emissive)


def _camera(world, name, pose, resolution, hfov, update_rate, clip, topic):
    model = static_model(world, name)
    sub(model, "pose", vector(pose))
    link = sub(model, "link", name="camera_link")
    sensor = sub(link, "sensor", name="camera", type="camera")
    sub(sensor, "always_on", True)
    sub(sensor, "update_rate", update_rate)
    sub(sensor, "visualize", False)
    sub(sensor, "topic", topic)
    camera = sub(sensor, "camera")
    sub(camera, "horizontal_fov", hfov)
    image = sub(camera, "image")
    sub(image, "width", resolution[0])
    sub(image, "height", resolution[1])
    sub(image, "format", "R8G8B8")
    limits = sub(camera, "clip")
    sub(limits, "near", clip[0])
    sub(limits, "far", clip[1])


def _scene(world, profile, render_engine, opaque_material_compat=True):
    settings = profile["scene"]
    scene = sub(world, "scene")
    sub(scene, "ambient", vector(settings["ambient"]))
    sub(scene, "background", vector(settings["background"]))
    sub(scene, "shadows", settings["shadows"])
    sub(scene, "grid", False)
    sub(world, "gravity", vector(settings["gravity"]))
    sun_settings = settings["sun"]
    light = sub(world, "light", name="sun", type="directional")
    sub(light, "pose", vector(sun_settings["pose"]))
    sub(light, "direction", vector(sun_settings["direction"]))
    sub(light, "diffuse", vector(sun_settings["diffuse"]))
    sub(light, "specular", vector(sun_settings["specular"]))
    sub(light, "cast_shadows", settings["shadows"])
    gui = sub(world, "gui", fullscreen="false")
    plugin = sub(gui, "plugin", filename="MinimalScene", name="3D View")
    view_config = sub(plugin, "ignition-gui")
    sub(view_config, "title", "3D View")
    sub(view_config, "property", False, type="bool", key="showTitleBar")
    sub(view_config, "property", "docked", type="string", key="state")
    sub(plugin, "engine", render_engine)
    sub(plugin, "scene", "scene")
    sub(plugin, "ambient_light", vector(settings["ambient"][:3]))
    sub(plugin, "background_color", vector(settings["background"][:3]))
    sub(plugin, "camera_pose", vector(profile["validation_camera"]["pose"]))
    sub(plugin, "grid", False)
    # MinimalScene creates a renderer but does not import Gazebo entities.
    # GzSceneManager is the required SceneBroadcaster client. The other two
    # invisible helpers provide orbit/zoom and camera pose/tracking control.
    for filename in ("GzSceneManager", "InteractiveViewControl", "CameraTracking"):
        helper = sub(gui, "plugin", filename=filename, name=filename)
        config = sub(helper, "ignition-gui")
        for key, kind, value in (("resizable", "bool", False), ("showTitleBar", "bool", False),
                                 ("state", "string", "floating"), ("width", "double", 5), ("height", "double", 5)):
            sub(config, "property", value, key=key, type=kind)
    sub(gui, "plugin", filename="WorldControl", name="World control")
    sub(gui, "plugin", filename="WorldStats", name="World statistics")
    if render_engine == "ogre" and opaque_material_compat:
        plugin = sub(gui, "plugin", filename="MaiconOpaqueMaterialGui", name="Opaque texture compatibility")
        config = sub(plugin, "ignition-gui")
        for key, kind, value in (("showTitleBar", "bool", False), ("state", "string", "floating"),
                                 ("width", "double", 1), ("height", "double", 1)):
            sub(config, "property", value, key=key, type=kind)


def generate_world(spec, layout, profile, *, package_root=PACKAGE_ROOT, render_engine="ogre2", uav_cam=True,
                   ugvs=0, ego_slot="B-2", simulator="fortress", validation_camera=False,
                   opaque_material_compat=True, gui_render_engine=None, robot_model="wingbot", full_sensors=False):
    """Return (ElementTree, metadata); generation writes only derived local meshes/atlases/materials."""
    if render_engine not in ("ogre", "ogre2") or simulator not in SYSTEMS:
        raise ValueError("Unsupported renderer or Gazebo system family")
    if robot_model == "wingbot" and simulator != "fortress":
        raise ValueError("WINGBOT currently supports Gazebo Fortress only; select legacy explicitly for Harmonic")
    if robot_model == "wingbot" and int(ugvs) and render_engine != "ogre2":
        raise ValueError("WINGBOT full360 LiDAR requires the verified Ogre2 sensor server")
    gui_render_engine = gui_render_engine or render_engine
    if gui_render_engine not in ("ogre", "ogre2"):
        raise ValueError("Unsupported GUI renderer")
    validation = validate_layout(spec, layout)
    if not validation["ok"]:
        raise ValueError("Invalid arena layout: " + "; ".join(validation["errors"]))
    package_root = Path(package_root).resolve()
    print_size = spec["floor"].get("print_size", spec["floor"]["size"])
    meshes = write_shared_meshes(package_root, tuple(p/s for p, s in zip(print_size, spec["floor"]["size"])))
    texture_root = package_root / "media/textures"
    atlas_root = texture_root / "atlases"
    tile_px = int(profile["textures"]["atlas_tile_px"])
    colors = profile["colors"]
    root = ET.Element("sdf", version="1.9")
    world = sub(root, "world", name="maicon_arena")
    physics = sub(world, "physics", name="dart", type="dart")
    for key in ("max_step_size", "real_time_update_rate", "real_time_factor"):
        sub(physics, key, profile["physics"][key])
    plugin = add_system(world, "Physics", simulator)
    sub(sub(plugin, "engine"), "filename", PHYSICS_ENGINE[simulator])
    add_system(world, "UserCommands", simulator)
    add_system(world, "SceneBroadcaster", simulator)
    if render_engine == "ogre" and opaque_material_compat:
        add_system(world, "OpaqueMaterial", simulator)
    _scene(world, profile, gui_render_engine, opaque_material_compat)

    floor_model = static_model(world, "floor")
    floor_link = sub(floor_model, "link", name="floor")
    arena = static_model(world, "arena")
    links = {kind: sub(arena, "link", name=kind) for kind in ("wall", "sensor_box", "center_pad")}
    fixed = fixed_geometry(spec, floor_thickness=profile["geometry"]["floor_thickness"])
    floor_key = "texture_lite" if profile["textures"]["floor_lite"] else "texture"
    floor_texture = package_root / "media" / spec["floor"][floor_key]
    hazard = make_atlas({face: "hazard" for face in FACES}, texture_root, atlas_root, tile_px)
    for geometry in fixed:
        kind, name = geometry["kind"], geometry["name"]
        center, size = geometry["center"], geometry["size"]
        if kind == "floor":
            box(floor_link, name, center, size)
            # Collision top and the one visual surface meet at world Z=0.
            visual = box(floor_link, "floor_print", [0, 0, 0], [*spec["floor"]["size"], 1], collide=False,
                         color=[1, 1, 1, 1], mesh=meshes["floor"], texture=floor_texture, package_root=package_root)
            collision = floor_link.find("collision")
            friction = sub(sub(collision, "surface"), "friction")
            ode = sub(friction, "ode")
            sub(ode, "mu", spec["floor"]["friction_mu"])
            sub(ode, "mu2", spec["floor"]["friction_mu"])
            sub(visual, "cast_shadows", False)
        else:
            color = [*spec["walls"]["color_rgb"], 1] if kind == "wall" else colors["black"]
            box(links[kind], name, center, size, color=[1, 1, 1, 1] if kind == "sensor_box" else color,
                mesh=meshes["cube"] if kind == "sensor_box" else None,
                texture=hazard if kind == "sensor_box" else None, package_root=package_root)
    tower = spec["tower_leds"]["geometry"]
    for index, (x, y) in enumerate(spec["tower_leds"]["mounts"]):
        base_z = spec["sensor_boxes"]["size"][2]
        pole_h = tower["pole_height"]
        cylinder_visual(links["sensor_box"], f"tower_{index}_pole", [x, y, base_z + pole_h/2],
                        tower["radius"]*profile["geometry"]["tower_pole_radius_scale"], pole_h, colors["pole"])
        for segment, color in enumerate(tower["segments"]):
            h = tower["segment_height"]
            cylinder_visual(links["sensor_box"], f"tower_{index}_{color}", [x, y, base_z + pole_h + (segment+.5)*h],
                            tower["radius"], h, colors[color], emissive=True)

    layout_model = static_model(world, "layout")
    expanded = expand_boxes(spec, layout)
    clusters = {name: sub(layout_model, "link", name=name) for name in dict.fromkeys(b["cluster"] for b in expanded)}
    collisions = merged_collisions(spec, layout)
    for collision in collisions:
        box(clusters[collision["cluster"]], collision["name"], collision["center"], collision["size"])
    targets = []
    for cube in expanded:
        name = f"cube_{cube['index']}_{cube['level']}"
        texture = make_atlas(cube["faces"], texture_root, atlas_root, tile_px)
        box(clusters[cube["cluster"]], name, cube["center"], cube["size"], collide=False,
            color=[1, 1, 1, 1], mesh=meshes["cube"], texture=texture, package_root=package_root)
        cube["visual_name"] = name
        cube["atlas"] = texture.relative_to(package_root).as_posix()
        for face, label in cube["faces"].items():
            if not str(label).startswith("target:"):
                continue
            normal = NORMALS[face]
            position = [cube["center"][0] + normal[0]*cube["size"][0]/2,
                        cube["center"][1] + normal[1]*cube["size"][1]/2,
                        cube["center"][2] - cube["size"][2]/2 + spec["cubes"]["target_receiver"]["z"]]
            targets.append({"cube": name, "cluster": cube["cluster"], "face": face,
                            "camp": "BLUE" if cube["center"][0] < 0 else "RED",
                            "kind": str(label).split(":", 1)[1].split("@", 1)[0],
                            "position": position, "normal": list(normal),
                            "radius": spec["cubes"]["target_receiver"]["diameter"]/2})
            led_size = spec["cubes"]["target_signal_led"]["size"]
            box(clusters[cube["cluster"]], name + "_signal", [cube["center"][0], cube["center"][1],
                cube["center"][2] + cube["size"][2]/2 + led_size[2]/2], led_size,
                collide=False, color=colors["target_idle"], emissive=True)

    robot_info = {"robot_models": [], "sensors": {"camera": 0, "gpu_lidar": 0, "imu": 0}}
    if ugvs:
        from robot_models import add_ugvs
        robot_info = add_ugvs(world, spec, profile, ugvs, ego_slot, simulator, system_adder=add_system,
                              render_engine=render_engine, robot_model=robot_model, full_sensors=full_sensors)
    if uav_cam:
        cam, overrides = spec["uav_camera"], profile["uav_camera"]
        # Keep the supplied mount pose byte-for-byte in the canonical spec.
        # This explicit optical mount offset maps image right/up to world +X/+Y.
        camera_pose = list(cam["pose"])
        camera_pose[5] += overrides.get("mount_yaw_offset", 0.0)
        _camera(world, "uav_camera", camera_pose, overrides["resolution"], overrides.get("optical_hfov", cam["hfov"]),
                overrides["update_rate"], overrides["clip"], "/uav/image_raw")
    if validation_camera:
        cam = profile["validation_camera"]
        _camera(world, "validation_camera", cam["pose"], cam["resolution"], cam["hfov"],
                cam["update_rate"], cam["clip"], "/arena/validation/image_raw")
    sensor_types = Counter(sensor.get("type") for sensor in world.findall(".//sensor"))
    if any(sensor_types.get(kind, 0) for kind in ("camera", "gpu_lidar", "depth_camera")):
        sub(add_system(world, "Sensors", simulator), "render_engine", render_engine)
    if sensor_types.get("imu", 0):
        add_system(world, "Imu", simulator)

    static_models = [model for model in world.findall("model") if model.findtext("static") == "true"]
    counts = {"collisions": sum(len(model.findall(".//collision")) for model in static_models),
              "visuals": sum(len(model.findall(".//visual")) for model in static_models),
              "textures": len({node.text for model in static_models for node in model.findall(".//albedo_map")}),
              "lights": len(world.findall("light")), "sensors": dict(sensor_types)}
    if counts["collisions"] > 40 or counts["visuals"] > 80 or counts["textures"] > 30:
        raise ValueError("Static arena exceeds the required collision/visual/texture budget")
    metadata = {"schema": "maicon_arena_world_v1", "world": "maicon_arena", "frame": spec["meta"]["frame"],
                "floor_size": spec["floor"]["size"], "static_geometry": fixed,
                "cubes": expanded, "targets": targets, "merged_collisions": collisions,
                "m3_ground_truth": layout.get("m3_ground_truth", {}), "counts": counts,
                "profile": profile.get("name", "custom"), "render_engine": render_engine,
                "gui_render_engine": gui_render_engine,
                "opaque_material_compat": render_engine == "ogre" and bool(opaque_material_compat),
                "layout_validation": {key: validation[key] for key in ("ok", "warnings", "metrics")},
                "uav_camera": bool(uav_cam), "validation_camera": bool(validation_camera),
                "robot_models": robot_info.get("robot_models", []), "robots": robot_info,
                "start_poses": spec["bases"]["start_pose"],
                "spawn_slots": [] if not ugvs else [ego_slot] if ugvs == 1 else list(spec["bases"]["start_pose"])}
    metadata["selected_start_poses"] = {slot: spec["bases"]["start_pose"][slot] for slot in metadata["spawn_slots"]}
    ET.indent(root, space="  ")
    return ET.ElementTree(root), metadata


def load_profile(value="standard", package_root=PACKAGE_ROOT):
    path = Path(value)
    if not path.is_file():
        path = Path(package_root) / "config/profiles" / (str(value) + ".yaml")
    with path.open(encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    profile.setdefault("name", path.stem)
    return profile


def write_world(spec_path, layout_path, output, profile_path="standard", *, package_root=PACKAGE_ROOT, **kwargs):
    spec_path, layout_path, output = Path(spec_path), Path(layout_path), Path(output)
    profile = load_profile(profile_path, package_root)
    tree, metadata = generate_world(load_spec(spec_path), load_layout(layout_path), profile, package_root=package_root, **kwargs)
    metadata.update({"spec_sha256": hashlib.sha256(spec_path.read_bytes()).hexdigest(),
                     "layout_sha256": hashlib.sha256(layout_path.read_bytes()).hexdigest(),
                     "spec_file": spec_path.name, "layout_file": layout_path.name})
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    output.with_suffix(".metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return metadata


def boolean(value):
    if str(value).lower() not in ("true", "false", "1", "0"):
        raise argparse.ArgumentTypeError("Use true or false")
    return str(value).lower() in ("true", "1")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=PACKAGE_ROOT / "config/arena_spec.yaml")
    parser.add_argument("--layout", type=Path, default=PACKAGE_ROOT / "config/layouts/layout_default.yaml")
    parser.add_argument("--profile", default="standard")
    parser.add_argument("--output", type=Path, default=PACKAGE_ROOT / "worlds/maicon_arena.sdf")
    parser.add_argument("--render-engine", choices=("ogre", "ogre2"), default="ogre2")
    parser.add_argument("--gui-render-engine", choices=("ogre", "ogre2"), default=None)
    parser.add_argument("--simulator", choices=tuple(SYSTEMS), default="fortress")
    parser.add_argument("--ugvs", type=int, choices=(0, 1, 6), default=0)
    parser.add_argument("--ego-slot", choices=("B-1", "B-2", "B-3", "R-1", "R-2", "R-3"), default="B-2")
    parser.add_argument("--robot-model", choices=("legacy", "wingbot"), default="wingbot",
                        help="Validated WINGBOT model; legacy requires an explicit selection")
    parser.add_argument("--uav-cam", type=boolean, default=True)
    parser.add_argument("--full-sensors", type=boolean, default=False,
                        help="Give every WINGBOT its own real sensors and isolated namespace")
    parser.add_argument("--validation-camera", type=boolean, default=False)
    parser.add_argument("--opaque-material-compat", type=boolean, default=True,
                        help="Use package-local opaque texture depth compatibility plugin with Ogre 1 only")
    args = parser.parse_args(argv)
    report = write_world(args.spec, args.layout, args.output, args.profile,
                         render_engine=args.render_engine, simulator=args.simulator, ugvs=args.ugvs,
                         ego_slot=args.ego_slot, uav_cam=args.uav_cam, validation_camera=args.validation_camera,
                         opaque_material_compat=args.opaque_material_compat, gui_render_engine=args.gui_render_engine,
                         robot_model=args.robot_model, full_sensors=args.full_sensors)
    print(json.dumps({"world": report["world"], "counts": report["counts"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
