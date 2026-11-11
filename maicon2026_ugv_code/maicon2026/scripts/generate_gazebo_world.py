"""Export the MAICON arena and the supplied UGV Rover meshes to Gazebo Fortress."""
from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from maicon26.config import Config  # noqa: E402
from maicon26.hal.gazebo_model import LIDAR_RPY, LIDAR_XYZ, lidar_frame  # noqa: E402
from maicon26.hal.lidar_sectors import SECTOR_HALF_FOV, SECTOR_YAWS, sector_frame, sector_samples, sector_topic  # noqa: E402
from maicon26.sim.world import World  # noqa: E402


def resource_uri(path):
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / "ros2" / "gazebo" / path
    if not path.is_file():
        raise FileNotFoundError(f"Gazebo asset missing: {path}")
    if os.name == "nt":
        drive = path.drive.rstrip(":").lower()
        return f"/mnt/{drive}/" + path.as_posix().split(":/", 1)[1]
    return path.as_posix()


def sub(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def box_model(world, name, x, y, z, sx, sy, sz, color, *, collide=True):
    model = sub(world, "model", name=name)
    sub(model, "static", "true")
    sub(model, "pose", f"{x:.4f} {y:.4f} {z:.4f} 0 0 0")
    link = sub(model, "link", name="link")
    if collide:
        collision = sub(link, "collision", name="collision")
        geometry = sub(collision, "geometry")
        sub(sub(geometry, "box"), "size", f"{sx:.4f} {sy:.4f} {sz:.4f}")
    visual = sub(link, "visual", name="visual")
    geometry = sub(visual, "geometry")
    sub(sub(geometry, "box"), "size", f"{sx:.4f} {sy:.4f} {sz:.4f}")
    material = sub(visual, "material")
    sub(material, "ambient", color)
    sub(material, "diffuse", color)
    sub(material, "specular", "0 0 0 1")
    return model


def paint(link, name, x, y, sx, sy, color, z=.003, yaw=0):
    visual = sub(link, "visual", name=name)
    sub(visual, "pose", f"{x:.5f} {y:.5f} {z:.5f} 0 0 {yaw:.6f}")
    sub(sub(sub(visual, "geometry"), "box"), "size", f"{sx:.5f} {sy:.5f} .002")
    material = sub(visual, "material")
    sub(material, "ambient", color)
    sub(material, "diffuse", color)
    sub(material, "specular", "0 0 0 1")


def segment(link, name, a, b, width, color, z=.006):
    x0, y0 = a
    x1, y1 = b
    paint(link, name, (x0+x1)/2, (y0+y1)/2, math.hypot(x1-x0, y1-y0),
          width, color, z, math.atan2(y1-y0, x1-x0))


GLYPHS = {
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "2": ("11110", "00001", "00001", "01110", "10000", "10000", "11111"),
    "3": ("11110", "00001", "00001", "01110", "00001", "00001", "11110"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "-": ("00000", "00000", "00000", "11111", "00000", "00000", "00000"),
}


def label(link, name, text, cx, cy, scale, color):
    width = (len(text)*6-1)*scale
    for i, char in enumerate(text):
        for row, bits in enumerate(GLYPHS[char]):
            for col, bit in enumerate(bits):
                if bit == "1":
                    x = cx-width/2+(i*6+col+.5)*scale
                    y = cy+(3-row)*scale
                    paint(link, f"{name}_{i}_{row}_{col}", x, y, scale*.85, scale*.85,
                          color, z=.012)


def track_markings(world, g):
    model = sub(world, "model", name="track_markings")
    sub(model, "static", "true")
    link = sub(model, "link", name="paint")
    grid = ".18 .19 .23 1"
    white = ".82 .84 .85 1"
    yellow = ".95 .63 .04 1"
    for i in range(1, round(g.W/.2)):
        x = i*.2
        if x < g.W:
            paint(link, f"grid_x_{i}", x, g.H/2, .004, g.H, grid)
    for i in range(1, round(g.H/.2)):
        paint(link, f"grid_y_{i}", g.W/2, i*.2, g.W, .004, grid)

    for name, x in (("blue_base", g.base_depth), ("blue_common", g.W/2-g.common_half),
                    ("center", g.W/2), ("red_common", g.W/2+g.common_half),
                    ("red_base", g.W-g.base_depth)):
        paint(link, name, x, g.H/2, .012 if name == "center" else .007, g.H, white, .005)
    for j in (1, 2):
        y = j*g.H/3
        for side, x0, x1 in (("B", g.base_depth, g.W/2-g.common_half),
                             ("R", g.W/2+g.common_half, g.W-g.base_depth)):
            paint(link, f"sector_{side}{j}", (x0+x1)/2, y, x1-x0, .008, white, .006)
    for i in range(48):
        a0, a1 = 2*math.pi*i/48, 2*math.pi*(i+1)/48
        a = (g.W/2+.59*math.cos(a0), g.H/2+.59*math.sin(a0))
        b = (g.W/2+.59*math.cos(a1), g.H/2+.59*math.sin(a1))
        segment(link, f"circle_{i}", a, b, .012, white)

    for side in ("B", "R"):
        for slot in (1, 2, 3):
            x0, y0, x1, y1 = g.base_rect(f"{side}-{slot}")
            cx, cy = (x0+x1)/2, (y0+y1)/2
            paint(link, f"base_fill_{side}{slot}", cx, cy, x1-x0-.025,
                  y1-y0-.025, ".10 .11 .15 1", .007)
            if side == "B":
                points = [(x0+.03,y0+.03),(x1-.08,y0+.03),(x1-.02,y0+.10),
                          (x1-.02,y1-.10),(x1-.08,y1-.03),(x0+.03,y1-.03)]
                entry_x, arrow = x1+.14, 1
            else:
                points = [(x1-.03,y0+.03),(x0+.08,y0+.03),(x0+.02,y0+.10),
                          (x0+.02,y1-.10),(x0+.08,y1-.03),(x1-.03,y1-.03)]
                entry_x, arrow = x0-.14, -1
            for i, a in enumerate(points):
                segment(link, f"base_edge_{side}{slot}_{i}", a, points[(i+1)%len(points)],
                        .025, yellow, .010)
            for k in (0, .07):
                tip = entry_x + arrow*k
                segment(link, f"arrow_{side}{slot}_{k}_a", (tip-arrow*.07,cy-.06),
                        (tip,cy), .018, yellow, .010)
                segment(link, f"arrow_{side}{slot}_{k}_b", (tip,cy),
                        (tip-arrow*.07,cy+.06), .018, yellow, .010)
            label(link, f"base_label_{side}{slot}", f"{side}-{slot}", cx, cy,
                  .013, white)

    for side, x in (("B", g.W/2-g.common_half-.24), ("R", g.W/2+g.common_half+.24)):
        for slot, cy in enumerate(g.base_cy, 1):
            label(link, f"sector_label_{side}{slot}", f"{side}-S{slot}", x, cy-.65,
                  .009, white)
    for i, y in enumerate((.28, g.H-.28)):
        label(link, f"common_{i}", "COMMON", g.W/2, y, .012, white)

    for side, x in (("B", 1.38), ("R", g.W-1.38)):
        for edge, y in (("south", .12), ("north", g.H-.12)):
            paint(link, f"caution_{side}_{edge}", x, y, .93, .16, ".09 .09 .10 1", .008)
            for i in range(7):
                paint(link, f"caution_{side}_{edge}_{i}", x-.39+i*.13, y,
                      .045, .14, yellow, .011, math.pi/4)
    return model


def cylinder_visual(link, name, x, y, z, radius, height, color):
    visual = sub(link, "visual", name=name)
    sub(visual, "pose", f"{x:.4f} {y:.4f} {z:.4f} 0 0 0")
    shape = sub(sub(visual, "geometry"), "cylinder")
    sub(shape, "radius", radius)
    sub(shape, "length", height)
    material = sub(visual, "material")
    sub(material, "ambient", color)
    sub(material, "diffuse", color)


def cube_face_panel(model, face, name, color, sx, sy, *, receiver=False):
    """Visual stand-in for the published target/object picture on one cube face."""
    link = model.find("link")
    visual = sub(link, "visual", name=name)
    offsets = ((sx/2+.003, 0, 0, 0, 0, 0), (0, sy/2+.003, 0, 0, 0, 0),
               (-sx/2-.003, 0, 0, 0, 0, 0), (0, -sy/2-.003, 0, 0, 0, 0))
    x, y, z, r, p, a = offsets[face]
    sub(visual, "pose", f"{x} {y} {z} {r} {p} {a}")
    panel_x, panel_y = (.006, min(sy-.10, .50)) if face % 2 == 0 else (min(sx-.10, .50), .006)
    sub(sub(sub(visual, "geometry"), "box"), "size", f"{panel_x} {panel_y} .28")
    material = sub(visual, "material")
    sub(material, "ambient", color)
    sub(material, "diffuse", color)
    if receiver:
        lens = sub(link, "visual", name=f"{name}_receiver")
        sub(lens, "pose", f"{x*1.015:.5f} {y*1.015:.5f} .135 "
            f"{math.pi/2 if face % 2 else 0:.6f} "
            f"{math.pi/2 if face % 2 == 0 else 0:.6f} 0")
        disk = sub(sub(lens, "geometry"), "cylinder")
        sub(disk, "radius", ".032")
        sub(disk, "length", ".008")
        mat = sub(lens, "material")
        sub(mat, "ambient", ".82 .84 .86 1")
        sub(mat, "diffuse", ".82 .84 .86 1")


def object_photo_panel(model, face, cls, sx, sy):
    """Single textured quad, offset 3mm to avoid coplanar flicker."""
    visual = sub(model.find("link"), "visual", name=f"object_{face}")
    x, y = ((sx/2+.003, 0), (0, sy/2+.003), (-sx/2-.003, 0), (0, -sy/2-.003))[face]
    sub(visual, "pose", f"{x:.5f} {y:.5f} 0 0 0 {face*math.pi/2:.8f}")
    mesh = sub(sub(visual, "geometry"), "mesh")
    sub(mesh, "uri", resource_uri(f"assets/m3_photos/{cls}.obj"))


def arena_devices(world, g):
    # Page 25 gives device types, not dimensioned CAD or a control protocol.
    # These are visual, nonfunctional stand-ins at the positions shown there.
    model = sub(world, "model", name="arena_devices")
    sub(model, "static", "true")
    link = sub(model, "link", name="devices")
    for side, x in (("B", -.10), ("R", g.W+.10)):
        for edge, y in (("south", .10), ("north", g.H-.10)):
            prefix = f"tower_{side}_{edge}"
            cylinder_visual(link, prefix+"_pole", x, y, g.wall_h+.08,
                            .015, .16, ".12 .12 .13 1")
            for i, color in enumerate((".08 .55 .08 1", ".9 .65 .03 1", ".75 .06 .04 1")):
                cylinder_visual(link, prefix+f"_{i}", x, y, g.wall_h+.18+i*.045,
                                .032, .042, color)
    for side in ("B", "R"):
        for slot in (1, 2, 3):
            x0, y0, x1, y1 = g.base_rect(f"{side}-{slot}")
            x = x1+.018 if side == "B" else x0-.018
            for edge, y in (("a", y0+.08), ("b", y1-.08)):
                cylinder_visual(link, f"photo_{side}{slot}_{edge}", x, y, .035,
                                .016, .07, ".10 .10 .11 1")
    box_model(world, "smog_generator", g.W/2, -.19, .10,
              .22, .18, .20, ".10 .10 .11 1")
    return model


def robot_model(world, cfg, x, y, yaw, name="wingbot_ugv", topic_prefix="", team="BLUE",
                lidar_samples=360, fpv=False, effective_track=.17452, lidar_mode="sector160"):
    dims = cfg.robot
    mesh_dir = ROOT / "ros2" / "gazebo" / "assets" / "ugv_rover"
    if not (mesh_dir / "base_link.stl").is_file():
        raise FileNotFoundError("UGV Rover STL assets missing; run scripts/import_ugv_rover_meshes.py")
    wheel_track = .17452  # UGV Rover URDF wheel joint y = +/-0.08726 m
    model = sub(world, "model", name=name)
    sub(model, "pose", f"{x:.4f} {y:.4f} {dims.wheel_radius:.4f} 0 0 {yaw:.6f}")
    sub(model, "canonical_link", "base_link")
    body = sub(model, "link", name="base_link")
    inertia = sub(body, "inertial")
    sub(inertia, "mass", "2.78375064359473")
    tensor = sub(inertia, "inertia")
    for axis, value in (("ixx", .00651568), ("iyy", .00933992), ("izz", .01215835)):
        sub(tensor, axis, value)
    for axis in ("ixy", "ixz", "iyz"):
        sub(tensor, axis, 0)

    collision = sub(body, "collision", name="chassis_collision")
    sub(collision, "pose", "0 0 0.040 0 0 0")
    sub(sub(sub(collision, "geometry"), "box"), "size", ".240 .180 .072")

    def part(link, label, shape, pose, size, color, transparency=None):
        visual = sub(link, "visual", name=label)
        sub(visual, "pose", " ".join(f"{v:.6f}" for v in pose))
        geometry = sub(visual, "geometry")
        geom = sub(geometry, shape)
        if shape == "box":
            sub(geom, "size", " ".join(f"{v:.5f}" for v in size))
        elif shape == "cylinder":
            sub(geom, "radius", f"{size[0]:.5f}")
            sub(geom, "length", f"{size[1]:.5f}")
        else:
            sub(geom, "radius", f"{size[0]:.5f}")
        material = sub(visual, "material")
        sub(material, "ambient", color)
        sub(material, "diffuse", color)
        sub(material, "specular", ".08 .08 .08 1")
        if transparency is not None:
            sub(visual, "transparency", transparency)
        return visual

    def mesh_part(link, label, filename, pose, color):
        visual = sub(link, "visual", name=label)
        sub(visual, "pose", " ".join(f"{v:.6f}" for v in pose))
        mesh = sub(sub(visual, "geometry"), "mesh")
        sub(mesh, "uri", resource_uri(mesh_dir / filename))
        material = sub(visual, "material")
        sub(material, "ambient", color)
        sub(material, "diffuse", color)
        sub(material, "specular", ".06 .06 .06 1")
        return visual

    black = ".035 .038 .043 1"
    armor = ".065 .068 .074 1"
    rubber = ".018 .020 .023 1"
    antenna = ".025 .18 .75 1" if team == "BLUE" else ".80 .025 .025 1"
    # The supplied UGV Rover URDF places its base_link 0.08 m above ground;
    # this Gazebo model origin is at wheel height (0.04 m), hence +0.04 here.
    mesh_part(body, "rover_chassis", "base_link.stl", (0, 0, .04, 0, 0, 0), armor)
    # Fixed sensor assemblies retain the joint placements from ugv_rover.urdf.
    for label, filename, pose, color in (
        ("rover_depth_camera", "3d_camera_link.stl", (.06531, 0, .061953, 0, 0, 0), black),
        ("rover_lidar", "base_lidar_link.stl", (.039815, 0, .08, 0, 0, math.pi/2), black),
        ("rover_pan_base", "pt_base_link.stl", (-.018892, 0, .08, 0, 0, 0), black),
        ("rover_pan_stage", "pt_link1.stl", (-.008741, 0, .128, 0, 0, 0), armor),
        ("rover_tilt_stage", "pt_link2.stl", (-.008741, .02, .171003, math.pi/2, 0, 0), armor),
        ("rover_pt_camera", "pt_camera_link.stl", (.038945, 0, .170434, 0, 0, 0), black),
    ):
        mesh_part(body, label, filename, pose, color)
    for side, sign in (("left", 1), ("right", -1)):
        for label, wx in (("front", .0855), ("rear", -.0855)):
            wheel_name = f"wheel_{label}_{side}"
            wheel = sub(model, "link", name=wheel_name)
            sub(wheel, "pose", f"{wx:.4f} {sign*wheel_track/2:.5f} 0 0 0 0")
            inertial = sub(wheel, "inertial")
            sub(inertial, "mass", ".207085933743005")
            tensor = sub(inertial, "inertia")
            for axis, value in (("ixx", .000028), ("iyy", .000048), ("izz", .000028)):
                sub(tensor, axis, value)
            for axis in ("ixy", "ixz", "iyz"):
                sub(tensor, axis, 0)
            tyre = sub(wheel, "collision", name="tyre_collision")
            sub(tyre, "pose", "0 0 0 1.5707963 0 0")
            cylinder = sub(sub(tyre, "geometry"), "cylinder")
            sub(cylinder, "radius", dims.wheel_radius)
            sub(cylinder, "length", ".03")
            friction = sub(sub(sub(tyre, "surface"), "friction"), "ode")
            sub(friction, "mu", "1.0" if label == "front" else ".2")
            sub(friction, "mu2", "1.0" if label == "front" else ".2")
            mesh_part(wheel, "rover_wheel",
                      f"{side}_{'up' if label == 'front' else 'down'}_wheel_link.stl",
                      (0, 0, 0, math.pi/2, 0, 0), rubber)
            joint = sub(model, "joint", name=f"{wheel_name}_joint", type="revolute")
            sub(joint, "parent", "base_link")
            sub(joint, "child", wheel_name)
            axis = sub(joint, "axis")
            sub(axis, "xyz", "0 1 0")
            limit = sub(axis, "limit")
            sub(limit, "lower", "-1e16")
            sub(limit, "upper", "1e16")
            dynamics = sub(axis, "dynamics")
            sub(dynamics, "damping", ".002")

    # Match equipment visible in the competition photo, absent from vendor CAD.
    for side, sign in (("left", 1), ("right", -1)):
        part(body, f"receiver_base_{side}", "box", (.052, sign*.074, .125, 0, 0, 0),
             (.061, .058, .027), black)
        part(body, f"receiver_dome_{side}", "sphere", (.052, sign*.074, .142, 0, 0, 0),
             (.027,), ".89 .92 .94 1", transparency=.16)
        part(body, f"antenna_sleeve_{side}", "cylinder", (-.095, sign*.075, .189, 0, 0, 0),
             (.012, .068), antenna)
        part(body, f"antenna_band_{side}", "cylinder", (-.095, sign*.075, .185, 0, 0, 0),
             (.0125, .006), black)
        part(body, f"antenna_rod_{side}", "cylinder", (-.095, sign*.075, .260, 0, 0, 0),
             (.004, .09), black)

    if lidar_mode not in ("sector160", "sectors360"):
        raise ValueError("lidar_mode must be sector160 or sectors360")
    # OGRE 1's single wide scan can collapse to range_min. The optional full
    # circle uses three actual narrow GPU sensors at the same optical origin.
    # A ROS merger rotates their measured beams; it never raycasts a truth map.
    sectors = range(3) if lidar_mode == "sectors360" else (None,)
    for index in sectors:
        merged = index is not None
        sector_yaw = SECTOR_YAWS[index] if merged else 0.
        laser = sub(body, "sensor", name=f"laser_sector_{index}" if merged else "laser", type="gpu_lidar")
        sub(laser, "pose", " ".join(str(v) for v in (*LIDAR_XYZ, LIDAR_RPY[0], LIDAR_RPY[1], LIDAR_RPY[2] + sector_yaw)))
        sub(laser, "topic", sector_topic(topic_prefix, index) if merged else f"{topic_prefix}/scan")
        sub(laser, "ignition_frame_id", sector_frame(topic_prefix, index) if merged else lidar_frame(topic_prefix))
        sub(laser, "always_on", "true")
        sub(laser, "update_rate", "10")
        ray = sub(laser, "ray")
        horizontal = sub(sub(ray, "scan"), "horizontal")
        sub(horizontal, "samples", str(sector_samples(lidar_samples) if merged else lidar_samples))
        sub(horizontal, "resolution", "1")
        half_fov = SECTOR_HALF_FOV if merged else 1.396263
        sub(horizontal, "min_angle", -half_fov)
        sub(horizontal, "max_angle", half_fov)
        distance = sub(ray, "range")
        sub(distance, "min", ".05")
        sub(distance, "max", "8.0")
        sub(distance, "resolution", ".01")

    if fpv:
        # Lightweight, fixed forward observation camera for recording. The
        # rendered frames are independent of the synthetic referee detections.
        camera = sub(body, "sensor", name="fpv_camera", type="camera")
        sub(camera, "pose", ".10 0 .19 0 0 0")
        sub(camera, "topic", "/B2/fpv/image")
        sub(camera, "always_on", "true")
        sub(camera, "update_rate", "10")
        optics = sub(camera, "camera")
        sub(optics, "horizontal_fov", "1.3962634")
        image = sub(optics, "image")
        sub(image, "width", "640")
        sub(image, "height", "360")
        sub(image, "format", "R8G8B8")
        clip = sub(optics, "clip")
        sub(clip, "near", ".04")
        sub(clip, "far", "12")

    drive = sub(model, "plugin", filename="libignition-gazebo-diff-drive-system.so",
                name="ignition::gazebo::systems::DiffDrive")
    sub(drive, "left_joint", "wheel_front_left_joint")
    sub(drive, "left_joint", "wheel_rear_left_joint")
    sub(drive, "right_joint", "wheel_front_right_joint")
    sub(drive, "right_joint", "wheel_rear_right_joint")
    # Skid steering needs an empirically measured kinematic track. This changes
    # drive/encoder conversion only; CAD wheel placement remains wheel_track.
    if not math.isfinite(effective_track) or not .15 <= effective_track <= .35:
        raise ValueError("Effective wheel track must be between .15 and .35 metres")
    sub(drive, "wheel_separation", effective_track)
    sub(drive, "wheel_radius", dims.wheel_radius)
    sub(drive, "topic", f"{topic_prefix}/cmd_vel")
    sub(drive, "odom_topic", f"{topic_prefix}/odom")
    sub(drive, "frame_id", "odom")
    sub(drive, "child_frame_id", "base_link")
    sub(drive, "odom_publish_frequency", "30")
    return model


def fixed_uav_camera(world, g):
    """Organizer-owned overhead camera; it never collides with the UGVs."""
    model = sub(world, "model", name="fixed_uav")
    sub(model, "static", "true")
    sub(model, "pose", f"{g.W/2:.4f} {g.H/2:.4f} 2.5000 0 0 0")
    link = sub(model, "link", name="camera_mount")
    for name, pose, size, color in (
        ("body", "0 0 .080 0 0 0", ".22 .16 .055", ".11 .13 .16 1"),
        ("arm_x", "0 0 .065 0 0 0", ".46 .025 .025", ".18 .20 .23 1"),
        ("arm_y", "0 0 .065 0 0 0", ".025 .46 .025", ".18 .20 .23 1"),
        ("lens", "0 0 -.010 0 0 0", ".06 .06 .025", ".02 .03 .04 1"),
    ):
        visual = sub(link, "visual", name=name)
        sub(visual, "pose", pose)
        sub(sub(sub(visual, "geometry"), "box"), "size", size)
        material = sub(visual, "material")
        sub(material, "ambient", color)
        sub(material, "diffuse", color)
    camera = sub(link, "sensor", name="arena_camera", type="camera")
    # Gazebo's camera faces +X. Positive pitch turns that axis toward -Z.
    sub(camera, "pose", "0 0 -.035 0 1.57079632679 1.57079632679")
    sub(camera, "topic", "/uav/image")
    sub(camera, "always_on", "true")
    sub(camera, "update_rate", "1")
    optics = sub(camera, "camera")
    sub(optics, "horizontal_fov", "2.40")
    image = sub(optics, "image")
    sub(image, "width", "1280")
    sub(image, "height", "720")
    sub(image, "format", "R8G8B8")
    clip = sub(optics, "clip")
    sub(clip, "near", ".05")
    sub(clip, "far", "10")
    return model


def third_person_camera(world, g):
    """Recording-only overview, separate from the organizer's UAV feed.

    The camera looks down from outside the south wall and includes all four
    arena corners. It has no visible or collision geometry and is never used
    for robot perception, localization, or planning.
    """
    model = sub(world, "model", name="recording_overview")
    sub(model, "static", "true")
    pitch = math.atan2(7.0, g.H / 2 + 2.0)
    sub(model, "pose", f"{g.W/2:.6f} -2 7 0 {pitch:.9f} {math.pi/2:.9f}")
    link = sub(model, "link", name="recording_camera_mount")
    camera = sub(link, "sensor", name="third_person_camera", type="camera")
    sub(camera, "topic", "/recording/arena/image")
    sub(camera, "always_on", "true")
    sub(camera, "update_rate", "5")
    optics = sub(camera, "camera")
    sub(optics, "horizontal_fov", ".95")
    image = sub(optics, "image")
    sub(image, "width", "960")
    sub(image, "height", "720")
    sub(image, "format", "R8G8B8")
    clip = sub(optics, "clip")
    sub(clip, "near", ".05")
    sub(clip, "far", "30")
    return model


def generate(seed, robot, render_engine="ogre", layout="fixed",
             banner_texture="assets/front_banner.png",
             field_mesh="assets/field_markings.obj",
             block_texture="assets/dark_block.png", lidar_samples=360, m3_count=4,
             fpv=False, third_person=False, effective_track=.17452, lidar_mode="sector160"):
    cfg = Config()
    cfg.arena = replace(cfg.arena, base_depth=.4, base_height=1.2)
    arena = World(cfg, seed=seed,
                  scenario="maicon_fixed" if layout in ("fixed", "pdf") else "random",
                  fixed_m3_count=m3_count)
    g = arena.geo
    root = ET.Element("sdf", version="1.8")
    world = sub(root, "world", name="maicon_arena")
    sub(world, "gravity", "0 0 -9.8")
    scene = sub(world, "scene")
    sub(scene, "ambient", ".7 .7 .7 1")
    sub(scene, "background", ".8 .8 .8 1")
    sun = sub(world, "light", name="sun", type="directional")
    sub(sun, "pose", "3 2 8 0 0 0")
    sub(sun, "diffuse", ".9 .9 .9 1")
    sub(sun, "specular", ".2 .2 .2 1")
    sub(sun, "direction", "-.4 -.3 -1")
    for filename, name in (("libignition-gazebo-physics-system.so", "Physics"),
                           ("libignition-gazebo-scene-broadcaster-system.so", "SceneBroadcaster"),
                           ("libignition-gazebo-sensors-system.so", "Sensors")):
        plugin = sub(world, "plugin", filename=filename, name=f"ignition::gazebo::systems::{name}")
        if name == "Sensors":
            sub(plugin, "render_engine", render_engine)
    floor = box_model(world, "floor", g.W / 2, g.H / 2, -.025,
                      g.W, g.H, .05, ".075 .080 .11 1")
    marking = sub(floor.find("link"), "visual", name="field_markings")
    sub(marking, "pose", "0 0 .026 0 0 0")
    sub(sub(marking, "geometry"), "mesh")
    sub(marking.find("geometry/mesh"), "uri", resource_uri(field_mesh))
    for name, x, y, sx, sy in (
        ("wall_west", -.025, g.H / 2, .05, g.H + .1),
        ("wall_east", g.W + .025, g.H / 2, .05, g.H + .1),
        ("wall_south", g.W / 2, -.025, g.W + .1, .05),
        ("wall_north", g.W / 2, g.H + .025, g.W + .1, .05),
    ):
        box_model(world, name, x, y, g.wall_h / 2, sx, sy, g.wall_h, ".095 .100 .12 1")
    banner = sub(world, "model", name="front_banner")
    sub(banner, "static", "true")
    sub(banner, "pose", f"{g.W/2:.4f} -.0600 {g.wall_h/2:.4f} 0 0 0")
    banner_link = sub(banner, "link", name="link")
    banner_visual = sub(banner_link, "visual", name="visual")
    sub(banner_visual, "pose", f"0 0 0 {math.pi/2:.6f} 0 0")
    banner_plane = sub(sub(banner_visual, "geometry"), "plane")
    sub(banner_plane, "normal", "0 0 1")
    sub(banner_plane, "size", f"{g.W:.4f} {g.wall_h-.04:.4f}")
    banner_material = sub(banner_visual, "material")
    sub(banner_material, "ambient", "1 1 1 1")
    sub(banner_material, "diffuse", "1 1 1 1")
    metal = sub(sub(banner_material, "pbr"), "metal")
    sub(metal, "albedo_map", banner_texture)
    sub(metal, "metalness", "0")
    sub(metal, "roughness", "1")
    for side, x in (("west", -.10), ("east", g.W+.10)):
        for i, y in enumerate((.10, g.H/3, 2*g.H/3, g.H-.10)):
            post = box_model(world, f"hazard_post_{side}_{i}", x, y, g.wall_h/2,
                             .20, .20, g.wall_h, ".09 .09 .10 1")
            for stripe in (-1, 0, 1):
                paint(post.find("link"), f"top_stripe_{stripe}", stripe*.058, 0,
                      .13, .036, ".97 .63 .04 1", z=g.wall_h/2+.002, yaw=math.pi/4)
    arena_devices(world, g)
    if layout in ("fixed", "pdf"):
        fixed_uav_camera(world, g)
    if third_person:
        third_person_camera(world, g)
    led_model = sub(world, "model", name="target_signal_lights")
    sub(led_model, "static", "true")
    led_link = sub(led_model, "link", name="lights")
    for i, cube in enumerate(arena.cubes):
        kind = getattr(cube, "kind", None) or (
            "target" if any(item.kind == "target" for item in cube.items.values()) else "object")
        color = "1 1 1 1"
        block = box_model(world, f"{kind}_{i:02d}", cube.cx, cube.cy, g.box_size/2,
                          getattr(cube, "sx", g.box_size), getattr(cube, "sy", g.box_size),
                          g.box_size, color)
        metal = sub(sub(block.find("./link/visual/material"), "pbr"), "metal")
        sub(metal, "albedo_map", block_texture)
        sub(metal, "metalness", "0")
        sub(metal, "roughness", "1")
        for face, item in cube.items.items():
            if item.kind == "object":
                object_photo_panel(block, face, item.cls, cube.sx, cube.sy)
                continue
            panel_color = ".20 .31 .41 1" if item.kind == "target" else ".32 .30 .25 1"
            cube_face_panel(block, face, f"{item.kind}_{face}", panel_color,
                            getattr(cube, "sx", g.box_size), getattr(cube, "sy", g.box_size),
                            receiver=item.kind == "target")
        if kind == "target":
            cylinder_visual(led_link, f"signal_{i:02d}", cube.cx, cube.cy,
                            g.box_size+.018, .05, .036, ".20 .03 .03 1")
    robots = [f"{s}{n}" for s in "BR" for n in (1, 2, 3)] if robot == "all" else (
        [] if robot == "none" else [robot])
    for ident in robots:
        side, slot = ident[0], int(ident[1])
        x, y = g.base_center(f"{side}-{slot}")
        x += .15 if side == "B" else -.15
        robot_model(world, arena.cfg, x, y, 0 if side == "B" else math.pi,
                    name="wingbot_ugv" if ident == "B2" else f"{ident}_ugv",
                    topic_prefix="" if ident == "B2" else f"/{ident}",
                    team="BLUE" if side == "B" else "RED",
                    lidar_samples=lidar_samples, fpv=fpv and ident == "B2", effective_track=effective_track,
                    lidar_mode=lidar_mode)
    ET.indent(root, space="  ")
    return root, arena


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=2)
    parser.add_argument("--robot", choices=["B1", "B2", "B3", "R1", "R2", "R3", "all", "none"], default="B2")
    parser.add_argument("--layout", choices=["fixed", "pdf", "random"], default="fixed")
    parser.add_argument("--render-engine", choices=["ogre", "ogre2"], default="ogre")
    parser.add_argument("--lidar-samples", type=int, choices=[360, 640], default=360)
    parser.add_argument("--lidar-mode", choices=["sector160", "sectors360"], default="sector160",
                        help="Optional full circle from three real co-located 120-degree scan sectors")
    parser.add_argument("--effective-track", type=float, default=.17452,
                        help="Measured skid-steer kinematic track; preserves CAD wheel positions")
    parser.add_argument("--m3-count", type=int, default=4,
                        help="Dedicated random M3 cubes in fixed arena; test quantity, not official count")
    parser.add_argument("--fpv", action="store_true", help="Render B2 forward camera for recording")
    parser.add_argument("--third-person", action="store_true",
                        help="Render recording-only oblique arena overview, never an Agent input")
    parser.add_argument("--banner-texture", default="assets/front_banner.png")
    parser.add_argument("--field-mesh", default="assets/field_markings.obj")
    parser.add_argument("--block-texture", default="assets/dark_block.png")
    parser.add_argument("--output", type=Path, default=ROOT / "ros2" / "gazebo" / "maicon_arena.sdf")
    args = parser.parse_args()
    tree, arena = generate(args.seed, args.robot, args.render_engine, args.layout,
                           args.banner_texture, args.field_mesh,
                           args.block_texture, args.lidar_samples, args.m3_count, args.fpv, args.third_person,
                           args.effective_track, args.lidar_mode)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(tree).write(args.output, encoding="unicode", xml_declaration=True)
    print(f"{args.output} | layout={args.layout} seed={args.seed} "
          f"used={arena.layout_meta['used_layout_seed']} boxes={len(arena.cubes)} robot={args.robot}")


if __name__ == "__main__":
    main()
