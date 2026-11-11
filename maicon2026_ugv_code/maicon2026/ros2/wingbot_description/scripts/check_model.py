#!/usr/bin/env python3
"""Independent Gate A audit of an expanded WINGBOT URDF.

Reads actual URDF geometry, masses and kinematics; deliberately does not import
the generator's transforms or its exported frames.yaml. No ROS installation is
needed after xacro expansion. Exit 1 means a failed gate, never a warning pass.
"""
import argparse
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml


def vector(value, default="0 0 0"):
    return np.array([float(x) for x in (value or default).split()], dtype=float)


def rotation(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def origin(element):
    pose = np.eye(4)
    node = element.find("origin")
    if node is not None:
        pose[:3, 3] = vector(node.get("xyz"))
        pose[:3, :3] = rotation(vector(node.get("rpy")))
    return pose


def forward(root, angles=None):
    angles = angles or {}
    links = {link.get("name") for link in root.findall("link")}
    joints = root.findall("joint")
    roots = links - {j.find("child").get("link") for j in joints}
    if len(roots) != 1:
        raise ValueError(f"Expected one URDF root, found {roots}")
    poses = {next(iter(roots)): np.eye(4)}
    pending = list(joints)
    while pending:
        progressed = False
        for joint in pending[:]:
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in poses:
                continue
            motion = np.eye(4)
            q = angles.get(joint.get("name"), 0.0)
            if joint.get("type") in ("revolute", "continuous", "prismatic"):
                axis_node = joint.find("axis")
                axis = vector(axis_node.get("xyz") if axis_node is not None else "1 0 0")
                axis /= np.linalg.norm(axis)
                if joint.get("type") == "prismatic":
                    motion[:3, 3] = axis * q
                else:
                    # Independent unit quaternion formulation.
                    w, xyz = math.cos(q/2), axis * math.sin(q/2)
                    x, y, z = xyz
                    motion[:3, :3] = [
                        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
            poses[child] = poses[parent] @ origin(joint) @ motion
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError("Disconnected or cyclic URDF joints")
    return poses


def mesh_path(uri, package_root):
    if uri.startswith("package://wingbot_description/"):
        return package_root / uri.split("package://wingbot_description/", 1)[1]
    if uri.startswith("file://"):
        return Path(uri[7:])
    path = Path(uri)
    return path if path.is_absolute() else package_root / path


def mesh_vertices(path):
    data = path.read_bytes()
    if path.suffix.lower() == ".stl":
        if len(data) >= 84:
            count = struct.unpack_from("<I", data, 80)[0]
            if len(data) == 84 + 50*count:
                vertices = [struct.unpack_from("<3f", data, 84+50*i+12+12*j)
                            for i in range(count) for j in range(3)]
                return np.asarray(vertices), count
        text = data.decode("ascii")
        vertices = [vector(line.strip()[7:]) for line in text.splitlines()
                    if line.strip().startswith("vertex ")]
        return np.asarray(vertices), len(vertices)//3
    if path.suffix.lower() == ".obj":
        lines = data.decode("utf-8").splitlines()
        vertices = [vector(line[2:]) for line in lines if line.startswith("v ")]
        faces = sum(len(line.split())-3 for line in lines if line.startswith("f "))
        return np.asarray(vertices), faces
    raise ValueError(f"Unsupported audit mesh format: {path}")


def geometry_vertices(geometry, package_root):
    box, cylinder, sphere, mesh = [geometry.find(tag) for tag in ("box", "cylinder", "sphere", "mesh")]
    if box is not None:
        size = vector(box.get("size"))
        return np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])*size, 12
    if cylinder is not None:
        radius, length = float(cylinder.get("radius")), float(cylinder.get("length"))
        theta = np.linspace(0, 2*math.pi, 192, endpoint=False)
        vertices = [[radius*math.cos(t), radius*math.sin(t), z] for t in theta for z in (-length/2, length/2)]
        return np.asarray(vertices), 4*24-4
    if sphere is not None:
        radius = float(sphere.get("radius"))
        vertices = [[radius*math.sin(p)*math.cos(t), radius*math.sin(p)*math.sin(t), radius*math.cos(p)]
                    for p in np.linspace(0, math.pi, 49)
                    for t in np.linspace(0, 2*math.pi, 192, endpoint=False)]
        return np.asarray(vertices), 2*24*(16-1)
    if mesh is not None:
        vertices, triangles = mesh_vertices(mesh_path(mesh.get("filename"), package_root))
        return vertices * vector(mesh.get("scale"), "1 1 1"), triangles
    raise ValueError("Missing supported visual geometry")


def audit(urdf_path, mods_path, package_root, wheels=6, sensors="full", prefix=""):
    root = ET.parse(urdf_path).getroot()
    mods = yaml.safe_load(Path(mods_path).read_text(encoding="utf-8"))
    poses = forward(root)
    checks = []

    def check(name, passed, actual=None, expected=None):
        if isinstance(actual, np.ndarray):
            actual = actual.tolist()
        checks.append(dict(name=name, passed=bool(passed), actual=actual, expected=expected))

    links = {e.get("name"): e for e in root.findall("link")}
    joints = {e.get("name"): e for e in root.findall("joint")}
    vertices_all, extra_triangles, mesh_collisions, collision_count, mass = [], 0, [], 0, 0.
    wheel_links = []
    for name, link in links.items():
        for visual in link.findall("visual"):
            vertices, triangles = geometry_vertices(visual.find("geometry"), package_root)
            pose = poses[name] @ origin(visual)
            vertices_all.append(vertices @ pose[:3, :3].T + pose[:3, 3])
            mesh = visual.find("geometry/mesh")
            if mesh is None or "/ugv_rover/" not in mesh.get("filename", "").replace("\\", "/"):
                extra_triangles += triangles
        collisions = link.findall("collision")
        collision_count += len(collisions)
        mesh_collisions.extend(name for c in collisions if c.find("geometry/mesh") is not None)
        if "wheel" in name and collisions:
            wheel_links.append(name)
            for collision in collisions:
                cylinder = collision.find("geometry/cylinder")
                check("wheel_cylinder:"+name, cylinder is not None)
                if cylinder is not None:
                    check("wheel_radius:"+name, abs(float(cylinder.get("radius"))-mods["wheels"]["radius"]) < 1e-8,
                          float(cylinder.get("radius")), mods["wheels"]["radius"])
        inertial = link.find("inertial")
        if inertial is not None:
            m = float(inertial.find("mass").get("value"))
            mass += m
            i = inertial.find("inertia")
            matrix = np.array([
                [float(i.get("ixx")), float(i.get("ixy")), float(i.get("ixz"))],
                [float(i.get("ixy")), float(i.get("iyy")), float(i.get("iyz"))],
                [float(i.get("ixz")), float(i.get("iyz")), float(i.get("izz"))]])
            eigen = np.linalg.eigvalsh(matrix)
            check("physical_inertia:"+name, m > 0 and np.all(eigen > 0) and eigen[-1] <= eigen[0]+eigen[1]+1e-9,
                  dict(mass=m, eigenvalues=eigen.tolist()))
    all_points = np.concatenate(vertices_all)
    minimum, maximum = all_points.min(axis=0), all_points.max(axis=0)
    bbox = maximum-minimum
    check("length_m", abs(bbox[0]-.255) <= .005+1e-9, bbox[0], [.250, .260])
    check("width_m", bbox[1] <= .245+1e-9, bbox[1], "<=0.245")
    check("height_m", abs(bbox[2]-mods["iff_antennas"]["tip_z"]) <= .005,
          bbox[2], mods["iff_antennas"]["tip_z"])
    check("total_mass_kg", abs(mass-mods["target_spec"]["mass_total"]) <= .05, mass, 2.9)
    check("wheel_count", len(wheel_links) == 6, len(wheel_links), 6)
    check("mesh_collisions", not mesh_collisions, mesh_collisions, [])
    check("collision_budget", collision_count <= 10, collision_count, "<=10")
    check("added_visual_triangle_budget_24_segments", extra_triangles <= 3000, extra_triangles, "<=3000")

    expected_frames = {
        mods["laser_emitter"]["frame"]: mods["laser_emitter"]["lens_xyz"],
        mods["camera"]["frame"]: mods["camera"]["lens_xyz"],
        "receiver_left_link": mods["receivers"]["left"]["center"],
        "receiver_right_link": mods["receivers"]["right"]["center"],
    }
    for side in ("left", "right"):
        bend = np.array(mods["iff_antennas"][side]["bend"], dtype=float)
        angle, azimuth = mods["iff_antennas"]["lean"][side]
        dz = mods["iff_antennas"]["tip_z"]-bend[2]
        bend += [dz*math.tan(angle)*math.cos(azimuth), dz*math.tan(angle)*math.sin(azimuth), dz]
        expected_frames[f"iff_{side}_tip_link"] = bend.tolist()
    for name, expected in expected_frames.items():
        actual = poses.get(prefix+name)
        check("frame_exists:"+name, actual is not None)
        if actual is not None:
            err = float(np.linalg.norm(actual[:3, 3]-expected))
            check("frame_position:"+name, err <= .001,
                  dict(xyz=actual[:3, 3].tolist(), error_m=err), expected)
    laser, camera = prefix+mods["laser_emitter"]["frame"], prefix+mods["camera"]["frame"]
    if laser in poses and camera in poses:
        offset = poses[laser][:3, 3]-poses[camera][:3, 3]
        check("boresight_offset", np.linalg.norm(offset-mods["laser_emitter"]["boresight_offset_from_camera"]) <= .001,
              offset, mods["laser_emitter"]["boresight_offset_from_camera"])
        for frame in (laser, camera):
            check("forward_axis:"+frame, np.linalg.norm(poses[frame][:3, 0]-[1, 0, 0]) <= 1e-4,
                  poses[frame][:3, 0], [1, 0, 0])
    optical = poses.get(prefix+"pt_camera_optical_link")
    if optical is not None and camera in poses:
        check("optical_axes", np.allclose(optical[:3, :3], [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-4),
              optical[:3, :3].tolist(), "optical z forward, x right, y down")
    else:
        check("optical_frame_exists", False)

    for kind, test_q in (("pan", .3), ("tilt", .2)):
        name = prefix+mods["pan_tilt"][kind+"_joint"]
        joint = joints.get(name)
        check("joint_exists:"+kind, joint is not None)
        if joint is None:
            continue
        limit = joint.find("limit")
        limit_value = mods["pan_tilt"][kind+"_limit_rad"]
        check("joint_limits:"+kind, limit is not None and
              abs(float(limit.get("lower"))+limit_value)<1e-6 and
              abs(float(limit.get("upper"))-limit_value)<1e-6 and
              abs(float(limit.get("velocity"))-mods["pan_tilt"]["max_speed_rad_s"])<1e-6,
              dict(limit.attrib) if limit is not None else None)
        changed = forward(root, {name: test_q})
        axis = changed[camera][:3, 0]
        yaw = math.atan2(axis[1], axis[0])
        elevation = math.atan2(axis[2], math.hypot(axis[0], axis[1]))
        measured = yaw if kind == "pan" else elevation
        check("joint_sign:"+kind, abs(measured-(-test_q if kind=="pan" else test_q))<1e-4,
              measured, -test_q if kind=="pan" else test_q)
        # Mast accessories must not follow pan/tilt.
        for side in ("left", "right"):
            name_fixed = prefix+"receiver_"+side+"_link"
            if name_fixed in poses:
                check("fixed_receiver:"+kind+":"+side, np.allclose(poses[name_fixed], changed[name_fixed]))
        relative0 = np.linalg.inv(poses[camera]) @ poses[laser]
        relative1 = np.linalg.inv(changed[camera]) @ changed[laser]
        check("laser_camera_rigid:"+kind, np.allclose(relative0, relative1, atol=1e-8))

    sensor_nodes = root.findall(".//sensor")
    types = [s.get("type") for s in sensor_nodes]
    check("no_wideangle_camera", not any("wideangle" in s for s in types), types)
    check("no_lights", not root.findall(".//light"))
    plugins = root.findall(".//plugin")
    check("no_classic_plugins", not any("gazebo_ros" in p.get("filename", "") for p in plugins))
    drives = [p for p in plugins if "DiffDrive" in p.get("name", "")]
    check("fortress_diff_drive", len(drives)==1, len(drives), 1)
    if drives:
        drive = drives[0]
        check("drive_radius", abs(float(drive.findtext("wheel_radius", "nan"))-.04)<1e-8)
        check("drive_joint_count", len(drive.findall("left_joint")) == wheels//2 and len(drive.findall("right_joint"))==wheels//2)
        for tag, expected in (("odom_publish_frequency",20), ("max_linear_velocity",.6),
                              ("min_linear_velocity",-.6), ("max_angular_velocity",3),
                              ("min_angular_velocity",-3)):
            actual = float(drive.findtext(tag, "nan"))
            check("drive:"+tag, abs(actual-expected)<1e-8, actual, expected)
    if sensors == "none":
        check("dummy_has_no_sensors", not sensor_nodes, types, [])
    else:
        cameras = [s for s in sensor_nodes if s.get("type")=="camera"]
        lidars = [s for s in sensor_nodes if s.get("type")=="gpu_lidar"]
        imus = [s for s in sensor_nodes if s.get("type")=="imu"]
        check("full_sensor_types", len(cameras)==1 and len(lidars)==1 and len(imus)==1, types)
        if cameras:
            c = cameras[0]
            for tag, expected in (("update_rate",10), ("camera/image/width",640), ("camera/image/height",480),
                                  ("camera/horizontal_fov",2), ("camera/distortion/k1",-.25)):
                actual = float(c.findtext(tag, "nan"))
                check("camera:"+tag, abs(actual-expected)<1e-8, actual, expected)
        if lidars:
            l = lidars[0]
            lidar = l.find("lidar")
            if lidar is None:
                lidar = l.find("ray")
            for tag, expected in (("scan/horizontal/samples",450), ("range/min",.02), ("range/max",12)):
                actual = float(lidar.findtext(tag, "nan")) if lidar is not None else float("nan")
                check("lidar:"+tag, abs(actual-expected)<1e-8, actual, expected)
            check("lidar:rate", abs(float(l.findtext("update_rate","nan"))-10)<1e-8)
        if imus:
            check("imu:rate", abs(float(imus[0].findtext("update_rate","nan"))-20)<1e-8)
    return dict(gate="A", passed=all(c["passed"] for c in checks), checks=checks,
                source=dict(urdf=str(urdf_path), urdf_sha256=hashlib.sha256(Path(urdf_path).read_bytes()).hexdigest(),
                            yaml_sha256=hashlib.sha256(Path(mods_path).read_bytes()).hexdigest()),
                bounds_m=dict(min=minimum.tolist(), max=maximum.tolist(), lwh=bbox.tolist()),
                profile=dict(wheels=wheels, sensors=sensors, prefix=prefix))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--mods", type=Path, required=True)
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--wheels", type=int, choices=(6, 2), default=6)
    parser.add_argument("--sensors", choices=("full", "none"), default="full")
    parser.add_argument("--prefix", default="")
    args = parser.parse_args()
    try:
        result = audit(args.urdf, args.mods, args.package_root, args.wheels, args.sensors, args.prefix)
    except Exception as exc:
        result = dict(gate="A", passed=False, error=f"{type(exc).__name__}: {exc}")
    args.json.parent.mkdir(parents=True, exist_ok=True)
    # Missing XML fields appear as nonfinite audit observations, never as a pass.
    def safe(value):
        if isinstance(value, dict):
            return {k: safe(v) for k, v in value.items()}
        if isinstance(value, list):
            return [safe(v) for v in value]
        if isinstance(value, (float, np.floating)) and not math.isfinite(value):
            return None
        return value
    result = safe(result)
    args.json.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps(dict(passed=result["passed"], report=str(args.json),
                         failed=[c for c in result.get("checks", []) if not c["passed"]],
                         error=result.get("error")), ensure_ascii=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
