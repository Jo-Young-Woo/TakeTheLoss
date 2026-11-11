#!/usr/bin/env python3
"""True orthographic views of actual expanded URDF visuals (diagnostic only).

This independent CPU triangle rasterizer does not provide Gate B image pixels.
Gate B uses only the separate actual Gazebo RGB capture and live TF messages.
"""
import hashlib
import json
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from wingbot_geometry import forward_kinematics, origin_matrix, read_stl, transformed_points


def _obj(path):
    vertices, triangles = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            vertices.append([float(v) for v in fields[1:4]])
        elif fields and fields[0] == "f":
            ids = [int(v.split("/")[0])-1 for v in fields[1:]]
            triangles.extend([[vertices[ids[0]], vertices[ids[i]], vertices[ids[i+1]]]
                              for i in range(1, len(ids)-1)])
    return np.asarray(triangles, dtype=float)


def _box(size):
    vertices = np.asarray([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])*size
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4),
             (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return np.asarray([[vertices[f[0]], vertices[f[i]], vertices[f[i+1]]]
                       for f in faces for i in (1, 2)])


def _cylinder(radius, length, segments=24):
    points = [[radius*np.cos(a), radius*np.sin(a), z]
              for z in (-length/2, length/2) for a in np.arange(segments)*2*np.pi/segments]
    triangles = []
    for i in range(segments):
        j = (i+1) % segments
        triangles.extend([[points[i], points[j], points[j+segments]],
                          [points[i], points[j+segments], points[i+segments]],
                          [[0, 0, -length/2], points[j], points[i]],
                          [[0, 0, length/2], points[i+segments], points[j+segments]]])
    return np.asarray(triangles)


def actual_visuals(urdf, package, joint_positions):
    root = ET.parse(urdf).getroot()
    transforms = forward_kinematics(root, joint_positions)
    global_colors = {m.get("name"): m.find("color").get("rgba") for m in root.findall("material") if m.find("color") is not None}
    visuals = []
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            geometry = list(visual.find("geometry"))[0]
            if geometry.tag == "mesh":
                uri = geometry.get("filename")
                prefix = "package://wingbot_description/"
                if not uri.startswith(prefix):
                    raise ValueError("Orthographic visual must resolve inside wingbot_description")
                path = (package / uri[len(prefix):]).resolve()
                if not path.is_relative_to(package.resolve()):
                    raise ValueError("Visual mesh escapes its package")
                tris = read_stl(path) if path.suffix.lower() == ".stl" else _obj(path)
                tris *= np.asarray([float(v) for v in geometry.get("scale", "1 1 1").split()])
            elif geometry.tag == "box":
                tris = _box(np.asarray([float(v) for v in geometry.get("size").split()]))
            elif geometry.tag == "cylinder":
                tris = _cylinder(float(geometry.get("radius")), float(geometry.get("length")))
            else:
                raise ValueError("Unsupported actual visual for orthographic diagnostic: " + geometry.tag)
            material = visual.find("material")
            color = material.find("color") if material is not None else None
            rgba = color.get("rgba") if color is not None else global_colors.get(material.get("name") if material is not None else None, ".3 .3 .3 1")
            rgb = np.clip(np.asarray([float(v) for v in rgba.split()[:3]]), 0, 1)
            matrix = transforms[link.get("name")] @ origin_matrix(visual)
            visuals.append((transformed_points(tris, matrix), rgb))
    return visuals


def render_orthographic(visuals, rotation, size=640, scale=1800.):
    image = np.full((size, size, 3), 242, dtype=np.uint8)
    depth = np.full((size, size), np.inf)
    light = np.asarray([.4, -.6, 1.])
    light /= np.linalg.norm(light)
    for triangles, rgb in visuals:
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        colors = np.clip(rgb[None, :] * (.45+.55*np.abs(normals@light))[:, None]*255, 0, 255).astype(np.uint8)[:, ::-1]
        points = (triangles-[0, 0, .14]) @ np.asarray(rotation).T
        pixels = points[:, :, :2]*scale + size/2
        for uv, z, color in zip(pixels, points[:, :, 2], colors):
            x0, y0 = np.maximum(np.floor(uv.min(axis=0)).astype(int), 0)
            x1, y1 = np.minimum(np.ceil(uv.max(axis=0)).astype(int), size-1)
            if x1 < x0 or y1 < y0:
                continue
            x, y = uv[:, 0], uv[:, 1]
            denominator = (y[1]-y[2])*(x[0]-x[2])+(x[2]-x[1])*(y[0]-y[2])
            if abs(denominator) < 1e-8:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1+1)+.5, np.arange(y0, y1+1)+.5)
            a = ((y[1]-y[2])*(gx-x[2])+(x[2]-x[1])*(gy-y[2]))/denominator
            b = ((y[2]-y[0])*(gx-x[2])+(x[0]-x[2])*(gy-y[2]))/denominator
            c = 1-a-b
            zz = a*z[0]+b*z[1]+c*z[2]
            view = depth[y0:y1+1, x0:x1+1]
            valid = (a >= 0) & (b >= 0) & (c >= 0) & (zz < view)
            view[valid] = zz[valid]
            image[y0:y1+1, x0:x1+1][valid] = color
    return image


def write_ortho(urdf, package, output, joint_positions):
    visuals = actual_visuals(urdf, package, joint_positions)
    views = [([[0, -1, 0], [0, 0, -1], [-1, 0, 0]], "Front (+X)"),
             ([[-1, 0, 0], [0, 0, -1], [0, -1, 0]], "Left side (+Y)"),
             ([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], "Top (+Z)")]
    sheet = np.full((690, 1940, 3), 242, dtype=np.uint8)
    for i, (rotation, title) in enumerate(views):
        image = render_orthographic(visuals, rotation)
        cv2.putText(image, title, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 0), 1)
        cv2.line(image, (25, 610), (205, 610), (0, 0, 0), 3)
        cv2.putText(image, "10 cm", (25, 595), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1)
        sheet[:640, i*650:i*650+640] = image
    cv2.putText(sheet, "Actual expanded URDF geometry - orthographic diagnostic, NOT the Gazebo RGB photo gate", (15, 670),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 1)
    if not cv2.imwrite(str(output / "ortho.png"), sheet):
        raise OSError("Failed to save orthographic diagnostic")
    report = {"renderer": "independent CPU triangle z-buffer", "projection": "orthographic",
              "source_urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
              "joint_positions": joint_positions, "triangle_count": sum(len(v[0]) for v in visuals),
              "scope": "Actual expanded URDF visual geometry; diagnostic only, never used for Gate B image observations."}
    (output / "ortho_provenance.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
