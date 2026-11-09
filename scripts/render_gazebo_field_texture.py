"""Rasterize the MAICON floor paint into one textured mesh quad.

Gazebo Fortress with the Ogre 1 software renderer loads the texture from the
COLLADA mesh.  The exported world then needs one floor visual instead of a
visual for every grid line and letter pixel.
"""
from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from maicon26.config import Config  # noqa: E402
from scripts.generate_gazebo_world import track_markings  # noqa: E402


def render(png: Path, dae: Path, pixels_per_metre: int = 400) -> int:
    g = replace(Config().arena, base_depth=.4, base_height=1.2)
    width, height = round(g.W * pixels_per_metre), round(g.H * pixels_per_metre)
    image = Image.new("RGB", (width, height), (18, 21, 31))
    draw = ImageDraw.Draw(image)
    node = ET.Element("world")
    markings = track_markings(node, g)
    count = 0
    for visual in markings.findall("./link/visual"):
        x, y, _, _, _, yaw = (float(v) for v in visual.findtext("pose").split())
        sx, sy, _ = (float(v) for v in visual.findtext("geometry/box/size").split())
        color = [float(v) for v in visual.findtext("material/diffuse").split()[:3]]
        rgb = tuple(round(255 * max(0, min(1, v))) for v in color)
        c, s = math.cos(yaw), math.sin(yaw)
        points = []
        for dx, dy in ((-sx/2, -sy/2), (sx/2, -sy/2),
                       (sx/2, sy/2), (-sx/2, sy/2)):
            wx, wy = x + c*dx - s*dy, y + s*dx + c*dy
            points.append((wx/g.W*width, (g.H-wy)/g.H*height))
        draw.polygon(points, fill=rgb)
        count += 1
    png.parent.mkdir(parents=True, exist_ok=True)
    image.save(png, optimize=True)

    # The upward-facing quad is a render-only mesh. Collision remains the
    # original floor box, so the texture has no effect on robot physics.
    x, y = g.W/2, g.H/2
    collada = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><contributor><authoring_tool>MAICON floor rasterizer</authoring_tool></contributor>
    <unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="field_image"><init_from>{png.name}</init_from></image></library_images>
  <library_effects><effect id="field_effect"><profile_COMMON>
    <newparam sid="field_surface"><surface type="2D"><init_from>field_image</init_from></surface></newparam>
    <newparam sid="field_sampler"><sampler2D><source>field_surface</source></sampler2D></newparam>
    <technique sid="common"><phong>
      <ambient><color>1 1 1 1</color></ambient>
      <diffuse><texture texture="field_sampler" texcoord="UVSET0"/></diffuse>
      <specular><color>0 0 0 1</color></specular>
    </phong></technique>
  </profile_COMMON></effect></library_effects>
  <library_materials><material id="field_material"><instance_effect url="#field_effect"/></material></library_materials>
  <library_geometries><geometry id="field_geometry"><mesh>
    <source id="positions"><float_array id="positions_array" count="12">{-x} {-y} 0 {x} {-y} 0 {x} {y} 0 {-x} {y} 0</float_array>
      <technique_common><accessor source="#positions_array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="texcoords"><float_array id="texcoords_array" count="8">0 0 1 0 1 1 0 1</float_array>
      <technique_common><accessor source="#texcoords_array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
    <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
    <triangles material="field_symbol" count="2"><input semantic="VERTEX" source="#vertices" offset="0"/>
      <input semantic="TEXCOORD" source="#texcoords" offset="1" set="0"/>
      <p>0 0 1 1 2 2 0 0 2 2 3 3</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene"><node id="field_node">
    <instance_geometry url="#field_geometry"><bind_material><technique_common>
      <instance_material symbol="field_symbol" target="#field_material">
        <bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/>
      </instance_material></technique_common></bind_material></instance_geometry>
  </node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>'''
    dae.write_text(collada, encoding="utf-8")
    obj = dae.with_suffix(".obj")
    mtl = dae.with_suffix(".mtl")
    obj.write_text(f"""mtllib {mtl.name}
o MAICON_field_markings
v {-x} {-y} 0
v {x} {-y} 0
v {x} {y} 0
v {-x} {y} 0
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 0 0 1
usemtl MAICON_field_markings
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
""", encoding="utf-8")
    mtl.write_text(f"""newmtl MAICON_field_markings
Ka 1 1 1
Kd 1 1 1
Ks 0 0 0
map_Kd {png.name}
""", encoding="utf-8")
    return count


if __name__ == "__main__":
    assets = ROOT / "ros2" / "gazebo" / "assets"
    n = render(assets / "field_markings.png", assets / "field_markings.dae")
    print(f"Rasterized {n} floor visuals into field_markings.png / field_markings.obj")
