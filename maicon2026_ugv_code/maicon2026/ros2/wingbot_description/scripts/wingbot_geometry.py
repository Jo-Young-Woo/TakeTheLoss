#!/usr/bin/env python3
"""Single geometry authority; all coordinates are metres in zero-angle base_footprint.

Photo measurements are read only from wingbot_mods.yaml. No photo keypoint answer
coordinates or world layout enter this module. FK uses URDF parent-local axes.
"""
import math
import struct
from pathlib import Path
import numpy as np
import yaml


def load_mods(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def matrix_rpy(matrix):
    m = np.asarray(matrix)
    p = math.atan2(-m[2, 0], math.hypot(m[0, 0], m[1, 0]))
    if abs(math.cos(p)) > 1e-8:
        return [math.atan2(m[2, 1], m[2, 2]), p, math.atan2(m[1, 0], m[0, 0])]
    return [math.atan2(-m[1, 2], m[1, 1]), p, 0.0]


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def origin_matrix(element):
    origin = element.find('origin')
    if origin is None:
        return np.eye(4)
    return transform([float(v) for v in origin.get('xyz', '0 0 0').split()],
                     [float(v) for v in origin.get('rpy', '0 0 0').split()])


def pose_in_parent(parent_world, desired_world):
    return np.linalg.inv(parent_world) @ desired_world


def joint_records(root):
    result = []
    for j in root.findall('joint'):
        o = origin_matrix(j)
        a = j.find('axis')
        result.append(dict(name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
                           child=j.find('child').get('link'), xyz=o[:3, 3].tolist(),
                           rpy=matrix_rpy(o[:3, :3]),
                           axis=[float(v) for v in a.get('xyz', '1 0 0').split()] if a is not None else [1., 0., 0.]))
    return result


def metadata_forward_kinematics(metadata, joint_positions=None):
    positions = joint_positions or {}
    joints = metadata['joints']
    root_names = {j['parent'] for j in joints} - {j['child'] for j in joints}
    result = {name: np.eye(4) for name in root_names}
    pending = list(joints)
    while pending:
        next_pending = []
        for j in pending:
            if j['parent'] not in result:
                next_pending.append(j)
                continue
            motion = np.eye(4)
            q = float(positions.get(j['name'], 0.))
            axis = np.asarray(j['axis'], float)
            if j['type'] in ('revolute', 'continuous', 'prismatic'):
                axis = axis / np.linalg.norm(axis)
                if j['type'] == 'prismatic':
                    motion[:3, 3] = axis * q
                else:
                    x, y, z = axis
                    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                    motion[:3, :3] = np.eye(3) + math.sin(q)*k + (1-math.cos(q))*(k@k)
            result[j['child']] = result[j['parent']] @ transform(j['xyz'], j['rpy']) @ motion
        if len(next_pending) == len(pending):
            raise ValueError('Disconnected or cyclic joint tree')
        pending = next_pending
    return result


def forward_kinematics(urdf_root, joint_positions=None):
    return metadata_forward_kinematics({'joints': joint_records(urdf_root)}, joint_positions)


def read_stl(path):
    data = Path(path).read_bytes()
    count = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84 + 50*count:
        raise ValueError(f'Expected binary vendor STL: {path}')
    dtype = np.dtype([('normal', '<3f4'), ('vertices', '<9f4'), ('attr', '<u2')])
    return np.frombuffer(data, dtype=dtype, offset=84, count=count)['vertices'].reshape(-1, 3, 3).astype(float)


def transformed_points(points, matrix):
    a = np.asarray(points)
    return (a.reshape(-1, 3) @ matrix[:3, :3].T + matrix[:3, 3]).reshape(a.shape)


def antenna_point(mods, side, z):
    a = mods['iff_antennas']
    bend = np.array(a[side]['bend'], float)
    tilt, azimuth = a['lean'][side]
    if abs(math.cos(tilt)) < .01:
        raise ValueError('Antenna tilt cannot be horizontal when height is specified')
    direction = np.array([math.sin(tilt)*math.cos(azimuth), math.sin(tilt)*math.sin(azimuth), math.cos(tilt)])
    return bend + direction * ((z-bend[2])/math.cos(tilt))


def axis_rotation(axis):
    """Rotation whose local +Z follows axis, for cylinders and domes."""
    z = np.array(axis, float)
    z /= np.linalg.norm(z)
    x = np.cross(z, [0., 0., 1.])
    if np.linalg.norm(x) < 1e-8:
        x = np.cross(z, [1., 0., 0.])
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def prism_vertices(poly, length):
    """Closed polygon extruded along local X; CCW polygon is local Y/Z."""
    rings = [np.array([[x, y, z] for y, z in poly]) for x in (-length/2, length/2)]
    out = []
    n = len(poly)
    for i in range(n):
        k = (i+1) % n
        a, b, c, d = rings[0][i], rings[0][k], rings[1][k], rings[1][i]
        out.extend([[a, b, c], [a, c, d]])
    for i in range(1, n-1):
        out.extend([[rings[0][0], rings[0][i+1], rings[0][i]], [rings[1][0], rings[1][i], rings[1][i+1]]])
    return np.array(out)


def octagon(width, height, chamfer, length):
    w, h, c = width/2, height/2, chamfer
    return prism_vertices([(w,h-c),(w-c,h),(-w+c,h),(-w,h-c),(-w,-h+c),(-w+c,-h),(w-c,-h),(w,-h+c)], length)


def dome(radius, height_ratio, lat=8, lon=24):
    tris = []
    def p(t, a):
        return [radius*math.sin(t)*math.cos(a), radius*math.sin(t)*math.sin(a), radius*height_ratio*math.cos(t)]
    for i in range(lat):
        t0, t1 = math.pi*i/(2*lat), math.pi*(i+1)/(2*lat)
        for k in range(lon):
            a0, a1 = 2*math.pi*k/lon, 2*math.pi*(k+1)/lon
            a,b,c,d = p(t1,a0),p(t1,a1),p(t0,a1),p(t0,a0)
            tris.append([a,b,c])
            if i:
                tris.append([a,c,d])
    # Close the base: physical visual is opaque from either side.
    for k in range(lon):
        tris.append([[0.,0.,0.], p(math.pi/2, 2*math.pi*(k+1)/lon), p(math.pi/2, 2*math.pi*k/lon)])
    return np.asarray(tris)


def write_obj(path, triangles):
    """Use explicit normals and indexed vertices; no material/texture dependencies."""
    lines = ['# Generated WINGBOT primitive visual, metres; collision never uses this mesh.']
    for tri in triangles:
        for vertex in tri:
            lines.append('v ' + ' '.join(f'{v:.12g}' for v in vertex))
    for tri in triangles:
        normal = np.cross(tri[1]-tri[0], tri[2]-tri[0])
        normal /= max(np.linalg.norm(normal), 1e-15)
        lines.append('vn ' + ' '.join(f'{v:.12g}' for v in normal))
    for i in range(len(triangles)):
        lines.append('f ' + ' '.join(f'{3*i+k+1}//{i+1}' for k in range(3)))
    Path(path).write_text('\n'.join(lines)+'\n', encoding='utf-8')
