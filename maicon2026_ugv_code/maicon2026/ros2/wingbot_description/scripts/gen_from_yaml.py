#!/usr/bin/env python3
"""Generate WINGBOT xacro properties, bounded visual primitives and referee FK.

Run again after changing measured wingbot_mods.yaml. Never edits that input file.
The camera CAD frame is deliberately re-anchored at its specified optical lens.
"""
import argparse
import copy
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from wingbot_geometry import (antenna_point, axis_rotation, dome, forward_kinematics,
                              joint_records, load_mods, matrix_rpy, octagon,
                              origin_matrix, pose_in_parent, prism_vertices,
                              read_stl, transform, transformed_points, write_obj)

X = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', X)


def fmt(values):
    return ' '.join(f'{float(v):.12g}' for v in values)


def origin(parent, pose):
    return ET.SubElement(parent, 'origin', xyz=fmt(pose[:3,3]), rpy=fmt(matrix_rpy(pose[:3,:3])))


def inertial(link, mass, size, pose=None, cylinder=False):
    node = ET.SubElement(link, 'inertial')
    origin(node, np.eye(4) if pose is None else pose)
    ET.SubElement(node, 'mass', value=f'{mass:.12g}')
    if cylinder:
        r, length = size
        values = [mass*(3*r*r+length*length)/12]*2 + [mass*r*r/2]
    else:
        x,y,z = size
        values = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
    ET.SubElement(node, 'inertia', ixx=str(values[0]), iyy=str(values[1]), izz=str(values[2]), ixy='0', ixz='0', iyz='0')


def horizontal_segments(triangles, height):
    """Exact visual triangle intersections with a horizontal LiDAR plane."""
    segments = []
    for triangle in triangles:
        hits = []
        for start, end in zip(triangle, np.roll(triangle, -1, axis=0)):
            a, b = start[2]-height, end[2]-height
            if abs(a) < 1e-10:
                hits.append(start[:2])
            if a*b < 0:
                hits.append((start + (end-start)*(-a/(b-a)))[:2])
        if len(hits) >= 2:
            pairs = [(a, b) for index, a in enumerate(hits) for b in hits[index+1:]]
            a, b = max(pairs, key=lambda pair: np.linalg.norm(pair[0]-pair[1]))
            if np.linalg.norm(a-b) > 1e-10:
                segments.append([a.tolist(), b.tolist()])
    return segments


def generate(package):
    package = Path(package)
    mods_path = package/'config/wingbot_mods.yaml'
    mods = load_mods(mods_path)
    robot = ET.parse(package/'vendor/ugv_rover.urdf').getroot()
    original = copy.deepcopy(robot)
    joints = {j.get('name'):j for j in robot.findall('joint')}
    lidar_joint = joints['base_lidar_link_joint']
    lidar_origin = lidar_joint.find('origin')
    xyz = [float(v) for v in lidar_origin.get('xyz').split()]
    xyz[2] += mods['lidar']['z_offset_from_base']
    lidar_origin.set('xyz', fmt(xyz))
    # The CAD camera link origin is not the lens. A local-z subtraction would
    # move laterally because pt_link2 has roll ~= pi/2. Preserve its name/parent
    # but use the supplied physical lens zero-world transform as authority.
    zero = forward_kinematics(robot)
    cj = joints['pt_link2_to_pt_camera_link']
    cj.remove(cj.find('origin'))
    origin(cj, pose_in_parent(zero[mods['camera']['parent']], transform(mods['camera']['lens_xyz'])))
    for key in ('pan', 'tilt'):
        joint = joints[mods['pan_tilt'][key+'_joint']]
        lim = mods['pan_tilt'][key+'_limit_rad']
        joint.find('limit').attrib.update(lower=str(-lim),upper=str(lim), effort='3.0', velocity=str(mods['pan_tilt']['max_speed_rad_s']))
        ET.SubElement(joint, 'dynamics', damping='0.02', friction='0.005')
    # Middle wheels clone the vendor front-wheel frame/visual exactly, translated
    # along the zero-world x axis. Original four joint origins remain byte-value equivalent.
    for side in ('left','right'):
        src = next(l for l in robot.findall('link') if l.get('name')==f'{side}_up_wheel_link')
        link = copy.deepcopy(src)
        link.set('name',f'{side}_middle_wheel_link')
        robot.append(link)
        joint = copy.deepcopy(joints[f'{side}_up_wheel_link_joint'])
        joint.set('name',f'{side}_middle_wheel_link_joint')
        joint.find('child').set('link',link.get('name'))
        values = [float(v) for v in joint.find('origin').get('xyz').split()]
        values[0] += mods['wheels']['x'][1]-mods['wheels']['x'][0]
        joint.find('origin').set('xyz',fmt(values))
        robot.append(joint)
    links = {l.get('name'):l for l in robot.findall('link')}
    for name, link in links.items():
        for tag in ('collision','inertial'):
            for node in link.findall(tag):
                link.remove(node)
        # Invalid link-level origins in the vendor model have no URDF meaning.
        for node in link.findall('origin'):
            link.remove(node)
        for visual in link.findall('visual'):
            if name == 'pt_camera_link':
                link.remove(visual)
                continue
            mesh = visual.find('geometry/mesh')
            mesh.set('filename', mesh.get('filename').replace('package://ugv_description/', 'package://wingbot_description/'))
            color = '.07 .07 .07 1' if 'wheel' in name else '.17 .17 .18 1' if name=='base_link' else '.11 .11 .12 1'
            visual.find('material').set('name', name+'_material')
            visual.find('material/color').set('rgba',color)
    zero = forward_kinematics(robot)
    mass = mods['mass_distribution']
    inertial(links['base_link'],mass['base_link'],mods['collision_primitives']['chassis_box']['size'])
    for name, link in links.items():
        if 'wheel_link' in name:
            inertial(link,mass['wheels_each'],[mods['wheels']['radius'],mods['wheels']['width']],cylinder=True)
    # Allocate head mass to both moving joints; these ratios affect inertia only,
    # not dimensions. Fixed hardware receives the remaining specified mass.
    inertial(links['pt_link1'],mass['pan_tilt_head']*2/7,[.05,.05,.043])
    h = mods['collision_primitives']['head_box']
    inertial(links['pt_link2'],mass['pan_tilt_head']*5/7,h['size'],pose_in_parent(zero['pt_link2'],transform(h['center'])))
    for name, fraction, size in [('pt_base_link',2/3,[.06,.06,.04]),('base_lidar_link',.2,[.05,.05,.035]),('3d_camera_link',2/15,[.018,.09,.022])]:
        inertial(links[name],mass['others']*fraction,size)
    collision_boxes = []
    for key in ('chassis_box','deck_box','head_box'):
        b=mods['collision_primitives'][key]
        parent=b.get('parent','base_link')
        pose=pose_in_parent(zero[parent],transform(b['center']))
        col=ET.SubElement(links[parent],'collision',name=key)
        origin(col,pose)
        ET.SubElement(ET.SubElement(col,'geometry'),'box',size=fmt(b['size']))
        collision_boxes.append(dict(name=key,parent=parent,local_xyz=pose[:3,3].tolist(),local_rpy=matrix_rpy(pose[:3,:3]),size=b['size']))
    for name,link in links.items():
        if 'wheel_link' in name:
            col=ET.SubElement(link,'collision',name=name+'_cylinder')
            origin(col,np.eye(4))
            ET.SubElement(ET.SubElement(col,'geometry'),'cylinder',radius=str(mods['wheels']['radius']),length=str(mods['wheels']['width']))
    generated=package/'meshes/generated'
    generated.mkdir(parents=True,exist_ok=True)
    # Finite, explicit tessellation keeps the added-visual budget reproducible.
    # Cylinders are ordinary cylinders, represented as low-poly visual OBJ only;
    # physics uses native cylinders. Dome is 8x24, tiny hinge balls are 3x12/half.
    circle=[(math.cos(2*math.pi*k/16),math.sin(2*math.pi*k/16)) for k in range(16)]
    unit_cylinder=prism_vertices(circle,1.)
    # Rotate extrusion +X to +Z for standard cylinder geometry.
    unit_cylinder=transformed_points(unit_cylinder,transform(rpy=(0,-math.pi/2,0)))
    write_obj(generated/'cylinder16.obj',unit_cylinder)
    sphere=np.concatenate([dome(1.,1.,3,12),transformed_points(dome(1.,1.,3,12),transform(rpy=(math.pi,0,0)))])
    write_obj(generated/'hinge_sphere.obj',sphere)
    pieces=[]
    total=0
    serial=0
    def add(name,parent,pose,shape,color,team=False):
        nonlocal total,serial
        serial+=1
        node=ET.SubElement(links[parent],'visual',name=f'{name}_{serial}')
        origin(node,pose_in_parent(zero[parent],pose))
        geo=ET.SubElement(node,'geometry')
        if shape[0]=='box':
            ET.SubElement(geo,'box',size=fmt(shape[1])); count=12
        else:
            uri,scale,count=shape[1:]
            ET.SubElement(geo,'mesh',filename='package://wingbot_description/meshes/generated/'+uri,scale=fmt(scale))
        mat=ET.SubElement(node,'material',name=f'{name}_{serial}_material')
        rgba="${'"+fmt(mods['iff_antennas']['team_color']['BLUE'])+" 1' if team == 'BLUE' else '"+fmt(mods['iff_antennas']['team_color']['RED'])+" 1'}" if team else fmt(list(color)+[1.])
        ET.SubElement(mat,'color',rgba=rgba)
        total+=count
        pieces.append(dict(name=name,parent=parent,triangles=count))
    def box(name,parent,center,size,color,rpy=(0,0,0)):
        add(name,parent,transform(center,rpy),('box',size),color)
    def cylinder(name,parent,p0,p1,r,color,team=False):
        p0,p1=np.asarray(p0,float),np.asarray(p1,float)
        t=transform((p0+p1)/2)
        t[:3,:3]=axis_rotation(p1-p0)
        add(name,parent,t,('mesh','cylinder16.obj',[r,r,float(np.linalg.norm(p1-p0))],len(unit_cylinder)),color,team)
    def mesh(name,parent,center,triangles,color,rotation=None):
        filename=name+'.obj'
        write_obj(generated/filename,triangles)
        t=transform(center)
        if rotation is not None:t[:3,:3]=rotation
        add(name,parent,t,('mesh',filename,[1.,1.,1.],len(triangles)),color)
    pm=mods['pan_mast']; cx,cy=pm['center_xy']
    for key,color in [('cylinder',[.08,.08,.09]),('ring',pm['ring']['color'])]:
        part=pm[key]
        cylinder('mast_'+key,'base_link',[cx,cy,part['z'][0]],[cx,cy,part['z'][1]],part['radius'],color)
    pl=pm['pillars']
    for k in range(pl['count']):
        a=math.pi/4+2*math.pi*k/pl['count']; x,y=cx+pl['ring_radius']*math.cos(a),cy+pl['ring_radius']*math.sin(a)
        cylinder('mast_pillar','base_link',[x,y,pl['z'][0]],[x,y,pl['z'][1]],pl['radius'],pl['color'])
    laser=mods['laser_emitter']; parent=laser['parent']; h=laser['housing']; hs=h['size']; c=np.array(h['center'])
    mesh('laser_housing',parent,c,octagon(hs[1],hs[2],h['chamfer'],hs[0]),[.09,.09,.10])
    ring=h['clamp_ring_x']; rc=c.copy();rc[0]=sum(ring)/2
    mesh('laser_clamp',parent,rc,octagon(hs[1]+.004,hs[2]+.004,h['chamfer'],ring[1]-ring[0]),[.06,.06,.06])
    lens=np.array(laser['lens_xyz'])
    cylinder('laser_bezel',parent,lens-[laser['bezel']['depth'],0,0],lens+[.001,0,0],laser['bezel']['radius'],[.04,.04,.05])
    cylinder('laser_glass',parent,lens+[.0005,0,0],lens+[.0016,0,0],laser['glass']['radius'],[.15,.22,.55])
    camera=mods['camera']; parent=camera['parent']; c=np.array(camera['lens_xyz']); b=camera['board']
    box('camera_board',parent,b['center'],b['size'],[.08,.08,.08])
    cylinder('camera_barrel',parent,c-[camera['barrel']['length'],0,0],c,camera['barrel']['radius'],[.06,.06,.07])
    cylinder('camera_lens',parent,c,c+[.001,0,0],camera['barrel']['radius']*.6,[.15,.22,.55])
    box('camera_holder',parent,c+[-.004,.004,-.009],[.008,.010,.006],[.72,.58,.30])
    led=np.array(camera['led']['center'])
    cylinder('camera_led',parent,led-[.002,0,0],led+[.001,0,0],camera['led']['radius'],[.97,.97,.97])
    rv=mods['receivers']
    for side in ('left','right'):
        c=np.array(rv[side]['center']);ax=np.array(rv[side]['facing'],float);ax/=np.linalg.norm(ax)
        box('receiver_'+side+'_mount','base_link',c+[0,-np.sign(c[1])*(rv['mount_box'][1]/2+.004),-.004],rv['mount_box'],[.06,.06,.06])
        sk0=c-ax*rv['skirt_height']*.8;sk1=c+ax*rv['skirt_height']*.2
        cylinder('receiver_'+side+'_skirt','base_link',sk0,sk1,rv['dome_radius'],[.93,.93,.97])
        mesh('receiver_'+side+'_dome','base_link',sk1,dome(rv['dome_radius'],rv['dome_height_ratio']),[.96,.96,1.],axis_rotation(ax))
    ant=mods['iff_antennas']
    for side in ('left','right'):
        bs=np.array(ant[side]['base']);bd=np.array(ant[side]['bend']);tip=antenna_point(mods,side,ant['tip_z'])
        cylinder('iff_'+side+'_hinge','base_link',bs-[0,0,.006],bs+[0,0,.004],ant['hinge_radius'],[.05,.05,.05])
        cylinder('iff_'+side+'_bend','base_link',bs,bd,ant['rod_radius']+.001,[.05,.05,.05])
        add('iff_'+side+'_ball','base_link',transform(bd),('mesh','hinge_sphere.obj',[ant['rod_radius']+.0015]*3,len(sphere)),[.05,.05,.05])
        cylinder('iff_'+side+'_rod','base_link',bd,tip,ant['rod_radius'],[.04,.04,.04])
        s0,s1=ant['sleeve']['z']
        cylinder('iff_'+side+'_sleeve','base_link',antenna_point(mods,side,s0),antenna_point(mods,side,s1),ant['sleeve']['radius'],None,True)
        for z in ant['sleeve']['bands_z']:
            half=ant['sleeve']['band_height']/2
            cylinder('iff_'+side+'_band','base_link',antenna_point(mods,side,z-half),antenna_point(mods,side,z+half),ant['sleeve']['radius']+.0004,[.03,.03,.03])
    p=mods['plaque'];box('plaque','base_link',p['center'],p['size'],p['color'],(0,-p['pitch'],0))
    for side,c in zip(('left','right'),mods['headlights']['centers']):
        c=np.array(c);cylinder('headlight_'+side,'base_link',c-[.003,0,0],c+[.002,0,0],mods['headlights']['radius'],[.97,.97,.97])
    if total>3000:raise ValueError(f'Added visual budget exceeded: {total}')
    def fixed(name,parent,world_pose):
        link=ET.SubElement(robot,'link',name=name);links[name]=link
        joint=ET.SubElement(robot,'joint',name=name+'_fixed',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=name)
        origin(joint,pose_in_parent(zero[parent],world_pose))
        zero[name]=world_pose
    fixed('laser_link',mods['laser_emitter']['parent'],transform(mods['laser_emitter']['lens_xyz']))
    fixed('pt_camera_optical_link','pt_camera_link',transform(camera['lens_xyz'],(-math.pi/2,0,-math.pi/2)))
    # Vendor SDF explicitly distinguishes the housing mount from its ray plane.
    # Preserve that local pose rather than inventing a scanner inside the deck.
    vendor_sdf=ET.parse(package/'vendor/ugv_rover.gazebo.sdf').getroot()
    vendor_scan=next(s for s in vendor_sdf.findall('.//sensor') if s.get('name')=='hls_lfcd_lds')
    scan_values=[float(v) for v in vendor_scan.findtext('pose').split()]
    fixed('lidar_scan_link','base_lidar_link',zero['base_lidar_link']@transform(scan_values[:3],scan_values[3:]))
    photo={}
    def landmark(key,name,parent,xyz,rpy=(0,0,0)):
        fixed(name,parent,transform(xyz,rpy));photo[key]={'frame':name,'local_xyz':[0.,0.,0.]}
    photo['laser_lens_center']={'frame':'laser_link','local_xyz':[0.,0.,0.]}
    photo['camera_lens_center']={'frame':'pt_camera_link','local_xyz':[0.,0.,0.]}
    landmark('camera_led','camera_led_link',camera['parent'],camera['led']['center'])
    for side,c in zip(('left','right'),mods['headlights']['centers']):
        landmark('headlight_'+side,'headlight_'+side+'_link','base_link',c)
    for side in ('left','right'):
        landmark('receiver_'+side,'receiver_'+side+'_link','base_link',rv[side]['center'])
        landmark('iff_'+side+'_tip','iff_'+side+'_tip_link','base_link',antenna_point(mods,side,ant['tip_z']))
        for end,z in zip(('bottom','top'),ant['sleeve']['z']):
            landmark('iff_'+side+'_sleeve_'+end,'iff_'+side+'_sleeve_'+end+'_link','base_link',antenna_point(mods,side,z))
    # Photo landmarks use actual vendor STL bounds, not photo_keypoints.model_xyz.
    for source,key,name,axis,end in [('3d_camera_link','depth_camera_front_ctr','depth_camera_front_link',0,1),('base_lidar_link','lidar_top_center','lidar_top_link',2,1)]:
        v=links[source].find('visual')
        points=transformed_points(read_stl(package/'meshes/ugv_rover'/Path(v.find('geometry/mesh').get('filename')).name),zero[source]@origin_matrix(v)).reshape(-1,3)
        lo,hi=points.min(0),points.max(0);point=(lo+hi)/2;point[axis]=(lo,hi)[end][axis]
        landmark(key,name,source,point)
    joints_data=joint_records(robot)
    frames={}
    for j in joints_data:
        w=zero.get(j['child'])
        if w is None:w=forward_kinematics(robot)[j['child']]
        frames[j['child']]=dict(xyz=w[:3,3].tolist(),rpy=matrix_rpy(w[:3,:3]),parent=j['parent'],local_xyz=j['xyz'],local_rpy=j['rpy'])
    for side in ('left','right'):
        frames['receiver_'+side+'_link'].update(radius=rv['dome_radius'],facing=rv[side]['facing'],acceptance=rv['acceptance'])
    frames['laser_link']['axis']=[1.,0.,0.]
    # Export the measured assembled visual bounds, including the added hardware.
    all_points=[]
    self_sections=[]
    parent_joints={j['child']:j for j in joints_data}
    def is_static(name):
        while name in parent_joints:
            joint=parent_joints[name]
            if joint['type'] != 'fixed':return False
            name=joint['parent']
        return name == 'base_footprint'
    scan_height=float(zero['lidar_scan_link'][2,3])
    for name,link in links.items():
        for v in link.findall('visual'):
            geo=v.find('geometry'); mesh_node=geo.find('mesh'); box_node=geo.find('box')
            if box_node is not None:
                size=np.array([float(a) for a in box_node.get('size').split()])
                points=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*size
                triangles=prism_vertices([(-size[1]/2,-size[2]/2),(size[1]/2,-size[2]/2),
                                          (size[1]/2,size[2]/2),(-size[1]/2,size[2]/2)],size[0])
            elif mesh_node is not None:
                filename=package/mesh_node.get('filename').split('package://wingbot_description/',1)[1]
                if filename.suffix.lower()=='.stl':
                    triangles=read_stl(filename)
                    points=triangles.reshape(-1,3)
                else:
                    lines=filename.read_text().splitlines()
                    points=np.array([[float(a) for a in line.split()[1:4]] for line in lines if line.startswith('v ')])
                    faces=np.array([[int(a.split('/')[0])-1 for a in line.split()[1:4]] for line in lines if line.startswith('f ')])
                    triangles=points[faces]
                scale=np.array([float(a) for a in mesh_node.get('scale','1 1 1').split()])
                points=points*scale
                triangles=triangles*scale
            else:continue
            pose=zero[name]@origin_matrix(v)
            all_points.append(transformed_points(points,pose))
            if is_static(name):
                segments=horizontal_segments(transformed_points(triangles,pose),scan_height)
                if segments:self_sections.append(dict(frame='base_footprint',z=scan_height,
                    source=name+'/'+v.get('name','vendor_visual'),segments=segments))
    points=np.concatenate(all_points);lo,hi=points.min(0),points.max(0)
    metadata=dict(schema='wingbot_frames_v1',source_sha256=hashlib.sha256(mods_path.read_bytes()).hexdigest(),root='base_footprint',joints=joints_data,frames=frames,
                  semantic=dict(laser='laser_link',camera='pt_camera_link',camera_optical='pt_camera_optical_link',receiver_left='receiver_left_link',receiver_right='receiver_right_link',iff_left_tip='iff_left_tip_link',iff_right_tip='iff_right_tip_link',lidar='lidar_scan_link',imu='base_imu_link'),
                  photo_keypoints=photo,pan_tilt=dict(pan_joint=mods['pan_tilt']['pan_joint'],tilt_joint=mods['pan_tilt']['tilt_joint'],pan_api_sign=1,tilt_api_sign=1),
                  lidar=dict(crop_deg=mods['lidar']['crop_deg'],crop_frame='lidar_scan_link',mount_frame='base_lidar_link',scan_plane_local_xyz=scan_values[:3],scan_plane_source='vendor/ugv_rover.gazebo.sdf:hls_lfcd_lds/pose',scan_plane_physical_verified=False,ros_angle_sign=-1,crop_audited=True,crop_input_convention='vendor_clockwise_degrees',yaw_source='description_joint',description_yaw_rad=1.5708,gazebo_legacy_yaw_rad=0.,audit_sources=['ugv_else/ldlidar/src/demo.cpp:199-215','ugv_else/ldlidar/launch/stl27l.launch.py:36'],physical_yaw_verified=False),
                  footprint_lwh=(hi-lo).tolist(),visual_bounds=dict(min=lo.tolist(),max=hi.tolist()),collision_boxes=collision_boxes,added_visual_triangles=total,added_visual_parts=pieces,
                  camera_frame_reanchor=dict(original_parent_local_xyz=origin_matrix(next(j for j in original.findall('joint') if j.get('name')=='pt_link2_to_pt_camera_link'))[:3,3].tolist(),new_parent_local_xyz=origin_matrix(cj)[:3,3].tolist(),new_parent_local_rpy=matrix_rpy(origin_matrix(cj)[:3,:3]),reason='CAD frame is not physical lens; literal local-z decrement moves world Y. YAML lens and +X boresight are authoritative.'),
                  topics={key:'{namespace}/'+value for key,value in dict(cmd_vel='cmd_vel',odom='odom',tf='tf',joint_states='joint_states',pan='pan/cmd_pos',tilt='tilt/cmd_pos',image='camera/image',camera_info='camera/camera_info',scan='scan',imu='imu').items()})
    # The rendered mast crosses the ray plane. Export its physical surfaces so
    # the sensor adapter can reject only returns on the robot itself.
    metadata['lidar']['self_return_cylinders'] = [
        dict(frame='base_footprint', center_xy=[cx+pl['ring_radius']*math.cos(math.pi/4+2*math.pi*k/pl['count']),
                                              cy+pl['ring_radius']*math.sin(math.pi/4+2*math.pi*k/pl['count'])],
             radius=pl['radius'], z=pl['z'], source='pan_mast.pillars') for k in range(pl['count'])]
    metadata['lidar']['self_return_sections'] = self_sections
    (package/'config/frames.yaml').write_text(yaml.safe_dump(metadata,sort_keys=False,allow_unicode=True),encoding='utf-8')
    (package/'frames.yaml').write_bytes((package/'config/frames.yaml').read_bytes())
    # Store a plain URDF for independent numeric review; xacro is the runtime entry.
    ET.indent(robot,space='  ')
    ET.ElementTree(robot).write(package/'urdf/wingbot.generated.urdf',encoding='utf-8',xml_declaration=True)
    macro_root=ET.Element('robot',name='wingbot')
    macro=ET.SubElement(macro_root,'{'+X+'}macro',name='wingbot_body',params='prefix team wheels')
    for node in list(robot):
        node=copy.deepcopy(node)
        for item in node.iter():
            if item.tag in ('link','joint') and 'name' in item.attrib:item.set('name','${prefix}'+item.get('name'))
            if item.tag in ('parent','child'):item.set('link','${prefix}'+item.get('link'))
        macro.append(node)
    ET.indent(macro_root,space='  ')
    ET.ElementTree(macro_root).write(package/'urdf/wingbot_props.xacro',encoding='utf-8',xml_declaration=True)
    return metadata


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package-root',type=Path,default=Path(__file__).resolve().parents[1])
    args=parser.parse_args()
    result=generate(args.package_root)
    print(f"Generated {result['added_visual_triangles']} added triangles; input SHA256 {result['source_sha256']}")


if __name__=='__main__':main()

