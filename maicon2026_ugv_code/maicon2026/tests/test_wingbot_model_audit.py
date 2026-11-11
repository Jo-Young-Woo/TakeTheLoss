"""Gate A must reject broken real expanded URDFs, not merely read its own metadata."""
import copy
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


PACKAGE=Path(__file__).resolve().parents[1]/'ros2/wingbot_description'
spec=importlib.util.spec_from_file_location('wingbot_independent_check',PACKAGE/'scripts/check_model.py')
checker=importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


@pytest.fixture(scope='module')
def expanded():
    xacro=pytest.importorskip('xacro',reason='Actual ROS xacro expansion is required for model contract tests')
    text=xacro.process_file(str(PACKAGE/'urdf/wingbot.urdf.xacro'),mappings={'team':'BLUE','sensors':'full','wheels':'6'}).toxml()
    return ET.fromstring(text)


def run_audit(tmp_path,root,**kwargs):
    path=tmp_path/'actual_model.urdf'
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return checker.audit(path,PACKAGE/'config/wingbot_mods.yaml',PACKAGE,**kwargs)


def test_actual_generated_full_model_passes_independent_gate_a(tmp_path,expanded):
    result=run_audit(tmp_path,expanded)
    assert result['passed'],[c for c in result['checks'] if not c['passed']]
    checks={c['name']:c for c in result['checks']}
    assert checks['collision_budget']['actual']==9
    assert checks['added_visual_triangle_budget_24_segments']['actual']==2792
    assert checks['total_mass_kg']['actual']==pytest.approx(2.9)


@pytest.mark.parametrize('mutation,expected_failure',[
    ('reverse_pan_axis','joint_sign:pan'),
    ('move_camera_centimetre','frame_position:pt_camera_link'),
    ('add_mesh_collision','mesh_collisions'),
    ('wrong_drive_radius','drive_radius'),
    ('missing_camera','full_sensor_types'),
    ('add_light','no_lights'),
    ('wrong_optical_rotation','optical_axes'),
    ('sensor_in_dummy','dummy_has_no_sensors'),
])
def test_gate_a_rejects_specific_broken_model(tmp_path,expanded,mutation,expected_failure):
    root=copy.deepcopy(expanded)
    kwargs={}
    if mutation=='reverse_pan_axis':
        root.find("joint[@name='pt_base_link_to_pt_link1']/axis").set('xyz','0 0 1')
    elif mutation=='move_camera_centimetre':
        node=root.find("joint[@name='pt_link2_to_pt_camera_link']/origin")
        xyz=[float(v) for v in node.get('xyz').split()];xyz[0]+=.01
        node.set('xyz',' '.join(map(str,xyz)))
    elif mutation=='add_mesh_collision':
        col=ET.SubElement(root.find("link[@name='base_link']"),'collision')
        ET.SubElement(ET.SubElement(col,'geometry'),'mesh',filename='package://wingbot_description/meshes/ugv_rover/base_link.stl')
    elif mutation=='wrong_drive_radius':
        root.find('.//plugin/wheel_radius').text='.025'
    elif mutation=='missing_camera':
        parent=next(g for g in root.findall('gazebo') if g.find("sensor[@type='camera']") is not None)
        parent.remove(parent.find("sensor[@type='camera']"))
    elif mutation=='add_light':
        ET.SubElement(ET.SubElement(root,'gazebo'),'light',name='forbidden',type='point')
    elif mutation=='wrong_optical_rotation':
        root.find("joint[@name='pt_camera_optical_link_fixed']/origin").set('rpy','0 0 0')
    elif mutation=='sensor_in_dummy':
        kwargs['sensors']='none'
    result=run_audit(tmp_path,root,**kwargs)
    assert not result['passed']
    assert expected_failure in {c['name'] for c in result['checks'] if not c['passed']}


def test_two_motor_dummy_keeps_six_wheel_physics_without_sensors(tmp_path):
    xacro=pytest.importorskip('xacro')
    root=ET.fromstring(xacro.process_file(str(PACKAGE/'urdf/wingbot.urdf.xacro'),mappings={'team':'RED','sensors':'none','wheels':'2','prefix':'test_'}).toxml())
    result=run_audit(tmp_path,root,wheels=2,sensors='none',prefix='test_')
    assert result['passed'],[c for c in result['checks'] if not c['passed']]
    assert next(c for c in result['checks'] if c['name']=='wheel_count')['actual']==6
    drive=next(p for p in root.findall('.//plugin') if 'DiffDrive' in p.get('name',''))
    assert [n.text for n in drive.findall('left_joint')]==['test_left_middle_wheel_link_joint']
    assert not root.findall('.//sensor')


def test_vendored_measurements_and_photo_inputs_match_source_manifest():
    import hashlib
    import json
    manifest=json.loads((PACKAGE/'vendor/source_manifest.json').read_text())
    for item in manifest['files']:
        raw=(PACKAGE/item['path']).read_bytes()
        assert len(raw)==item['bytes']
        assert hashlib.sha256(raw).hexdigest()==item['sha256'],item['path']
    # Geometry generation is forbidden from importing the photo answer model_xyz.
    code=(PACKAGE/'scripts/gen_from_yaml.py').read_text(encoding='utf-8-sig')
    assert "load_mods(mods_path)" in code
    assert 'photo_keypoints.yaml' not in code


def test_skid_friction_direction_is_invariant_under_wheel_spin(expanded):
    import numpy as np
    zero=checker.forward(expanded)
    turned=checker.forward(expanded,{'left_up_wheel_link_joint':1.3})
    settings=expanded.find("gazebo[@reference='left_up_wheel_link']")
    axis=np.array([float(v) for v in settings.findtext('fdir1').split()])
    assert np.allclose(zero['left_up_wheel_link'][:3,:3]@axis,turned['left_up_wheel_link'][:3,:3]@axis)
    assert abs((zero['left_up_wheel_link'][:3,:3]@axis)[1])>0.999
    assert float(settings.findtext('mu1')) < float(settings.findtext('mu2'))
