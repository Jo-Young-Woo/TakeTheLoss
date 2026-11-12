"""Record native UAV RGB and a synchronized, image-built face inventory."""
import json
import math
import threading
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from maicon26.arena import ArenaGeometry
from maicon26.hal.fpv_recording import SimulationVideoWriter
from maicon26.perception.uav_calibration import load_calibration
from maicon26.perception.uav_face_ledger import UavImageFaceLedger, draw_uav_image_ledger
from run_wingbot_autonomous_trial import CameraRecorder, write_json


def prepare_uav_recording_profile(output, share, fps=30.):
    """Co-located real render camera; the Agent's UAV sensor stays at 1 Hz."""
    if not math.isfinite(fps) or not 2 <= fps <= 30:
        raise ValueError('UAV recording rate must be between 2 and 30 Hz')
    share, output = Path(share), Path(output)
    profile = yaml.safe_load((share/'config/profiles/wingbot_recording.yaml').read_text())
    spec = yaml.safe_load((share/'config/arena_spec.yaml').read_text())
    cam = profile['uav_camera']
    pose = list(spec['uav_camera']['pose'])
    pose[5] += cam.get('mount_yaw_offset', 0.)
    profile['name'] = 'uav_actual_video'
    profile['validation_camera'] = dict(
        pose=pose, resolution=list(cam['resolution']),
        hfov=cam.get('optical_hfov', spec['uav_camera']['hfov']),
        update_rate=float(fps), clip=list(cam['clip']))
    path = output/'uav_recording_profile.yaml'
    path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding='utf-8')
    return path


def camera_description(world_path, calibration, arena):
    # Only camera metadata is read here; no object/model positions enter vision.
    model=ET.parse(world_path).getroot().find(".//model[@name='uav_camera']")
    if model is None or model.findtext('static')!='true':
        raise ValueError('Expected the fixed UAV camera model')
    pose=list(map(float,model.findtext('pose').split()))
    link=model.find('link'); sensor=link.find("sensor[@type='camera']")
    for mounted in (link,sensor):
        if any(abs(v)>1e-9 for v in map(float,mounted.findtext('pose','0 0 0 0 0 0').split())):
            raise ValueError('Camera has a nonzero relative mount; resolve its full transform first')
    optical=(*arena.world_to_nav(*pose[:2]),pose[2])
    if math.dist(optical,calibration.camera_center)>.001:
        raise ValueError('World camera optical centre differs from calibration')
    size=[int(sensor.findtext('camera/image/'+a)) for a in ('width','height')]
    if size!=list(calibration.image_size):
        raise ValueError('World camera resolution differs from calibration')
    return {'model':'uav_camera','fixed':True,'world_pose_xyz_rpy':pose,
            'arena_optical_center_m':list(optical),'image_size':size,
            'horizontal_fov_deg':math.degrees(float(sensor.findtext('camera/horizontal_fov'))),
            'update_rate_hz':float(sensor.findtext('update_rate')),
            'topic':sensor.findtext('topic'),'world_file':str(world_path),
            'calibration_validation':calibration.validation_report(),
            'scope':'Simulator camera settings; real competition camera placement is not verified'}


class UavViewRecording:
    def __init__(self, output, share, ground_provider=lambda: (None,None),
                 recording_profile=None):
        self.output=Path(output)
        share=Path(share)
        cal=load_calibration(share/'config/uav_grid_calibration.json')
        arena=ArenaGeometry.from_spec(yaml.safe_load((share/'config/arena_spec.yaml').read_text()))
        self.camera=camera_description(share/'worlds/maicon_arena.sdf',cal,arena)
        self.camera['agent_uav_update_rate_hz'] = self.camera['update_rate_hz']
        if recording_profile is not None:
            profile = yaml.safe_load(Path(recording_profile).read_text())
            recorded = profile['validation_camera']
            if (not np_allclose(recorded['pose'], self.camera['world_pose_xyz_rpy'])
                    or recorded['resolution'] != self.camera['image_size']
                    or not math.isclose(math.degrees(recorded['hfov']), self.camera['horizontal_fov_deg'])):
                raise ValueError('Recording camera must match the calibrated UAV viewpoint')
            self.camera.update(model='validation_camera', topic='/arena/validation/image_raw',
                               update_rate_hz=recorded['update_rate'],
                               recording_profile=str(recording_profile),
                               configuration_source='launch profile; co-located UAV recording camera')
        fps=self.camera['update_rate_hz']
        write_json(self.output/'uav_camera.json',self.camera)
        self.ledger=UavImageFaceLedger(cal,arena)
        self.ground_provider=ground_provider
        self.raw=SimulationVideoWriter(self.output/'uav_native_rgb.mp4',fps,.1)
        self.snapshot=None
        self.mission_finished=threading.Event()
        self.finalized=False
        self.history=(self.output/'uav_image_face_ledger.jsonl').open('x',encoding='utf-8')
        self.recorder=CameraRecorder(self.output/'uav_image_face_ledger.mp4',fps,
            'Actual Gazebo UAV view | B-2 ground observations',decorate=False,frame_transform=self.process)
        self.recorder.writer.max_hold_s=.1

    @property
    def error(self): return self.recorder.error

    @property
    def accepted(self): return self.recorder.accepted

    def receive(self,message): self.recorder.receive(message)

    def finish_mission(self):
        """Keep recording real frames, while retaining labelled final evidence."""
        self.mission_finished.set()

    def process(self,image,stamp):
        self.raw.add(image,stamp)
        # Analyze at 1 Hz while preserving every real high-rate video frame.
        final=self.mission_finished.is_set()
        if not self.finalized and (self.snapshot is None or final or stamp-self.snapshot['image_stamp_s'] >= 1.-1e-6):
            ground,ground_stamp=self.ground_provider()
            self.snapshot=self.ledger.update(image,stamp,ground,ground_stamp)
            if self.snapshot is None:
                raise ValueError('Repeated or out-of-order UAV image')
            self.snapshot['source']=self.camera['topic']+' RGB + measured calibration only'
            self.snapshot['mission_complete']=final
            self.finalized=final
            self.history.write(json.dumps(self.snapshot,ensure_ascii=False)+'\n');self.history.flush()
            write_json(self.output/'uav_image_face_ledger.json',self.snapshot)
        return draw_uav_image_ledger(image,self.snapshot,self.ledger.calibration,self.camera,video_stamp=stamp)

    def close(self):
        report=self.recorder.close()
        raw=self.raw.close()
        self.history.close()
        write_json(self.raw.metadata,raw)
        report.update(view='actual_uav_camera',raw_recording=raw,camera=self.camera,
                      ledger='uav_image_face_ledger.json',ledger_history='uav_image_face_ledger.jsonl',
                      final_ledger_frozen=self.finalized,
                      geometry_source='actual RGB frames only; no SLAM or Gazebo layout truth')
        write_json(self.recorder.writer.metadata,report)
        return report


def np_allclose(a,b):
    return len(a)==len(b) and all(math.isclose(x,y,abs_tol=1e-6) for x,y in zip(a,b))
