#!/usr/bin/env python3
"""One real Gazebo B2: existing Agent, ROS MIL HAL, image perception and video.

Requires an already running fresh arena (ugvs:=1, ego_slot:=B-2, WINGBOT),
SHM64 transport and a recording camera. Never creates scripted waypoints,
synthetic detections, model-pose localization, or additional simulator processes.
Gazebo model poses are written only to evaluation artifacts.
"""
from __future__ import annotations

import argparse
import dataclasses
import importlib
import json
import math
from pathlib import Path
import queue
import secrets
import signal
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
if __name__=='__main__':
    from selected_runtime import activate
    activate(ROOT)
ARTIFACT_IO_STATS={'replace_retries':0,'preview_skipped':0}


def load_factory(spec):
    module,separator,name=spec.partition(':')
    if not separator or not module or not name:
        raise ValueError('Factory must be module:function')
    value=getattr(importlib.import_module(module),name)
    if not callable(value):raise TypeError('Factory is not callable')
    return value


def fixture(slot="B-2"):
    """Explicit local rehearsal order, not a statement about the real match."""
    team = "BLUE" if slot.startswith("B") else "RED"
    enemy = "RED" if team == "BLUE" else "BLUE"
    prefix = "R" if team == "BLUE" else "B"
    return {"orders":{slot:f"작전 명령. 우선 타격 표적은 레이더시설. {prefix}-S1과 {prefix}-S2 구역을 정찰하라. 후속 작전 거점은 정비소로 설정하라."},
            "targets":{team:{"objective":enemy+"_MAINTENANCE","objective_label":"정비소"}},"test_mode":False}


def json_value(value):
    # asdict reconstructs mapping subclasses and fails on Face.counts'
    # defaultdict factory. Read fields directly, then use our JSON conversion.
    if dataclasses.is_dataclass(value):
        return {field.name:json_value(getattr(value,field.name)) for field in dataclasses.fields(value)}
    # HAL callbacks can add/remove diagnostic keys while a checkpoint is saved.
    if isinstance(value,dict):return {str(k):json_value(v) for k,v in list(value.items())}
    if isinstance(value,(list,tuple)):return [json_value(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    if hasattr(value,'tolist'):return json_value(value.tolist())
    if hasattr(value,'value'):return value.value
    return value


def replace_artifact(temporary,path):
    """Bound only transient Windows/OneDrive sharing locks, never all I/O."""
    for attempt in range(5):
        try:
            temporary.replace(path)
            return
        except PermissionError as exc:
            ARTIFACT_IO_STATS['last_error']=str(exc)
            if attempt==4:
                raise
            ARTIFACT_IO_STATS['replace_retries']+=1
            time.sleep(min(.02*2**attempt,.1))


def write_json(path,value):
    from maicon26.link.secrets import REDACT
    path=Path(path)
    text=REDACT(json.dumps(json_value(value),ensure_ascii=False,indent=2,allow_nan=False))+'\n'
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(text,encoding='utf-8')
    replace_artifact(temporary,path)


def write_preview_json(path,value):
    """A persistent preview lock skips one preview, never the robot tick."""
    try:
        write_json(path,value)
        return True
    except PermissionError as exc:
        ARTIFACT_IO_STATS['preview_skipped']+=1
        ARTIFACT_IO_STATS['last_skipped_path']=str(path)
        ARTIFACT_IO_STATS['last_error']=str(exc)
        return False


def write_preview_npz(path,**arrays):
    import numpy as np
    path=Path(path)
    temporary=path.with_suffix('.tmp.npz')
    try:
        np.savez_compressed(temporary,**arrays)
        replace_artifact(temporary,path)
        return True
    except PermissionError as exc:
        ARTIFACT_IO_STATS['preview_skipped']+=1
        ARTIFACT_IO_STATS['last_skipped_path']=str(path)
        ARTIFACT_IO_STATS['last_error']=str(exc)
        return False


def apply_trial_variant(cfg, variant):
    """Candidate-only switches; a v2 run preserves the production baseline."""
    cfg.progress.enabled = variant != 'v2'
    cfg.hal.imu_heading_fusion = variant != 'v2'
    cfg.engage.fire_on_blocker = variant == 'blocker_fire'
    if variant == 'dwell_zero':
        cfg.risk.dwell_cap_from_hits = 0


def latency_summary(samples):
    import numpy as np
    values=[float(value) for value in samples if math.isfinite(value)]
    return dict(count=len(values),p50_ms=float(np.percentile(values,50)) if values else None,
                p95_ms=float(np.percentile(values,95)) if values else None,
                max_ms=max(values) if values else None)


def processing_summary(agent, risk_worker):
    snapshot=(risk_worker.lineage_snapshot() if callable(getattr(risk_worker,'lineage_snapshot',None)) else None)
    records=[] if snapshot is None else snapshot['records']
    categories=sorted({r['category'] for r in records})
    return dict(risk_submit_to_publish=latency_summary(getattr(risk_worker,'latency_ms',[])),
                risk_build=latency_summary(getattr(risk_worker,'build_ms',[])),
                lidar_blob_processing=latency_summary(getattr(getattr(agent,'blobs',None),'latency_ms',[])),
                worker_errors=list(getattr(risk_worker,'errors',[])),
                scan_callback_to_publish=latency_summary([r['scan_callback_to_publish_ms'] for r in records if r['scan_callback_to_publish_ms'] is not None]),
                scan_callback_to_publish_by_category={c:latency_summary([r['scan_callback_to_publish_ms'] for r in records if r['category']==c and r['scan_callback_to_publish_ms'] is not None]) for c in categories},
                scan_lineage=None if snapshot is None else {k:v for k,v in snapshot.items() if k!='records'},
                scope='Worker submission and exact scan-callback-entry to field publication are distinct wall-time intervals. No DDS-arrival or Agent-reaction claim. Worker histories retain up to 10000 publications; LiDAR processing retains its last 2000 samples.')


def decision_with_route(agent):
    """Read-only provenance for the actual sensor-planned progress distance."""
    decision=agent.deadlock_telemetry()
    goal=decision.get('goal')
    if goal is not None and hasattr(agent,'_remaining_path_route'):
        source,path=agent._remaining_path_route(goal)
        endpoint=path[-1] if path else None
        decision.update(remaining_path_source=source,
                        remaining_path_endpoint=endpoint,
                        remaining_path_points=len(path) if path else 0,
                        remaining_path_endpoint_error_m=(math.hypot(endpoint[0]-goal[0],endpoint[1]-goal[1]) if endpoint else None))
    return decision


class CameraRecorder:
    """One waiting real frame; encoding cannot block the Agent or ROS callbacks."""
    def __init__(self,path,fps,label='B2 autonomous Agent | actual Gazebo third-person'):
        from maicon26.hal.fpv_recording import SimulationVideoWriter
        self.writer=SimulationVideoWriter(path,fps,.35,label)
        self.queue=queue.Queue(maxsize=1)
        self.stop=threading.Event()
        self.error=None
        self.dropped=0
        self.accepted=0
        self.max_changed_pixels=0
        self._previous=None
        self.worker=threading.Thread(target=self._run,name='third-person-encoder',daemon=True)
        self.worker.start()

    def receive(self,message):
        if self.stop.is_set():return
        try:self.queue.put_nowait(message)
        except queue.Full:
            try:self.queue.get_nowait()
            except queue.Empty:pass
            self.dropped+=1
            try:self.queue.put_nowait(message)
            except queue.Full:self.dropped+=1

    def _run(self):
        import cv2
        import numpy as np
        from maicon26.hal.fpv_recording import decode_bgr
        while not self.stop.is_set() or not self.queue.empty():
            try:message=self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:
                stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
                image=decode_bgr(message)
                gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
                if self._previous is not None:
                    changed=int(np.count_nonzero(cv2.absdiff(gray,self._previous)>12))
                    self.max_changed_pixels=max(self.max_changed_pixels,changed)
                self._previous=gray
                if self.writer.add(image,stamp):self.accepted+=1
            except Exception as error:
                self.error=type(error).__name__+': '+str(error)
                self.stop.set()
                break

    def close(self):
        self.stop.set();self.worker.join(timeout=10.)
        if self.worker.is_alive():raise RuntimeError('Video encoder did not finish')
        report=self.writer.close()
        report.update(callback_queue_dropped=self.dropped,max_changed_pixels=self.max_changed_pixels,
                      visible_change_detected=self.max_changed_pixels>=25,error=self.error)
        write_json(self.writer.metadata,report)
        return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--slot',default='B-2',choices=['B-1','B-2','B-3','R-1','R-2','R-3'])
    parser.add_argument('--multi-agent',action='store_true')
    parser.add_argument('--await-start',action='store_true')
    parser.add_argument('--no-record',action='store_true')
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--policy',default='agent',choices=['agent','idle','patrol_own','evader','hunter','blocker'])
    parser.add_argument('--variant',default='progress',choices=['v2','progress','blocker_fire','dwell_zero'])
    parser.add_argument('--output-dir' ,type=Path,required=True)
    parser.add_argument('--arena-spec',type=Path,default=ROOT/'ros2/maicon_arena_gz/config/arena_spec.yaml')
    parser.add_argument('--uav-calibration',type=Path,default=ROOT/'ros2/maicon_arena_gz/config/uav_grid_calibration.json')
    parser.add_argument('--no-uav',action='store_true',help='Disable UAV delivery; ground perception stays real')
    parser.add_argument('--operator-config',type=Path,help='Explicit local orders/targets JSON; omitted uses the documented rehearsal fixture')
    parser.add_argument('--config',type=Path,default=ROOT/'config/waveshare_real.json')
    parser.add_argument('--configure-factory',default='maicon26.hal.wingbot_calibration:configure_wingbot_simulation')
    parser.add_argument('--detector-factory',default='maicon26.sim.image_detector:make_wingbot_detector')
    parser.add_argument('--record-topic',default='/arena/validation/image_raw')
    parser.add_argument('--video-fps',type=float,default=10.)
    parser.add_argument('--ready-timeout',type=float,default=60.)
    parser.add_argument('--wall-timeout',type=float,default=650.)
    parser.add_argument('--report-seconds',type=float,default=35.)
    args=parser.parse_args(argv)
    if (not all(math.isfinite(v) for v in (args.video_fps,args.ready_timeout,args.wall_timeout,args.report_seconds))
            or not 1<=args.video_fps<=30 or args.ready_timeout<=0 or args.wall_timeout<=300 or args.report_seconds<1):
        parser.error('Invalid recording/readiness/wall/report bounds')
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error('Output directory must be new or empty; recordings are never overwritten')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    return run(args)


def run(args):
    import yaml
    from maicon26.arena import ArenaGeometry
    from maicon26.hal.wingbot import RosWingbot
    from maicon26.launch_profile import load_run_config
    from maicon26.link.async_link import AsyncServerLink
    from maicon26.link.http import HttpServerLink
    from maicon26.link.secrets import register_secret,REDACT
    from maicon26.mission.manager import Agent,Mode
    from maicon26.sim.sensor_policies import make_sensor_policy
    from maicon26.nav.factory import make_stack
    from maicon26.nav.ros_runtime import ensure_ros,add_node,shutdown
    from maicon26.perception.detector import AsyncDetector,SimDetector
    from maicon26.perception.simulation_uav import load_simulation_uav_detector, load_simulation_ugv_detector
    from maicon26.preflight import wait_ready
    from maicon26.sim.gazebo_api import GazeboApiAdapter,make_http_server,jpeg_from_ros_image,validate_operator_config
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image,LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    cfg,_=load_run_config(str(args.config),None,'builtin')
    # In Gazebo, one camera classification is enough to mark a pictured M3
    # face as recognized. This is a simulation policy, not the competition
    # server's all-counts-exact sector score.
    cfg.perception.min_votes=1
    arena_spec=yaml.safe_load(args.arena_spec.read_text(encoding='utf-8'))
    cfg.arena=ArenaGeometry.from_spec(arena_spec)
    cfg.team = 'BLUE' if args.slot.startswith('B') else 'RED'
    cfg.slot = int(args.slot[-1])
    apply_trial_variant(cfg, args.variant)
    cfg.nav.backend='builtin';cfg.nav2.use_sim_time=True
    cfg.depth.enable=False;cfg.map3d.enable=False
    cfg.hal.restart_state_path='';cfg.hal.freeze_clock_on_pause=False
    cfg.mission.t_total=300.
    calibration=load_factory(args.configure_factory)(cfg)
    if args.multi_agent:
        api='/robots/'+args.slot.replace('-','')
        raw='/arena/robots/'+args.slot.replace('-','')
        cfg.hal.ros_namespace=api
        for field in ('cmd_topic','odom_topic','image_topic','imu_topic','state_topic','match_topic','oled_topic','gimbal_srv','fire_srv','scan_topic'):
            setattr(cfg.hal,field,api+getattr(cfg.hal,field))
        cfg.camera.calibration_profile['info_topic']=raw+'/camera_info'
        cfg.camera.calibration_profile['joint_topic']=raw+'/joint_states' 
    # Match the actual model's known dimensions, never any Gazebo world pose.
    # The visual bbox is slightly asymmetric about base_footprint; use a
    # conservative symmetric navigation footprint around that true frame.
    model_root=ROOT/'ros2/wingbot_description'
    model_frames=yaml.safe_load((model_root/'config/frames.yaml').read_text(encoding='utf-8'))
    model_mods=yaml.safe_load((model_root/'config/wingbot_mods.yaml').read_text(encoding='utf-8'))
    lo,hi=model_frames['visual_bounds']['min'],model_frames['visual_bounds']['max']
    cfg.robot.length=2*max(abs(float(lo[0])),abs(float(hi[0])))
    cfg.robot.width=2*max(abs(float(lo[1])),abs(float(hi[1])))
    cfg.robot.height=float(hi[2])
    cfg.robot.radius=math.hypot(cfg.robot.length,cfg.robot.width)/2
    cfg.robot.track_width=2*float(model_mods['wheels']['y'])
    cfg.robot.wheel_radius=float(model_mods['wheels']['radius'])
    inner_detector=load_factory(args.detector_factory)(cfg)
    if isinstance(inner_detector,SimDetector) or not callable(getattr(inner_detector,'detect',None)):
        raise TypeError('The autonomous trial requires an actual-image detector, never SimDetector')
    operator=validate_operator_config(json.loads(args.operator_config.read_text(encoding='utf-8-sig')) if args.operator_config else fixture(args.slot))
    if args.slot not in operator['orders'] or cfg.team not in operator['targets']:
        raise ValueError('Local fixture needs a B-2 order and BLUE M5 objective')
    credential=secrets.token_hex(4).upper();register_secret(credential)
    adapter=GazeboApiAdapter({credential:args.slot},operator_config=operator,max_input_age_s=2.)
    quit_event=threading.Event()
    previous={sig:signal.signal(sig,lambda *_:quit_event.set()) for sig in (signal.SIGINT,signal.SIGTERM)}
    class NullRecorder:
        accepted=2
        error=None
        def receive(self,message): pass
        def close(self): return {'enabled':False,'recorded_by':'multi-agent coordinator'}
    recorder = NullRecorder() if args.no_record else CameraRecorder(args.output_dir/'b2_autonomous_third_person.mp4',args.video_fps)
    truth_file=(args.output_dir/'evaluation_truth.jsonl').open('x',encoding='utf-8')
    trace_file=(args.output_dir/'agent_trace.jsonl').open('x',encoding='utf-8')
    started_wall=time.monotonic()
    rclpy,executor=ensure_ros()

    class Observer(Node):
        def __init__(self):
            super().__init__('wingbot_trial_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
            self.guard=threading.RLock()
            self.stamp=None;self.clock_wall=time.monotonic();self.state={};self.events=[]
            self.truth_first=None;self.truth_last=None;self.truth_distance=0.;self.truth_samples=0;self.truth_jumps=0
            self.ugv_names=set();self.errors=[]
            self.initial_scans={}
            self.controls=self.create_publisher(String,'/arena/control',10)
            self.create_subscription(Clock,'/clock',self.on_clock,qos_profile_sensor_data)
            self.create_subscription(String,'/arena/state',self.on_state,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(String,'/arena/events',self.on_event,100)
            if not args.no_record:self.create_subscription(Image,args.record_topic,recorder.receive,qos_profile_sensor_data)
            if not args.no_uav:self.create_subscription(Image,'/uav/image_raw',self.on_uav,qos_profile_sensor_data)
            self.create_subscription(TFMessage,'/arena/dynamic_pose',self.on_truth,qos_profile_sensor_data)
            for key,topic in [('raw',('/arena/robots/'+args.slot.replace('-','') if args.multi_agent else '/arena/ego')+'/scan_raw'),('filtered',cfg.hal.scan_topic)]:
                self.create_subscription(LaserScan,topic,lambda m,key=key:self.on_scan_audit(key,m),qos_profile_sensor_data)
            self.create_timer(.05,self.flush_controls)

        def sim_now(self):
            with self.guard:return float(self.stamp or 0.)

        def on_scan_audit(self,key,message):
            # Read-only READY evidence; never used as an alternative Agent scan.
            if self.state_copy().get('phase')!='ready':return
            values=list(message.ranges)
            self.initial_scans[key]={'sensor_stamp':message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                'frame':message.header.frame_id,'samples':len(values),
                'near_under_15cm':sum(math.isfinite(v) and 0<v<.15 for v in values),
                'nan_count':sum(math.isnan(v) for v in values),
                'min_finite_m':min((v for v in values if math.isfinite(v)),default=None)}

        def state_copy(self):
            with self.guard:return dict(self.state)

        def on_clock(self,message):
            stamp=message.clock.sec+message.clock.nanosec*1e-9
            with self.guard:
                if self.stamp is not None and stamp<self.stamp:
                    self.errors.append('Simulation clock moved backwards');quit_event.set();return
                if self.stamp is None or stamp>self.stamp:self.clock_wall=time.monotonic()
                self.stamp=stamp
            adapter.update_time(stamp)

        def on_state(self,message):
            try:
                state=json.loads(message.data)
                with self.guard:self.state=state
                adapter.update_state(state)
            except (TypeError,ValueError):self.errors.append('Invalid referee state')

        def on_event(self,message):
            try:event=json.loads(message.data)
            except ValueError:return
            with self.guard:self.events.append(event)

        def on_uav(self,message):
            try:adapter.capture_jpeg(jpeg_from_ros_image(message),message.header.stamp.sec+message.header.stamp.nanosec*1e-9)
            except (TypeError,ValueError,OSError):adapter.stats['image_decode_rejected']+=1

        def on_truth(self,message):
            # Deliberately isolated: this data never enters cfg/Robot/Agent/detector.
            for t in message.transforms:
                if '::' in t.child_frame_id:continue  # Link-local poses are not world model poses.
                name=t.child_frame_id.split('::')[0]
                if not name.startswith('ugv_'):continue
                self.ugv_names.add(name)
                if name!='ugv_'+args.slot.replace('-',''):continue
                p,q=t.transform.translation,t.transform.rotation
                pose=(float(p.x),float(p.y),math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                with self.guard:
                    if self.truth_first is None:self.truth_first=pose
                    if self.truth_last is not None:
                        distance=math.dist(pose[:2],self.truth_last[:2])
                        if distance<.5:self.truth_distance+=distance
                        else:self.truth_jumps+=1
                    self.truth_last=pose;self.truth_samples+=1
                truth_file.write(json.dumps({'sim_s':self.sim_now(),'world_xy_yaw':pose})+'\n')

        def flush_controls(self):
            for control in adapter.drain_controls():self.control(control)

        def control(self,value):self.controls.publish(String(data=json.dumps(value)))

    observer=Observer();add_node(observer)
    robot=stack=link=http=detector=agent=server=worker=None
    risk_worker_for_report=None
    aim_recorder=aim_evidence=None
    original_inner_detect=None
    error=None;start_sim=None;result={};last_trace=-math.inf;last_print=-math.inf;last_overlay=-math.inf;report_wall=None;coverage210=False
    try:
        def wait(predicate,seconds,description):
            until=time.monotonic()+seconds
            while not predicate():
                if quit_event.is_set():raise InterruptedError('Trial interrupted')
                if recorder.error:raise RuntimeError(recorder.error)
                if time.monotonic()>until:raise TimeoutError(description)
                time.sleep(.05)
        wait(lambda:observer.stamp is not None and observer.state_copy().get('phase')=='ready' and recorder.accepted>=2 and observer.truth_samples>=2,args.ready_timeout,'Fresh ready arena, truth evaluation and actual recording frames required')
        expected={'ugv_'+s for s in ('B1','B2','B3','R1','R2','R3')} if args.multi_agent else {'ugv_'+args.slot.replace('-','')}
        if observer.ugv_names!=expected:
            raise ValueError('Trial requires exactly one physical UGV: ugv_B2')
        robot=RosWingbot(cfg,clock=observer.sim_now)
        robot.tr.node.set_parameters([Parameter('use_sim_time',value=True)])
        stack=make_stack(cfg,robot)
        wait_ready(robot,stack,args.ready_timeout,should_quit=quit_event.is_set)
        frame=robot.get_frame()
        if frame is None or frame.image is None:raise RuntimeError('Actual rectified ground camera is required')
        import cv2
        cv2.imwrite(str(args.output_dir/'ready_ground_rgb.png'),frame.image)
        inner_detector.detect(frame)  # Real-image warm-up before READY/start.
        write_json(args.output_dir/'sensor_readiness.json',{'scans':observer.initial_scans,
            'camera_and_gimbal':robot.stats,'frame_metadata':frame.meta,'frame_pan_tilt':[frame.pan,frame.tilt]})
        detector=AsyncDetector(inner_detector)
        server=make_http_server(adapter,'127.0.0.1',0)
        worker=threading.Thread(target=server.serve_forever,name='local-rehearsal-api',daemon=True);worker.start()
        uav_callback=None if args.no_uav else load_simulation_uav_detector(args.uav_calibration,cfg.arena).detect
        ugv_callback=None if args.no_uav else load_simulation_ugv_detector(args.uav_calibration,cfg.arena).detect
        http=HttpServerLink(f'http://127.0.0.1:{server.server_address[1]}',credential,cfg.team,cfg.slot,
                            clock=robot.now,uav_layout_detector=uav_callback,uav_robot_detector=ugv_callback,uav_layout_max_age_s=cfg.explore.uav_layout_timeout_s)
        handshake_until=time.monotonic()+args.ready_timeout
        if not http.ready_handshake(lambda:robot.readiness()[0],should_quit=lambda:quit_event.is_set() or time.monotonic()>handshake_until,poll_s=.25):
            raise RuntimeError('Local API readiness handshake did not complete')
        if http.identity()!=(cfg.team,cfg.slot):raise RuntimeError('Local API identity mismatch')
        link=AsyncServerLink(http)
        agent=make_sensor_policy(args.policy,robot,link,cfg=cfg,detector=detector,stack=stack,llm=None,name=args.slot,seed=args.seed)
        from aim_diagnostics import install_aim_diagnostics
        from trial_evidence import BoundedAimEvidence
        aim_recorder=install_aim_diagnostics(agent)
        aim_evidence=BoundedAimEvidence(args.output_dir/'aim_evidence',replace_artifact,write_preview_json)
        original_inner_detect=inner_detector.detect
        def detect_with_evidence(frame):
            aim_recorder.observe_frame(frame)
            return original_inner_detect(frame)
        inner_detector.detect=detect_with_evidence
        # Populate the observed pose/map while motors remain locked in READY.
        agent.tick()
        write_json(args.output_dir/'trial_config.json',{'scope':('one of six isolated physical Agents' if args.multi_agent else 'one physical Agent'),'operator_fixture':operator,
            'nav':'builtin scan/odom SLAM + A*','detector_factory':args.detector_factory,'camera_calibration':calibration,
            'uav':'disabled' if args.no_uav else 'actual JPEG via loopback API; measured image-only cube candidates',
            'truth_access':'observer/evaluation only; never Agent or perception','llm':'not connected; existing rule-based M1/M6 fallback',
            'policy':args.policy,'record_topic':args.record_topic,'config':dataclasses.asdict(cfg)})
        blue_objective=operator['targets'][cfg.team]['objective'].removeprefix('RED_').removeprefix('BLUE_')
        rows=[row for row,label in arena_spec['bases']['label_map'].items() if label==blue_objective]
        if len(rows)!=1:raise ValueError('Operator objective does not resolve in the measured base label map')
        write_json(args.output_dir/'ready.json',{'slot':args.slot,'sensor_ready':robot.readiness(),'sim_s':observer.sim_now()})
        if not args.await_start:
            observer.control({'cmd':'start','objectives':{'BLUE':'R-'+rows[0].removeprefix('row'),'RED':'B-2'}})
        wait(lambda:observer.state_copy().get('phase')=='running',args.ready_timeout if args.await_start else 5.,'Referee did not start')
        state=observer.state_copy();start_sim=observer.sim_now()-float(state.get('elapsed') or 0.)
        print('B2 autonomous Agent started; real sensor input, local fixture, third-person recording',flush=True)
        previous_sim=observer.sim_now()
        while not quit_event.is_set():
            wall=time.monotonic();now=observer.sim_now()
            if wall-started_wall>args.wall_timeout:raise TimeoutError('Bounded trial wall timeout')
            if wall-observer.clock_wall>5.:raise RuntimeError('Gazebo clock stalled')
            if recorder.error:raise RuntimeError(recorder.error)
            if now<=previous_sim:
                time.sleep(.005);continue
            previous_sim=now
            # REPORT/DONE closes and clears the Agent's worker. Retain only a
            # diagnostic reference so its completed samples survive shutdown.
            risk_worker_for_report=getattr(agent,'_risk_worker',None) or risk_worker_for_report
            agent.tick()
            risk_worker_for_report=getattr(agent,'_risk_worker',None) or risk_worker_for_report
            aim_snapshot=aim_recorder.snapshot()
            if aim_evidence.wants(aim_snapshot):
                aim_evidence.offer(aim_snapshot,aim_recorder.exact_source_image())
            state=observer.state_copy()
            if not coverage210 and float(state.get('elapsed') or 0.)>=210.:
                write_json(args.output_dir/'coverage_210.json', {'elapsed':state.get('elapsed'),
                    'observed_classified_faces':agent.omap.confirmed_faces(),
                    'coverage_faces':[f for b in agent.coverage.boxes.values() for f in b.faces]})
                coverage210=True
            if now-last_trace>=.045:
                pose=stack.pose
                trace={'sim_s':now,'elapsed':state.get('elapsed'),'phase':state.get('phase'),'mode':agent.state_name,
                       'estimated_nav_xy_yaw':[pose.x,pose.y,pose.th],'cmd_vw':[agent._v_cmd,agent._w_cmd],
                       'decision':decision_with_route(agent),'readiness':robot.readiness(),'target_done':agent.target_done,'m3_counts':agent._m3_counts(),
                       'det_consumed':agent.stats['det_consumed'],'detector_errors':agent.stats['detector_errors'],
                       'aim':aim_snapshot}
                trace_file.write(REDACT(json.dumps(json_value(trace),ensure_ascii=False))+'\n');trace_file.flush()
                last_trace=now
            if now-last_overlay>=1. and args.multi_agent:
                write_preview_json(args.output_dir/'overlay.json', {'decision':agent.deadlock_telemetry(),'pose':[stack.pose.x,stack.pose.y,stack.pose.th],'mode':agent.state_name,'sim_s':now})
                if agent._field is not None:
                    grid=stack.grid
                    write_preview_npz(args.output_dir/'risk_overlay.npz',risk=agent._field.risk[0],ox=grid.ox,oy=grid.oy,res=grid.res)
                last_overlay=now
            if now-last_print>=10.:
                print(f"simulation elapsed={float(state.get('elapsed') or 0.):.1f}s mode={agent.state_name} real_video_frames={recorder.accepted}",flush=True)
                write_preview_json(args.output_dir/'checkpoint.json',{'sim_s':now,'state':state,'agent_stats':agent.stats,'last_trace':trace,'hal_stats':robot.stats,'artifact_io_stats':dict(ARTIFACT_IO_STATS)})
                last_print=now
            if agent.mode==Mode.DONE:
                report_wall=report_wall or wall
                if wall-report_wall>=1.:break
            if now-start_sim>cfg.mission.t_total+args.report_seconds:
                raise TimeoutError('Mission reporting deadline exceeded')
            time.sleep(max(0.,1./cfg.loop_hz-(time.monotonic()-wall)))
    except Exception as exc:
        error=REDACT(type(exc).__name__+': '+str(exc));print(error,file=sys.stderr,flush=True)
    finally:
        if robot is not None:robot.set_velocity(0.,0.)
        if observer.state_copy().get('phase')=='running':observer.control({'cmd':'report'})
        time.sleep(.2)
        if agent is not None:
            risk_worker=getattr(agent,'_risk_worker',None) or risk_worker_for_report
            agent.close_deadlock()
            write_json(args.output_dir/'risk_worker_summary.json',processing_summary(agent,risk_worker))
            if callable(getattr(risk_worker,'lineage_snapshot',None)):
                write_json(args.output_dir/'scan_lineage.json',risk_worker.lineage_snapshot())
            result.update(agent_mode=agent.mode.value,stop_source=agent.stop_source,m6_result=agent.m6_result,
                          agent_stats=agent.stats,agent_log=agent.log,m3_counts=agent._m3_counts(),
                          target_done=agent.target_done,target_failed=agent.target_failed,
                          coverage=agent.coverage.summary(),observed_classified_faces=agent.omap.confirmed_faces(),estimated_pose=dataclasses.asdict(stack.pose),
                          lidar_blob_stats=dict(getattr(getattr(agent,'blobs',None),'stats',{})),
                          m3_arrival_events=list(getattr(agent,'_m3_arrival_events',())),
                          detector_stats=getattr(inner_detector,'stats',{}),hal_stats=robot.stats)
        with adapter._condition:
            result.update(api_stats=dict(adapter.stats),http_attempts=dict(adapter.mock.attempts),
                          submitted_reports=adapter.mock.team(credential).reports)
        result.update(error=error,interrupted=quit_event.is_set(),match_start_sim_s=start_sim,
                      end_sim_s=observer.sim_now(),referee_state=observer.state_copy(),referee_events=observer.events,
                      evaluation={'truth_only_for_evaluation':True,'physical_ugvs':sorted(observer.ugv_names),
                                  'truth_samples':observer.truth_samples,'distance_m':observer.truth_distance,
                                  'first_world_pose':observer.truth_first,'last_world_pose':observer.truth_last,
                                  'pose_jumps_rejected':observer.truth_jumps,'robot_moved':observer.truth_distance>.1},
                      sensor_source='actual ROS camera/scan/odom/imu; no synthetic detections',
                      all_missions_success_claimed=False,
                      limitation=('Six physical UGVs; no external LLM. Submission ACKs do not prove M3 correctness.' if args.multi_agent else
                                  'Single UGV: no physical opponent engagement; no external LLM. Submission ACKs do not prove M3 correctness.'))
        # Stop queued ROS callbacks before destroying subscriptions/nodes. A
        # running executor can otherwise take an already destroyed handle.
        executor.shutdown()
        # Humble inherits Executor.shutdown and does not drain its separate
        # concurrent.futures pool. Drain queued takes before freeing handles.
        callback_pool=getattr(executor,'_executor',None)
        if callback_pool is not None:
            callback_pool.shutdown(wait=True)
        fusion=getattr(getattr(robot,'tr',None),'imu_odometry',None)
        if fusion is not None:
            try:
                result['imu_odometry']=fusion.diagnostics()
                result['imu_odometry']['saved_rows']=fusion.dump(args.output_dir/'imu_odometry.jsonl')
                write_json(args.output_dir/'imu_odometry_summary.json',result['imu_odometry'])
            except Exception as exc:
                result.setdefault('cleanup_errors',[]).append('IMU diagnostics: '+type(exc).__name__)
        for item in (detector,stack,robot):
            if item is not None:
                try:item.close()
                except Exception as exc:result.setdefault('cleanup_errors',[]).append(type(exc).__name__)
        if aim_recorder is not None:
            aim_recorder.close()
        if original_inner_detect is not None:
            inner_detector.detect=original_inner_detect
        if aim_evidence is not None:
            result['aim_evidence']=aim_evidence.close()
        adapter.close()
        if link is not None:link.close()
        elif http is not None:http.close()
        if server is not None:server.shutdown();server.server_close()
        if worker is not None:worker.join(timeout=3.)
        observer.destroy_node();shutdown()
        truth_file.close();trace_file.close()
        result['recording']=recorder.close()
        result['artifact_io_stats']=dict(ARTIFACT_IO_STATS)
        write_json(args.output_dir/'mission_report.json',result)
        for sig,handler in previous.items():signal.signal(sig,handler)
    return 0 if error is None and not quit_event.is_set() and agent is not None and agent.mode==Mode.DONE else 1


if __name__=='__main__':raise SystemExit(main())
