"""Read-only evidence from actual aim/guard calls; never calls a guard twice.

This helper does not infer success from a geometric image centre. A guard not
called because an earlier condition failed is explicitly absent. Camera pixels
are saved only when the caller supplies the exact capture-stamped frame.
"""
from collections import deque
import copy
import functools
import threading


class AimDiagnostics:
    GUARDS=('_may_fire','_fire_conf_ok','_shot_geometry_ok','_wrong_target_in_line',
            '_friend_in_line','_strike_incidence_ok','vision_fresh')

    def __init__(self,agent):
        self.agent=agent
        self.latest=None
        self.guards={}
        self._key=None
        self._reused=0
        self._restores=[]
        self._frames=deque(maxlen=8)
        self._frame_lock=threading.Lock()
        self._install()

    @staticmethod
    def _pose(pose):
        return [float(pose.x),float(pose.y),float(pose.th)]

    def _install(self):
        a=self.agent
        original=a.aim.update
        @functools.wraps(original)
        def update(det,pan,tilt,now,range_m,**kwargs):
            result=original(det,pan,tilt,now,range_m,**kwargs)
            pan_command,tilt_command,aim=result
            context=getattr(a,'_det_ctx',None)
            consumed=getattr(a,'_det_consumed',None)
            key=(consumed,det.cls,tuple(det.box))
            self._reused=self._reused+1 if key==self._key else 0
            self._key=key
            self.latest=dict(tick=getattr(a,'_tick_no',None),now=float(now),
                mode=getattr(getattr(a,'mode',None),'value',None),
                detection_stamp=getattr(det,'stamp',None),consumed_frame_stamp=consumed,
                capture_robot_time=None if context is None else context[0],
                capture_pose=None if context is None else self._pose(context[1]),
                capture_pan_tilt=None if context is None else list(context[2:4]),
                current_pose=self._pose(a.stack.pose),current_pan_tilt=[pan,tilt],
                pan_tilt_command=[pan_command,tilt_command],range_m=range_m,
                target_class=det.cls,target_confidence=det.conf,target_box=list(det.box),
                receiver_candidates=copy.deepcopy((det.extra or {}).get('sensors',[])),
                selected_filtered_pixel=getattr(a.aim,'_last_pt',None),
                same_detection_reuse_count=self._reused,
                err_pan=aim.err_pan,err_tilt=aim.err_tilt,body_w=aim.body_w,
                aligned=aim.aligned,sensor_locked=aim.sensor_locked,sensor_ok=aim.sensor_ok,
                held_for_s=a.aim.held_for(now),
                source_frame_available=self._has_source(consumed))
            return result
        a.aim.update=update
        self._restores.append((a.aim,'update',original))
        for name in self.GUARDS:
            if not callable(getattr(a,name,None)):continue
            old=getattr(a,name)
            @functools.wraps(old)
            def observe(*args,_name=name,_old=old,**kwargs):
                result=_old(*args,**kwargs)
                self.guards[_name]=dict(tick=getattr(a,'_tick_no',None),value=bool(result))
                return result
            setattr(a,name,observe)
            self._restores.append((a,name,old))

    def observe_frame(self,frame):
        """Optional detector-side hook; retain the exact source, never latest RGB.

        The runner may call this once for the frame actually submitted to its
        detector. Pixels are copied so downstream buffers cannot overwrite it.
        This does not call the robot, camera, detector, or control function.
        """
        if frame is None or frame.image is None:return
        stamp=frame.stamp
        with self._frame_lock:
            if self._frames and self._frames[-1][0]==stamp:return
        pixels=frame.image.copy()
        with self._frame_lock:self._frames.append((stamp,pixels))

    def _has_source(self,stamp):
        with self._frame_lock:return any(observed==stamp for observed,_ in self._frames)

    def exact_source_image(self):
        """Return a copy only on an exact detection/capture identity match."""
        if self.latest is None:return None
        stamp=self.latest['consumed_frame_stamp']
        with self._frame_lock:
            image=next((image for observed,image in reversed(self._frames) if observed==stamp),None)
        return None if image is None else image.copy()

    def snapshot(self):
        if self.latest is None:return None
        out=copy.deepcopy(self.latest)
        tick=getattr(self.agent,'_tick_no',None)
        out['updated_this_tick']=out['tick']==tick
        out['guards_called_this_tick']={name:copy.deepcopy(value) for name,value in self.guards.items()
                                        if value['tick']==tick}
        out['uncalled_guards']='unknown; short-circuit or outside the firing branch'
        return out

    def close(self):
        for obj,name,original in reversed(self._restores):setattr(obj,name,original)
        self._restores.clear()
        with self._frame_lock:self._frames.clear()


def install_aim_diagnostics(agent):
    return AimDiagnostics(agent)
