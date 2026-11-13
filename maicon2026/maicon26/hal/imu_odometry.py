"""Opt-in relative IMU heading + wheel translation, with continuous fallback.

Only changes in measured IMU yaw are used. No global yaw, map pose or simulator
truth is an input. A validated level sensor mounting is the transport's duty.
"""
from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import threading

from ..types import Pose2D


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


class RelativeImuOdometry:
    def __init__(self,max_age_s=.15,history_size=16000):
        self.max_age_s=float(max_age_s)
        if not math.isfinite(self.max_age_s) or not 0<self.max_age_s<=.15:
            raise ValueError("IMU association age must be in (0, 0.15] seconds")
        self._lock=threading.RLock()
        self._imu=deque(maxlen=64)
        self._imu_stamp=None
        self._imu_stream_valid=False
        self._wheel_stamp=None
        self._raw=None
        self._fused=None
        self._previous_imu_yaw=None
        self.samples=deque(maxlen=history_size)
        self.stats=dict(wheel_samples=0,imu_samples=0,fused_steps=0,wheel_fallback_steps=0,
            imu_rejected=0,wheel_rejected=0,reanchors=0,mode="uninitialized")

    def add_imu(self,stamp,yaw,*,valid=True,reason=""):
        with self._lock:
            if not math.isfinite(stamp) or stamp<=0 or (self._imu_stamp is not None and stamp<=self._imu_stamp):
                self.stats["imu_rejected"]+=1
                self._imu_stream_valid=False
                return False
            self._imu_stamp=float(stamp)
            valid=bool(valid and math.isfinite(yaw))
            self._imu_stream_valid=valid
            self._imu.append((float(stamp),float(yaw) if valid else None))
            self.stats["imu_samples"]+=1
            if not valid:
                self.stats["imu_rejected"]+=1
            self.samples.append(dict(kind="imu",stamp=float(stamp),yaw_rad=float(yaw) if valid else None,
                                     valid=valid,reason=reason))
            return valid

    def _at(self,stamp):
        if stamp is None or not self._imu or not self._imu_stream_valid:
            return None,None
        # A newest invalid orientation invalidates the stream until a new valid
        # measurement arrives; cached valid quaternions are not a fallback IMU.
        if self._imu[-1][1] is None:
            return None,None
        before=next((row for row in reversed(self._imu) if row[0]<=stamp),None)
        after=next((row for row in self._imu if row[0]>=stamp),None)
        if before is not None and after is not None and before[1] is not None and after[1] is not None:
            if max(stamp-before[0],after[0]-stamp)<=self.max_age_s:
                if after[0]==before[0]:
                    return before[1],before[0]
                fraction=(stamp-before[0])/(after[0]-before[0])
                return wrap(before[1]+fraction*wrap(after[1]-before[1])),stamp
        closest=min(self._imu,key=lambda row:abs(row[0]-stamp))
        if closest[1] is None or abs(closest[0]-stamp)>self.max_age_s:
            return None,None
        return closest[1],closest[0]

    def update_wheel(self,stamp,pose):
        with self._lock:
            if not all(math.isfinite(v) for v in (pose.x,pose.y,pose.th)):
                self.stats["wheel_rejected"]+=1
                return self._fused.copy() if self._fused is not None else None
            stamp=float(stamp) if stamp is not None and math.isfinite(stamp) and stamp>0 else None
            if stamp is not None and self._wheel_stamp is not None and stamp<=self._wheel_stamp:
                self.stats["wheel_rejected"]+=1
                return self._fused.copy()
            imu_yaw,imu_stamp=self._at(stamp)
            if self._raw is None:
                self._raw=pose.copy();self._fused=pose.copy()
                self._previous_imu_yaw=imu_yaw
                mode="initial_imu" if imu_yaw is not None else "initial_wheel"
            else:
                raw_delta=wrap(pose.th-self._raw.th)
                if imu_yaw is not None and self._previous_imu_yaw is not None:
                    delta=wrap(imu_yaw-self._previous_imu_yaw)
                    self.stats["fused_steps"]+=1;mode="imu_relative"
                else:
                    delta=raw_delta
                    self.stats["wheel_fallback_steps"]+=1
                    mode="wheel_fallback"
                    if imu_yaw is not None:
                        # Reacquisition aligns at this wheel sample; an old
                        # absolute IMU angle cannot cause an instantaneous jump.
                        self.stats["reanchors"]+=1;mode="wheel_reanchor"
                old_fused=self._fused.th
                new_heading=wrap(old_fused+delta)
                raw_mid=self._raw.th+raw_delta*.5
                fused_mid=old_fused+delta*.5
                rotation=fused_mid-raw_mid
                dx,dy=pose.x-self._raw.x,pose.y-self._raw.y
                self._fused=Pose2D(self._fused.x+math.cos(rotation)*dx-math.sin(rotation)*dy,
                                  self._fused.y+math.sin(rotation)*dx+math.cos(rotation)*dy,new_heading)
                self._raw=pose.copy()
                self._previous_imu_yaw=imu_yaw
            if stamp is not None:
                self._wheel_stamp=stamp
            self.stats["wheel_samples"]+=1;self.stats["mode"]=mode
            self.stats["imu_age_s"]=None if imu_stamp is None or stamp is None else abs(stamp-imu_stamp)
            self.stats["raw_pose"]=[pose.x,pose.y,pose.th]
            self.stats["fused_pose"]=[self._fused.x,self._fused.y,self._fused.th]
            self.samples.append(dict(kind="odom",stamp=stamp,raw=list(self.stats["raw_pose"]),
                fused=list(self.stats["fused_pose"]),imu_stamp=imu_stamp,imu_yaw=imu_yaw,mode=mode))
            return self._fused.copy()

    def diagnostics(self):
        with self._lock:
            return dict(self.stats)

    def dump(self,path):
        with self._lock:
            rows=list(self.samples)
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        with path.open("w",encoding="utf-8") as stream:
            for row in rows:
                stream.write(json.dumps(row,allow_nan=False)+"\n")
        return len(rows)
