"""Pixel-only WINGBOT fixture recognition, explicitly simulation opt-in.

The two coloured antenna sleeves and a visible white receiver are required.
This is a geometric fixture recognizer, not a trained detector for real robots.
No simulator IDs, spawn positions, or Frame metadata are accepted.
"""
from __future__ import annotations
import math
import cv2
import numpy as np

from ..types import Detection
from ..link.base import UavDet
from .receiver import find_receivers


def colour_masks(bgr):
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    vivid=(hsv[:,:,1]>=115)&(hsv[:,:,2]>=45)
    return {"RED":vivid&((hsv[:,:,0]<=9)|(hsv[:,:,0]>=172)),
            "BLUE":vivid&(hsv[:,:,0]>=100)&(hsv[:,:,0]<=128)}


def sleeve_components(bgr, *, overhead=False):
    components=[]
    for team,raw in colour_masks(bgr).items():
        mask=raw.astype(np.uint8)*255
        if not overhead:
            # Join the two black sleeve bands; do not join distant antennas.
            k=max(3,int(bgr.shape[0]/70)|1)
            mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((k,1),np.uint8))
        n,labels,stats,centres=cv2.connectedComponentsWithStats(mask)
        for i in range(1,n):
            x,y,w,h,area=map(int,stats[i])
            if overhead:
                if not 2<=area<=100 or max(w,h)>25:
                    continue
            elif (area<5 or h<4 or not 1.5<=h/max(w,1)<=9.
                  or area/(w*h)<.40):
                continue
            components.append(dict(team=team,x=x,y=y,w=w,h=h,area=area,
                u=float(centres[i][0]),v=float(centres[i][1])))
    return components


class WingbotGroundDetector:
    def detect(self,frame):
        bgr=frame.image
        if not isinstance(bgr,np.ndarray) or bgr.ndim!=3:
            return []
        comps=sleeve_components(bgr); H,W=bgr.shape[:2]
        out=[]; used=set()
        for i,a in enumerate(comps):
            for j in range(i+1,len(comps)):
                b=comps[j]
                if i in used or j in used or a["team"]!=b["team"]:
                    continue
                h=(a["h"]+b["h"])/2
                dx=abs(a["u"]-b["u"])
                if (max(a["h"],b["h"])/min(a["h"],b["h"])>1.8
                    or abs(a["v"]-b["v"])>.75*h or not .9*h<dx<6.*h):
                    continue
                cx=(a["u"]+b["u"])/2; top=min(a["y"],b["y"])
                width=max(dx+1.6*h,4.*h)
                box=(max(0.,cx-width/2),max(0.,top-.85*h),
                     min(float(W),cx+width/2),min(float(H),top+5.2*h))
                sensor_roi=(box[0],box[1],box[2],min(box[3],top+3.3*h))
                sensors=find_receivers(bgr,sensor_roi,max_n=2,kind="ugv")
                # A white sleeve highlight or background sky is not a dome:
                # the receiver lies below the two physical antenna sleeves.
                sensors=[s for s in sensors if s[1]>top+h*.9]
                if not sensors:
                    continue
                out.append(Detection("UGV",.88,box,frame.stamp,team=a["team"],
                    extra={"method":"rgb_wingbot_sleeve_pair_and_dome",
                           "sensors":sensors,"team_confidence":.88}))
                used.update((i,j))
        return out


class WingbotUavDetector:
    """Two sleeve observations projected on their measured height, then paired.

    Pair separation is known model geometry, not an arena layout. The pair
    midpoint is 4cm behind the body origin; without reliable heading it is kept
    as a bounded observed position error, never corrected from world pose.
    """
    def __init__(self,calibration,sleeve_height_m=.217,separation_m=.214):
        self.calibration=calibration
        self.sleeve_height_m=float(sleeve_height_m)
        self.separation_m=float(separation_m)
        self.last_observations=[]

    def _domes_visible(self,bgr,a,b,midpoint,hsv):
        """Both model-sized dome regions must contain white RGB evidence.

        The antenna baseline fixes lateral direction. Its two possible forward
        directions are tested against actual dome pixels, never world heading.
        """
        from .uav_calibration import _project
        along=(np.asarray(b["xy"])-np.asarray(a["xy"]))
        along/=max(float(np.linalg.norm(along)),1e-9)
        normal=np.array([-along[1],along[0]])
        inverse=np.linalg.inv(self.calibration.plane_homography(.122))
        white=(hsv[:,:,1]<65)&(hsv[:,:,2]>150)
        # The fixed Gazebo body is neutral grey, unlike the dark printed floor.
        # This corroborates tiny JPEG sleeve fragments without promoting floor
        # compression noise to robots. Real lighting needs a trained detector.
        centre=_project(inverse,np.asarray([midpoint]))[0]
        edge=_project(inverse,np.asarray([midpoint+np.array([.07,0.])]))[0]
        rad=max(2,int(math.ceil(np.linalg.norm(edge-centre))))
        ux,vy=map(int,np.rint(centre))
        crop=hsv[max(0,vy-rad):min(bgr.shape[0],vy+rad+1),max(0,ux-rad):min(bgr.shape[1],ux+rad+1)]
        if not crop.size or np.mean((crop[:,:,1]<65)&(crop[:,:,2]>=60))<.30:
            return False
        for sign in (-1,1):
            world=[midpoint+sign*.045*normal+side*.103*along for side in (-1,1)]
            pixels=_project(inverse,np.asarray(world))
            passed=[]
            for position,pixel in zip(world,pixels):
                edge=_project(inverse,np.asarray([position+np.array([.040,0.])]))[0]
                radius=max(2,int(math.ceil(np.linalg.norm(edge-pixel))))
                ux,vy=map(int,np.rint(pixel))
                x0,y0=max(0,ux-radius),max(0,vy-radius)
                x1,y1=min(bgr.shape[1],ux+radius+1),min(bgr.shape[0],vy+radius+1)
                patch=white[y0:y1,x0:x1]
                yy,xx=np.ogrid[y0:y1,x0:x1]
                passed.append(np.count_nonzero(patch&((xx-ux)**2+(yy-vy)**2<=radius**2))>=2)
            if all(passed):
                return True
        return False

    def detect(self,bgr):
        cal=self.calibration
        if bgr is None or bgr.shape[:2]!=(cal.image_size[1],cal.image_size[0]):
            return []
        comps=sleeve_components(bgr,overhead=True)
        hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
        for c in comps:
            try:
                xy=cal.project([[c["u"],c["v"]]],self.sleeve_height_m)[0]
                c["xy"]=(float(xy[0]),float(xy[1]))
            except ValueError:
                c["xy"]=None
        proposals=[]
        for i,a in enumerate(comps):
            if a["xy"] is None:
                continue
            for j in range(i+1,len(comps)):
                b=comps[j]
                if b["xy"] is None or a["team"]!=b["team"]:
                    continue
                if a["area"]+b["area"]<4 or max(a["area"],b["area"])/min(a["area"],b["area"])>6.:
                    continue
                d=math.dist(a["xy"],b["xy"])
                if abs(d-self.separation_m)>.055:
                    continue
                # Adjacent white receiver evidence on its own height plane.
                midpoint=(np.asarray(a["xy"])+np.asarray(b["xy"]))/2
                if not self._domes_visible(bgr,a,b,midpoint,hsv):
                    continue
                proposals.append((abs(d-self.separation_m),i,j,midpoint))
        used=set(); out=[]; observations=[]
        for error,i,j,midpoint in sorted(proposals,key=lambda v:v[0]):
            if i in used or j in used:
                continue
            used.update((i,j)); team=comps[i]["team"]
            # Black bands can split each sleeve into two colour components.
            # Both resulting pairs belong to one body, not two adjacent UGVs.
            duplicate=next((k for k,d in enumerate(out) if d.team==team and
                            math.hypot(d.x-midpoint[0],d.y-midpoint[1])<.15),None)
            if duplicate is not None:
                old=out[duplicate]; obs=observations[duplicate]
                count=obs.get("merged_pair_count",1)
                old.x=(old.x*count+float(midpoint[0]))/(count+1)
                old.y=(old.y*count+float(midpoint[1]))/(count+1)
                obs["xy"]=[old.x,old.y]; obs["merged_pair_count"]=count+1
                continue
            out.append(UavDet(float(midpoint[0]),float(midpoint[1]),team=team))
            observations.append({"team":team,"xy":midpoint.tolist(),"pixel_pair":[
                [comps[i]["u"],comps[i]["v"]],[comps[j]["u"],comps[j]["v"]]],
                "pair_separation_error_m":error,"bounded_body_offset_m":.04})
        self.last_observations=observations
        return out
