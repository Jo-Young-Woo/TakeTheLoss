"""Sensor-only birth support; test geometry generates scans, never enters detector."""
import math
import numpy as np
import pytest
from maicon26.config import Config
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.lidar_blobs import BlobDetector
from maicon26.types import Pose2D,Scan
from maicon26.sim.world import _ray_aabb

def scene_scan(pose, bodies=(), stamp=0.):
    angles = np.linspace(-math.pi,math.pi,640,endpoint=False)
    r = np.full(640,8.)
    dx,dy = np.cos(angles),np.sin(angles)
    for x,y,radius in bodies:
        px,py=x-pose.x,y-pose.y
        b=px*dx+py*dy
        disc=b*b-(px*px+py*py-radius*radius)
        hit=(disc>=0)&(b>0)
        r[hit]=np.minimum(r[hit],b[hit]-np.sqrt(disc[hit]))
    return Scan(-math.pi,2*math.pi/640,r,stamp,8.)


def rectscan(pose,rectangles,stamp,noise=0.,seed=0):
 angles=np.linspace(-math.pi,math.pi,640,endpoint=False);r=np.full(640,8.)
 for j,a in enumerate(angles+pose.th):
  for rectangle in rectangles:
   hit=_ray_aabb(pose.x,pose.y,math.cos(a),math.sin(a),rectangle,8.)
   if hit is not None:r[j]=min(r[j],hit[0])
 if noise:
  mask=r<8.;r[mask]+=np.random.default_rng(seed).normal(0,noise,int(mask.sum()))
 return Scan(-math.pi,2*math.pi/640,r,stamp,8.)

def static_case(rectangles,require_prior=True):
 cfg=Config();cfg.progress.enabled=True;cfg.threat.lidar_birth_require_prior_interior=require_prior;g=OccupancyGrid(cfg.arena);det=BlobDetector(cfg);true=Pose2D(1.,2.,0.)
 counts=[]
 for i in range(40):
  offset=.08 if i==0 else -.04 if i==1 else .025*math.sin(i*.7)
  estimate=Pose2D(true.x+offset,true.y+.008*math.cos(i),true.th+.002*math.sin(i))
  sc=rectscan(true,rectangles,i*.1,.003,i);g.update(sc,estimate)
  found=det.update(sc,estimate,g,i*.1);counts.append(len(found))
 return det,counts


@pytest.mark.parametrize('scene',[
 [(2.,1.8,2.4,2.2)],
 [(2.,1.8,2.4,2.2),(2.,2.8,3.8,3.2)]])
def test_static_rectangles_with_pose_and_beam_noise_reproduce_then_reject(scene):
    old,old_counts=static_case(scene,require_prior=False)
    new,new_counts=static_case(scene,require_prior=True)
    assert old_counts[1]==1 and sum(old_counts)>=30
    assert not any(new_counts)
    assert new.stats['birth_rejected_no_prior_interior']>0
    assert new.stats['birth_accepted']==0
    assert new.stats['guarded_free_cells']>0


def test_map_free_labels_cannot_replace_actual_prior_ray_support():
    cfg=Config();cfg.progress.enabled=True;grid=OccupancyGrid(cfg.arena)
    grid.L[:]=grid.L_MIN
    pose=Pose2D(1.,2.,0.);det=BlobDetector(cfg)
    for i in range(8):
        scan=scene_scan(pose,[(2.,2.,.13)],i*.1)
        assert det.update(scan,pose,grid,i*.1)==[]
    assert det.stats['birth_rejected_no_prior_interior']>0


def test_invalid_rays_cannot_create_interior_support_and_reset_clears_history():
    cfg=Config();cfg.progress.enabled=True;grid=OccupancyGrid(cfg.arena)
    pose=Pose2D(1.,2.,0.);det=BlobDetector(cfg)
    for i,value in enumerate([np.nan,-np.inf,0.,-.1]):
        scan=Scan(-math.pi,2*math.pi/640,np.full(640,value),i*.1,8.)
        det.update(scan,pose,grid,i*.1)
        assert det.stats['guarded_free_cells']==0
    det.update(Scan(-math.pi,2*math.pi/640,np.full(640,np.inf),.5,8.),pose,grid,.5)
    assert det.stats['guarded_free_cells']>0
    det.reset()
    assert det._interior_free is None and det.stats['guarded_free_cells']==0


def test_legacy_detector_does_not_allocate_or_use_birth_support():
    pose=Pose2D(1.,2.,0.);results=[]
    for enabled in [True,False]:
        cfg=Config();cfg.progress.enabled=False
        cfg.threat.lidar_birth_require_prior_interior=enabled
        grid=OccupancyGrid(cfg.arena);grid.L[:]=grid.L_MIN;det=BlobDetector(cfg);row=[]
        for i in range(45):
            scan=scene_scan(pose,[(2.+.01*i,2.,.13)],i*.1)
            grid.update(scan,pose);row.append([vars(b) for b in det.update(scan,pose,grid,i*.1)])
        assert det._interior_free is None
        assert det.stats['birth_support_checks']==0
        results.append(row)
    assert results[0]==results[1]


def test_initially_unmapped_stationary_object_stays_occupancy_then_motion_into_surveyed_space_detects():
    cfg=Config();cfg.progress.enabled=True;grid=OccupancyGrid(cfg.arena)
    true=Pose2D(1.,2.,0.);det=BlobDetector(cfg)
    for i in range(12):
        scan=scene_scan(true,[(2.,2.,.13)],i*.1)
        grid.update(scan,true)
        assert det.update(scan,true,grid,i*.1)==[]
    # It remains a sensed obstacle, even though no robot-type threat is born.
    assert np.any(grid.occupied())
    scan=scene_scan(true,[(2.,2.7,.13)],1.3)
    grid.update(scan,true);found=det.update(scan,true,grid,1.3)
    assert len(found)==1 and abs(found[0].y-2.7)<.15


def test_appearance_in_surveyed_corridor_and_standing_retention():
    cfg=Config();cfg.progress.enabled=True;grid=OccupancyGrid(cfg.arena)
    pose=Pose2D(1.,2.,0.);det=BlobDetector(cfg)
    walls=[(.7,0.,5.,1.45),(.7,2.55,5.,4.8)]
    for i in range(3):
        scan=rectscan(pose,walls,i*.1);grid.update(scan,pose);det.update(scan,pose,grid,i*.1)
    bids=[]
    for i in range(3,30):
        scan=rectscan(pose,walls,i*.1)
        body=scene_scan(pose,[(2.,2.,.13)],i*.1)
        scan.ranges=np.minimum(scan.ranges,body.ranges)
        grid.update(scan,pose);found=det.update(scan,pose,grid,i*.1)
        assert len(found)==1
        assert math.hypot(found[0].x-2.,found[0].y-2.)<.15
        bids.append(found[0].bid)
    assert len(set(bids))==1
