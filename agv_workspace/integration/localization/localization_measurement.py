"""Static-map evidence for an already tracked pose, with bounded short-return censoring.

A return before the first mapped surface can be an unmapped object. It supplies
collision evidence but no endpoint evidence about that hidden static surface.
Returns behind a mapped wall, unknown-space rays and insufficient static support
are never excused by this model. Global acquisition remains conservative.
"""
import math
import numpy as np

def static_support(cells, points, pose, resolution, endpoint_scores, return_mask=False):
    h,w=cells.shape
    c,s=math.cos(pose[2]),math.sin(pose[2])
    points=np.asarray(points);scores=np.asarray(endpoint_scores)
    # Laser extrinsic matches the direct hardware profile, as in the ray model.
    lengths=np.hypot(points[:,0]-.27,points[:,1])
    gx=np.floor((pose[0]+c*points[:,0]-s*points[:,1])/resolution).astype(int)
    gy=np.floor((pose[1]+s*points[:,0]+c*points[:,1])/resolution).astype(int)
    valid=(gx>=0)&(gx<w)&(gy>=0)&(gy<h)
    endpoint_free=valid&(cells[np.clip(gy,0,h-1),np.clip(gx,0,w-1)]==0)
    candidates=endpoint_free&(scores<math.exp(-2))&(lengths>0)
    censored=np.zeros(len(points),dtype=bool)
    # March only unsupported endpoints, stopping at the first occupied, unknown
    # or out-of-map cell. Never invent a static surface behind unobserved space.
    ids=np.flatnonzero(candidates)
    if len(ids):
        distances=np.arange(resolution,math.hypot(w,h)*resolution+resolution,resolution)
        dx=(points[ids,0]-.27)/lengths[ids];dy=points[ids,1]/lengths[ids]
        rx=.27+dx[:,None]*distances;ry=dy[:,None]*distances
        xx=np.floor((pose[0]+c*rx-s*ry)/resolution).astype(int)
        yy=np.floor((pose[1]+s*rx+c*ry)/resolution).astype(int)
        inside=(xx>=0)&(xx<w)&(yy>=0)&(yy<h)
        sampled=cells[np.clip(yy,0,h-1),np.clip(xx,0,w-1)]
        stop=(~inside)|(sampled<0)|(sampled>=65)
        any_stop=stop.any(axis=1);first=stop.argmax(axis=1);rows=np.arange(len(ids))
        mapped=any_stop&inside[rows,first]&(sampled[rows,first]>=65)
        censored[ids]=mapped&(distances[first]-lengths[ids]>.20)
    keep=~censored
    bearings=np.arctan2(points[keep,1],points[keep,0]-.27)
    # Circular span is independent of scan array ordering and angle wrap.
    if len(bearings)>1:
        angles=np.sort(np.mod(bearings,2*math.pi))
        span=2*math.pi-np.max(np.diff(np.r_[angles,angles[0]+2*math.pi]))
    else:span=0.
    fraction=float(keep.mean()) if len(keep) else 0.
    sufficient=int(keep.sum())>=80 and fraction>=.60 and span>=math.pi/3
    result=dict(endpoint=float(scores[keep].mean()) if keep.any() else 0.,
                raw_endpoint=float(scores.mean()),support_beams=int(keep.sum()),
                support_fraction=fraction,occluded_fraction=1.-fraction,
                support_angular_span=float(span),sufficient=bool(sufficient))
    return (result,keep) if return_mask else result
