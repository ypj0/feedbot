"""Stationary scan-to-map candidate search. No ROS, pose publication or motor output."""
import math
import numpy as np
from localization_measurement import static_support

from functools import lru_cache

@lru_cache(maxsize=2)
def likelihood_field(w,h,res,encoded):
    # Cache by immutable map content, not object identity: map edits cannot
    # accidentally reuse a previous obstacle field.
    occupied=np.frombuffer(encoded,dtype=np.int16).reshape(h,w)>=65
    # Endpoint agreement field; this tolerance is not navigation clearance.
    field=np.zeros((h,w),dtype=np.float32)
    n=math.ceil(.20/res)
    for dy in range(-n,n+1):
        for dx in range(-n,n+1):
            weight=math.exp(-.5*(math.hypot(dx,dy)*res/.10)**2)
            if weight<.01:continue
            y0=max(0,dy);y1=min(h,h+dy);x0=max(0,dx);x1=min(w,w+dx)
            field[y0:y1,x0:x1]=np.maximum(field[y0:y1,x0:x1],occupied[y0-dy:y1-dy,x0-dx:x1-dx]*weight)
    return field

def search(grid, scan, pose=None, covariance=None):
    res=grid['resolution'];w=grid['width'];h=grid['height'];origin=grid['origin']
    cells=np.asarray(grid['data']).reshape(h,w);occupied=cells>=65
    field=likelihood_field(w,h,res,cells.astype(np.int16).tobytes())
    pts=np.asarray(scan,dtype=float)
    pts=pts[np.isfinite(pts).all(axis=1)]
    if len(pts)<80:return {'accepted':False,'reason':'有效激光不足'}
    pts=pts[::max(1,math.ceil(len(pts)/240))]
    def score(poses,per_beam=False):
        result=[]
        for start in range(0,len(poses),256):
            p=poses[start:start+256];c=np.cos(p[:,2,None]);s=np.sin(p[:,2,None])
            gx=np.floor((p[:,0,None]+c*pts[:,0]-s*pts[:,1])/res).astype(int)
            gy=np.floor((p[:,1,None]+s*pts[:,0]+c*pts[:,1])/res).astype(int)
            inside=(gx>=0)&(gx<w)&(gy>=0)&(gy<h)
            q=field[np.clip(gy,0,h-1),np.clip(gx,0,w-1)]*inside
            result.extend(q if per_beam else q.mean(axis=1))
        return np.asarray(result)
    # Sample ray interiors once; evaluate all samples in NumPy rather than a
    # Python loop for every beam of every global candidate.
    interiors=[];beam_ids=[]
    for beam,(x,y) in enumerate(pts):
        length=math.hypot(x-.27,y);count=max(1,int((length-.2)/res))
        f=np.arange(1,count+1)*res/max(length,1e-6);f=f[f<1.-.15/max(length,1e-6)]
        interiors.extend(zip(.27+f*(x-.27),f*y));beam_ids.extend([beam]*len(f))
    interiors=np.asarray(interiors,dtype=float).reshape(-1,2);beam_ids=np.asarray(beam_ids,dtype=int)
    def ray_error(p):
        if not len(interiors):return 0.
        c=math.cos(p[2]);s=math.sin(p[2]);rx=interiors[:,0];ry=interiors[:,1]
        gx=np.floor((p[0]+c*rx-s*ry)/res).astype(int);gy=np.floor((p[1]+s*rx+c*ry)/res).astype(int)
        valid=(gx>=0)&(gx<w)&(gy>=0)&(gy<h)
        hit=occupied[np.clip(gy,0,h-1),np.clip(gx,0,w-1)]&valid
        return np.count_nonzero(np.bincount(beam_ids[hit],minlength=len(pts)))/len(pts)
    def combined(endpoint,conflict):
        # Two complementary map-agreement terms, equal weight in log space.
        # A map discrepancy is evidence, not an independent collision veto.
        return math.sqrt(max(0.,endpoint)*max(0.,1.-conflict))
    support=None;posterior=None;nominal_score=None;nominal_joint=None;predicted=None;recovery=None
    if pose is None:
        yy,xx=np.where(cells==0);stride=max(1,round(.15/res));keep=(xx%stride==0)&(yy%stride==0)
        xy=np.stack(((xx[keep]+.5)*res,(yy[keep]+.5)*res),axis=1)
        poses=np.concatenate([np.column_stack((xy,np.full(len(xy),a))) for a in np.arange(-math.pi,math.pi,math.pi/36)])
        if not len(poses):return {'accepted':False,'reason':'地图无可定位区域'}
        scores=score(poses);seeds=[]
        def different(a,b,dist=.25,angle=.15):
            return math.hypot(a[0]-b[0],a[1]-b[1])>dist or abs(math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2])))>angle
        for idx in np.argsort(scores)[::-1]:
            p=poses[idx]
            if all(different(p,b) for b in seeds):seeds.append(p)
            if len(seeds)>=40:break
        refined=[]
        offsets=np.array([(x,y,a) for x in (-.075,-.025,.025,.075) for y in (-.075,-.025,.025,.075) for a in np.deg2rad([-4,-2,0,2,4])])
        for p in seeds:
            candidates=p+offsets;sc=score(candidates);idx=np.argmax(sc);refined.append((float(sc[idx]),candidates[idx]))
        ranked=[(combined(v,ray_error(q)),v,q) for v,q in refined]
        ranked.sort(key=lambda item:item[0],reverse=True)
        joint,best,p=ranked[0]
        second=next((v for v,endpoint,q in ranked[1:] if different(p,q,.4,.26)),0.)
    else:
        co,so=math.cos(origin[2]),math.sin(origin[2]);dx=pose[0]-origin[0];dy=pose[1]-origin[1]
        p=np.array([co*dx+so*dy,-so*dx+co*dy,pose[2]-origin[2]])
        predicted=p.copy()
        beams=score(np.array([p]),per_beam=True)[0]
        support,static_mask=static_support(cells,pts,p,res,beams,return_mask=True)
        nominal_score=support['endpoint']
        nominal_joint=combined(nominal_score,ray_error(p))
        if covariance is not None:
            from localization_posterior import covariance_matrix,innovation,MODE_DISTANCE,MODE_ANGLE
            converged=covariance_matrix(covariance) is not None
            covariance=covariance_matrix(covariance,require_converged=False)
            if covariance is None:return dict(accepted=False,reason='定位协方差无效',invalid_data=True,measurement_model='tracked_posterior_consistency')
            # Convert the map-frame posterior into grid coordinates, just like p.
            rotation=np.array([[co,so,0.],[-so,co,0.],[0.,0.,1.]])
            covariance=rotation@covariance@rotation.T
            # Search independently of the covariance ellipse; a better match
            # outside it must be rejected, not clipped into an acceptable gate.
            xy=np.arange(-MODE_DISTANCE,MODE_DISTANCE+res*.25,res)
            angles=np.linspace(-MODE_ANGLE,MODE_ANGLE,17)
            offsets=np.array([(x,y,a) for x in xy for y in xy for a in angles])
            candidates=predicted+offsets
            # Freeze the static returns identified at the supplied pose. Searching
            # all returns lets unmapped foreground objects pull the optimum onto
            # a mapped wall, then relabel those same objects as localization evidence.
            def tracked_score(poses):
                values=score(poses,per_beam=True)
                return values[:,static_mask].mean(axis=1) if static_mask.any() else np.zeros(len(poses))
            coarse_scores=tracked_score(candidates)
            coarse=candidates[int(np.argmax(coarse_scores))]
            fine=np.array([(x,y,a) for x in np.linspace(-res,res,9) for y in np.linspace(-res,res,9) for a in np.linspace(-MODE_ANGLE/8,MODE_ANGLE/8,9)])
            refined=coarse+fine
            match=refined[int(np.argmax(tracked_score(refined)))]
            # The exact prediction remains a candidate when local occlusion
            # makes the all-return endpoint optimum worse on static evidence.
            choices=[]
            for q in (predicted,match):
                values=score(np.array([q]),per_beam=True)[0]
                evidence=dict(support,endpoint=float(values[static_mask].mean()) if static_mask.any() else 0.,raw_endpoint=float(values.mean()))
                choices.append((combined(evidence['endpoint'],ray_error(q)),q,evidence))
            _,p,support=max(choices,key=lambda item:item[0])
            posterior=innovation(predicted,p,covariance)
            # A tracking-prior disagreement is not permission to move, but it
            # can carry a separately certified stationary correction hypothesis.
            # Competing modes are scored with the SAME frozen static returns.
            separation=np.hypot(candidates[:,0]-p[0],candidates[:,1]-p[1])
            angles_delta=np.abs(np.arctan2(np.sin(candidates[:,2]-p[2]),np.cos(candidates[:,2]-p[2])))
            alternatives=(separation>MODE_DISTANCE)|(angles_delta>MODE_ANGLE)
            # sqrt(endpoint) is an upper bound on the competitor's joint score;
            # omitting its ray penalty is conservative, not optimistic.
            competitor=math.sqrt(float(coarse_scores[alternatives].max())) if alternatives.any() else 1.
            local_joint=combined(support['endpoint'],ray_error(p))
            delta=posterior['delta']
            recovery=dict(eligible=bool(converged and not posterior['compatible'] and
                nominal_joint>=.75 and support['endpoint']>=.75 and support['sufficient'] and
                local_joint>=.75 and local_joint-competitor>=.08 and
                math.hypot(delta[0],delta[1])<MODE_DISTANCE and abs(delta[2])<MODE_ANGLE),
                competing_mode_upper_bound=competitor,margin=local_joint-competitor)

        best=support['endpoint'];second=0.
    co,so=math.cos(origin[2]),math.sin(origin[2]);world=[origin[0]+co*p[0]-so*p[1],origin[1]+so*p[0]+co*p[1],math.atan2(math.sin(p[2]+origin[2]),math.cos(p[2]+origin[2]))]
    ray_conflict=ray_error(p)
    joint=combined(best,ray_conflict)
    accepted=(nominal_score is None or nominal_joint>=.75) and best>=.75 and joint>=.75 and joint-second>=.08 and (support is None or support['sufficient']) and (posterior is None or posterior['compatible'])
    if pose is not None and covariance is not None and posterior is not None and not converged and accepted:
        return dict(accepted=False,pending_convergence=True,reason='激光匹配一致，等待 AMCL 后验收敛',measurement_model='tracked_posterior_consistency',pose=world,score=best,joint_score=joint,support=support,innovation=posterior)
    return dict(accepted=accepted,stationary_recovery=recovery,measurement_model=("tracked_posterior_consistency" if posterior is not None else "tracked_static_support") if support is not None else "global_all_returns",support=support,innovation=posterior,nominal_score=nominal_score,nominal_joint_score=nominal_joint,pose=world,score=best,joint_score=joint,alternative_score=second,margin=joint-second,ray_conflict=ray_conflict,reason=(('激光与定位后验一致' if posterior is not None else '当前位置激光核验通过') if pose is not None else '激光匹配唯一且通过射线核验') if accepted else '匹配置信度或唯一性不足，保持静止')
