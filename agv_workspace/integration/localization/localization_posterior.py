"""Innovation gate for an AMCL pose distribution, in map-frame metres/radians.

The observer tests compatibility of scan evidence with the supplied posterior;
it does not publish a corrected pose or modify collision geometry.
"""
import math
import numpy as np
# 95% chi-square gate for three jointly observed planar pose dimensions.
CHI2_95=7.814727903251179
# Same spatial/angular separation used by global acquisition for distinct modes.
MODE_DISTANCE=.4
MODE_ANGLE=.26

def covariance_matrix(value, require_converged=True):
    try:c=np.asarray(value,dtype=float).reshape(3,3)
    except (ValueError,TypeError):return None
    if not np.isfinite(c).all() or not np.allclose(c,c.T,atol=1e-10):return None
    if np.linalg.eigvalsh(c).min()<=0:return None
    if require_converged and math.sqrt(CHI2_95*np.linalg.eigvalsh(c[:2,:2]).max())>MODE_DISTANCE:return None
    if require_converged and math.sqrt(CHI2_95*c[2,2])>MODE_ANGLE:return None
    return c

def innovation(predicted,measured,covariance):
    d=np.asarray(measured)-np.asarray(predicted)
    d[2]=math.atan2(math.sin(d[2]),math.cos(d[2]))
    value=float(d@np.linalg.solve(covariance,d))
    return dict(delta=d.tolist(),mahalanobis_squared=value,limit=CHI2_95,compatible=value<=CHI2_95)

class CovarianceHistory:
    """Timestamped AMCL posteriors; a future estimate cannot verify an older scan."""
    def __init__(self):
        from collections import deque
        self.samples=deque(maxlen=64);self.minimum_stamp=float("-inf")
    def clear(self, minimum_stamp=float("-inf")):
        self.samples.clear();self.minimum_stamp=minimum_stamp
    def add(self,stamp,frame,covariance):
        if frame!='map' or not math.isfinite(stamp) or stamp<self.minimum_stamp:return
        if self.samples and stamp<self.samples[-1][0]:return
        c=list(covariance)
        if len(c)!=36:return
        self.samples.append((stamp,[[c[i*6+j] for j in (0,1,5)] for i in (0,1,5)]))
    def at(self,stamp):
        return next(((t,c) for t,c in reversed(self.samples) if t<=stamp),None)
