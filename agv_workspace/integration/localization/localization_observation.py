"""Exact acquisition-time pairing; never attach an older covariance to a TF pose."""
from collections import OrderedDict
import math

class Observations:
    def __init__(self, capacity=128):
        self.capacity=capacity;self.scans=OrderedDict();self.posteriors=OrderedDict();self.minimum=0
    def clear(self, minimum=0):
        self.scans.clear();self.posteriors.clear();self.minimum=minimum
    def _put(self, target, stamp, value):
        if stamp<self.minimum:return
        target[stamp]=value
        while len(target)>self.capacity:target.popitem(last=False)
    def scan(self, stamp, value):self._put(self.scans,stamp,value)
    def posterior(self, stamp, frame, pose, covariance):
        if frame!='map' or len(pose)!=3 or not all(math.isfinite(x) for x in pose):return
        self._put(self.posteriors,stamp,(list(pose),list(covariance)))
    def latest(self, now_ns, after_ns):
        for stamp in sorted(self.posteriors,reverse=True):
            if stamp>after_ns and 0<=now_ns-stamp<2_000_000_000 and stamp in self.scans:
                return stamp,self.scans[stamp],self.posteriors[stamp]
        return None
