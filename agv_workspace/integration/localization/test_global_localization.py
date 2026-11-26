import math,unittest
import numpy as np
from global_localization import search

def fixture(symmetric=False):
    a=np.zeros((90,100),dtype=int);a[:2,:]=100;a[-2:,:]=100;a[:,:2]=100;a[:,-2:]=100
    if not symmetric:a[10:30,20:28]=100;a[60:70,65:85]=100;a[40:46,75:98]=100
    grid=dict(width=100,height=90,resolution=.05,origin=[-2.,-1.,.2],data=a.ravel().tolist())
    p=(2.4,2.3,.45);points=[]
    for theta in np.linspace(-math.pi,math.pi,360,endpoint=False):
        for r in np.arange(.05,8.,.025):
            x=.27+r*math.cos(theta);y=r*math.sin(theta);gx=int((p[0]+math.cos(p[2])*x-math.sin(p[2])*y)/.05);gy=int((p[1]+math.sin(p[2])*x+math.cos(p[2])*y)/.05)
            if gx<0 or gx>=100 or gy<0 or gy>=90:break
            if a[gy,gx]:points.append([x,y]);break
    co,so=math.cos(.2),math.sin(.2);pose=[-2.+co*p[0]-so*p[1],-1.+so*p[0]+co*p[1],p[2]+.2]
    return grid,points,pose
class TestSearch(unittest.TestCase):
    def test_unique_and_origin(self):
        g,s,p=fixture();r=search(g,s);self.assertTrue(r['accepted'],r);self.assertLess(math.hypot(r['pose'][0]-p[0],r['pose'][1]-p[1]),.12);self.assertLess(abs(r['pose'][2]-p[2]),.08)
        self.assertTrue(search(g,s,p)['accepted'])
        self.assertFalse(search(g,s,[p[0]+1.,p[1],p[2]])['accepted'])
    def test_ambiguity(self):
        g,s,p=fixture(True);self.assertFalse(search(g,s)['accepted'])
    def test_insufficient(self):
        g,s,p=fixture();self.assertFalse(search(g,s[:20])['accepted'])
if __name__=='__main__':unittest.main()
