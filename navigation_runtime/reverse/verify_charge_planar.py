"""Whole original function execution vs recovered planar C, math/ROS ABI stubs.
No hardware execution; finite planar inputs only; logs/strings not validated.
"""
import runpy,sys,math,ctypes as C,struct,json,hashlib
from pathlib import Path
# Reuse fingerprint-bound ELF emulator and run the existing regression first.
g=runpy.run_path(str(Path(__file__).with_name('verify_charge_core.py')))
u,lib,BASE,TWIST,STACK,END,ROOT,rng=[g[k] for k in ('u','lib','BASE','TWIST','STACK','END','ROOT','rng')]
from unicorn import UC_HOOK_CODE
from unicorn.x86_const import *
RPOSE,DPOSE=BASE+0x4000,BASE+0x4200
u.mem_write(0x37abc0,struct.pack('<Q',BASE+0x1000));u.mem_write(BASE+0x1000,b'\1')
for addr in (0x37e550,0x37e560,0x37e570):u.mem_write(addr,b'\1\0\0\0\1\0\0\0')
def getx(reg=UC_X86_REG_XMM0):return struct.unpack('<d',struct.pack('<Q',u.reg_read(reg)&((1<<64)-1)))[0]
def setx(val):u.reg_write(UC_X86_REG_XMM0,struct.unpack('<Q',struct.pack('<d',val))[0])
maths={0x6a3d0:lambda:math.atan2(getx(),getx(UC_X86_REG_XMM1)),0x6c280:lambda:math.fmod(getx(),getx(UC_X86_REG_XMM1)),0x6cb00:lambda:math.asin(getx()),0x6cfc0:lambda:math.cos(getx()),0x6b470:lambda:math.sqrt(getx())}
def hooks(uc,a,size,_):
    if a in maths:setx(maths[a]())
    elif a==0x6b050:
        x=getx();u.mem_write(u.reg_read(UC_X86_REG_RDI),struct.pack('<d',math.sin(x)));u.mem_write(u.reg_read(UC_X86_REG_RSI),struct.pack('<d',math.cos(x)))
    elif a in (0x103620,0x6ae10):
        p=u.reg_read(UC_X86_REG_RDI);u.mem_write(p,struct.pack('<QQ',p+16,0));u.mem_write(p+16,bytes(16))
    elif a==0x6b910:pass
    else:return
    sp=u.reg_read(UC_X86_REG_RSP);dest=struct.unpack('<Q',u.mem_read(sp,8))[0]
    u.reg_write(UC_X86_REG_RSP,sp+8);u.reg_write(UC_X86_REG_RIP,dest)
u.hook_add(UC_HOOK_CODE,hooks)
def pose(ptr,p):
    b=bytearray(104);struct.pack_into('<QQ',b,16,ptr+32,0)
    struct.pack_into('<ddd',b,48,p[0],p[1],0)
    z,w=math.sin(p[2]/2),math.cos(p[2]/2)
    struct.pack_into('<dddd',b,72,0,0,z,w);u.mem_write(ptr,bytes(b))
    # Same planar matrix yaw extraction as the original TF conversion.
    k=2/(z*z+w*w)
    return (C.c_double*3)(p[0],p[1],math.atan2(w*z*k,1-z*z*k))
counts={};maxerr=0.
for name,entry in [('approach',0x104710),('final',0x103bc0)]:
    fn=getattr(lib,'recovered_'+name+'_planar');fn.argtypes=[C.c_void_p,C.c_void_p,C.c_void_p,C.c_void_p];fn.restype=C.c_int
    for i in range(1200):
        s,t=g['blob']()
        for off,v in [(0x6f8,.7),(0x700,.07),(0x708,.0349),(0x720,.3),(0x728,-.07),(0x730,.08),(0x740,.4),(0x768,rng.uniform(0,2)),(0x770,rng.uniform(0,1))]:struct.pack_into('<d',s,off,v)
        for off in (0x77d,0x77e,0x77f):s[off]=rng.randrange(2)
        for off in (0x764,0x778):struct.pack_into('<I',s,off,rng.choice([0,3,4,19,20,21]))
        struct.pack_into('<6d',t,0,rng.uniform(-.1,.1),0,0,0,0,rng.uniform(-.4,.4))
        r=[rng.uniform(-2,2),rng.uniform(-2,2),rng.uniform(-math.pi,math.pi)]
        d=[rng.uniform(-1,1),rng.uniform(-1,1),rng.uniform(-math.pi,math.pi)]
        if i%5==0:d[:2]=[r[0]+.2,r[1]]
        cs,ct=g['setup'](s,t)
        rp,dp=pose(RPOSE,r),pose(DPOSE,d)
        u.reg_write(UC_X86_REG_RSI,RPOSE);u.reg_write(UC_X86_REG_RDX,DPOSE);u.reg_write(UC_X86_REG_RCX,TWIST)
        u.emu_start(entry,END,count=10000)
        assert u.reg_read(UC_X86_REG_RIP)==END,(name,i,hex(u.reg_read(UC_X86_REG_RIP)))
        ret=fn(cs,ct,rp,dp)
        got=u.reg_read(UC_X86_REG_EAX)
        if name=='final':got&=255
        assert got==ret,(name,i,'return',got,ret,r,d)
        actual=bytearray(u.mem_read(BASE,len(s)));expected=bytearray(cs.raw)
        for off in (0x758,0x768,0x770):
            a=struct.unpack_from('<d',actual,off)[0];b=struct.unpack_from('<d',expected,off)[0]
            if actual[off:off+8]!=expected[off:off+8]:
                assert math.isclose(a,b,rel_tol=1e-11,abs_tol=1e-11),(name,i,hex(off),a,b)
                maxerr=max(maxerr,abs(a-b));actual[off:off+8]=expected[off:off+8]
        assert actual==expected,(name,i,'other state')
        for a,b in zip(struct.unpack('<6d',u.mem_read(TWIST,48)),struct.unpack('<6d',ct.raw)):
            assert math.isclose(a,b,rel_tol=1e-11,abs_tol=1e-11),(name,i,'twist',a,b,r,d)
            maxerr=max(maxerr,abs(a-b))
    counts[name]=1200
report={'cases':counts,'total':sum(counts.values()),'passed':True,'max_abs_difference':maxerr,'scope':'Whole approach/startGoCharing original functions, finite planar poses; ROS logging/string operations and external math intercepted. No deployment/TF/hardware verification. 1e-11 numeric comparison tolerance.','binary_sha256':g['SHA'],'c_sha256':hashlib.sha256((ROOT/'navigation_runtime/reverse/recovered/charge_core.c').read_bytes()).hexdigest()}
(ROOT/'analysis/chassis_stack/charge_planar_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
