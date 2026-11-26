"""Full original target routine including original software float comparisons."""
import ctypes as C,hashlib,itertools,json,random,struct,sys
from pathlib import Path
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import *
root=Path(__file__).resolve().parents[2];p=root/'analysis/reverse_artifacts/sub_image.bin'
sha='dbc09a32d4c548b5ddd6c9c60bf622bbb0a9120dddd1f78fed2ed873aec9d9c1';assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
class State(C.Structure):_fields_=[('pv',C.c_int16),('pw',C.c_int16),('tv',C.c_int16),('tw',C.c_int16),('calls',C.c_uint32),('arg',C.c_uint32)]
lib=C.CDLL(sys.argv[1]);lib.recovered_sub_target.argtypes=[C.POINTER(State),C.c_float,C.c_int,C.c_int32,C.c_int16,C.c_uint32]
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.mem_map(0x8000000,0x20000);u.mem_write(0x8005000,p.read_bytes());u.mem_map(0x20000000,0x10000)
ctx={};trace=[]
def hook(uc,a,n,d):
 if a==0x8006300:uc.reg_write(UC_ARM_REG_R0,ctx['mode'])
 elif a==0x800d414:trace.append(uc.reg_read(UC_ARM_REG_R0))
 else:return
 uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
u.hook_add(UC_HOOK_CODE,hook)
r=random.Random(48);count=0
sensors=[-10,0,.29999998,.3,.30000004,1,1.4999999,1.5,1.5000001,10]
values=[-32768,-100,-81,-80,-51,-50,0,50,51,80,81,100,32767]
for sensor,mode,v,pv in itertools.product(sensors,[0,1,2],[-32768,-1,0,1,100,32767],values):
 pw=r.choice(values);w=r.choice(values);arg=r.getrandbits(32);s=State(pv,pw,13,-14,0,0);ctx['mode']=mode;trace.clear()
 u.mem_write(0x200000e4,struct.pack('<f',sensor));u.mem_write(0x200000a8,struct.pack('<hh',pv,pw));u.mem_write(0x20001e6a,struct.pack('<hh',13,-14))
 for reg,val in [(UC_ARM_REG_R0,v&0xffffffff),(UC_ARM_REG_R1,w&0xffffffff),(UC_ARM_REG_R2,arg),(UC_ARM_REG_SP,0x2000f000),(UC_ARM_REG_LR,0x8018001)]:u.reg_write(reg,val)
 u.emu_start(0x800d171,0x8018000,count=10000);assert u.reg_read(UC_ARM_REG_PC)==0x8018000
 lib.recovered_sub_target(C.byref(s),sensor,mode,v,w,arg)
 assert struct.pack('<hhhh',s.pv,s.pw,s.tv,s.tw)==u.mem_read(0x200000a8,4)+u.mem_read(0x20001e6a,4),(sensor,mode,v,pv,'state')
 assert trace==([s.arg] if s.calls else []),(sensor,mode,v,pv,'call')
 count+=1
report={'passed':True,'cases':count,'binary_sha256':sha,'c_sha256':hashlib.sha256((root/'navigation_runtime/reverse/recovered/sub_target.c').read_bytes()).hexdigest(),'scope':'Full 0x0800d170 including original float helper execution; mode input 0x08006300 and downstream sink 0x0800d414 intercepted without field mutation. Finite sensor inputs; no physical meaning assigned to sensor/mode.'}
(root/'analysis/reverse_artifacts/sub_target_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
