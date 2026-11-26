"""Full MCU 0x0800d414 original soft-float execution vs C model."""
import ctypes as C,hashlib,json,random,struct,sys
from pathlib import Path
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import *
root=Path(__file__).resolve().parents[2];p=root/'analysis/reverse_artifacts/sub_image.bin';sha='dbc09a32d4c548b5ddd6c9c60bf622bbb0a9120dddd1f78fed2ed873aec9d9c1';assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.mem_map(0x8000000,0x20000);u.mem_write(0x8005000,p.read_bytes());u.mem_map(0x20000000,0x10000)
ctx={};trace=[]
def hook(uc,a,n,d):
 if a==0x80062bc:uc.reg_write(UC_ARM_REG_R0,ctx['gate']&0xffffffff)
 elif a==0x800b0e8:trace.append([C.c_int32(uc.reg_read(r)).value for r in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2]])
 else:return
 uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
u.hook_add(UC_HOOK_CODE,hook)
lib=C.CDLL(sys.argv[1]);fn=lib.recovered_sub_kinematics;fn.argtypes=[C.c_void_p,C.c_uint8,C.c_int,C.POINTER(C.c_uint16),C.c_uint32,C.POINTER(C.c_int32)]
r=random.Random(69);count=0
for i in range(4000):
 s=bytearray(r.randbytes(24));struct.pack_into('<Hhh',s,0,r.randrange(3),r.choice([0,1,-1,500,-500,r.randint(-2000,2000)]),r.choice([0,100,-100,r.randint(-3000,3000)]))
 s[7]=r.randint(50,255);struct.pack_into('<I',s,8,r.randint(1,500));struct.pack_into('<H',s,12,r.randint(100,1000));s[14]=r.randint(1,50);struct.pack_into('<h',s,16,r.choice([-1,0,1,10,32767]))
 mode=r.randrange(4);ctx['gate']=r.choice([-1,0,1,100]);near=(C.c_uint16*3)(*[r.choice([0,19,20,29,30,31,500]) for _ in range(3)]);arg=r.randint(0,65535)
 u.mem_write(0x20001e68,bytes(s));u.mem_write(0x20001e4c,bytes([mode]));u.mem_write(0x20001e58,struct.pack('<I',0x20003000))
 for j,n in enumerate(near):u.mem_write(0x20003000+j*14,struct.pack('<H',n))
 trace.clear()
 for reg,val in [(UC_ARM_REG_R0,arg),(UC_ARM_REG_SP,0x2000f000),(UC_ARM_REG_LR,0x8018001)]:u.reg_write(reg,val)
 u.emu_start(0x800d415,0x8018000,count=20000);assert u.reg_read(UC_ARM_REG_PC)==0x8018000
 cs=C.create_string_buffer(bytes(s),len(s));out=(C.c_int32*3)();fn(cs,mode,ctx['gate'],near,arg,out)
 assert trace==[list(out)],(i,trace,list(out),s.hex(),arg)
 assert cs.raw==u.mem_read(0x20001e68,len(s)),(i,'state')
 count+=1
report={'passed':True,'cases':count,'binary_sha256':sha,'c_sha256':hashlib.sha256((root/'navigation_runtime/reverse/recovered/sub_kinematics.c').read_bytes()).hexdigest(),'scope':'Full 0x0800d414 including original software floating-point arithmetic, local near-threshold branch, counter updates, signed16 targets and third sink argument. External gate/sink intercepted. Nonzero calibration divisor, finite values and in-range int32 conversion. No physical-unit or hardware proof.'}
(root/'analysis/reverse_artifacts/sub_kinematics_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
