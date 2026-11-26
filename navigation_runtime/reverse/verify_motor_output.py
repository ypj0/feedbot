"""Original Thumb execution vs recovered motor-output C. Offline CAN sink only."""
import ctypes as C,hashlib,json,random,struct,sys
from pathlib import Path
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import *
from inspect_hex import read_hex
root=Path(__file__).resolve().parents[2];p=root/'analysis/reverse_artifacts/sub.hex'
assert hashlib.sha256(p.read_bytes()).hexdigest()=='5baeec4873058a2e0b92c04a979b8acd5bae2062cf47194f06e909fc8e9304df'
m,_,_=read_hex(p);base=min(m);raw=bytes(m[a] for a in range(base,max(m)+1))
class Frame(C.Structure):_fields_=[('id',C.c_uint32),('data',C.c_uint8*8)]
class Output(C.Structure):_fields_=[('count',C.c_uint32),('frames',Frame*2)]
lib=C.CDLL(sys.argv[1]);lib.recovered_motor_commit.argtypes=[C.c_void_p,C.c_void_p,C.c_int32,C.c_int32,C.c_uint32,C.POINTER(Output)]
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.mem_map(0x8000000,0x20000);u.mem_write(base,raw);u.mem_map(0x20000000,0x10000)
END=0x8018000;trace=[]
def hook(uc,a,n,data):
 if a==0x8005bfc:
  ident=uc.reg_read(UC_ARM_REG_R0);ptr=uc.reg_read(UC_ARM_REG_R1);size=uc.reg_read(UC_ARM_REG_R2)
  assert size==8;trace.append((ident,bytes(uc.mem_read(ptr,size))))
  uc.reg_write(UC_ARM_REG_R0,0);uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
u.hook_add(UC_HOOK_CODE,hook)
rng=random.Random(441);cases=0;example=None
for sa in [0,0x31,0x37]:
 for sb in [0,0x31,0x37]:
  for a,b in [(0,0),(100,100),(-100,100),(32767,-32768),(-32768,32767)]+[(rng.randint(-32768,32767),rng.randint(-32768,32767)) for _ in range(40)]:
   s=bytearray(rng.randbytes(0x48));s[0x24]=sa;s[0x3c]=sb
   flags=bytes([rng.randrange(2),rng.randrange(2)]);span=rng.randrange(65536)
   u.mem_write(0x20001ea4,bytes(s));u.mem_write(0x200000b5,flags)
   u.reg_write(UC_ARM_REG_SP,0x2000f000);u.reg_write(UC_ARM_REG_LR,END|1)
   for reg,v in [(UC_ARM_REG_R0,a),(UC_ARM_REG_R1,b),(UC_ARM_REG_R2,span)]:u.reg_write(reg,v&0xffffffff)
   trace.clear();u.emu_start(0x800b0e9,END,count=10000);assert u.reg_read(UC_ARM_REG_PC)==END
   cs=(C.c_uint8*len(s)).from_buffer_copy(s);cf=(C.c_uint8*2).from_buffer_copy(flags);o=Output()
   lib.recovered_motor_commit(cs,cf,a,b,span,C.byref(o))
   assert bytes(cs)==bytes(u.mem_read(0x20001ea4,len(s)))
   assert bytes(cf)==bytes(u.mem_read(0x200000b5,2))
   expected=[(f.id,bytes(f.data)) for f in o.frames[:o.count]];assert trace==expected,(a,b,trace,expected)
   if sa==sb==0x37 and a==b==100:example=[dict(id=hex(i),data=d.hex()) for i,d in trace]
   cases+=1
print(json.dumps(dict(passed=True,cases=cases,functions=['0x0800b0e8','0x0800ab00','0x0800ab84'],
 example_input=[100,100],example_sink=example,scope='Original instructions through recorded send boundary 0x08005bfc; no CAN hardware or target-unit validation'),indent=2))
