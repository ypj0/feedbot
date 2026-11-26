"""Differential tests: original Thumb code vs recovered C, entirely offline.
Peripheral functions are recorded/stubbed, never hardware emulation claims.
"""
import ctypes as C,json,random,struct,sys
from pathlib import Path
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import *
from inspect_hex import read_hex
import hashlib
ROOT=Path(__file__).resolve().parents[2]
p=ROOT/'analysis/reverse_artifacts/sub.hex'
assert hashlib.sha256(p.read_bytes()).hexdigest()=='5baeec4873058a2e0b92c04a979b8acd5bae2062cf47194f06e909fc8e9304df'
mem,_,_=read_hex(p);base=min(mem);raw=bytes(mem[a] for a in range(base,max(mem)+1))
class Call(C.Structure):_fields_=[('function',C.c_uint32),('a',C.c_uint32),('b',C.c_uint32),('c',C.c_uint32)]
class State(C.Structure):
 _fields_=[('tick',C.c_uint32),('last_tick',C.c_uint32),('silence_count',C.c_uint16),('pending_action',C.c_uint16),('flag_c2',C.c_uint8),('flag_c1',C.c_uint8),('call_count',C.c_uint32),('calls',Call*16)]
class Task(C.Structure):_fields_=[('due',C.c_uint8),('pad',C.c_uint8),('remaining',C.c_uint16),('period',C.c_uint16),('pad2',C.c_uint16),('function',C.c_uint32)]
lib=C.CDLL(sys.argv[1]);lib.recovered_periodic.argtypes=[C.POINTER(State)]
lib.recovered_check_frame.argtypes=[C.c_void_p,C.c_size_t];lib.recovered_check_frame.restype=C.c_int
lib.recovered_scheduler.argtypes=[C.POINTER(Task)]
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.mem_map(0x8000000,0x20000);u.mem_write(base,raw);u.mem_map(0x20000000,0x10000)
END=0x8018000
arity={0x800d170:3,0x800dee4:2,0x800e53c:0,0x8008014:3,0x800a9cc:1,0x8007968:1}
calls=[]
def hook(uc,a,size,data):
 if a in arity:
  args=[uc.reg_read(r) for r in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2)][:arity[a]]
  calls.append([a]+args);uc.reg_write(UC_ARM_REG_R0,0);uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
u.hook_add(UC_HOOK_CODE,hook)
def run(addr,args=()):
 u.reg_write(UC_ARM_REG_SP,0x2000f000);u.reg_write(UC_ARM_REG_LR,END|1)
 for reg,v in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),args):u.reg_write(reg,v)
 u.emu_start(addr|1,END,count=100000)
 assert u.reg_read(UC_ARM_REG_PC)==END,'instruction budget exceeded'
 return u.reg_read(UC_ARM_REG_R0)
def write(a,fmt,v):u.mem_write(a,struct.pack('<'+fmt,v))
def read(a,fmt):return struct.unpack('<'+fmt,u.mem_read(a,struct.calcsize(fmt)))[0]
rng=random.Random(441);n=0;override=None
for action in [0,0x385,0x386,0x387,0x388,0x389,0x134,0x135,0x136,0x137,0x138,0x139,0xffff]:
 for count in [0,14,15,17,18,19,65535]:
  for tick,last in [(1000,0),(1001,0),(5,0xffffff00)]:
   s=State(tick,last,count,action,1,1,0)
   for a,f,v in [(0x20000074,'I',tick),(0x200000d8,'I',last),(0x200000bc,'H',count),(0x200000d4,'H',action),(0x200000c2,'B',1),(0x200000c1,'B',1)]:write(a,f,v)
   calls.clear();run(0x800c1f8);actual=list(calls);lib.recovered_periodic(C.byref(s))
   expected=[[c.function]+[c.a,c.b,c.c][:arity[c.function]] for c in s.calls[:s.call_count]]
   assert actual==expected,(action,count,actual,expected)
   assert [read(a,f) for a,f in [(0x200000bc,'H'),(0x200000d4,'H'),(0x200000c2,'B'),(0x200000c1,'B')]]==[s.silence_count,s.pending_action,s.flag_c2,s.flag_c1]
   if action==0x385 and count==18 and tick==1000:override=actual
   n+=1
framecases=0
for length in [40,80]:
 for trial in range(200):
  b=bytearray(rng.randbytes(length));b[:2]=b'\x7f\x7f';b[-2:]=b'\r\n';b[-3]=(-sum(b[:-3]))&255
  if trial%2:b[rng.randrange(length)]^=1
  u.mem_write(0x20004000,bytes(b));actual=run(0x800ceb6,(0x20004000,length))
  buf=C.create_string_buffer(bytes(b));expected=lib.recovered_check_frame(buf,length)
  assert actual==expected;framecases+=1
for _ in range(200):
 tasks=(Task*8)()
 for t in tasks:t.due=rng.randrange(2);t.remaining=rng.choice([0,1,2,100,65535]);t.period=rng.choice([0,1,10,100,65535]);t.function=0x800c1f9
 u.mem_write(0x20000000,bytes(tasks));run(0x800b7fc);lib.recovered_scheduler(tasks)
 assert bytes(u.mem_read(0x20000000,C.sizeof(tasks)))==bytes(tasks)
print(json.dumps(dict(periodic_cases=n,frame_cases=framecases,scheduler_cases=200,passed=True,
 pending_action_after_timeout_calls=override,scope='Three recovered functions, opaque peripheral call boundaries; no physical MCU or whole firmware equivalence'),indent=2))
