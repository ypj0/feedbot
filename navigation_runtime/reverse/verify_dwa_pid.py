"""Original custom DWA PID functions vs recovered explicit numeric model."""
import ctypes as C,hashlib,json,random,struct,sys
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_X86,UC_MODE_64
from unicorn.x86_const import *
root=Path(__file__).resolve().parents[2];p=root/'analysis/chassis_stack/binaries/libdwa_local_planner.so';sha='c5465c47683100fa835625e09ce53ef91620eaa20fb97c21b0b2d1503183e5e4';assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
u=Uc(UC_ARCH_X86,UC_MODE_64);u.mem_map(0,0x400000);u.mem_map(0x500000,0x40000)
with p.open('rb') as f:
 for seg in ELFFile(f).iter_segments():
  if seg['p_type']=='PT_LOAD':u.mem_write(seg['p_vaddr'],seg.data())
u.mem_write(0x27ae50,struct.pack('<Q',0x503000));u.mem_write(0x503000,b'\1');u.mem_write(0x27bcc0,b'\1\0\0\0\1\0\0\0')
lib=C.CDLL(sys.argv[1]);fn=lib.recovered_dwa_pid;fn.argtypes=[C.c_void_p,C.c_float,C.c_int];fn.restype=C.c_float
r=random.Random(77);count=0
for direction in [0,1]:
 for i in range(2500):
  s=bytearray(r.randbytes(0xa00));off=0x900 if direction else 0x8d4
  for k in range(11):struct.pack_into('<f',s,off+k*4,r.uniform(-2,2))
  struct.pack_into('<d',s,0x970,r.choice([.399999999,.4,.400000001,.699999999,.7,.700000001,r.uniform(0,1)]))
  command=C.c_float(r.choice([0.,r.uniform(-2,2)])).value
  u.mem_write(0x500000,bytes(s));u.mem_write(0x520000,struct.pack('<Q',0x530000))
  for reg,val in [(UC_X86_REG_RSP,0x520000),(UC_X86_REG_RDI,0x500000),(UC_X86_REG_EFLAGS,2),(UC_X86_REG_XMM0,struct.unpack('<I',struct.pack('<f',command))[0])]:u.reg_write(reg,val)
  u.emu_start(0x31bc0 if direction else 0x317e0,0x530000,count=1000);assert u.reg_read(UC_X86_REG_RIP)==0x530000
  cs=C.create_string_buffer(bytes(s),len(s));result=fn(cs,command,direction)
  assert cs.raw==u.mem_read(0x500000,len(s)),(direction,i,'state',[(hex(j),a,b) for j,(a,b) in enumerate(zip(cs.raw,u.mem_read(0x500000,len(s)))) if a!=b][:12])
  assert struct.pack('<f',result)==struct.pack('<I',u.reg_read(UC_X86_REG_XMM0)&0xffffffff),(direction,i,'result')
  count+=1
report={'passed':True,'cases':count,'binary_sha256':sha,'c_sha256':hashlib.sha256((root/'navigation_runtime/reverse/recovered/dwa_pid.c').read_bytes()).hexdigest(),'scope':'Full line and direction PID functions, finite states incl saturation and speed threshold boundaries; log flags initialized disabled, no ROS log effects. Exact state and returned float bits. No claim of complete DWA planner equivalence.'}
(root/'analysis/chassis_stack/dwa_pid_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
