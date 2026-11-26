"""Execute original orchestrator with controlled external effects vs recovered C."""
import ctypes as C, hashlib,itertools,json,random,struct,sys
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_X86,UC_MODE_64,UC_HOOK_CODE
from unicorn.x86_const import *
root=Path(__file__).resolve().parents[2];p=root/'analysis/chassis_stack/binaries/libmove_base.so'
sha='971b0895544c43f5ece5fd5cb73ac0ae03d5895046c373ffd01e4a94209e837e'
assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
u=Uc(UC_ARCH_X86,UC_MODE_64);u.mem_map(0,0x400000);u.mem_map(0x500000,0x40000)
with p.open('rb') as f:
 for seg in ELFFile(f).iter_segments():
  if seg['p_type']=='PT_LOAD':u.mem_write(seg['p_vaddr'],seg.data())
B,T,P,SP,END=0x500000,0x502000,0x504000,0x520000,0x530000
u.mem_write(0x37abc0,struct.pack('<Q',B+0x2000+0x1000));u.mem_write(B+0x3000,b'\1')
u.mem_write(0x37e590,b'\1\0\0\0\3\0\0\0')
u.mem_write(P,bytes(104))
ctx={};trace=[]
def action(op,arg,write):
 trace.append((op,arg))
 if op in [1,3,4]:
  # Model a controlled downstream control write; exercise same-cycle retention.
  write(T,struct.pack('<d',op*.125));write(B+0x748,bytes([ctx['mutation']])) if op==3 else None
 return {1:ctx['found'],2:ctx['expired'],3:ctx['approach'],4:ctx['final']}.get(op,0)
def hook(uc,a,n,_):
 op={0x6d0a0:1,0x6d080:2,0x6a930:3,0x6c980:4,0x6ba30:5,0x6cb20:6,0x6b120:7}.get(a)
 if op:
  arg=struct.unpack('<h',uc.mem_read(uc.reg_read(UC_X86_REG_RSI),2))[0] if op==6 else (uc.mem_read(B+0x67c,1)[0] if op==7 else 0)
  uc.reg_write(UC_X86_REG_RAX,action(op,arg,uc.mem_write)&0xffffffffffffffff)
 elif a==0x103620:
  ptr=uc.reg_read(UC_X86_REG_RDI);uc.mem_write(ptr,struct.pack('<QQ',ptr+16,0));uc.mem_write(ptr+16,bytes(16))
 elif a in (0x6b910,0x6b760):pass
 elif a in (0x6b420,0x6b6c0):uc.reg_write(UC_X86_REG_RAX,0)
 else:return
 sp=uc.reg_read(UC_X86_REG_RSP);dest=struct.unpack('<Q',uc.mem_read(sp,8))[0];uc.reg_write(UC_X86_REG_RSP,sp+8);uc.reg_write(UC_X86_REG_RIP,dest)
u.hook_add(UC_HOOK_CODE,hook)
lib=C.CDLL(sys.argv[1]);CALL=C.CFUNCTYPE(C.c_int,C.c_int,C.c_int,C.c_void_p)
lib.recovered_charge_execute.argtypes=[C.c_void_p,C.c_void_p,CALL,C.c_void_p];lib.recovered_charge_execute.restype=C.c_int
r=random.Random(84);count=0
for init,charging,arrived,found,expired,approach,final,mutation in itertools.product([0,1], [0,1], [0,1], [0,1,4], [0,1], [-1,0,1,4], [0,1], [0,1]):
 ctx.update(found=found,expired=expired,approach=approach,final=final,mutation=mutation)
 s=bytearray(r.randbytes(0x800));t=r.randbytes(48);s[8]=init;s[0x6e8]=charging;s[0x748]=arrived
 u.mem_write(B,bytes(s));u.mem_write(T,t);u.mem_write(SP,struct.pack('<Q',END))
 for reg,val in [(UC_X86_REG_RSP,SP),(UC_X86_REG_RDI,B),(UC_X86_REG_RSI,P),(UC_X86_REG_RDX,T),(UC_X86_REG_EFLAGS,2)]:u.reg_write(reg,val)
 trace.clear();u.emu_start(0x1066a0,END,count=3000)
 assert u.reg_read(UC_X86_REG_RIP)==END
 expected_trace=trace[:];trace.clear();cs=C.create_string_buffer(bytes(s),len(s));ct=C.create_string_buffer(t,len(t))
 def write(addr,data):C.memmove((C.addressof(ct)+addr-T) if addr>=T else (C.addressof(cs)+addr-B),data,len(data))
 @CALL
 def effect(op,arg,_):return action(op,arg,write)
 ret=lib.recovered_charge_execute(cs,ct,effect,None)
 assert ret==u.reg_read(UC_X86_REG_EAX),(count,'ret')
 assert trace==expected_trace,(count,'trace',trace,expected_trace)
 assert cs.raw==u.mem_read(B,len(s)),(count,'state')
 assert ct.raw==u.mem_read(T,len(t)),(count,'twist')
 count+=1
report={'passed':True,'cases':count,'binary_sha256':sha,'c_sha256':hashlib.sha256((root/'navigation_runtime/reverse/recovered/charge_execute.c').read_bytes()).hexdigest(),'scope':'Whole execute normal-return control flow; controlled downstream state effects, timeout predicate and publisher calls. Pose-copy/log/allocation/ROS time arithmetic and exceptions excluded.'}
(root/'analysis/chassis_stack/charge_execute_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
