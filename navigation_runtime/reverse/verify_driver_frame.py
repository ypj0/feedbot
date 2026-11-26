"""Execute full normal-return getdatafromdriver, capture serial writes offline."""
import ctypes as C,hashlib,json,random,struct,sys
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_X86,UC_MODE_64,UC_HOOK_CODE
from unicorn.x86_const import *
root=Path(__file__).resolve().parents[2];p=root/'analysis/reverse_artifacts/robot_driver';sha='03d96108d91b8661c24310e2d13f558b9c5865bd2aef268f9c0664e37cc23640';assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
u=Uc(UC_ARCH_X86,UC_MODE_64);u.mem_map(0,0x400000);u.mem_map(0x500000,0x40000)
with p.open('rb') as f:
 for seg in ELFFile(f).iter_segments():
  if seg['p_type']=='PT_LOAD':u.mem_write(seg['p_vaddr'],seg.data())
ctx={};frames=[];sm=[];reads=[]
def hook(uc,a,n,_):
 if a==0x3f680:
  sm.append((C.c_int16(uc.reg_read(UC_X86_REG_ESI)).value,C.c_int16(uc.reg_read(UC_X86_REG_EDX)).value))
  uc.reg_write(UC_X86_REG_EAX,(ctx['smooth']&65535)|(ctx['smooth_w']&65535)<<16)
 elif a in (0x3bc10,0x3b330):
  x=uc.reg_read(UC_X86_REG_ESI)&65535;uc.reg_write(UC_X86_REG_EAX,((x&255)<<8)|(x>>8))
 elif a==0x3b3c0:
  x=uc.reg_read(UC_X86_REG_ESI);uc.reg_write(UC_X86_REG_EAX,int.from_bytes(x.to_bytes(4,'little'),'big'))
 elif a==0x6c750:
  ptr,n=struct.unpack('<QQ',uc.mem_read(uc.reg_read(UC_X86_REG_RSI),16));assert n==40;frames.append(bytes(uc.mem_read(ptr,n)));uc.reg_write(UC_X86_REG_RAX,n)
 elif a==0x46b60:
  reads.append((uc.reg_read(UC_X86_REG_ESI),uc.reg_read(UC_X86_REG_EDX)))
  uc.mem_write(uc.reg_read(UC_X86_REG_RCX),ctx['response']);uc.reg_write(UC_X86_REG_RAX,ctx['read_ok'])
 else:return
 sp=uc.reg_read(UC_X86_REG_RSP);dest=struct.unpack('<Q',uc.mem_read(sp,8))[0];uc.reg_write(UC_X86_REG_RSP,sp+8);uc.reg_write(UC_X86_REG_RIP,dest)
u.hook_add(UC_HOOK_CODE,hook);lib=C.CDLL(sys.argv[1]);fn=lib.recovered_driver_frame;fn.argtypes=[C.c_void_p,C.c_int,C.c_int16,C.c_void_p];fn.restype=C.c_int
r=random.Random(28);count=0
for i in range(2400):
 s=bytearray(r.randbytes(0xb00));msg=r.choice([-1,0,1,2,2,2,3,4]);s[0x738]=s[0x718] if i%2 else (s[0x718]^255)
 for off in [0x398,0x3a0,0x3a8]:struct.pack_into('<d',s,off,r.uniform(-500,500))
 if i%3==0:s[0x722:0x726]=bytes(4)
 struct.pack_into('<Q',s,0x578,r.choice([0,1,100]));s[0x40a]=s[0x40b]=r.choice([0,99,100,101,255])
 ctx.update(smooth=r.randint(-32768,32767),smooth_w=r.randint(-32768,32767),response=r.randbytes(40),read_ok=r.randrange(2));frames.clear();sm.clear();reads.clear()
 original_output=r.randbytes(40);u.mem_write(0x500000,bytes(s));u.mem_write(0x502000,original_output);u.mem_write(0x520000,struct.pack('<Q',0x530000))
 for reg,val in [(UC_X86_REG_RDI,0x500000),(UC_X86_REG_ESI,msg&0xffffffff),(UC_X86_REG_RDX,0x502000),(UC_X86_REG_RSP,0x520000),(UC_X86_REG_EFLAGS,2)]:u.reg_write(reg,val)
 u.emu_start(0x47830,0x530000,count=10000);assert u.reg_read(UC_X86_REG_RIP)==0x530000
 cs=C.create_string_buffer(bytes(s),len(s));out=C.create_string_buffer(40);sent=fn(cs,msg,ctx['smooth'],out)
 assert frames==([out.raw] if sent else []),(i,'frame',frames,out.raw.hex())
 assert cs.raw==u.mem_read(0x500000,len(s)),(i,'state',[(hex(j),a,b) for j,(a,b) in enumerate(zip(cs.raw,u.mem_read(0x500000,len(s)))) if a!=b][:10])
 assert len(sm)==int(msg==2 and s[0x718]==s[0x738])
 assert reads==([(msg,40)] if sent else [])
 assert (u.reg_read(UC_X86_REG_RAX)&255)==int(bool(sent and ctx['read_ok']))
 assert u.mem_read(0x502000,40)==(ctx['response'] if sent and ctx['read_ok'] else original_output)
 count+=1
report={'passed':True,'cases':count,'binary_sha256':sha,'c_sha256':hashlib.sha256((root/'navigation_runtime/reverse/recovered/driver_frame.c').read_bytes()).hexdigest(),'scope':'Whole original getdatafromdriver normal-return control flow: serial writes captured, read result injected, smoothing/helper calls intercepted; exact 40-byte output and entire driver object state. Different injected smoothing angular outputs verify it is discarded. Exceptions/concurrency/blocking IO excluded.'}
(root/'analysis/chassis_stack/driver_frame_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
