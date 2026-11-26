"""Integration differential: target -> kinematics -> motor -> recorded CAN sink."""
import ctypes as C,hashlib,json,random,struct,sys
from pathlib import Path
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import *
root=Path(__file__).resolve().parents[2];p=root/'analysis/reverse_artifacts/sub_image.bin';sha='dbc09a32d4c548b5ddd6c9c60bf622bbb0a9120dddd1f78fed2ed873aec9d9c1';assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
class Target(C.Structure):_fields_=[('pv',C.c_int16),('pw',C.c_int16),('tv',C.c_int16),('tw',C.c_int16),('calls',C.c_uint32),('arg',C.c_uint32)]
class Frame(C.Structure):_fields_=[('id',C.c_uint32),('data',C.c_uint8*8)]
class Output(C.Structure):_fields_=[('count',C.c_uint32),('frames',Frame*2)]
lib=C.CDLL(sys.argv[1]);lib.recovered_sub_target.argtypes=[C.POINTER(Target),C.c_float,C.c_int,C.c_int32,C.c_int16,C.c_uint32];lib.recovered_sub_kinematics.argtypes=[C.c_void_p,C.c_uint8,C.c_int,C.POINTER(C.c_uint16),C.c_uint32,C.POINTER(C.c_int32)];lib.recovered_motor_commit.argtypes=[C.c_void_p,C.c_void_p,C.c_int32,C.c_int32,C.c_uint32,C.POINTER(Output)]
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.mem_map(0x8000000,0x20000);u.mem_write(0x8005000,p.read_bytes());u.mem_map(0x20000000,0x10000);trace=[]
def hook(uc,a,n,d):
 if a==0x8005bfc:
  ident=uc.reg_read(UC_ARM_REG_R0);ptr=uc.reg_read(UC_ARM_REG_R1);assert uc.reg_read(UC_ARM_REG_R2)==8
  trace.append((ident,bytes(uc.mem_read(ptr,8))));uc.reg_write(UC_ARM_REG_R0,0);uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
u.hook_add(UC_HOOK_CODE,hook);r=random.Random(501)
for i in range(1500):
 s=bytearray(r.randbytes(24));struct.pack_into('<Hhh',s,0,r.randrange(3),13,-14);s[7]=r.randint(50,255);struct.pack_into('<I',s,8,r.randint(1,500));struct.pack_into('<H',s,12,r.randint(100,1000));s[14]=r.randint(1,50);struct.pack_into('<h',s,16,r.choice([-1,0,1,20]))
 motor=bytearray(r.randbytes(0x48));motor[1]=r.randrange(3);motor[0x24]=r.choice([0,0x37]);motor[0x3c]=r.choice([0,0x37]);struct.pack_into('<i',motor,4,r.choice([-1,0,1,100]))
 flags=r.randbytes(2);pv,pw=r.randint(-500,500),r.randint(-500,500);v,w=r.choice([-500,0,500]),r.choice([-1000,0,1000]);arg=r.randint(0,65535);sensor=r.choice([0,.3,.8,1.5,5]);mode=r.randrange(4);near=(C.c_uint16*3)(*[r.choice([0,19,20,29,30,500]) for _ in range(3)])
 for addr,data in [(0x20001e68,bytes(s)),(0x20001ea4,bytes(motor)),(0x200000a8,struct.pack('<hh',pv,pw)),(0x200000e4,struct.pack('<f',sensor)),(0x200000b5,flags),(0x20001e4c,bytes([mode])),(0x20001e58,struct.pack('<I',0x20003000))]:u.mem_write(addr,data)
 for j,n in enumerate(near):u.mem_write(0x20003000+14*j,struct.pack('<H',n))
 for reg,val in [(UC_ARM_REG_R0,v&0xffffffff),(UC_ARM_REG_R1,w&0xffffffff),(UC_ARM_REG_R2,arg),(UC_ARM_REG_SP,0x2000f000),(UC_ARM_REG_LR,0x8018001)]:u.reg_write(reg,val)
 trace.clear();u.emu_start(0x800d171,0x8018000,count=30000);assert u.reg_read(UC_ARM_REG_PC)==0x8018000
 cs=C.create_string_buffer(bytes(s),len(s));cm=C.create_string_buffer(bytes(motor),len(motor));cf=C.create_string_buffer(flags,2);target=Target(pv,pw,13,-14,0,0);out=Output()
 lib.recovered_sub_target(C.byref(target),sensor,motor[1],v,w,arg)
 if target.calls:
  C.memmove(C.addressof(cs)+2,struct.pack('<hh',target.tv,target.tw),4);targets=(C.c_int32*3)()
  lib.recovered_sub_kinematics(cs,mode,struct.unpack_from('<i',motor,4)[0],near,target.arg,targets)
  lib.recovered_motor_commit(cm,cf,*targets,C.byref(out))
 expected=[(f.id,bytes(f.data)) for f in out.frames[:out.count]]
 assert trace==expected,(i,'CAN',trace,expected)
 for addr,data in [(0x20001e68,cs.raw),(0x20001ea4,cm.raw),(0x200000b5,cf.raw),(0x200000a8,struct.pack('<hh',target.pv,target.pw))]:assert u.mem_read(addr,len(data))==data,(i,'memory',hex(addr))
report={'passed':True,'cases':1500,'binary_sha256':sha,'scope':'Original target -> kinematics -> motor commit -> frame encoding, including actual mode/gate getters and soft-float helpers; only final 0x08005bfc external send stubbed. Compare target/motor/flags/previous-state memory and all emitted IDs/payloads. No hardware/interruption or physical-units validation.'}
(root/'analysis/reverse_artifacts/sub_motion_chain_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
