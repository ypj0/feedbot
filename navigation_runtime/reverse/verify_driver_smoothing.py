"""Differential execution of original x86-64 smoothing vs explicit-state C."""
import ctypes as C, hashlib, json, random, struct, sys
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_X86, UC_MODE_64, UC_HOOK_CODE
from unicorn.x86_const import *
ROOT=Path(__file__).resolve().parents[2]
p=ROOT/'analysis/reverse_artifacts/robot_driver'
sha='03d96108d91b8661c24310e2d13f558b9c5865bd2aef268f9c0664e37cc23640'
assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
u=Uc(UC_ARCH_X86,UC_MODE_64);u.mem_map(0,0x400000);u.mem_map(0x500000,0x30000)
with p.open('rb') as f:
    for s in ELFFile(f).iter_segments():
        if s['p_type']=='PT_LOAD':u.mem_write(s['p_vaddr'],s.data())
lib=C.CDLL(sys.argv[1]);fn=lib.recovered_smooth_pwm
fn.argtypes=[C.POINTER(C.c_float),C.c_int16,C.c_int16,C.c_int32,C.c_int32,C.c_float,C.c_float,C.POINTER(C.c_double)];fn.restype=C.c_uint32
ctx={}
def hook(uc,a,size,_):
    if a==0x3b8e0:uc.reg_write(UC_X86_REG_RAX,123)
    elif a==0x3b380:uc.mem_write(uc.reg_read(UC_X86_REG_RDI),struct.pack('<ii',ctx['sec'],ctx['nsec']))
    else:return
    sp=uc.reg_read(UC_X86_REG_RSP);dest=struct.unpack('<Q',uc.mem_read(sp,8))[0]
    uc.reg_write(UC_X86_REG_RSP,sp+8);uc.reg_write(UC_X86_REG_RIP,dest)
u.hook_add(UC_HOOK_CODE,hook)
u.mem_write(0x29fb00,b'\1')
r=random.Random(20261107);count=0
# First 5000 independent states, followed by 5000 sequential updates.
state=(C.c_float*2)(0,0)
for i in range(10000):
    if i<5000:state=(C.c_float*2)(r.uniform(-32768,32767),r.uniform(-32768,32767))
    v,w=(0,0) if i%5==0 else (r.randint(-32768,32767),r.randint(-32768,32767))
    ctx.update(sec=r.choice([0,0,1]),nsec=r.choice([0,1,10000000,50000000,999999999,r.randrange(1000000000)]))
    cap=C.c_float(r.choice([0,.01,.05,.1,1])).value;ran=C.c_float(r.uniform(1,65536)).value
    params=(C.c_double*4)(r.random(),r.uniform(1,5000),r.uniform(1,5000),r.random())
    u.mem_write(0x500460,bytes(params));u.mem_write(0x29fb14,struct.pack('<f',state[0]));u.mem_write(0x29fb10,struct.pack('<f',state[1]))
    u.mem_write(0x510000,struct.pack('<Q',0x520000));u.reg_write(UC_X86_REG_RSP,0x510000)
    for reg,val in [(UC_X86_REG_RDI,0x500000),(UC_X86_REG_ESI,v&0xffffffff),(UC_X86_REG_EDX,w&0xffffffff),(UC_X86_REG_EFLAGS,2)]:u.reg_write(reg,val)
    for reg,val in [(UC_X86_REG_XMM0,cap),(UC_X86_REG_XMM1,ran)]:u.reg_write(reg,struct.unpack('<I',struct.pack('<f',val))[0])
    u.emu_start(0x3f680,0x520000,count=1000)
    assert u.reg_read(UC_X86_REG_RIP)==0x520000
    result=fn(state,v,w,ctx['sec'],ctx['nsec'],cap,ran,params)
    assert result==u.reg_read(UC_X86_REG_EAX),(i,'packed output')
    assert bytes(state)==bytes(u.mem_read(0x29fb14,4)+u.mem_read(0x29fb10,4)),(i,'state',list(state))
    count+=1
report={'passed':True,'cases':count,'comparison':'bit-exact two float states and packed int16 output','binary_sha256':sha,'c_sha256':hashlib.sha256((ROOT/'navigation_runtime/reverse/recovered/driver_smoothing.c').read_bytes()).hexdigest(),'scope':'Finite states, positive range/rates, nonnegative duration; initialized original statics. ROS time now/subtraction intercepted. No first-call guard, exceptions, threads, or hardware verification.'}
(ROOT/'analysis/chassis_stack/driver_smoothing_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
