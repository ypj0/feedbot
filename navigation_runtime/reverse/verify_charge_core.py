#!/usr/bin/env python3
"""Compare offline C recovery to original x86-64 instructions, not ROS execution."""
import ctypes as C
import hashlib
import json
import random
import struct
import sys
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_X86, UC_MODE_64, UC_HOOK_CODE
from unicorn.x86_const import *

ROOT = Path(__file__).resolve().parents[2]
BINARY = ROOT / 'analysis/chassis_stack/binaries/libmove_base.so'
SHA = '971b0895544c43f5ece5fd5cb73ac0ae03d5895046c373ffd01e4a94209e837e'
assert hashlib.sha256(BINARY.read_bytes()).hexdigest() == SHA
lib = C.CDLL(sys.argv[1])
lib.recovered_found_charge.argtypes = [C.c_void_p,C.c_void_p,C.c_int,C.c_uint64]
lib.recovered_found_charge.restype = C.c_int
lib.recovered_charge_state.argtypes = [C.c_void_p,C.c_uint8]
lib.recovered_charge_approach_slice.argtypes = [C.c_void_p,C.c_void_p,C.c_double,C.c_int]
u = Uc(UC_ARCH_X86,UC_MODE_64)
u.mem_map(0,0x400000)
with BINARY.open('rb') as f:
    elf = ELFFile(f)
    for seg in elf.iter_segments():
        if seg['p_type'] == 'PT_LOAD': u.mem_write(seg['p_vaddr'],seg.data())
BASE, TWIST, STACK, END = 0x500000,0x502000,0x510000,0x520000
u.mem_map(BASE,0x30000)
ctx = {}
def intercept(uc,addr,size,_):
    if addr in (0x6c860,0x6b6c0):
        ctx['calls'].append(hex(addr))
        uc.reg_write(UC_X86_REG_RAX,ctx['perception'] if addr==0x6c860 else ctx['now'])
        sp=uc.reg_read(UC_X86_REG_RSP)
        dest=struct.unpack('<Q',uc.mem_read(sp,8))[0]
        uc.reg_write(UC_X86_REG_RSP,sp+8)
        uc.reg_write(UC_X86_REG_RIP,dest)
u.hook_add(UC_HOOK_CODE,intercept)
rng=random.Random(20261107)
counts={'found_charge':0,'state_callback':0,'approach_slice':0}
def setup(s,t):
    u.mem_write(BASE,bytes(s));u.mem_write(TWIST,bytes(t))
    u.mem_write(STACK,struct.pack('<Q',END)+bytes(64))
    u.reg_write(UC_X86_REG_RSP,STACK)
    u.reg_write(UC_X86_REG_RDI,BASE)
    u.reg_write(UC_X86_REG_RDX,TWIST)
    u.reg_write(UC_X86_REG_EFLAGS,2)
    return C.create_string_buffer(bytes(s),len(s)),C.create_string_buffer(bytes(t),len(t))
def check(s,t,cs,ct,label):
    assert u.mem_read(BASE,len(s))==cs.raw,(label,'object mismatch')
    assert u.mem_read(TWIST,len(t))==ct.raw,(label,'twist mismatch')
def blob():return bytearray(rng.randbytes(0x800)),bytearray(rng.randbytes(48))
for count in [0,148,149,150,448,449,450,748,749,750,0x7fffffff,0xffffffff]+[rng.randrange(2**32) for _ in range(100)]:
    for flag in [0,1]:
        for perception in [0,1]:
            for skip in [0,1]:
                s,t=blob();s[0xe4]=flag;s[0x6f0]=skip
                struct.pack_into('<I',s,0x6ec,count)
                cs,ct=setup(s,t)
                ctx.update(perception=perception,now=0x1234567812345678,calls=[])
                u.emu_start(0x103af0,END,count=500)
                assert u.reg_read(UC_X86_REG_RIP)==END
                result=lib.recovered_found_charge(cs,ct,perception,ctx['now'])
                assert u.reg_read(UC_X86_REG_EAX)==result
                check(s,t,cs,ct,('found',count,flag,perception,skip))
                expected=[] if flag else ['0x6c860']
                if flag or perception:expected.append('0x6b6c0')
                assert ctx['calls']==expected
                counts['found_charge']+=1
for value in range(256):
    s,t=blob();cs,ct=setup(s,t)
    u.mem_write(STACK+11,bytes([value]))
    u.emu_start(0x103ad0,END,count=50)
    assert u.reg_read(UC_X86_REG_RIP)==END
    lib.recovered_charge_state(cs,value)
    check(s,t,cs,ct,('state',value));counts['state_callback']+=1
cases=[]
for error in [-1.5,-0.8,-0.6,-0.5,-0.1,0,0.1,0.5,0.6,0.8,1.5]:
    for speed in [-0.8,-0.15,0,0.15,0.6,0.8]:
        for limit in [0.3,0.6,1.0]: cases.append((error,speed,limit,0.1,0.3))
for _ in range(500):cases.append((rng.uniform(-3.14,3.14),rng.uniform(-1,1),rng.uniform(.1,1.5),rng.uniform(0,.2),rng.uniform(.03,.5)))
for second in [0,1]:
    for error,speed,limit,tolerance,linear_max in cases:
        s,t=blob()
        for off,val in [(0x708,tolerance),(0x720,limit),(0x730,linear_max)]:struct.pack_into('<d',s,off,val)
        struct.pack_into('<d',t,0,rng.uniform(-.1,.6));struct.pack_into('<d',t,40,speed)
        cs,ct=setup(s,t)
        u.reg_write(UC_X86_REG_R12,BASE);u.reg_write(UC_X86_REG_R14,TWIST)
        u.reg_write(UC_X86_REG_XMM0,struct.unpack('<Q',struct.pack('<d',error))[0])
        u.emu_start(0x105768 if second else 0x1056e2,0x1055d0,count=150)
        assert u.reg_read(UC_X86_REG_RIP)==0x1055d0
        lib.recovered_charge_approach_slice(cs,ct,error,second)
        check(s,t,cs,ct,('approach',second,error,speed,limit,tolerance))
        counts['approach_slice']+=1
lib.recovered_charge_final_slice.argtypes=[C.c_void_p,C.c_void_p,C.c_double,C.c_double,C.c_int]
counts['final_slice']=0
for reverse in [0,1]:
    for error,speed,limit,tolerance,linear_max in cases:
        s,t=blob()
        distance=rng.uniform(0,2)
        struct.pack_into('<d',s,0x720,limit)
        struct.pack_into('<d',s,0x728,-linear_max)
        struct.pack_into('<d',s,0x768,rng.choice([distance,distance-0.01,distance+0.01]))
        struct.pack_into('<d',t,0,rng.uniform(-.5,.1))
        struct.pack_into('<d',t,40,speed)
        cs,ct=setup(s,t)
        frame=STACK+0x1000
        u.reg_write(UC_X86_REG_RBP,frame)
        u.reg_write(UC_X86_REG_R13,TWIST)
        u.mem_write(frame-0xe8,struct.pack('<Q',BASE))
        u.mem_write(frame-0xf0,struct.pack('<d',distance))
        u.mem_write(frame-0xf8,struct.pack('<d',0.0))
        for reg,val in [(UC_X86_REG_XMM2,error),(UC_X86_REG_XMM3,abs(error))]:
            u.reg_write(reg,struct.unpack('<Q',struct.pack('<d',val))[0])
        u.emu_start(0x104092 if reverse else 0x103f65,0x1040f4,count=150)
        assert u.reg_read(UC_X86_REG_RIP)==0x1040f4
        lib.recovered_charge_final_slice(cs,ct,error,distance,reverse)
        check(s,t,cs,ct,('final',reverse,error,speed,limit,distance))
        counts['final_slice']+=1
report={'binary_sha256':SHA,'cases':counts,'total':sum(counts.values()),'passed':True,
        'scope':'Full foundChargePoint and stateCallback leaves; two numeric approach slices and two final-entry slices. getRobotPose and Time::now intercepted; no ROS/TF/hardware validation. Finite numeric inputs only.',
        'c_sha256':hashlib.sha256((ROOT/'navigation_runtime/reverse/recovered/charge_core.c').read_bytes()).hexdigest()}
(ROOT/'analysis/chassis_stack/charge_core_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
