#!/usr/bin/env python3
"""Differential test against original x86 conversion instructions, no devices."""
import ctypes as C,hashlib,json,pathlib,random,struct,subprocess,tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_X86,UC_MODE_64
from unicorn.x86_const import *
here=pathlib.Path(__file__).resolve().parent;root=here.parents[2]
binary=root/'analysis/reverse_artifacts/robot_driver'
sha=hashlib.sha256(binary.read_bytes()).hexdigest()
assert sha=='03d96108d91b8661c24310e2d13f558b9c5865bd2aef268f9c0664e37cc23640'
u=Uc(UC_ARCH_X86,UC_MODE_64);u.mem_map(0,0x400000);u.mem_map(0x500000,0x40000)
with binary.open('rb') as f:
    for segment in ELFFile(f).iter_segments():
        if segment['p_type']=='PT_LOAD':u.mem_write(segment['p_vaddr'],segment.data())
def reg(register,value):u.reg_write(register,value)
def put(address,value):u.mem_write(address,struct.pack('<d',value))
def xmm(register,value):reg(register,int.from_bytes(struct.pack('<d',value),'little'))
def double(address):return struct.unpack('<d',u.mem_read(address,8))[0]
with tempfile.TemporaryDirectory() as d:
    src=pathlib.Path(d)/'bridge.cpp';lib=pathlib.Path(d)/'bridge.so'
    src.write_text('''#include "odometry.hpp"
extern "C" void convert(int l,int r,int v,int w,double ls,double rs,double* out){
 feedbot_chassis::Feedback f{0,1,62,0,0,0,0,2,0,441,l,r,v,w};
 auto s=feedbot_chassis::wheel_sample(f,{ls,rs,.455});
 out[0]=s.left_m;out[1]=s.right_m;out[2]=s.v_mps;out[3]=s.w_radps;
}
''')
    subprocess.run(['clang++','-std=c++17','-O2','-ffp-contract=off','-shared','-fPIC','-I',str(here),str(src),'-o',str(lib)],check=True)
    shared=C.CDLL(str(lib));fn=shared.convert;fn.argtypes=[C.c_int]*4+[C.c_double]*2+[C.POINTER(C.c_double)]
    rng=random.Random(441);count=0
    for i in range(2000):
        l,r,v,w=[rng.randint(-32768,32767) for _ in range(4)]
        if i<5:l=r=v=w=[-32768,-1,0,1,32767][i]
        ls,rs=[rng.choice([1.,1.01,1.5,2.]) for _ in range(2)]
        reg(UC_X86_REG_RBX,0x500000);reg(UC_X86_REG_RBP,0x520000)
        reg(UC_X86_REG_EDX,l&255);reg(UC_X86_REG_ECX,r&255)
        reg(UC_X86_REG_EDI,0);reg(UC_X86_REG_ESI,0)
        u.mem_write(0x520000-0x115,bytes([(l>>8)&255]))
        u.mem_write(0x520000-0x113,bytes([(r>>8)&255]))
        put(0x500000+0x4f0,ls);put(0x500000+0x4e8,rs);xmm(UC_X86_REG_XMM1,1000.)
        u.emu_start(0x488c6,0x4895b,count=100)
        assert u.reg_read(UC_X86_REG_RIP)==0x4895b
        left=struct.unpack('<h',u.mem_read(0x520000-0x1f4,2))[0]/100000.
        right=struct.unpack('<h',u.mem_read(0x520000-0x1f6,2))[0]/100000.
        # Original publish_ODOM's twist conversion. ROS time call is outside
        # this slice; supply its duration stack slots, without invoking ROS.
        reg(UC_X86_REG_R13,v&65535);reg(UC_X86_REG_R12,w&65535)
        reg(UC_X86_REG_R14,0);reg(UC_X86_REG_R15,0)
        put(0x520000-0x408,1000.)
        u.mem_write(0x520000-0x3e8,struct.pack('<ii',0,20000000))
        u.emu_start(0x43803,0x4388f,count=100)
        assert u.reg_read(UC_X86_REG_RIP)==0x4388f
        expected=[left,right,double(0x520000-0x438),double(0x520000-0x410)]
        actual=(C.c_double*4)();fn(l,r,v,w,ls,rs,actual)
        assert list(actual)==expected,(i,list(actual),expected)
        count+=1
report={'passed':True,'cases':count,'original_sha256':sha,
        'candidate_sha256':hashlib.sha256((here/'odometry.hpp').read_bytes()).hexdigest(),
        'original_instruction_slices':['0x488c6..0x4895b','0x43803..0x4388f'],
        'scope':'left/right scale and truncation plus v441 twist sign/divisor; not full odometry, packet-loss behavior or physical calibration'}
out=root/'analysis/body_navigation_20261107/readiness/chassis_odometry';out.mkdir(parents=True,exist_ok=True)
(out/'feedback_units_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
