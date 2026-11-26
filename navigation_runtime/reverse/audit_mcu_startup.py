"""Execute firmware startup RAM initialization and SysTick config offline."""
import hashlib,json,struct
from pathlib import Path
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS
from unicorn.arm_const import *
root=Path(__file__).resolve().parents[2];p=root/'analysis/reverse_artifacts/sub_image.bin';b=p.read_bytes();sha=hashlib.sha256(b).hexdigest();assert sha=='dbc09a32d4c548b5ddd6c9c60bf622bbb0a9120dddd1f78fed2ed873aec9d9c1'
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.mem_map(0x8000000,0x20000);u.mem_write(0x8005000,b);u.mem_map(0x20000000,0x10000);u.mem_map(0xe000e000,0x1000)
u.mem_write(0x20000000,b'\xa5'*0x3000);u.reg_write(UC_ARM_REG_SP,0x2000f000)
u.emu_start(0x80059b1,0x8005138,count=30000);assert u.reg_read(UC_ARM_REG_PC)==0x8005138
assert u.mem_read(0x20000000,0x114)==b[0xe658-0x5000:0xe76c-0x5000]
assert u.mem_read(0x20000114,0x2304)==bytes(0x2304)
tasks=[]
for i in range(8):
 due,pad,remaining,period,pad2,fn=struct.unpack('<BBHHHI',u.mem_read(0x20000000+i*12,12))
 tasks.append(dict(index=i,due=due,initial_ticks=remaining,period_ticks=period,function=hex(fn)))
clock_pointer=struct.unpack_from('<I',b,0x80a4-0x5000)[0];clocks=[]
for hz in [8000000,72000000,168000000]:
 u.mem_write(clock_pointer,struct.pack('<I',hz));u.reg_write(UC_ARM_REG_SP,0x2000f000);u.reg_write(UC_ARM_REG_LR,0x8018001);u.reg_write(UC_ARM_REG_R0,0)
 u.emu_start(0x800806d,0x8018000,count=3000);assert u.reg_read(UC_ARM_REG_PC)==0x8018000
 ctrl,load,current=struct.unpack('<III',u.mem_read(0xe000e010,12));assert (ctrl,load,current)==(7,hz//1000-1,0)
 clocks.append(dict(assumed_core_hz=hz,control=ctrl,reload=load))
report={'passed':True,'binary_sha256':sha,'ram_copy':{'source':'0x0800e658','destination':'0x20000000','bytes':276},'zero_region':{'destination':'0x20000114','bytes':8964},'tasks':tasks,'systick_config':clocks,'scope':'Original startup copy/zero routines executed; original SysTick setup with injected core-clock variable. Establishes configured nominal 1ms ticks, not measured interrupt latency/clock accuracy, scheduler dispatch deadlines or physical stop time.'}
(root/'analysis/reverse_artifacts/startup_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
