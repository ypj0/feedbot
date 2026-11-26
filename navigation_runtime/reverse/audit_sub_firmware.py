"""Reviewed-build evidence extraction. Static evidence only, no device access."""
import hashlib,json,struct
from pathlib import Path
from capstone import Cs,CS_ARCH_ARM,CS_MODE_THUMB,CS_MODE_MCLASS
from capstone.arm import ARM_OP_MEM,ARM_REG_PC
from inspect_hex import read_hex
root=Path(__file__).resolve().parents[2]
p=root/'analysis/reverse_artifacts/sub.hex'
expected='5baeec4873058a2e0b92c04a979b8acd5bae2062cf47194f06e909fc8e9304df'
assert hashlib.sha256(p.read_bytes()).hexdigest()==expected
memory,_,_=read_hex(p);base=min(memory);raw=bytes(memory[a] for a in range(base,max(memory)+1))
def word(a):return struct.unpack_from('<I',raw,a-base)[0]
md=Cs(CS_ARCH_ARM,CS_MODE_THUMB|CS_MODE_MCLASS);md.detail=True

def excerpt(lo,hi):
 out=[]
 for i in md.disasm(raw[lo-base:hi-base],lo):
  note=''
  if i.mnemonic.startswith('ldr') and len(i.operands)>1 and i.operands[1].type==ARM_OP_MEM and i.operands[1].mem.base==ARM_REG_PC:
   a=((i.address+4)&~3)+i.operands[1].mem.disp;note=' ; literal '+hex(a)+'='+hex(word(a))
  out.append(hex(i.address)+': '+i.mnemonic+' '+i.op_str+note)
 return out
assert raw[0x800d0fc-base:0x800d102-base]==b'V4.41\0'
assert word(0x800c390)==0x200000bc
assert word(0x800dcd8)==0x200000bc
rows=[]
for a in range(0x800e658,0x800e6b8,12):
 flag,pad,count,period,pad2,fn=struct.unpack_from('<BBHHHI',raw,a-base)
 rows.append(dict(flash=hex(a),flag=flag,initial_count=count,period_ticks=period,function=hex(fn)))
assert rows[1]['function']=='0x800c1f9' and rows[1]['period_ticks']==100
ranges={
 'systick_scheduler':(0x800b21c,0x800b232),
 'task_countdown':(0x800b7fc,0x800b85c),
 'main_task_dispatch':(0x800b7c0,0x800b7f6),
 'silence_counter_and_zero_call':(0x800c1f8,0x800c23e),
 'type_1_2_reset_counter':(0x800d8c2,0x800d90a),
 'type_2_target_decode':(0x800db6a,0x800db9a),
 'target_state_write':(0x800d26a,0x800d282),
 'version_assignment':(0x800d108,0x800d114),
 'zero_reply_builder':(0x800dee4,0x800df60),
}
report=dict(sha256=expected,static_only=True,firmware_identity_verified=False,
 version_string='V4.41',version_value=441,task_table=rows,
 conclusions=[
 'Received type 1 and type 2 branches reset uint16 counter at 0x200000bc.',
 'Task at 0x0800c1f8 increments counter; above 18 sets it to 15, clears flag and calls shared target routine with three zero arguments.',
 'Task table candidate period is 100 SysTick ticks; wall-time deadline and installed-image identity are not verified.',
 'Same target routine handles signed big-endian type-2 bytes 10..11 and 12..13.',
 'This is a main-loop scheduled timeout, not proof of an independent hardware watchdog or measured braking.'
 ],evidence={k:excerpt(*v) for k,v in ranges.items()})
out=root/'analysis/reverse_artifacts/sub_timeout_evidence.json';out.write_text(json.dumps(report,indent=2)+'\n')
print('Wrote',out,'with',len(ranges),'bounded address excerpts')
