#!/usr/bin/env python3
"""Fingerprint-bound constants at verified constructor/call sites."""
import hashlib,json,struct
from pathlib import Path
from elftools.elf.elffile import ELFFile
root=Path(__file__).resolve().parents[2]
p=root/'analysis/chassis_stack/binaries/libmove_base.so'
sha=hashlib.sha256(p.read_bytes()).hexdigest()
assert sha=='971b0895544c43f5ece5fd5cb73ac0ae03d5895046c373ffd01e4a94209e837e'
with p.open('rb') as f:
    elf=ELFFile(f)
    def read(a):
        sec=next(s for s in elf.iter_sections() if s['sh_addr']<=a<s['sh_addr']+s['sh_size'])
        return sec.data()[a-sec['sh_addr']:]
    parameters=[]
    for offset,name_addr,default_addr in [(0x6f8,0x14976a,0x1501f0),(0x700,0x149779,0x1501f8),(0x740,0x149789,0x1501e0),(0x708,0x14979d,0x150200),(0x738,0x1497a9,0x1500e0),(0x150,0x149818,0x150130),(0x158,0x1497b8,0x150208),(0x720,0x1497d2,0x150190),(0x730,0x1497e2,0x150158),(0x728,0x1497f0,0x1501c8)]:
        parameters.append({'object_offset':hex(offset),'name':read(name_addr).split(b'\0')[0].decode(),'constructor_default':struct.unpack('<d',read(default_addr)[:8])[0]})
    constants={hex(a):struct.unpack('<d',read(a)[:8])[0] for a in [0x154398,0x150190,0x1543a8,0x1543c8,0x1543c0,0x150108,0x150188,0x1501c0,0x150240]}
report={'sha256':sha,'parameters':parameters,'constants':constants,'scope':'Constructor defaults, not observed current object values. Parameter names and offsets manually traced in constructor disassembly.'}
out=root/'analysis/chassis_stack/charge_constants.json';out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
