"""Offline Intel HEX inventory. Never opens hardware or executes firmware."""
import hashlib,json,struct,sys
from pathlib import Path

def read_hex(path):
    memory={};base=0;eof=False;records=0;entry=None
    for n,line in enumerate(Path(path).read_text().splitlines(),1):
        if not line.strip():continue
        if eof or not line.startswith(':'):raise ValueError('invalid record at '+str(n))
        b=bytes.fromhex(line[1:]);records+=1
        if len(b)<5 or len(b)!=b[0]+5 or sum(b)%256:raise ValueError('checksum/length at '+str(n))
        count,offset,kind=b[0],int.from_bytes(b[1:3],'big'),b[3];data=b[4:-1]
        if kind==0:
            for i,v in enumerate(data):
                a=base+offset+i
                if a in memory and memory[a]!=v:raise ValueError('conflicting overlap')
                memory[a]=v
        elif kind==1:
            if count or offset:raise ValueError('bad EOF')
            eof=True
        elif kind in (2,4):
            if count!=2 or offset:raise ValueError('bad extended address')
            base=int.from_bytes(data,'big') << (4 if kind==2 else 16)
        elif kind==5:
            if count!=4 or offset:raise ValueError('bad entry')
            entry=int.from_bytes(data,'big')
        elif kind==3:
            if count!=4 or offset:raise ValueError('bad segment entry')
        else:raise ValueError('unknown record type')
    if not eof or not memory:raise ValueError('missing EOF/data')
    return memory,records,entry

def inspect(path,out):
    memory,records,entry=read_hex(path);lo=min(memory);hi=max(memory)+1
    if hi-lo>16*1024*1024:raise ValueError('unbounded sparse image')
    raw=bytes(memory.get(a,255) for a in range(lo,hi));Path(out).mkdir(parents=True,exist_ok=True)
    Path(out,'sub_image.bin').write_bytes(raw)
    vectors=list(struct.unpack('<16I',raw[:64]))
    r=dict(source=str(path),sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
           records=records,address_start=hex(lo),address_end=hex(hi),data_bytes=len(memory),
           gap_bytes=len(raw)-len(memory),entry=None if entry is None else hex(entry),
           initial_sp=hex(vectors[0]),reset_vector=hex(vectors[1]),vectors=[hex(x) for x in vectors],
           architecture_hint='Cortex-M Thumb candidate; not hardware identity proof',
           installed_firmware_match=False)
    from capstone import Cs,CS_ARCH_ARM,CS_MODE_THUMB,CS_MODE_MCLASS
    md=Cs(CS_ARCH_ARM,CS_MODE_THUMB|CS_MODE_MCLASS);md.skipdata=True
    lines=[]
    for i in md.disasm(raw,lo):
        lines.append('0x%08x: %-12s %s %s'%(i.address,i.bytes.hex(),i.mnemonic,i.op_str))
    Path(out,'sub_thumb_linear.txt').write_text('\n'.join(lines)+'\n')
    Path(out,'sub_inventory.json').write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps(r,indent=2))

if __name__=='__main__':inspect(sys.argv[1],sys.argv[2])
