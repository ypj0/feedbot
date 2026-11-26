"""Original LTME OOB verifier vs recovered C; no sockets/device use."""
import binascii,ctypes as C,hashlib,json,random,struct,sys
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_X86,UC_MODE_64
from unicorn.x86_const import *
root=Path(__file__).resolve().parents[2];p=root/'analysis/chassis_stack/binaries/ltme_node'
sha='5c558970dcb6eb81244e713c4c1836b15803363f9828a7ea356afbaf11b9df1e';assert hashlib.sha256(p.read_bytes()).hexdigest()==sha
u=Uc(UC_ARCH_X86,UC_MODE_64);u.mem_map(0,0x400000);u.mem_map(0x500000,0x40000)
with p.open('rb') as f:
 for seg in ELFFile(f).iter_segments():
  if seg['p_type']=='PT_LOAD':u.mem_write(seg['p_vaddr'],seg.data())
lib=C.CDLL(sys.argv[1]);lib.recovered_lidar_verify.argtypes=[C.c_void_p,C.c_int];lib.recovered_lidar_verify.restype=C.c_int
r=random.Random(41);count=0
for n in list(range(40))+[r.randint(16,4096) for _ in range(500)]:
 for mode in range(4):
  b=bytearray(r.randbytes(max(n,16)))
  if mode:
   b[:2]=b'\xff\xff';b[12:14]=b'\0\0';crc=binascii.crc_hqx(bytes(b[:n]),0xffff)
   b[12:14]=struct.pack('<H',crc if mode!=2 else crc^1)
   if mode==3:b[0]=0
  u.mem_write(0x500000,bytes(b));u.mem_write(0x520000,struct.pack('<Q',0x530000))
  for reg,val in [(UC_X86_REG_RSP,0x520000),(UC_X86_REG_RDI,0),(UC_X86_REG_RSI,0x500000),(UC_X86_REG_EDX,n),(UC_X86_REG_EFLAGS,2)]:u.reg_write(reg,val)
  u.emu_start(0x4dfb0,0x530000,count=200000);assert u.reg_read(UC_X86_REG_RIP)==0x530000
  cb=C.create_string_buffer(bytes(b),len(b));got=lib.recovered_lidar_verify(cb,n)
  assert got==(u.reg_read(UC_X86_REG_RAX)&255),(n,mode)
  assert cb.raw==bytes(b)==u.mem_read(0x500000,len(b)),(n,mode,'buffer restore')
  if n>=16:assert got==(mode==1)
  count+=1
report={'passed':True,'cases':count,'binary_sha256':sha,'c_sha256':hashlib.sha256((root/'navigation_runtime/reverse/recovered/lidar_packet.c').read_bytes()).hexdigest(),'scope':'Full OOB verify function, nonnegative lengths 0..4096; return byte and exact buffer restoration. CRC crosscheck via binascii.crc_hqx. No payload decoder/network/hardware validation.'}
(root/'analysis/chassis_stack/lidar_packet_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
