#!/usr/bin/env python3
"""Loopback-only LDCP peer: verify SDK checksum framing and lifecycle command order."""
import binascii,json,socket,threading,subprocess,sys
server=socket.socket();server.bind(('127.0.0.1',0));server.listen(1);server.settimeout(10)
calls=[];errors=[]
def run():
 try:
  conn,_=server.accept();conn.settimeout(5)
  with conn,conn.makefile('rb') as f:
   while True:
    line=f.readline(65537)
    if not line:break
    if len(line)>65536:raise ValueError('line bound')
    header,body=line.split(b',0:,',1);length,payload=body.split(b':',1);n=int(length)
    assert len(payload)==n+3 and payload[n:]==b',\r\n'
    assert header.startswith(b'15:checksum=0x') and int(header.split(b'0x')[1],16)==binascii.crc_hqx(payload[:n],65535)
    req=json.loads(payload[:n]);method=req['method'];calls.append(method)
    entry=req.get('params',{}).get('entry')
    values={'identity.model':'LT-R2','transport.oob.enabled':True,'transport.oob.targetAddress':'127.0.0.1','transport.oob.targetPort':2105}
    if method in ('device/queryInfo','settings/get'):result=values[entry]
    elif method in ('scan/startMeasurement','scan/startStreaming','scan/stopStreaming','scan/stopMeasurement'):result=True
    else:raise ValueError('unexpected '+method)
    data=json.dumps({'jsonrpc':'2.0','id':req['id'],'result':result},separators=(',',':')).encode()
    response=('15:checksum=0x%04X,0:,%d:'%(binascii.crc_hqx(data,65535),len(data))).encode()+data+b',\r\n'
    conn.sendall(response)
 except Exception as e:errors.append(repr(e))
t=threading.Thread(target=run);t.start()
r=subprocess.run([sys.argv[1],str(server.getsockname()[1])],capture_output=True,text=True,timeout=20);t.join(6);server.close()
expected=['device/queryInfo']+['settings/get']*3+['scan/startMeasurement','scan/startStreaming','scan/stopStreaming','scan/stopMeasurement']*2+['scan/stopStreaming','scan/stopMeasurement']
print(json.dumps({'exit':r.returncode,'stdout':r.stdout,'stderr':r.stderr,'calls':calls,'errors':errors,'expected_order':calls==expected},indent=2))
assert r.returncode==0 and not errors and calls==expected and not t.is_alive()
