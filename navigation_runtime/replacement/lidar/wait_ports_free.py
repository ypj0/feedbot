#!/usr/bin/env python3
import socket,time,subprocess
until=time.monotonic()+8
while time.monotonic()<until:
 s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
 try:
  s.bind(('0.0.0.0',2105))
  text=subprocess.check_output(['ss','-Hnt'],universal_newlines=True)
  if not any('192.168.10.160:2105' in line and line.split()[0] in ('ESTAB','SYN-SENT','SYN-RECV') for line in text.splitlines()):break
 except OSError:pass
 finally:s.close()
 time.sleep(.1)
else:raise SystemExit('lidar TCP/UDP still owned; refusing second driver')
