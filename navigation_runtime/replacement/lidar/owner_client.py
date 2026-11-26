#!/usr/bin/env python3
import socket,json,sys
s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.settimeout(8);s.connect('/run/feedbot-lidar/factory.sock');s.sendall((json.dumps({'op':sys.argv[1]})+'\n').encode());b=b''
while b'\n' not in b:
 x=s.recv(4096)
 if not x:raise RuntimeError('owner connection ended')
 b+=x
 if len(b)>16384:raise RuntimeError('response limit')
s.close();r=json.loads(b);print(json.dumps(r));sys.exit(0 if r['ok'] else 2)
