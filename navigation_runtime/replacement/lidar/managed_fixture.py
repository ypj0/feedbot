#!/usr/bin/env python3
"""Disconnected integration: official ROS2 lifecycle calls + SDK TCP + recorded UDP."""
import fcntl
import socket,threading,subprocess,pathlib,sys,time,json,struct,binascii,tempfile,os
import rclpy
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
source=pathlib.Path(sys.argv[1]);binary=sys.argv[2]
raw=source.read_bytes();pos=0;packets=[]
while pos<len(raw):
 _,n=struct.unpack_from('<dI',raw,pos);pos+=12;p=raw[pos:pos+n];pos+=n
 if len(p)>42 and p[23]==17:
  o=14+(p[14]&15)*4;length=struct.unpack_from('!H',p,o+4)[0];packets.append(p[o+8:o+length])
server=socket.socket();server.bind(('127.0.0.1',0));server.listen(3);server.settimeout(.2)
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',0))
probe=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);probe.bind(('127.0.0.1',0));udp_port=probe.getsockname()[1];probe.close()
stop=threading.Event();state={'measure':False,'stream':False,'suppress':False};calls=[];errors=[]
def tcp():
 while not stop.is_set():
  try:c,_=server.accept()
  except socket.timeout:continue
  try:
   with c,c.makefile('rb') as f:
    c.settimeout(4)
    while not stop.is_set():
     line=f.readline(65537)
     if not line:break
     h,b=line.split(b',0:,',1);ns,payload=b.split(b':',1);n=int(ns);assert len(payload)==n+3 and payload[n:]==b',\r\n';assert int(h.split(b'0x')[1],16)==binascii.crc_hqx(payload[:n],65535)
     q=json.loads(payload[:n]);method=q['method'];calls.append(method)
     vals={'identity.model':'LT-R2-B','transport.oob.enabled':True,'transport.oob.targetAddress':'127.0.0.1','transport.oob.targetPort':udp_port}
     if method in ('device/queryInfo','settings/get'):v=vals[q['params']['entry']]
     else:
      mapping={'scan/startMeasurement':('measure',True),'scan/stopMeasurement':('measure',False),'scan/startStreaming':('stream',True),'scan/stopStreaming':('stream',False)}
      key,v=mapping[method];state[key]=v;v=True
     b=json.dumps({'jsonrpc':'2.0','id':q['id'],'result':v},separators=(',',':')).encode();c.sendall(('15:checksum=0x%04X,0:,%d:'%(binascii.crc_hqx(b,65535),len(b))).encode()+b+b',\r\n')
  except Exception as e:
   if not stop.is_set():errors.append(repr(e))
def udp():
 i=0
 while not stop.wait(.00825):
  if state['measure'] and state['stream'] and not state['suppress']:sock.sendto(packets[i%len(packets)],('127.0.0.1',udp_port));i+=1
threads=[threading.Thread(target=t,daemon=True) for t in (tcp,udp)]
for t in threads:t.start()
rclpy.init();node=rclpy.create_node('lifecycle_fixture');client=node.create_client(ChangeState,'/ltr2_driver/change_state');seen=[];ready=[]
subscriptions=[node.create_subscription(LaserScan,'/scan',lambda m:seen.append(len(m.ranges)),qos_profile_sensor_data),node.create_subscription(Bool,'/lidar_ready',lambda m:ready.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
def spin(seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
def change(id,expect=True):
 req=ChangeState.Request();req.transition.id=id;future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=5);assert future.done() and future.result().success==expect,(id,future.result())
with tempfile.TemporaryDirectory() as tmp:
 root=pathlib.Path(tmp);log=open(root/'node.log','w');proc=subprocess.Popen([binary,'--ros-args','-p','device_address:=127.0.0.1','-p','target_address:=127.0.0.1','-p','control_port:='+str(server.getsockname()[1]),'-p','data_port:='+str(udp_port),'-p','owner_directory:='+tmp,'-p','owner_token:=fixture'],stdout=log,stderr=subprocess.STDOUT)
 try:
  assert client.wait_for_service(timeout_sec=10)
  change(Transition.TRANSITION_CONFIGURE,False);assert not calls
  (root/'permit').write_text('fixture\n')
  with (root/'hardware.lock').open('a') as held:
   fcntl.flock(held,fcntl.LOCK_EX|fcntl.LOCK_NB);(root/'status.json').write_text('sentinel');change(Transition.TRANSITION_CONFIGURE,False);assert (root/'status.json').read_text()=='sentinel';assert not calls
  busy=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);busy.bind(('0.0.0.0',udp_port));change(Transition.TRANSITION_CONFIGURE,False);assert not calls;busy.close()
  change(Transition.TRANSITION_CONFIGURE);change(Transition.TRANSITION_ACTIVATE);spin(1)
  assert len(seen)>5 and set(seen)=={1075} and ready[-1]
  change(Transition.TRANSITION_DEACTIVATE);spin(.2);assert not ready[-1] and not state['measure'] and not state['stream']
  change(Transition.TRANSITION_ACTIVATE);spin(.3);assert ready[-1]
  state['suppress']=True;spin(.7);assert not ready[-1] and not json.loads((root/'status.json').read_text())['ready']
  # Fault teardown via lifecycle cleanup path; readiness must remain false.
  change(Transition.TRANSITION_DEACTIVATE,False);spin(.1)
  # on_error returns SUCCESS -> unconfigured. Reconfigure establishes a new session.
  state['suppress']=False;change(Transition.TRANSITION_CONFIGURE);change(Transition.TRANSITION_ACTIVATE);spin(.3);assert ready[-1]
  (root/'permit').unlink();spin(.3);assert not ready[-1]
  print(json.dumps({'passed':True,'scan_messages':len(seen),'tests':['missing permit','hardware lock contention','UDP port contention','configure/activate','real UDP frame readiness','sleep/wake','scan loss','fault cleanup','reconfigure/reactivate','permit revocation'],'errors':errors,'hardware_commands':0},indent=2));assert not errors
 finally:
  proc.terminate()
  try:proc.wait(timeout=4)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
  log.close();print((root/'node.log').read_text()[-3000:]);stop.set();server.close();sock.close()
node.destroy_node();rclpy.shutdown()
