#!/usr/bin/env python
"""Finite, passive validation only: raw Ethernet receive + ROS1 scan subscription."""
from __future__ import print_function
import socket,struct,time,json,threading,os,sys
import rospy
from sensor_msgs.msg import LaserScan
root=sys.argv[1]
os.mkdir(root)
rospy.init_node('ltr2_passive_pair',anonymous=True,disable_signals=True)
scans=open(root+'/factory.jsonl','w');raw=open(root+'/raw.bin','wb')
lock=threading.Lock();counts={'packets':0,'scans':0};done=threading.Event()
def scan(m):
 row={'received':time.time(),'seq':m.header.seq,'stamp':m.header.stamp.to_sec(),'frame':m.header.frame_id}
 for k in ('angle_min','angle_max','angle_increment','time_increment','scan_time','range_min','range_max','ranges','intensities'):row[k]=getattr(m,k)
 with lock:scans.write(json.dumps(row,separators=(',',':'))+'\n');counts['scans']+=1
s=socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(3));s.bind(('enp2s0',0));s.settimeout(.2)
def receive():
 while not done.is_set():
  try:b=s.recv(65535)
  except socket.timeout:continue
  if len(b)<34 or b[12:14]!=b'\x08\x00' or b[26:30]!=socket.inet_aton('192.168.10.160'):continue
  raw.write(struct.pack('<dI',time.time(),len(b))+b);counts['packets']+=1
thread=threading.Thread(target=receive);thread.start()
sub=rospy.Subscriber('/scan',LaserScan,scan,queue_size=32)
try:time.sleep(12)
finally:
 sub.unregister();done.set();thread.join();s.close();raw.close()
 with lock:scans.close()
 with open(root+'/capture.json','w') as f:json.dump(counts,f)
print(json.dumps(counts))
