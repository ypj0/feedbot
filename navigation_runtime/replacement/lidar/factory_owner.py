# Python 2/3 compatible. Installed in the factory manager ONLY by an explicit migration.
from __future__ import print_function
import os,json,socket,threading
ROOT='/run/feedbot-lidar'
class FactoryOwner(object):
 def __init__(self,manager,root=ROOT,start_server=True):
  self.manager=manager;self.root=root;self.lock=threading.RLock();self.desired_awake=True
  if not os.path.isdir(root):os.makedirs(root,0o700)
  if start_server:
   import rospy
   from std_msgs.msg import Int16
   self.sub=rospy.Subscriber('/laser_switch',Int16,self.intent,queue_size=4)
   self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);path=root+'/factory.sock'
   if os.path.exists(path):
    probe=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    try:probe.connect(path)
    except socket.error:os.unlink(path)
    else:raise RuntimeError('factory owner already running')
    finally:probe.close()
   self.socket.bind(path);os.chmod(path,0o600);self.socket.listen(2)
   t=threading.Thread(target=self.serve);t.daemon=True;t.start()
 def intent(self,msg):
  with self.lock:self.desired_awake=(msg.data==1)
 def mode(self):
  try:
   with open(self.root+'/mode') as f:value=f.read().strip()
  except IOError:return 'factory'
  if value not in ('native','factory'):raise RuntimeError('invalid ownership mode')
  return value
 def persist(self,value):
  p=self.root+'/mode'
  with open(p+'.tmp','w') as f:f.write(value+'\n');f.flush();os.fsync(f.fileno())
  os.rename(p+'.tmp',p)
 def loading(self):
  with self.lock:
   if self.mode()=='factory':self.manager.vendor_loading_scan()
 def shutdown(self):
  with self.lock:
   if self.mode()=='factory':self.manager.vendor_shutdown_scan()
 def request(self,op):
  with self.lock:
   if op=='factory_off':
    self.persist('native');self.manager.vendor_shutdown_scan()
    if self.manager.scan_launch_flag:raise RuntimeError('factory shutdown unconfirmed')
   elif op=='factory_on':
    self.persist('factory');self.manager.vendor_loading_scan()
    if not self.manager.scan_launch_flag:raise RuntimeError('factory start unconfirmed')
   elif op!='status':raise RuntimeError('unknown owner operation')
   return {'mode':self.mode(),'factory_launch':bool(self.manager.scan_launch_flag),'desired_awake':self.desired_awake}
 def serve(self):
  while True:
   c,_=self.socket.accept()
   try:
    c.settimeout(3);b=b''
    while b'\n' not in b and len(b)<1024:
     v=c.recv(1024-len(b))
     if not v:break
     b+=v
    req=json.loads(b.decode());reply={'ok':True,'result':self.request(req['op'])}
   except Exception as e:reply={'ok':False,'error':str(e)}
   try:c.sendall((json.dumps(reply)+'\n').encode())
   except socket.error:pass
   c.close()
