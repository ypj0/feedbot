#!/usr/bin/env python3
"""Transactional lidar owner supervisor. No motor control; explicit navigation inhibit hook required."""
import pathlib,json,os,fcntl,time,secrets,subprocess,signal,sys
class Supervisor:
 def __init__(self,root,hooks):
  self.root=pathlib.Path(root);self.root.mkdir(parents=True,exist_ok=True);self.h=hooks
  self.lock=open(self.root/'transaction.lock','a')
  try:fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
  except Exception:self.lock.close();raise
  self.awake=False;self.needs_restore=False
 def record(self,state,error=None):
  p=self.root/'transaction.json';t=p.with_suffix('.tmp')
  with t.open('w') as f:json.dump({'state':state,'error':error,'time':time.time()},f);f.flush();os.fsync(f.fileno())
  os.replace(t,p);fd=os.open(str(self.root),os.O_RDONLY);os.fsync(fd);os.close(fd)
 def revoke(self):
  try:(self.root/'permit').unlink()
  except FileNotFoundError:pass
 def restore(self):
  self.needs_restore=True;self.record('restoring');self.revoke()
  try:
   self.h.native_stop();self.h.wait_free();self.h.factory_on();self.h.factory_healthy();self.record('factory');self.needs_restore=False
  except Exception as e:self.record('rollback_failed',str(e));raise
 def start(self):
  # A previous interrupted transaction is recovered before acquiring new ownership.
  p=self.root/'transaction.json'
  if p.exists() and json.loads(p.read_text())['state']!='factory':self.restore()
  self.h.inhibit();self.needs_restore=True;self.record('prepared')
  try:
   self.record('stopping_factory');self.h.factory_off();self.h.wait_free()
   token=secrets.token_hex(24);tmp=self.root/'permit.tmp';tmp.write_text(token+'\n');os.chmod(tmp,0o600);os.replace(tmp,self.root/'permit')
   self.record('starting_native');self.h.native_start(token);self.h.transition('configure');self.awake=self.h.desired_awake()
   if self.awake:self.h.transition('activate')
   self.h.native_healthy(self.awake);self.record('native_active' if self.awake else 'native_sleeping')
  except Exception:
   self.restore();raise
 def monitor(self):
  awake=self.h.desired_awake()
  if awake!=self.awake:
   self.h.transition('activate' if awake else 'deactivate');self.awake=awake;self.record('native_active' if awake else 'native_sleeping')
  self.h.native_healthy(self.awake)
 def close(self):self.lock.close()
class Hooks:
 def __init__(self,cfg,root):self.c=cfg;self.root=pathlib.Path(root);self.started=0
 def run(self,key,*args):
  cmd=self.c.get(key)
  if not isinstance(cmd,list) or not cmd or not os.path.isabs(cmd[0]):raise RuntimeError('explicit absolute argv required: '+key)
  r=subprocess.run(cmd+list(args),universal_newlines=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=20)
  if r.returncode:raise RuntimeError(key+': '+r.stderr[-1000:]+r.stdout[-1000:])
  return r.stdout
 def inhibit(self):self.run('navigation_inhibit')
 def factory_off(self):self.run('factory_off')
 def factory_on(self):self.run('factory_on')
 def factory_healthy(self):self.run('factory_healthy')
 def wait_free(self):self.run('wait_free')
 def native_stop(self):self.run('native_stop')
 def native_start(self,token):self.started=time.monotonic();self.run('native_start',token)
 def transition(self,name):self.run('transition',name)
 def desired_awake(self):return bool(json.loads(self.run('owner_status'))['result']['desired_awake'])
 def native_healthy(self,awake):
  until=time.monotonic()+2
  while time.monotonic()<until:
   try:
    s=json.loads((self.root/'status.json').read_text());now=time.monotonic()
    if self.started<=s['heartbeat']<=now and now-s['heartbeat']<.5 and s.get('healthy',False) and s['ready']==awake:return
   except (OSError,ValueError,KeyError):pass
   time.sleep(.05)
  raise RuntimeError('native heartbeat/readiness lost')
def main():
 cfg=json.loads(pathlib.Path(sys.argv[1]).read_text());root=cfg['owner_directory'];h=Hooks(cfg,root);s=Supervisor(root,h);done=False
 def stopping(*_):
  nonlocal done;done=True
 signal.signal(signal.SIGTERM,stopping);signal.signal(signal.SIGINT,stopping)
 try:
  s.start()
  while not done:s.monitor();time.sleep(.1)
 finally:
  # If rollback already failed, retry once while retaining factory inhibition until ports free.
  try:
   if s.needs_restore:s.restore()
  finally:s.close()
if __name__=='__main__':main()
