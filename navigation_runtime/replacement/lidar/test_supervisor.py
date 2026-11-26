#!/usr/bin/env python3
import tempfile,pathlib,json,unittest
from supervisor import Supervisor
from factory_owner import FactoryOwner
class Fake:
 def __init__(self):self.factory=True;self.native=False;self.awake=True;self.ready=False;self.fail=None;self.events=[]
 def step(self,s):
  self.events.append(s)
  if self.fail==s:raise RuntimeError(s)
 def inhibit(self):self.step('inhibit')
 def factory_off(self):self.step('factory_off');self.factory=False
 def wait_free(self):self.step('wait_free');assert not self.factory and not self.native
 def factory_on(self):self.step('factory_on');assert not self.native;self.factory=True
 def factory_healthy(self):self.step('factory_healthy');assert self.factory
 def native_start(self,token):self.step('native_start');assert not self.factory;self.native=True
 def native_stop(self):self.step('native_stop');self.native=False
 def transition(self,name):self.step(name);assert self.native;self.ready=name=='activate'
 def desired_awake(self):return self.awake
 def native_healthy(self,awake):self.step('native_healthy');assert self.native and self.ready==awake
class Tests(unittest.TestCase):
 def test_handoff_sleep_restore(self):
  with tempfile.TemporaryDirectory() as d:
   h=Fake();s=Supervisor(d,h);s.start();self.assertTrue(h.native);h.awake=False;s.monitor();self.assertFalse(h.ready);h.awake=True;s.monitor();self.assertTrue(h.ready);s.restore();self.assertTrue(h.factory);self.assertFalse(h.native);self.assertFalse((pathlib.Path(d)/'permit').exists());s.close()
 def test_preflight_does_not_mutate(self):
  with tempfile.TemporaryDirectory() as d:
   h=Fake();h.fail='inhibit';s=Supervisor(d,h)
   with self.assertRaises(RuntimeError):s.start()
   self.assertEqual(h.events,['inhibit']);self.assertFalse(s.needs_restore);s.close()
 def test_failure_rollback(self):
  for fault in ('native_start','configure','activate','native_healthy'):
   with self.subTest(fault=fault),tempfile.TemporaryDirectory() as d:
    h=Fake();h.fail=fault;s=Supervisor(d,h)
    with self.assertRaises(RuntimeError):s.start()
    self.assertTrue(h.factory);self.assertFalse(h.native);self.assertFalse((pathlib.Path(d)/'permit').exists());s.close()
 def test_failed_native_stop_never_starts_factory(self):
  with tempfile.TemporaryDirectory() as d:
   h=Fake();s=Supervisor(d,h);s.start();h.fail='native_stop'
   with self.assertRaises(RuntimeError):s.restore()
   self.assertFalse(h.factory);self.assertTrue(h.native);self.assertEqual(json.loads((pathlib.Path(d)/'transaction.json').read_text())['state'],'rollback_failed');s.close()
 def test_crash_recovery(self):
  with tempfile.TemporaryDirectory() as d:
   h=Fake();s=Supervisor(d,h);s.start();s.close();s=Supervisor(d,h);s.start();self.assertLess(h.events.index('native_stop'),len(h.events)-1);s.restore();s.close()
 def test_exclusive_transaction(self):
  with tempfile.TemporaryDirectory() as d:
   s=Supervisor(d,Fake())
   with self.assertRaises(BlockingIOError):Supervisor(d,Fake())
   s.close()
 def test_factory_watchdog_and_restart(self):
  class Manager:
   scan_launch_flag=True
   def vendor_loading_scan(self):self.scan_launch_flag=True
   def vendor_shutdown_scan(self):self.scan_launch_flag=False
  with tempfile.TemporaryDirectory() as d:
   m=Manager();o=FactoryOwner(m,d,False);o.request('factory_off');o.loading();o.shutdown();self.assertFalse(m.scan_launch_flag)
   o=FactoryOwner(m,d,False);o.loading();self.assertFalse(m.scan_launch_flag);o.request('factory_on');self.assertTrue(m.scan_launch_flag)
if __name__=='__main__':unittest.main()
