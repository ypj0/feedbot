#!/usr/bin/env python3
"""Generate complete reviewed factory-manager variant OFFLINE. Never install/restart."""
import pathlib,hashlib,sys,difflib
root=pathlib.Path(__file__).resolve().parents[3]
p=root/'analysis/chassis_stack/opt/ros/melodic/lib/spirit_main/mapserver_manager.py'
b=p.read_bytes();assert hashlib.sha256(b).hexdigest()=='1a00b171baa1f75d8fc68296f3ca08f94afe7c68e45931616edd6fdafbf65884'
s=b.decode('utf-8');old=s
s=s.replace('    def loading_scan(self):','    def vendor_loading_scan(self):',1).replace('    def shutdown_scan(self):','    def vendor_shutdown_scan(self):',1)
needle='        self.loading_scan()';assert s.count(needle)>=1
s=s.replace(needle,'        from feedbot_factory_owner import FactoryOwner\n        self.lidar_owner = FactoryOwner(self)\n'+needle,1)
needle='    def vendor_loading_scan(self):'
s=s.replace(needle,'    def loading_scan(self):\n        return self.lidar_owner.loading()\n\n    def shutdown_scan(self):\n        return self.lidar_owner.shutdown()\n\n'+needle,1)
watchdog='if self.scan_status >= 20 and self.init_status == 1 and self.startScan == True and self.startDebug == False:'
assert s.count(watchdog)==1
s=s.replace(watchdog,watchdog[:-1]+" and self.lidar_owner.mode() == 'factory':")
out=root/'navigation_runtime/replacement/lidar/prepared';out.mkdir(exist_ok=True)
(out/'mapserver_manager.py').write_text(s)
(out/'manager.diff').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile='factory_original',tofile='factory_with_owner')))
(out/'feedbot_factory_owner.py').write_bytes((pathlib.Path(__file__).parent/'factory_owner.py').read_bytes())
print(out)
