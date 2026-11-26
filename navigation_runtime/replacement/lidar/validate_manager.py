#!/usr/bin/env python3
"""Validate prepared manager with target Python2 compiler without installing/importing it."""
import pathlib,base64,subprocess,tempfile,hashlib,json
root=pathlib.Path(__file__).resolve().parent
# Send only exact small unified transformation spec; remote input hash must match local original.
s="""import hashlib,json
p='/opt/ros/melodic/lib/spirit_main/mapserver_manager.py'
s=open(p,'rb').read();assert hashlib.sha256(s).hexdigest()=='1a00b171baa1f75d8fc68296f3ca08f94afe7c68e45931616edd6fdafbf65884'
s=s.replace('    def loading_scan(self):','    def vendor_loading_scan(self):',1).replace('    def shutdown_scan(self):','    def vendor_shutdown_scan(self):',1)
s=s.replace('        self.loading_scan()','        from feedbot_factory_owner import FactoryOwner\\n        self.lidar_owner = FactoryOwner(self)\\n        self.loading_scan()',1)
s=s.replace('    def vendor_loading_scan(self):','    def loading_scan(self):\\n        return self.lidar_owner.loading()\\n\\n    def shutdown_scan(self):\\n        return self.lidar_owner.shutdown()\\n\\n    def vendor_loading_scan(self):',1)
w='if self.scan_status >= 20 and self.init_status == 1 and self.startScan == True and self.startDebug == False:'
s=s.replace(w,w[:-1]+\" and self.lidar_owner.mode() == 'factory':\")
compile(s,'prepared_manager.py','exec')
compile(open('/opt/feedbot-nav2/lidar/factory_owner.py','rb').read(),'factory_owner.py','exec')
print(json.dumps({'python2_compile':True,'prepared_sha256':hashlib.sha256(s).hexdigest()}))
"""
with tempfile.NamedTemporaryFile('w') as f:
 f.write("python - 2>&1 <<'PY'\n"+s+"PY\n");f.flush();r=subprocess.run(['python3',str(root.parent/'tools/onboard_admin.py'),f.name],capture_output=True,text=True,check=True)
d=json.loads(next(x for x in r.stdout.splitlines() if x.startswith('{')));assert d['prepared_sha256']==hashlib.sha256((root/'prepared/mapserver_manager.py').read_bytes()).hexdigest();print(json.dumps(d))
p=root.parents[2]/'analysis/body_navigation_20261107/lidar_native/control';(p/'prepared_manager_validation.json').write_text(json.dumps(d,indent=2)+'\n')
