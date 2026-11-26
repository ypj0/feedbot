#!/usr/bin/env python3
"""Build/run isolated numeric tests on internal chassis; no device access."""
import pathlib,base64,json,subprocess,tempfile,hashlib
root=pathlib.Path(__file__).resolve().parent.parent
names=['chassis/protocol.hpp','chassis/odometry.hpp','chassis/odometry_test.cpp','imu/wit_decoder.hpp','imu/test.cpp']
files={n:base64.b64encode((root/n).read_bytes()).decode() for n in names}
hashes={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in names}
code='''import pathlib,base64,hashlib,subprocess,json
p=pathlib.Path('/opt/feedbot-nav2/native_units');p.mkdir(exist_ok=True)
files='''+repr(files)+'''\nhashes='''+repr(hashes)+'''
for name,data in files.items():
 target=p/name;target.parent.mkdir(exist_ok=True);target.write_bytes(base64.b64decode(data))
 assert hashlib.sha256(target.read_bytes()).hexdigest()==hashes[name]
results={}
for name,src in [('odometry','chassis/odometry_test.cpp'),('imu','imu/test.cpp')]:
 subprocess.check_call(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',str(p/src),'-o',str(p/name)+'_test'])
 results[name]=subprocess.check_output([str(p/name)+'_test'],universal_newlines=True)
print(json.dumps({'passed':True,'results':results,'source_sha256':hashes}))
print('REMOTE_OK')
'''
with tempfile.NamedTemporaryFile('w') as f:
    f.write("python3 - 2>&1 <<'PYREMOTE'\n"+code+'PYREMOTE\n');f.flush()
    result=subprocess.run(['python3',str(root/'tools/onboard_admin.py'),f.name],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True)
print(result.stdout)
if result.returncode or 'REMOTE_OK' not in result.stdout:raise SystemExit('remote validation failed')
data=json.loads(next(line for line in result.stdout.splitlines() if line.startswith('{')))
out=root.parents[1]/'analysis/body_navigation_20261107/readiness/chassis_odometry';out.mkdir(exist_ok=True)
(out/'onboard_units_tests.json').write_text(json.dumps(data,indent=2)+'\n')
