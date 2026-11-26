#!/usr/bin/env python3
"""Stage/build offline first; explicit --run starts bounded stationary hardware test."""
import base64,hashlib,pathlib,subprocess,sys,tempfile
ROOT=pathlib.Path(__file__).resolve().parent
def remote(code):
    with tempfile.NamedTemporaryFile('w') as f:
        f.write("python3 - 2>&1 <<'PYREMOTE'\n"+code+'\nPYREMOTE\n');f.flush()
        result=subprocess.run(['python3',str(ROOT.parent/'tools/onboard_admin.py'),f.name],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True)
        print(result.stdout)
        if result.returncode or 'REMOTE_OK' not in result.stdout:raise RuntimeError('remote command did not confirm completion')
if '--run' not in sys.argv:
    names=['protocol.hpp','frame_stream.hpp','stationary_probe.cpp','probe_test.py','stationary_trial.py']
    files={name:base64.b64encode((ROOT/name).read_bytes()).decode() for name in names}
    hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in names}
    remote('''import pathlib,base64,hashlib,subprocess
p=pathlib.Path('/opt/feedbot-nav2/chassis');p.mkdir(exist_ok=True)
files='''+repr(files)+'''
hashes='''+repr(hashes)+'''
for name,data in files.items():
 (p/name).write_bytes(base64.b64decode(data))
 assert hashlib.sha256((p/name).read_bytes()).hexdigest()==hashes[name]
subprocess.check_call(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-pthread',str(p/'stationary_probe.cpp'),'-o',str(p/'stationary_probe')])
r=subprocess.run(['python3',str(p/'probe_test.py'),str(p/'stationary_probe')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=20)
(p/'offline_tests.log').write_text(r.stdout);print(r.stdout);assert r.returncode==0
print('REMOTE_OK')''')
else:
    remote('''import pathlib,sys,subprocess
sys.path.insert(0,'/opt/feedbot-nav2/readiness');from maintenance_guard import current
print(current(True))
p=pathlib.Path('/opt/feedbot-nav2/chassis')
assert (p/'stationary_probe').exists()
for unit in ('feedbot-chassis-stationary-trial','feedbot-chassis-trial-restore'):
 r=subprocess.run(['systemctl','is-active',unit],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
 assert r.stdout.strip() not in (b'active',b'activating'), 'existing trial active'
subprocess.run(['systemctl','stop','feedbot-chassis-trial-restore.timer'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
subprocess.run(['systemctl','reset-failed','feedbot-chassis-stationary-trial','feedbot-chassis-trial-restore'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
for name in ('restored.json','trial.json','probe.log'):
 if (p/name).exists(): (p/name).rename(p/(name+'.previous'))
subprocess.check_call(['systemd-run','--unit=feedbot-chassis-trial-restore','--on-active=200s','/usr/bin/python3',str(p/'stationary_trial.py'),'restore'])
subprocess.check_call(['systemd-run','--unit=feedbot-chassis-stationary-trial','--property=RuntimeMaxSec=180','/bin/bash','-c','exec python3 /opt/feedbot-nav2/chassis/stationary_trial.py > /opt/feedbot-nav2/chassis/trial.log 2>&1'])
print('REMOTE_OK stationary trial launched')''')
