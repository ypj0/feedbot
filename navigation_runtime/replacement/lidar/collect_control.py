#!/usr/bin/env python3
import pathlib,subprocess,json,tempfile,hashlib
root=pathlib.Path(__file__).resolve().parents[3]
s="""python3 - 2>&1 <<'PY'
import pathlib,json,hashlib,subprocess
p=pathlib.Path('/opt/feedbot-nav2/lidar')
files={}
for name in ('control_build.log','control_build.exit'):
 f=p/name
 if f.exists():
  b=f.read_bytes();files[name]={'text':b.decode(),'sha256':hashlib.sha256(b).hexdigest()}
print(json.dumps({'files':files,'factory':subprocess.check_output(['systemctl','show','spirit.service','-p','MainPID','-p','ActiveState']).decode(),'running_containers':subprocess.check_output(['/opt/feedbot-nav2/docker/docker','-H','unix:///run/feedbot-nav2-docker.sock','ps','--format','{{.Names}}']).decode()}))
PY
"""
with tempfile.NamedTemporaryFile('w') as f:
 f.write(s);f.flush();r=subprocess.run(['python3',str(root/'navigation_runtime/replacement/tools/onboard_admin.py'),f.name],capture_output=True,text=True,check=True)
d=json.loads(next(x for x in r.stdout.splitlines() if x.startswith('{')));p=root/'analysis/body_navigation_20261107/lidar_native/control';p.mkdir(exist_ok=True)
for n,item in d['files'].items():
 b=item['text'].encode();assert hashlib.sha256(b).hexdigest()==item['sha256'];(p/n).write_bytes(b)
(p/'result.json').write_text(json.dumps(d,indent=2)+'\n')
print(d['files']['control_build.log']['text'][-2000:]);print({k:v for k,v in d.items() if k!='files'});print('exit',d['files'].get('control_build.exit'))
