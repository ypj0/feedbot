#!/usr/bin/env python3
import pathlib,json,subprocess,tempfile,hashlib
root=pathlib.Path(__file__).resolve().parents[3]
s="""python3 - 2>&1 <<'PY'
import pathlib,json,hashlib,subprocess
p=pathlib.Path('/opt/feedbot-nav2/lidar');out={}
for name in ('acceptance.log','acceptance.exit','managed_fixture.log','managed_fixture.exit'):
 f=p/name
 if f.exists():
  b=f.read_bytes();out[name]={'text':b.decode(),'sha256':hashlib.sha256(b).hexdigest()}
print(json.dumps({'files':out,'factory':subprocess.check_output(['systemctl','show','spirit.service','-p','MainPID','-p','ActiveState']).decode(),'containers':subprocess.check_output(['/opt/feedbot-nav2/docker/docker','-H','unix:///run/feedbot-nav2-docker.sock','ps','--format','{{.Names}}']).decode(),'factory_manager_sha256':hashlib.sha256(pathlib.Path('/opt/ros/melodic/lib/spirit_main/mapserver_manager.py').read_bytes()).hexdigest()}))
PY
"""
with tempfile.NamedTemporaryFile('w') as f:
 f.write(s);f.flush();r=subprocess.run(['python3',str(root/'navigation_runtime/replacement/tools/onboard_admin.py'),f.name],text=True,capture_output=True,check=True)
d=json.loads(next(x for x in r.stdout.splitlines() if x.startswith('{')));p=root/'analysis/body_navigation_20261107/lidar_native/acceptance';p.mkdir(exist_ok=True)
for name,row in d['files'].items():
 b=row['text'].encode();assert hashlib.sha256(b).hexdigest()==row['sha256'];(p/name).write_bytes(b)
(p/'result.json').write_text(json.dumps(d,indent=2)+'\n')
print(json.dumps({'exit':d['files'].get('acceptance.exit',{}).get('text'),'factory':d['factory'],'containers':d['containers'],'factory_manager_sha256':d['factory_manager_sha256']}))
