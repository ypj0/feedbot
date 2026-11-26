#!/usr/bin/env python3
import pathlib,subprocess,tempfile,json,hashlib
root=pathlib.Path(__file__).resolve().parents[3]
s="""python3 - 2>&1 <<'PY'
import pathlib,json,subprocess
p=pathlib.Path('/opt/feedbot-nav2/lidar');session=sorted(p.glob('live.*'),key=lambda x:x.stat().st_mtime)
session=[x for x in session if x.is_dir()][-1]
files={str(f.relative_to(p)):f.read_text() for f in list(session.iterdir())+[p/'live.log',p/'live.exit',p/'build.log',p/'build.exit'] if f.is_file()}
print(json.dumps({'files':files,'running_containers':subprocess.check_output(['/opt/feedbot-nav2/docker/docker','-H','unix:///run/feedbot-nav2-docker.sock','ps','--format','{{.Names}}']).decode()}))
PY
"""
with tempfile.NamedTemporaryFile('w') as f:
 f.write(s);f.flush();r=subprocess.run(['python3',str(root/'navigation_runtime/replacement/tools/onboard_admin.py'),f.name],text=True,capture_output=True,check=True)
rows=[x for x in r.stdout.splitlines() if x.startswith('{')]
if not rows:raise RuntimeError(r.stdout)
d=json.loads(rows[-1]);dest=root/'analysis/body_navigation_20261107/lidar_native/live';dest.mkdir(exist_ok=True)
for name,value in d['files'].items():
 p=dest/name;p.parent.mkdir(exist_ok=True);p.write_text(value)
(dest/'result.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps({k:v for k,v in d['files'].items() if k!='build.log'},indent=2));print('running:',d['running_containers'])
