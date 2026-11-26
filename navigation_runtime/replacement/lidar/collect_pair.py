#!/usr/bin/env python3
"""Collect latest bounded passive capture, validate SHA256, preserve evidence."""
import pathlib,subprocess,tempfile,json,base64,hashlib
root=pathlib.Path(__file__).resolve().parents[3]
s="""python3 - 2>&1 <<'PY'
import pathlib,json,base64,hashlib
p=sorted(x for x in pathlib.Path('/opt/feedbot-nav2/lidar').glob('pair_*') if x.is_dir())[-1]
assert (p/'capture.json').exists(), 'capture not finished: '+str(p)
out={}
for f in p.iterdir():
 b=f.read_bytes();out[f.name]={'data':base64.b64encode(b).decode(),'sha256':hashlib.sha256(b).hexdigest()}
print(json.dumps({'session':p.name,'files':out}))
PY
"""
with tempfile.NamedTemporaryFile('w') as f:
 f.write(s);f.flush();r=subprocess.run(['python3',str(root/'navigation_runtime/replacement/tools/onboard_admin.py'),f.name],capture_output=True,text=True,check=True)
rows=[x for x in r.stdout.splitlines() if x.startswith('{')]
if not rows:raise RuntimeError(r.stdout)
d=json.loads(rows[-1]);dest=root/'analysis/body_navigation_20261107/lidar_native'/d['session'];dest.mkdir(exist_ok=True)
for name,item in d['files'].items():
 b=base64.b64decode(item['data']);assert hashlib.sha256(b).hexdigest()==item['sha256'];(dest/name).write_bytes(b)
(dest/'manifest.json').write_text(json.dumps({k:v['sha256'] for k,v in d['files'].items()},indent=2)+'\n');print(dest)
