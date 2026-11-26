#!/usr/bin/env python3
"""Verify recovery, cancel redundant rollback timer, retrieve hashed trial evidence."""
import hashlib,json,pathlib,subprocess,tempfile
here=pathlib.Path(__file__).resolve().parent
code='''import pathlib,json,hashlib,subprocess,sys
sys.path.insert(0,'/opt/feedbot-nav2/readiness');from maintenance_guard import current
p=pathlib.Path('/opt/feedbot-nav2/chassis');state=current(True)
assert json.loads((p/'restored.json').read_text())['factory_restored']
manager=pathlib.Path('/opt/ros/melodic/lib/spirit_main/mapserver_manager.py')
sha=hashlib.sha256(manager.read_bytes()).hexdigest()
assert sha=='1a00b171baa1f75d8fc68296f3ca08f94afe7c68e45931616edd6fdafbf65884'
factory=subprocess.check_output(['systemctl','show','spirit','--property=ActiveState','--property=MainPID'],universal_newlines=True)
assert 'ActiveState=active' in factory
r=subprocess.run(['systemctl','stop','feedbot-chassis-trial-restore.timer'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
assert r.returncode in (0,5)
final={'state':state,'factory':factory,'manager_sha256':sha,'rollback_timer_cancelled':True}
(p/'final_state.json').write_text(json.dumps(final,indent=2)+'\\n')
files={}
for name in ('trial.json','trial.log','probe.log','restored.json','final_state.json','offline_tests.log','stationary_trial.py'):
 b=(p/name).read_bytes();files[name]={'text':b.decode(),'sha256':hashlib.sha256(b).hexdigest()}
print(json.dumps({'files':files}))
'''
with tempfile.NamedTemporaryFile('w') as f:
    f.write("python3 - 2>&1 <<'PYREMOTE'\n"+code+'\nPYREMOTE\n');f.flush()
    r=subprocess.run(['python3',str(here.parent/'tools/onboard_admin.py'),f.name],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,check=True)
data=json.loads(next(line for line in r.stdout.splitlines() if line.startswith('{')))
out=here.parents[2]/'analysis/body_navigation_20261107/readiness/chassis_stationary'
out.mkdir(parents=True,exist_ok=True)
for name,row in data['files'].items():
    b=row['text'].encode();assert hashlib.sha256(b).hexdigest()==row['sha256'];(out/name).write_bytes(b)
(out/'manifest.json').write_text(json.dumps({name:row['sha256'] for name,row in data['files'].items()},indent=2)+'\n')
print(out)
print(data['files']['final_state.json']['text'])
