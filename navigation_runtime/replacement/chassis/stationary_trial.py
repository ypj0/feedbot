#!/usr/bin/env python3
"""Bounded E-stop maintenance only. Runs outside spirit's cgroup; never arms motion."""
import hashlib,json,pathlib,subprocess,sys,time
ROOT=pathlib.Path('/opt/feedbot-nav2/chassis')
sys.path.insert(0,'/opt/feedbot-nav2/readiness')
from maintenance_guard import current
MANAGER=pathlib.Path('/opt/ros/melodic/lib/spirit_main/mapserver_manager.py')
SHA='1a00b171baa1f75d8fc68296f3ca08f94afe7c68e45931616edd6fdafbf65884'
def save(name,value):
    p=ROOT/name;t=ROOT/(name+'.tmp');t.write_text(json.dumps(value,indent=2)+'\n');t.replace(p)
def restore():
    subprocess.check_call(['systemctl','start','spirit'],timeout=110)
    # ROS1 subscribers do not automatically re-register with a replacement
    # master. Recreate the read-only observer for this factory generation.
    subprocess.check_call(['systemctl','restart','feedbot-maintenance-observer'],timeout=15)
    end=time.monotonic()+40
    while time.monotonic()<end:
        try:
            state=current(True)
            assert hashlib.sha256(MANAGER.read_bytes()).hexdigest()==SHA
            save('restored.json',{'factory_restored':True,'state':state})
            return
        except Exception:time.sleep(.25)
    raise RuntimeError('factory fresh stationary feedback not restored')
def run():
    report={'scope':'stationary serial maintenance; no motion API','passed':False}
    try:
        report['before']=current(True)
        assert hashlib.sha256(MANAGER.read_bytes()).hexdigest()==SHA
        assert pathlib.Path('/dev/rikibase').resolve()==pathlib.Path('/dev/ttyUSB1')
        report['binary_sha256']=hashlib.sha256((ROOT/'stationary_probe').read_bytes()).hexdigest()
        subprocess.check_call(['systemctl','stop','spirit'],timeout=110)
        state=subprocess.check_output(['systemctl','show','spirit','--property=ActiveState'],universal_newlines=True).strip()
        assert state=='ActiveState=inactive',state
        # The physical E-stop remains pressed throughout. Native feedback checks it
        # on every response; no software routine here releases it or locks wheels.
        result=subprocess.run([str(ROOT/'stationary_probe'),'--stationary-maintenance','/dev/rikibase'],
                              stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=15)
        (ROOT/'probe.log').write_text(result.stdout)
        report['probe_exit']=result.returncode
        report['probe_output']=result.stdout
        if result.returncode:raise RuntimeError('native stationary probe failed')
        records=[json.loads(line) for line in result.stdout.splitlines()]
        final=records[-1]
        assert final['event']=='complete' and final['rx']>=100 and final['motion_commands']==0
        report['passed']=True
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        save('trial.json',report)
        restore()
if __name__=='__main__':
    if len(sys.argv)==2 and sys.argv[1]=='restore':restore()
    else:run()
