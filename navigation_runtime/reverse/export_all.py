"""Export all detected non-external/non-thunk functions from existing projects."""
import concurrent.futures,json,os,subprocess,time
from pathlib import Path
root=Path(__file__).resolve().parents[2]
projects=['nav','dwa','base','global','driver','driverlib','lidar','mcu']
out=root/'analysis/decompiled_full';out.mkdir(exist_ok=True)
env=dict(os.environ,JAVA_HOME='/Applications/CLion.app/Contents/jbr/Contents/Home',GHIDRA_HEADLESS_MAXMEM='1G')
def run(name):
    start=time.monotonic()
    cmd=['/tmp/feedbot_tools/ghidra_12.2_PUBLIC/support/analyzeHeadless','/tmp/feedbot_ghidra_projects',name,'-process','-noanalysis','-readOnly','-max-cpu','1','-scriptPath',str(root/'navigation_runtime/reverse/ghidra'),'-postScript','ExportCore.java',str(out),'all']
    with (out/(name+'.log')).open('w') as f:
        r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=1800)
    row={'project':name,'exit_code':r.returncode,'seconds':round(time.monotonic()-start,1)}
    print(json.dumps(row),flush=True);return row
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:results=list(ex.map(run,projects))
(out/'export_manifest.json').write_text(json.dumps(results,indent=2)+'\n')
if any(r['exit_code'] for r in results):raise SystemExit(1)
