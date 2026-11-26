"""Offline all-function export for non-runtime libraries confirmed loaded on chassis."""
import concurrent.futures,json,os,subprocess,time
from pathlib import Path
root=Path(__file__).resolve().parents[2];out=root/'analysis/decompiled_full'
items=json.loads((root/'analysis/loaded_artifacts/manifest.json').read_text())
env=dict(os.environ,JAVA_HOME='/Applications/CLion.app/Contents/jbr/Contents/Home',GHIDRA_HEADLESS_MAXMEM='1500M')
def run(item):
 binary=root/'analysis/loaded_artifacts'/item['path'].lstrip('/');name='loaded_'+binary.name.replace('.','_')
 cmd=['/tmp/feedbot_tools/ghidra_12.2_PUBLIC/support/analyzeHeadless','/tmp/feedbot_ghidra_projects',name,'-import',str(binary),'-overwrite','-analysisTimeoutPerFile','600','-max-cpu','1','-scriptPath',str(root/'navigation_runtime/reverse/ghidra'),'-postScript','ExportCore.java',str(out),'all']
 start=time.monotonic()
 with (out/(name+'.log')).open('w') as f:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=3600)
 result={**item,'project':name,'exit_code':r.returncode,'seconds':round(time.monotonic()-start,1)};print(json.dumps(result),flush=True);return result
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:results=list(ex.map(run,items))
(out/'loaded_export_manifest.json').write_text(json.dumps(results,indent=2)+'\n')
if any(r['exit_code'] for r in results):raise SystemExit(1)
