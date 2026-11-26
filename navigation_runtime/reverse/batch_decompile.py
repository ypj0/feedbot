"""Offline batch export. Does not communicate with the chassis or deploy code."""
import argparse,concurrent.futures,hashlib,json,os,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--ghidra',required=True);p.add_argument('--java',required=True);p.add_argument('--workers',type=int,default=2);args=p.parse_args()
root=Path(__file__).resolve().parents[2];out=root/'analysis/decompiled';out.mkdir(exist_ok=True)
projects=Path('/tmp/feedbot_ghidra_projects');projects.mkdir(exist_ok=True)
items=[('move_base','analysis/chassis_stack/binaries/libmove_base.so','core'),('dwa','analysis/chassis_stack/binaries/libdwa_local_planner.so','core'),('base','analysis/chassis_stack/binaries/libbase_local_planner.so','core'),('global','analysis/chassis_stack/binaries/libglobal_planner.so','core'),('driver','analysis/reverse_artifacts/robot_driver','core'),('driverlib','analysis/reverse_artifacts/librobot_driver_lib.so','all'),('lidar','analysis/chassis_stack/binaries/ltme_node','all'),('mcu','analysis/reverse_artifacts/sub_image.bin','all')]
def run(item):
 name,rel,mode=item;b=root/rel;log=out/(name+'.log');start=time.monotonic()
 cmd=[str(Path(args.ghidra)/'support/analyzeHeadless'),str(projects),name,'-import',str(b),'-overwrite','-analysisTimeoutPerFile','180','-max-cpu','1','-scriptPath',str(root/'navigation_runtime/reverse/ghidra')]
 if name=='mcu':cmd+=['-processor','ARM:LE:32:Cortex','-loader','BinaryLoader','-loader-baseAddr','0x08005000','-preScript','SeedThumb.java']
 cmd+=['-postScript','ExportCore.java',str(out),mode]
 env=dict(os.environ,JAVA_HOME=args.java,GHIDRA_HEADLESS_MAXMEM='1G')
 with log.open('w') as f:
  try:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=1800);code=r.returncode
  except subprocess.TimeoutExpired:code=-1
 cov=out/b.name/'coverage.txt'
 result={'module':name,'binary':rel,'sha256':hashlib.sha256(b.read_bytes()).hexdigest(),'exit_code':code,'seconds':round(time.monotonic()-start,1),'coverage':cov.read_text() if cov.exists() else None,'log':str(log.relative_to(root))}
 print(json.dumps(result),flush=True);return result
with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as ex:results=list(ex.map(run,items))
(out/'batch_manifest.json').write_text(json.dumps(results,indent=2)+'\n')
