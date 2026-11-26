#!/usr/bin/env python3
import pathlib,json,math,sys,statistics
p=pathlib.Path(sys.argv[1]);native=[json.loads(x) for x in (p/'native.jsonl').read_text().splitlines()];factory=[json.loads(x) for x in (p/'factory.jsonl').read_text().splitlines()]
def errors(a,b):
 if len(a)!=len(b):return (float('inf'),len(a)+len(b))
 maximum=0;bad=0
 for x,y in zip(a,b):
  if x is None:x=float('inf')
  if not math.isfinite(x) or not math.isfinite(y):
   if x!=y:bad+=1
  else:maximum=max(maximum,abs(x-y))
 return maximum,bad
used=set();matches=[];unmatched=[]
for n in native:
 candidates=[]
 for i,f in enumerate(factory):
  if i in used or abs(f['received']-n['received'])>.15:continue
  re,rb=errors(n['ranges'],f['ranges']);ie,ib=errors(n['intensities'],f['intensities'])
  candidates.append((rb+ib,re,ie,abs(f['received']-n['received']),i))
 if not candidates:unmatched.append(n['frame']);continue
 score=min(candidates);i=score[-1];f=factory[i]
 if score[0] or score[1]>1e-5 or score[2]>0:unmatched.append(n['frame']);continue
 used.add(i);matches.append({'frame':n['frame'],'seq':f['seq'],'range_max_error_m':score[1],'intensity_max_error':score[2],'stamp_native_minus_factory_ms':(n['stamp']-f['stamp'])*1000,'delivery_factory_minus_raw_ms':(f['received']-n['received'])*1000,'period_native_minus_factory_ms':(n['scan_time']-f['scan_time'])*1000,'angle_min_native_minus_factory':n['angle_min']-f['angle_min'],'angle_max_native_minus_factory':n['angle_max']-f['angle_max']})
def summary(key):
 v=sorted(x[key] for x in matches);return {'min':v[0],'p50':statistics.median(v),'max':v[-1]}
r={'native_frames':len(native),'factory_frames':len(factory),'matched':len(matches),'unmatched_native':unmatched,'unmatched_factory_seq':[f['seq'] for i,f in enumerate(factory) if i not in used],'beam_comparisons':sum(len(n['ranges']) for n in native if n['frame'] not in unmatched),'range_max_error_m':max((x['range_max_error_m'] for x in matches),default=None),'intensity_max_error':max((x['intensity_max_error'] for x in matches),default=None),'statistics':{k:summary(k) for k in ('stamp_native_minus_factory_ms','delivery_factory_minus_raw_ms','period_native_minus_factory_ms','angle_min_native_minus_factory','angle_max_native_minus_factory')} if matches else {},'hardware_accuracy_verified':False,'time_sync_verified':False}
(p/'comparison.json').write_text(json.dumps(r,indent=2)+'\n');(p/'matches.json').write_text(json.dumps(matches,indent=2)+'\n');print(json.dumps(r,indent=2))
