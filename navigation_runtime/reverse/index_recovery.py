"""Build auditable index and upstream header diffs, offline only."""
import csv,difflib,hashlib,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];out=ROOT/('analysis/decompiled_full' if '--full' in sys.argv else 'analysis/decompiled')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
modules=[]
for idx in sorted(out.glob('*/index.tsv')):
    rows=list(csv.DictReader(idx.open(),delimiter='\t'))
    names=subprocess.run(['/opt/homebrew/opt/llvm/bin/llvm-cxxfilt'],input='\n'.join(r['name'] for r in rows)+'\n',text=True,capture_output=True,check=True).stdout.splitlines()
    assert len(rows)==len(names)
    cov=dict(line.split('=',1) for line in (idx.parent/'coverage.txt').read_text().splitlines())
    exported=[];failed=[];warn=0
    with (idx.parent/'readable.tsv').open('w') as f:
        f.write('original_address\tghidra_address\tdemangled_name\tfile\tsha256\twarnings\n')
        for r,n in zip(rows,names):
            if r['status']!='ok':
                failed.append(r);continue
            p=idx.parent/r['file'];data=p.read_text();w=data.count('WARNING:');warn+=bool(w)
            addr=int(r['address'],16)-(0 if idx.parent.name=='sub_image.bin' else 0x100000)
            h=sha(p);f.write(f"{addr:08x}\t{r['address']}\t{n}\t{r['file']}\t{h}\t{w}\n")
            exported.append({'file':r['file'],'sha256':h})
    assert len(exported)==int(cov['exported']) and len(failed)==int(cov['failed'])
    assert len(rows)==int(cov['selected'])
    modules.append({'module':idx.parent.name,**cov,'files_with_warnings':warn,'files':exported,'failed_functions':failed})
(out/'corpus_manifest.json').write_text(json.dumps(modules,indent=2)+'\n')
lines=['# 自动反编译索引','', '这些文件是自动伪代码，不是可直接编译的原源码。selected 为本次选择导出的函数（mode=all时包含全部已识别非外部、非跳板函数）；functions_detected 包含外部/跳板等，不能相除得出源码恢复率。', '', 'ELF Ghidra 加载基址为 0x100000；readable.tsv 已减去该偏移，可对照原始反汇编。MCU 保留 0x0800xxxx 地址。', '', '| 模块 | 导出函数 | 含警告的文件 | 符号索引 |','|---|---:|---:|---|']
for m in modules:lines.append(f"| {m['module']} | {m['exported']} | {m['files_with_warnings']} | [readable.tsv]({m['module']}/readable.tsv) |")
lines+=['',f"总计 {sum(int(m['exported']) for m in modules)} 个函数。另有 {sum(int(m['failed']) for m in modules)} 个导出失败（见清单）；工具导出成功不代表无反编译错误。警告、未识别函数边界、错误类型/浮点表达式均须人工核对；编译且通过原指令差分的模型另见 navigation_runtime/reverse/recovered。",'', '每模块 calls.tsv 为静态调用关系，不能覆盖所有虚调用/函数指针。corpus_manifest.json 保存各伪代码文件 SHA256。']
(out/'README.md').write_text('\n'.join(lines)+'\n')
up=ROOT/'analysis/upstream';records=[]
for name in ['fetch_open_auto_dock','ros_navigation','ldcp_sdk','ltme_node','robot_localization','cartographer','cartographer_ros','costmap_prohibition_layer']:
    p=up/name
    commit=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
    url=subprocess.check_output(['git','-C',str(p),'remote','get-url','origin'],text=True).strip()
    records.append({'name':name,'commit':commit,'url':url,'purpose':'reference source, NOT proof of vendor binary identity','licenses':[str(q.relative_to(p)) for q in p.rglob('*') if q.is_file() and q.name.lower().startswith(('license','copying'))]})
(up/'provenance.json').write_text(json.dumps(records,indent=2)+'\n')
installed=ROOT/'analysis/chassis_headers/opt/ros/melodic/include/move_base/fetch_auto_dock';diffs=up/'vendor_header_diffs';diffs.mkdir(exist_ok=True)
summary=[]
for p in sorted(installed.glob('*.h')):
    matches=list((up/'fetch_open_auto_dock').rglob(p.name))
    if len(matches)!=1:summary.append({'header':p.name,'match':'missing_or_ambiguous'});continue
    q=matches[0];diff=''.join(difflib.unified_diff(q.read_text().splitlines(True),p.read_text().splitlines(True),fromfile='upstream/'+str(q.relative_to(up/'fetch_open_auto_dock')),tofile='installed/'+p.name))
    (diffs/(p.name+'.diff')).write_text(diff)
    summary.append({'header':p.name,'identical':not diff,'installed_sha256':sha(p),'upstream_sha256':sha(q)})
(diffs/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({'functions':sum(int(m['exported']) for m in modules),'warnings':sum(m['files_with_warnings'] for m in modules),'upstream':records,'headers':summary},indent=2))
