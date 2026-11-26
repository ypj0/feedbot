"""Snapshot source/evidence hashes after successful offline validation."""
import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parents[2]
paths=[]
for pattern in ['navigation_runtime/reverse/recovered/*','navigation_runtime/reverse/verify_*.py','navigation_runtime/reverse/audit_mcu_startup.py','analysis/chassis_stack/*verification.json','analysis/reverse_artifacts/*verification.json','analysis/chassis_stack/recovery_validation_full.log']:
 paths += [p for p in root.glob(pattern) if p.is_file()]
records=[dict(path=str(p.relative_to(root)),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),bytes=p.stat().st_size) for p in sorted(set(paths))]
report={'purpose':'Offline recovery source and evidence snapshot; not whole-system equivalence','numeric_differential_cases':35790,'separate_startup_audit':True,'files':records}
(root/'analysis/recovery_delivery_manifest.json').write_text(json.dumps(report,indent=2)+'\n');print('Manifest:',len(records),'files')
