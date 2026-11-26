#!/usr/bin/env python3
import pathlib,base64,tempfile,subprocess
p=pathlib.Path(__file__).resolve().parent
files={x.name:base64.b64encode(x.read_bytes()).decode() for x in p.iterdir() if x.is_file()}
s="import pathlib,base64,hashlib\np=pathlib.Path('/opt/feedbot-nav2/lidar');p.mkdir(exist_ok=True)\nfiles="+repr(files)+"\nfor name,data in files.items():\n b=base64.b64decode(data)\n if not (p/name).exists() or (p/name).read_bytes()!=b:(p/name).write_bytes(b)\n assert hashlib.sha256((p/name).read_bytes()).digest()==hashlib.sha256(b).digest()\nprint('source upload verified')\n"
with tempfile.NamedTemporaryFile('w') as f:
 f.write("python3 - 2>&1 <<'PY'\n"+s+"PY\n");f.flush();subprocess.run(['python3',str(p.parent/'tools/onboard_admin.py'),f.name],check=True)
