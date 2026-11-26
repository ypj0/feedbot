#!/usr/bin/env python3
"""Stage pinned local vendor SDK source; no hardware commands, no network downloads."""
import pathlib,tarfile,io,base64,hashlib,tempfile,subprocess
root=pathlib.Path(__file__).resolve().parents[3];sdk=root/'analysis/upstream/ldcp_sdk';buf=io.BytesIO()
with tarfile.open(fileobj=buf,mode='w:gz') as tar:
 for name in ('CMakeLists.txt','LICENSE','src','include','third_party/Asio/asio-1.18.0/include','third_party/Asio/asio-1.18.0/LICENSE_1_0.txt','third_party/RapidJSON/rapidjson-1.1.0/include','third_party/RapidJSON/rapidjson-1.1.0/license.txt'):tar.add(sdk/name,arcname='ldcp_sdk/'+name)
b=buf.getvalue();digest=hashlib.sha256(b).hexdigest();remote='/opt/feedbot-nav2/lidar/sdk-'+digest+'.tgz'
for offset in range(0,len(b),32768):
 chunk=base64.b64encode(b[offset:offset+32768]).decode()
 script="import base64\nwith open("+repr(remote)+","+repr('wb' if offset==0 else 'ab')+") as f:f.write(base64.b64decode("+repr(chunk)+"))\n"
 if offset+32768>=len(b):script+="import hashlib,tarfile\nassert hashlib.sha256(open("+repr(remote)+",'rb').read()).hexdigest()=="+repr(digest)+"\nwith tarfile.open("+repr(remote)+") as t:t.extractall('/opt/feedbot-nav2/lidar/vendor')\nprint('SDK SHA256 verified: '+"+repr(digest)+")\n"
 with tempfile.NamedTemporaryFile('w') as f:
  f.write("python3 - 2>&1 <<'PY'\n"+script+"PY\n");f.flush();r=subprocess.run(['python3',str(root/'navigation_runtime/replacement/tools/onboard_admin.py'),f.name],capture_output=True,text=True,check=True)
  if 'Traceback' in r.stdout:raise RuntimeError(r.stdout)
print('staged',len(b),digest)
