from pathlib import Path
import subprocess
import sys
import tempfile


root = Path(__file__).resolve().parent
mock = """
window.fixtureErrors=[];window.fixtureRequests=[];window.fixtureOffline=false;
window.onerror=message=>window.fixtureErrors.push(String(message));
window.fixtureData={data_ready:true,tracking_valid:true,motion_authorized:false,errors:{},
 map:{width:2,height:2,resolution:1,origin:{x:0,y:0,yaw:0},data:[0,100,-1,0]},
 scan:{points:[[1,1]]},robot_tf:{x:1,y:1,yaw:0},
 amcl:{x:0.9,y:1,yaw:0,yaw_std_deg:19,observation_age_s:2,stamp_ns:7000000123}};
window.fetch=async(url,options)=>{
 window.fixtureRequests.push({url,options});
 if(window.fixtureOffline)throw new Error('fixture offline');
 return {ok:true,json:async()=>window.fixtureData};
};
"""
checks = """
setTimeout(async()=>{
 const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
 try {
  if(!document.querySelector('#robot').textContent.includes('1.000'))throw new Error('initial pose absent');
  if(!document.querySelector('#amcl').textContent.includes('19.00'))throw new Error('paired AMCL absent');
  if(!document.querySelector('#status').textContent.includes('核验通过'))throw new Error('tracking status absent');
  window.fixtureData.robot_tf={x:1.5,y:1,yaw:0.2};
  window.fixtureData.tracking_valid=false;
  window.fixtureData.verification={phase:'waiting_pair',reason:'waiting'};
  await pause(550);
  if(!document.querySelector('#robot').textContent.includes('1.500'))throw new Error('live position not updated');
  if(!document.querySelector('#amcl').textContent.includes('0.900'))throw new Error('AMCL mixed with TF');
  if(!document.querySelector('#status').textContent.includes('静止等待不等于丢定位'))throw new Error('waiting misclassified');
  window.fixtureOffline=true;
  await pause(550);
  if(current!==null||!document.querySelector('#robot').textContent.includes('无当前'))throw new Error('stale position retained');
  if(!savedMap)throw new Error('saved map lost on disconnect');
  if(window.fixtureRequests.some(item=>item.url!=='/api/state'||item.options.method==='POST'))throw new Error('unexpected write request');
  if(window.fixtureErrors.length)throw new Error(window.fixtureErrors.join(';'));
  document.body.dataset.fixture='PASS';
 } catch(error) {document.body.dataset.fixture='FAIL';document.body.dataset.error=error.message;}
},100);
"""
html = (root / "index.html").read_text().replace('<script src="/app.js"></script>',
    '<script>' + mock + '</script><script>' + (root / "app.js").read_text() + '</script><script>' + checks + '</script>')
with tempfile.TemporaryDirectory(prefix="localization-browser-") as directory:
    page = Path(directory) / "fixture.html"
    page.write_text(html)
    result = subprocess.run([sys.argv[1], "--headless", "--disable-gpu", "--no-first-run",
                             "--virtual-time-budget=1600", "--dump-dom", page.as_uri()],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=30)
assert result.returncode == 0 and 'data-fixture="PASS"' in result.stdout, result.stdout + result.stderr
print("PASS: live position update, same-frame AMCL, offline clear, no POST or motion request")
