from pathlib import Path
import subprocess
import sys
import tempfile


root = Path(__file__).resolve().parents[1]
html = (root / "static/index.html").read_text().replace('<link rel="stylesheet" href="/style.css">', '<style>' + (root / "static/style.css").read_text() + '</style>')
mock = """
window.fixtureCommands=[];window.fixtureFailed=false;window.fixtureErrors=[];
window.fixtureSaves=[];window.confirm=()=>true;
window.fixtureSave={phase:'saved',result:{id:'fixture',width:1,height:1,resolution:1,files:{'feedbot.yaml':{url:'/api/maps/fixture/feedbot.yaml'},'feedbot.pgm':{url:'/api/maps/fixture/feedbot.pgm'}}}};
window.onerror=(message)=>{window.fixtureErrors.push(String(message));};
window.fixtureState={ready:true,sampled_at:10,control:{epoch:'x',sequence:0,action:'stop'},samples:{
  diagnostics:{age_s:.1,value:{level:0,values:{motor_free:'1',command_nonzero_count:'0'}}},
  joints:{age_s:.1,value:{left_wheel_joint:1,right_wheel_joint:2}},odom:{age_s:.1,value:{vx:.1,wz:.2}}}};
window.fixtureMapping={mapping_ready:true,transform_errors:{},map:{width:1,height:1,resolution:1,origin:{x:0,y:0,yaw:1},data:[0],age_s:2},scan:{points:[],age_s:.1},robot:{x:0,y:0,yaw:1,stamp_s:1,age_s:.1}};
window.fetch=async(url,options={})=>{
  if(window.fixtureFailed)throw new Error('fixture offline');
  if(url==='/api/command'){window.fixtureCommands.push(JSON.parse(options.body));return{ok:true,json:async()=>window.fixtureState};}
  if(url==='/api/save'){
    if(options.method==='POST'){window.fixtureSaves.push(JSON.parse(options.body));window.fixtureSave={phase:'saving'};}
    return{ok:true,json:async()=>window.fixtureSave};
  }
  if(url==='/api/state')return{ok:true,json:async()=>window.fixtureState};
  if(url==='/api/map')return{ok:true,json:async()=>window.fixtureMapping};
  throw new Error('unexpected URL '+url);
};
"""
checks = """
setTimeout(async()=>{
  const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
  try{
    if(window.fixtureErrors.length)throw new Error(window.fixtureErrors.join(';'));
    if(window.fixtureCommands.length)throw new Error('startup command sent');
    if(window.fixtureSaves.length)throw new Error('startup save sent');
    if(document.querySelectorAll('#downloads a').length!==2)throw new Error('downloads absent');
    document.querySelector('#saveMap').click();await pause(20);
    if(window.fixtureSaves.length!==1||!window.fixtureSaves[0].confirm_save)throw new Error('save request incorrect');
    if(!document.querySelector('#saveMap').disabled)throw new Error('saving button enabled');
    renderSave({phase:'failed',error:'fixture failure'});
    if(!document.querySelector('#saveState').textContent.includes('fixture failure'))throw new Error('save failure hidden');
    if(document.querySelectorAll('#downloads a').length)throw new Error('failed save shows success downloads');
    if(document.querySelector('[data-action]'))throw new Error('motion control remains');
    if(!document.querySelector('#status').textContent.includes('建图数据新鲜'))throw new Error('status not updated');
    if(!document.querySelector('#chassis').textContent.includes('1.00000 rad'))throw new Error('joint reading absent');
    if(!document.querySelector('#feedbackState').textContent.includes('不代表电机已解锁'))throw new Error('unlock boundary absent');
    if(document.querySelector('#map').getContext('2d').getImageData(20,20,1,1).data[3]===0)throw new Error('canvas not rendered');
    const cell=worldToCell(1,3,{origin:{x:1,y:1,yaw:Math.PI/2},resolution:.5});
    if(Math.abs(cell[0]-4)>1e-10||Math.abs(cell[1])>1e-10)throw new Error('rotated map geometry');
    document.querySelector('#follow').click();
    if(view.mode!=='follow')throw new Error('follow failed');
    document.querySelector('#fit').click();
    if(view.mode!=='fit')throw new Error('fit failed');
    canvas.dispatchEvent(new WheelEvent('wheel',{deltaY:-100,cancelable:true}));
    if(view.mode!=='manual')throw new Error('zoom failed');
    window.fixtureState.samples.joints.age_s=2;
    window.fixtureMapping.robot=null;window.fixtureMapping.scan=null;window.fixtureMapping.mapping_ready=false;
    window.fixtureMapping.transform_errors={map_to_scan:'fixture TF missing'};
    await pause(250);
    if(!document.querySelector('#streams').textContent.includes('过期'))throw new Error('stale feedback hidden');
    if(!document.querySelector('#pose').textContent.includes('—'))throw new Error('old pose retained');
    if(!document.querySelector('#warnings').textContent.includes('fixture TF missing'))throw new Error('TF error hidden');
    window.fixtureFailed=true;await pause(250);
    if(!document.querySelector('#status').textContent.includes('连接失败'))throw new Error('offline status absent');
    if(window.fixtureCommands.length)throw new Error('fixture sent command');
    if(window.fixtureSaves.length!==1)throw new Error('save repeated');
    if(window.fixtureErrors.length)throw new Error(window.fixtureErrors.join(';'));
    document.body.dataset.fixture='PASS';
  }catch(error){document.body.dataset.fixture='FAIL';document.body.dataset.error=error.message;}
},300);
"""
html = html.replace('<script src="/app.js"></script>', '<script>'+mock+'</script><script>'+(root / "static/app.js").read_text()+'</script><script>'+checks+'</script>')
with tempfile.TemporaryDirectory(prefix="mapping-browser-") as directory:
    page = Path(directory) / "fixture.html"
    page.write_text(html)
    result = subprocess.run([sys.argv[1], "--headless", "--disable-gpu", "--no-first-run",
                             "--virtual-time-budget=1200", "--dump-dom", page.as_uri()],
                            capture_output=True, text=True, timeout=30)
assert result.returncode == 0 and 'data-fixture="PASS"' in result.stdout, result.stdout + result.stderr
print("PASS: geometry, map controls, raw status, stale/TF/offline states; no JS error or command")
