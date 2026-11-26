const canvas = document.querySelector('#map');
const context = canvas.getContext('2d');
let current = null;
let savedMap = null;

function draw() {
  canvas.width = Math.round(canvas.clientWidth);
  canvas.height = Math.round(canvas.clientHeight);
  context.fillStyle = '#40515d';
  context.fillRect(0, 0, canvas.width, canvas.height);
  const map = savedMap;
  if (!map) return;
  const scale = Math.min(canvas.width / map.width, canvas.height / map.height) * 0.96;
  const offsetX = (canvas.width - map.width * scale) / 2;
  const offsetY = (canvas.height + map.height * scale) / 2;
  const point = (x, y) => {
    const dx = x - map.origin.x, dy = y - map.origin.y;
    const angle = map.origin.yaw;
    return [offsetX + (Math.cos(angle) * dx + Math.sin(angle) * dy) / map.resolution * scale,
      offsetY - (-Math.sin(angle) * dx + Math.cos(angle) * dy) / map.resolution * scale];
  };
  for (let row = 0; row < map.height; row++) {
    for (let column = 0; column < map.width; column++) {
      const value = map.data[row * map.width + column];
      context.fillStyle = value < 0 ? '#6a777e' : value >= 50 ? '#101820' : '#e0e8e8';
      context.fillRect(offsetX + column * scale, offsetY - (row + 1) * scale,
        Math.ceil(scale), Math.ceil(scale));
    }
  }
  context.fillStyle = '#00bada';
  for (const beam of current?.scan?.points ?? []) {
    const pixel = point(...beam);
    context.fillRect(pixel[0] - 1, pixel[1] - 1, 2, 2);
  }
  if (current?.amcl) {
    context.strokeStyle = '#ffcf70';
    context.beginPath();
    context.arc(...point(current.amcl.x, current.amcl.y), 7, 0, 2 * Math.PI);
    context.stroke();
  }
  if (current?.robot_tf) {
    const robot = current.robot_tf;
    context.save();
    context.translate(...point(robot.x, robot.y));
    context.rotate(-(robot.yaw - map.origin.yaw));
    context.fillStyle = '#ff5968';
    context.beginPath();
    context.moveTo(14, 0); context.lineTo(-9, -8); context.lineTo(-9, 8);
    context.closePath(); context.fill(); context.restore();
  }
}

const formatPose = pose => `x=${pose.x.toFixed(3)} m，y=${pose.y.toFixed(3)} m，yaw=${(pose.yaw * 180 / Math.PI).toFixed(1)}°`;

async function poll() {
  try {
    const response = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(1800)});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    current = await response.json();
    if (current.map) savedMap = current.map;
    document.querySelector('#status').textContent = current.tracking_valid ?
      '当前同帧定位核验通过 · 只读显示，未授权运动' : current.verification?.phase === 'waiting_pair' ?
      '等待新后验核验 · 静止等待不等于丢定位' : current.verification?.phase === 'pending_convergence' ?
      '定位尚待收敛 · 未授权运动' : current.verification?.phase === 'rejected' ?
      '当前定位核验未通过 · 未授权运动' : '定位数据未就绪；不显示未配对或过期位置';
    document.querySelector('#robot').textContent = current.robot_tf ? formatPose(current.robot_tf) : '无新鲜 TF 位姿';
    document.querySelector('#amcl').textContent = current.amcl ?
      `${formatPose(current.amcl)}；航向标准差 ${current.amcl.yaw_std_deg.toFixed(2)}°；观测年龄 ${current.amcl.observation_age_s.toFixed(1)} s；源时间 ${(current.amcl.stamp_ns / 1e9).toFixed(6)}` : '等待当前地图/进程的同帧 AMCL 核验';
    document.querySelector('#detail').textContent = JSON.stringify({errors: current.errors, chassis: current.chassis,
      robot_tf: current.robot_tf, amcl: current.amcl, verification: current.verification,
      tracking_valid: current.tracking_valid, motion_authorized: current.motion_authorized,
      scan_pairing: current.scan_pairing}, null, 2);
    document.querySelector('#verification').textContent = current.verification ?
      current.verification.reason : '核验数据未就绪；不代表 AMCL 已丢定位';
  } catch (error) {
    current = null;
    document.querySelector('#status').textContent = `连接中断：${error.message}；实时位置已清除`;
    document.querySelector('#robot').textContent = '无当前位姿';
    document.querySelector('#amcl').textContent = '无当前观测';
    document.querySelector('#verification').textContent = '核验连接中断';
    document.querySelector('#detail').textContent = '';
  }
  draw();
  setTimeout(poll, 500);
}
window.addEventListener('resize', draw);
poll();
