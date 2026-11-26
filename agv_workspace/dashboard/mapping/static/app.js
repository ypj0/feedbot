const canvas = document.querySelector('#map');
const context = canvas.getContext('2d');
const view = {centerX: 0, centerY: 0, scale: 1, mode: 'fit'};
let mapping = null;
let trajectory = [];
let drag = null;
let lastPoseStamp = null;

function number(value, digits = 3) {
  return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '—';
}

function metrics(selector, rows) {
  const target = document.querySelector(selector);
  target.replaceChildren();
  for (const [label, value] of rows) {
    const term = document.createElement('dt');
    const description = document.createElement('dd');
    term.textContent = label;
    description.textContent = value ?? '—';
    target.append(term, description);
  }
}

function worldToCell(x, y, map) {
  const deltaX = x - map.origin.x, deltaY = y - map.origin.y;
  const cosine = Math.cos(map.origin.yaw), sine = Math.sin(map.origin.yaw);
  return [(cosine * deltaX + sine * deltaY) / map.resolution,
    (-sine * deltaX + cosine * deltaY) / map.resolution];
}

function screenPoint(x, y) {
  const cell = worldToCell(x, y, mapping.map);
  return [canvas.width / 2 + (cell[0] - view.centerX) * view.scale,
    canvas.height / 2 - (cell[1] - view.centerY) * view.scale];
}

function draw() {
  const bounds = canvas.getBoundingClientRect();
  canvas.width = Math.round(bounds.width);
  canvas.height = Math.round(bounds.height);
  context.fillStyle = '#243440';
  context.fillRect(0, 0, canvas.width, canvas.height);
  if (!mapping?.map) return;
  const map = mapping.map;
  if (view.mode === 'fit') {
    view.scale = Math.min(canvas.width / map.width, canvas.height / map.height) * 0.94;
    view.centerX = map.width / 2;
    view.centerY = map.height / 2;
  } else if (view.mode === 'follow' && mapping.robot) {
    [view.centerX, view.centerY] = worldToCell(mapping.robot.x, mapping.robot.y, map);
  }
  const offsetX = canvas.width / 2 - view.centerX * view.scale;
  const offsetY = canvas.height / 2 + view.centerY * view.scale;
  for (let row = 0; row < map.height; row++) {
    for (let column = 0; column < map.width; column++) {
      const value = map.data[row * map.width + column];
      context.fillStyle = value < 0 ? '#4b5c69' : value >= 50 ? '#101820' : '#e0e8e8';
      context.fillRect(offsetX + column * view.scale, offsetY - (row + 1) * view.scale,
        Math.ceil(view.scale), Math.ceil(view.scale));
    }
  }
  context.strokeStyle = '#a78bfa';
  context.lineWidth = 2;
  context.beginPath();
  trajectory.forEach((pose, index) => {
    if (pose === null) return;
    const point = screenPoint(pose.x, pose.y);
    if (index === 0 || trajectory[index - 1] === null) context.moveTo(...point);
    else context.lineTo(...point);
  });
  context.stroke();
  context.fillStyle = '#00bad5';
  for (const point of mapping.scan?.points ?? []) {
    const pixel = screenPoint(...point);
    context.fillRect(pixel[0] - 1, pixel[1] - 1, 2, 2);
  }
  if (mapping.robot) {
    const robot = mapping.robot, point = screenPoint(robot.x, robot.y);
    const yaw = robot.yaw - map.origin.yaw;
    context.save();
    context.translate(...point);
    context.rotate(-yaw);
    context.fillStyle = '#ff5968';
    context.beginPath();
    context.moveTo(14, 0); context.lineTo(-9, -8); context.lineTo(-5, 0); context.lineTo(-9, 8);
    context.closePath(); context.fill(); context.restore();
  }
}

function render(state, payload) {
  mapping = payload;
  const samples = state?.samples ?? {};
  const diagnostics = samples.diagnostics?.value;
  const raw = diagnostics?.values ?? {};
  const odom = samples.odom?.value;
  const joints = samples.joints?.value;
  const errors = payload?.transform_errors ?? {};
  const status = document.querySelector('#status');
  status.dataset.ok = String(Boolean(payload?.mapping_ready));
  status.textContent = payload ? '已连接 · ' + (payload.mapping_ready ? '建图数据新鲜' : '建图数据缺失或过期') : '连接中断';
  document.querySelector('#mappingState').textContent = payload?.mapping_ready ? '正在接收地图、激光与位姿' : '等待完整建图数据';
  document.querySelector('#feedbackState').textContent = !state ? '反馈不可用' :
    state.ready ? '底盘反馈新鲜；不代表电机已解锁' : '底盘未就绪：请核对数据年龄与原始状态';
  metrics('#chassis', [
    ['电机 motor_free（原始）', raw.motor_free], ['急停 / 充电（原始）', `${raw.emergency ?? '—'} / ${raw.charge ?? '—'}`],
    ['左右报警（原始）', `${raw.left_alarm ?? '—'} / ${raw.right_alarm ?? '—'}`], ['电量 power（原始）', raw.power],
    ['左轮累计位置', number(joints?.left_wheel_joint, 5) + ' rad'],
    ['右轮累计位置', number(joints?.right_wheel_joint, 5) + ' rad'],
    ['轮式线速度', number(odom?.vx, 4) + ' m/s'], ['轮式角速度', number(odom?.wz, 4) + ' rad/s'],
    ['反馈包 / 超时次数', `${raw.feedback_count ?? '—'} / ${raw.feedback_timeout_count ?? '—'}`],
    ['拒绝字节 / 字段异常', `${raw.rejected_bytes ?? '—'} / ${raw.runtime_field_error_count ?? '—'}`],
    ['非零指令累计次数', raw.command_nonzero_count]
  ]);
  const robot = payload?.robot;
  metrics('#pose', [['X / Y', `${number(robot?.x)} / ${number(robot?.y)} m`],
    ['朝向', number(robot ? robot.yaw * 180 / Math.PI : null, 1) + ' °']]);
  if (robot && robot.stamp_s !== lastPoseStamp) {
    trajectory.push({x: robot.x, y: robot.y});
    if (trajectory.length > 3000) trajectory.shift();
    lastPoseStamp = robot.stamp_s;
  } else if (!robot && trajectory.length && trajectory[trajectory.length - 1] !== null) {
    trajectory.push(null);
    lastPoseStamp = null;
  }
  const streams = document.querySelector('#streams');
  streams.replaceChildren();
  for (const [name, sample, limit] of [['地图', payload?.map, null], ['激光', payload?.scan, 0.5],
    ['地图位姿', robot, 0.5], ['轮关节', samples.joints, 0.3], ['轮里程计', samples.odom, 0.3], ['底盘诊断', samples.diagnostics, 0.6]]) {
    const row = document.createElement('tr');
    for (const value of [name, sample ? number(sample.age_s, 2) + ' s' : '—',
      !sample ? '缺失' : limit === null ? '最近地图' : sample.age_s <= limit ? '新鲜' : '过期']) {
      const cell = document.createElement('td'); cell.textContent = value; row.append(cell);
    }
    streams.append(row);
  }
  const map = payload?.map;
  document.querySelector('#mapInfo').textContent = map ? `${map.width} × ${map.height} · ${number(map.resolution, 2)} m/格` : '等待地图';
  document.querySelector('#warnings').textContent = Object.keys(errors).length ? JSON.stringify(errors, null, 2) :
    payload?.mapping_ready ? 'TF 当前无报错；保存结果见右侧，重载定位未验收。' : '数据不完整，不以旧激光或旧位姿替代当前数据。';
  document.querySelector('#detail').textContent = JSON.stringify({diagnostics, transform_errors: errors, control: state?.control}, null, 2);
  draw();
}

async function refresh() {
  try {
    const responses = await Promise.all([fetch('/api/state'), fetch('/api/map')]);
    for (let index = 0; index < responses.length; index++) {
      if (!responses[index].ok) throw new Error(`${index === 0 ? '/api/state' : '/api/map'} HTTP ${responses[index].status}`);
    }
    const [state, payload] = await Promise.all(responses.map(response => response.json()));
    render(state, payload);
  } catch (error) {
    render(null, null);
    document.querySelector('#status').textContent = '连接失败：' + error.message;
  }
  setTimeout(refresh, 200);
}

function setMode(mode) {
  view.mode = mode;
  document.querySelector('#follow').setAttribute('aria-pressed', String(mode === 'follow'));
  draw();
}
document.querySelector('#fit').addEventListener('click', () => setMode('fit'));
document.querySelector('#follow').addEventListener('click', () => setMode(view.mode === 'follow' ? 'manual' : 'follow'));
canvas.addEventListener('wheel', event => {
  event.preventDefault();
  view.scale *= Math.exp(-event.deltaY * 0.001);
  setMode('manual');
}, {passive: false});
canvas.addEventListener('pointerdown', event => {
  drag = {x: event.clientX, y: event.clientY};
  canvas.setPointerCapture(event.pointerId);
  setMode('manual');
});
canvas.addEventListener('pointermove', event => {
  if (!drag) return;
  view.centerX -= (event.clientX - drag.x) / view.scale;
  view.centerY += (event.clientY - drag.y) / view.scale;
  drag = {x: event.clientX, y: event.clientY};
  draw();
});
canvas.addEventListener('pointerup', () => { drag = null; });
canvas.addEventListener('pointercancel', () => { drag = null; });
window.addEventListener('resize', draw);
refresh();

let savePosting = false;
function renderSave(value) {
  const message = document.querySelector('#saveState');
  const downloads = document.querySelector('#downloads');
  downloads.replaceChildren();
  document.querySelector('#saveMap').disabled = savePosting || value.phase === 'saving';
  if (value.phase === 'saved') {
    const result = value.result;
    message.textContent = `已保存：${result.id} · ${result.width}×${result.height} · ${result.resolution} m/格`;
    for (const [name, file] of Object.entries(result.files)) {
      const link = document.createElement('a');
      link.href = file.url; link.download = name; link.textContent = '下载 ' + name;
      downloads.append(link, document.createElement('br'));
    }
  } else if (value.phase === 'saving') {
    message.textContent = '正在保存，请保持停稳；不重复提交。';
  } else if (value.phase === 'failed') {
    message.textContent = '保存失败：' + value.error;
  } else {
    message.textContent = '尚无保存记录。';
  }
}

async function refreshSave() {
  if (!savePosting) {
    try {
      const response = await fetch('/api/save');
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      renderSave(await response.json());
    } catch (error) {
      document.querySelector('#saveState').textContent = '保存状态不可用：' + error.message;
      document.querySelector('#saveMap').disabled = true;
      document.querySelector('#downloads').replaceChildren();
    }
  }
  setTimeout(refreshSave, 1000);
}

document.querySelector('#saveMap').addEventListener('click', async () => {
  if (!window.confirm('确认已停稳并完成扫描？将保存当前 ROS 2 地图，不加载地图或移动车辆。')) return;
  savePosting = true;
  document.querySelector('#saveMap').disabled = true;
  try {
    const response = await fetch('/api/save', {method: 'POST',
      headers: {'Content-Type': 'application/json', 'X-Test-UI': '1'},
      body: JSON.stringify({confirm_save: true})});
    const value = await response.json();
    if (!response.ok) throw new Error(value.error);
    renderSave(value);
  } catch (error) {
    document.querySelector('#saveState').textContent = '请求未确认：' + error.message + '；将查询保存状态，不自动重发。';
  } finally {
    savePosting = false;
  }
});
refreshSave();
