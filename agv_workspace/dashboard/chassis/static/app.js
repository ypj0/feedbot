"use strict";
let snapshot = null;
let receivedAt = 0;
let sequence = 0;
let pending = false;
let origin = null;
let path = [];
const element = id => document.getElementById(id);
const motionButtons = Array.from(document.querySelectorAll("[data-action]"));
const value = (number, unit) => Number.isFinite(number) ? `${number.toFixed(4)} ${unit}` : "未收到";

function live() {
  return snapshot && snapshot.ready && receivedAt > 0 && performance.now() - receivedAt < 500;
}

function render() {
  const online = live();
  element("connection").textContent = online ? "反馈在线 · 可手动测试" : "反馈未就绪 / 过期 · 禁止运动";
  motionButtons.forEach(button => { button.disabled = !online || pending || snapshot.control.action !== "stop"; });
  element("duration").disabled = !online || pending || snapshot.control.action !== "stop";
  if (!snapshot) return;
  const samples = snapshot.samples;
  const odom = samples.odom?.value;
  const joints = samples.joints?.value;
  const fields = [["左轮累计角", joints?.left_wheel_joint, "rad"], ["右轮累计角", joints?.right_wheel_joint, "rad"],
    ["前向速度", odom?.vx, "m/s"], ["偏航角速度", odom?.wz, "rad/s"], ["X", odom?.x, "m"], ["Y", odom?.y, "m"], ["航向", odom?.yaw, "rad"]];
  element("telemetry").replaceChildren(...fields.flatMap(([name, number, unit]) => {
    const title = document.createElement("dt"); title.textContent = name;
    const reading = document.createElement("dd"); reading.textContent = online ? value(number, unit) : "—（无新鲜反馈）";
    return [title, reading];
  }));
  element("age").textContent = `当前动作：${snapshot.control.action} · 剩余 ${snapshot.control.remaining_s.toFixed(1)} 秒 · 收到页面数据后 ${(performance.now() - receivedAt).toFixed(0)} ms`;
  element("details").textContent = JSON.stringify({diagnostics: samples.diagnostics, covariance: snapshot.covariance,
    twist_covariance: odom?.twist_covariance, control_error: snapshot.control.error}, null, 2);
  if (snapshot.control.error) element("result").textContent = snapshot.control.error;
  if (origin && odom && online) {
    const distance = Math.hypot(odom.x - origin.x, odom.y - origin.y);
    const angle = Math.atan2(Math.sin(odom.yaw - origin.yaw), Math.cos(odom.yaw - origin.yaw));
    element("displacement").textContent = `相对起点位移 ${distance.toFixed(4)} m · 转角 ${(angle * 180 / Math.PI).toFixed(2)}°（须与实测对照）`;
  }
}

function draw() {
  const canvas = element("plot");
  const context = canvas.getContext("2d");
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.strokeStyle = "#384956";
  context.beginPath(); context.moveTo(280, 0); context.lineTo(280, 320); context.moveTo(0, 160); context.lineTo(560, 160); context.stroke();
  context.fillStyle = "#b9c8d6"; context.fillText("1 米 = 200 像素；仅显示最近 600 个样本", 10, 18);
  if (!origin || !path.length) return;
  context.strokeStyle = "#53c5d4"; context.lineWidth = 2; context.beginPath();
  path.forEach((point, index) => {
    const horizontal = 280 + (point.x - origin.x) * 200;
    const vertical = 160 - (point.y - origin.y) * 200;
    if (index === 0) context.moveTo(horizontal, vertical); else context.lineTo(horizontal, vertical);
  });
  context.stroke();
}

async function poll() {
  const started = performance.now();
  try {
    const response = await fetch("/api/state", {cache: "no-store", signal: AbortSignal.timeout(900)});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error);
    if (performance.now() - started > 400) throw new Error("链路延迟过大，运动按钮保持禁用");
    snapshot = data;
    receivedAt = started;
    sequence = Math.max(sequence, data.control.sequence);
    if (live() && data.samples.odom) {
      const point = data.samples.odom.value;
      if (!origin) origin = {...point};
      path.push({x: point.x, y: point.y});
      if (path.length > 600) path.shift();
      draw();
    }
  } catch (error) {
    receivedAt = 0;
    element("result").textContent = `读取失败：${error.message}；不会自动发送运动请求`;
  }
  render();
  setTimeout(poll, 150);
}

async function command(action) {
  if (!snapshot) { element("result").textContent = "尚未连接车机；请使用实体急停"; return; }
  if (action !== "stop" && (!live() || pending)) return;
  pending = true;
  const body = {action, epoch: snapshot.control.epoch, sequence: ++sequence, issued_at: snapshot.sampled_at};
  if (action !== "stop") body.duration_s = Number(element("duration").value);
  render();
  try {
    const response = await fetch("/api/command", {method: "POST", headers: {"Content-Type": "application/json", "X-Test-UI": "1"},
      body: JSON.stringify(body), signal: AbortSignal.timeout(1000)});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error);
    snapshot = data;
    element("result").textContent = action === "stop" ? "已请求零速；请核对实车停止。网页停止不等于实体急停。" : `已接受 ${data.control.pulse_s} 秒低速测试；请观察实车并测量，必要时点击停止。`;
  } catch (error) {
    element("result").textContent = `请求失败：${error.message}。不自动重试；无法确认停车时使用实体急停。`;
  } finally { pending = false; render(); }
}

motionButtons.forEach(button => button.addEventListener("click", () => command(button.dataset.action)));
element("stop").addEventListener("click", () => command("stop"));
element("mark").addEventListener("click", () => {
  if (!live() || !snapshot.samples.odom) return;
  origin = {...snapshot.samples.odom.value}; path = []; draw(); render();
});
element("save").addEventListener("click", () => {
  const data = JSON.stringify({snapshot, origin, path, saved_at: new Date().toISOString()}, null, 2);
  const url = URL.createObjectURL(new Blob([data], {type: "application/json"}));
  const link = document.createElement("a"); link.href = url; link.download = "chassis-test-reading.json"; link.click(); URL.revokeObjectURL(url);
});
setInterval(render, 100);
draw(); poll();
