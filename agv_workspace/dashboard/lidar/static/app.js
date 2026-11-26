(function (globalScope) {
  "use strict";

  const POINT_COUNT = 1075;
  const SOURCE_FRESHNESS_SECONDS = 0.5;
  const REQUEST_TIMEOUT_MILLISECONDS = 1200;
  const zoomLevels = [3, 5, 8, 12, 20, 40];

  function polarToCanvas(range, angle, scale, centerX, centerY) {
    const forward = range * Math.cos(angle);
    const left = range * Math.sin(angle);
    return {
      x: centerX - left * scale,
      y: centerY - forward * scale
    };
  }

  function geometryMatches(scan) {
    if (!scan || !Array.isArray(scan.ranges) || scan.ranges.length !== POINT_COUNT) return false;
    const geometry = scan.geometry;
    if (!geometry || !Number.isFinite(geometry.angle_min) || !Number.isFinite(geometry.angle_max) || !Number.isFinite(geometry.angle_increment)) return false;
    if (geometry.angle_increment <= 0) return false;
    const impliedCount = (geometry.angle_max - geometry.angle_min) / geometry.angle_increment + 1;
    return Math.abs(impliedCount - POINT_COUNT) <= 0.05;
  }

  function effectiveSourceAge(sourceAge, requestElapsed, receiverElapsed) {
    if (!Number.isFinite(sourceAge) || !Number.isFinite(requestElapsed) || !Number.isFinite(receiverElapsed)) return Infinity;
    return sourceAge + Math.max(0, requestElapsed) + Math.max(0, receiverElapsed);
  }

  globalScope.LidarGeometry = { polarToCanvas, geometryMatches, effectiveSourceAge };
  if (typeof document === "undefined") return;

  const canvas = document.getElementById("scan");
  const context = canvas.getContext("2d");
  const stateLabel = document.getElementById("state");
  const watermark = document.getElementById("watermark");
  const pauseButton = document.getElementById("pause");
  const downloadButton = document.getElementById("download");
  let zoomIndex = 2;
  let paused = false;
  let displayedSnapshot = null;
  let receiverMonotonic = null;
  let requestElapsedAtReceipt = null;
  let transportStatus = "offline";
  let refreshInFlight = false;

  function resizeCanvas() {
    const ratio = globalScope.devicePixelRatio || 1;
    const bounds = canvas.getBoundingClientRect();
    canvas.width = Math.max(1, Math.floor(bounds.width * ratio));
    canvas.height = Math.max(1, Math.floor(bounds.height * ratio));
    draw();
  }

  function drawGrid(centerX, centerY, scale, maxRange) {
    context.strokeStyle = "#17483d";
    context.lineWidth = 1;
    for (let radius = 1; radius <= maxRange; radius += 1) {
      context.beginPath();
      context.arc(centerX, centerY, radius * scale, 0, Math.PI * 2);
      context.stroke();
    }
    context.strokeStyle = "#2b7564";
    context.beginPath();
    context.moveTo(centerX, 0);
    context.lineTo(centerX, canvas.height);
    context.moveTo(0, centerY);
    context.lineTo(canvas.width, centerY);
    context.stroke();
    context.fillStyle = "#e7fff6";
    context.beginPath();
    context.moveTo(centerX, centerY - 10);
    context.lineTo(centerX - 7, centerY + 8);
    context.lineTo(centerX + 7, centerY + 8);
    context.closePath();
    context.fill();
  }

  function draw() {
    context.clearRect(0, 0, canvas.width, canvas.height);
    const centerX = canvas.width / 2;
    const centerY = canvas.height * 0.72;
    const maxRange = zoomLevels[zoomIndex];
    const scale = Math.min(canvas.width * 0.46, canvas.height * 0.68) / maxRange;
    drawGrid(centerX, centerY, scale, maxRange);
    if (!displayedSnapshot || !geometryMatches(displayedSnapshot.scan)) return;
    const scan = displayedSnapshot.scan;
    context.fillStyle = paused ? "#ffd66b" : "#5fffb0";
    for (let index = 0; index < scan.ranges.length; index += 1) {
      const range = scan.ranges[index];
      if (!Number.isFinite(range) || range < scan.geometry.range_min || range > scan.geometry.range_max || range > maxRange) continue;
      const angle = scan.geometry.angle_min + index * scan.geometry.angle_increment;
      const point = polarToCanvas(range, angle, scale, centerX, centerY);
      context.fillRect(point.x - 1.2, point.y - 1.2, 2.4, 2.4);
    }
  }

  function textValue(identifier, value) {
    document.getElementById(identifier).textContent = value;
  }

  function yesNo(value) {
    if (value === true) return "YES";
    if (value === false) return "NO";
    return "--";
  }

  function updateLabels() {
    const receiverAge = receiverMonotonic === null ? null : (performance.now() - receiverMonotonic) / 1000;
    const effectiveAge = displayedSnapshot && receiverAge !== null ? effectiveSourceAge(displayedSnapshot.source_age_s, requestElapsedAtReceipt, receiverAge) : Infinity;
    if (!paused && displayedSnapshot && effectiveAge > SOURCE_FRESHNESS_SECONDS) {
      displayedSnapshot = null;
      transportStatus = "stale";
      draw();
    }
    if (paused && displayedSnapshot) {
      stateLabel.textContent = transportStatus === "live" ? "PAUSED · NOT LIVE" : "PAUSED · UPSTREAM OFFLINE";
      stateLabel.className = "state paused";
      watermark.textContent = "PAUSED · 历史帧";
      watermark.hidden = false;
    } else if (transportStatus === "live" && displayedSnapshot) {
      stateLabel.textContent = "LIVE CANDIDATE";
      stateLabel.className = "state live";
      watermark.hidden = true;
    } else {
      stateLabel.textContent = transportStatus.toUpperCase();
      stateLabel.className = "state offline";
      watermark.textContent = transportStatus === "stale" ? "STALE · 已清空" : "OFFLINE · 已清空";
      watermark.hidden = false;
    }

    if (!displayedSnapshot) {
      ["source-age", "frequency", "points", "valid", "ready", "healthy", "navigation", "diagnostic"].forEach(identifier => textValue(identifier, "--"));
      textValue("receiver-age", receiverAge === null ? "--" : receiverAge.toFixed(2) + " s");
      downloadButton.disabled = true;
      return;
    }
    const diagnostic = displayedSnapshot.diagnostic || {};
    const stats = displayedSnapshot.stats || {};
    textValue("source-age", Number.isFinite(displayedSnapshot.source_age_s) ? displayedSnapshot.source_age_s.toFixed(3) + " s" : "--");
    textValue("receiver-age", receiverAge === null ? "--" : receiverAge.toFixed(2) + " s");
    textValue("frequency", Number.isFinite(displayedSnapshot.frequency_hz) ? displayedSnapshot.frequency_hz.toFixed(2) + " Hz" : "--");
    textValue("points", stats.point_count === POINT_COUNT ? String(stats.point_count) : "--");
    textValue("valid", Number.isInteger(stats.valid_return_count) ? String(stats.valid_return_count) : "--");
    textValue("ready", diagnostic.fresh ? yesNo(diagnostic.ready) : "STALE");
    textValue("healthy", diagnostic.fresh ? yesNo(diagnostic.healthy) : "STALE");
    textValue("navigation", diagnostic.fresh ? yesNo(diagnostic.navigation_accepted) + (diagnostic.navigation_accepted === false ? " · NOT ACCEPTED" : "") : "STALE");
    textValue("diagnostic", diagnostic.fresh ? diagnostic.state : "STALE");
    downloadButton.disabled = false;
  }

  async function refresh() {
    if (refreshInFlight) return;
    refreshInFlight = true;
    const requestStarted = performance.now();
    const controller = new AbortController();
    const timeout = globalScope.setTimeout(function () { controller.abort(); }, REQUEST_TIMEOUT_MILLISECONDS);
    try {
      const response = await fetch("/api/scan", { cache: "no-store", signal: controller.signal });
      const payload = await response.json();
      const requestElapsed = (performance.now() - requestStarted) / 1000;
      const freshAtReceipt = effectiveSourceAge(payload.source_age_s, requestElapsed, 0) <= SOURCE_FRESHNESS_SECONDS;
      if (response.ok && payload.live && freshAtReceipt && geometryMatches(payload.scan)) {
        transportStatus = "live";
        if (!paused) {
          displayedSnapshot = payload;
          receiverMonotonic = performance.now();
          requestElapsedAtReceipt = requestElapsed;
          draw();
        }
      } else {
        transportStatus = payload.status === "stale" ? "stale" : "offline";
        if (!paused) {
          displayedSnapshot = null;
          receiverMonotonic = performance.now();
          requestElapsedAtReceipt = null;
          draw();
        }
      }
    } catch (error) {
      transportStatus = "offline";
      if (!paused) {
        displayedSnapshot = null;
        receiverMonotonic = performance.now();
        requestElapsedAtReceipt = null;
        draw();
      }
    } finally {
      globalScope.clearTimeout(timeout);
      refreshInFlight = false;
    }
    updateLabels();
  }

  document.getElementById("zoom-out").addEventListener("click", function () {
    zoomIndex = Math.min(zoomLevels.length - 1, zoomIndex + 1);
    textValue("zoom", zoomLevels[zoomIndex] + " m");
    draw();
  });
  document.getElementById("zoom-in").addEventListener("click", function () {
    zoomIndex = Math.max(0, zoomIndex - 1);
    textValue("zoom", zoomLevels[zoomIndex] + " m");
    draw();
  });
  pauseButton.addEventListener("click", function () {
    paused = !paused;
    pauseButton.textContent = paused ? "继续实时" : "暂停画面";
    if (!paused) {
      displayedSnapshot = null;
      receiverMonotonic = null;
      requestElapsedAtReceipt = null;
      transportStatus = "refreshing";
      refresh();
    }
    updateLabels();
    draw();
  });
  downloadButton.addEventListener("click", function () {
    if (!displayedSnapshot) return;
    const blob = new Blob([JSON.stringify(displayedSnapshot, null, 2)], { type: "application/json" });
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob);
    link.download = "ltr2-managed-candidate-snapshot.json";
    link.click();
    URL.revokeObjectURL(link.href);
  });
  globalScope.addEventListener("resize", resizeCanvas);
  resizeCanvas();
  refresh();
  globalScope.setInterval(refresh, 250);
  globalScope.setInterval(updateLabels, 100);
})(typeof globalThis !== "undefined" ? globalThis : this);
