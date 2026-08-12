'use strict';

const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');
const statusBanner = document.getElementById('status-banner');
const waypointListEl = document.getElementById('waypoint-list');
const btnUndo = document.getElementById('btn-undo');
const btnClear = document.getElementById('btn-clear');
const btnSend = document.getElementById('btn-send');
const btnCancel = document.getElementById('btn-cancel');

let mapMeta = null;
let mapImage = null;
let markers = []; // {x, y, yaw} in map frame (meters, radians)
let drag = null; // {startX, startY} in canvas pixel coords, while dragging

const DRAG_THRESHOLD_PX = 6;

function pixelToWorld(px, py) {
  const { resolution, origin, height } = mapMeta;
  const x = origin.x + px * resolution;
  const y = origin.y + (height - py) * resolution;
  return { x, y };
}

function canvasPointFromEvent(evt) {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  return {
    x: (evt.clientX - rect.left) * scaleX,
    y: (evt.clientY - rect.top) * scaleY,
  };
}

function redraw(previewArrow) {
  if (!mapImage) return;
  ctx.drawImage(mapImage, 0, 0, canvas.width, canvas.height);

  markers.forEach((m, i) => drawMarker(m, i + 1));

  if (previewArrow) {
    drawArrow(previewArrow.startPx, previewArrow.startPy, previewArrow.endPx, previewArrow.endPy, '#4f9dff');
  }
}

function worldToPixel(x, y) {
  const { resolution, origin, height } = mapMeta;
  const px = (x - origin.x) / resolution;
  const py = height - (y - origin.y) / resolution;
  return { px, py };
}

function drawMarker(marker, label) {
  const { px, py } = worldToPixel(marker.x, marker.y);
  const arrowLen = Math.max(canvas.width, canvas.height) * 0.04;
  const endPx = px + Math.cos(marker.yaw) * arrowLen;
  const endPy = py - Math.sin(marker.yaw) * arrowLen;
  drawArrow(px, py, endPx, endPy, '#ff9d3e');

  ctx.beginPath();
  ctx.arc(px, py, 6, 0, Math.PI * 2);
  ctx.fillStyle = '#ff5c5c';
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  ctx.fillStyle = '#ffffff';
  ctx.font = 'bold 12px sans-serif';
  ctx.fillText(String(label), px + 8, py - 8);
}

function drawArrow(x1, y1, x2, y2, color) {
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.stroke();
}

function renderWaypointList() {
  waypointListEl.innerHTML = '';
  markers.forEach((m, i) => {
    const li = document.createElement('li');
    const label = document.createElement('span');
    label.textContent = `#${i + 1}  (${m.x.toFixed(2)}, ${m.y.toFixed(2)})`;
    const del = document.createElement('button');
    del.textContent = '✕';
    del.title = '削除';
    del.addEventListener('click', () => {
      markers.splice(i, 1);
      renderWaypointList();
      redraw();
    });
    li.appendChild(label);
    li.appendChild(del);
    waypointListEl.appendChild(li);
  });
  btnSend.disabled = markers.length === 0;
}

async function loadMap() {
  const res = await fetch('/api/map');
  const meta = await res.json();
  if (!meta.has_map) {
    statusBanner.textContent = '地図を待っています... (/map トピック未受信)';
    return;
  }
  mapMeta = meta;
  canvas.width = meta.width;
  canvas.height = meta.height;

  const img = new Image();
  img.onload = () => {
    mapImage = img;
    redraw();
  };
  img.src = `${meta.image_url}?t=${Date.now()}`;
}

canvas.addEventListener('pointerdown', (evt) => {
  if (!mapMeta) return;
  const p = canvasPointFromEvent(evt);
  drag = { startPx: p.x, startPy: p.y };
  canvas.setPointerCapture(evt.pointerId);
});

canvas.addEventListener('pointermove', (evt) => {
  if (!drag) return;
  const p = canvasPointFromEvent(evt);
  redraw({ startPx: drag.startPx, startPy: drag.startPy, endPx: p.x, endPy: p.y });
});

canvas.addEventListener('pointerup', (evt) => {
  if (!drag || !mapMeta) return;
  const p = canvasPointFromEvent(evt);
  const dx = p.x - drag.startPx;
  const dy = p.y - drag.startPy;
  const dragDistance = Math.hypot(dx, dy);

  const yaw = dragDistance < DRAG_THRESHOLD_PX ? 0 : Math.atan2(-dy, dx);
  const world = pixelToWorld(drag.startPx, drag.startPy);
  markers.push({ x: world.x, y: world.y, yaw });

  drag = null;
  renderWaypointList();
  redraw();
});

btnUndo.addEventListener('click', () => {
  markers.pop();
  renderWaypointList();
  redraw();
});

btnClear.addEventListener('click', () => {
  markers = [];
  renderWaypointList();
  redraw();
});

btnSend.addEventListener('click', async () => {
  if (markers.length === 0) return;
  btnSend.disabled = true;
  try {
    await fetch('/api/nav', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ waypoints: markers }),
    });
  } finally {
    btnSend.disabled = markers.length === 0;
  }
});

btnCancel.addEventListener('click', () => {
  fetch('/api/cancel', { method: 'POST' });
});

async function pollStatus() {
  try {
    const res = await fetch('/api/status');
    const status = await res.json();
    statusBanner.textContent = status.message || status.state;
    statusBanner.className = `status-${status.state}`;
    const busy = status.state === 'sending' || status.state === 'navigating';
    btnCancel.disabled = !busy;
  } catch (e) {
    // server not reachable yet — keep last banner text
  }
}

loadMap();
setInterval(loadMap, 5000);
setInterval(pollStatus, 1000);
pollStatus();
