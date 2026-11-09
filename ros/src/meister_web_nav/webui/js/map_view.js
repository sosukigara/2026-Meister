// SPDX-License-Identifier: MIT
// 地図キャンバス。表示は「地図ピクセル → 画面 px」の相似変換 1 つで決める。
// 目的地の一覧は waypoints.js が持ち、ここは描画と入力だけを担う。
'use strict';

import { meters } from './format.js';

const DRAG_THRESHOLD_PX = 6;
const TRAIL_MAX_POINTS = 300;
const TRAIL_MIN_SPACING = 0.05; // meters: 間引きして点の塊を防ぐ
const ZOOM_MIN = 0.15;
const ZOOM_MAX = 14;
const MARKER_RADIUS = 9;

export function createMapView({ canvas, stage, onPlace, onNotice }) {
  const ctx = canvas.getContext('2d');
  let meta = null; // {resolution, origin:{x,y}, width, height}
  let image = null;
  let metaKey = null; // 地図の同一性。変わったら表示を合わせ直す
  let view = { scale: 1, x: 0, y: 0 };
  let waypoints = [];
  let robot = null;
  let trail = [];
  let hovered = null;
  let drag = null;
  let dpr = 1;

  const worldToMapPixels = (x, y) => ({
    x: (x - meta.origin.x) / meta.resolution,
    y: meta.height - (y - meta.origin.y) / meta.resolution,
  });
  const mapPixelsToWorld = (x, y) => ({
    x: meta.origin.x + x * meta.resolution,
    y: meta.origin.y + (meta.height - y) * meta.resolution,
  });
  const toScreen = (x, y) => ({ x: x * view.scale + view.x, y: y * view.scale + view.y });
  const toMapPixels = (x, y) => ({ x: (x - view.x) / view.scale, y: (y - view.y) / view.scale });

  function screenPoint(event) {
    const rect = canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  function stageSize() {
    const rect = stage.getBoundingClientRect();
    return { width: Math.max(1, rect.width), height: Math.max(1, rect.height) };
  }

  function resize() {
    const size = stageSize();
    dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(size.width * dpr);
    canvas.height = Math.round(size.height * dpr);
    draw();
  }

  function fit() {
    if (!meta) return;
    const size = stageSize();
    const scale = Math.min(size.width / meta.width, size.height / meta.height) * 0.94;
    view = {
      scale,
      x: (size.width - meta.width * scale) / 2,
      y: (size.height - meta.height * scale) / 2,
    };
    draw();
  }

  function zoomAt(factor, anchor) {
    const size = stageSize();
    const center = anchor ?? { x: size.width / 2, y: size.height / 2 };
    const next = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, view.scale * factor));
    if (next === view.scale) return;
    const before = toMapPixels(center.x, center.y);
    view.scale = next;
    // 拡大の中心にした点が動かないように offset を決め直す。
    view.x = center.x - before.x * view.scale;
    view.y = center.y - before.y * view.scale;
    draw();
  }

  function drawBackdrop() {
    const size = stageSize();
    const step = 28;
    ctx.save();
    ctx.strokeStyle = 'rgba(233, 230, 223, 0.045)';
    ctx.lineWidth = 1;
    const offsetX = ((view.x % step) + step) % step;
    const offsetY = ((view.y % step) + step) % step;
    for (let x = offsetX; x < size.width; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, size.height);
      ctx.stroke();
    }
    for (let y = offsetY; y < size.height; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(size.width, y);
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawTrail() {
    if (trail.length < 2) return;
    ctx.save();
    ctx.beginPath();
    trail.forEach((point, index) => {
      const p = toScreen(...Object.values(worldToMapPixels(point.x, point.y)));
      if (index === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    });
    ctx.strokeStyle = 'rgba(99, 199, 214, 0.55)';
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.stroke();
    ctx.restore();
  }

  function drawTour() {
    if (waypoints.length === 0) return;
    ctx.save();
    ctx.beginPath();
    waypoints.forEach((point, index) => {
      const p = toScreen(...Object.values(worldToMapPixels(point.x, point.y)));
      if (index === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    });
    ctx.setLineDash([7, 5]);
    ctx.strokeStyle = 'rgba(217, 164, 65, 0.85)';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.restore();
  }

  function drawArrow(from, to, color, width = 2) {
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(from.x, from.y);
    ctx.lineTo(to.x, to.y);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
    ctx.restore();
  }

  function drawWaypoints() {
    waypoints.forEach((point, index) => {
      const p = toScreen(...Object.values(worldToMapPixels(point.x, point.y)));
      const length = 26;
      drawArrow(
        p,
        { x: p.x + Math.cos(point.yaw) * length, y: p.y - Math.sin(point.yaw) * length },
        hovered === index ? '#f0c05a' : 'rgba(217, 164, 65, 0.9)',
        2,
      );
      ctx.save();
      ctx.beginPath();
      ctx.arc(p.x, p.y, MARKER_RADIUS, 0, Math.PI * 2);
      ctx.fillStyle = hovered === index ? '#d9a441' : '#1b2126';
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = hovered === index ? '#f0c05a' : '#d9a441';
      ctx.stroke();
      ctx.fillStyle = hovered === index ? '#14181c' : '#e9e6df';
      ctx.font = '600 10px ui-monospace, SFMono-Regular, Menlo, monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(index + 1), p.x, p.y + 0.5);
      ctx.restore();
    });
  }

  function drawRobot() {
    if (!robot) return;
    const p = toScreen(...Object.values(worldToMapPixels(robot.x, robot.y)));
    const length = 30;
    drawArrow(
      p,
      { x: p.x + Math.cos(robot.yaw) * length, y: p.y - Math.sin(robot.yaw) * length },
      '#63c7d6',
      2.5,
    );
    ctx.save();
    ctx.beginPath();
    ctx.arc(p.x, p.y, 5, 0, Math.PI * 2);
    ctx.fillStyle = '#63c7d6';
    ctx.fill();
    ctx.strokeStyle = '#0f1418';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.restore();
  }

  /// 縮尺。地図の resolution が効いているかを目で確かめる手段でもある。
  function drawScaleBar() {
    if (!meta) return;
    const choices = [0.5, 1, 2, 5, 10, 20, 50];
    const target = 120 / view.scale * meta.resolution; // 120 px に収まる距離
    const distance = choices.find((value) => value >= target) ?? choices.at(-1);
    const width = (distance / meta.resolution) * view.scale;
    const size = stageSize();
    const x = 16;
    const y = size.height - 18;
    ctx.save();
    ctx.strokeStyle = 'rgba(233, 230, 223, 0.75)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + width, y);
    ctx.moveTo(x, y - 4);
    ctx.lineTo(x, y + 4);
    ctx.moveTo(x + width, y - 4);
    ctx.lineTo(x + width, y + 4);
    ctx.stroke();
    ctx.fillStyle = 'rgba(233, 230, 223, 0.75)';
    ctx.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`${meters(distance, distance < 1 ? 1 : 0)} m`, x + width + 6, y + 4);
    ctx.restore();
  }

  function draw() {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const size = stageSize();
    ctx.clearRect(0, 0, size.width, size.height);
    drawBackdrop();
    if (!meta || !image) return;
    const origin = toScreen(0, 0);
    ctx.save();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(image, origin.x, origin.y, meta.width * view.scale, meta.height * view.scale);
    ctx.restore();
    drawTour();
    drawTrail();
    drawWaypoints();
    drawRobot();
    drawScaleBar();
  }

  function previewArrow() {
    if (!drag || drag.mode !== 'place') return;
    const distance = Math.hypot(drag.currentX - drag.startX, drag.currentY - drag.startY);
    if (distance < DRAG_THRESHOLD_PX) return;
    drawArrow({ x: drag.startX, y: drag.startY },
              { x: drag.currentX, y: drag.currentY }, '#f0c05a', 2);
  }

  canvas.addEventListener('pointerdown', (event) => {
    // 右/中ドラッグと Alt+ドラッグは移動。左クリックは目的地の追加に使う。
    if (event.button === 1 || event.button === 2 || event.altKey) {
      drag = { mode: 'pan', clientX: event.clientX, clientY: event.clientY, x: view.x, y: view.y };
      canvas.setPointerCapture(event.pointerId);
      canvas.classList.add('is-panning');
      return;
    }
    if (event.button !== 0) return;
    const point = screenPoint(event);
    drag = { mode: 'place', startX: point.x, startY: point.y, currentX: point.x, currentY: point.y };
    canvas.setPointerCapture(event.pointerId);
  });

  canvas.addEventListener('pointermove', (event) => {
    if (!drag) return;
    if (drag.mode === 'pan') {
      view.x = drag.x + (event.clientX - drag.clientX);
      view.y = drag.y + (event.clientY - drag.clientY);
      draw();
      return;
    }
    const point = screenPoint(event);
    drag.currentX = point.x;
    drag.currentY = point.y;
    draw();
    previewArrow();
  });

  function endDrag(event) {
    if (!drag) return;
    const finished = drag;
    drag = null;
    canvas.classList.remove('is-panning');
    if (canvas.hasPointerCapture?.(event.pointerId)) {
      canvas.releasePointerCapture(event.pointerId);
    }
    if (finished.mode !== 'place') return;
    if (!meta || !image) {
      onNotice('地図がまだ届いていません');
      draw();
      return;
    }
    const dx = finished.currentX - finished.startX;
    const dy = finished.currentY - finished.startY;
    const distance = Math.hypot(dx, dy);
    // 動かしていないクリックは向き 0。ドラッグの向きがそのまま姿勢になる。
    const yaw = distance < DRAG_THRESHOLD_PX ? 0 : Math.atan2(-dy, dx);
    const mapPoint = toMapPixels(finished.startX, finished.startY);
    onPlace(mapPixelsToWorld(mapPoint.x, mapPoint.y), yaw);
    draw();
  }

  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);
  canvas.addEventListener('contextmenu', (event) => event.preventDefault());

  canvas.addEventListener(
    'wheel',
    (event) => {
      if (!meta) return;
      event.preventDefault();
      zoomAt(event.deltaY < 0 ? 1.12 : 1 / 1.12, screenPoint(event));
    },
    { passive: false },
  );

  if (typeof ResizeObserver === 'function') {
    new ResizeObserver(resize).observe(stage);
  } else {
    window.addEventListener('resize', resize);
  }

  return {
    fit,
    zoomBy: (factor) => zoomAt(factor),
    setMapMeta(next) {
      meta = next;
      const key = `${next.width}x${next.height}@${next.resolution},${next.origin.x},${next.origin.y}`;
      if (key !== metaKey) {
        metaKey = key;
        trail = []; // 地図座標が変わり得るため軌跡をリセット
        fit();
      } else {
        draw();
      }
    },
    setImage(next) {
      image = next;
      fit();
    },
    setWaypoints(list) {
      waypoints = list;
      draw();
    },
    setRobot(pose) {
      robot = pose;
      const last = trail[trail.length - 1];
      if (!last || Math.hypot(robot.x - last.x, robot.y - last.y) >= TRAIL_MIN_SPACING) {
        trail.push({ x: robot.x, y: robot.y });
        if (trail.length > TRAIL_MAX_POINTS) trail.shift();
      }
      draw();
    },
    clearTrail() {
      trail = [];
      draw();
    },
    highlight(index) {
      hovered = index;
      draw();
    },
    hasMap: () => Boolean(meta && image),
    resize,
  };
}
