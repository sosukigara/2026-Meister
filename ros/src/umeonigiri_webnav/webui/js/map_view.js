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
const MARKER_RADIUS = 8;
const KEY_PAN_STEP = 40;
const SCALE_CHOICES = [0.5, 1, 2, 5, 10, 20, 50];

export function createMapView({ canvas, stage, onPlace, onPose, onNotice, onCursor }) {
  const ctx = canvas.getContext('2d');
  // 色とフォントは style.css の 1 か所から読む。canvas だけ別の色を持たない。
  const rootStyle = getComputedStyle(document.documentElement);
  const token = (name) => rootStyle.getPropertyValue(name).trim();
  const palette = {
    accent: token('--accent'),
    accentStrong: token('--accent-strong'),
    cyan: token('--cyan'),
    ink: token('--ink'),
    surface: token('--surface'),
    bg: token('--bg'),
  };
  const monoFont = token('--font-mono') || 'monospace';
  const labelFont = (size) => `500 ${size}px ${monoFont}`;

  let meta = null; // {resolution, origin:{x,y}, width, height}
  let image = null;
  let metaKey = null; // 地図の同一性。変わったら表示を合わせ直す
  let view = { scale: 1, x: 0, y: 0 };
  let waypoints = [];
  let robot = null;
  let trail = [];
  let hovered = null;
  // タッチ用の「地図を移動」モード。左ドラッグの意味を置く/ずらすで切り替える。
  let panMode = false;
  // Nav2 が向かっている地点。地図上でも 1 点だけ輪で示す。
  let activeWaypoint = null;
  let drag = null;
  let dpr = 1;
  // 利用者がパン・ズームしたか。していない間だけ、リサイズに合わせて全体表示を
  // 取り直す (操作した表示を勝手に動かさない)。
  let userAdjusted = false;
  // ステージの大きさは ResizeObserver が動いたときだけ測る。draw のたびに
  // getBoundingClientRect を呼ぶと、描く前に強制レイアウトが入る。
  let size = { width: 1, height: 1 };
  // 描画は 1 フレーム 1 回にまとめる。pointermove は 1 フレームに何度も来るので、
  // そのまま描くと後から来た座標で同じ絵を作り直してしまう。
  let frame = 0;

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
  const mapToScreen = (x, y) => {
    const pixels = worldToMapPixels(x, y);
    return { x: pixels.x * view.scale + view.x, y: pixels.y * view.scale + view.y };
  };

  function requestDraw() {
    if (frame !== 0) return;
    frame = requestAnimationFrame(() => {
      frame = 0;
      draw();
    });
  }

  function screenPoint(event) {
    const rect = canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  /// 地図のピクセル矩形の内側か。黒い余白 (縦横比の違い) を目的地にしない。
  function insideMap(mapPoint) {
    return (
      mapPoint.x >= 0 &&
      mapPoint.y >= 0 &&
      mapPoint.x <= meta.width &&
      mapPoint.y <= meta.height
    );
  }

  /// 読み手に「いま地図のどこを指しているか」を返す。drag 中は向きも渡す。
  function emitCursor(screenX, screenY, yaw) {
    if (!onCursor) return;
    if (!meta) {
      onCursor(null);
      return;
    }
    const mapPoint = toMapPixels(screenX, screenY);
    const world = mapPixelsToWorld(mapPoint.x, mapPoint.y);
    onCursor({ x: world.x, y: world.y, yaw, outside: !insideMap(mapPoint) });
  }

  function resize() {
    const rect = stage.getBoundingClientRect();
    size = { width: Math.max(1, rect.width), height: Math.max(1, rect.height) };
    dpr = window.devicePixelRatio || 1;
    const width = Math.round(size.width * dpr);
    const height = Math.round(size.height * dpr);
    // 同じ大きさで width/height を代入するとキャンバスが消えるだけなので避ける。
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    if (!userAdjusted && meta) fit();
    else requestDraw();
  }

  function fit() {
    if (!meta) return;
    const scale = Math.min(size.width / meta.width, size.height / meta.height) * 0.94;
    view = {
      scale,
      x: (size.width - meta.width * scale) / 2,
      y: (size.height - meta.height * scale) / 2,
    };
    userAdjusted = false;
    requestDraw();
  }

  function zoomAt(factor, anchor) {
    if (!meta) return;
    const center = anchor ?? { x: size.width / 2, y: size.height / 2 };
    const next = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, view.scale * factor));
    if (next === view.scale) return;
    const before = toMapPixels(center.x, center.y);
    view.scale = next;
    // 拡大の中心にした点が動かないように offset を決め直す。
    view.x = center.x - before.x * view.scale;
    view.y = center.y - before.y * view.scale;
    userAdjusted = true;
    requestDraw();
  }

  function drawTrail() {
    if (trail.length < 2) return;
    ctx.save();
    ctx.beginPath();
    trail.forEach((point, index) => {
      const p = mapToScreen(point.x, point.y);
      if (index === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    });
    ctx.strokeStyle = palette.cyan;
    ctx.globalAlpha = 0.38;
    ctx.lineWidth = 1.5;
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
      const p = mapToScreen(point.x, point.y);
      if (index === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    });
    ctx.setLineDash([5, 4]);
    ctx.strokeStyle = palette.accent;
    ctx.globalAlpha = 0.72;
    ctx.lineWidth = 1;
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
      const p = mapToScreen(point.x, point.y);
      const length = 22;
      const highlighted = hovered === index;
      const current = activeWaypoint === index;
      drawArrow(
        p,
        { x: p.x + Math.cos(point.yaw) * length, y: p.y - Math.sin(point.yaw) * length },
        highlighted || current ? palette.accentStrong : palette.accent,
        highlighted || current ? 2 : 1.5,
      );
      ctx.save();
      if (current) {
        // 巡回中の 1 点。色を足すだけでなく輪を足し、番号の並びの中で目が止まる。
        ctx.beginPath();
        ctx.arc(p.x, p.y, MARKER_RADIUS + 4, 0, Math.PI * 2);
        ctx.strokeStyle = palette.accent;
        ctx.globalAlpha = 0.45;
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
      ctx.beginPath();
      ctx.arc(p.x, p.y, MARKER_RADIUS, 0, Math.PI * 2);
      ctx.fillStyle = highlighted || current ? palette.accent : palette.surface;
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = highlighted || current ? palette.accentStrong : palette.accent;
      ctx.globalAlpha = highlighted || current ? 1 : 0.8;
      ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.fillStyle = highlighted || current ? palette.bg : palette.ink;
      ctx.font = labelFont(10);
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(String(index + 1), p.x, p.y + 0.5);
      ctx.restore();
    });
  }

  function drawPreview() {
    if (!drag || drag.mode !== 'place') return;
    const distance = Math.hypot(drag.currentX - drag.startX, drag.currentY - drag.startY);
    if (distance < DRAG_THRESHOLD_PX) return;
    // 目的地は amber、初期位置は「生きているロボット」の cyan。色で意味を分ける。
    drawArrow(
      { x: drag.startX, y: drag.startY },
      { x: drag.currentX, y: drag.currentY },
      drag.shiftKey ? palette.cyan : palette.accentStrong,
      2,
    );
  }

  function drawRobot() {
    if (!robot) return;
    const p = mapToScreen(robot.x, robot.y);
    const length = 26;
    drawArrow(
      p,
      { x: p.x + Math.cos(robot.yaw) * length, y: p.y - Math.sin(robot.yaw) * length },
      palette.cyan,
      2,
    );
    ctx.save();
    ctx.beginPath();
    ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
    ctx.fillStyle = palette.cyan;
    ctx.fill();
    ctx.strokeStyle = palette.bg;
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.restore();
  }

  /// 縮尺。地図の resolution が効いているかを目で確かめる手段でもある。
  function drawScaleBar() {
    if (!meta) return;
    const target = 120 / view.scale * meta.resolution; // 120 px に収まる距離
    const distance = SCALE_CHOICES.find((value) => value >= target) ?? SCALE_CHOICES.at(-1);
    const width = (distance / meta.resolution) * view.scale;
    // 左下は操作ヒントの場所だった。いまは地図の外に置いたので右下に縮尺だけ残す。
    const x = size.width - 16 - width;
    const y = size.height - 18;
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + width, y);
    ctx.moveTo(x, y - 4);
    ctx.lineTo(x, y + 4);
    ctx.moveTo(x + width, y - 4);
    ctx.lineTo(x + width, y + 4);
    // 地図が明るい場所でも読めるように、暗い下敷きを1回敷いてから細線を描く。
    ctx.strokeStyle = palette.bg;
    ctx.globalAlpha = 0.85;
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.strokeStyle = palette.ink;
    ctx.globalAlpha = 0.72;
    ctx.lineWidth = 1;
    ctx.stroke();
    const label = `${meters(distance, distance < 1 ? 1 : 0)} m`;
    ctx.font = labelFont(11);
    ctx.textAlign = 'right';
    ctx.textBaseline = 'bottom';
    ctx.strokeStyle = palette.bg;
    ctx.globalAlpha = 0.85;
    ctx.lineWidth = 3;
    ctx.strokeText(label, x + width, y - 6);
    ctx.fillStyle = palette.ink;
    ctx.globalAlpha = 0.72;
    ctx.fillText(label, x + width, y - 6);
    ctx.restore();
  }

  function draw() {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, size.width, size.height);
    if (!meta || !image) return;
    const origin = toScreen(0, 0);
    ctx.save();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(image, origin.x, origin.y, meta.width * view.scale, meta.height * view.scale);
    ctx.restore();
    drawTour();
    drawTrail();
    drawWaypoints();
    drawPreview();
    drawRobot();
    drawScaleBar();
  }

  canvas.addEventListener('pointerdown', (event) => {
    // ここで canvas.focus() を呼ぶと、マウスで触っただけでも :focus-visible の
    // アンバーの輪が付く。canvas は tabindex を持つので、クリック時の焦点は
    // ブラウザの既定に任せる (キーボードのときだけ輪が出る)。
    // 右/中ドラッグと Alt+ドラッグは移動。左クリックは目的地の追加に使う。
    // タッチで「移動」を選んでいるときは左ドラッグも移動にする。
    if (panMode || event.button === 1 || event.button === 2 || event.altKey) {
      drag = { mode: 'pan', clientX: event.clientX, clientY: event.clientY, x: view.x, y: view.y };
      canvas.setPointerCapture(event.pointerId);
      canvas.classList.add('is-panning');
      return;
    }
    if (event.button !== 0) return;
    const point = screenPoint(event);
    // Shift+クリックは初期位置 (AMCL 用)。目的地の追加と混ざらないよう、
    // 押した瞬間の修飾キーを覚えておく (押下と離す間で変わることがある)。
    drag = {
      mode: 'place',
      shiftKey: event.shiftKey,
      startX: point.x,
      startY: point.y,
      currentX: point.x,
      currentY: point.y,
    };
    canvas.setPointerCapture(event.pointerId);
  });

  canvas.addEventListener('pointermove', (event) => {
    const point = screenPoint(event);
    if (!drag) {
      emitCursor(point.x, point.y);
      return;
    }
    if (drag.mode === 'pan') {
      view.x = drag.x + (event.clientX - drag.clientX);
      view.y = drag.y + (event.clientY - drag.clientY);
      userAdjusted = true;
      emitCursor(point.x, point.y);
      requestDraw();
      return;
    }
    drag.currentX = point.x;
    drag.currentY = point.y;
    const dx = drag.currentX - drag.startX;
    const dy = drag.currentY - drag.startY;
    const distance = Math.hypot(dx, dy);
    // 動かしていないクリックに向きは無い。ドラッグ中だけ角度を読む。
    emitCursor(point.x, point.y, distance < DRAG_THRESHOLD_PX ? undefined : Math.atan2(-dy, dx));
    requestDraw();
  });

  function endDrag(event) {
    if (!drag) return;
    const finished = drag;
    drag = null;
    canvas.classList.remove('is-panning');
    if (canvas.hasPointerCapture?.(event.pointerId)) {
      canvas.releasePointerCapture(event.pointerId);
    }
    if (finished.mode !== 'place') {
      requestDraw();
      return;
    }
    if (!meta || !image) {
      onNotice('地図がまだ届いていません');
      requestDraw();
      return;
    }
    const dx = finished.currentX - finished.startX;
    const dy = finished.currentY - finished.startY;
    const distance = Math.hypot(dx, dy);
    // 動かしていないクリックは向き 0。ドラッグの向きがそのまま姿勢になる。
    const yaw = distance < DRAG_THRESHOLD_PX ? 0 : Math.atan2(-dy, dx);
    const mapPoint = toMapPixels(finished.startX, finished.startY);
    if (!insideMap(mapPoint)) {
      onNotice('地図の範囲外です');
      requestDraw();
      return;
    }
    const world = mapPixelsToWorld(mapPoint.x, mapPoint.y);
    if (finished.shiftKey) {
      onPose(world, yaw);
    } else {
      onPlace(world, yaw);
    }
    requestDraw();
  }

  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);
  canvas.addEventListener('pointerleave', () => {
    if (!drag) onCursor?.(null);
  });
  canvas.addEventListener('contextmenu', (event) => event.preventDefault());

  canvas.addEventListener(
    'wheel',
    (event) => {
      if (!meta) return;
      event.preventDefault();
      const point = screenPoint(event);
      zoomAt(event.deltaY < 0 ? 1.12 : 1 / 1.12, point);
      emitCursor(point.x, point.y);
    },
    { passive: false },
  );

  // 焦点が地図にあるときは矢印で視点を動かせる。+/- は画面共通のショートカット。
  canvas.addEventListener('keydown', (event) => {
    const step = KEY_PAN_STEP;
    let dx = 0;
    let dy = 0;
    if (event.key === 'ArrowLeft') dx = step;
    else if (event.key === 'ArrowRight') dx = -step;
    else if (event.key === 'ArrowUp') dy = step;
    else if (event.key === 'ArrowDown') dy = -step;
    else return;
    view.x += dx;
    view.y += dy;
    userAdjusted = true;
    requestDraw();
    event.preventDefault();
  });

  // Shift を押している間は「初期位置モード」であることをカーソルで示す。
  function setPoseMode(on) {
    canvas.classList.toggle('is-pose-mode', on);
  }
  window.addEventListener('keydown', (event) => {
    if (event.key === 'Shift') setPoseMode(true);
  });
  window.addEventListener('keyup', (event) => {
    if (event.key === 'Shift') setPoseMode(false);
  });
  window.addEventListener('blur', () => setPoseMode(false));

  if (typeof ResizeObserver === 'function') {
    new ResizeObserver(resize).observe(stage);
  } else {
    window.addEventListener('resize', resize);
  }
  resize();

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
        requestDraw();
      }
    },
    setImage(next) {
      // 地図は 2 秒ごとに再取得される。見た目が同じでも Image は作り直されるので、
      // ここで fit すると操作中のパン・ズームが定期的に巻き戻る。合わせ直すのは
      // 地図の同一性が変わったとき (setMapMeta) だけにする。
      image = next;
      requestDraw();
    },
    setWaypoints(list) {
      waypoints = list;
      if (activeWaypoint !== null && activeWaypoint >= list.length) activeWaypoint = null;
      requestDraw();
    },
    setRobot(pose) {
      robot = pose;
      const last = trail[trail.length - 1];
      if (!last || Math.hypot(robot.x - last.x, robot.y - last.y) >= TRAIL_MIN_SPACING) {
        trail.push({ x: robot.x, y: robot.y });
        if (trail.length > TRAIL_MAX_POINTS) trail.shift();
      }
      requestDraw();
    },
    clearTrail() {
      trail = [];
      requestDraw();
    },
    highlight(index) {
      if (hovered === index) return;
      hovered = index;
      requestDraw();
    },
    setActiveWaypoint(index) {
      const next = Number.isInteger(index) ? index : null;
      if (next === activeWaypoint) return;
      activeWaypoint = next;
      requestDraw();
    },
    setPanMode(on) {
      panMode = Boolean(on);
      canvas.classList.toggle('is-pan-mode', panMode);
    },
    hasMap: () => Boolean(meta && image),
    resize,
  };
}
