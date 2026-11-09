// SPDX-License-Identifier: MIT
// 画面の組み立てと配線。値の取得は各パネルに任せ、ここは「誰が何を持つか」と
// 操作の入口だけを決める。
'use strict';

import {
  getJson,
  onConnectionChange,
  postJson,
  secondsSinceLastSuccess,
} from './js/api.js';
import { createEventLog } from './js/events.js';
import { createSignalStrip } from './js/instruments.js';
import { createMapView } from './js/map_view.js';
import { BUSY_STATES, createNavStatus, stateLabel, stateTone } from './js/nav_status.js';
import { createVisionPanel } from './js/vision_panel.js';
import { createWaypointPanel } from './js/waypoints.js';
import { degrees, position } from './js/format.js';

const el = (id) => document.getElementById(id);

const log = createEventLog({
  listEl: el('event-log'),
  toastsEl: el('toasts'),
  clearButton: el('btn-log-clear'),
});
const signals = createSignalStrip();
let busy = false;

const mapView = createMapView({
  canvas: el('map-canvas'),
  stage: el('map-stage'),
  onPlace: (point, yaw) => {
    waypoints.add({ x: point.x, y: point.y, yaw });
    log.info(`目的地を追加 (${position(point.x, point.y)} m / 向き ${degrees(yaw)})`);
  },
  onNotice: (message) => log.warn(message),
});

const waypoints = createWaypointPanel({
  listEl: el('waypoint-list'),
  emptyEl: el('waypoint-empty'),
  countEl: el('tour-count'),
  distanceEl: el('tour-distance'),
  onHover: (index) => mapView.highlight(index),
  onCountChange: () => {
    mapView.setWaypoints(waypoints.list());
    refreshActions();
  },
});

const vision = createVisionPanel({
  imageEl: el('vision-stream'),
  emptyEl: el('vision-empty'),
  badgeEl: el('vision-badge'),
  fpsEl: el('vision-fps'),
  ageEl: el('vision-age'),
  sizeEl: el('vision-size'),
  countEl: el('vision-count'),
  listEl: el('vision-detections'),
  sourceEl: el('vision-source'),
  toggleEl: el('btn-vision-toggle'),
  onNotice: (message, kind) => (kind === 'error' ? log.error(message) : log.info(message)),
  onState: (status) => {
    signals.setVision({
      value: status.has_frame ? `${status.fps.toFixed(1)} fps` : '—',
      note: status.monitor ? `検出 ${status.detection_count ?? 0} 件` : '停止中',
      tone: !status.monitor ? 'idle' : status.has_frame && !status.stale ? 'ok' : 'warn',
    });
  },
});

const nav = createNavStatus({
  badgeEl: el('nav-badge'),
  messageEl: el('nav-message'),
  onSignal: (status) => {
    signals.setNav({
      value: stateLabel(status.state),
      note: status.nav2_ready ? 'Nav2 準備完了' : 'Nav2 待ち',
      tone: stateTone(status.state),
    });
  },
  onState: (status) => {
    busy = BUSY_STATES.has(status.state);
    refreshActions();
    document.title = `${status.message} — Meister`;
    log.info(`航行: ${status.message}`);
  },
});

function refreshActions() {
  const count = waypoints.count();
  el('btn-send').disabled = count === 0 || busy;
  el('btn-cancel').disabled = !busy;
  el('btn-undo').disabled = count === 0 || busy;
  el('btn-clear').disabled = count === 0 || busy;
}

async function loadMap() {
  if (document.hidden) return; // 見えていない画面のために地図を運ばない
  try {
    const meta = await getJson('/api/map');
    if (!meta.has_map) {
      signals.setMap({ value: '—', note: '未受信', tone: 'idle' });
      el('map-empty').hidden = false;
      return;
    }
    mapView.setMapMeta(meta);
    el('map-empty').hidden = true;
    signals.setMap({
      value: `${meta.width}×${meta.height}`,
      note: `${meta.resolution} m/px`,
      tone: 'ok',
    });
    const image = new Image();
    image.onload = () => mapView.setImage(image);
    image.onerror = () => log.warn('地図画像を読み込めませんでした');
    // クエリを付けると URL が毎回変わり、ETag の条件付きリクエストが成立しない。
    // サーバーは毎回 ETag と no-cache を返すので、地図が変わっていなければ
    // ブラウザは 304 を受け取り、本文は転送されない。
    image.src = meta.image_url;
  } catch (error) {
    signals.setMap({ value: '—', note: '未接続', tone: 'bad' });
  }
}

async function pollPose() {
  if (document.hidden) return;
  try {
    const pose = await getJson('/api/pose');
    if (!pose.has_pose) {
      signals.setPose({ value: '—', note: '未受信', tone: 'idle' });
      return;
    }
    mapView.setRobot({ x: pose.x, y: pose.y, yaw: pose.yaw });
    signals.setPose({ value: position(pose.x, pose.y), note: degrees(pose.yaw), tone: 'ok' });
  } catch (error) {
    signals.setPose({ value: '—', note: '未接続', tone: 'bad' });
  }
}

async function sendTour() {
  const tour = waypoints.list();
  if (tour.length === 0) return;
  try {
    const result = await postJson('/api/nav', { waypoints: tour });
    log.ok(`${result.count} 地点へのルートを送信しました`);
  } catch (error) {
    log.error(`ルートを送信できません: ${error.message}`);
  }
}

async function cancelTour() {
  try {
    await postJson('/api/cancel');
    log.warn('停止を要求しました');
  } catch (error) {
    log.error(`停止を要求できません: ${error.message}`);
  }
}

el('btn-send').addEventListener('click', sendTour);
el('btn-cancel').addEventListener('click', cancelTour);
el('btn-undo').addEventListener('click', () => {
  waypoints.undo();
  log.info('最後の目的地を消しました');
});
el('btn-clear').addEventListener('click', () => {
  waypoints.clear();
  log.info('目的地をすべて消しました');
});
el('btn-fit').addEventListener('click', () => mapView.fit());
el('btn-zoom-in').addEventListener('click', () => mapView.zoomBy(1.25));
el('btn-zoom-out').addEventListener('click', () => mapView.zoomBy(1 / 1.25));
el('btn-clear-trail').addEventListener('click', () => {
  mapView.clearTrail();
  log.info('軌跡を消しました');
});

const SHORTCUTS = [
  ['f', () => mapView.fit()],
  ['z', () => {
    waypoints.undo();
    log.info('最後の目的地を消しました');
  }],
  ['c', () => {
    waypoints.clear();
    log.info('目的地をすべて消しました');
  }],
  ['v', () => el('btn-vision-toggle').click()],
  ['+', () => mapView.zoomBy(1.25)],
  ['-', () => mapView.zoomBy(1 / 1.25)],
];

window.addEventListener('keydown', (event) => {
  if (event.metaKey || event.ctrlKey) return;
  if (event.target instanceof Element && event.target.closest('input, textarea, select')) {
    return;
  }
  if (event.key === 'Enter') {
    if (!el('btn-send').disabled) sendTour();
  } else if (event.key === 'Escape') {
    if (!el('btn-cancel').disabled) cancelTour();
  } else {
    const found = SHORTCUTS.find(([key]) => key === event.key.toLowerCase());
    if (!found) return;
    found[1]();
  }
  event.preventDefault();
});

onConnectionChange((online) => {
  el('offline').hidden = online;
  if (!online) log.warn('サーバーに接続できません');
});

setInterval(() => {
  if (el('offline').hidden) return;
  const seconds = secondsSinceLastSuccess();
  el('offline-age').textContent =
    seconds === null ? 'まだ値がありません' : `最後の受信 ${Math.round(seconds)} 秒前`;
}, 1000);

refreshActions();
waypoints.render();
log.info('画面を初期化しました');
loadMap();
pollPose();
nav.poll();
vision.poll();
// 地図は /map が更新された時だけ変わる。5 秒より短くしても地図は早くならない。
setInterval(loadMap, 5000);
setInterval(pollPose, 500);
setInterval(() => nav.poll(), 1000);
setInterval(() => vision.poll(), 1000);
