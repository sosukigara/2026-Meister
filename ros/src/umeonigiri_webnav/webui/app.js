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
import { createMapReadout } from './js/map_readout.js';
import { createMapView } from './js/map_view.js';
import { createModePanel } from './js/mode_panel.js';
import { BUSY_STATES, createNavStatus, stateLabel, stateTone } from './js/nav_status.js';
import { createParamPanel } from './js/param_panel.js';
import { createShortcuts } from './js/shortcuts.js';
import { createVisionPanel } from './js/vision_panel.js';
import { createWaypointPanel } from './js/waypoints.js';
import { degrees, meters, position } from './js/format.js';

const el = (id) => document.getElementById(id);

const log = createEventLog({
  listEl: el('event-log'),
  toastsEl: el('toasts'),
  clearButton: el('btn-log-clear'),
});
const signals = createSignalStrip();
let busy = false;
// Nav2 が受け付けられる状態か。準備前に送信ボタンを押させない。
let navReady = false;
// 応答が遅いときに interval が要求を積み上げないようにする。古い応答が
// 新しい値を上書きするのも防げる。
let mapBusy = false;
let poseBusy = false;

const readout = createMapReadout(el('map-readout'));

const mapView = createMapView({
  canvas: el('map-canvas'),
  stage: el('map-stage'),
  onCursor: readout.update,
  onPlace: (point, yaw) => {
    // 航行中に足すと、画面のルートと実際に走っているルートが別物になる。
    if (busy) {
      log.warn('航行中は目的地を追加できません');
      return;
    }
    waypoints.add({ x: point.x, y: point.y, yaw });
    // 履歴の 1 行は狭い列で折り返さない長さにする。向きは一覧と地図の矢印で読める。
    log.info(`目的地 ${waypoints.count()}: ${position(point.x, point.y)} m`);
  },
  // 保存地図ナビへ切り替えると AMCL は初期位置を待つ。RViz を開かずに済むよう、
  // 地図の上で Shift+クリック (ドラッグで向き) した点をそのまま初期位置にする。
  onPose: async (point, yaw) => {
    if (busy) {
      log.warn('航行中は初期位置を設定できません');
      return;
    }
    try {
      await postJson('/api/initialpose', { x: point.x, y: point.y, yaw });
      log.ok(`初期位置を設定 (${position(point.x, point.y)} m / 向き ${degrees(yaw)})`);
    } catch (error) {
      log.error(`初期位置を設定できません: ${error.message}`);
    }
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
  // 座標の直接入力。地図クリックの代替として、ロボットの現在位置を初期値にする。
  defaults: () => mapView.getRobot(),
  isInside: (x, y) => mapView.contains(x, y),
  onNotice: (text) => log.info(text),
  dialogEl: el('point-dialog'),
  formEl: el('point-form'),
  titleEl: el('point-dialog-title'),
  xEl: el('point-x'),
  yEl: el('point-y'),
  yawEl: el('point-yaw'),
  errorEl: el('point-error'),
  submitEl: el('point-submit'),
  cancelEl: el('point-cancel'),
  closeEl: el('point-close'),
  prevEl: el('point-prev'),
  nextEl: el('point-next'),
  reorderEl: el('point-reorder'),
});

const vision = createVisionPanel({
  imageEl: el('vision-stream'),
  emptyEl: el('vision-empty'),
  emptyTitleEl: el('vision-empty-title'),
  emptyDetailEl: el('vision-empty-detail'),
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
    // 計器は「信用できるか」だけを言い、件数はパネルに任せる。同じ数字を 2 か所に
    // 出すと、片方だけ古い瞬間に矛盾して見える (実測: 計器 0 件 / パネル 2 件)。
    let note = '受信中';
    let tone = 'ok';
    if (!status.monitor) {
      note = '停止中';
      tone = 'idle';
    } else if (!status.has_frame) {
      note = '映像待ち';
      tone = 'warn';
    } else if (status.stale) {
      note = '古い';
      tone = 'warn';
    }
    signals.setVision({
      value: status.has_frame ? `${status.fps.toFixed(1)} fps` : '—',
      note,
      tone,
    });
  },
});

const mode = createModePanel({
  badgeEl: el('mode-badge'),
  switchEl: el('btn-switch-mode'),
  saveEl: el('btn-save-map'),
  nameEl: el('map-name'),
  onNotice: (message, kind) => (kind === 'error' ? log.error(message) : log.info(message)),
});

const params = createParamPanel({
  dialogEl: el('param-dialog'),
  formEl: el('param-form'),
  openEl: el('btn-params'),
  closeEl: el('param-close'),
  nodeEl: el('param-node'),
  refreshEl: el('param-refresh'),
  filterEl: el('param-filter'),
  listEl: el('param-list'),
  statusEl: el('param-status'),
  onNotice: (message, kind) => (kind === 'error' ? log.error(message) : log.info(message)),
});

const nav = createNavStatus({
  badgeEl: el('nav-badge'),
  messageEl: el('nav-message'),
  onSignal: (status) => {
    signals.setNav({
      value: stateLabel(status.state),
      note: status.nav2_ready ? 'Nav2 準備完了' : 'Nav2 待ち',
      tone: status.nav2_ready ? stateTone(status.state) : 'warn',
    });
    const ready = Boolean(status.nav2_ready);
    if (ready !== navReady) {
      navReady = ready;
      refreshActions();
    }
    // 巡回中の 1 点を地図とリストで共有する。busy でないときは何も光らせない。
    const current =
      (status.state === 'navigating' || status.state === 'sending') &&
      Number.isInteger(status.current_waypoint)
        ? status.current_waypoint
        : null;
    waypoints.setActive(current);
    mapView.setActiveWaypoint(current);
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
  const send = el('btn-send');
  send.disabled = count === 0 || busy || !navReady;
  // スマホでは計器の注記を隠している。無効の理由が画面に出ないと
  // 「押せないボタン」に見えるので、ラベル自体で待ち状態を言う。
  send.textContent = navReady ? 'ルート送信' : 'Nav2 待ち';
  send.title = !navReady
    ? 'Nav2 の準備を待っています'
    : count === 0
      ? '目的地を追加してください'
      : '';
  el('btn-cancel').disabled = !busy;
  el('btn-undo').disabled = count === 0 || busy;
  el('btn-clear').disabled = count === 0 || busy;
  const addCoord = el('btn-add-coord');
  addCoord.disabled = busy;
  addCoord.title = busy ? '航行中は目的地を追加できません' : '座標から目的地を追加';
  // 航行中はリストとモード切替を凍結する。地図の追加は onPlace で止める。
  waypoints.setLocked(busy);
  mode.setLocked(busy);
}

async function loadMap() {
  // 見えていない画面のために地図を運ばない。遅い応答が重ならないように 1 本に絞る。
  if (document.hidden || mapBusy) return;
  mapBusy = true;
  try {
    const meta = await getJson('/api/map');
    if (!meta.has_map) {
      signals.setMap({ value: '—', note: '未受信', tone: 'idle' });
      el('map-empty').hidden = false;
      return;
    }
    // 地図の縦横比を CSS に渡す。横長の地図で高さを余らせないための 1 点。
    document.documentElement.style.setProperty('--map-aspect', (meta.width / meta.height).toFixed(4));
    mapView.setMapMeta(meta);
    el('map-empty').hidden = true;
    const widthMeters = meta.width * meta.resolution;
    const heightMeters = meta.height * meta.resolution;
    signals.setMap({
      value: `${meters(widthMeters, 1)}×${meters(heightMeters, 1)} m`,
      note: `${meters(meta.resolution, 2)} m/px`,
      tone: 'ok',
    });
    el('gauge-map').title = `${meta.width}×${meta.height} セル`;
    const image = new Image();
    image.onload = () => mapView.setImage(image);
    image.onerror = () => log.warn('地図画像を読み込めませんでした');
    // クエリを付けると URL が毎回変わり、ETag の条件付きリクエストが成立しない。
    // サーバーは毎回 ETag と no-cache を返すので、地図が変わっていなければ
    // ブラウザは 304 を受け取り、本文は転送されない。
    image.src = meta.image_url;
  } catch (error) {
    signals.setMap({ value: '—', note: '未接続', tone: 'bad' });
  } finally {
    mapBusy = false;
  }
}

async function pollPose() {
  if (document.hidden || poseBusy) return;
  poseBusy = true;
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
  } finally {
    poseBusy = false;
  }
}

async function sendTour() {
  const tour = waypoints.list();
  if (tour.length === 0 || busy) return;
  try {
    const result = await postJson('/api/nav', { waypoints: tour });
    log.ok(`${result.count} 地点へのルートを送信しました`);
  } catch (error) {
    log.error(`ルートを送信できません: ${error.message}`);
  }
}

async function cancelTour() {
  if (!busy) return;
  try {
    await postJson('/api/cancel');
    log.warn('停止を要求しました');
  } catch (error) {
    log.error(`停止を要求できません: ${error.message}`);
  }
}

/// 全消去は取り消せるようにする。10 地点のルートを 1 クリックで失わせない。
function clearRoute() {
  if (busy || waypoints.count() === 0) return;
  const snapshot = waypoints.list();
  waypoints.clear();
  log.undoable('目的地をすべて消しました', '元に戻す', () => {
    if (busy) {
      log.warn('航行中は元に戻せません');
      return;
    }
    if (waypoints.count() > 0) {
      log.warn('新しい目的地があるため、全消去は元に戻せません');
      return;
    }
    waypoints.set(snapshot);
    log.info('目的地を元に戻しました');
  });
}

/// 1 つ戻す。ボタンと Z キーの両方から呼ぶ。
function undoLast() {
  if (busy || waypoints.count() === 0) return;
  waypoints.undo();
  log.info('最後の目的地を消しました');
}

el('btn-send').addEventListener('click', sendTour);
el('btn-cancel').addEventListener('click', cancelTour);
el('btn-undo').addEventListener('click', undoLast);
el('btn-clear').addEventListener('click', clearRoute);
el('btn-add-coord').addEventListener('click', () => waypoints.openAdd());
el('btn-fit').addEventListener('click', () => mapView.fit());
el('btn-zoom-in').addEventListener('click', () => mapView.zoomBy(1.25));
el('btn-zoom-out').addEventListener('click', () => mapView.zoomBy(1 / 1.25));
el('btn-clear-trail').addEventListener('click', () => {
  mapView.clearTrail();
  log.info('軌跡を消しました');
});

// タッチで地図をずらすモード。マウスでは右/Alt ドラッグがあるのでボタンは隠す。
let panMode = false;
const panButton = el('btn-pan');
panButton.addEventListener('click', () => {
  panMode = !panMode;
  panButton.setAttribute('aria-pressed', String(panMode));
  mapView.setPanMode(panMode);
  document.documentElement.classList.toggle('pan-mode', panMode);
});

createShortcuts({
  sendButton: el('btn-send'),
  cancelButton: el('btn-cancel'),
  actions: {
    fit: () => mapView.fit(),
    undo: undoLast,
    clear: clearRoute,
    toggleVision: () => el('btn-vision-toggle').click(),
    openParams: () => params.open(),
    zoomIn: () => mapView.zoomBy(1.25),
    zoomOut: () => mapView.zoomBy(1 / 1.25),
    send: sendTour,
    cancel: cancelTour,
  },
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

function refreshAll() {
  loadMap();
  pollPose();
  nav.poll();
  vision.poll();
  mode.poll();
}

refreshActions();
waypoints.render();
log.info('画面を初期化しました');
refreshAll();
// 位置と航行状態は 10 Hz。サーバ側は /tf (20〜50 Hz) から姿勢を合成して
// キャッシュしているだけなので、ポーリングを上げれば素直に追従する。
// 各 poll は busy フラグで多重要求を防ぐので、応答が遅いときは自然に間引かれる。
const POSE_PERIOD_MS = 100;
const STATUS_PERIOD_MS = 100;
// 認識は検出一覧の JSON を組み立てる分だけ重い。MJPEG 自体はカメラレートで
// 流れているので、数値の更新は 2 Hz で足りる。
const VISION_PERIOD_MS = 500;
// /map は SLAM が 1 Hz で配る。2 秒に 1 回引けばその 1 枚を取りこぼさない。
// 変わっていないときは ETag 一致で 304（本文なし）なので、通信も PNG 生成も
// 走らない。地図生成中に地図が育つのが見えるようになる。
const MAP_PERIOD_MS = 2000;
setInterval(loadMap, MAP_PERIOD_MS);
setInterval(pollPose, POSE_PERIOD_MS);
setInterval(() => {
  if (!document.hidden) mode.poll();
}, 2000);

setInterval(() => {
  if (!document.hidden) nav.poll();
}, STATUS_PERIOD_MS);
setInterval(() => {
  if (!document.hidden) vision.poll();
}, VISION_PERIOD_MS);

document.addEventListener('visibilitychange', () => {
  if (document.hidden) return;
  // 隠れている間は取得を止めている。戻った瞬間に古い表示を更新する。
  refreshAll();
});
