// SPDX-License-Identifier: MIT
// 認識モニター。画像は MJPEG を <img> に張って受ける (JS で毎フレーム描き直さない)。
// 数値と検出一覧は /api/vision を 1 秒ごとに読む。
'use strict';

import { getJson, postJson } from './api.js';
import { age, percent, pixels } from './format.js';

const STREAM_URL = '/api/vision.mjpg';
const RETRY_MS = 3000;
// <img> は MJPEG の接続断で error を出さないことがある。最初のフレームが
// 来ないまま一定時間経ったら、新しい URL で張り直す (途中で固まるケースは
// error に頼る)。
const STALL_MS = 4000;
const STALL_MAX_MS = 30000;

export function createVisionPanel({
  imageEl,
  emptyEl,
  badgeEl,
  fpsEl,
  ageEl,
  sizeEl,
  countEl,
  listEl,
  sourceEl,
  toggleEl,
  onState,
  onNotice,
}) {
  let current = null;
  let streaming = false;
  let retryTimer = null;
  let polling = false;
  let detectionsKey = null;
  let streamStartedAt = 0;
  let stallRetryMs = STALL_MS;
  let stallNotified = false;
  // MJPEG は終わらない応答なので、読み込み中に張ると load イベントが永久に完了せず、
  // タブが「読み込み中」のままになる (ヘッドレスブラウザの --screenshot も返ってこない)。
  // 読み込みが終わってから stream を張る。
  let pageLoaded = document.readyState === 'complete';
  if (!pageLoaded) {
    window.addEventListener('load', () => {
      pageLoaded = true;
      ensureStream(current ? Boolean(current.monitor && current.has_frame) : false);
    }, { once: true });
  }

  function ensureStream(shouldStream) {
    if (!pageLoaded) return;
    if (retryTimer !== null) return; // 失敗直後は次の試行まで待つ
    // フレームが来ていれば正常。次に詰まったときは短い間隔からやり直す。
    if (imageEl.naturalWidth > 0) {
      stallRetryMs = STALL_MS;
      stallNotified = false;
    }
    if (
      streaming &&
      shouldStream &&
      imageEl.naturalWidth === 0 &&
      Date.now() - streamStartedAt > stallRetryMs
    ) {
      if (!stallNotified) {
        onNotice('認識映像が止まったため再接続します', 'info');
        stallNotified = true;
      }
      stallRetryMs = Math.min(stallRetryMs * 2, STALL_MAX_MS);
      streaming = false; // フレームが来ない。下で新しい URL を張る
    }
    if (shouldStream === streaming) return;
    streaming = shouldStream;
    if (shouldStream) {
      imageEl.hidden = false;
      // サーバー再起動後に同じ URL を張り直すと、ブラウザが失敗した画像を
      // 再利用して再接続しないことがある。取得のたびに URL を変える。
      streamStartedAt = Date.now();
      imageEl.src = `${STREAM_URL}?t=${streamStartedAt}`;
    } else {
      imageEl.removeAttribute('src');
      imageEl.hidden = true;
    }
  }

  imageEl.addEventListener('error', () => {
    // サーバー再起動などで切れた。張り直しを繰り返すとログが埋まるので間を置く。
    streaming = false;
    imageEl.hidden = true;
    imageEl.removeAttribute('src');
    if (retryTimer === null) {
      retryTimer = setTimeout(() => {
        retryTimer = null;
      }, RETRY_MS);
    }
  });

  // 見えていないタブのために MJPEG を流し続けない。復帰時は poll が張り直す。
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden) return;
    streaming = false;
    imageEl.removeAttribute('src');
    imageEl.hidden = true;
  });

  function renderDetections(status) {
    const detections = status.detections ?? [];
    // 1 秒ごとに同じ一覧を作り直さない。表示に使う値だけで同一性を見る。
    const key = `${Boolean(status.monitor)}|${detections
      .map((d) => `${d.label},${percent(d.score)},${pixels(d.width)},${pixels(d.height)}`)
      .join(';')}`;
    if (key === detectionsKey) return;
    detectionsKey = key;

    listEl.replaceChildren();
    if (detections.length === 0) {
      const item = document.createElement('li');
      item.className = 'detection detection-empty';
      item.textContent = status.monitor ? '検出なし' : '監視を停止しています';
      listEl.append(item);
      return;
    }
    detections.forEach((detection) => {
      const item = document.createElement('li');
      item.className = 'detection';
      const label = document.createElement('span');
      label.className = 'detection-label';
      label.textContent = detection.label;
      const score = document.createElement('span');
      score.className = 'detection-score';
      score.textContent = percent(detection.score);
      const size = document.createElement('span');
      size.className = 'detection-size';
      size.textContent = `${pixels(detection.width)}×${pixels(detection.height)} px`;
      item.append(label, score, size);
      listEl.append(item);
    });
  }

  function render(status) {
    current = status;
    const monitor = Boolean(status.monitor);
    const hasFrame = Boolean(status.has_frame);
    const stale = Boolean(status.stale);

    let badge = '待機';
    let tone = 'idle';
    if (!monitor) {
      badge = '監視 off';
    } else if (!hasFrame) {
      badge = '映像待ち';
      tone = 'warn';
    } else if (stale) {
      badge = '古い';
      tone = 'bad';
    } else {
      badge = '受信中';
      tone = 'ok';
    }
    badgeEl.textContent = badge;
    badgeEl.dataset.tone = tone;

    fpsEl.textContent = Number.isFinite(status.fps) ? status.fps.toFixed(1) : '—';
    ageEl.textContent = status.frame_age_ms === null ? '—' : age(status.frame_age_ms);
    sizeEl.textContent = hasFrame ? `${pixels(status.width)}×${pixels(status.height)}` : '—';
    countEl.textContent = `${status.detection_count ?? 0} 件`;

    const publishers = status.publishers ?? {};
    const source = [`配信 ${publishers.detection_image ? 'あり' : 'なし'}`];
    if (status.frame_id) source.push(`frame ${status.frame_id}`);
    if (status.detection_age_ms !== null && status.detection_age_ms !== undefined) {
      source.push(`検出 ${age(status.detection_age_ms)} 前`);
    }
    sourceEl.textContent = source.join(' / ');

    // has_frame は「最後のフレームがあるか」で、監視中かどうかとは別。
    // 監視 OFF のときは画像を隠して「停止中」を出す。
    emptyEl.hidden = hasFrame && monitor;
    emptyEl.textContent = monitor ? '認識画像を待っています…' : '監視を停止しています';

    renderDetections(status);
    ensureStream(monitor && hasFrame);
    imageEl.classList.toggle('is-stale', stale || !monitor);

    toggleEl.textContent = monitor ? '認識モニター ON' : '認識モニター OFF';
    toggleEl.setAttribute('aria-pressed', String(monitor));
    toggleEl.dataset.tone = monitor ? 'ok' : 'idle';

    onState(status);
  }

  async function poll() {
    if (document.hidden || polling) return;
    polling = true;
    try {
      render(await getJson('/api/vision'));
    } catch (error) {
      // 取得できない間は最後の表示のまま (api.js が接続断を出す)。
    } finally {
      polling = false;
    }
  }

  toggleEl.addEventListener('click', async () => {
    const next = !(current?.monitor ?? false);
    toggleEl.disabled = true;
    try {
      await postJson('/api/vision', { enabled: next });
      onNotice(
        next
          ? '認識モニターを開始しました (検出ノードの推論が動きます)'
          : '認識モニターを停止しました (検出ノードの推論が止まります)',
        'info',
      );
    } catch (error) {
      onNotice(`認識モニターを切り替えられません: ${error.message}`, 'error');
    } finally {
      toggleEl.disabled = false;
      await poll();
    }
  });

  return { poll, get: () => current };
}
