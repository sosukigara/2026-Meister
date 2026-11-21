// SPDX-License-Identifier: MIT
// 航行状態。サーバーの文字列 (state) を日本語ラベルと色に写すだけで、判断は持たない。
'use strict';

import { getJson } from './api.js';

const STATE_LABELS = {
  starting: '初期化中',
  idle: '待機中',
  sending: '送信中',
  navigating: '移動中',
  succeeded: '到着',
  failed: '失敗',
  canceled: '停止',
  error: 'エラー',
};

const STATE_TONES = {
  starting: 'warn',
  idle: 'idle',
  sending: 'active',
  navigating: 'active',
  succeeded: 'ok',
  failed: 'bad',
  canceled: 'warn',
  error: 'bad',
};

export const BUSY_STATES = new Set(['sending', 'navigating']);

export function stateLabel(state) {
  return STATE_LABELS[state] ?? state;
}

export function stateTone(state) {
  return STATE_TONES[state] ?? 'idle';
}

export function createNavStatus({ badgeEl, messageEl, onState, onSignal }) {
  let current = null;
  let polling = false;

  function describe(status) {
    const text = status.message || STATE_LABELS[status.state] || status.state;
    // サーバーの文言がすでに「地点 1 へ移動中」と番号を言っているなら重ねない。
    const hasIndex = /地点\s*\d+/.test(text);
    if (
      !hasIndex &&
      Number.isInteger(status.current_waypoint) &&
      Number.isInteger(status.waypoint_count)
    ) {
      return `${text} — 地点 ${status.current_waypoint + 1} / ${status.waypoint_count}`;
    }
    return text;
  }

  function render(status) {
    const label = STATE_LABELS[status.state] ?? status.state;
    const detail = describe(status);
    badgeEl.textContent = label;
    badgeEl.dataset.tone = STATE_TONES[status.state] ?? 'idle';
    messageEl.textContent = detail;
    messageEl.title = detail;
    // 記章と同じ文言のときは視覚的には隠す。role=status の要素としては残すので、
    // 読み上げには状態の変化が届く (「待機中」を 2 回並べない)。
    messageEl.classList.toggle('visually-hidden', detail === label);
    onSignal(status);
  }

  async function poll() {
    if (document.hidden || polling) return;
    polling = true;
    try {
      const status = await getJson('/api/status');
      render(status);
      // 変化した時だけ知らせる。1 秒ごとの同じ文言をイベント欄に積まない。
      if (!current || current.state !== status.state || current.message !== status.message) {
        onState(status, current);
      }
      current = status;
    } catch (error) {
      // 取得できない間は最後の状態を出したままにする (api.js が接続断を知らせる)。
    } finally {
      polling = false;
    }
  }

  return { poll, get: () => current };
}
