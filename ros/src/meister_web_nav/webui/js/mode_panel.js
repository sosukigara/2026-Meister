// SPDX-License-Identifier: MIT
// 地図モードと保存。判断は持たず、サーバーの答えを表示するだけ。
// 切替に失敗したときは "切替済み" を出さない（握り潰すと以後の操作が全部嘘になる）。
'use strict';

import { getJson, postJson } from './api.js';

const MODE_LABELS = {
  mapping: '地図生成',
  localization: '保存地図ナビ',
  unknown: 'モード不明',
};

const MODE_TONES = {
  mapping: 'active',
  localization: 'ok',
  unknown: 'warn',
};

export function createModePanel({ badgeEl, switchEl, saveEl, nameEl, onNotice }) {
  let mode = 'unknown';

  function render(next) {
    mode = next;
    badgeEl.textContent = MODE_LABELS[next] ?? next;
    badgeEl.dataset.tone = MODE_TONES[next] ?? 'warn';
    // 地図生成 → 保存地図ナビ の一方通行。戻すのは launch 再起動だけ。
    switchEl.hidden = next !== 'mapping';
  }

  async function poll() {
    try {
      const payload = await getJson('/api/mode');
      render(payload.mode);
    } catch {
      render('unknown');
    }
  }

  switchEl.addEventListener('click', async () => {
    switchEl.disabled = true;
    try {
      const payload = await postJson('/api/mode', { mode: 'localization' });
      render(payload.mode);
      onNotice('保存地図ナビに切り替えました');
    } catch (error) {
      onNotice(`切り替えできません: ${error.message}`, 'error');
      await poll();
    } finally {
      switchEl.disabled = false;
    }
  });

  saveEl.addEventListener('click', async () => {
    saveEl.disabled = true;
    try {
      const name = nameEl.value.trim();
      await postJson('/api/map/save', name ? { name } : {});
      onNotice(name ? `地図 ${name} を保存しました` : '地図を保存しました');
      nameEl.value = '';
    } catch (error) {
      onNotice(`保存できません: ${error.message}`, 'error');
    } finally {
      saveEl.disabled = false;
    }
  });

  poll();

  return { poll, render, currentMode: () => mode };
}
