// SPDX-License-Identifier: MIT
// 地図モードと保存。判断は持たず、サーバーの答えを表示するだけ。
// 切替に失敗したときは "切替済み" を出さない（握り潰すと以後の操作が全部嘘になる）。
'use strict';

import { getJson, postJson } from './api.js';

/// 切替は lifecycle 遷移を 2 回挟むので、サーバー側は最大 40 秒待つ。
const SWITCH_TIMEOUT_MS = 45000;
const SAVE_TIMEOUT_MS = 30000;

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
  // 応答を待つ間に次の要求を積まない。積むと待ち行列が伸びて、同じ画面の
  // 地図や姿勢の要求まで詰まらせる。
  let polling = false;

  function render(next) {
    mode = next;
    badgeEl.textContent = MODE_LABELS[next] ?? next;
    badgeEl.dataset.tone = MODE_TONES[next] ?? 'warn';
    // 双方向に切り替えられる。今と反対のモードを行き先としてボタンに出す。
    const target = next === 'mapping' ? 'localization' : 'mapping';
    switchEl.textContent = target === 'localization' ? '保存地図ナビへ' : '地図生成へ戻す';
    switchEl.hidden = next !== 'mapping' && next !== 'localization';
  }

  async function poll() {
    if (polling) return;
    polling = true;
    try {
      const payload = await getJson('/api/mode');
      render(payload.mode);
    } catch {
      render('unknown');
    } finally {
      polling = false;
    }
  }

  switchEl.addEventListener('click', async () => {
    switchEl.disabled = true;
    // 押した時点のモードから行き先を決める（応答待ちの間に表示が変わっても追従しない）。
    const target = mode === 'mapping' ? 'localization' : 'mapping';
    try {
      const payload = await postJson('/api/mode', { mode: target }, SWITCH_TIMEOUT_MS);
      render(payload.mode);
      onNotice(
        target === 'localization'
          ? '保存地図ナビに切り替えました'
          : '地図生成に切り替えました（地図は引き継ぎます）',
      );
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
      await postJson('/api/map/save', name ? { name } : {}, SAVE_TIMEOUT_MS);
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
