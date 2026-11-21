// SPDX-License-Identifier: MIT
// イベント履歴とトースト。状態の変化を「いつ何が起きたか」で追えるようにする。
// 失敗は必ずここに出し、画面のどこにも出ない失敗を作らない。
'use strict';

import { clock } from './format.js';

const MAX_ENTRIES = 80;
const TOAST_MS = 4200;
// 取り消せる操作は、押し直す時間を長めに取る。
const UNDO_TOAST_MS = 8000;

export function createEventLog({ listEl, toastsEl, clearButton }) {
  const entries = [];

  // 追記型のログなので、読み手が一番下を見ている間だけ最新行へ追従する。
  // 上へ遡って読んでいる最中に勝手に飛ばさない。
  function followingLatest() {
    return listEl.scrollHeight - listEl.scrollTop - listEl.clientHeight < 24;
  }

  function render(follow) {
    listEl.replaceChildren();
    entries.forEach((entry) => {
      const item = document.createElement('li');
      item.className = `event event-${entry.kind}`;
      const time = document.createElement('time');
      time.textContent = clock(entry.at);
      const text = document.createElement('span');
      text.textContent = entry.text;
      item.append(time, text);
      listEl.append(item);
    });
    if (follow) listEl.scrollTop = listEl.scrollHeight;
  }

  function add(kind, text) {
    const follow = followingLatest();
    entries.push({ kind, text, at: new Date() });
    if (entries.length > MAX_ENTRIES) entries.shift();
    render(follow);
  }

  function toast(kind, text, action) {
    const node = document.createElement('p');
    node.className = `toast toast-${kind}`;
    const label = document.createElement('span');
    label.textContent = text;
    node.append(label);

    let timer = null;
    const dismiss = () => {
      if (timer !== null) clearTimeout(timer);
      node.remove();
    };
    if (action) {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'toast-action';
      button.textContent = action.label;
      button.addEventListener('click', () => {
        dismiss();
        action.onClick();
      });
      node.append(button);
    }

    toastsEl.append(node);
    timer = setTimeout(dismiss, action ? UNDO_TOAST_MS : TOAST_MS);
  }

  function report(kind, text) {
    add(kind, text);
    toast(kind, text);
  }

  clearButton.addEventListener('click', () => {
    entries.length = 0;
    render(false);
  });

  return {
    info: (text) => add('info', text),
    ok: (text) => add('ok', text),
    warn: (text) => report('warn', text),
    error: (text) => report('error', text),
    /// 消したものを戻せる操作。トーストに取り消しの口を付ける。
    undoable: (text, label, onClick) => {
      add('warn', text);
      toast('warn', text, { label, onClick });
    },
  };
}
