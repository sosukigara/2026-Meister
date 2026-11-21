// SPDX-License-Identifier: MIT
// キーボード操作。画面の部品ではなく「どのキーがどの操作か」だけを持つ。
// 文字入力中と、ボタン/リンクにフォーカスがあるときの Enter/Space はブラウザに渡す。
'use strict';

const LETTER_SHORTCUTS = [
  ['f', 'fit'],
  ['z', 'undo'],
  ['c', 'clear'],
  ['v', 'toggleVision'],
  ['+', 'zoomIn'],
  ['-', 'zoomOut'],
];

export function createShortcuts({ actions, sendButton, cancelButton }) {
  window.addEventListener('keydown', (event) => {
    if (event.metaKey || event.ctrlKey || event.altKey) return;
    if (event.isComposing) return;
    const target = event.target instanceof Element ? event.target : null;
    if (!target) return;

    // Esc は非常停止。文字入力中でも、ボタンにフォーカスがあっても効かせる。
    if (event.key === 'Escape') {
      if (!cancelButton.disabled) {
        actions.cancel();
        event.preventDefault();
      }
      return;
    }

    // 文字を入力している間は、それ以外のショートカットを横取りしない。
    if (target.closest('input, textarea, select, [contenteditable="true"]')) return;

    // ボタン/リンク上の Enter/Space はその要素の操作に使う。ここで受けると
    // 「全消去」にフォーカスして Enter しただけでルートが送信される。
    if (target.closest('button, a, [role="button"], [role="switch"]')) return;
    if (event.repeat) return;

    if (event.key === 'Enter') {
      if (!sendButton.disabled) {
        actions.send();
        event.preventDefault();
      }
      return;
    }

    const found = LETTER_SHORTCUTS.find(([key]) => key === event.key.toLowerCase());
    if (!found) return;
    const action = actions[found[1]];
    if (typeof action !== 'function') return;
    action();
    event.preventDefault();
  });
}
