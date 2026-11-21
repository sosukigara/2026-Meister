// SPDX-License-Identifier: MIT
// 地図の足元の読み取り。pointermove は 1 フレームに何度も来るので、textContent の
// 更新は requestAnimationFrame で 1 回にまとめる (計器の文字でレイアウトを揺らさない)。
'use strict';

import { degrees, meters } from './format.js';

export function createMapReadout(readoutEl) {
  let info = null;
  let frame = 0;

  function paint() {
    frame = 0;
    if (!info) {
      readoutEl.textContent = '—';
      readoutEl.classList.remove('is-active');
      return;
    }
    if (info.outside) {
      readoutEl.textContent = '地図外';
      readoutEl.classList.remove('is-active');
      return;
    }
    if (Number.isFinite(info.yaw)) {
      readoutEl.textContent = `向き ${degrees(info.yaw)}`;
      readoutEl.classList.add('is-active');
      return;
    }
    readoutEl.textContent = `x ${meters(info.x, 2)} / y ${meters(info.y, 2)} m`;
    readoutEl.classList.remove('is-active');
  }

  return {
    update(next) {
      info = next;
      if (frame !== 0) return;
      frame = requestAnimationFrame(paint);
    },
  };
}
