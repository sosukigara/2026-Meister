// SPDX-License-Identifier: MIT
// ヘッダの計器 4 つ (地図 / 姿勢 / 認識 / 航行) の表示を 1 箇所で決める。
// 同じ値でも「取れていない」と「古い」は色を変える。運用中に効くのは
// 数値そのものより「いま信用できるか」なので、そこを計器の役目にする。
'use strict';

function gauge(root, name) {
  const element = root.getElementById(`gauge-${name}`);
  return {
    element,
    value: element.querySelector('[data-value]'),
    note: element.querySelector('[data-note]'),
  };
}

export function createSignalStrip(root = document) {
  const gauges = {
    map: gauge(root, 'map'),
    pose: gauge(root, 'pose'),
    vision: gauge(root, 'vision'),
    nav: gauge(root, 'nav'),
  };

  function set(name, { value, note, tone = 'idle' }) {
    const target = gauges[name];
    target.value.textContent = value;
    target.note.textContent = note;
    target.element.dataset.tone = tone;
  }

  return {
    set,
    setMap: (state) => set('map', state),
    setPose: (state) => set('pose', state),
    setVision: (state) => set('vision', state),
    setNav: (state) => set('nav', state),
  };
}
