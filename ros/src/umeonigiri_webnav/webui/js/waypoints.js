// SPDX-License-Identifier: MIT
// 目的地リスト。地点の順序・区間距離・合計距離・削除をここだけが持つ。
// 地図側 (map_view.js) は表示のためにリストを受け取るだけで、書き換えない。
'use strict';

import { degrees, meters, position } from './format.js';

export function createWaypointPanel({
  listEl,
  emptyEl,
  countEl,
  distanceEl,
  onHover,
  onCountChange,
}) {
  let waypoints = [];
  // 航行中はリストを凍結する。地図の追加・削除・全消去と食い違うと、
  // 画面のルートと実際に走っているルートが別物になる。
  let locked = false;
  // いま Nav2 が向かっている地点 (0 始まり)。地図とリストで同じ点を光らせる。
  let active = null;

  function legDistance(index) {
    if (index === 0) return null;
    const previous = waypoints[index - 1];
    const current = waypoints[index];
    return Math.hypot(current.x - previous.x, current.y - previous.y);
  }

  function totalDistance() {
    let sum = 0;
    for (let index = 1; index < waypoints.length; index += 1) {
      sum += legDistance(index);
    }
    return sum;
  }

  function render() {
    if (active !== null && active >= waypoints.length) active = null;
    listEl.replaceChildren();
    waypoints.forEach((point, index) => {
      const item = document.createElement('li');
      item.className = 'waypoint';
      if (index === active) {
        item.classList.add('is-active');
        item.setAttribute('aria-current', 'step');
      }

      const order = document.createElement('span');
      order.className = 'waypoint-order';
      order.textContent = String(index + 1);

      const body = document.createElement('span');
      body.className = 'waypoint-body';
      const coords = document.createElement('span');
      coords.className = 'waypoint-coords';
      coords.textContent = `${position(point.x, point.y)} m`;
      const heading = document.createElement('span');
      heading.className = 'waypoint-heading';
      const distance = legDistance(index);
      // 「区間」は文脈 (ルート線と合計) で分かる。狭い停止点で折れない長さにする。
      const leg = distance === null ? '起点' : `${meters(distance)} m`;
      heading.textContent = `向き ${degrees(point.yaw)} · ${leg}`;
      body.append(coords, heading);

      const remove = document.createElement('button');
      remove.type = 'button';
      remove.className = 'waypoint-remove';
      // 定数だけを入れるので innerHTML でも安全。線アイコンは Heading と同じ 1.5 の線幅。
      remove.innerHTML =
        '<svg viewBox="0 0 16 16" width="14" height="14" aria-hidden="true">' +
        '<path d="M4 4 12 12M12 4 4 12" fill="none" stroke="currentColor" ' +
        'stroke-width="1.5" stroke-linecap="round"/></svg>';
      remove.title = locked ? '航行中は編集できません' : `${index + 1} 番目を削除`;
      remove.setAttribute('aria-label', `${index + 1} 番目の目的地を削除`);
      remove.disabled = locked;
      remove.addEventListener('click', () => {
        if (locked) return;
        waypoints.splice(index, 1);
        update();
      });

      item.addEventListener('pointerenter', () => onHover(index));
      item.addEventListener('pointerleave', () => onHover(null));
      item.addEventListener('focusin', () => onHover(index));
      item.addEventListener('focusout', () => onHover(null));

      item.append(order, body, remove);
      listEl.append(item);
    });

    emptyEl.hidden = waypoints.length > 0;
    countEl.textContent = String(waypoints.length);
    distanceEl.textContent = meters(totalDistance());
  }

  function update() {
    render();
    onCountChange();
  }

  /// 追加した地点が画面外だと気づけない。追加時だけ末尾へ送る。
  function scrollToEnd() {
    requestAnimationFrame(() => {
      if (listEl.scrollWidth > listEl.clientWidth) {
        listEl.scrollLeft = listEl.scrollWidth;
      }
      if (listEl.scrollHeight > listEl.clientHeight) {
        listEl.scrollTop = listEl.scrollHeight;
      }
    });
  }

  return {
    add(point) {
      if (locked) return;
      waypoints.push(point);
      update();
      scrollToEnd();
    },
    /// 全消去の取り消しなど、一覧をまるごと差し替えるときに使う。
    set(points) {
      if (locked) return;
      waypoints = points.map((point) => ({ ...point }));
      update();
    },
    undo() {
      if (locked) return;
      waypoints.pop();
      update();
    },
    clear() {
      if (locked) return;
      waypoints.length = 0;
      active = null;
      update();
    },
    setLocked(next) {
      locked = Boolean(next);
      listEl.querySelectorAll('.waypoint-remove').forEach((button, index) => {
        button.disabled = locked;
        button.title = locked ? '航行中は編集できません' : `${index + 1} 番目を削除`;
      });
    },
    setActive(index) {
      const next = Number.isInteger(index) ? index : null;
      if (next === active) return;
      active = next;
      [...listEl.children].forEach((item, i) => {
        const current = i === active;
        item.classList.toggle('is-active', current);
        if (current) item.setAttribute('aria-current', 'step');
        else item.removeAttribute('aria-current');
      });
      // 巡回中の地点がスクロールの外にいると見失う。ただしページ全体は動かさず、
      // 一覧のスクロールだけを寄せる (地図を見ている人の視界を奪わない)。
      const item = active === null ? null : listEl.children[active];
      if (!item) return;
      const box = listEl.getBoundingClientRect();
      const rect = item.getBoundingClientRect();
      if (rect.left < box.left) listEl.scrollLeft -= box.left - rect.left;
      else if (rect.right > box.right) listEl.scrollLeft += rect.right - box.right;
      if (rect.top < box.top) listEl.scrollTop -= box.top - rect.top;
      else if (rect.bottom > box.bottom) listEl.scrollTop += rect.bottom - box.bottom;
    },
    list: () => waypoints.map((point) => ({ ...point })),
    count: () => waypoints.length,
    totalDistance,
    render,
  };
}
