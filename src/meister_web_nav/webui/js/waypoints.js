// SPDX-License-Identifier: MIT
// 目的地リスト。地点の順序・合計距離・削除をここだけが持つ。
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

  function totalDistance() {
    let sum = 0;
    for (let index = 1; index < waypoints.length; index += 1) {
      const previous = waypoints[index - 1];
      const current = waypoints[index];
      sum += Math.hypot(current.x - previous.x, current.y - previous.y);
    }
    return sum;
  }

  function render() {
    listEl.replaceChildren();
    waypoints.forEach((point, index) => {
      const item = document.createElement('li');
      item.className = 'waypoint';

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
      heading.textContent = `向き ${degrees(point.yaw)}`;
      body.append(coords, heading);

      const remove = document.createElement('button');
      remove.type = 'button';
      remove.className = 'waypoint-remove';
      remove.textContent = '✕';
      remove.title = `${index + 1} 番目を削除`;
      remove.setAttribute('aria-label', `${index + 1} 番目の目的地を削除`);
      remove.addEventListener('click', () => {
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

  return {
    add(point) {
      waypoints.push(point);
      update();
    },
    undo() {
      waypoints.pop();
      update();
    },
    clear() {
      waypoints.length = 0;
      update();
    },
    list: () => waypoints.map((point) => ({ ...point })),
    count: () => waypoints.length,
    totalDistance,
    render,
  };
}
