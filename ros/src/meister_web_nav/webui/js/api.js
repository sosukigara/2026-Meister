// SPDX-License-Identifier: MIT
// JSON API の薄いラッパ。失敗を握りつぶさず例外にし、接続状態だけを購読させる。
// 画面側は「取れなかった」と「値が古い」を区別して出す必要がある。
'use strict';

const listeners = new Set();
let consecutiveFailures = 0;
let lastSuccessAt = null;

function notify(online) {
  listeners.forEach((listener) => listener(online));
}

export function onConnectionChange(listener) {
  listeners.add(listener);
}

/// 最後に値が取れてからの秒数。切断中の画面が「いつの値か」を出せるようにする。
export function secondsSinceLastSuccess() {
  if (lastSuccessAt === null) return null;
  return (Date.now() - lastSuccessAt) / 1000;
}

async function request(path, options) {
  try {
    const response = await fetch(path, options);
    if (!response.ok) {
      throw new Error(`${path} が ${response.status} を返しました`);
    }
    const body = await response.json();
    consecutiveFailures = 0;
    lastSuccessAt = Date.now();
    notify(true);
    return body;
  } catch (error) {
    consecutiveFailures += 1;
    if (consecutiveFailures >= 2) notify(false);
    throw error;
  }
}

export function getJson(path) {
  return request(path, { headers: { Accept: 'application/json' } });
}

export function postJson(path, body) {
  return request(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body ?? {}),
  });
}
