// SPDX-License-Identifier: MIT
// 数値の表示形式。UI 側で単位と桁を揃えるため、整形はここに集める。
'use strict';

/// -0 を 0 にする。度や座標で "-0°" "-0.00" と出ると、符号に意味があるように
/// 見えてしまう (実測: 起動直後の姿勢が "-0°" と表示された)。
function fixed(value, digits) {
  const factor = 10 ** digits;
  const rounded = Math.round(value * factor) / factor;
  return (Object.is(rounded, -0) ? 0 : rounded).toFixed(digits);
}

export function meters(value, digits = 1) {
  if (!Number.isFinite(value)) return '—';
  return fixed(value, digits);
}

export function pixels(value) {
  if (!Number.isFinite(value)) return '—';
  return `${Math.round(value)}`;
}

export function percent(value) {
  if (!Number.isFinite(value)) return '—';
  return `${Math.round(value * 100)}%`;
}

export function degrees(radians) {
  if (!Number.isFinite(radians)) return '—';
  const rounded = Math.round((radians * 180) / Math.PI);
  const deg = Object.is(rounded, -0) ? 0 : rounded;
  return `${deg > 0 ? '+' : ''}${deg}°`;
}

/// 経過時間。1 秒未満は ms、それ以上は s で出す (桁が飛ぶと読み取りにくい)。
export function age(milliseconds) {
  if (!Number.isFinite(milliseconds)) return '—';
  if (milliseconds < 1000) return `${Math.round(milliseconds)} ms`;
  return `${(milliseconds / 1000).toFixed(1)} s`;
}

export function clock(date = new Date()) {
  const pad = (value) => String(value).padStart(2, '0');
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
}

export function position(x, y) {
  if (!Number.isFinite(x) || !Number.isFinite(y)) return '—';
  return `${fixed(x, 2)}, ${fixed(y, 2)}`;
}
