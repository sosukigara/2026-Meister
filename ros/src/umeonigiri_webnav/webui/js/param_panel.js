// SPDX-License-Identifier: MIT
// ROS パラメータの一覧・編集。値の型はノードが持っている既存の定義に合わせる
// (画面から型を決めない)。一覧は開いたときと「再読込」のときだけ引く: 10 Hz で
// 回っている画面のポーリングに重い経路を足さない。
'use strict';

import { getJson, postJson } from './api.js';

const LIST_TIMEOUT_MS = 15000;
const SET_TIMEOUT_MS = 10000;
const DEFAULT_NODE = '/controller_server';
const ARRAY_HINT = 'JSON 配列 (例: [0.5, 1.0])';

function formatText(value) {
  if (value === null || value === undefined) return '';
  if (Array.isArray(value)) return JSON.stringify(value);
  return String(value);
}

function typeIsNumber(type) {
  return type === 'integer' || type === 'double';
}

function typeIsArray(type) {
  return type === 'byte_array' || type.endsWith('_array');
}

/// 送信する形に変換する。数字は数値、配列は JSON、bool は真偽値。
/// ここで先に弾いて、サーバーの英語の型エラーだけを見せない。
function readValue(param, input) {
  if (param.type === 'bool') {
    return input.value === 'true';
  }
  if (param.type === 'string') {
    return input.value;
  }
  if (param.type === 'integer') {
    const text = input.value.trim();
    if (text === '') throw new Error('値を入力してください');
    const number = Number(text);
    if (!Number.isInteger(number)) throw new Error('整数を入力してください');
    return number;
  }
  if (param.type === 'double') {
    const text = input.value.trim();
    if (text === '') throw new Error('値を入力してください');
    const number = Number(text);
    if (!Number.isFinite(number)) throw new Error('数値を入力してください');
    return number;
  }
  if (typeIsArray(param.type)) {
    let parsed;
    try {
      parsed = JSON.parse(input.value);
    } catch {
      throw new Error('JSON 配列で入力してください');
    }
    if (!Array.isArray(parsed)) throw new Error('JSON 配列で入力してください');
    return parsed;
  }
  throw new Error(`未対応の型です (${param.type})`);
}

function buildEditor(param) {
  if (param.type === 'bool') {
    const select = document.createElement('select');
    select.className = 'param-input';
    for (const [value, label] of [['true', 'true'], ['false', 'false']]) {
      const option = document.createElement('option');
      option.value = value;
      option.textContent = label;
      select.append(option);
    }
    select.value = param.value ? 'true' : 'false';
    return { input: select, read: () => readValue(param, select) };
  }

  const input = document.createElement('input');
  input.className = 'param-input';
  input.type = typeIsNumber(param.type) ? 'number' : 'text';
  if (param.type === 'integer') input.step = '1';
  if (param.type === 'double') input.step = 'any';
  if (typeIsArray(param.type)) input.placeholder = ARRAY_HINT;
  input.value = formatText(param.value);
  input.autocomplete = 'off';
  input.spellcheck = false;
  return { input, read: () => readValue(param, input) };
}

export function createParamPanel({
  dialogEl,
  formEl,
  openEl,
  closeEl,
  nodeEl,
  refreshEl,
  filterEl,
  listEl,
  statusEl,
  onNotice,
}) {
  let rows = [];
  let selectedNode = '';
  let loadingNodes = false;
  let loadSeq = 0;

  function setStatus(text, tone = '') {
    statusEl.textContent = text;
    statusEl.dataset.tone = tone;
  }

  function applyFilter() {
    const query = filterEl.value.trim().toLowerCase();
    let visible = 0;
    for (const row of rows) {
      const match = query === '' || row.name.toLowerCase().includes(query);
      row.rowEl.hidden = !match;
      if (match) visible += 1;
    }
    if (rows.length === 0) return;
    setStatus(
      query === ''
        ? `${rows.length} 個のパラメータ。値はその場で反映されます（再起動不要）`
        : `${visible} / ${rows.length} 個を表示中`,
    );
  }

  function render(parameters) {
    rows = [];
    listEl.replaceChildren();
    if (parameters.length === 0) {
      const empty = document.createElement('p');
      empty.className = 'param-empty';
      empty.textContent = 'このノードはパラメータを持っていません。';
      listEl.append(empty);
      return;
    }

    for (const param of parameters) {
      const rowEl = document.createElement('div');
      rowEl.className = 'param-row';
      rowEl.setAttribute('role', 'listitem');
      rowEl.dataset.type = param.type;

      const ident = document.createElement('div');
      ident.className = 'param-ident';
      const nameEl = document.createElement('code');
      nameEl.className = 'param-name';
      nameEl.textContent = param.name;
      const typeEl = document.createElement('span');
      typeEl.className = 'param-type';
      typeEl.textContent = param.type;
      ident.append(nameEl, typeEl);

      const { input, read } = buildEditor(param);
      input.setAttribute('aria-label', `${param.name} の値`);

      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'btn btn-quiet param-set';
      button.textContent = '設定';
      button.setAttribute('aria-label', `${param.name} を設定`);

      const edit = document.createElement('div');
      edit.className = 'param-edit';
      edit.append(input, button);

      async function submit() {
        if (button.disabled) return;
        let value;
        try {
          value = read();
        } catch (error) {
          rowEl.dataset.state = 'error';
          setStatus(`${param.name}: ${error.message}`, 'error');
          return;
        }
        button.disabled = true;
        rowEl.dataset.state = 'busy';
        try {
          const payload = await postJson(
            '/api/params',
            { node: nodeEl.value, name: param.name, value },
            SET_TIMEOUT_MS,
          );
          const applied = payload.value;
          input.value = formatText(applied);
          if (param.type === 'bool') input.value = applied ? 'true' : 'false';
          rowEl.dataset.state = 'ok';
          setStatus(`${param.name} = ${formatText(applied)} に設定しました`, 'ok');
          onNotice(`${nodeEl.value} ${param.name} = ${formatText(applied)}`, 'info');
        } catch (error) {
          rowEl.dataset.state = 'error';
          setStatus(`設定できません: ${error.message}`, 'error');
          onNotice(`パラメータ設定に失敗: ${error.message}`, 'error');
        } finally {
          button.disabled = false;
          setTimeout(() => {
            if (rowEl.dataset.state !== 'error') rowEl.dataset.state = '';
          }, 1800);
        }
      }

      button.addEventListener('click', submit);
      input.addEventListener('keydown', (event) => {
        if (event.key !== 'Enter' || event.isComposing) return;
        event.preventDefault();
        submit();
      });
      // 選択を変えただけでは送らない。何が送られるか見える状態を保つ。
      input.addEventListener('input', () => {
        if (rowEl.dataset.state !== '') rowEl.dataset.state = '';
      });

      rowEl.append(ident, edit);
      listEl.append(rowEl);
      rows.push({ name: param.name, rowEl });
    }
  }

  async function loadParams() {
    const node = nodeEl.value;
    if (!node) return;
    const seq = ++loadSeq;
    setStatus(`${node} のパラメータを読み込み中…`);
    listEl.replaceChildren();
    try {
      const payload = await getJson(
        `/api/params?node=${encodeURIComponent(node)}`,
        LIST_TIMEOUT_MS,
      );
      if (seq !== loadSeq) return;
      selectedNode = node;
      const parameters = Array.isArray(payload.parameters) ? payload.parameters : [];
      render(parameters);
      applyFilter();
    } catch (error) {
      if (seq !== loadSeq) return;
      rows = [];
      listEl.replaceChildren();
      setStatus(`読み込めません: ${error.message}`, 'error');
    }
  }

  async function loadNodes() {
    if (loadingNodes) return;
    loadingNodes = true;
    setStatus('ノードを一覧しています…');
    try {
      const payload = await getJson('/api/params/nodes', LIST_TIMEOUT_MS);
      const nodes = Array.isArray(payload.nodes) ? payload.nodes : [];
      nodeEl.replaceChildren();
      for (const name of nodes) {
        const option = document.createElement('option');
        option.value = name;
        option.textContent = name;
        nodeEl.append(option);
      }
      if (nodes.length === 0) {
        setStatus('パラメータを持つノードが見つかりません', 'error');
        return;
      }
      if (nodes.includes(selectedNode)) {
        nodeEl.value = selectedNode;
      } else if (nodes.includes(DEFAULT_NODE)) {
        nodeEl.value = DEFAULT_NODE;
      } else {
        nodeEl.value = nodes[0];
      }
      await loadParams();
    } catch (error) {
      setStatus(`ノードを取得できません: ${error.message}`, 'error');
    } finally {
      loadingNodes = false;
    }
  }

  function open() {
    if (dialogEl.open) {
      filterEl.focus();
      return;
    }
    dialogEl.showModal();
    if (nodeEl.options.length === 0) {
      loadNodes();
    } else {
      // 前回開いたままの値は、外から ros2 param set されると古い。開くたびに
      // 読み直して「見えている値 = ノードの値」を保つ。
      if (selectedNode) nodeEl.value = selectedNode;
      loadParams();
    }
    filterEl.focus();
  }

  function close() {
    dialogEl.close();
  }

  openEl.addEventListener('click', open);
  closeEl.addEventListener('click', close);
  nodeEl.addEventListener('change', loadParams);
  refreshEl.addEventListener('click', () => {
    if (nodeEl.options.length === 0) {
      loadNodes();
    } else {
      loadParams();
    }
  });
  filterEl.addEventListener('input', applyFilter);
  formEl.addEventListener('submit', (event) => event.preventDefault());
  // 背景クリックで閉じる。中身のクリックは target が dialog にならない。
  dialogEl.addEventListener('click', (event) => {
    if (event.target === dialogEl) close();
  });
  // Esc はダイアログを閉じるだけにする。window 側の Esc は航行の停止なので、
  // ここで伝播を止めて「設定を閉じたつもりがロボットが止まる」を避ける。
  dialogEl.addEventListener('keydown', (event) => {
    if (event.key === 'Escape') event.stopPropagation();
  });

  return { open, close };
}
