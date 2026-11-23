const test = require('node:test');
const assert = require('node:assert/strict');
const {JSDOM} = require('jsdom');
const {mount, watchBadge, cleanDisplayMessage, explainEntry, collapseEntries, sourceLabel} = require('../middleware/core/static/diagnostic_logs.js');

const response = body => ({ok: true, json: async () => body});
const entry = (id, message = 'entry ' + id, level = 'info') => ({
  id, message, level, time: '2026-11-16T01:00:00.000Z', source: 'middleware',
  kind: 'log', details: {sequence: id}
});
const catalog = {sessions: [{id: 'alpha', label: 'Alpha'}, {id: 'beta', label: 'Beta'}], current_session_id: 'alpha'};
const page = (session, entries = [], extra = {}) => ({
  session_id: session, entries, priority_entries: [], incidents: [], summary: [],
  cursor: entries.length ? Math.max(...entries.map(e => e.id)) : 0, has_more: false, ...extra
});
const settle = () => new Promise(resolve => setImmediate(resolve));

function harness(onLogs) {
  const dom = new JSDOM('<article id="logs"></article>');
  const doc = dom.window.document;
  Object.defineProperty(doc, 'hidden', {get: () => false});
  const root = doc.querySelector('#logs');
  root.getClientRects = () => [{}];
  const requests = [];
  const timers = new Map();
  let timerId = 0;
  const panel = mount(root, {
    fetch: (url, options) => {
      if (url === '/api/logs/sessions') return Promise.resolve(response(catalog));
      const parsed = new URL(url, 'http://localhost');
      requests.push(parsed);
      return onLogs(parsed.searchParams, options, requests.length);
    },
    setTimeout: (fn, ms) => { const id = ++timerId; timers.set(id, {fn, ms}); return id; },
    clearTimeout: id => timers.delete(id)
  });
  function change(node, value) {
    if (node.type === 'checkbox') node.checked = value;
    else node.value = value;
    node.dispatchEvent(new dom.window.Event('change'));
  }
  const automatic = () => root.querySelectorAll('input[type=checkbox]')[1];
  return {root, panel, requests, timers, change, automatic,
    close() { panel.dispose(); dom.window.close(); }};
}

test('session switch during an old request starts the new request with automatic refresh off', async () => {
  let resolveOld;
  const h = harness((params, options, count) => {
    // Deliberately ignore AbortController: generation checks must still fence
    // stale evidence even if the browser resolves an already cancelled fetch.
    if (count === 1) return new Promise(resolve => { resolveOld = resolve; });
    return Promise.resolve(response(page(params.get('session'), [entry(2, 'new session')])));
  });
  try {
    await settle();
    h.change(h.automatic(), false);
    h.change(h.root.querySelector('select'), 'beta');
    resolveOld(response(page('alpha', [entry(999, 'stale session')])));
    await settle();
    assert.equal(h.requests.length, 2);
    assert.equal(h.requests[1].searchParams.get('session'), 'beta');
    assert.equal(h.panel.model.session, 'beta');
    assert.equal(h.panel.model.cursor, 2);
    assert.deepEqual(h.panel.model.entries.map(e => e.message), ['new session']);
    assert.equal(h.root.textContent.includes('stale session'), false);
    assert.equal([...h.timers.values()].some(t => t.ms === 2000), false);
  } finally { h.close(); }
});

test('filter switch during an aborted request still fetches once with automatic refresh off', async () => {
  const h = harness((params, {signal}, count) => {
    if (count === 1) return new Promise((resolve, reject) => {
      signal.addEventListener('abort', () => reject(Object.assign(new Error('aborted'), {name: 'AbortError'})));
    });
    return Promise.resolve(response(page(params.get('session'), [entry(3, 'filtered error', 'error')])));
  });
  try {
    await settle();
    h.change(h.automatic(), false);
    h.change(h.root.querySelector('[aria-label="日志等级"]'), 'error');
    await settle();
    assert.equal(h.requests.length, 2);
    assert.equal(h.requests[1].searchParams.get('level'), 'error');
    assert.equal(h.root.querySelector('.diagnostic-log-message').textContent, 'filtered error');
    assert.equal([...h.timers.values()].some(t => t.ms === 2000), false);
  } finally { h.close(); }
});

test('poll preserves expanded fields and scroll positions but isolates sessions', async () => {
  const info = entry(1, 'timeout', 'error');
  const h = harness(params => Promise.resolve(response(page(params.get('session'), [info]))));
  try {
    await settle();
    const list = h.root.querySelector('.diagnostic-log-list');
    const details = [...h.root.querySelectorAll('details[data-diagnostic-key]')];
    assert.equal(details.length, 1);
    details.forEach(node => { node.open = true; });
    list.scrollTop = 123;
    await h.panel.refresh();
    assert.ok([...h.root.querySelectorAll('details[data-diagnostic-key]')].every(node => node.open));
    assert.equal(list.scrollTop, 123);
    assert.equal(h.requests[1].searchParams.get('after'), '1');
    h.change(h.root.querySelector('select'), 'beta');
    await settle();
    assert.ok([...h.root.querySelectorAll('details[data-diagnostic-key]')].every(node => !node.open));
    assert.equal(list.scrollTop, 0);
  } finally { h.close(); }
});

test('latest error preview is visible before paging without advancing the chronological cursor', async () => {
  const recentError = entry(900, 'latest error', 'error');
  const normal = Array.from({length: 200}, (_, i) => entry(i + 1));
  const h = harness((params, options, count) => Promise.resolve(response(page('alpha',
    count === 1 ? normal : [entry(201)], {cursor: count === 1 ? 200 : 201,
      priority_entries: [recentError], has_more: true}))));
  try {
    await settle();
    let rows = h.root.querySelectorAll('.diagnostic-log-list > li');
    assert.equal(rows[0].dataset.logId, '900');
    assert.equal(rows.length, 201);
    assert.equal(h.panel.model.cursor, 200);
    await h.panel.refresh();
    assert.equal(h.requests[1].searchParams.get('after'), '200');
    assert.equal(h.panel.model.cursor, 201);
    h.change(h.root.querySelectorAll('input[type=checkbox]')[0], false);
    rows = h.root.querySelectorAll('.diagnostic-log-list > li');
    assert.equal(rows[0].dataset.logId, '201');
    assert.equal([...rows].some(row => row.dataset.logId === '900'), false);
  } finally { h.close(); }
});

test('log HTML stays literal text, including priority previews', async () => {
  const payload = '<img src=x onerror="globalThis.stolen=true">';
  const bad = {...entry(10, payload, 'error'), source: payload, file: payload, details: {message: payload}};
  const h = harness(() => Promise.resolve(response(page('alpha', [], {
    priority_entries: [bad],
  }))));
  try {
    await settle();
    assert.equal(h.root.querySelectorAll('img,script,[onerror]').length, 0);
    assert.ok(h.root.querySelector('.diagnostic-log-message').textContent.includes(payload));
    assert.equal(globalThis.stolen, undefined);
  } finally { h.close(); }
});

test('first entry request failures remain visible after resolving the initial session', async () => {
  const h = harness(() => Promise.resolve({ok: false, status: 503}));
  try {
    await settle();
    const error = h.root.querySelector('.diagnostic-log-error');
    assert.equal(error.hidden, false);
    assert.match(error.textContent, /HTTP 503/);
  } finally { h.close(); }
});

test('compact layout keeps only overview cards and lower list with pattern-based duplicate folding', async () => {
  const marvin = (id, ts) => ({
    id, level: 'warning', source: 'hardware/upper_body', subsystem: 'hardware', kind: 'saved_log',
    time: `2026-11-23T06:33:${String(ts).padStart(2, '0')}.000Z`,
    message: `[hc_tj_marvin_bridge_node-1] [WARN] [17954155${ts}.119133303] [hc_tj_marvin_bridge]: Marvin SDK 上一帧仍待发送，本帧不追加，等待下一条实时目标`
  });
  const vrErr1 = {
    id: 10, level: 'error', source: 'vr_gateway', subsystem: 'middleware', kind: 'vr_input_fault',
    time: '2026-11-23T06:32:03.000Z', message: 'no VR pose data for 612 ms'
  };
  const vrErr2 = {
    id: 12, level: 'error', source: 'vr_gateway', subsystem: 'middleware', kind: 'vr_input_fault',
    time: '2026-11-23T06:32:05.000Z', message: 'no VR pose data for 645 ms'
  };
  assert.equal(sourceLabel('hardware/upper_body'), '双臂驱动 (Marvin)');
  assert.equal(cleanDisplayMessage(marvin(1, 10).message).text, 'Marvin SDK 上一帧仍待发送，本帧不追加，等待下一条实时目标');
  assert.match(explainEntry(vrErr1).title, /VR 位姿数据断流/);

  // Same pattern with different timeout ms collapses cleanly into 1 row.
  const foldedVr = collapseEntries([vrErr2, vrErr1]);
  assert.equal(foldedVr.length, 1);
  assert.equal(foldedVr[0].count, 2);
  assert.equal(foldedVr[0].entry.id, 12);

  const h = harness(params => Promise.resolve(response(page(params.get('session'), [marvin(3, 12), marvin(2, 11), marvin(1, 10)], {
    priority_entries: [vrErr2, vrErr1],
    incidents: [{id: 'i2', title: 'no VR pose data for 645 ms', root_message: 'no VR pose data for 645 ms', source: 'vr_gateway', subsystem: 'middleware', time: vrErr2.time, relation: 'explicit', root_cause_id: 12, chain: [vrErr2]}],
    summary: [
      {subsystem: 'middleware', state: 'error', errors: 2, warnings: 0, recent_errors: 2, total: 10, latest_error: vrErr2, sources: {vr_gateway: {errors: 2, warnings: 0}}},
      {subsystem: 'hardware', state: 'warning', errors: 0, warnings: 3, recent_errors: 0, total: 3, latest_error: null, latest_warning: marvin(3, 12), sources: {'hardware/upper_body': {errors: 0, warnings: 3}}},
    ]
  }))));
  try {
    await settle();
    assert.equal(h.requests[0].searchParams.get('level'), 'warning');
    // No bloated alarm / diagnosis / incidents sections above the tabs.
    assert.equal(h.root.querySelector('.diagnostic-log-alarm'), null);
    assert.equal(h.root.querySelector('.diagnostic-log-diagnosis'), null);
    assert.equal(h.root.querySelector('.diagnostic-log-incidents'), null);
    // Overview cards remain visible.
    assert.equal(h.root.querySelectorAll('.diagnostic-log-card').length, 2);
    // 2 VR pose timeouts collapse to 1 row + 3 Marvin warnings collapse to 1 row = 2 rows.
    const rows = h.root.querySelectorAll('.diagnostic-log-list > li');
    assert.equal(rows.length, 2);
    // Clicking a category card filters by subsystem.
    h.root.querySelector('.diagnostic-log-card').click();
    await settle();
    assert.equal(h.requests[1].searchParams.get('subsystem'), 'middleware');
  } finally { h.close(); }
});

test('inference faults, boundary rejections, grip return timeout, and velocity clipping translate cleanly', () => {
  assert.equal(sourceLabel('inference/holobrain_hc_tj_async_client'), '推理客户端 (HoloBrain)');
  const jump = explainEntry({
    kind: 'inference_fault',
    message: 'inference/protocol failure: first action jumps 0.021554 rad from feedback; limit is 0.020000 rad; joint=Joint7_R target_rad=-0.107931 feedback_rad=-0.086377'
  });
  assert.match(jump.title, /推理首帧动作跳变超限.*Joint7_R.*0\.022 rad > 上限 0\.020 rad/);

  const rej = explainEntry({
    kind: 'control_diagnostic',
    message: 'control command rejected (4x): target too far from measured position: Joint3_R target=1.200000 measured=0.700000 error=0.500000 limit=0.300000'
  });
  assert.match(rej.title, /目标关节角偏离当前位置过大.*Joint3_R.*0\.500 rad > 上限 0\.300 rad/);

  const clip = explainEntry({
    kind: 'control_clip',
    message: 'arm velocity clipped (3x): Joint7_R requested=-0.0660 sent=-0.0676 delta=+0.0016 rad (limit=1 rad/s)'
  });
  assert.match(clip.title, /中间件双臂速度限幅已截断动作.*Joint7_R.*1 rad\/s/);

  const grip = explainEntry({
    kind: 'revoked',
    message: 'Grip return timed out waiting for the first model arm command'
  });
  assert.match(grip.title, /DAgger 退回自动模式超时/);
});

test('clearing focus errors clears overview cards and sidebar badge while keeping entries in the lower list', async () => {
  const err1 = {
    id: 10, level: 'error', source: 'inference/holobrain_hc_tj_async_client', subsystem: 'inference',
    kind: 'inference_fault', time: '2026-11-23T07:01:00.000Z',
    message: 'inference/protocol failure: first action jumps 0.021554 rad from feedback; limit is 0.020000 rad; joint=Joint7_R'
  };
  const err2 = {
    id: 12, level: 'error', source: 'vr_gateway', subsystem: 'middleware',
    kind: 'vr_input_fault', time: '2026-11-23T07:01:05.000Z',
    message: 'head tracking invalid'
  };
  const err3 = {
    id: 25, level: 'error', source: 'inference/holobrain_hc_tj_async_client', subsystem: 'inference',
    kind: 'inference_fault', time: '2026-11-23T07:02:00.000Z',
    message: 'inference/protocol failure: first action jumps 0.050000 rad from feedback; limit is 0.020000 rad; joint=Joint3_R'
  };

  let phase = 1;
  const currentPayload = () => phase === 1
    ? page('alpha', [err2, err1], {
        priority_entries: [err2, err1],
        summary: [
          {subsystem: 'inference', state: 'error', errors: 1, warnings: 0, recent_errors: 1, total: 1, latest_error: err1, sources: {[err1.source]: {errors: 1, warnings: 0}}},
          {subsystem: 'middleware', state: 'error', errors: 1, warnings: 0, recent_errors: 1, total: 1, latest_error: err2, sources: {vr_gateway: {errors: 1, warnings: 0}}},
        ],
      })
    : page('alpha', [err3], {
        cursor: 25,
        priority_entries: [err3, err2, err1],
        summary: [
          {subsystem: 'inference', state: 'error', errors: 2, warnings: 0, recent_errors: 2, total: 2, latest_error: err3, sources: {[err1.source]: {errors: 2, warnings: 0}}},
          {subsystem: 'middleware', state: 'error', errors: 1, warnings: 0, recent_errors: 1, total: 1, latest_error: err2, sources: {vr_gateway: {errors: 1, warnings: 0}}},
        ],
      });

  const h = harness(() => Promise.resolve(response(currentPayload())));
  const doc = h.root.ownerDocument;
  const navLink = doc.createElement('a');
  doc.body.append(navLink);
  const watcher = watchBadge(navLink, {fetch: () => Promise.resolve(response(currentPayload())), interval: 60000});
  try {
    await settle();
    assert.equal(h.root.querySelectorAll('.diagnostic-log-card-error').length, 2);
    assert.equal(watcher.badge.hidden, false);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-list > li').length, 2);

    // Per-card clear: clears ONLY the first card's error highlight without filtering; both entries stay below.
    h.root.querySelector('.diagnostic-log-card-clear').click();
    assert.equal(h.root.querySelectorAll('.diagnostic-log-card-error').length, 1);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-card-cleared').length, 1);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-list > li').length, 2);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-entry-cleared').length, 1);

    // One-click clear all: all cards clear, sidebar badge hides, both entries remain in lower list.
    h.root.querySelector('.diagnostic-log-clear-btn').click();
    assert.equal(h.root.querySelectorAll('.diagnostic-log-card-error').length, 0);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-card-cleared').length, 2);
    assert.equal(watcher.badge.hidden, true);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-list > li').length, 2);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-entry-cleared').length, 2);

    // New error (id=25 > 12) re-highlights ONLY inference card and shows un-cleared above cleared entries below.
    phase = 2;
    await h.panel.refresh();
    await settle();
    assert.equal(h.root.querySelectorAll('.diagnostic-log-card-error').length, 1);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-card-cleared').length, 1);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-list > li').length, 3);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-entry-cleared').length, 2);

    // Restore re-highlights all cards and removes cleared tags below.
    h.root.querySelector('.diagnostic-log-restore-btn').click();
    assert.equal(h.root.querySelectorAll('.diagnostic-log-card-error').length, 2);
    assert.equal(h.root.querySelectorAll('.diagnostic-log-entry-cleared').length, 0);
  } finally {
    watcher.dispose();
    h.close();
  }
});
