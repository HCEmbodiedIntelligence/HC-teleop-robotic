const test = require('node:test');
const assert = require('node:assert/strict');
const {JSDOM} = require('jsdom');
const {mount} = require('../middleware/core/static/diagnostic_logs.js');

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
    return Promise.resolve(response(page(params.get('session'), [entry(2, 'new session')] )));
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

test('poll preserves expanded fault chain, fields, and scroll positions but isolates sessions', async () => {
  const info = entry(1, 'timeout', 'error');
  const incident = {id: 'fault-1', title: 'Fault', relation: 'explicit', root_message: 'timeout', chain: [info]};
  const h = harness(params => Promise.resolve(response(page(params.get('session'), [info], {incidents: [incident]}))));
  try {
    await settle();
    const list = h.root.querySelector('.diagnostic-log-list');
    const chain = h.root.querySelector('.diagnostic-log-chain');
    const details = [...h.root.querySelectorAll('details[data-diagnostic-key]')];
    assert.equal(details.length, 3);
    details.forEach(node => { node.open = true; });
    list.scrollTop = 123;
    chain.scrollTop = 77;
    await h.panel.refresh();
    assert.ok([...h.root.querySelectorAll('details[data-diagnostic-key]')].every(node => node.open));
    assert.equal(list.scrollTop, 123);
    assert.equal(h.root.querySelector('.diagnostic-log-chain').scrollTop, 77);
    assert.equal(h.requests[1].searchParams.get('after'), '1');
    h.change(h.root.querySelector('select'), 'beta');
    await settle();
    assert.ok([...h.root.querySelectorAll('details[data-diagnostic-key]')].every(node => !node.open));
    assert.equal(list.scrollTop, 0);
    assert.equal(h.root.querySelector('.diagnostic-log-chain').scrollTop, 0);
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

test('log and incident HTML stays literal text, including priority previews', async () => {
  const payload = '<img src=x onerror="globalThis.stolen=true">';
  const bad = {...entry(10, payload, 'error'), source: payload, file: payload, details: {message: payload}};
  const h = harness(() => Promise.resolve(response(page('alpha', [], {
    priority_entries: [bad], incidents: [{id: 'x', title: payload, relation: 'unknown', root_message: payload, chain: [bad]}]
  }))));
  try {
    await settle();
    assert.equal(h.root.querySelectorAll('img,script,[onerror]').length, 0);
    assert.ok(h.root.querySelector('.diagnostic-log-message').textContent.includes(payload));
    assert.ok(h.root.querySelector('.diagnostic-log-incident').textContent.includes(payload));
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
