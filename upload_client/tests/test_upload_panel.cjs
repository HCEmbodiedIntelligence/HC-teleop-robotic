const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../../middleware/core/static/upload_panel.js'), 'utf8');
const flush = () => new Promise(resolve => setImmediate(resolve));
class Element {
  constructor() {
    this.textContent = ''; this.children = []; this.classList = {add() {}};
    this.elements = { enabled: { checked: false }, auto_enqueue: { checked: false }, namedItem: () => null };
  }
  addEventListener(name, callback) { this['on' + name] = callback; }
  append(child) { this.children.push(child); }
  replaceChildren() { this.children = []; }
}
function harness() {
  const elements = new Map();
  const root = new Element();
  root.querySelector = selector => {
    if (!elements.has(selector)) elements.set(selector, new Element());
    return elements.get(selector);
  };
  const requests = [];
  let tick, cleared = false;
  const context = {window: {}, document: {createElement: () => new Element()}, AbortController,
    fetch: (url, options) => new Promise((resolve, reject) => requests.push({url, options, resolve, reject})),
    setInterval: fn => {tick = fn; return 1;}, clearInterval: () => {cleared = true;},
    FormData: class { constructor(form) { return Object.entries(form.values); } }};
  vm.runInNewContext(source, context);
  const dispose = context.window.mountUploadPanel(root);
  return {elements, requests, dispose, tick: () => tick(), cleared: () => cleared};
}
const state = jobs => ({enabled: true, running: true, auto_enqueue: false, default_binding: null,
  jobs: jobs || [], error: null, scan_errors: {}});
const answer = (request, value, ok = true) => request.resolve({ok, json: async () => value});

test('slow polling coalesces refreshes and never applies after disposal', async () => {
  const h = harness();
  h.tick(); h.tick(); h.elements.get('[data-refresh]').onclick();
  assert.equal(h.requests.length, 1);
  answer(h.requests[0], state()); await flush();
  assert.equal(h.requests.length, 2);
  const before = h.elements.get('[data-status]').textContent;
  h.dispose();
  assert.equal(h.cleared(), true);
  assert.equal(h.requests[1].options.signal.aborted, true);
  answer(h.requests[1], {...state(), enabled: false}); await flush();
  assert.equal(h.elements.get('[data-status]').textContent, before);
  h.tick(); assert.equal(h.requests.length, 2);
});

test('network errors clear after recovery; awaiting slice stays separate from quality', async () => {
  const h = harness(); h.requests[0].reject(new Error('offline')); await flush();
  assert.equal(h.elements.get('[data-error]').textContent, 'offline');
  h.tick();
  answer(h.requests[1], state([{id: 'job', artifact_id: '<script>unsafe</script>', task_id: 'old-task',
    capture_state: 'source_complete', artifact_state: 'ready', upload_state: 'submitted',
    total_bytes: 0, processing_state: 'awaiting_slice', quality_state: 'PENDING'}]));
  await flush();
  assert.equal(h.elements.get('[data-error]').textContent, '');
  const cells = h.elements.get('tbody').children[0].children;
  assert.equal(cells[0].textContent, '<script>unsafe</script> / old-task');
  assert.equal(cells[4].textContent, '100%');
  assert.equal(cells[5].textContent, 'awaiting_slice');
  assert.equal(cells[6].textContent, 'PENDING');
  h.dispose();
});

test('binding form and retry send the correct identities and operation flags', async () => {
  const h = harness();
  answer(h.requests[0], state([{id:'frozen-job',artifact_id:'artifact',task_id:'old-task',
    total_bytes:0,upload_state:'failed'}])); await flush();
  const cells = h.elements.get('tbody').children[0].children;
  assert.equal(cells[4].textContent, '0%');
  cells[7].children[1].onclick();
  assert.equal(h.requests[1].url, '/api/uploads/retry');
  assert.deepEqual(JSON.parse(h.requests[1].options.body), {job_id:'frozen-job',resume:true});
  answer(h.requests[1], {}); await flush(); answer(h.requests[2],state()); await flush();
  const form = h.elements.get('[data-binding]');
  form.values = {platform_base_url:'https://example.invalid',robot_id:'robot',local_robot_id:'local',
    task_id:'new-task',credential_ref:'env:UPLOAD_TOKEN'};
  form.onsubmit({preventDefault(){}});
  assert.equal(h.requests[3].url, '/api/uploads/binding');
  assert.deepEqual(JSON.parse(h.requests[3].options.body),form.values);
  answer(h.requests[3],{code:'BINDING_INVALID',action:'check'},false); await flush();
  assert.match(h.elements.get('[data-error]').textContent,/BINDING_INVALID/);
  h.dispose();
});
