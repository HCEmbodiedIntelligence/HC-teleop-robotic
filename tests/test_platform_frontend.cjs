const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { JSDOM } = require('jsdom');

test('task selection, visible refresh, offline retention and history are explicit', async () => {
  const root = path.resolve(__dirname, '../middleware/core/static');
  const dom = new JSDOM(fs.readFileSync(path.join(root, 'index.html'), 'utf8'), {
    url: 'http://robot.example/', runScripts: 'outside-only', pretendToBeVisual: true,
  });
  const w = dom.window, doc = w.document;
  const requests = [], timers = new Map();
  const connection = { id: 'connection-a', platform_name: 'Platform', robot_name: 'Robot', verified: true };
  let offline = false, available = true, uploadEnabled = true;
  let savedSelection = { mode: 'local' }, uploadFormat = 'mcap';
  const task = { name: 'Task A', collection_task_id: 'A', project_name: 'Project', task_code: '0001', description: 'Collect A' };
  const originalSet = w.Storage.prototype.setItem;
  w.Storage.prototype.setItem = () => { throw new Error('secrets must not enter browser storage'); };
  w.setInterval = (fn, delay) => { timers.set(delay, fn); return delay; };
  w.clearInterval = delay => timers.delete(delay);
  w.HTMLDialogElement.prototype.close = function() { this.open = false; };
  w.HTMLDialogElement.prototype.showModal = function() { this.open = true; };
  w.fetch = async (url, options) => {
    requests.push({ url, options });
    const operation = url.replace('/api/platform/', '').split('?')[0];
    if (operation === 'tasks' && offline) throw new Error('offline');
    if (operation === 'preflight' && !available) return { ok: false, json: async () => ({ reason: '任务已关闭' }) };
    if (operation === 'upload-format') uploadFormat = JSON.parse(options.body).format;
    if (operation === 'enabled') uploadEnabled = JSON.parse(options.body).enabled;
    if (operation === 'selection') savedSelection = JSON.parse(options.body);
    const body = ['status','verify','connect','upload-format','enabled'].includes(operation) ? { connection, uploads_enabled: uploadEnabled, upload_format: uploadFormat, auto_upload: uploadEnabled, recent: null, recording_selection: uploadEnabled ? savedSelection : {mode:'local'}, local_format: 'both' }
      : operation === 'selection' ? { recording_selection: savedSelection }
      : operation === 'tasks' ? { items: available ? [task] : [], total: available ? 1 : 0, projects: [{ id: 'p', name: 'Project' }] }
      : operation === 'preflight' ? { task }
      : operation === 'recordings' ? { items: [], jobs: [] } : { ok: true };
    return { ok: true, json: async () => body };
  };
  const settle = () => new Promise(resolve => setTimeout(resolve, 15));
  try {
    w.eval(fs.readFileSync(path.join(root,'platform_panel.js'),'utf8'));
    await settle();
    assert.equal(requests.filter(r => r.url.startsWith('/api/platform/tasks')).length, 1);
    const enabled = doc.getElementById('platformEnabled');
    assert.equal(enabled.checked, true);
    enabled.checked = false; await enabled.onchange();
    const disabledRequests = requests.length;
    timers.get(5000)(); await settle();
    assert.equal(requests.length, disabledRequests);
    assert.match(doc.getElementById('platformRecordings').textContent, /总开关已关闭/);
    enabled.checked = true; await enabled.onchange();
    const format = doc.getElementById('platformUploadFormat');
    assert.equal(format.value, 'mcap');
    format.value = 'lerobot'; await format.onchange();
    assert.equal(uploadFormat, 'lerobot');
    assert.equal(format.value, 'lerobot');
    w.platformRecordingStatus({recording:true});
    assert.equal(format.disabled, true);
    w.platformRecordingStatus({recording:false});
    assert.equal(format.disabled, false);

    // Test format linkage with local recording format
    w.platformSyncLocalFormat('mcap');
    assert.equal(format.value, 'mcap');
    const lerobotOpt = [...format.options].find(o => o.value === 'lerobot');
    const mcapOpt = [...format.options].find(o => o.value === 'mcap');
    assert.equal(lerobotOpt.disabled, true);
    assert.equal(mcapOpt.disabled, false);

    w.platformSyncLocalFormat('lerobot');
    assert.equal(format.value, 'lerobot');
    assert.equal(mcapOpt.disabled, true);
    assert.equal(lerobotOpt.disabled, false);

    w.platformSyncLocalFormat('both');
    assert.equal(mcapOpt.disabled, false);
    assert.equal(lerobotOpt.disabled, false);

    assert.equal((await w.platformRecordingSelection()).mode, 'local');
    assert.equal(doc.getElementById('platformLocal'), null);

    doc.getElementById('platformTasks').value = 'A';
    doc.getElementById('platformTasks').onchange(); await settle();
    const selected = await w.platformRecordingSelection();
    assert.equal(selected.task_id, 'A'); assert.equal(selected.connection_id, 'connection-a');
    assert.equal((await w.platformRecordingSelection()).request_id, selected.request_id);
    assert.deepEqual(savedSelection, { mode: 'platform', connection_id: 'connection-a', task_id: 'A' });
    available = false; timers.get(30000)(); await settle();
    assert.equal(doc.getElementById('platformTasks').value, 'A');
    assert.match(doc.getElementById('platformMessage').textContent, /任务已关闭/);
    offline = true; timers.get(30000)(); await settle();
    assert.equal(doc.getElementById('platformTasks').value, 'A');
    assert.match(doc.getElementById('platformMessage').textContent, /任务置空/);
    const before = requests.length;
    Object.defineProperty(doc, 'hidden', { configurable: true, value: true });
    timers.get(30000)(); await settle(); assert.equal(requests.length, before);
    w.platformRecordingStatus({ recording: true, task_recording: { binding: { task } } });
    assert.equal(doc.getElementById('platformTasks').disabled, true);
    assert.match(doc.getElementById('platformActive').textContent, /Task A/);
    w.platformRecordingStatus({ recording: false }); w.platformRecordingStopped();
    doc.getElementById('platformTasks').value = ''; doc.getElementById('platformTasks').onchange();
    await settle();
    assert.equal((await w.platformRecordingSelection()).mode, 'local');
    assert.deepEqual(savedSelection, { mode: 'local' });
    offline = false; available = true;
    doc.getElementById('platformRefresh').onclick(); await settle();
    doc.getElementById('platformTasks').value = 'A';
    await doc.getElementById('platformTasks').onchange();
    await settle();
    assert.deepEqual(savedSelection, { mode: 'platform', connection_id: connection.id, task_id: 'A' });
    await w.platformUploadHistory('local-recording', 'offline.mcap');
    assert.equal(doc.getElementById('platformHistoryTask').value, '');
    await doc.getElementById('platformHistoryUpload').onclick();
    assert.match(doc.getElementById('platformHistoryError').textContent, /明确选择/);
    doc.getElementById('platformLink').value = 'http://platform/#robot-connect=secret';
    await doc.getElementById('platformConnect').onclick();
    assert.equal(doc.getElementById('platformLink').value, '');
    assert.equal(JSON.parse(requests.find(r => r.url.endsWith('/connect')).options.body).link, 'http://platform/#robot-connect=secret');
  } finally {
    w.Storage.prototype.setItem = originalSet; dom.window.close();
  }
});

test('saved selection is restored, rapid changes stay ordered, and failed saves block a mismatched browser start', async () => {
  const root = path.resolve(__dirname, '../middleware/core/static');
  const dom = new JSDOM(fs.readFileSync(path.join(root, 'index.html'), 'utf8'), {
    url: 'http://robot.example/', runScripts: 'outside-only', pretendToBeVisual: true,
  });
  const w = dom.window, doc = w.document;
  const connection = { id: 'connection-a', platform_name: 'Platform', robot_name: 'Robot', verified: true };
  const tasks = ['A', 'B'].map(id => ({ collection_task_id: id, name: id, project_name: 'Project', task_code: id }));
  let saved = { mode: 'platform', connection_id: connection.id, task_id: 'A' }, fail = false;
  const pending = [];
  w.setInterval = () => 1; w.clearInterval = () => {};
  w.fetch = async (url, options) => {
    const op = url.replace('/api/platform/', '').split('?')[0];
    const value = options.body ? JSON.parse(options.body) : null;
    if (op === 'selection') {
      if (fail) return { ok: false, json: async () => ({ reason: '任务验证失败' }) };
      await new Promise(resolve => pending.push(resolve));
      saved = value;
    }
    const body = ['status', 'verify'].includes(op) ? { connection, recording_selection: saved, local_format: 'both' }
      : op === 'tasks' ? { items: tasks, projects: [], total: 2 }
      : op === 'preflight' ? { task: tasks.find(task => task.collection_task_id === value.task_id) }
      : op === 'selection' ? { recording_selection: saved } : { items: [], jobs: [] };
    return { ok: true, json: async () => body };
  };
  const settle = () => new Promise(resolve => setTimeout(resolve, 15));
  try {
    w.eval(fs.readFileSync(path.join(root, 'platform_panel.js'), 'utf8'));
    assert.equal((await w.platformRecordingSelection()).task_id, 'A');
    assert.equal(doc.getElementById('platformTasks').value, 'A');
    doc.getElementById('platformTasks').value = 'B'; doc.getElementById('platformTasks').onchange();
    doc.getElementById('platformTasks').value = ''; doc.getElementById('platformTasks').onchange();
    let readDone = false;
    const read = w.platformRecordingSelection().then(value => { readDone = true; return value; });
    await settle();
    assert.equal(pending.length, 1); assert.equal(readDone, false);
    pending.shift()(); await settle();
    assert.equal(pending.length, 1); assert.equal(readDone, false);
    pending.shift()();
    assert.equal((await read).mode, 'local');
    assert.deepEqual(saved, { mode: 'local' });
    fail = true;
    doc.getElementById('platformTasks').value = 'A'; await doc.getElementById('platformTasks').onchange();
    await assert.rejects(w.platformRecordingSelection(), /任务验证失败/);
    assert.match(doc.getElementById('platformMessage').textContent, /手柄仍使用上次保存的设置/);
    assert.deepEqual(saved, { mode: 'local' });
  } finally { dom.window.close(); }
});

test('shared scheme details, pinned revision and independent receipt states remain visible', async () => {
  const root = path.resolve(__dirname, '../middleware/core/static');
  const dom = new JSDOM(fs.readFileSync(path.join(root, 'index.html'), 'utf8'), {
    url: 'http://robot.example/', runScripts: 'outside-only', pretendToBeVisual: true,
  });
  const w = dom.window, doc = w.document;
  const connection = { id: 'connection', platform_name: 'Platform', robot_name: 'Robot', verified: true };
  const task = { collection_task_id: 'A', name: '任务 A', task_code: 'A', project_name: 'Project', selectable: true,
    capture_plan: { revision: 1, scheme: { scheme_id: '视觉方案', version: 2 }, source_format: 'LEROBOT_V3', mapping_version: 3,
      output: { frequency_hz: 15, features: { 'observation.state': { shape: [21], required: true }, camera: { shape: [32, 32, 3], required: false } } } } };
  let saved = { mode: 'local' };
  const requests = [];
  w.setInterval = () => 1; w.clearInterval = () => {};
  w.fetch = async (url, options) => {
    const operation = url.split('/').pop().split('?')[0]; requests.push(operation);
    if (operation === 'selection') saved = JSON.parse(options.body);
    const body = ['status', 'verify'].includes(operation) ? { connection, recording_selection: saved }
      : operation === 'selection' ? { recording_selection: saved }
      : operation === 'preflight' ? { task }
      : operation === 'tasks' ? { items: [task, { ...task, collection_task_id: 'B', selectable: false, reason: 'CAPTURE_MAPPING_UNVERIFIED' }], projects: [], total: 2 }
      : operation === 'recordings' ? { items: [{ recording_id: 'r', path: '/data/synthetic.mcap', state: 'queued', binding: { task } }],
        jobs: [{ id: 'u', local_recording_id: 'r', server_state: 'COMMITTED', processing_state: 'READY', quality_state: 'PASS',
                 data_status: 'READABLE', annotation_status: 'NOT_CONFIGURED', training_status: 'NOT_READY' }] } : {};
    return { ok: true, json: async () => structuredClone(body) };
  };
  const settle = () => new Promise(resolve => setTimeout(resolve, 20));
  try {
    w.eval(fs.readFileSync(path.join(root, 'platform_panel.js'), 'utf8')); await settle();
    doc.getElementById('platformTasks').value = 'A'; await doc.getElementById('platformTasks').onchange();
    assert.match(doc.getElementById('platformDescription').textContent, /视觉方案 v2.*LEROBOT_V3.*15 Hz.*映射 v3.*可选/);
    assert.equal((await w.platformRecordingSelection()).expected_plan_revision, 1);
    task.capture_plan.revision = 2;
    await doc.getElementById('platformRefresh').onclick(); await settle();
    assert.equal((await w.platformRecordingSelection()).expected_plan_revision, 1);
    assert.match(doc.getElementById('platformMessage').textContent, /修订已变化/);
    assert.equal([...doc.getElementById('platformTasks').options].find(o => o.value === 'B').disabled, true);
    assert.match(doc.getElementById('platformTasks').textContent, /CAPTURE_MAPPING_UNVERIFIED/);
    assert.match(doc.getElementById('platformRecordings').textContent, /数据：可读.*标注待配置.*训练交付：未就绪/);
    await doc.getElementById('platformTasks').onchange();
    assert.equal((await w.platformRecordingSelection()).expected_plan_revision, 2);
    assert.equal(requests.some(op => op.includes('prepare')), false);
  } finally { w.close(); }
});
