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
  let offline = false, available = true, savedSelection = { mode: 'local' };
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
    if (operation === 'selection') savedSelection = JSON.parse(options.body);
    const body = ['status','verify','connect'].includes(operation) ? { connection, auto_upload: true, recent: null, recording_selection: savedSelection }
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
    assert.equal((await w.platformRecordingSelection()).mode, 'local');
    assert.equal(doc.getElementById('platformArm'), null);
    const local = doc.getElementById('platformLocal');
    local.checked = false; await local.onchange();
    assert.equal(local.checked, false);
    assert.equal(doc.activeElement, doc.getElementById('platformTasks'));
    assert.match(doc.getElementById('platformMessage').textContent, /请选择采集任务/);
    assert.equal(requests.some(r => r.url.endsWith('/selection')), false);
    w.platformRecordingStatus({ recording: false, recording_selection: savedSelection });
    timers.get(30000)(); await settle();
    assert.equal(local.checked, false);
    assert.match(doc.getElementById('platformMessage').textContent, /请选择采集任务/);
    assert.equal((await w.platformRecordingSelection()).mode, 'local');
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
    assert.match(doc.getElementById('platformMessage').textContent, /暂不关联平台任务/);
    const before = requests.length;
    Object.defineProperty(doc, 'hidden', { configurable: true, value: true });
    timers.get(30000)(); await settle(); assert.equal(requests.length, before);
    w.platformRecordingStatus({ recording: true, task_recording: { binding: { task } } });
    assert.equal(doc.getElementById('platformTasks').disabled, true);
    assert.match(doc.getElementById('platformActive').textContent, /Task A/);
    w.platformRecordingStatus({ recording: false }); w.platformRecordingStopped();
    doc.getElementById('platformLocal').checked = true; doc.getElementById('platformLocal').onchange();
    assert.equal((await w.platformRecordingSelection()).mode, 'local');
    assert.deepEqual(savedSelection, { mode: 'local' });
    local.checked = false; await local.onchange();
    assert.equal(local.checked, false);
    offline = false; available = true;
    doc.getElementById('platformRefresh').onclick(); await settle();
    doc.getElementById('platformTasks').value = 'A';
    await doc.getElementById('platformTasks').onchange();
    assert.equal(local.checked, false);
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
    const body = ['status', 'verify'].includes(op) ? { connection, recording_selection: saved }
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
    doc.getElementById('platformLocal').checked = true; doc.getElementById('platformLocal').onchange();
    let readDone = false;
    const read = w.platformRecordingSelection().then(value => { readDone = true; return value; });
    await settle();
    assert.equal(pending.length, 1); assert.equal(readDone, false);
    pending.shift()(); await settle();
    assert.equal(pending.length, 1); assert.equal(readDone, false);
    pending.shift()();
    assert.equal((await read).mode, 'local');
    assert.deepEqual(saved, { mode: 'local' });
    const local = doc.getElementById('platformLocal');
    local.checked = false; await local.onchange();
    // A delayed local save must not undo a later request to choose a task.
    local.checked = true; const saveLocal = local.onchange();
    local.checked = false; const chooseTask = local.onchange();
    await settle();
    assert.equal(pending.length, 1);
    pending.shift()(); await saveLocal; await chooseTask;
    assert.equal(local.checked, false);
    assert.match(doc.getElementById('platformMessage').textContent, /请选择采集任务/);
    assert.deepEqual(saved, { mode: 'local' });
    fail = true;
    doc.getElementById('platformTasks').value = 'A'; await doc.getElementById('platformTasks').onchange();
    await assert.rejects(w.platformRecordingSelection(), /任务验证失败/);
    assert.match(doc.getElementById('platformMessage').textContent, /手柄仍使用上次保存的设置/);
    assert.deepEqual(saved, { mode: 'local' });
  } finally { dom.window.close(); }
});
