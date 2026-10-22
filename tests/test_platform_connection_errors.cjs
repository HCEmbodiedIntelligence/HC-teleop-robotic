const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { JSDOM } = require('jsdom');

const root = path.resolve(__dirname, '../middleware/core/static');
const settle = () => new Promise(resolve => setTimeout(resolve, 10));

function page() {
  const dom = new JSDOM(fs.readFileSync(path.join(root, 'index.html'), 'utf8'), {
    url: 'http://robot.example/', runScripts: 'outside-only', pretendToBeVisual: true,
  });
  const w = dom.window, doc = w.document, timers = new Map(), timeouts = new Map();
  let nextTimeout = 0;
  const connection = { id: 'original', robot_name: 'Robot', platform_name: 'Platform', verified: true };
  const control = { connect: null, verify: null };
  w.setInterval = (fn, delay) => { timers.set(delay, fn); return delay; };
  w.clearInterval = delay => timers.delete(delay);
  w.setTimeout = (fn, delay) => { const id = ++nextTimeout; timeouts.set(id, { fn, delay }); return id; };
  w.clearTimeout = id => timeouts.delete(id);
  w.fetch = async (url, options) => {
    const op = url.replace('/api/platform/', '').split('?')[0];
    if (control[op]) return control[op](options);
    const body = ['status', 'verify', 'connect'].includes(op)
      ? { connection, recording_selection: { mode: 'local' } }
      : op === 'tasks' ? { items: [], total: 0, projects: [] } : { items: [], jobs: [] };
    return { ok: true, status: 200, json: async () => body };
  };
  return { dom, w, doc, timers, timeouts, control, start: () => w.eval(fs.readFileSync(path.join(root, 'platform_panel.js'), 'utf8')) };
}

for (const failure of [
  { code: 'NETWORK_TIMEOUT', reason: '连接平台超时，请检查网络', retryable: true },
  { code: 'NETWORK_DNS_FAILED', reason: '无法解析平台域名', retryable: true },
  { code: 'CONNECTION_TICKET_EXPIRED', reason: '连接链接已过期，请重新生成', retryable: false },
  { code: 'CONNECTION_TICKET_USED', reason: '连接链接已被使用，请重新生成', retryable: false },
  { code: 'ROBOT_CREDENTIAL_REVOKED', reason: '上传凭据已撤销，请使用新连接链接', retryable: false },
]) {
  test(`${failure.code} remains visible through background refresh and clears on success`, async () => {
    const p = page();
    try {
      p.start(); await p.w.platformRecordingSelection();
      p.control.connect = async () => ({ ok: false, status: 409, json: async () => failure });
      p.doc.getElementById('platformLink').value = 'http://platform/#robot-connect=secret';
      await p.doc.getElementById('platformConnect').onclick();
      const notice = p.doc.getElementById('platformConnectMessage');
      assert.ok(notice.textContent.includes(failure.reason));
      assert.ok(notice.textContent.includes(failure.code));
      assert.equal(notice.textContent.includes('同一链接重试'), failure.retryable);
      assert.match(notice.textContent, /原连接设置已保留/);
      assert.equal(p.doc.getElementById('platformLink').value, '');
      assert.ok(!notice.textContent.includes('secret'));
      const visible = notice.textContent;
      p.timers.get(30000)(); await settle();
      assert.equal(notice.textContent, visible);
      p.control.connect = null;
      await p.doc.getElementById('platformConnect').onclick();
      assert.match(notice.textContent, /连接成功/);
      assert.equal(notice.classList.contains('error'), false);
    } finally { p.dom.window.close(); }
  });
}

test('browser network failure and non-JSON server response have readable messages', async () => {
  const p = page();
  try {
    p.start(); await p.w.platformRecordingSelection();
    p.control.connect = async () => { throw new TypeError('Failed to fetch'); };
    await p.doc.getElementById('platformConnect').onclick();
    assert.match(p.doc.getElementById('platformConnectMessage').textContent, /浏览器无法连接采集端/);
    p.control.connect = async () => ({ ok: false, status: 502, json: async () => { throw new SyntaxError('<html>private response</html>'); } });
    await p.doc.getElementById('platformConnect').onclick();
    const text = p.doc.getElementById('platformConnectMessage').textContent;
    assert.match(text, /采集端返回了无效响应（HTTP 502）/);
    assert.ok(!text.includes('private response'));
  } finally { p.dom.window.close(); }
});

test('revoked saved connection is visible on page load', async () => {
  const p = page();
  try {
    p.control.verify = async () => ({ ok: false, status: 409,
      json: async () => ({ code: 'ROBOT_CREDENTIAL_REVOKED', reason: '上传凭据已撤销，请重新生成连接链接' }) });
    p.start(); await p.w.platformRecordingSelection();
    assert.match(p.doc.getElementById('platformConnectMessage').textContent, /当前连接验证失败.*上传凭据已撤销/);
  } finally { p.dom.window.close(); }
});

test('recording updates cannot re-enable or duplicate a pending connection', async () => {
  const p = page();
  try {
    p.start(); await p.w.platformRecordingSelection();
    let finish, calls = 0;
    p.control.connect = () => { calls++; return new Promise(resolve => { finish = resolve; }); };
    const button = p.doc.getElementById('platformConnect');
    const pending = button.onclick(); await settle();
    p.w.platformRecordingStatus({ recording: false });
    assert.equal(button.disabled, true);
    await button.onclick();
    assert.equal(calls, 1);
    finish({ ok: false, status: 409, json: async () => ({ code: 'NETWORK_TIMEOUT', reason: '连接平台超时', retryable: true }) });
    await pending;
    assert.equal(button.disabled, false);
  } finally { p.dom.window.close(); }
});

for (const stage of ['headers', 'body']) {
  test(`a stalled ${stage} response times out, releases the connect button and allows retry`, async () => {
    const p = page();
    try {
      p.start(); await p.w.platformRecordingSelection();
      p.control.connect = options => {
        const stalled = new Promise((resolve, reject) => {
          options.signal.addEventListener('abort', () => reject(new p.w.DOMException('aborted', 'AbortError')));
        });
        return stage === 'headers' ? stalled : { ok: true, status: 200, json: () => stalled };
      };
      const button = p.doc.getElementById('platformConnect');
      const pending = button.onclick(); await settle();
      assert.equal(button.disabled, true);
      const deadline = [...p.timeouts.values()].find(timer => timer.delay === 20000);
      assert.ok(deadline);
      deadline.fn();
      await pending;
      assert.match(p.doc.getElementById('platformConnectMessage').textContent, /请求超时.*20 秒/);
      assert.equal(button.disabled, false);
      assert.equal(p.timeouts.size, 0);
      p.control.connect = null;
      await button.onclick();
      assert.match(p.doc.getElementById('platformConnectMessage').textContent, /连接成功/);
    } finally { p.dom.window.close(); }
  });
}

test('unavailable connection service is shown beside the connection button on page load', async () => {
  const p = page();
  try {
    p.control.status = async () => ({ ok: false, status: 503, json: async () => ({
      code: 'PLATFORM_UNAVAILABLE', reason: '平台连接服务未启动，请重启采集端服务', retryable: true,
    }) });
    p.start(); await p.w.platformRecordingSelection();
    assert.match(p.doc.getElementById('platformConnectMessage').textContent, /重启采集端服务/);
    assert.equal(p.doc.getElementById('platformConnection').textContent, '平台连接状态读取失败');
    assert.equal(p.timeouts.size, 0);
  } finally { p.dom.window.close(); }
});

for (const phase of ['status', 'verify']) {
  for (const failed of [false, true]) {
    test(`late initial ${phase} ${failed ? 'failure' : 'success'} cannot overwrite a new connection`, async () => {
      const p = page();
      try {
        let finish;
        p.control[phase] = () => new Promise(resolve => { finish = resolve; });
        p.start(); await settle();
        assert.ok(finish);
        p.control.connect = async () => ({ ok: true, status: 200, json: async () => ({
          connection: { id:'replacement', robot_name:'Current Robot', platform_name:'Platform', verified:true },
          recording_selection: { mode:'local' },
        }) });
        await p.doc.getElementById('platformConnect').onclick();
        finish({ ok:!failed, status:failed ? 401 : 200, json:async () => failed
          ? { code:'ROBOT_CREDENTIAL_INVALID', reason:'旧凭据失效' }
          : { connection:{ id:'old', robot_name:'Old Robot', platform_name:'Platform' }, recording_selection:{ mode:'local' } } });
        await p.w.platformRecordingSelection();
        assert.match(p.doc.getElementById('platformConnectMessage').textContent, /连接成功/);
        assert.match(p.doc.getElementById('platformConnection').textContent, /Current Robot/);
      } finally { p.dom.window.close(); }
    });
  }
}
