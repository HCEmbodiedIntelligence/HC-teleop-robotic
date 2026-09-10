const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

function page(fail = false) {
  const elements = new Map();
  const get = key => {
    if (!elements.has(key)) elements.set(key, {textContent: '', disabled: true});
    return elements.get(key);
  };
  const calls = [];
  let state = 'stopped';
  const inputs = ['headless', 'with_control', 'debug_joints'].map(key => ({
    dataset: {path: `simulation.${key}`}, checked: key === 'headless',
  }));
  const context = vm.createContext({
    document: {querySelector: get, querySelectorAll: () => inputs},
    FormData: class {}, AbortController, setTimeout, clearTimeout,
    fetch: async (url, options = {}) => {
      calls.push([url, options]);
      if (url === '/api/simulation/options') return {ok: true, json: async () => JSON.parse(options.body)};
      if (url === '/api/simulation/start') {
        if (fail) return {ok: false, text: async () => '没有可用的桌面 DISPLAY'};
        state = 'running';
      }
      return {ok: true, json: async () => ({state, active_profile:'x1', profile_id:'x1', logs:[], available:true})};
    },
  });
  const source = fs.readFileSync(path.join(__dirname, '../middleware/core/static/app.js'), 'utf8');
  vm.runInContext(source.replace(/init\(\);\s*$/, ''), context);
  vm.runInContext('config={}; bindSimulationControls();', context);
  return {context, get, calls};
}

test('the actual start button saves options and sends the launch request', async () => {
  const {context, get, calls} = page();
  await vm.runInContext('loadSimulation()', context);
  assert.equal(get('#startSimulation').disabled, false);
  const pending = get('#startSimulation').onclick();
  assert.equal(get('#startSimulation').disabled, true);
  assert.match(get('#simulationState').textContent, /正在提交/);
  await pending;
  assert.deepEqual(calls.map(([url]) => url), [
    '/api/simulation', '/api/simulation/options', '/api/simulation/start', '/api/simulation',
  ]);
  assert.equal(JSON.parse(calls[1][1].body).headless, true);
  assert.equal(get('#simulationState').textContent, '运行中');
  assert.equal(get('#stopSimulation').disabled, false);
  vm.runInContext('clearTimeout(toast.timer)', context);
});

test('launch rejection remains visible after status polling and permits retry', async () => {
  const {context, get} = page(true);
  await get('#startSimulation').onclick();
  await vm.runInContext('loadSimulation()', context);
  assert.match(get('#simulationError').textContent, /DISPLAY/);
  assert.equal(get('#startSimulation').disabled, false);
  vm.runInContext('clearTimeout(toast.timer)', context);
});
