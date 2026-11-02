const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const { webcrypto } = require('node:crypto');

const source = fs.readFileSync(path.join(__dirname, '../middleware/core/static/app.js'), 'utf8');

function page() {
  const elements = new Map();
  const get = selector => {
    if (!elements.has(selector)) elements.set(selector, {textContent:'', disabled:false});
    return elements.get(selector);
  };
  const buttons = Object.fromEntries(['start_model','enable_dagger','disable_dagger','hold','recover','disable'].map(action =>
    [action, {dataset:{intervention:action}, disabled:false}]));
  const sources = ['vr','motion','exoskeleton'].map(source=>({dataset:{source},disabled:false}));
  const state = {state:'OFF', session_id:'', epoch:0, peers:{motion:'motion-1',teleop:'teleop-1'}, capture:null, dagger_enabled:false, recording_enabled:false};
  const posts = [];
  const errors = [];
  let handlePost;
  const context = vm.createContext({
    document:{querySelector:get, querySelectorAll:selector=>selector==='[data-intervention]'?Object.values(buttons):selector==='.source-switch button'?sources:[]},
    FormData:class {},
    // LAN HTTP exposes getRandomValues but does not expose randomUUID.
    crypto:{getRandomValues:array=>webcrypto.getRandomValues(array)},
    fetch:async (url, options={}) => {
      if (url === '/api/teleop/home') { posts.push({action:'home'}); return {ok:true,json:async()=>({accepted:true})}; }
      if (url === '/api/control/ownership') return {ok:true,json:async()=>structuredClone(state)};
      assert.ok(url.startsWith('/api/control/intervention/'));
      const body=JSON.parse(options.body);
      posts.push({action:url.split('/').at(-1),body});
      if (handlePost) return handlePost(posts.at(-1));
      assert.equal(body.session_id,state.session_id);
      assert.equal(body.expected_epoch,state.epoch);
      state.epoch++;
      if (posts.at(-1).action==='start_model') {
        state.state='AUTO';state.session_id='session-1';
      } else if (posts.at(-1).action==='enable_dagger') {
        state.dagger_enabled=true;
        if(state.recording_enabled)state.capture={ready:true,capture_mode:body.capture_mode,prepared_slots:4,sources:{issues:[]}};
      } else if (posts.at(-1).action==='disable_dagger') {
        state.dagger_enabled=false;state.capture=null;
      } else if (posts.at(-1).action==='disable') {
        state.state='OFF';state.capture=null;state.dagger_enabled=false;
      } else if (posts.at(-1).action==='recover'||posts.at(-1).action==='hold') {
        state.state='HOLD';
      }
      return {ok:true,json:async()=>({ownership:structuredClone(state),capture:structuredClone(state.capture)})};
    },
  });
  vm.runInContext(source.replace(/init\(\);\s*$/, ''),context);
  context.reportError=message=>errors.push(message);
  vm.runInContext('toast=reportError; bindInterventionControls();',context);
  function render() {
    context.currentState=structuredClone(state);
    vm.runInContext('renderIntervention(currentState,currentState.capture)',context);
  }
  render();
  return {context,get,buttons,sources,state,posts,errors,render,setPost:handler=>{handlePost=handler;}};
}

test('start model, enable and disable DAgger, then end model are distinct actions on LAN HTTP',async()=>{
  const p=page();
  assert.equal(vm.runInContext('typeof crypto.randomUUID',p.context),'undefined');
  assert.equal(p.buttons.start_model.disabled,false);
  assert.equal(p.buttons.enable_dagger.disabled,true);
  await p.buttons.start_model.onclick();
  assert.equal(p.state.state,'AUTO');
  assert.equal(p.state.dagger_enabled,false);
  assert.match(p.get('#controlSourceHint').textContent,/手柄不会抢占/);
  await p.buttons.enable_dagger.onclick();
  assert.equal(p.state.dagger_enabled,true);
  assert.equal(p.buttons.disable_dagger.disabled,false);
  await p.buttons.disable_dagger.onclick();
  assert.equal(p.state.state,'AUTO');
  assert.equal(p.state.dagger_enabled,false);
  await p.buttons.disable.onclick();
  assert.equal(p.state.state,'OFF');
  assert.equal(new Set(p.posts.map(p=>p.body.request_id)).size,4);
});

test('new controls wait for matching HC backend during rollout',()=>{
  const p=page();delete p.state.dagger_enabled;p.render();
  assert.equal(p.buttons.start_model.disabled,true);
  assert.match(p.get('#interventionReadiness').textContent,/重启/);
  p.state.state='AUTO';p.render();
  assert.equal(p.buttons.enable_dagger.disabled,true);
  assert.equal(p.buttons.disable_dagger.disabled,true);
  assert.equal(p.buttons.hold.disabled,false);
});

test('capture selection belongs to DAgger and cannot block model start',async()=>{
  const p=page();p.state.recording_enabled=true;p.render();
  p.get('#interventionCaptureMode').value='full_session';
  await p.buttons.start_model.onclick();
  assert.equal(p.posts[0].body.capture_mode,undefined);
  await p.buttons.enable_dagger.onclick();
  assert.equal(p.posts[1].body.capture_mode,'full_session');
  assert.equal(p.get('#interventionCaptureMode').disabled,true);
  assert.match(p.get('#interventionReadiness').textContent,/连续录制/);
});

test('fault reset remains paused and no held Grip blocks model-only start',async()=>{
  const p=page();
  Object.assign(p.state,{state:'ESTOP',gripped:true,input_valid:true});p.render();
  assert.equal(p.buttons.start_model.disabled,true);
  assert.equal(p.buttons.recover.disabled,false);
  await p.buttons.recover.onclick();
  assert.equal(p.state.state,'HOLD');
  assert.equal(p.buttons.start_model.disabled,false);
  p.state.dagger_enabled=true;p.render();
  assert.equal(p.buttons.start_model.disabled,true);
});

test('home requires DAgger manual authority and displays Grip guidance outside the button',async()=>{
  const p=page();
  for(const state of ['OFF','HOLD','AUTO','FAULT']) {
    Object.assign(p.state,{state,dagger_enabled:true,gripped:true});p.render();
    assert.equal(p.get('#interventionHome').disabled,true);
  }
  p.state.state='MANUAL';p.render();
  assert.equal(p.get('#interventionHome').disabled,false);
  assert.match(p.get('#interventionReadiness').textContent,/保持右 Grip/);
  await p.get('#interventionHome').onclick();
  assert.equal(p.posts.at(-1).action,'home');
});

test('pending request cannot be duplicated or enabled by polling',async()=>{
  const p=page();let resolve,entered;
  const submitted=new Promise(r=>entered=r);
  p.setPost(()=>new Promise(r=>{resolve=r;entered();}));
  const pending=p.buttons.start_model.onclick();await submitted;p.render();
  assert.ok(Object.values(p.buttons).every(b=>b.disabled));
  await p.buttons.start_model.onclick();assert.equal(p.posts.length,1);
  Object.assign(p.state,{state:'AUTO',session_id:'s',epoch:1});
  resolve({ok:true,json:async()=>({ownership:p.state,capture:null})});await pending;
  assert.equal(p.buttons.enable_dagger.disabled,false);
});

test('API failure remains visible and refreshes authority',async()=>{
  const p=page();
  p.setPost(async()=>({ok:false,status:409,text:async()=> 'feedback not ready'}));
  await p.buttons.start_model.onclick();
  assert.match(p.get('#interventionError').textContent,/feedback not ready/);
  assert.equal(p.state.state,'OFF');
  assert.equal(p.buttons.start_model.disabled,false);
});
