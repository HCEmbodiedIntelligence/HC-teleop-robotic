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
  const buttons = Object.fromEntries(['enable','return','hold','recover','disable'].map(action =>
    [action, {dataset:{intervention:action}, disabled:false}]));
  const sources = ['vr','motion','exoskeleton'].map(source=>({dataset:{source},disabled:false}));
  const state = {state:'OFF', session_id:'', epoch:0, peers:{motion:'motion-1',teleop:'teleop-1'}, capture:null};
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
      if (posts.at(-1).action==='enable') {
        state.state='HOLD';state.session_id='session-1';
        state.capture={ready:false,capture_mode:body.capture_mode,prepared_slots:4,sources:{issues:[]}};
      } else if (posts.at(-1).action==='disable') {
        state.state='OFF';state.capture=null;
      } else if (posts.at(-1).action==='recover') {
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

test('LAN HTTP buttons enable and exit a session without crypto.randomUUID',async()=>{
  const p=page();
  assert.equal(vm.runInContext('typeof crypto.randomUUID',p.context),'undefined');
  await p.buttons.enable.onclick();
  assert.equal(p.state.state,'HOLD');
  assert.equal(p.posts[0].body.capture_mode,'corrections_only');
  assert.match(p.posts[0].body.request_id,/^[0-9a-f]{32}$/);
  assert.equal(p.get('#interventionState').textContent,'保持');
  assert.match(p.get('#interventionReadiness').textContent,/DAgger 已启用/);
  assert.equal(p.buttons.disable.disabled,false);
  await p.buttons.disable.onclick();
  assert.equal(p.state.state,'OFF');
  assert.equal(p.buttons.enable.disabled,false);
  assert.notEqual(p.posts[0].body.request_id,p.posts[1].body.request_id);
  assert.deepEqual(p.errors,[]);
});

test('whole session selection reaches the API, locks during capture and restores from server status',async()=>{
  const p=page();
  p.get('#interventionCaptureMode').value='full_session';
  p.get('#interventionCaptureMode').onchange();
  assert.match(p.get('#interventionCaptureHint').textContent,/连续录制/);
  await p.buttons.enable.onclick();
  assert.equal(p.posts[0].body.capture_mode,'full_session');
  assert.equal(p.get('#interventionCaptureMode').disabled,true);
  assert.ok(p.sources.every(button=>button.disabled));
  assert.match(p.get('#interventionReadiness').textContent,/正在连续录制整个会话/);
  p.get('#interventionCaptureMode').value='corrections_only';
  p.render();
  assert.equal(p.get('#interventionCaptureMode').value,'full_session');
  await p.buttons.disable.onclick();
  assert.equal(p.get('#interventionCaptureMode').disabled,false);
  assert.ok(p.sources.every(button=>!button.disabled));
});

test('polling cannot unlock the buttons or submit a second request while enabling',async()=>{
  const p=page();
  let complete, entered;
  const submitted=new Promise(resolve=>{entered=resolve;});
  p.setPost(()=>new Promise(resolve=>{complete=resolve;entered();}));
  const pending=p.buttons.enable.onclick();
  await submitted;
  p.render();
  assert.ok(Object.values(p.buttons).every(button=>button.disabled));
  assert.ok(p.sources.every(button=>button.disabled));
  await p.buttons.enable.onclick();
  assert.equal(p.posts.length,1);
  Object.assign(p.state,{state:'HOLD',session_id:'session-1',epoch:1,capture:{ready:false,prepared_slots:4}});
  complete({ok:true,json:async()=>({ownership:p.state,capture:p.state.capture})});
  await pending;
  assert.equal(p.buttons.disable.disabled,false);
});

test('capture preparation failure stays visible and refreshes authority so the session can be exited',async()=>{
  const p=page();
  p.setPost(async()=>{
    Object.assign(p.state,{state:'HOLD',session_id:'session-1',epoch:1});
    return {ok:false,status:409,text:async()=> '录制目录不可写'};
  });
  await p.buttons.enable.onclick();
  p.render();
  assert.match(p.get('#interventionError').textContent,/录制目录不可写/);
  assert.equal(p.get('#interventionState').textContent,'保持');
  assert.equal(p.buttons.enable.disabled,true);
  assert.equal(p.buttons.disable.disabled,false);
  p.setPost(null);
  await p.buttons.disable.onclick();
  assert.equal(p.state.state,'OFF');
  assert.equal(p.get('#interventionError').textContent,'');
});

test('program start requires observations but no headset; active Grip still blocks start',()=>{
  const p=page();
  Object.assign(p.state,{state:'HOLD',input_valid:false,input_armed:false,gripped:false,
    capture:{ready:false,prepared_slots:4,sources:{issues:['/observation/cameras/head/color/compressed']}}});
  p.render();
  assert.match(p.get('#interventionReadiness').textContent,/等待观测：头部相机/);
  assert.match(p.get('#interventionReadiness').textContent,/VR 未就绪，仅人工接管不可用/);
  assert.equal(p.buttons.return.disabled,true);
  p.state.capture.ready=true;
  p.state.capture.sources.issues=[];
  p.render();
  assert.equal(p.buttons.return.disabled,false);
  p.state.input_valid=true;
  p.state.input_armed=true;
  p.state.gripped=true;
  p.render();
  assert.equal(p.buttons.return.disabled,true);
  assert.match(p.get('#interventionReadiness').textContent,/右 Grip 正握持/);
  p.state.gripped=false;
  p.render();
  assert.equal(p.buttons.return.disabled,false);
});

test('fault recovery is explicit and only returns to HOLD before another start',async()=>{
  const p=page();
  Object.assign(p.state,{state:'ESTOP',session_id:'session-1',epoch:3,
    recording_enabled:false,input_valid:false,input_armed:false,gripped:false});
  p.render();
  assert.equal(p.posts.length,0);
  assert.equal(p.buttons.return.disabled,true);
  assert.equal(p.buttons.recover.disabled,false);
  assert.match(p.get('#interventionReadiness').textContent,/故障复位（保持暂停）/);
  await p.buttons.recover.onclick();
  assert.equal(p.posts.length,1);
  assert.equal(p.posts[0].action,'recover');
  assert.equal(p.state.state,'HOLD');
  assert.equal(p.buttons.recover.disabled,true);
  assert.equal(p.buttons.return.disabled,false);
});


test('v23 control can start without capture and explains disabled automatic recording',()=>{
  const p=page();
  Object.assign(p.state,{state:'HOLD',required_peers:['solver','teleop'],peers:{solver:'v23',teleop:'vr'},recording_enabled:false,input_valid:false,input_armed:false,gripped:false,capture:null});
  p.render();
  assert.equal(p.buttons.return.disabled,false);
  assert.equal(p.get('#interventionCaptureMode').disabled,true);
  assert.match(p.get('#interventionReadiness').textContent,/自动录制已关闭/);
  assert.match(p.get('#interventionCaptureHint').textContent,/自动录制已关闭/);
  p.state.legacy_control={hands_ready:{left:false,right:true},last_rejection:'target too far'};
  p.render();
  assert.match(p.get('#interventionReadiness').textContent,/左手反馈未就绪/);
  assert.match(p.get('#interventionReadiness').textContent,/target too far/);
});

test('home is available only during manual takeover and sends the home request',async()=>{
  const p=page();
  for(const state of ['OFF','HOLD','AUTO','FAULT']) {
    Object.assign(p.state,{state,gripped:true});p.render();
    assert.equal(p.get('#interventionHome').disabled,true);
  }
  Object.assign(p.state,{state:'MANUAL',gripped:true,recording_enabled:false});p.render();
  assert.equal(p.get('#interventionHome').disabled,false);
  await p.get('#interventionHome').onclick();
  assert.equal(p.posts.at(-1).action,'home');
  assert.match(p.get('#interventionCaptureHint').textContent,/左 X 开始录制，左 Y 停止录制/);
});
