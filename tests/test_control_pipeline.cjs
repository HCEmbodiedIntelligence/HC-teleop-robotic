const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');

function setup(){
  const elements=new Map();
  for(const name of ['Vr','Solver','Mux','Action','Feedback']){
    for(const suffix of ['', 'Rate', 'Meta'])elements.set('#pipeline'+name+suffix,{dataset:{},textContent:suffix==='Meta'?name:''});
  }
  elements.set('#controlPipelineBadge',{});
  const diagnosis={};
  elements.set('#controlPipelineDiagnosis',{querySelector:()=>diagnosis});
  const ctx=vm.createContext({document:{querySelector:s=>elements.get(s)||null}});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../middleware/core/static/app.js'),'utf8').replace(/init\(\);\s*$/,''),ctx);
  const healthy=(hz)=>({has_data:true,hz,max_gap_ms:1000/hz});
  const ros={state:'running',topic_health:{'/vrdata':healthy(60),'/control/solver/joint_targets':healthy(100),
    '/action/joint_states':healthy(74),'/observation/joint_states':healthy(100)},command_mux:{source:'vr',
      output_enabled:true,received_hz:{vr:240},forwarded_hz:240,forwarded:2400,received:{vr:2400},rejected:0}};
  const render=()=>{ctx.ros=ros;vm.runInContext('renderControlPipeline(ros)',ctx);};
  return {ros,elements,render,diagnosis};
}

test('arm publication uses its channel rate and gap rather than combined arm and finger rates',()=>{
  const {ros,elements,render,diagnosis}=setup();
  ros.command_mux.channels={arms:{received_hz:100,forwarded_hz:74,last_received_age:.01,
    last_forwarded_age:.01,max_publication_gap_ms:50.2},right_hand:{forwarded_hz:100},left_hand:{forwarded_hz:100}};
  render();
  assert.equal(elements.get('#pipelineActionRate').textContent,'74.0 Hz');
  assert.equal(elements.get('#pipelineMuxRate').textContent,'100.0 Hz');
  assert.match(elements.get('#pipelineActionMeta').textContent,/最大间隔 50.2 ms/);
  assert.match(diagnosis.textContent,/各关节通道累计转发合计/);
});

test('zero arm rate stays zero while both finger channels continue at 200 Hz combined',()=>{
  const {ros,elements,render}=setup();
  ros.command_mux.forwarded_hz=200;
  ros.command_mux.channels={arms:{received_hz:100,forwarded_hz:0,last_received_age:.01,
    last_forwarded_age:.1,max_publication_gap_ms:null},right_hand:{forwarded_hz:100},left_hand:{forwarded_hz:100}};
  render();
  assert.equal(elements.get('#pipelineActionRate').textContent,'0.0 Hz');
  assert.equal(elements.get('#controlPipelineBadge').textContent,'命令转发掉速');
});

test('older backend uses the actual action topic and never guesses arm rate from combined counters',()=>{
  const {elements,render}=setup();
  render();
  assert.equal(elements.get('#pipelineActionRate').textContent,'74.0 Hz');
  assert.equal(elements.get('#pipelineMuxRate').textContent,'无数据');
});
