const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../middleware/core/static/app.js'),'utf8');
const html=fs.readFileSync(path.join(__dirname,'../middleware/core/static/index.html'),'utf8');
function page(){
  const elements=new Map();
  const get=key=>{if(!elements.has(key))elements.set(key,{value:'',textContent:''});return elements.get(key);};
  const context=vm.createContext({document:{querySelector:get,querySelectorAll:()=>[]}});
  vm.runInContext(source.replace(/init\(\);\s*$/,''),context);
  vm.runInContext('config={ros:{domain_id:14}}',context);
  return {get,context};
}
test('teleop shows launch domain 13 and keeps saved default 14 distinct',()=>{
  const p=page();
  vm.runInContext("renderRuntimeDomain({runtime_mode:'teleop',ros_domain_id:13,hardware_domain_id:'13'})",p.context);
  assert.equal(p.get('#runtimeRosDomain').value,'13');
  assert.match(p.get('#runtimeRosDomainHint').textContent,/覆盖.*默认域 14/);
  assert.equal(vm.runInContext('config.ros.domain_id',p.context),14);
  assert.match(html,/<input id="runtimeRosDomain" readonly/);
  assert.doesNotMatch(html,/<input id="runtimeRosDomain"[^>]*data-path/);
});
test('isolated simulation shows private domain separately from hardware domain',()=>{
  const p=page();
  vm.runInContext("renderRuntimeDomain({runtime_mode:'simulation',ros_domain_id:179,hardware_domain_id:'13'})",p.context);
  assert.equal(p.get('#runtimeRosDomain').value,'179');
  assert.match(p.get('#runtimeRosDomainHint').textContent,/真机域 13 不用于仿真/);
  assert.equal(vm.runInContext('config.ros.domain_id',p.context),14);
});
test('missing runtime state never presents stored default as the actual domain',()=>{
  const p=page();vm.runInContext('renderRuntimeDomain(null)',p.context);
  assert.equal(p.get('#runtimeRosDomain').value,'—');
});
