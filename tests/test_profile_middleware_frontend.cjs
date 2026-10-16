const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../middleware/core/static/app.js'), 'utf8');

function page() {
  const elements = new Map();
  const get = key => {
    if (!elements.has(key)) elements.set(key, {value:'b', textContent:'', append(){}, disabled:true});
    return elements.get(key);
  };
  const context = vm.createContext({document:{querySelector:get, querySelectorAll:()=>[], createElement:()=>({append(){}})}});
  vm.runInContext(source.replace(/init\(\);\s*$/, ''), context);
  return {context,get};
}

test('applying a package replaces stale GUI settings before a later save', async () => {
  const {context,get}=page();
  vm.runInContext(`
    config={robot_profiles:{active:'a'},camera:{fps:12}};
    profilesData={active:'a',profiles:[]};
    api=async()=>({active:'b',config:{robot_profiles:{active:'b'},camera:{fps:48}},restart_simulation_required:true});
    renderConfig=()=>{document.querySelector('#fps').value=config.camera.fps;};
    loadProfiles=async()=>{};
    toast=()=>{};
  `,context);
  const handler=source.match(/\$\('#activateProfile'\)\.onclick=async\(\)=>\{[\s\S]*?\n  \};/)[0];
  vm.runInContext(handler,context);
  await get('#activateProfile').onclick();
  assert.equal(vm.runInContext('config.robot_profiles.active',context),'b');
  assert.equal(vm.runInContext('config.camera.fps',context),48);
  assert.equal(get('#fps').value,48);
});

test('an already active valid package can be reapplied after an update', () => {
  const {context,get}=page();
  vm.runInContext("profilesData={active:'b',profiles:[{id:'b',schema:'hc-robot-config-v1'}]}; renderProfileDetail();",context);
  assert.equal(get('#activateProfile').disabled,false);
  vm.runInContext("profilesData.profiles[0].schema='invalid';renderProfileDetail();",context);
  assert.equal(get('#activateProfile').disabled,true);
});
