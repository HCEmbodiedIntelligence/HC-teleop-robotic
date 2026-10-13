const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
function setup(){
 const button={},calls=[];
 const ctx=vm.createContext({document:{querySelector:()=>button},Date});
 vm.runInContext(fs.readFileSync(require('node:path').join(__dirname,'../middleware/core/static/app.js'),'utf8').replace(/init\(\);\s*$/,''),ctx);
 ctx.calls=calls;
 vm.runInContext(`renderStatus=data=>renderSafetyButton(data.ros);toast=()=>{};api=async(url)=>{calls.push(url);return {ros:{state:'running',command_mux:{output_enabled:true}}};};`,ctx);
 return {ctx,button,calls};
}
test('button reflects reported gate state and unknown state',()=>{
 const {ctx,button}=setup();
 vm.runInContext(`renderSafetyButton({state:'running',command_mux:{output_enabled:false}})`,ctx);
 assert.equal(button.textContent,'解除急停');assert.equal(button.disabled,false);
 vm.runInContext('renderSafetyButton({})',ctx);
 assert.equal(button.textContent,'急停状态未知');assert.equal(button.disabled,true);
});
test('stop does not assume success; uses state fetched after POST',async()=>{
 const {ctx,button,calls}=setup();
 vm.runInContext(`renderSafetyButton({state:'running',command_mux:{output_enabled:true}})`,ctx);
 await vm.runInContext('toggleSafety()',ctx);
 assert.deepEqual(calls,['/api/safety/stop','/api/status']);
 assert.equal(button.textContent,'急停');
});
test('reported stop selects resume endpoint',async()=>{
 const {ctx,button,calls}=setup();
 vm.runInContext(`renderSafetyButton({state:'running',command_mux:{output_enabled:false}})`,ctx);
 await vm.runInContext('toggleSafety()',ctx);
 assert.equal(calls[0],'/api/safety/resume');assert.equal(button.textContent,'急停');
});
