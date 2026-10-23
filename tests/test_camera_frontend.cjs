const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
class Element {
  constructor(tag='div'){this.tag=tag;this.children=[];this.dataset={};this.value='';this.type='';}
  append(...children){this.children.push(...children);}
  set textContent(value){this.text=value;this.children=[];}
  get textContent(){return this.text||'';}
  setAttribute(key,value){this[key]=value;}
  querySelectorAll(selector){
    const all=this.children.flatMap(child=>[child,...child.querySelectorAll('*')]);
    if(selector==='*')return all;
    if(selector.startsWith('.'))return all.filter(e=>e.className===selector.slice(1));
    const key=selector.slice(6,-1).replace(/-([a-z])/g,(_,c)=>c.toUpperCase());
    return all.filter(e=>key in e.dataset);
  }
}
function setup(){
  const cards=new Element();const options=new Element();
  const ctx=vm.createContext({document:{createElement:tag=>new Element(tag),
    querySelector:s=>s==='#cameraStreamCards'?cards:s==='#cameraTopicOptions'?options:null,
    querySelectorAll:s=>cards.querySelectorAll(s)}});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../middleware/core/static/app.js'),'utf8').replace(/init\(\);\s*$/,''),ctx);
  vm.runInContext(`config={camera:{enabled:true,streams:[
    {id:'head',name:'头部',topic:'/head',enabled:true,width:640,height:400,fps:30},
    {id:'overhead',topic:'/overhead',enabled:true},
    {id:'d405_left',topic:'/left',enabled:false},
    {id:'d405_right',topic:'/right',enabled:true}]},ros:{subscriptions:[
      {topic:'/head',type:'sensor_msgs/msg/Image',outputs:['record'],enabled:true,max_hz:7},
      {topic:'/other',type:'sensor_msgs/msg/CompressedImage',outputs:['record'],enabled:false,max_hz:12},
      {topic:'/joint',type:'sensor_msgs/msg/JointState',outputs:['record'],enabled:true}
    ]}};renderCameraStreams();`,ctx);
  return {ctx,cards,field:i=>cards.children[i].querySelectorAll('[data-camera-select]')[0]};
}
test('four numbered selectors only reuse custom image topics; no duplicate controls',()=>{
  const {ctx,cards,field}=setup();assert.equal(cards.children.length,4);
  for(let i=0;i<4;i++){
    assert.equal(cards.children[i].children[0].textContent,`${i+1}号相机`);
    assert.deepEqual(field(i).children.map(o=>o.value),['','/head','/other']);
    assert.equal(cards.children[i].querySelectorAll('[data-camera-field]').length,0);
  }
  const before=vm.runInContext('JSON.stringify(config.ros.subscriptions)',ctx);
  field(0).value='/other';field(1).value='';
  vm.runInContext('collectCameraStreams()',ctx);
  assert.equal(vm.runInContext('config.camera.streams[0].topic',ctx),'/other');
  assert.equal(vm.runInContext('config.camera.streams[0].type',ctx),'sensor_msgs/msg/CompressedImage');
  assert.equal(vm.runInContext('config.camera.streams[0].fps',ctx),30);
  assert.equal(vm.runInContext('config.camera.streams[1].enabled',ctx),false);
  assert.equal(vm.runInContext('JSON.stringify(config.ros.subscriptions)',ctx),before);
});
test('unconfigured topics cannot be selected',()=>{
  const {ctx,field}=setup();field(0).value='/unknown';
  assert.throws(()=>vm.runInContext('collectCameraStreams()',ctx),/自定义话题/);
});
test('custom topic changes refresh selectors without losing pending choices',()=>{
  const {ctx,field}=setup();field(0).value='/other';
  vm.runInContext("config.ros.subscriptions.push({topic:'/new',type:'sensor_msgs/msg/Image',outputs:['record']});renderCameraTopicOptions()",ctx);
  assert.equal(field(0).value,'/other');assert.ok(field(0).children.some(o=>o.value==='/new'));
});
test('monitor distinguishes no image, receiving and failed streams',()=>{
  const {ctx}=setup();
  assert.match(vm.runInContext("cameraMonitorSummary({capture_fps:0,peers:0})",ctx),/等待图像/);
  assert.strictEqual(vm.runInContext("cameraActualFps({capture_fps:60,webrtc_send_fps:30},{has_data:true,hz:16.4})",ctx),16.4);
  assert.strictEqual(vm.runInContext("cameraActualFps({capture_fps:60},{has_data:false,hz:60})",ctx),0);
  assert.match(vm.runInContext("cameraMonitorSummary({capture_fps:30,decoded_fps:28,jpeg_decode_ms:4.2,webrtc_send_fps:25,webrtc_encode_ms:6.3,peers:1})",ctx),/正在接收.*ROS输入 30.0 FPS.*JPEG解码 28.0 FPS\/4.2 ms.*头显发送 25.0 FPS.*H264 6.3 ms.*连接 1/);
  assert.match(vm.runInContext("cameraMonitorSummary({capture_fps:60,webrtc_send_fps:25},{has_data:true,hz:16.4})",ctx),/ROS输入 16.4 FPS.*头显发送 25.0 FPS/);
  assert.match(vm.runInContext("cameraMonitorSummary({capture_fps:30},{has_data:false})",ctx),/等待图像.*0.0/);
  assert.match(vm.runInContext("cameraMonitorSummary({error:'decode failed'})",ctx),/异常.*decode failed/);
});
