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
test('monitor separates codec, transport, local skips and receiver display',()=>{
  const {ctx}=setup();
  vm.runInContext("stream={capture_fps:30,decoded_fps:30,webrtc_submit_fps:28,webrtc_output_fps:27,fps_window_seconds:2,jpeg_decode_ms:4.2,webrtc_encode_ms:6.3,webrtc_submit_to_transport_ms:14.5,webrtc_transport_ms:3.1,decode_frames_overwritten:2,webrtc_frames_skipped:4,webrtc_feedback_received:true,webrtc_fraction_lost_pct:0,codec:'H264',peers:1}",ctx);
  const text=vm.runInContext("cameraMonitorSummary(stream,{has_data:true,hz:16.4})",ctx);
  assert.match(text,/同窗 2.0 s.*ROS输入 30.0 FPS.*图像解码 30.0 FPS.*编码输入 28.0 FPS.*RTP输出 27.0 FPS/);
  assert.match(text,/H264编码器内 6.3 ms.*发送处理 14.5 ms.*其中传输调用 3.1 ms/);
  assert.match(text,/本地覆盖 2 帧\/跳帧 4 帧.*网络丢包 0.00%.*头显显示帧率 未上报/);
  assert.doesNotMatch(text,/头显发送/);
  const help=vm.runInContext('cameraMonitorHelp(stream)',ctx);
  assert.match(help,/排队.*线程唤醒.*加密.*UDP/);
  assert.match(help,/不能重复相加.*不代表头显接收或显示/);
});
test('legacy and missing feedback are labeled without inventing measurements',()=>{
  const {ctx}=setup();
  assert.equal(vm.runInContext("cameraActualFps({capture_fps:60},{has_data:true,hz:16.4})",ctx),16.4);
  const text=vm.runInContext("cameraMonitorSummary({capture_fps:60,webrtc_send_fps:25,webrtc_fraction_lost_pct:0},{has_data:true,hz:16.4})",ctx);
  assert.match(text,/独立统计窗口.*ROS输入 16.4 FPS.*编码输入 25.0 FPS.*RTP输出 -- FPS/);
  assert.match(text,/发送处理 -- ms/);
  assert.match(text,/网络丢包 未收到反馈/);
  assert.match(vm.runInContext("cameraMonitorSummary({capture_fps:0,peers:0})",ctx),/等待图像/);
  assert.match(vm.runInContext("cameraMonitorSummary({capture_fps:30},{has_data:false})",ctx),/等待图像.*0.0/);
  assert.match(vm.runInContext("cameraMonitorSummary({error:'decode failed'})",ctx),/异常.*decode failed/);
});

// A deliberate per-camera codec selection survives a topic list refresh.
test('VP8 selection survives refresh and is saved for FEC',()=>{
  const {ctx,cards}=setup();
  const selector=cards.children[0].querySelectorAll('[data-camera-codec]')[0];
  selector.value='VP8';
  vm.runInContext('renderCameraTopicOptions();collectCameraStreams()',ctx);
  assert.equal(vm.runInContext('config.camera.streams[0].codec',ctx),'VP8');
});
test('FEC state distinguishes enabled negotiation from receiver recovery',()=>{
  const {ctx}=setup();
  const text=vm.runInContext("cameraMonitorSummary({capture_fps:30,webrtc_fec:{state:'enabled',fec_packets:12,overhead_pct:25},receiver_fec_packets_received:4})",ctx);
  assert.match(text,/FEC enabled.*校验包 12.*冗余 25.0%.*接收FEC 4/);
});

test('fresh server config replaces prior codec choices',()=>{
  const {ctx,cards}=setup();
  vm.runInContext("config.camera.streams[0].codec='VP8';renderCameraStreams()",ctx);
  assert.equal(cards.children[0].querySelectorAll('[data-camera-codec]')[0].value,'VP8');
});
