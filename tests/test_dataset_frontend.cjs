const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
function element(tag='div') {
  return {tag,children:[],style:{},dataset:{},textContent:'',append(...items){this.children.push(...items);},setAttribute(key,value){this[key]=value;},showModal(){this.open=true;},pause(){this.paused=true;},load(){this.loaded=true;}};
}
function setup(files=[]) {
  const fields=Object.fromEntries(['datasetRows','datasetsSummary','selectAllDatasets','selectedDatasetCount','batchDeleteDatasets','datasetDetailDialog','datasetDetailTitle','datasetDetailMeta','datasetDetailSummary','datasetDetailChannelRows','datasetDetailVideos'].map(key=>['#'+key,element()]));
  const calls=[];
  const ctx=vm.createContext({document:{querySelector:s=>fields[s]||null,querySelectorAll:()=>[],createElement:element},confirm:()=>true});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../middleware/core/static/app.js'),'utf8').replace(/init\(\);\s*$/,''),ctx);
  ctx.api=async(url,options)=>{calls.push({url,options});return {files};};
  ctx.toast=()=>{};
  return {ctx,fields,calls};
}
const dataset={id:'lerobot:teleop test',filename:'teleop test',format:'LeRobot',frames:122,total_episodes:1,fps:15,camera_count:3,tasks:['搬箱子'],channels:[{topic:'observation.state',type:'float32',count:122,rate_hz:15}],videos:[{feature:'observation.images.left',url:'/api/recordings/lerobot%3Ateleop%20test/video?feature=left'}]};
test('LeRobot list uses opaque identifier, ZIP download and details without ROS replay',async()=>{
  const {ctx,fields,calls}=setup([dataset]);
  await vm.runInContext('loadDatasets()',ctx);
  const cells=fields['#datasetRows'].children[0].children;
  assert.equal(cells[0].children[0].dataset.filename,dataset.id);
  assert.match(cells[5].textContent,/122 帧.*1 episode/);
  assert.match(cells[6].textContent,/3 路相机.*15 FPS/);
  const actions=cells[9].children[0].children;
  assert.deepEqual(actions.map(el=>el.textContent),['详情','下载','删除']);
  assert.equal(actions[1].href,'/api/recordings/lerobot%3Ateleop%20test/download');
  assert.equal(actions[1].download,'teleop test.zip');
  await actions[2].onclick();
  assert.equal(calls.find(call=>call.options?.method==='DELETE').url,'/api/recordings/lerobot%3Ateleop%20test');
});
test('LeRobot details show aligned frame statistics, task, features and camera preview',()=>{
  const {ctx,fields}=setup();ctx.dataset=dataset;
  vm.runInContext('showDatasetDetails(dataset)',ctx);
  const stats=fields['#datasetDetailSummary'].children.map(el=>el.children.map(child=>child.textContent));
  assert.ok(stats.some(([label,value])=>label==='总帧数'&&value==='122'));
  assert.ok(stats.some(([label,value])=>label==='任务'&&value==='搬箱子'));
  assert.equal(fields['#datasetDetailChannelRows'].children[0].children[2].textContent,'122 帧');
  assert.equal(fields['#datasetDetailVideos'].hidden,false);
  assert.equal(fields['#datasetDetailVideos'].children[1].src,dataset.videos[0].url);
});
test('MCAP keeps replay and plain file download',async()=>{
  const {ctx,fields}=setup([{filename:'test.mcap',format:'MCAP',message_count:12,channels:[{topic:'/clock'}]}]);
  await vm.runInContext('loadDatasets()',ctx);
  const actions=fields['#datasetRows'].children[0].children[9].children[0].children;
  assert.deepEqual(actions.map(el=>el.textContent),['重放','详情','下载','删除']);
  assert.equal(actions[2].href,'/api/recordings/test.mcap/download');
  assert.equal(actions[2].download,'test.mcap');
});

test('pending export appears immediately with stage, percentage and counters; operations disabled',async()=>{
  const file={...dataset,is_current:true,export_state:'exporting',export_progress:{stage:'编码相机视频',percent:62,current:1,total:3,unit:'路视频'}};
  const {ctx,fields}=setup([file]);
  await vm.runInContext('loadDatasets()',ctx);
  const cells=fields['#datasetRows'].children[0].children;
  assert.equal(cells[0].children[0].disabled,true);
  const progress=cells[8].children[0];
  assert.match(progress.children[0].textContent,/编码相机视频.*62%.*1 \/ 3 路视频/);
  assert.equal(progress.children[1].value,62);
  const actions=cells[9].children[0].children;
  assert.deepEqual(actions.map(el=>el.textContent),['下载','删除']);
  assert.ok(actions.every(el=>el.disabled));
  assert.equal(actions[0].href,undefined);
});
test('failed export exposes reason and cannot download, delete or open details',async()=>{
  const {ctx,fields}=setup([{...dataset,is_current:false,export_state:'error',export_progress:{state:'error',error:'编码失败'}}]);
  await vm.runInContext('loadDatasets()',ctx);
  const cells=fields['#datasetRows'].children[0].children;
  assert.match(cells[8].children[0].children[0].textContent,/导出失败：编码失败/);
  assert.ok(cells[9].children[0].children.every(el=>el.disabled));
});
test('refresh does not overlap requests and preserves checked dataset IDs',async()=>{
  const {ctx,fields}=setup([dataset]);
  let resolve,calls=0;
  ctx.api=()=>{calls++;return new Promise(done=>{resolve=done;});};
  const first=vm.runInContext('loadDatasets()',ctx);
  await vm.runInContext('loadDatasets()',ctx);
  assert.equal(calls,1);
  ctx.document.querySelectorAll=selector=>selector.includes(':checked') ? [{dataset:{filename:dataset.id}}] : [];
  resolve({files:[dataset]});await first;
  assert.equal(fields['#datasetRows'].children[0].children[0].children[0].checked,true);
});
test('dataset polling runs only on visible dataset page',()=>{
  const {ctx}=setup();let count=0,clears=0;
  ctx.location={hash:'#datasets'};ctx.document.visibilityState='visible';
  ctx.setInterval=(callback,ms)=>{assert.equal(ms,2000);ctx.tick=callback;return 7;};
  ctx.clearInterval=()=>clears++;
  ctx.loadDatasets=()=>count++;
  vm.runInContext("updateDatasetPolling('datasets')",ctx);
  assert.equal(count,1);ctx.tick();assert.equal(count,2);
  ctx.location.hash='#topics';ctx.tick();assert.equal(count,2);
  vm.runInContext("updateDatasetPolling('topics')",ctx);assert.equal(clears,1);
  ctx.location.hash='#datasets';ctx.document.visibilityState='hidden';
  vm.runInContext("updateDatasetPolling('datasets')",ctx);assert.equal(count,2);
});
