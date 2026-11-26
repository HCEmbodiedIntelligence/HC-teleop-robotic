const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
function element(tag='div') {
  let _text = '';
  return {
    tag,children:[],style:{},dataset:{},
    get textContent(){return _text;},
    set textContent(v){_text=String(v??'');if(_text==='')this.children=[];},
    append(...items){this.children.push(...items);},
    prepend(...items){this.children.unshift(...items);},
    setAttribute(key,value){this[key]=value;},
    removeAttribute(key){delete this[key];},
    remove(){this.removed=true;},
    showModal(){this.open=true;},
    close(){this.open=false;},
    pause(){this.paused=true;},
    load(){this.loaded=true;},
  };
}
function setup(files=[]) {
  const fields=Object.fromEntries([
    'batchEditDatasets','batchDownloadDatasets','datasetDownloadPanel','datasetDownloadJobs',
    'datasetRows','datasetsSummary','datasetsTitle','datasetFormatTabs','datasetMcapCount',
    'datasetLerobotCount','datasetBundleCount','selectAllDatasets','selectedDatasetCount',
    'batchDeleteDatasets','datasetDetailDialog','datasetDetailTitle','datasetDetailMeta',
    'datasetDetailSummary','datasetDetailAnnotations','datasetDetailEditBtn',
    'datasetDetailChannelRows','datasetDetailVideos',
    'datasetSearchInput','datasetFilterRobot','datasetFilterStatus','datasetFilterTag',
    'datasetFilterDuration','datasetFilterTime','datasetDateRange','datasetFilterDateFrom',
    'datasetFilterDateTo','datasetSortSelect','datasetFilterReset',
    'datasetEditDialog','datasetEditForm','datasetEditId','datasetEditTitle','datasetEditSubtitle',
    'datasetEditFilename','datasetEditFilenameHint','datasetEditTask','datasetEditRobot',
    'datasetEditRobotHint','datasetEditQuality','datasetEditMarked','datasetEditNotes',
    'datasetEditTagChips','datasetEditTagInput','datasetEditTagSuggestions','datasetEditError',
    'saveDatasetEdit','datasetTaskSuggestions','datasetRobotSuggestions',
    'datasetBatchEditDialog','datasetBatchEditForm','datasetBatchEditSubtitle',
    'datasetBatchTaskMode','datasetBatchTaskValue','datasetBatchRobotMode','datasetBatchRobotValue',
    'datasetBatchQuality','datasetBatchMarked','datasetBatchAddTags','datasetBatchRemoveTags',
    'datasetBatchEditError','saveDatasetBatchEdit',
  ].map(key=>['#'+key,element()]));
  fields['#datasetSortSelect'].value='time_desc';
  const calls=[];
  const ctx=vm.createContext({document:{querySelector:s=>fields[s]||null,querySelectorAll:()=>[],createElement:element,body:element('body')},confirm:()=>true});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../middleware/core/static/app.js'),'utf8').replace(/init\(\);\s*$/,''),ctx);
  ctx.api=async(url,options)=>{calls.push({url,options});return {files};};
  ctx.toast=()=>{};
  return {ctx,fields,calls};
}
const dataset={id:'lerobot:teleop test',filename:'teleop test',format:'LeRobot',frames:122,total_episodes:1,fps:15,camera_count:3,tasks:['搬箱子'],task:'搬箱子',channels:[{topic:'observation.state',type:'float32',count:122,rate_hz:15}],videos:[{feature:'observation.images.left',url:'/api/recordings/lerobot%3Ateleop%20test/video?feature=left'}]};
test('LeRobot list uses opaque identifier, ZIP download and details without ROS replay',async()=>{
  const {ctx,fields,calls}=setup([dataset]);
  await vm.runInContext('loadDatasets()',ctx);
  const cells=fields['#datasetRows'].children[0].children;
  assert.equal(cells[0].children[0].dataset.filename,dataset.id);
  assert.match(cells[5].textContent,/122 帧.*1 episode/);
  assert.match(cells[6].textContent,/3 路相机.*15 FPS/);
  const actions=cells[9].children[0].children;
  assert.deepEqual(actions.map(el=>el.textContent),['详情','编辑','下载','选择任务后上传','删除']);
  assert.equal(actions[2].href,'/api/recordings/lerobot%3Ateleop%20test/download?with_tags=1');
  assert.equal(actions[2].download,''); // Server Content-Disposition selects the ZIP filename.
  await actions.find(el=>el.textContent==='删除').onclick();
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
  // Detail dialog edit button hands off cleanly to the edit dialog.
  fields['#datasetDetailEditBtn'].onclick();
  assert.equal(fields['#datasetDetailDialog'].open,false);
  assert.equal(fields['#datasetEditDialog'].open,true);
  assert.equal(fields['#datasetEditFilename'].value,dataset.filename);
});
test('MCAP keeps replay and a server-named download with optional tags',async()=>{
  const {ctx,fields}=setup([{filename:'test.mcap',format:'MCAP',message_count:12,channels:[{topic:'/clock'}]}]);
  await vm.runInContext('loadDatasets()',ctx);
  const actions=fields['#datasetRows'].children[0].children[9].children[0].children;
  assert.deepEqual(actions.map(el=>el.textContent),['重放','详情','编辑','下载','选择任务后上传','删除']);
  assert.equal(actions[3].href,'/api/recordings/test.mcap/download?with_tags=1');
  assert.equal(actions[3].download,''); // Tagged MCAP may be returned as a ZIP.
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
  assert.deepEqual(actions.map(el=>el.textContent),['编辑','下载','选择任务后上传','删除']);
  assert.ok(actions.every(el=>el.disabled));
  assert.equal(actions[1].href,undefined);
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
  ctx.setTimeout=(callback,ms)=>{assert.equal(ms,30000);ctx.tick=callback;return 7;};
  ctx.clearTimeout=()=>clears++;
  ctx.loadDatasets=()=>count++;
  vm.runInContext("updateDatasetPolling('datasets')",ctx);
  assert.equal(count,1);
  vm.runInContext('scheduleDatasetPolling([])',ctx);ctx.tick();assert.equal(count,2);
  ctx.location.hash='#topics';ctx.tick();assert.equal(count,2);
  vm.runInContext("updateDatasetPolling('topics')",ctx);
  ctx.location.hash='#datasets';ctx.document.visibilityState='hidden';
  vm.runInContext("updateDatasetPolling('datasets')",ctx);assert.equal(count,2);
});

test('historical recording opens task selection without asking for artifact IDs',async()=>{
  const {ctx,fields}=setup([{...dataset,artifact_id:'artifact-123',upload_artifact_state:'ready'},
    {...dataset,id:'pending',is_current:true}]);
  let selected=null;
  ctx.window={platformUploadHistory:(...args)=>{selected=args;}};
  await vm.runInContext('loadDatasets()',ctx);
  const action = row => row.children[9].children[0].children.find(button=>button.textContent==='选择任务后上传');
  const upload=action(fields['#datasetRows'].children[0]);
  assert.equal(upload.disabled,false); upload.onclick();
  assert.equal(selected[0],dataset.id);
  assert.equal(selected[2],'LEROBOT_V3');
  assert.equal(action(fields['#datasetRows'].children[1]).disabled,true);
});


test('row download starts a tracked single job without navigating or fetching a ZIP', async()=>{
  const {ctx,fields,calls}=setup([dataset]);
  await vm.runInContext('loadDatasets()',ctx);
  const download=fields['#datasetRows'].children[0].children[9].children[0].children[2];
  ctx.api=async(url,options)=>{calls.push({url,options});return {job:{id:'single-1',state:'queued',selection_count:1}};};
  let prevented=false;
  await download.onclick({preventDefault(){prevented=true;}});
  assert.equal(prevented,true);
  const request=calls.find(call=>call.options?.method==='POST');
  assert.equal(request.url,'/api/recordings/download-jobs');
  assert.deepEqual(JSON.parse(request.options.body),{filenames:[dataset.id],single:true});
  assert.equal(fields['#datasetDownloadPanel'].hidden,false);
  assert.equal(fields['#datasetDownloadJobs'].children[0].children[2].value,undefined);
  assert.equal(ctx.document.body.children.length,0);
});

test('packing progress exposes bytes, files and accessible percentage; cancellation uses job API', async()=>{
  const {ctx,fields,calls}=setup();
  ctx.snapshot={id:'job-1',state:'packing',stage:'打包文件',percent:40,bytes_done:4096,bytes_total:10240,files_done:2,files_total:5,current_file:'data/part.parquet'};
  vm.runInContext("acceptDatasetDownloadJob(snapshot,{filenames:['lerobot:test'],single:false})",ctx);
  const card=fields['#datasetDownloadJobs'].children[0];
  assert.equal(card.children[2].value,40);
  assert.match(card.children[2]['aria-valuetext'],/40.0%/);
  assert.match(card.children[3].textContent,/4.0 KB \/ 10.0 KB.*2 \/ 5 个文件/);
  assert.equal(card.children[4].textContent,'data/part.parquet');
  ctx.api=async(url,options)=>{calls.push({url,options});return {job:{id:'job-1',state:'cancelled'}};};
  await card.children[6].children[0].onclick();
  assert.equal(calls[0].url,'/api/recordings/download-jobs/job-1/cancel');
  assert.equal(calls[0].options.method,'POST');
  assert.equal(card.children[6].children[0].hidden,true);
  assert.equal(card.children[6].children[1].textContent,'重新打包');
});

test('ready job launches native download once, tracks server transfer and stops its frame on cancel', async()=>{
  const {ctx,fields}=setup();
  ctx.snapshot={id:'ready-1',state:'ready',download_url:'/api/recordings/download-jobs/ready-1/download',archive_size:1000};
  vm.runInContext('acceptDatasetDownloadJob(snapshot);acceptDatasetDownloadJob(snapshot)',ctx);
  assert.equal(ctx.document.body.children.length,1);
  const frame=ctx.document.body.children[0];
  assert.equal(frame.tag,'iframe');assert.equal(frame.hidden,true);assert.equal(frame.src,ctx.snapshot.download_url);
  ctx.snapshot={...ctx.snapshot,state:'downloading',bytes_sent:350,files_done:2,files_total:2};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  const card=fields['#datasetDownloadJobs'].children[0];
  assert.equal(card.children[2].value,35);
  assert.match(card.children[1].textContent,/传输到浏览器/);
  assert.equal(card.children[6].children[0].textContent,'取消传输');
  ctx.api=async()=>({job:{id:'ready-1',state:'cancelled'}});
  await card.children[6].children[0].onclick();
  assert.equal(frame.src,'about:blank');
});

test('transient poll failure preserves job and retries; completed copy avoids claiming saved file', async()=>{
  const {ctx,fields}=setup();const scheduled=[];
  ctx.setTimeout=(callback,delay)=>{scheduled.push({callback,delay});return scheduled.length;};
  ctx.snapshot={id:'poll-1',state:'packing',percent:20};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  ctx.api=async()=>{throw new Error('offline');};
  await vm.runInContext('pollDatasetDownloadJobs()',ctx);
  const card=fields['#datasetDownloadJobs'].children[0];
  assert.match(card.children[5].textContent,/进度读取失败.*offline/);
  assert.equal(scheduled[0].delay,500);
  ctx.snapshot={id:'poll-1',state:'completed',archive_size:1000,bytes_sent:1000};
  ctx.api=async()=>({job:ctx.snapshot});
  await vm.runInContext('pollDatasetDownloadJobs()',ctx);
  assert.equal(fields['#datasetDownloadJobs'].children.length,1); // Stable controls while polling.
  assert.equal(card.children[2].value,100);
  assert.match(card.children[5].textContent,/请在浏览器下载列表确认保存结果/);
  assert.equal(card.children[6].children[0].hidden,true);
});

test('refresh restores owned job metadata without starting another browser job or duplicate transfer', async()=>{
  const {ctx,fields}=setup();let saved='';
  const own={id:'owned',state:'ready',download_url:'/api/recordings/download-jobs/owned/download'};
  ctx.sessionStorage={getItem:()=>JSON.stringify([{job:own,filenames:['one.mcap'],single:true,downloadStarted:true}]),setItem:(key,value)=>saved=value};
  ctx.api=async()=>({jobs:[own,{id:'other-browser',state:'ready',download_url:'/other'}]});
  await vm.runInContext('restoreDatasetDownloadJobs()',ctx);
  assert.equal(fields['#datasetDownloadJobs'].children.length,1);
  assert.equal(ctx.document.body.children.length,0);
  assert.equal(JSON.parse(saved)[0].filenames[0],'one.mcap');
  const retry=fields['#datasetDownloadJobs'].children[0].children[6].children[1];
  assert.equal(retry.hidden,false);retry.onclick();
  assert.equal(ctx.document.body.children[0].src,own.download_url);
});

test('failed creation offers retry using the original selection and single-file mode', async()=>{
  const {ctx,fields,calls}=setup();
  ctx.api=async()=>{throw new Error('磁盘空间不足');};
  await vm.runInContext("startDatasetDownload(['one.mcap'],true)",ctx);
  const card=fields['#datasetDownloadJobs'].children[0];
  assert.match(card.children[5].textContent,/磁盘空间不足/);
  ctx.api=async(url,options)=>{calls.push({url,options});return {job:{id:'retried',state:'queued'}};};
  await card.children[6].children[1].onclick();
  // Button starts an async job; let the promise continuation render it.
  await Promise.resolve();
  assert.deepEqual(JSON.parse(calls[0].options.body),{filenames:['one.mcap'],single:true});
});


test('terminal retry recreates server job without copying the dataset selection', async()=>{
  const {ctx,fields,calls}=setup();
  ctx.snapshot={id:'old-job',state:'completed',archive_size:1000,bytes_sent:1000};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  ctx.api=async(url,options)=>{calls.push({url,options});return {job:{id:'new-job',state:'queued'}};};
  const retry=fields['#datasetDownloadJobs'].children[0].children[6].children[1];
  assert.equal(retry.textContent,'重试下载');
  await retry.onclick();await Promise.resolve();
  assert.equal(calls[0].url,'/api/recordings/download-jobs/old-job/retry');
  assert.equal(calls[0].options.method,'POST');
  assert.equal(fields['#datasetDownloadJobs'].children.length,2);
});

test('restore marks expired server jobs as errors instead of polling indefinitely', async()=>{
  const {ctx,fields}=setup();
  ctx.sessionStorage={getItem:()=>JSON.stringify([{job:{id:'expired',state:'packing',percent:3}}]),setItem(){}};
  ctx.api=async()=>({jobs:[]});
  await vm.runInContext('restoreDatasetDownloadJobs()',ctx);
  const card=fields['#datasetDownloadJobs'].children[0];
  assert.match(card.children[5].textContent,/下载任务已失效/);
  assert.equal(card.children[6].children[0].hidden,true);
  assert.equal(card.children[6].children[1].hidden,true);
  assert.equal(card.children[6].children[2].hidden,false);
});


test('bulk download submits the checked opaque IDs to a job with bulk archive mode', async()=>{
  const {ctx,fields,calls}=setup();
  ctx.document.querySelectorAll=()=>[{dataset:{filename:'lerobot:甲'}},{dataset:{filename:'two.mcap'}}];
  ctx.api=async(url,options)=>{calls.push({url,options});return {job:{id:'bulk',state:'queued'}};};
  const source=fs.readFileSync(path.join(__dirname,'../middleware/core/static/app.js'),'utf8');
  const handler=source.match(/\$\('#batchDownloadDatasets'\)\.onclick = \(\) => \{[\s\S]*?\n    \};/)[0];
  vm.runInContext(handler,ctx);
  await fields['#batchDownloadDatasets'].onclick();
  assert.equal(calls[0].url,'/api/recordings/download-jobs');
  assert.deepEqual(JSON.parse(calls[0].options.body),{filenames:['lerobot:甲','two.mcap'],single:false});
});

test('late polling snapshot cannot restart a cancelled task or launch its native download',()=>{
  const {ctx,fields}=setup();
  ctx.snapshot={id:'race',state:'cancelled'};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  ctx.snapshot={id:'race',state:'ready',download_url:'/cancelled-archive'};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  assert.equal(ctx.document.body.children.length,0);
  assert.equal(fields['#datasetDownloadJobs'].children[0].children[1].textContent,'已取消');
});


test('HTTP 404 expires polling job after backend restart; network failures remain retryable',async()=>{
  const {ctx,fields}=setup();const scheduled=[];
  ctx.setTimeout=(callback,delay)=>{scheduled.push({callback,delay});return scheduled.length;};
  ctx.snapshot={id:'missing',state:'packing',percent:20};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  ctx.api=async()=>{const error=new Error('not found');error.status=404;throw error;};
  await vm.runInContext('pollDatasetDownloadJobs()',ctx);
  const card=fields['#datasetDownloadJobs'].children[0];
  assert.match(card.children[5].textContent,/任务已失效/);
  assert.equal(card.children[6].children[0].hidden,true);
  assert.equal(card.children[6].children[1].hidden,true);
  assert.equal(scheduled.length,0);
});

test('API errors preserve HTTP status so only reliable 404 responses expire tasks',async()=>{
  const {ctx}=setup();
  delete ctx.api;
  ctx.FormData=class FormData {};
  ctx.fetch=async()=>({ok:false,status:404,text:async()=>JSON.stringify({error:'任务不存在'})});
  // Re-evaluate the API declaration after setup replaced it with a test stub.
  const source=fs.readFileSync(path.join(__dirname,'../middleware/core/static/app.js'),'utf8');
  vm.runInContext(source.match(/async function api\(url, options=\{\}\) \{[\s\S]*?\n\}/)[0],ctx);
  await assert.rejects(vm.runInContext("api('/api/recordings/download-jobs/missing')",ctx),error=>error.status===404&&error.message==='任务不存在');
});

test('new jobs appear first in the floating panel without scrolling the document',()=>{
  const {ctx,fields}=setup();
  fields['#datasetDownloadPanel'].scrollTop=120;
  ctx.snapshot={id:'old',state:'completed',filename:'old.zip'};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  ctx.snapshot={id:'new',state:'packing',filename:'new.zip'};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  assert.equal(fields['#datasetDownloadJobs'].children[0].children[0].textContent,'new.zip');
  assert.equal(fields['#datasetDownloadPanel'].scrollTop,0);
});

test('completed job automatically closes its card and hides the panel after 5 seconds',()=>{
  const {ctx,fields}=setup();const scheduled=[];let saved='';
  ctx.setTimeout=(callback,delay)=>{scheduled.push({callback,delay});return scheduled.length;};
  ctx.sessionStorage={getItem:()=>'[]',setItem:(_key,value)=>{saved=value;}};
  ctx.snapshot={id:'auto-close',state:'ready',download_url:'/api/recordings/download-jobs/auto-close/download',archive_size:1024};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  assert.equal(ctx.document.body.children.length,1);
  const frame=ctx.document.body.children[0];
  ctx.snapshot={...ctx.snapshot,state:'completed',bytes_sent:1024,percent:100};
  vm.runInContext('acceptDatasetDownloadJob(snapshot)',ctx);
  const autoClose=scheduled.find(item=>item.delay===5000);
  assert.ok(autoClose,'expected a 5000ms auto-close timer');
  assert.equal(fields['#datasetDownloadPanel'].hidden,false);
  const card=fields['#datasetDownloadJobs'].children[0];
  autoClose.callback();
  assert.equal(card.removed,true);
  assert.equal(frame.removed,true);
  assert.equal(fields['#datasetDownloadPanel'].hidden,true);
  assert.deepEqual(JSON.parse(saved),[]);
});

test('dataset search, multi-facet filters, sorting and reset filter rows cleanly',async()=>{
  const nowSec=Math.floor(Date.UTC(2026,10,26,12,0,0)/1000);
  const items=[
    {id:'a.mcap',filename:'pick_box_01.mcap',format:'MCAP',profile_id:'x1',profile_display_name:'HC-X1',size_bytes:5000,size_human:'5.0 KB',duration_sec:8,duration_human:'8.0s',message_count:200,timestamp:nowSec-3600,created_at:'2026-11-26 11:00:00',task:'搬运箱子',tags:['高质量','双臂'],notes:'流畅抓取',quality:'good',marked:true},
    {id:'b.mcap',filename:'pour_water_02.mcap',format:'MCAP',profile_id:'openarm',profile_display_name:'OpenArm',size_bytes:25000,size_human:'25.0 KB',duration_sec:45,duration_human:'45.0s',message_count:900,timestamp:nowSec-3*86400,created_at:'2026-11-23 12:00:00',task:'倒水',tags:['待复查'],notes:'左手轻微抖动',quality:'review',marked:false},
    {id:'c.mcap',filename:'legacy_03.mcap',format:'MCAP',profile_id:'',profile_display_name:'',size_bytes:1000,size_human:'1.0 KB',duration_sec:90,duration_human:'1m 30s',message_count:50,timestamp:nowSec-40*86400,created_at:'2026-10-17 12:00:00',task:'',tags:[],notes:'',quality:'discard',marked:false},
  ];
  const {ctx,fields}=setup(items);
  await vm.runInContext('loadDatasets()',ctx);
  assert.equal(fields['#datasetRows'].children.length,3);
  assert.equal(fields['#datasetFilterReset'].hidden,true);
  // Multi-keyword search across filename, task, notes and robot.
  const searchRes=vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, query: 'x1 流畅'}, ${nowSec*1000})`,ctx);
  assert.deepEqual(searchRes.map(x=>x.id),['a.mcap']);
  // Filter by robot (including unknown legacy).
  const unknownRobot=vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, robot: '__unknown__'}, ${nowSec*1000})`,ctx);
  assert.deepEqual(unknownRobot.map(x=>x.id),['c.mcap']);
  // Filter by status / quality / tag / duration / time.
  assert.deepEqual(vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, status: 'marked'}, ${nowSec*1000})`,ctx).map(x=>x.id),['a.mcap']);
  assert.deepEqual(vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, status: 'quality:review'}, ${nowSec*1000})`,ctx).map(x=>x.id),['b.mcap']);
  assert.deepEqual(vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, tag: '双臂'}, ${nowSec*1000})`,ctx).map(x=>x.id),['a.mcap']);
  assert.deepEqual(vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, duration: 'medium'}, ${nowSec*1000})`,ctx).map(x=>x.id),['b.mcap']);
  assert.deepEqual(vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, duration: 'long'}, ${nowSec*1000})`,ctx).map(x=>x.id),['c.mcap']);
  assert.deepEqual(vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, time: '7d'}, ${nowSec*1000})`,ctx).map(x=>x.id),['a.mcap','b.mcap']);
  assert.deepEqual(vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, time: 'custom', dateFrom: '2026-11-20', dateTo: '2026-11-25'}, ${nowSec*1000})`,ctx).map(x=>x.id),['b.mcap']);
  // Sort by size descending and name ascending.
  assert.deepEqual(vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, sort: 'size_desc'}, ${nowSec*1000})`,ctx).map(x=>x.id),['b.mcap','a.mcap','c.mcap']);
  assert.deepEqual(vm.runInContext(`filterAndSortDatasets(datasetsData.files, {...datasetFilters, sort: 'name_asc'}, ${nowSec*1000})`,ctx).map(x=>x.id),['c.mcap','a.mcap','b.mcap']);
  // Empty filter result shows reset button and resetting restores all rows.
  vm.runInContext("datasetFilters.query='不存在的关键字'; renderDatasetTable();",ctx);
  assert.equal(fields['#datasetRows'].children.length,1);
  assert.match(fields['#datasetRows'].children[0].children[0].children[0].textContent,/没有符合当前搜索或筛选条件/);
  assert.equal(fields['#datasetFilterReset'].hidden,false);
  fields['#datasetRows'].children[0].children[0].children[1].onclick();
  assert.equal(fields['#datasetRows'].children.length,3);
  assert.equal(fields['#datasetFilterReset'].hidden,true);
});

test('quick star toggle and tag chip click work directly from dataset rows',async()=>{
  const item={id:'run_1.mcap',filename:'run_1.mcap',format:'MCAP',size_bytes:100,size_human:'100 B',tags:['高质量'],marked:false};
  const {ctx,fields,calls}=setup([item]);
  await vm.runInContext('loadDatasets()',ctx);
  const nameCell=fields['#datasetRows'].children[0].children[1];
  const starBtn=nameCell.children[0].children[0];
  assert.equal(starBtn.textContent,'☆');
  ctx.api=async(url,options)=>{
    calls.push({url,options});
    return {ok:true,id:'run_1.mcap',item:{...item,marked:true}};
  };
  await starBtn.onclick({});
  const patch=calls.find(c=>c.options?.method==='PATCH');
  assert.equal(patch.url,'/api/recordings/run_1.mcap');
  assert.deepEqual(JSON.parse(patch.options.body),{marked:true});
  assert.equal(fields['#datasetRows'].children[0].children[1].children[0].children[0].textContent,'★');
  // Clicking tag chip filters by that tag; clicking again clears.
  const tagChip=fields['#datasetRows'].children[0].children[1].children[1].children[0];
  tagChip.onclick({});
  assert.equal(vm.runInContext('datasetFilters.tag',ctx),'高质量');
  fields['#datasetRows'].children[0].children[1].children[1].children[0].onclick({});
  assert.equal(vm.runInContext('datasetFilters.tag',ctx),'');
});

test('single edit dialog saves metadata without sending unchanged filename, and supports rename',async()=>{
  const item={id:'old.mcap',filename:'old.mcap',format:'MCAP',profile_id:'x1',task:'旧任务',tags:['初始'],notes:'旧备注',quality:'',marked:false,details_available:true};
  const {ctx,fields,calls}=setup([item]);
  await vm.runInContext('loadDatasets()',ctx);
  const editBtn=fields['#datasetRows'].children[0].children[9].children[0].children.find(b=>b.textContent==='编辑');
  editBtn.onclick();
  assert.equal(fields['#datasetEditDialog'].open,true);
  assert.equal(fields['#datasetEditFilename'].value,'old.mcap');
  // Edit metadata without changing filename.
  fields['#datasetEditTask'].value='双臂装配';
  fields['#datasetEditRobot'].value='X1-一号机';
  fields['#datasetEditQuality'].value='good';
  fields['#datasetEditMarked'].checked=true;
  fields['#datasetEditNotes'].value='动作平稳';
  fields['#datasetEditTagInput'].value='优选, 双臂';
  ctx.api=async(url,options)=>{
    calls.push({url,options});
    if(options?.method==='PATCH') return {ok:true,id:'old.mcap',renamed:false,item:{...item,task:'双臂装配',robot_label:'X1-一号机',quality:'good',marked:true,tags:['初始','优选','双臂'],notes:'动作平稳'}};
    return {files:[{...item,task:'双臂装配'}]};
  };
  await vm.runInContext('saveDatasetEdit()',ctx);
  const patchCall=calls.find(c=>c.options?.method==='PATCH');
  assert.equal(patchCall.url,'/api/recordings/old.mcap');
  const sent=JSON.parse(patchCall.options.body);
  assert.equal('filename' in sent,false);
  assert.deepEqual(sent,{task:'双臂装配',robot_label:'X1-一号机',quality:'good',marked:true,tags:['初始','优选','双臂'],notes:'动作平稳'});
  assert.equal(fields['#datasetEditDialog'].open,false);

  // Reopen and rename filename.
  ctx.item=item;
  vm.runInContext('openDatasetEditDialog(item)',ctx);
  fields['#datasetEditFilename'].value='renamed.mcap';
  calls.length=0;
  await vm.runInContext('saveDatasetEdit()',ctx);
  const renamePatch=calls.find(c=>c.options?.method==='PATCH');
  assert.equal(JSON.parse(renamePatch.options.body).filename,'renamed.mcap');
});

test('batch edit dialog submits only modified fields to batch-metadata endpoint',async()=>{
  const {ctx,fields,calls}=setup([dataset]);
  ctx.document.querySelectorAll=s=>s.includes(':checked') ? [{dataset:{filename:'a.mcap'}},{dataset:{filename:'b.mcap'}}] : [];
  vm.runInContext('openDatasetBatchEditDialog()',ctx);
  assert.equal(fields['#datasetBatchEditDialog'].open,true);
  fields['#datasetBatchTaskMode'].value='set';
  fields['#datasetBatchTaskValue'].value='抓取瓶子';
  fields['#datasetBatchQuality'].value='good';
  fields['#datasetBatchMarked'].value='true';
  fields['#datasetBatchAddTags'].value='批量标签, 合格';
  fields['#datasetBatchRemoveTags'].value='草稿';
  ctx.api=async(url,options)=>{
    calls.push({url,options});
    if(options?.method==='POST') return {ok:true,updated:['a.mcap','b.mcap'],failed:[]};
    return {files:[]};
  };
  await vm.runInContext('saveDatasetBatchEdit()',ctx);
  const post=calls.find(c=>c.options?.method==='POST');
  assert.equal(post.url,'/api/recordings/batch-metadata');
  assert.deepEqual(JSON.parse(post.options.body),{
    filenames:['a.mcap','b.mcap'],
    updates:{task:'抓取瓶子',quality:'good',marked:true,add_tags:['批量标签','合格'],remove_tags:['草稿']},
  });
  assert.equal(fields['#datasetBatchEditDialog'].open,false);
});
