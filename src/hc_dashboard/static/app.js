'use strict';
const $ = id => document.getElementById(id);
const pages = {overview:'状态监控',topics:'话题录制',datasets:'数据集管理',config:'系统配置'};
const streams = [
 ['joints','实际关节反馈','standard/joint_states','sensor_msgs/msg/JointState'],
 ['backend_candidates','控制器关节目标','standard/joint_targets','sensor_msgs/msg/JointState'],
 ['commands','机器人最终关节命令','standard/joint_commands','sensor_msgs/msg/JointState'],
];
function standardStreams(snapshot) {
 return [...streams,
 ['cartesian_targets','控制器末端目标','standard/controller_target_ee_poses','geometry_msgs/msg/PoseArray'],
 ['cartesian_targets','可视化末端目标','standard/target_ee_poses','geometry_msgs/msg/PoseArray'],
 ['cartesian_feedback','实际末端位姿','standard/actual_ee_poses','geometry_msgs/msg/PoseArray']];
}
let runtime = null, selectedProfile = null, datasetItems = [];
let latest = null, connected = false, pending = false, reconnectTimer, watchdog, toastTimer;
const set = (id,value) => {if ($(id)) $(id).textContent = value ?? '—';};
const num = (value,digits=4) => value === null || value === undefined || !Number.isFinite(Number(value)) ? '--' : Number(value).toFixed(digits);
const live = metric => Boolean(connected && metric && metric.total && !metric.stale);
function toast(message) {
  set('toast',message); $('toast').classList.add('show'); clearTimeout(toastTimer);
  toastTimer=setTimeout(()=>$('toast').classList.remove('show'),3500);
}
function row(body,values) {
  const tr=document.createElement('tr');
  for (const value of values) {const td=document.createElement('td');td.textContent=value ?? '—';td.title=value ?? '';tr.append(td);}
  body.append(tr);return tr;
}
function empty(id,message,columns) {
  const body=$(id);body.replaceChildren();const tr=row(body,[message]);tr.firstChild.colSpan=columns;tr.firstChild.className='empty-monitor';
}
function route() {
  const page=Object.hasOwn(pages,location.hash.slice(1)) ? location.hash.slice(1) : 'overview';
  document.querySelectorAll('.page').forEach(el=>el.classList.toggle('hidden',el.id!==page));
  document.querySelectorAll('nav a').forEach(el=>el.classList.toggle('active',el.dataset.page===page));
  set('pageTitle',pages[page]);$('nativeDiagnostics').classList.toggle('hidden',page!=='overview');
}
function controls() {
  $('stopButton').disabled=!connected || pending;
  $('resumeButton').disabled=!connected || pending || !latest;
}
function connection(value) {
  connected=value;set('connectionText',value?'实时状态已连接':'连接断开 · 正在重连');
  $('connectionDot').className=value?'online':'offline';controls();
  if (!value && latest) render(latest);
}
function metric(id,meta,stream,path) {
  const active=live(stream), slow=active && stream.hz<50;
  set(id,active?(slow?'偏低':'正常'):'无数据');
  $(id).className=active?(slow?'warning':'running'):'error';
  set(meta,`${active?num(stream.hz,1):'0.0'} Hz · ${path}`);
}
function render(snapshot) {
  latest=snapshot;const state=snapshot.safety || {}, metrics=snapshot.streams || {}, namespace=`/robots/${snapshot.robot_id}`;
  set('rosState',connected?'RUNNING':'DISCONNECTED');$('rosState').className=connected?'running':'error';
  const total=Object.values(metrics).reduce((sum,item)=>sum+(item.total || 0),0);
  set('rosMeta',`${Object.keys(metrics).length} 个监测流 · ${total} 条消息 · Domain ${runtime?.ros?.domain_id ?? '--'}`);
  const owner=state.active_source;
  const sourceLabel=owner && (owner===snapshot.vr?.source_id || /^(vr|pico)$/i.test(owner))?'VR / IK':owner || '待授权';
  set('controlSourceState',connected ? sourceLabel : '--');
  $('controlSourceState').className=!connected?'':state.enabled?'running':'error';
  set('controlSourceMeta',!connected?'等待数据':state.enabled?'命令输出已使能':'命令输出已停用');
  metric('jointCommandState','jointCommandMeta',metrics.commands,`${namespace}/control/joint_command`);
  metric('jointFeedbackState','jointFeedbackMeta',metrics.joints,`${namespace}/state/joints`);
  set('muxSafetyBadge',!connected?'状态未知':state.estop_active?'急停':state.fault_latched?'故障锁存':state.enabled?'已使能':'已停用');
  set('vrSourceStatus',`${live(metrics.backend_candidates)?'输入正常':'无数据'} · ${metrics.backend_candidates?.total || 0} 帧`);set('exoSourceStatus','未配置 · 0 帧');
  $('sourceVr').classList.toggle('active',connected && sourceLabel==='VR / IK');
  set('vrCommandTopic',`${namespace}/motion/backend/joint_candidate`);
  set('exoCommandTopic','未配置');set('outputCommandTopic',`${namespace}/control/joint_command`);
  set('muxForwarded',metrics.commands?.total || 0);
  set('vrState',live(metrics.vr)?'running':'无数据');set('vrMeta',`v${snapshot.vr?.protocol_version || '--'} · ${metrics.vr?.total || 0} 包`);
  set('vrPeer',live(metrics.vr)?snapshot.vr?.source_id || 'VR 已连接':'VR 未连接');
  set('teleopBackend',live(metrics.backend_candidates)?'Motion Server 在线':'Motion Server 未响应');set('teleopMode',live(metrics.cartesian_targets)?'跟随':'保持');
  set('cameraState','未接入');set('cameraMeta','当前 Dashboard 无相机接口');set('wsClients',connected ? snapshot.ws_clients ?? '--' : '0');
  $('wsClients').nextElementSibling.textContent='WebSocket 客户端';
  for (const [key,id] of [['head','trackHead'],['left','trackLeft'],['right','trackRight']]) {
    $(id).className=live(metrics.vr) && snapshot.vr?.tracking?.[key]?.tracked?'online':'offline';
  }
  set('teleopLeft','未提供');
  set('teleopRight','未提供');
  set('teleopFeedback',live(metrics.joints)?'正常':'等待反馈');
  const commands=new Map(), feedback=new Map();
  Object.values(snapshot.commands || {}).forEach(group=>(group.names || []).forEach((name,i)=>commands.set(name,group.positions?.[i])));
  (snapshot.joints?.values || []).forEach(item=>feedback.set(item.name,item.position));
  const names=[...new Set([...commands.keys(),...feedback.keys()])];
  $('jointMonitorRows').replaceChildren();set('jointMonitorCount',`${names.length} 个关节`);
  for(const name of names) {
    const command=live(metrics.commands)?commands.get(name):null, actual=live(metrics.joints)?feedback.get(name):null;
    const degrees=value=>value==null?null:value*180/Math.PI;
    const error=command==null || actual==null?null:degrees(command-actual);
    const tr=row($('jointMonitorRows'),[name,num(command),num(degrees(command),2),num(actual),num(degrees(actual),2),num(error,2)]);
    if(error!==null && Math.abs(error)>0.08*180/Math.PI) tr.className='joint-error';
  }
  if (!names.length) empty('jointMonitorRows','等待 JointState 消息…',6);
  renderTopics(snapshot);
  renderPoses(snapshot);
  $('diagnosticRows').replaceChildren();
  Object.entries(snapshot.diagnostics?.statuses || {}).forEach(([key,status])=>row($('diagnosticRows'),[key,!connected?'连接断开':status.level===0?'正常':status.level===1?'警告':'异常',status.message]));
  if (!$('diagnosticRows').children.length) empty('diagnosticRows','等待控制链路诊断消息',3);
  controls();
}
async function post(path,body) {
  const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:AbortSignal.timeout(5000)});
  const result=await response.json();if(!response.ok || !result.success) throw new Error(result.reason || '操作失败');return result;
}
async function safety(resume) {
  if(!connected || pending)return;
  pending=true;controls();
  try {
    if(resume && (latest?.safety?.fault_latched || latest?.safety?.estop_active))await post('/api/v1/safety/reset',{});
    await post('/api/v1/safety/enabled',{enabled:resume});toast(resume?'使能请求已确认':'停止请求已确认');
  }catch(error){toast(error.message);}finally{pending=false;controls();}
}
function connect() {
  const socket=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`);
  socket.onmessage=event=>{
    try {const snapshot=JSON.parse(event.data);if(snapshot.schema!=='hc-dashboard/v1')throw new Error('未知数据格式');connection(true);render(snapshot);
      clearTimeout(watchdog);watchdog=setTimeout(()=>{connection(false);socket.close();},2000);
    }catch(error){connection(false);socket.close();}
  };
  socket.onerror=()=>socket.close();socket.onclose=()=>{clearTimeout(watchdog);connection(false);clearTimeout(reconnectTimer);reconnectTimer=setTimeout(connect,1500);};
}
function initialize() {
  const unsupported=['sourceVr','sourceExoskeleton','startRecordingBtn','stopRecordingBtn','importMcapBtn','batchDeleteDatasets','openImportProfile','deleteProfile','activateProfile','saveConfig'];
  unsupported.forEach(id=>{if($(id)){$(id).disabled=true;$(id).title='当前 Dashboard 未提供此操作接口';}});
  document.querySelectorAll('#config input').forEach(el=>{el.readOnly=true;});
  document.querySelector('[data-path="vr.enabled"]').disabled=true;
  set('recordingMeta','未录制 · 选择与查看当前机器人话题');
  set('recordingReadinessText','正在检测话题消息发布与频率标准…');set('recordingBadgeText','未录制');
  empty('recordingRows','暂无自定义录制规则',7);
  empty('datasetRows','正在加载数据集列表…',10);set('datasetsSummary','正在加载数据集列表…');
  set('profileDetail','等待运行配置…');
  document.querySelector('[data-path="server.host"]').value=location.hostname;
  document.querySelector('[data-path="server.port"]').value=location.port || (location.protocol==='https:'?'443':'80');
  document.querySelector('.profile-manager .panel-title p').textContent='查看已安装的机器人 URDF、关节组与运行配置';
  $('stopButton').title='调用当前安全服务停用运动输出';$('resumeButton').title='复位故障（如有）并请求使能';
  $('stopButton').onclick=()=>safety(false);$('resumeButton').onclick=()=>safety(true);
  $('selectAllDatasets').onchange=()=>{document.querySelectorAll('#datasetRows input[type=checkbox]').forEach(input=>input.checked=$('selectAllDatasets').checked);updateDatasetSelection();};
  $('refreshDatasets').onclick=loadDatasets;
  $('refreshTopics').onclick=()=>loadRuntime(true);
  $('refreshProfiles').onclick=()=>loadRuntime();
  $('robotProfileSelect').onchange=()=>renderProfile($('robotProfileSelect').value);
  $('saveRecording').onclick=()=>{try{localStorage.setItem('hc.dashboard.selection',JSON.stringify([...selection]));saveRules();toast('已保存浏览器内的选择；当前未接入录制服务');}catch{toast('浏览器不允许保存本地配置');}};
  $('addRecording').onclick=addRule;
  window.addEventListener('hashchange',route);route();connect();loadRuntime();loadDatasets();
}

const selection = new Map();
try { const entries=JSON.parse(localStorage.getItem('hc.dashboard.selection') || '[]'); if(Array.isArray(entries))entries.forEach(item=>{if(Array.isArray(item)&&typeof item[0]==='string'&&typeof item[1]==='boolean')selection.set(...item);}); } catch {}
let customRules = [];
try { customRules=JSON.parse(localStorage.getItem('hc.dashboard.rules') || '[]'); if(!Array.isArray(customRules))customRules=[];customRules=customRules.filter(rule=>rule&&typeof rule.topic==='string'&&typeof rule.type==='string'); } catch {customRules=[];}
function element(tag,text,className='') {const el=document.createElement(tag);el.textContent=text;el.className=className;return el;}
function formatBytes(bytes) {if(!Number.isFinite(Number(bytes)))return '--';let n=Number(bytes);let unit=0;const units=['B','KB','MB','GB'];while(n>=1024 && unit<3){n/=1024;unit++;}return `${n.toFixed(unit?1:0)} ${units[unit]}`;}
function rateCells(tr,metric,key) {
  const active=live(metric), thresholds={vr:[60,30],joints:[100,50],cartesian_feedback:[100,50],commands:[100,50],backend_candidates:[60,30],cartesian_targets:[60,30]};
  const [target,min]=thresholds[key] || [0,0], low=active && metric.hz<min;
  tr.append(document.createElement('td'),document.createElement('td'),document.createElement('td'));
  const cells=[...tr.children].slice(-3);
  cells[0].append(element('span',active?`${num(metric.hz,1)} Hz`:'0.0 Hz',`topic-rate ${active?(low?'low':'ok'):'none'}`));
  cells[1].append(element('span',min?`≥ ${min} Hz (${target} Hz)`:'--','topic-target-rate'));
  const badge=element('span','',`topic-status-badge ${active?(low?'status-low':'status-ok'):'status-none'}`);
  badge.append(document.createElement('i'),document.createTextNode(active?(low?`偏低 (${num(metric.hz,1)} Hz)`:'消息正常'):'无消息 (0 Hz)'));cells[2].append(badge);
}
function renderTopics(snapshot) {
  const ns=`/robots/${snapshot.robot_id}`;
  const body=$('standardTopicRows');body.replaceChildren();$('configInterfaceRows').replaceChildren();
  let selected=0,ready=0;
  for(const [key,label,path,type] of standardStreams(snapshot)) {
    const topic=`${ns}/${path}`, metric=snapshot.streams?.[path.startsWith('standard/') && path.includes('_ee_pose')?path:key];
    const tr=row(body,['',label,'',type]);tr.children[2].append(element('code',topic));tr.children[2].title=topic;
    const checkbox=document.createElement('input');checkbox.type='checkbox';checkbox.checked=selection.get(topic) ?? true;
    checkbox.setAttribute('aria-label',`录制 ${label}`);checkbox.onchange=()=>{selection.set(topic,checkbox.checked);renderTopics(latest);};tr.firstChild.append(checkbox);
    if(checkbox.checked){selected++;if(live(metric) && metric.hz>=(['joints','commands','cartesian_feedback'].includes(key)?50:30))ready++;}rateCells(tr,metric,key);
    const interfaceRow=row($('configInterfaceRows'),[label,'',type]);interfaceRow.children[1].append(element('code',topic));
  }
  $('recordingReadinessBanner').className=`recording-readiness-banner ${ready===selected?'ready':'warn'}`;
  set('recordingReadinessText',ready===selected?`标准话题已达到监测频率（共勾选 ${selected} 个）`:`⚠️ 待录制话题检测未达标：${selected-ready} 个无数据或频率偏低（共勾选 ${selected} 个话题，已就绪 ${ready} 个）`);
  if($('recordingRows').contains(document.activeElement))return;
  const custom=$('recordingRows');custom.replaceChildren();
  customRules.forEach((rule,index)=>{
    const tr=row(custom,['','','','未监测','仅保存本地规则','','']);
    const check=document.createElement('input');check.type='checkbox';check.checked=rule.enabled!==false;check.onchange=()=>{rule.enabled=check.checked;saveRules();};tr.firstChild.append(check);
    tr.children[1].append(element('code',rule.topic));tr.children[2].textContent=rule.type;
    const rate=document.createElement('input');rate.type='number';rate.min='0';rate.value=rule.max_hz;rate.setAttribute('aria-label',`${rule.topic} 最大频率`);rate.onchange=()=>{rule.max_hz=Math.max(0,Number(rate.value)||0);saveRules();};tr.children[5].append(rate);
    const remove=element('button','删除');remove.onclick=()=>{customRules.splice(index,1);saveRules();renderTopics(latest);};tr.lastChild.append(remove);
  });
  if(!customRules.length)empty('recordingRows','暂无自定义录制规则',7);
}
function saveRules() {try{localStorage.setItem('hc.dashboard.rules',JSON.stringify(customRules));}catch{toast('浏览器不允许保存本地配置');}}
function addRule() {
  const topic=$('recordTopic').value.trim(),type=$('recordType').value;
  if(!/^\/[A-Za-z0-9_/]+$/.test(topic) || !type){toast('请输入有效的绝对话题名称并选择消息类型');return;}
  if(customRules.some(rule=>rule.topic===topic)){toast('此话题已在自定义列表中');return;}
  customRules.push({topic,type,max_hz:Math.max(0,Number($('recordMaxHz').value)||0),enabled:true});saveRules();if(latest)renderTopics(latest);toast('已添加本地录制规则');
}
function renderProfile(id) {
  const profile=runtime?.profiles?.find(item=>item.id===id);if(!profile)return;
  selectedProfile=id;set('activeProfileBadge',`当前：${runtime.active}`);
  const detail=$('profileDetail');detail.replaceChildren();
  const heading=element('div','','profile-heading');heading.append(element('strong',profile.display_name),element('code',profile.id));detail.append(heading);
  const summary=element('div','','profile-summary');
  for(const [label,value] of [['格式',profile.schema],['URDF',profile.urdf?.split('/').pop()],['关节',profile.joint_count],['自由关节',profile.free_joint_count],['任务',profile.task_count],['机械臂',profile.arm_count]]){
    const cell=document.createElement('span');cell.append(element('small',label),element('b',value ?? '--'));summary.append(cell);
  }detail.append(summary);summary.children[1].title=profile.urdf || '';
}
async function loadRuntime(notify=false) {
  try{
    const response=await fetch('/api/v1/runtime',{signal:AbortSignal.timeout(5000)});if(!response.ok)throw new Error('运行配置读取失败，请重启 Dashboard 服务');runtime=await response.json();
    const select=$('robotProfileSelect');const previous=selectedProfile || runtime.active;select.replaceChildren();
    runtime.profiles.forEach(profile=>select.append(new Option(`${profile.display_name}${profile.id===runtime.active?'（当前）':''}`,profile.id)));
    select.value=runtime.profiles.some(p=>p.id===previous)?previous:runtime.active;renderProfile(select.value);
    const active=runtime.profiles.find(profile=>profile.id===runtime.active);
    const fields={...Object.fromEntries(Object.entries(runtime.server).map(([k,v])=>[`server.${k}`,v])),...Object.fromEntries(Object.entries(runtime.ros).map(([k,v])=>[`ros.${k}`,v])),...Object.fromEntries(Object.entries(active?.vr || {}).map(([k,v])=>[`vr.${k}`,v]))};
    for(const [path,value] of Object.entries(fields)){const input=document.querySelector(`[data-path="${path}"]`);if(input)input.value=value;}
    const enabled=document.querySelector('[data-path="vr.enabled"]');enabled.checked=Boolean(active?.vr?.pose_port);
    document.querySelectorAll('[data-path]').forEach(input=>{if(!(input.dataset.path in fields) && input.type!=='checkbox')input.placeholder='当前未配置';});
    $('rosTopicOptions').replaceChildren();const types=new Set(['sensor_msgs/msg/JointState','geometry_msgs/msg/PoseStamped','std_msgs/msg/String']);
    runtime.topics.forEach(topic=>{$('rosTopicOptions').append(new Option(topic.name,topic.name));topic.types.forEach(type=>types.add(type));});
    $('recordType').replaceChildren();[...types].sort().forEach(type=>$('recordType').append(new Option(type,type)));$('recordType').value='sensor_msgs/msg/JointState';
    if(latest)render(latest);if(notify)toast(`发现 ${runtime.topics.length} 个 ROS 2 话题`);
  }catch(error){set('profileDetail',error.message);if(notify)toast(error.message);}
}
async function loadDatasets() {
  try{
    const response=await fetch('/api/v1/datasets',{signal:AbortSignal.timeout(5000)});if(!response.ok)throw new Error('数据集列表读取失败，请重启 Dashboard 服务');
    const data=await response.json();datasetItems=data.items;const total=data.items.reduce((sum,item)=>sum+(Number(item.bytes_written)||0),0);
    set('datasetsSummary',`共 ${data.items.length} 个数据集 · 总大小 ${formatBytes(total)} · 存储目录: ${data.directory}`);
    $('topicRecordingDirectory').value=data.directory;$('topicRecordingDirectory').readOnly=true;
    set('recordingMeta',`未录制 · 保存目录：${data.directory}`);
    $('datasetRows').replaceChildren();
    data.items.forEach(item=>{
      const duration=item.completed_at?Math.max(0,(Date.parse(item.completed_at)-Date.parse(item.created_at))/1000):null;
      const tr=row($('datasetRows'),['',item.name || item.dataset_id,item.metadata?.robot_id || '--',formatBytes(item.bytes_written),duration===null?'--':`${num(duration,1)} s`,item.message_count || 0,item.topics?.length || 0,item.created_at?new Date(item.created_at).toLocaleString():'--',item.state,'']);
      const checkbox=document.createElement('input');checkbox.type='checkbox';checkbox.setAttribute('aria-label',`选择 ${item.name || item.dataset_id}`);checkbox.onchange=updateDatasetSelection;tr.firstChild.append(checkbox);
      const detail=element('button','详情');detail.onclick=()=>showDataset(item);tr.lastChild.append(detail);
    });
    $('selectAllDatasets').checked=false;updateDatasetSelection();
    if(!data.items.length)empty('datasetRows','暂无录制数据集文件。当前目录尚未生成 MCAP 数据集。',10);
  }catch(error){set('datasetsSummary',error.message);empty('datasetRows',error.message,10);}
}
function updateDatasetSelection() {
 const boxes=[...document.querySelectorAll('#datasetRows input[type=checkbox]')], count=boxes.filter(box=>box.checked).length;set('selectedDatasetCount',count);$('selectAllDatasets').checked=boxes.length>0 && count===boxes.length;$('selectAllDatasets').indeterminate=count>0 && count<boxes.length;
}
function showDataset(item) {
  set('datasetDetailTitle',item.name || item.dataset_id);set('datasetDetailMeta',item.dataset_id);
  const summary=$('datasetDetailSummary');summary.replaceChildren();
  for(const [label,value] of [['文件大小',formatBytes(item.bytes_written)],['消息总数',item.message_count || 0],['录制话题',item.topics?.length || 0],['状态',item.state]]){
    const cell=document.createElement('span');cell.append(element('small',label),element('b',value));summary.append(cell);
  }
  $('datasetDetailChannelRows').replaceChildren();(item.topics || []).forEach(topic=>row($('datasetDetailChannelRows'),[topic,'--','--','--']));
  $('datasetDetailDialog').showModal();
}
initialize();

function renderPoses(snapshot) {
 const body=$('cartesianMonitorRows');body.replaceChildren();
 for(const [key,kind,metric] of [['cartesian','目标','cartesian_targets'],['cartesian_feedback','反馈','cartesian_feedback']]) {
  for(const group of snapshot[key]?.groups || []) {
   const valid=live(snapshot.streams?.[metric]) && group.valid!==false;
   const pose=group.pose || {};
   row(body,[group.name,kind,group.reference_frame,group.tip_frame,valid?'有效':'无效 / 超时',...['x','y','z','qx','qy','qz','qw'].map(k=>valid?num(pose[k]):'--')]);
  }
 }
 if(!body.children.length)empty('cartesianMonitorRows','等待末端目标与实际位姿反馈…',12);
}
