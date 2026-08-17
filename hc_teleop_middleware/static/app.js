const $ = (selector, root=document) => root.querySelector(selector);
const $$ = (selector, root=document) => [...root.querySelectorAll(selector)];
let config = null;
let statusData = null;
let ws = null;
let pendingVrPose = null;
let vrPoseFramePending = false;
const eventBuffer = [];
const titles = {overview:'运行概览', topics:'ROS 2 话题', config:'系统配置', events:'实时事件'};

function toast(message, error=false) {
  const el = $('#toast'); el.textContent = message; el.className = error ? 'show error' : 'show';
  clearTimeout(toast.timer); toast.timer = setTimeout(() => el.className='', 2600);
}
async function api(url, options={}) {
  const response = await fetch(url, {headers:{'Content-Type':'application/json'}, ...options});
  if (!response.ok) throw new Error((await response.text()) || `${response.status}`);
  return response.json();
}
function route() {
  const page = location.hash.slice(1) || 'overview';
  $$('.page').forEach(el => el.classList.toggle('hidden', el.id !== page));
  $$('nav a').forEach(el => el.classList.toggle('active', el.dataset.page === page));
  $('#pageTitle').textContent = titles[page] || titles.overview;
  if (page === 'topics') loadTopics();
}
function setState(id, state) {
  const el = $(id); el.textContent = state || '--'; el.className = state === 'running' ? 'running' : (state === 'error' ? 'error' : '');
}
function renderStatus(data) {
  statusData = data; const ros=data.ros, vr=data.vr, cam=data.camera;
  setState('#rosState', ros.state); $('#rosMeta').textContent = `${(ros.subscriptions||[]).length} 个订阅 · ${ros.messages||0} 条消息`;
  setState('#vrState', vr.state); $('#vrMeta').textContent = vr.timeout ? '位姿流已超时' : `v${vr.protocol_version||'--'} · ${vr.received||0} 包 · ${vr.controller_events||0} 事件`;
  setState('#cameraState', cam.state); $('#cameraMeta').textContent = cam.error || `${cam.capture_fps||0} FPS · ${cam.peers||0} peers`;
  $('#wsClients').textContent = data.websocket_clients;
  $('#vrPeer').textContent = vr.peer ? `${vr.peer[0]}:${vr.peer[1]}` : '未连接';
  for (const name of ['Head','Left','Right']) $('#track'+name).className = vr.tracking?.[name.toLowerCase()] ? 'online' : 'offline';
  $('#vrReceived').textContent=vr.received||0; $('#vrLost').textContent=vr.lost||0; $('#vrInvalid').textContent=vr.invalid||0; $('#vrSent').textContent=vr.sent||0;
  $('#vrProtocol').textContent=protocolLabel(vr.protocol_version);
  renderController('left',vr.inputs?.left||{}); renderController('right',vr.inputs?.right||{});
}
function protocolLabel(version) { return version===1?'协议 v1 · 不含手柄输入':`协议 v${version||'--'}`; }
function renderController(side,input) {
  const prefix=side[0].toUpperCase()+side.slice(1), held=input.held||[], clamp=value=>Math.max(0,Math.min(1,Number(value)||0)), number=value=>(Number(value)||0).toFixed(3), axis=value=>(value||[0,0]).map(number).join(', ');
  $(`#${side}Held`).textContent=held.length?held.join(' · '):'无按键';
  for(const name of ['Trigger','Grip']) { const value=clamp(input[name.toLowerCase()]); $(`#${side}${name}`).style.width=`${value*100}%`; $(`#${side}${name}Value`).textContent=number(value); }
  $(`#${side}PrimaryAxis`).textContent=axis(input.primary_axis); $(`#${side}SecondaryAxis`).textContent=axis(input.secondary_axis);
}
function renderVrPose(pose) {
  const tracking=pose.tracking||{};
  for(const name of ['Head','Left','Right']) $('#track'+name).className=tracking[name.toLowerCase()]?'online':'offline';
  $('#vrProtocol').textContent=protocolLabel(pose.protocol_version);
  renderController('left',pose.inputs?.left||{});
  renderController('right',pose.inputs?.right||{});
}
function queueVrPose(pose) {
  pendingVrPose=pose;
  if(vrPoseFramePending)return;
  vrPoseFramePending=true;
  requestAnimationFrame(()=>{
    vrPoseFramePending=false;
    if(pendingVrPose)renderVrPose(pendingVrPose);
    pendingVrPose=null;
  });
}
function eventRow(event) {
  const row=document.createElement('div'); row.className='event'+(event.payload?.level==='warning'?' warning':'');
  const time=document.createElement('time'); time.textContent=new Date(event.timestamp*1000).toLocaleTimeString();
  const kind=document.createElement('b'); kind.textContent=event.kind||'event';
  const body=document.createElement('span');
  if(event.kind==='vr_controller_event') { const p=event.payload||{}, actions=[]; if(p.pressed?.length)actions.push(`pressed=[${p.pressed.join(', ')}]`); if(p.released?.length)actions.push(`released=[${p.released.join(', ')}]`); body.textContent=`${p.side||'?'}: ${actions.join(' ')}`; }
  else body.textContent=event.payload?.message || event.payload?.reason || JSON.stringify(event.payload);
  row.append(time,kind,body); return row;
}
function addEvent(event) {
  if(event.kind==='ros_message' && event.topic==='/teleop/arm/status') renderTeleopStatus(event);
  eventBuffer.push(event); if(eventBuffer.length>300) eventBuffer.shift();
  for(const selector of ['#allEvents','#recentEvents']) { const box=$(selector); box.prepend(eventRow(event)); while(box.children.length>(selector==='#recentEvents'?8:300)) box.lastChild.remove(); }
}
function renderTeleopStatus(event) {
  try { const value=JSON.parse(event.payload?.data||'{}'), names={hold:'保持',base_waist:'底盘 + 腰部',arms_grippers:'双臂 + 夹爪',both:'全部并发',homing:'双臂回零'};
    const external=value.backend==='generic'||value.backend==='v23', solver=value.generic_controller||{}, healthy=solver.solver_fresh&&solver.command_fresh;
    const backendLabel=value.backend==='v23'?'重构 v2.3':value.backend==='generic'?'原版通用 IK':'旧版 PyBullet IK';
    $('#teleopMode').textContent=value.enabled?(names[value.mode]||value.mode):'已停用'; $('#teleopBackend').textContent=external?`${backendLabel}${healthy?' 正常':' 未响应'}`:backendLabel; $('#teleopLeft').textContent=value.left_clutch?'已离合':'保持'; $('#teleopRight').textContent=value.right_clutch?'已离合':'保持'; $('#teleopFeedback').textContent=value.feedback_fresh?'正常':'超时';
  } catch(_) {}
}
function connectWebSocket() {
  const scheme=location.protocol==='https:'?'wss':'ws'; ws=new WebSocket(`${scheme}://${location.host}/ws`);
  ws.onopen=()=>{ $('#connectionDot').className='online'; $('#connectionText').textContent='实时通道已连接'; };
  ws.onmessage=e=>{ try {
    const event=JSON.parse(e.data);
    if(event.kind==='connected')renderStatus(event.payload);
    else if(event.kind==='vr_pose')queueVrPose(event.payload||{});
    else addEvent(event);
  } catch (_) {} };
  ws.onclose=()=>{ $('#connectionDot').className='offline'; $('#connectionText').textContent='连接断开，正在重试'; setTimeout(connectWebSocket,1500); };
}
function getPath(object,path) { return path.split('.').reduce((value,key)=>value?.[key],object); }
function setPath(object,path,value) { const keys=path.split('.'); const last=keys.pop(); const target=keys.reduce((value,key)=>value[key],object); target[last]=value; }
function renderConfig() {
  $$('[data-path]').forEach(input=>{ const value=getPath(config,input.dataset.path); if(input.type==='checkbox') input.checked=Boolean(value); else input.value=value??''; });
  const body=$('#subscriptionRows'); body.textContent=''; (config.ros.subscriptions||[]).forEach(addSubscriptionRow);
}
function addSubscriptionRow(item={enabled:true,topic:'/topic',type:'std_msgs/msg/String',max_hz:0,outputs:['websocket']}) {
  const row=document.createElement('tr');
  const values=[['checkbox','enabled',item.enabled],['text','topic',item.topic],['text','type',item.type],['number','max_hz',item.max_hz],['checkbox','websocket',item.outputs.includes('websocket')],['checkbox','udp',item.outputs.includes('udp')]];
  for(const [type,name,value] of values) { const td=document.createElement('td'), input=document.createElement('input'); input.type=type; input.dataset.field=name; if(type==='checkbox') input.checked=value; else {input.value=value; if(type==='number') input.min=0;} td.append(input); row.append(td); }
  const action=document.createElement('td'), button=document.createElement('button'); button.className='remove'; button.textContent='删除'; button.onclick=()=>row.remove(); action.append(button); row.append(action); $('#subscriptionRows').append(row);
}
function collectConfig() {
  $$('[data-path]').forEach(input=>{ let value=input.type==='checkbox'?input.checked:input.value; if(input.type==='number') value=Number(value); setPath(config,input.dataset.path,value); });
  config.ros.subscriptions=$$('#subscriptionRows tr').map(row=>{ const field=name=>$(`[data-field="${name}"]`,row); return {enabled:field('enabled').checked,topic:field('topic').value.trim(),type:field('type').value.trim(),max_hz:Number(field('max_hz').value),outputs:['websocket','udp'].filter(name=>field(name).checked)}; }); return config;
}
async function loadTopics() {
  try { const topics=await api('/api/ros/topics'), active=new Set(config?.ros?.subscriptions?.filter(x=>x.enabled).map(x=>x.topic)||[]), body=$('#topicRows'); body.textContent='';
    for(const item of topics) { const row=document.createElement('tr'); for(const value of [item.topic,item.types.join(', '),active.has(item.topic)?'已订阅':'—']) { const td=document.createElement('td'); if(value===item.topic){const code=document.createElement('code');code.textContent=value;td.append(code);}else td.textContent=value; row.append(td); } body.append(row); }
    if(!topics.length) body.innerHTML='<tr><td colspan="3">暂无话题；请确认 ROS 环境和 ROS_DOMAIN_ID。</td></tr>';
  } catch(error) { toast(error.message,true); }
}
async function init() {
  route(); window.addEventListener('hashchange',route);
  try { config=await api('/api/config'); renderConfig(); const events=await api('/api/events?limit=100'); events.forEach(addEvent); } catch(error){toast(error.message,true);}
  connectWebSocket();
  const update=async()=>{try{renderStatus(await api('/api/status'));}catch(_){} }; update(); setInterval(update,1000);
  $('#refreshTopics').onclick=loadTopics; $('#addSubscription').onclick=()=>addSubscriptionRow(); $('#clearEvents').onclick=()=>{$('#allEvents').textContent='';};
  $('#saveConfig').onclick=async()=>{try{const result=await api('/api/config',{method:'PUT',body:JSON.stringify(collectConfig())}); config=result.config; renderConfig(); toast(result.server_restart_required?'已保存；HTTP 地址变更需重启进程':'配置已保存并应用');}catch(error){toast(error.message,true);}};
  $('#publishButton').onclick=async()=>{try{await api('/api/ros/publish',{method:'POST',body:JSON.stringify({topic:$('#pubTopic').value,type:$('#pubType').value,data:JSON.parse($('#pubData').value)})});toast('消息已进入 ROS 发布队列');}catch(error){toast(error.message,true);}};
  $('#stopButton').onclick=async()=>{if(!confirm('确认向机器人发送急停信号？'))return;try{await api('/api/safety/stop',{method:'POST',body:JSON.stringify({reason:'dashboard emergency stop'})});toast('急停信号已发送');}catch(error){toast(error.message,true);}};
  const setTeleop=async enabled=>{try{await api('/api/ros/publish',{method:'POST',body:JSON.stringify({topic:'/teleop/arm/enabled',type:'std_msgs/msg/Bool',data:{data:enabled}})});toast(enabled?'遥操作已使能；按住离合才会运动':'遥操作已停用');}catch(error){toast(error.message,true);}};
  $('#teleopEnable').onclick=()=>setTeleop(true); $('#teleopDisable').onclick=()=>setTeleop(false);
}
init();
