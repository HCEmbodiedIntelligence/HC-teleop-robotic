const $ = (selector, root=document) => root.querySelector(selector);
const $$ = (selector, root=document) => [...root.querySelectorAll(selector)];
let config = null;
let profilesData = {active:'', profiles:[], standard_topics:{}};
let ws = null;
let pendingVrPose = null;
let vrPoseFramePending = false;
const titles = {overview:'状态监控', topics:'话题录制', datasets:'数据集管理', config:'系统配置'};
let discoveredTopics = [];
let latestTopicHealth = {};
const jointMonitor = {command:new Map(), feedback:new Map()};
const standardTopicStandards = {
  vr_frame: { target_hz: 60, min_hz: 30 },
  joint_state: { target_hz: 100, min_hz: 50 },
  joint_target: { target_hz: 100, min_hz: 50 },
  joint_command: { target_hz: 100, min_hz: 50 },
  ee_target: { target_hz: 60, min_hz: 30 },
  ee_visual_target: { target_hz: 60, min_hz: 30 },
  ee_actual: { target_hz: 60, min_hz: 30 },
  solver_state: { target_hz: 100, min_hz: 50 },
  base_move: { target_hz: 60, min_hz: 20 },
};
const standardTopicTypes = {
  vr_frame: 'std_msgs/msg/String',
  joint_state:'sensor_msgs/msg/JointState', joint_target:'sensor_msgs/msg/JointState',
  joint_command:'sensor_msgs/msg/JointState', ee_target:'geometry_msgs/msg/PoseArray',
  ee_visual_target:'geometry_msgs/msg/PoseArray', ee_actual:'geometry_msgs/msg/PoseArray',
  solver_state:'sensor_msgs/msg/JointState', base_move:'std_msgs/msg/Float64MultiArray',
};

const STANDARD_ROS_TYPES = [
  'sensor_msgs/msg/JointState',
  'sensor_msgs/msg/Joy',
  'sensor_msgs/msg/Imu',
  'sensor_msgs/msg/Image',
  'sensor_msgs/msg/CompressedImage',
  'sensor_msgs/msg/LaserScan',
  'sensor_msgs/msg/PointCloud2',
  'geometry_msgs/msg/Pose',
  'geometry_msgs/msg/PoseStamped',
  'geometry_msgs/msg/PoseArray',
  'geometry_msgs/msg/TransformStamped',
  'geometry_msgs/msg/Twist',
  'geometry_msgs/msg/TwistStamped',
  'geometry_msgs/msg/WrenchStamped',
  'std_msgs/msg/String',
  'std_msgs/msg/Float64MultiArray',
  'std_msgs/msg/Float32MultiArray',
  'std_msgs/msg/Bool',
  'std_msgs/msg/Int32',
  'std_msgs/msg/Float64',
  'tf2_msgs/msg/TFMessage',
  'nav_msgs/msg/Odometry',
  'trajectory_msgs/msg/JointTrajectory',
];

function toast(message, error=false) {
  const el = $('#toast');
  el.textContent = message;
  el.className = error ? 'show error' : 'show';
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => el.className='', 3200);
}

async function api(url, options={}) {
  const request = {...options};
  if (request.body && !(request.body instanceof FormData)) {
    request.headers = {'Content-Type':'application/json', ...(request.headers||{})};
  }
  const response = await fetch(url, request);
  if (!response.ok) throw new Error((await response.text()) || `${response.status}`);
  return response.json();
}

function route() {
  const requested = location.hash.slice(1) || 'overview';
  const page = titles[requested] ? requested : 'overview';
  $$('.page').forEach(el => el.classList.toggle('hidden', el.id !== page));
  $$('nav a').forEach(el => el.classList.toggle('active', el.dataset.page === page));
  $('#pageTitle').textContent = titles[page];
  if (page === 'topics') loadTopics();
  if (page === 'datasets') loadDatasets();
  if (page === 'config') loadProfiles($('#robotProfileSelect')?.value);
}

function setState(id, state) {
  const el = $(id);
  if(!el)return;
  el.textContent = state || '--';
  el.className = state === 'running' || state === '正常'
    ? 'running'
    : (state === 'error' || state === '无数据' ? 'error' : (state === '偏低' ? 'warning' : ''));
}

function setText(id,value) {
  const el=$(id);
  if(el)el.textContent=value;
}

function messageState(info) {
  if(!info?.has_data)return ['无数据','0.0 Hz','error'];
  if(info.state==='low_rate')return ['偏低',`${Number(info.hz||0).toFixed(1)} Hz`,'warning'];
  return ['正常',`${Number(info.hz||0).toFixed(1)} Hz`,'running'];
}

function renderCommandMux(ros) {
  const mux=ros.command_mux||{};
  const source=mux.source||'vr';
  const sourceLabel=source==='exoskeleton'?'外骨骼':'VR / IK';
  setText('#controlSourceState',sourceLabel);
  setText('#controlSourceMeta',mux.output_enabled===false?'急停门控已关闭':'命令输出已使能');
  const sourceState=$('#controlSourceState');
  if(sourceState)sourceState.className=mux.output_enabled===false?'error':'running';
  for(const button of $$('.source-switch button')) {
    button.classList.toggle('active',button.dataset.source===source);
  }
  if($('#homeButton'))$('#homeButton').disabled=source!=='vr';
  setText('#vrCommandTopic',mux.vr_topic||'/hc_teleop/joint_cmd_vr');
  setText('#exoCommandTopic',mux.exoskeleton_topic||'/hc_teleop/joint_cmd_exoskeleton');
  setText('#outputCommandTopic',mux.output_topic||'/hc_teleop/joint_cmd');
  setText('#muxForwarded',String(mux.forwarded||0));
  const sourceStatus=(name,label)=>{
    const age=mux.last_received_age?.[name];
    const count=mux.received?.[name]||0;
    return age!==null&&age!==undefined&&age<=1.8?`${label} · ${count} 帧`:`无数据 · ${count} 帧`;
  };
  setText('#vrSourceStatus',sourceStatus('vr','输入正常'));
  setText('#exoSourceStatus',sourceStatus('exoskeleton','输入正常'));
  const badge=$('#muxSafetyBadge');
  if(badge){
    badge.textContent=mux.output_enabled===false?'急停：禁止转发':`${sourceLabel} 已选中`;
    badge.className=mux.output_enabled===false?'tag danger-tag':'tag';
  }
}

function renderJointMonitor() {
  const body=$('#jointMonitorRows');
  if(!body)return;
  const names=[...new Set([...jointMonitor.command.keys(),...jointMonitor.feedback.keys()])];
  body.textContent='';
  if(!names.length){
    const row=document.createElement('tr'),cell=document.createElement('td');
    cell.colSpan=6; cell.className='empty-monitor'; cell.textContent='等待 JointState 消息…';
    row.append(cell); body.append(row); setText('#jointMonitorCount','0 个关节'); return;
  }
  const format=value=>Number.isFinite(value)?value.toFixed(4):'--';
  const degrees=value=>Number.isFinite(value)?(value*180/Math.PI).toFixed(2):'--';
  for(const name of names){
    const command=jointMonitor.command.get(name);
    const feedback=jointMonitor.feedback.get(name);
    const error=Number.isFinite(command)&&Number.isFinite(feedback)?command-feedback:NaN;
    const row=document.createElement('tr');
    for(const value of [name,format(command),degrees(command),format(feedback),degrees(feedback),degrees(error)]){
      const cell=document.createElement('td'); cell.textContent=value; row.append(cell);
    }
    if(Number.isFinite(error)&&Math.abs(error)>0.08)row.className='joint-error';
    body.append(row);
  }
  setText('#jointMonitorCount',`${names.length} 个关节`);
}

function updateJointMonitor(event,kind) {
  const names=event.payload?.name||[], positions=event.payload?.position||[];
  if(!Array.isArray(names)||!Array.isArray(positions)||names.length!==positions.length)return;
  const target=kind==='command'?jointMonitor.command:jointMonitor.feedback;
  target.clear();
  names.forEach((name,index)=>{
    const value=Number(positions[index]);
    if(Number.isFinite(value))target.set(String(name),value);
  });
  renderJointMonitor();
}

function renderStatus(data) {
  const ros=data.ros||{}, vr=data.vr||{}, cam=data.camera||{};
  const domain = ros.domain_id ?? config?.ros?.domain_id ?? 0;
  setState('#rosState',ros.state);
  setText('#rosMeta',`${(ros.subscriptions||[]).length} 个订阅 · ${ros.messages||0} 条消息 · Domain ${domain}`);
  renderCommandMux(ros);
  setState('#vrState', vr.state);
  setText('#vrMeta',vr.timeout ? '位姿流已超时' : `v${vr.protocol_version||'--'} · ${vr.received||0} 包`);
  setState('#cameraState', cam.state);
  setText(
    '#cameraMeta',
    cam.error || `接收 ${cam.capture_fps||0} FPS · WebRTC 发送 ${cam.webrtc_send_fps||0} FPS · ${cam.peers||0} peers`
  );
  setText('#wsClients',data.websocket_clients||0);
  const recording=data.recording||{};
  const isRecording=Boolean(recording.recording);
  if($('#recordingBadge')) {
    $('#recordingBadge').className = isRecording ? 'recording-badge active' : 'recording-badge';
    $('#recordingBadgeText').textContent = isRecording ? `正在录制 (${recording.messages||0} 条)` : '未录制';
  }
  if($('#startRecordingBtn')) $('#startRecordingBtn').disabled = isRecording;
  if($('#stopRecordingBtn')) $('#stopRecordingBtn').disabled = !isRecording;
  if($('#recordingMeta')) {
    $('#recordingMeta').textContent = isRecording
      ? `正在录制：已写入 ${recording.messages||0} 条消息到 ${recording.path||''}`
      : `未录制 · 保存目录：${config?.ros?.recording?.directory || 'runtime/topic_recordings'}`;
  }
  setText('#vrPeer',vr.peer ? `${vr.peer[0]}:${vr.peer[1]}` : 'VR 未连接');
  for (const name of ['Head','Left','Right']) {
    const indicator=$('#track'+name);
    if(indicator)indicator.className = vr.tracking?.[name.toLowerCase()] ? 'online' : 'offline';
  }
  if (ros.topic_health) {
    latestTopicHealth = ros.topic_health;
    const commandInfo=messageState(ros.topic_health['/hc_teleop/joint_cmd']);
    const feedbackInfo=messageState(ros.topic_health['/hc_teleop/joint_states']);
    setState('#jointCommandState',commandInfo[0]);
    setText('#jointCommandMeta',commandInfo[1]+' · /hc_teleop/joint_cmd');
    setState('#jointFeedbackState',feedbackInfo[0]);
    setText('#jointFeedbackMeta',feedbackInfo[1]+' · /hc_teleop/joint_states');
    updateReadinessBanner();
    if (window.location.hash === '#topics' || (!window.location.hash && $('#topics')?.classList.contains('active'))) {
      renderStandardTopics();
      renderRecordingRules();
    }
  }
  const replay = data.replay || {};
  const isReplaying = Boolean(replay.is_active || replay.state === 'playing' || replay.state === 'paused');
  const replayNeedsReset = Boolean(replay.requires_reset);
  const replayPanel = $('#replayControlPanel');
  if (replayPanel) {
    if (isReplaying || replayNeedsReset) {
      replayPanel.classList.remove('hidden');
      if ($('#replayStatusBadge')) {
        $('#replayStatusBadge').className = replayNeedsReset ? 'recording-badge' : (replay.state === 'paused' ? 'recording-badge' : 'recording-badge active');
        $('#replayStatusText').textContent = replayNeedsReset
          ? (replay.state === 'error' ? '重放异常 · 请按 A 恢复' : '重放结束 · 请按 A 恢复遥操作')
          : (replay.state === 'paused' ? '已暂停' : '正在重放');
      }
      if ($('#replayFileName')) $('#replayFileName').textContent = replay.filename || '';
      if ($('#pauseResumeReplayBtn')) {
        $('#pauseResumeReplayBtn').textContent = replay.state === 'paused' ? '▶️ 继续' : '⏸️ 暂停';
        $('#pauseResumeReplayBtn').disabled = replayNeedsReset;
      }
      if ($('#stopReplayBtn')) $('#stopReplayBtn').disabled = replayNeedsReset;
      if ($('#replayProgressBar')) $('#replayProgressBar').style.width = `${replay.progress || 0}%`;
      if ($('#replayProgressText')) $('#replayProgressText').textContent = `${(replay.progress || 0).toFixed(1)}%`;
      if ($('#replayTimeText')) $('#replayTimeText').textContent = `${(replay.current_time_sec || 0).toFixed(1)}s / ${(replay.duration_sec || 0).toFixed(1)}s`;
      if ($('#replayMsgText')) $('#replayMsgText').textContent = `${(replay.current_message || 0).toLocaleString()} / ${(replay.total_messages || 0).toLocaleString()}`;
      if ($('#replaySpeedSelect')) $('#replaySpeedSelect').value = String(replay.speed || 1.0);
      if ($('#replayLoopToggle')) $('#replayLoopToggle').checked = Boolean(replay.loop);
    } else {
      replayPanel.classList.add('hidden');
    }
  }
}

function protocolLabel(version) {
  return version===1 ? '协议 v1 · 不含手柄输入' : `协议 v${version||'--'}`;
}

function renderController(side,input) {
  const held=input.held||[];
  const clamp=value=>Math.max(0,Math.min(1,Number(value)||0));
  const number=value=>(Number(value)||0).toFixed(3);
  const axis=value=>(value||[0,0]).map(number).join(', ');
  if(!$(`#${side}Held`))return;
  for(const name of ['Trigger','Grip']) {
    const value=clamp(input[name.toLowerCase()]);
    $(`#${side}${name}`).style.width=`${value*100}%`;
    $(`#${side}${name}Value`).textContent=number(value);
  }
  $(`#${side}PrimaryAxis`).textContent=axis(input.primary_axis);
  $(`#${side}SecondaryAxis`).textContent=axis(input.secondary_axis);
}

function renderVrPose(pose) {
  const tracking=pose.tracking||{};
  for(const name of ['Head','Left','Right']) {
    const indicator=$('#track'+name);
    if(indicator)indicator.className=tracking[name.toLowerCase()]?'online':'offline';
  }
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

function renderTeleopStatus(event) {
  try {
    const value=JSON.parse(event.payload?.data||'{}');
    const names={hold:'保持',base_waist:'底盘 + 腰部',arms_grippers:'双臂 + 夹爪',both:'全部并发',homing:'双臂 + 腰部回零'};
    const rearmNames={wait_feedback:'等待回零反馈',wait_reset:'准备复位 IK',wait_ack:'等待 IK 复位确认',wait_solver:'等待复位后的新 IK 解',wait_release:'请松开右 Grip',wait_press:'请重新按下右 Grip'};
    const external=value.backend==='generic'||value.backend==='v23';
    const solver=value.generic_controller||{};
    const healthy=solver.solver_fresh&&solver.command_fresh;
    const backendLabel=value.backend==='v23'?'重构 v2.3':value.backend==='generic'?'原版通用 IK':'旧版 PyBullet IK';
    const modeName=value.mode==='ik_rearming'?(rearmNames[solver.rearm_state]||'IK 重新武装'):(names[value.mode]||value.mode);
    setText('#teleopMode',value.enabled?modeName:'已停用');
    setText('#teleopBackend',external?`${backendLabel}${healthy?' 正常':' 未响应'}`:backendLabel);
    setText('#teleopLeft',value.left_clutch?'已离合':'保持');
    setText('#teleopRight',value.right_clutch?'已离合':'保持');
    setText('#teleopFeedback',value.feedback_fresh?'正常':'超时');
  } catch(_) {}
}

function connectWebSocket() {
  const scheme=location.protocol==='https:'?'wss':'ws';
  ws=new WebSocket(`${scheme}://${location.host}/ws`);
  ws.onopen=()=>{
    $('#connectionDot').className='online';
    $('#connectionText').textContent='实时状态已连接';
  };
  ws.onmessage=event=>{
    try {
      const value=JSON.parse(event.data);
      if(value.kind==='connected')renderStatus(value.payload);
      else if(value.kind==='vr_pose')queueVrPose(value.payload||{});
      else if(value.kind==='ros_message'&&value.topic==='/teleop/arm/status')renderTeleopStatus(value);
      else if(value.kind==='ros_message'&&value.topic==='/hc_teleop/joint_cmd')updateJointMonitor(value,'command');
      else if(value.kind==='ros_message'&&value.topic==='/hc_teleop/joint_states')updateJointMonitor(value,'feedback');
      else if(value.kind==='control_source')api('/api/status').then(renderStatus).catch(()=>{});
      else if(String(value.kind||'').startsWith('replay_') || value.kind==='safety_resume') {
        api('/api/status').then(renderStatus).catch(()=>{});
        if(value.payload?.message) toast(value.payload.message, value.kind==='replay_error');
      }
      else if(value.kind==='recording_marked' || value.kind==='recording_mark_error') {
        if(value.payload?.message) toast(value.payload.message, value.kind==='recording_mark_error');
        if($('#datasetRows')) loadDatasets().catch(()=>{});
      }
    } catch (_) {}
  };
  ws.onclose=()=>{
    $('#connectionDot').className='offline';
    $('#connectionText').textContent='连接断开，正在重试';
    setTimeout(connectWebSocket,1500);
  };
}

function getPath(object,path) {
  return path.split('.').reduce((value,key)=>value?.[key],object);
}

function setPath(object,path,value) {
  const keys=path.split('.');
  const last=keys.pop();
  const target=keys.reduce((value,key)=>value[key],object);
  target[last]=value;
}

function renderConfig() {
  $$('[data-path]').forEach(input=>{
    const value=getPath(config,input.dataset.path);
    if(input.type==='checkbox') input.checked=Boolean(value);
    else input.value=value??'';
  });
  if($('#topicRecordingDirectory')) {
    $('#topicRecordingDirectory').value = config?.ros?.recording?.directory || 'runtime/topic_recordings';
  }
  renderRecordingRules();
  renderStandardTopics();
  renderConfigInterface();
}

function standardTopicSet() {
  return new Set(Object.values(profilesData.standard_topics||{}));
}

function customRecordingRules() {
  const standard = standardTopicSet();
  return (config?.ros?.subscriptions||[]).filter(item=>(item.outputs||[]).includes('record') && !standard.has(item.topic));
}

function setTopicRecording(topic,type,enabled,maxHz=0) {
  const subscriptions=config.ros.subscriptions||(config.ros.subscriptions=[]);
  let item=subscriptions.find(value=>value.topic===topic);
  if(enabled) {
    if(!item) {
      item={enabled:true,topic,type,max_hz:Number(maxHz)||0,outputs:[]};
      subscriptions.push(item);
    }
    item.type=type||item.type;
    item.enabled=true;
    item.max_hz=Number(maxHz)||0;
    if(!item.outputs.includes('record'))item.outputs.push('record');
  } else if(item) {
    item.outputs=(item.outputs||[]).filter(output=>output!=='record');
    if(!item.outputs.length)subscriptions.splice(subscriptions.indexOf(item),1);
  }
  renderRecordingRules();
  renderStandardTopics();
}

function renderTopicHealthBadge(topic, defaultMinHz = 10, defaultTargetHz = 50) {
  const info = latestTopicHealth[topic];
  const hz = Number(info?.hz || 0);
  const minHz = Number(info?.min_hz || defaultMinHz || 10);
  const targetHz = Number(info?.target_hz || defaultTargetHz || 50);
  const hasData = Boolean(info?.has_data);
  const state = info?.state || (!hasData ? 'no_data' : (minHz > 0 && hz < minHz ? 'low_rate' : 'ok'));

  let badgeClass = 'status-none';
  let badgeText = '无消息 (0 Hz)';
  let hzClass = 'none';

  if (state === 'ok') {
    badgeClass = 'status-ok';
    badgeText = `达标 (${hz.toFixed(1)} Hz)`;
    hzClass = 'ok';
  } else if (state === 'low_rate') {
    badgeClass = 'status-low';
    badgeText = `偏低 (${hz.toFixed(1)} Hz)`;
    hzClass = 'low';
  }

  const rateCell = document.createElement('td');
  const rateSpan = document.createElement('span');
  rateSpan.className = `topic-rate ${hzClass}`;
  rateSpan.textContent = hasData ? `${hz.toFixed(1)} Hz` : '0.0 Hz';
  rateCell.append(rateSpan);

  const stdCell = document.createElement('td');
  const stdSpan = document.createElement('span');
  stdSpan.className = 'topic-target-rate';
  stdSpan.textContent = minHz > 0 ? `≥ ${minHz.toFixed(0)} Hz (${targetHz.toFixed(0)} Hz)` : '--';
  stdCell.append(stdSpan);

  const statusCell = document.createElement('td');
  const badgeSpan = document.createElement('span');
  badgeSpan.className = `topic-status-badge ${badgeClass}`;
  const dot = document.createElement('i');
  badgeSpan.append(dot, document.createTextNode(badgeText));
  statusCell.append(badgeSpan);

  return { rateCell, stdCell, statusCell };
}

function updateReadinessBanner() {
  const banner = $('#recordingReadinessBanner');
  const text = $('#recordingReadinessText');
  if (!banner || !text || !config) return;

  const recordingSubs = (config.ros?.subscriptions || []).filter(
    item => item.enabled !== false && (item.outputs || []).includes('record')
  );

  if (!recordingSubs.length) {
    banner.className = 'recording-readiness-banner warn';
    text.textContent = '⚠️ 当前未勾选任何需要录制的话题，请在下方勾选内置或自定义话题';
    return;
  }

  let noDataCount = 0;
  let lowRateCount = 0;
  let okCount = 0;

  for (const item of recordingSubs) {
    const info = latestTopicHealth[item.topic];
    if (!info || info.state === 'no_data') {
      noDataCount++;
    } else if (info.state === 'low_rate') {
      lowRateCount++;
    } else {
      okCount++;
    }
  }

  if (noDataCount === 0 && lowRateCount === 0) {
    banner.className = 'recording-readiness-banner ready';
    text.textContent = `🟢 录制检测就绪：已勾选的 ${recordingSubs.length} 个话题全部正常接收且频率达标`;
  } else {
    banner.className = 'recording-readiness-banner warn';
    const problems = [];
    if (noDataCount > 0) problems.push(`${noDataCount} 个无数据`);
    if (lowRateCount > 0) problems.push(`${lowRateCount} 个频率不足`);
    text.textContent = `⚠️ 待录制话题检测未达标：${problems.join('，')}（共勾选 ${recordingSubs.length} 个话题，已就绪 ${okCount} 个）`;
  }
}

function renderRecordingRules() {
  const body=$('#recordingRows');
  if(!body||!config)return;
  body.textContent='';
  for(const item of customRecordingRules()) {
    const row=document.createElement('tr');
    const enabledCell=document.createElement('td'), enabled=document.createElement('input');
    enabled.type='checkbox'; enabled.checked=item.enabled!==false;
    enabled.onchange=()=>{item.enabled=enabled.checked;};
    enabledCell.append(enabled);
    const topic=document.createElement('td'), topicCode=document.createElement('code');
    topicCode.textContent=item.topic; topic.append(topicCode);
    const type=document.createElement('td'); type.textContent=item.type;

    const { rateCell, stdCell } = renderTopicHealthBadge(item.topic, Number(item.min_hz)||5, Number(item.target_hz)||10);

    const rateCellInput=document.createElement('td'), rate=document.createElement('input');
    rate.type='number'; rate.min='0'; rate.value=item.max_hz||0;
    rate.onchange=()=>{item.max_hz=Number(rate.value)||0;}; rateCellInput.append(rate);
    const action=document.createElement('td'), remove=document.createElement('button');
    remove.className='remove'; remove.textContent='删除';
    remove.onclick=()=>setTopicRecording(item.topic,item.type,false);
    action.append(remove); row.append(enabledCell,topic,type,rateCell,stdCell,rateCellInput,action); body.append(row);
  }
  if(!body.children.length) {
    const row=document.createElement('tr'), cell=document.createElement('td');
    cell.colSpan=7; cell.textContent='暂无自定义录制消息，可从下方添加。'; row.append(cell); body.append(row);
  }
}

function collectConfig() {
  $$('[data-path]').forEach(input=>{
    let value=input.type==='checkbox'?input.checked:input.value;
    if(input.type==='number') value=Number(value);
    setPath(config,input.dataset.path,value);
  });
  if($('#topicRecordingDirectory')) {
    if(!config.ros) config.ros = {};
    if(!config.ros.recording) config.ros.recording = {};
    config.ros.recording.directory = $('#topicRecordingDirectory').value.trim() || 'runtime/topic_recordings';
  }
  return config;
}

function renderStandardTopics() {
  const labels={
    vr_frame:'VR Frame', joint_state:'关节反馈', joint_target:'控制器关节目标', joint_command:'机器人关节命令',
    ee_target:'控制器末端目标', ee_visual_target:'可视化末端目标', ee_actual:'实际末端位姿',
    solver_state:'求解器状态', base_move:'底盘运动',
  };
  const body=$('#standardTopicRows');
  if(!body)return;
  body.textContent='';
  for(const [key,topic] of Object.entries(profilesData.standard_topics||{})) {
    const type=standardTopicTypes[key]||'std_msgs/msg/String';
    const std = standardTopicStandards[key] || { target_hz: 50, min_hz: 20 };
    const row=document.createElement('tr');
    const recordCell=document.createElement('td'), record=document.createElement('input');
    record.type='checkbox';
    record.checked=(config?.ros?.subscriptions||[]).some(item=>item.topic===topic&&item.enabled!==false&&(item.outputs||[]).includes('record'));
    record.onchange=()=>setTopicRecording(topic,type,record.checked);
    recordCell.append(record);
    const purpose=document.createElement('td');
    purpose.textContent=labels[key]||key;
    const value=document.createElement('td'), code=document.createElement('code');
    code.textContent=topic;
    value.append(code);
    const typeCell=document.createElement('td');
    typeCell.textContent=type;

    const { rateCell, stdCell, statusCell } = renderTopicHealthBadge(topic, std.min_hz, std.target_hz);
    row.append(recordCell,purpose,value,typeCell,rateCell,stdCell,statusCell);
    body.append(row);
  }
}

function renderConfigInterface() {
  const labels={
    vr_frame:'VR Frame', joint_state:'关节反馈', joint_target:'控制器关节目标', joint_command:'机器人关节命令',
    ee_target:'控制器末端目标', ee_visual_target:'可视化末端目标', ee_actual:'实际末端位姿',
    solver_state:'求解器状态', base_move:'底盘运动',
  };
  const body=$('#configInterfaceRows');
  if(!body)return;
  body.textContent='';
  for(const [key,topic] of Object.entries(profilesData.standard_topics||{})) {
    const row=document.createElement('tr');
    const purpose=document.createElement('td');
    purpose.textContent=labels[key]||key;
    const value=document.createElement('td'), code=document.createElement('code');
    code.textContent=topic;
    value.append(code);
    const typeCell=document.createElement('td');
    typeCell.textContent=standardTopicTypes[key]||'std_msgs/msg/String';
    row.append(purpose,value,typeCell);
    body.append(row);
  }
}

function renderProfileDetail() {
  const select=$('#robotProfileSelect');
  const profile=profilesData.profiles.find(item=>item.id===select.value);
  const detail=$('#profileDetail');
  detail.textContent='';
  const isCurrent = profile && profile.id === profilesData.active;
  $('#activateProfile').disabled=!profile||isCurrent||profile.schema==='invalid';
  if($('#deleteProfile')) $('#deleteProfile').disabled=!profile;
  $('#activeProfileBadge').textContent=profilesData.active?`当前：${profilesData.active}`:'未选择';
  if(!profile) {
    detail.textContent='暂无可用机器人配置，请先导入 URDF 和 YAML。';
    return;
  }
  const title=document.createElement('strong');
  title.textContent=profile.display_name||profile.id;
  const id=document.createElement('code');
  id.textContent=profile.id;
  const summary=document.createElement('div');
  summary.className='profile-summary';
  const values=[
    ['格式',profile.schema||'--'],
    ['URDF',profile.urdf||'--'],
    ['关节',profile.joint_count??'--'],
    ['自由关节',profile.free_joint_count??'--'],
    ['任务',profile.task_count??'--'],
    ['机械臂',profile.arm_count??'--'],
  ];
  for(const [label,value] of values) {
    const cell=document.createElement('span');
    const small=document.createElement('small'); small.textContent=label;
    const text=document.createElement('b'); text.textContent=String(value);
    cell.append(small,text); summary.append(cell);
  }
  const heading=document.createElement('div');
  heading.className='profile-heading';
  heading.append(title,id);
  detail.append(heading,summary);
  for(const warning of profile.warnings||[]) {
    const message=document.createElement('p');
    message.className='profile-warning';
    message.textContent=warning;
    detail.append(message);
  }
}

function renderProfiles(preselect='') {
  const select=$('#robotProfileSelect');
  const desired=preselect||select.value||profilesData.active;
  select.textContent='';
  for(const profile of profilesData.profiles) {
    const option=document.createElement('option');
    option.value=profile.id;
    option.textContent=`${profile.display_name||profile.id}${profile.id===profilesData.active?'（当前）':''}`;
    option.disabled=profile.schema==='invalid';
    select.append(option);
  }
  if(profilesData.profiles.some(profile=>profile.id===desired))select.value=desired;
  renderStandardTopics();
  renderConfigInterface();
  renderProfileDetail();
}

async function loadProfiles(preselect='') {
  try {
    profilesData=await api('/api/robot-profiles');
    profilesData.standard_topics={vr_frame:config?.vr?.data_topic||'/vrdata',...profilesData.standard_topics};
    renderProfiles(preselect);
  } catch(error) {
    toast(`机器人配置加载失败：${error.message}`,true);
  }
}

function renderRecordTypeOptions(selectedType='') {
  const select=$('#recordType');
  if(!select)return;
  const current=selectedType||select.value||STANDARD_ROS_TYPES[0];
  const types=new Set(STANDARD_ROS_TYPES);
  for(const item of discoveredTopics) {
    for(const type of item.types||[])types.add(type);
  }
  select.textContent='';
  for(const type of [...types].sort()) {
    const option=document.createElement('option');
    option.value=type;
    option.textContent=type;
    select.append(option);
  }
  if(types.has(current))select.value=current;
}

async function loadTopics() {
  try {
    discoveredTopics=await api('/api/ros/topics');
    const topicOptions=$('#rosTopicOptions');
    topicOptions.textContent='';
    for(const item of discoveredTopics) {
      const option=document.createElement('option');
      option.value=item.topic;
      topicOptions.append(option);
    }
    renderRecordTypeOptions();
    toast(discoveredTopics.length?`发现 ${discoveredTopics.length} 个 ROS 2 话题`:'未发现 ROS 2 话题');
  } catch(error) { toast(error.message,true); }
}

let datasetsData = { files: [] };

function updateSelectedDatasetCount() {
  const checked = $$('#datasetRows input[type=checkbox]:checked');
  const count = checked.length;
  if ($('#selectedDatasetCount')) $('#selectedDatasetCount').textContent = count;
  if ($('#batchDeleteDatasets')) $('#batchDeleteDatasets').disabled = count === 0;
  const all = $$('#datasetRows input[type=checkbox]:not(:disabled)');
  if ($('#selectAllDatasets')) {
    $('#selectAllDatasets').checked = all.length > 0 && checked.length === all.length;
    $('#selectAllDatasets').indeterminate = checked.length > 0 && checked.length < all.length;
  }
}

async function loadDatasets() {
  const body = $('#datasetRows');
  const summary = $('#datasetsSummary');
  if (!body) return;
  body.textContent = '';
  try {
    datasetsData = await api('/api/recordings');
    const files = datasetsData.files || [];
    if (summary) {
      summary.textContent = `共 ${datasetsData.total_count || 0} 个数据集 · 总大小 ${datasetsData.total_size_human || '0 B'} · 存储目录: ${datasetsData.directory || ''}`;
    }
    if ($('#selectAllDatasets')) $('#selectAllDatasets').checked = false;
    updateSelectedDatasetCount();

    if (!files.length) {
      const row = document.createElement('tr');
      const cell = document.createElement('td');
      cell.colSpan = 10;
      cell.style.textAlign = 'center';
      cell.style.color = 'var(--muted)';
      cell.style.padding = '24px 0';
      cell.textContent = '暂无录制数据集文件。可在“话题录制”页面点击“开始录制”生成 MCAP 数据集。';
      row.append(cell);
      body.append(row);
      return;
    }

    for (const file of files) {
      const row = document.createElement('tr');

      const selectCell = document.createElement('td');
      const check = document.createElement('input');
      check.type = 'checkbox';
      check.dataset.filename = file.filename;
      check.disabled = Boolean(file.is_current);
      check.onchange = updateSelectedDatasetCount;
      selectCell.append(check);

      const nameCell = document.createElement('td');
      const nameCode = document.createElement('code');
      nameCode.textContent = `${file.marked ? '★ ' : ''}${file.filename}`;
      nameCell.append(nameCode);

      const profileCell = document.createElement('td');
      const profileTag = document.createElement('span');
      profileTag.className = 'tag';
      profileTag.textContent = file.profile_display_name || file.profile_id || '未知（旧数据）';
      profileTag.title = file.profile_id
        ? `Profile: ${file.profile_id}${file.robot_name ? ` · Robot: ${file.robot_name}` : ''}`
        : '该文件没有 HC Teleop Profile metadata';
      if (!file.profile_id) {
        profileTag.style.color = 'var(--muted)';
        profileTag.style.borderColor = 'var(--border)';
      }
      profileCell.append(profileTag);

      const sizeCell = document.createElement('td');
      sizeCell.style.font = '12px monospace';
      sizeCell.textContent = file.size_human;

      const durationCell = document.createElement('td');
      durationCell.style.font = '12px monospace';
      durationCell.textContent = file.duration_human || '--';

      const msgCountCell = document.createElement('td');
      msgCountCell.style.font = '12px monospace';
      msgCountCell.style.color = 'var(--cyan)';
      msgCountCell.textContent = file.message_count ? `${file.message_count.toLocaleString()} 条` : (file.is_current ? '录制中…' : '--');

      const topicsCell = document.createElement('td');
      topicsCell.style.fontSize = '12px';
      if (file.topic_count) {
        topicsCell.textContent = `${file.topic_count} 个话题 (${file.avg_rate_hz || 0} Hz)`;
      } else {
        topicsCell.textContent = file.is_current ? '录制中…' : '--';
      }

      const timeCell = document.createElement('td');
      timeCell.style.fontSize = '12px';
      timeCell.textContent = file.created_at || file.modified_at;

      const statusCell = document.createElement('td');
      if (file.is_current) {
        statusCell.innerHTML = `<span class="recording-badge active" style="padding:2px 6px;font-size:10px;"><i></i>${file.marked ? '已标记 · ' : ''}正在写入</span>`;
      } else if (file.marked) {
        statusCell.innerHTML = '<span class="tag" style="border-color:#f5a344;color:#f5a344;">★ 已标记</span>';
      } else {
        statusCell.innerHTML = '<span class="tag" style="border-color:#1c664b;color:#55d98b;">就绪</span>';
      }

      const actionCell = document.createElement('td');
      actionCell.style.textAlign = 'right';
      const group = document.createElement('div');
      group.className = 'btn-group';

      if (!file.is_current && (file.channels?.length || file.message_count)) {
        const replayBtn = document.createElement('button');
        replayBtn.className = 'primary btn-sm';
        replayBtn.textContent = '重放';
        replayBtn.onclick = () => openStartReplayDialog(file);
        group.append(replayBtn);

        const detailBtn = document.createElement('button');
        detailBtn.className = 'btn-sm';
        detailBtn.textContent = '详情';
        detailBtn.onclick = () => showDatasetDetails(file);
        group.append(detailBtn);
      }

      const downloadLink = document.createElement('a');
      downloadLink.className = 'btn-sm';
      downloadLink.href = `/api/recordings/${encodeURIComponent(file.filename)}/download`;
      downloadLink.setAttribute('download', file.filename);
      downloadLink.textContent = '下载';

      const deleteBtn = document.createElement('button');
      deleteBtn.className = 'danger btn-sm';
      deleteBtn.textContent = '删除';
      deleteBtn.disabled = Boolean(file.is_current);
      deleteBtn.onclick = async () => {
        if (!confirm(`确认删除数据集文件 “${file.filename}”？`)) return;
        try {
          await api(`/api/recordings/${encodeURIComponent(file.filename)}`, { method: 'DELETE' });
          toast(`已删除数据集：${file.filename}`);
          await loadDatasets();
        } catch (err) {
          toast(`删除失败：${err.message}`, true);
        }
      };

      group.append(downloadLink, deleteBtn);
      actionCell.append(group);

      row.append(selectCell, nameCell, profileCell, sizeCell, durationCell, msgCountCell, topicsCell, timeCell, statusCell, actionCell);
      body.append(row);
    }
  } catch (error) {
    if (summary) summary.textContent = `读取数据集列表失败: ${error.message}`;
    toast(`加载数据集失败：${error.message}`, true);
  }
}

function showDatasetDetails(file) {
  const dialog = $('#datasetDetailDialog');
  if (!dialog) return;
  $('#datasetDetailTitle').textContent = file.filename;
  const profileLabel = file.profile_display_name || file.profile_id || '未知（旧数据）';
  $('#datasetDetailMeta').textContent = `机器人: ${profileLabel} · Profile: ${file.profile_id || '--'} · 生成时间: ${file.created_at || file.modified_at} · 文件格式: ${file.format || 'MCAP'}`;

  const summary = $('#datasetDetailSummary');
  summary.textContent = '';
  const values = [
    ['机器人', profileLabel],
    ['文件大小', file.size_human || '--'],
    ['录制时长', file.duration_human || '--'],
    ['总消息数', file.message_count ? `${file.message_count.toLocaleString()} 条` : '--'],
    ['总平均吞吐', file.avg_rate_hz ? `${file.avg_rate_hz} msg/s` : '--'],
  ];
  for (const [label, val] of values) {
    const span = document.createElement('span');
    const small = document.createElement('small'); small.textContent = label;
    const b = document.createElement('b'); b.textContent = val;
    span.append(small, b);
    summary.append(span);
  }

  const tbody = $('#datasetDetailChannelRows');
  tbody.textContent = '';
  const channels = file.channels || [];
  if (!channels.length) {
    const tr = document.createElement('tr');
    const td = document.createElement('td');
    td.colSpan = 4;
    td.style.textAlign = 'center';
    td.style.color = 'var(--muted)';
    td.textContent = '暂无话题分布详情（文件可能为空或正在录制中）';
    tr.append(td);
    tbody.append(tr);
  } else {
    for (const ch of channels) {
      const tr = document.createElement('tr');
      const tdTopic = document.createElement('td');
      const code = document.createElement('code');
      code.textContent = ch.topic;
      tdTopic.append(code);
      const tdType = document.createElement('td');
      tdType.textContent = ch.type;
      const tdCount = document.createElement('td');
      tdCount.style.font = '12px monospace';
      tdCount.textContent = `${(ch.count || 0).toLocaleString()} 条`;
      const tdRate = document.createElement('td');
      tdRate.className = 'topic-rate ok';
      tdRate.textContent = `${Number(ch.rate_hz || 0).toFixed(1)} Hz`;
      tr.append(tdTopic, tdType, tdCount, tdRate);
      tbody.append(tr);
    }
  }

  dialog.showModal();
}

function openStartReplayDialog(file) {
  const dialog = $('#startReplayDialog');
  if (!dialog) return;
  $('#startReplayFilename').value = file.filename;
  const profileLabel = file.profile_display_name || file.profile_id || '未知（旧数据）';
  $('#startReplayMeta').textContent = `数据集：${file.filename} · 机器人：${profileLabel} · 时长 ${file.duration_human || '--'} · ${(file.message_count || 0).toLocaleString()} 条消息`;
  dialog.showModal();
}

function configureProfileDialog() {
  const dialog=$('#importProfileDialog');
  const form=$('#importProfileForm');
  const errorBox=$('#importProfileError');
  const clearError=()=>{
    if(errorBox){
      errorBox.textContent='';
      errorBox.classList.add('hidden');
    }
  };
  const close=()=>{
    clearError();
    dialog.close();
  };
  $('#openImportProfile').onclick=()=>{
    clearError();
    dialog.showModal();
  };
  $('#closeImportProfile').onclick=close;
  $('#cancelImportProfile').onclick=close;
  form.onsubmit=async event=>{
    event.preventDefault();
    clearError();
    const button=$('#submitImportProfile');
    button.disabled=true;
    button.textContent='正在校验…';
    try {
      const result=await api('/api/robot-profiles/import',{method:'POST',body:new FormData(form)});
      form.reset();
      close();
      await loadProfiles(result.profile.id);
      toast(`已导入 ${result.profile.display_name||result.profile.id}`);
    } catch(error) {
      let msg = error.message;
      if (msg.includes('already exists')) {
        const id = msg.replace('robot profile already exists:', '').trim();
        msg = `导入失败：配置 ID “${id}” 已存在。若需重新导入，请先在主界面点击“删除配置”删除旧配置，或在上方填写不同的“配置 ID”。`;
      } else {
        msg = `导入失败：${msg}`;
      }
      if(errorBox){
        errorBox.textContent=msg;
        errorBox.classList.remove('hidden');
      }
      toast(msg,true);
    } finally {
      button.disabled=false;
      button.textContent='校验并导入';
    }
  };
}

let simulationBusy=false;
let simulationActionError='';
async function loadSimulation() {
  try {
    const value=await api('/api/simulation');
    const states={stopped:'已停止',starting:'启动中',running:'运行中',stopping:'停止中',failed:'启动失败 / 已退出'};
    if(!simulationBusy)$('#simulationState').textContent=states[value.state]||value.state;
    $('#simulationProfile').textContent=`当前配置：${value.active_profile||'未选择'}${value.profile_id&&value.state!=='stopped'?` · 仿真模型：${value.profile_id}`:''}`;
    $('#simulationError').textContent=simulationActionError||value.error||value.unavailable_reason||'';
    $('#simulationLogs').textContent=value.logs.join('\n')||'暂无日志';
    const active=['starting','running','stopping'].includes(value.state);
    $('#startSimulation').disabled=simulationBusy||active||!value.available;
    $('#restartSimulation').disabled=simulationBusy||!value.available||value.state==='stopping';
    $('#stopSimulation').disabled=simulationBusy||!active;
  } catch(error) {
    $('#simulationError').textContent=`无法读取仿真状态：${error.message}`;
    for(const id of ['startSimulation','restartSimulation','stopSimulation']) $('#'+id).disabled=true;
  }
}
async function simulationAction(action) {
  if(simulationBusy)return;
  simulationBusy=true;
  simulationActionError='';
  $('#simulationState').textContent=action==='stop'?'正在停止…':'正在提交启动请求…';
  $('#simulationError').textContent='';
  const controller=new AbortController();
  const timeout=setTimeout(()=>controller.abort(),30000);
  for(const id of ['startSimulation','restartSimulation','stopSimulation']) $('#'+id).disabled=true;
  try {
    if(action!=='stop') {
      const options={};
      $$('[data-path^="simulation."]').forEach(input=>{options[input.dataset.path.split('.')[1]]=input.checked;});
      await api('/api/simulation/options',{method:'PUT',body:JSON.stringify(options),signal:controller.signal});
      if(config)config.simulation=options;
    }
    await api(`/api/simulation/${action}`,{method:'POST',signal:controller.signal});
    toast(action==='stop'?'仿真已停止，中间件继续运行':'正在启动当前机器人仿真，请查看运行状态');
  } catch(error) {
    simulationActionError=error.name==='AbortError'?'请求超过 30 秒，正在查询实际运行状态；请查看下方日志。':error.message;
    $('#simulationError').textContent=simulationActionError;
    toast(simulationActionError,true);
  }
  finally {clearTimeout(timeout);simulationBusy=false;await loadSimulation();}
}

function bindSimulationControls() {
  for(const [id,action] of [['startSimulation','start'],['restartSimulation','restart'],['stopSimulation','stop']]) {
    $('#'+id).onclick=()=>simulationAction(action);
  }
}

async function init() {
  route();
  bindSimulationControls();
  await loadSimulation();
  setInterval(()=>{if(document.visibilityState==='visible')loadSimulation();},2000);
  window.addEventListener('hashchange',route);
  try {
    [config,profilesData]=await Promise.all([api('/api/config'),api('/api/robot-profiles')]);
    renderConfig();
    renderProfiles();
    renderRecordTypeOptions();
  } catch(error){toast(error.message,true);}
  connectWebSocket();
  const update=async()=>{try{renderStatus(await api('/api/status'));}catch(_){} };
  update();
  setInterval(update,1000);
  $('#refreshTopics').onclick=loadTopics;
  $('#recordTopic').onchange=()=>{
    const match=discoveredTopics.find(item=>item.topic===$('#recordTopic').value.trim());
    if(match?.types?.length)renderRecordTypeOptions(match.types[0]);
  };
  $('#addRecording').onclick=()=>{
    const topic=$('#recordTopic').value.trim();
    const type=$('#recordType').value.trim();
    const maxHz=Number($('#recordMaxHz').value)||0;
    if(!topic.startsWith('/'))return toast('消息名称必须以 / 开头',true);
    if(!type)return toast('请选择有效的消息类型',true);
    setTopicRecording(topic,type,true,maxHz);
    $('#recordTopic').value=''; $('#recordMaxHz').value='0';
    toast('已添加；点击“保存录制配置”写入文件');
  };
  $('#refreshProfiles').onclick=()=>loadProfiles($('#robotProfileSelect').value);
  $('#robotProfileSelect').onchange=renderProfileDetail;
  $('#deleteProfile').onclick=async()=>{
    const profileId=$('#robotProfileSelect').value;
    if(!profileId)return;
    const profile=profilesData.profiles.find(p=>p.id===profileId);
    const name=profile?.display_name||profileId;
    const isActive=profileId===profilesData.active;
    const msg=isActive
      ?`确认删除当前活动机器人配置 “${name}” (${profileId}) 及其全部模型和配置文件？\n注意：删除后当前活动配置将被清空。`
      :`确认删除机器人配置 “${name}” (${profileId}) 及其全部模型和配置文件？`;
    if(!confirm(msg))return;
    try {
      const result=await api(`/api/robot-profiles/${encodeURIComponent(profileId)}`,{method:'DELETE'});
      if(result.cleared_active){
        profilesData.active='';
        if(config?.robot_profiles)config.robot_profiles.active='';
      }
      toast(`已删除机器人配置：${name}`);
      await loadProfiles();
    } catch(error){toast(`删除失败：${error.message}`,true);}
  };
  $('#activateProfile').onclick=async()=>{
    const profileId=$('#robotProfileSelect').value;
    if(!profileId)return;
    try {
      const result=await api(`/api/robot-profiles/${encodeURIComponent(profileId)}/activate`,{method:'POST'});
      profilesData.active=result.active;
      if(config?.robot_profiles)config.robot_profiles.active=result.active;
      renderProfiles(result.active);
      toast(result.restart_simulation_required?'配置已应用；可在下方启动当前机器人仿真':'配置已经是活动配置');
    } catch(error){toast(`应用失败：${error.message}`,true);}
  };
  $('#saveConfig').onclick=async()=>{
    try {
      const result=await api('/api/config',{method:'PUT',body:JSON.stringify(collectConfig())});
      config=result.config;
      renderConfig();
      toast(result.server_restart_required?'已保存；服务地址或配置目录变更需重启进程':'配置已保存并应用');
    }catch(error){toast(error.message,true);}
  };
  $('#saveRecording').onclick=async()=>{
    try {
      const result=await api('/api/config',{method:'PUT',body:JSON.stringify(collectConfig())});
      config=result.config; renderConfig();
      toast('录制配置已写入 middleware/config.yaml 并应用');
    }catch(error){toast(error.message,true);}
  };
  async function triggerStartRecording(force = false) {
    try {
      const result = await api('/api/recording/start', {
        method: 'POST',
        body: JSON.stringify({ force }),
      });
      toast(`已开始话题录制：${result.path}`);
      renderStatus(await api('/api/status'));
    } catch (error) {
      toast(`启动录制失败：${error.message}`, true);
    }
  }

  if($('#startRecordingBtn')) {
    $('#startRecordingBtn').onclick=async()=>{
      try {
        const precheck = await api('/api/recording/precheck', { method: 'POST' });
        if (!precheck.ready) {
          if (!precheck.topic_count) {
            toast('尚未勾选任何需要录制的话题，请先在下方勾选待录制话题', true);
            return;
          }
          const dialog = $('#precheckWarningDialog');
          if (dialog) {
            $('#precheckWarningSummary').textContent = `检测到 ${precheck.issues.length} 个待录制话题未检测到消息或频率未达标：`;
            const tbody = $('#precheckIssueRows');
            tbody.textContent = '';
            for (const iss of precheck.issues) {
              const tr = document.createElement('tr');
              const tdTopic = document.createElement('td');
              const code = document.createElement('code');
              code.textContent = iss.topic;
              tdTopic.append(code);
              const tdHz = document.createElement('td');
              tdHz.className = 'topic-rate ' + (iss.hz > 0 ? 'low' : 'none');
              tdHz.textContent = `${Number(iss.hz || 0).toFixed(1)} Hz`;
              const tdStd = document.createElement('td');
              tdStd.className = 'topic-target-rate';
              tdStd.textContent = `≥ ${Number(iss.min_hz || 0).toFixed(0)} Hz`;
              const tdDiag = document.createElement('td');
              const badge = document.createElement('span');
              badge.className = 'topic-status-badge ' + (iss.hz > 0 ? 'status-low' : 'status-none');
              badge.textContent = iss.state === 'no_data' ? '未检测到消息' : '频率偏低';
              tdDiag.append(badge);
              tr.append(tdTopic, tdHz, tdStd, tdDiag);
              tbody.append(tr);
            }
            $('#confirmForceStartBtn').onclick = async () => {
              dialog.close();
              await triggerStartRecording(true);
            };
            dialog.showModal();
            return;
          }
        }
        await triggerStartRecording(false);
      } catch(error) {
        toast(`录制预检失败：${error.message}`, true);
      }
    };
  }
  if($('#stopRecordingBtn')) {
    $('#stopRecordingBtn').onclick=async()=>{
      try {
        const result=await api('/api/recording/stop',{method:'POST'});
        toast(`话题录制已停止，本次共录制 ${result.status?.messages||0} 条消息`);
        renderStatus(await api('/api/status'));
      } catch(error){toast(`停止录制失败：${error.message}`,true);}
    };
  }
  if($('#refreshDatasets')) $('#refreshDatasets').onclick = loadDatasets;
  if($('#selectAllDatasets')) {
    $('#selectAllDatasets').onchange = () => {
      const checked = $('#selectAllDatasets').checked;
      $$('#datasetRows input[type=checkbox]:not(:disabled)').forEach(cb => {
        cb.checked = checked;
      });
      updateSelectedDatasetCount();
    };
  }
  if($('#batchDeleteDatasets')) {
    $('#batchDeleteDatasets').onclick = async () => {
      const selected = $$('#datasetRows input[type=checkbox]:checked')
        .map(cb => cb.dataset.filename)
        .filter(Boolean);
      if (!selected.length) return;
      if (!confirm(`确认批量删除选中的 ${selected.length} 个数据集文件？此操作不可恢复。`)) return;
      try {
        const res = await api('/api/recordings/batch-delete', {
          method: 'POST',
          body: JSON.stringify({ filenames: selected }),
        });
        toast(`已批量删除 ${res.deleted?.length || 0} 个数据集文件`);
        await loadDatasets();
      } catch (err) {
        toast(`批量删除失败：${err.message}`, true);
      }
    };
  }
  if($('#importMcapBtn') && $('#importMcapInput')) {
    $('#importMcapBtn').onclick = () => $('#importMcapInput').click();
    $('#importMcapInput').onchange = async () => {
      const file = $('#importMcapInput').files?.[0];
      if (!file) return;
      const formData = new FormData();
      formData.append('file', file);
      try {
        const res = await fetch('/api/recordings/upload', {
          method: 'POST',
          body: formData,
        });
        const data = await res.json();
        if (!res.ok) throw new Error(data.error || '上传失败');
        toast(`成功导入 MCAP 数据集：${data.filename}`);
        await loadDatasets();
      } catch (err) {
        toast(`导入失败：${err.message}`, true);
      } finally {
        $('#importMcapInput').value = '';
      }
    };
  }

  if($('#startReplayForm')) {
    $('#startReplayForm').onsubmit = async (e) => {
      e.preventDefault();
      const filename = $('#startReplayFilename').value;
      const mode = $('#startReplayMode').value;
      const speed = parseFloat($('#startReplaySpeed').value) || 1.0;
      const loop = $('#startReplayLoop').checked;

      try {
        await api('/api/replay/start', {
          method: 'POST',
          body: JSON.stringify({
            filename,
            speed,
            loop,
            mode: mode === 'direct' ? 'drive' : 'raw',
          }),
        });
        $('#startReplayDialog').close();
        toast(`已开始重放 ${filename} (${speed}x) · 遥操作已自动暂停以防冲突`);
        renderStatus(await api('/api/status'));
      } catch (err) {
        toast(`启动重放失败：${err.message}`, true);
      }
    };
  }

  if($('#pauseResumeReplayBtn')) {
    $('#pauseResumeReplayBtn').onclick = async () => {
      const isPaused = $('#pauseResumeReplayBtn').textContent.includes('继续');
      try {
        await api(isPaused ? '/api/replay/resume' : '/api/replay/pause', { method: 'POST' });
        renderStatus(await api('/api/status'));
      } catch (err) {
        toast(`操作失败：${err.message}`, true);
      }
    };
  }

  if($('#stopReplayBtn')) {
    $('#stopReplayBtn').onclick = async () => {
      try {
        await api('/api/replay/stop', { method: 'POST' });
        toast('重放已停止');
        renderStatus(await api('/api/status'));
      } catch (err) {
        toast(`停止失败：${err.message}`, true);
      }
    };
  }

  if($('#replaySpeedSelect')) {
    $('#replaySpeedSelect').onchange = async () => {
      const speed = parseFloat($('#replaySpeedSelect').value) || 1.0;
      const filename = $('#replayFileName').textContent;
      if (!filename) return;
      try {
        await api('/api/replay/start', {
          method: 'POST',
          body: JSON.stringify({
            filename,
            speed,
            loop: $('#replayLoopToggle').checked,
          }),
        });
        renderStatus(await api('/api/status'));
      } catch (err) {
        toast(`更改倍速失败：${err.message}`, true);
      }
    };
  }

  if($('#replayLoopToggle')) {
    $('#replayLoopToggle').onchange = async () => {
      const loop = $('#replayLoopToggle').checked;
      const speed = parseFloat($('#replaySpeedSelect').value) || 1.0;
      const filename = $('#replayFileName').textContent;
      if (!filename) return;
      try {
        await api('/api/replay/start', {
          method: 'POST',
          body: JSON.stringify({
            filename,
            speed,
            loop,
          }),
        });
        renderStatus(await api('/api/status'));
      } catch (err) {
        toast(`更改循环模式失败：${err.message}`, true);
      }
    };
  }

  if($('#stopButton')) {
    $('#stopButton').onclick=async()=>{
      if(!confirm('确认向机器人发送急停信号？'))return;
      try {
        await api('/api/safety/stop',{method:'POST',body:JSON.stringify({reason:'dashboard emergency stop'})});
        toast('急停信号已发送');
      }catch(error){toast(error.message,true);}
    };
  }
  if($('#resumeButton')) {
    $('#resumeButton').onclick=async()=>{
      try {
        await api('/api/safety/resume',{method:'POST',body:JSON.stringify({reason:'dashboard safety resume'})});
        toast('急停已解除，当前控制源的关节命令输出已恢复');
      }catch(error){toast(error.message,true);}
    };
  }
  if($('#homeButton')) {
    $('#homeButton').onclick=async()=>{
      try {
        await api('/api/teleop/home',{method:'POST'});
        toast('双臂与腰部开始平滑回零至标准初始姿态...');
      }catch(error){toast(error.message,true);}
    };
  }
  const setControlSource=async source=>{
    try {
      const label=source==='exoskeleton'?'外骨骼':'VR / IK';
      await api('/api/teleop/source',{method:'POST',body:JSON.stringify({source})});
      renderStatus(await api('/api/status'));
      toast(`控制源已切换为 ${label}`);
    }catch(error){toast(error.message,true);}
  };
  if($('#sourceVr'))$('#sourceVr').onclick=()=>setControlSource('vr');
  if($('#sourceExoskeleton'))$('#sourceExoskeleton').onclick=()=>setControlSource('exoskeleton');
  configureProfileDialog();
}

init();
