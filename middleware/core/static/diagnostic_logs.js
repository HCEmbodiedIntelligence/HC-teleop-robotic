(function (global) {
  'use strict';
  const MAX_ENTRIES = 500;
  const LEVELS = {critical: 4, fatal: 4, error: 3, warning: 2, warn: 2, info: 1, debug: 0};
  const LABELS = {critical: '严重', fatal: '严重', error: '错误', warning: '警告', warn: '警告', info: '信息', debug: '调试'};
  const SUBSYSTEMS = {
    inference: '推理客户端',
    middleware: '中间件与通信',
    control: '控制与遥操作',
    hardware: '硬件驱动 (HC_X1)',
    ros: '远端 ROS 节点',
    diagnostics: '日志系统',
  };
  const SOURCE_LABELS = {
    vr_gateway: 'VR 头显网关',
    vr_udp: 'VR 手柄通道',
    ros_bridge: 'ROS 控制桥接',
    control_process: '隔离控制核心',
    middleware: '中间件服务',
    'middleware/middleware': '中间件服务',
    'middleware/teleop_operations': '遥操作记录',
    'control/teleop_controller': 'VR 遥操作节点',
    'control/v23_solver': '上肢逆解 (IK)',
    'control/launcher': '控制层启动器',
    'hardware/upper_body': '双臂驱动 (Marvin)',
    'hardware/joint_smoother': '关节平滑器',
    'hardware/hands': '灵巧手驱动',
    'hardware/chassis': '底盘驱动',
    'hardware/camera': '相机驱动',
    'hardware/control_adapter': '硬件控制适配',
    'hardware/hardware': '硬件启动器',
    'inference/holobrain_hc_tj_async_client': '推理客户端 (HoloBrain)',
    diagnostics: '日志采集器',
  };
  const KIND_LABELS = {
    inference_fault: '推理故障停机',
    inference_clip: '推理动作截断',
    inference_pause: '推理等待暂停',
    inference_timing_rejection: '推理时钟拒收',
    inference_gate_blocked: '推理输出受阻',
    inference_recovered: '推理恢复',
    control_diagnostic: '控制边界拦截',
    control_clip: '速度限幅截断',
    rosout: 'ROS 日志',
    vr_input_fault: 'VR 输入异常',
    safety_stop: '安全急停',
    safety_trigger: '触发急停',
    safety_resume: '恢复输出',
    manual_input_timeout: '输入超时',
    heartbeat_timeout: '心跳超时',
    recording_failed: '录制失败',
    control_rejection: '指令被拒',
    log_gap: '日志丢弃',
    revoked: '控制权收回',
    permission_open: '控制权放行',
    ack: '状态确认',
    dagger_enabled: 'DAgger 开启',
    vr_controller_event: '手柄事件',
    right_gripper_source_changed: '夹爪控制切换',
    grip_takeover_hold_open: '介入保持开启',
    grip_takeover_hold_closed: '介入保持结束',
    grip_return_hold_open: '退回保持开启',
    grip_return_hold_closed: '退回保持结束',
    saved_log: '',
    log: '',
  };
  const ANSI_RE = /\x1b\[[0-9;?]*[ -/]*[@-~]/g;
  const ROS_PREFIX_RE = /^(?:\[[^\]]+\]\s+)?\[(?:DEBUG|INFO|WARN(?:ING)?|ERROR|FATAL|CRITICAL)\]\s+\[\d{9,}\.\d+\]\s+\[([^\]]+)\]:\s*/i;
  const LOCAL_PREFIX_RE = /^\[\d{4}-\d{2}-\d{2}[ T]\d{2}:\d{2}:\d{2}(?:\.\d{1,6})?\]\s*/;

  function levelOf(entry) { return String(entry.level || 'info').toLowerCase(); }
  function sortEntries(entries, priority) {
    return entries.slice().sort((a, b) => (priority ? (LEVELS[levelOf(b)] || 0) - (LEVELS[levelOf(a)] || 0) : 0) || Number(b.id) - Number(a.id));
  }
  function mergeEntries(previous, incoming) {
    const byId = new Map(previous.map(entry => [Number(entry.id), entry]));
    incoming.forEach(entry => {
      if (entry && Number.isSafeInteger(Number(entry.id)) && Number(entry.id) > 0) byId.set(Number(entry.id), entry);
    });
    // Bound retention by chronology, never let old errors crowd out new evidence.
    return Array.from(byId.values()).sort((a, b) => Number(b.id) - Number(a.id)).slice(0, MAX_ENTRIES);
  }
  function visibleEntries(entries, priorityEntries, priority) {
    const byId = new Map(entries.map(entry => [Number(entry.id), entry]));
    if (priority) (priorityEntries || []).forEach(entry => {
      if (entry && Number.isSafeInteger(Number(entry.id)) && Number(entry.id) > 0) byId.set(Number(entry.id), entry);
    });
    // Priority previews are independent of the chronological cursor. An error
    // at the end of the retained window must be visible on its first page.
    return sortEntries(Array.from(byId.values()), priority).slice(0, MAX_ENTRIES);
  }
  function formatTime(value) {
    const timestamp = typeof value === 'number' ? value * (value < 1e12 ? 1000 : 1) : value;
    const date = new Date(timestamp);
    return Number.isNaN(date.getTime()) ? String(value || '时间未知') : date.toLocaleString('zh-CN', {hour12: false});
  }
  function formatShortTime(value) {
    const timestamp = typeof value === 'number' ? value * (value < 1e12 ? 1000 : 1) : value;
    const date = new Date(timestamp);
    return Number.isNaN(date.getTime()) ? String(value || '') : date.toLocaleTimeString('zh-CN', {hour12: false});
  }
  function clip(value, max = 12000) {
    let text;
    if (typeof value === 'string') text = value;
    else { try { text = JSON.stringify(value, null, 2); } catch (_) { text = String(value); } }
    return String(text == null ? '' : text).slice(0, max);
  }
  function subsystemLabel(name) { return SUBSYSTEMS[name] || name || '未知模块'; }
  function sourceLabel(source) {
    const raw = String(source || '');
    if (!raw) return '未知模块';
    if (Object.hasOwn(SOURCE_LABELS, raw)) return SOURCE_LABELS[raw];
    const parts = raw.split('/');
    if (parts.length === 2 && Object.hasOwn(SUBSYSTEMS, parts[0])) return SUBSYSTEMS[parts[0]] + ' · ' + parts[1];
    return raw;
  }
  function kindLabel(kind) {
    const raw = String(kind || '');
    if (!raw) return '';
    return Object.hasOwn(KIND_LABELS, raw) ? KIND_LABELS[raw] : raw;
  }
  function cleanDisplayMessage(raw) {
    const original = clip(raw == null ? '' : raw);
    const strippedAnsi = original.replace(ANSI_RE, '');
    let text = strippedAnsi;
    let rosNode = '';
    const rosMatch = text.match(ROS_PREFIX_RE);
    if (rosMatch) {
      rosNode = rosMatch[1] || '';
      text = text.slice(rosMatch[0].length);
    } else if (LOCAL_PREFIX_RE.test(text)) {
      text = text.replace(LOCAL_PREFIX_RE, '');
    }
    return {text, rosNode, stripped: text !== original};
  }
  function explainEntry(entry) {
    const cleaned = cleanDisplayMessage(entry && entry.message).text.trim();
    const kind = String((entry && entry.kind) || '');
    const lower = cleaned.toLowerCase();
    let match;
    if (lower.includes('head tracking invalid')) {
      return {
        groupKey: 'vr_head_tracking_invalid',
        title: 'VR 头显定位丢失',
        hint: '头显未能提供有效位姿（可能摘下头显、环境过暗或遮挡），已暂停遥操作输出。重新佩戴并恢复定位后按恢复即可。',
        secondary: false,
      };
    }
    if ((match = cleaned.match(/no VR pose data for (\d+(?:\.\d+)?)\s*ms/i))) {
      return {
        groupKey: 'vr_pose_timeout',
        title: 'VR 位姿数据断流（超时 ' + match[1] + ' ms）',
        hint: '超过 ' + match[1] + ' ms 未收到头显位姿包（可能头显息屏、休眠、网络波动或断开），系统已暂停输出。',
        secondary: false,
      };
    }
    if (kind === 'manual_input_timeout' || lower.includes('teleop input timeout')) {
      const gap = cleaned.match(/(\d+(?:\.\d+)?)\s*ms\s*>\s*(\d+(?:\.\d+)?)\s*ms/i);
      return {
        groupKey: 'manual_input_timeout',
        title: gap ? ('遥操作输入超时（' + Math.round(Number(gap[1])) + ' ms > ' + Math.round(Number(gap[2])) + ' ms）') : '遥操作输入超时',
        hint: '遥操作控制指令中断超过保护阈值，系统已自动保护停机。',
        secondary: false,
      };
    }
    if (lower.includes('grip return timed out waiting for the first model arm command')) {
      return {
        groupKey: 'dagger_return_model_timeout',
        title: 'DAgger 退回自动模式超时（未收到模型首帧机械臂指令）',
        hint: '松开 Grip 切回自动推理后，超时仍未收到推理端下发新的机械臂指令（请检查推理端是否故障停机或未开启）。',
        secondary: false,
      };
    }
    if (kind === 'heartbeat_timeout' || lower.includes('heartbeat timeout') || lower.includes('heartbeat lost')) {
      return {
        groupKey: 'heartbeat_timeout',
        title: '控制链路心跳超时',
        hint: '控制节点或求解器心跳中断，系统已收回控制权并保护停机。',
        secondary: false,
      };
    }
    if (lower.includes('teleop disabled by emergency stop')) {
      return {
        groupKey: 'teleop_disabled_estop',
        title: '遥操作已响应急停关闭',
        hint: '本条为收到急停信号后的联动关闭日志，并非遥操作节点自身崩溃；请看对应时间点的上游急停原因。',
        secondary: true,
      };
    }
    if (lower.includes('control process exited')) {
      return {
        groupKey: 'control_process_exited',
        title: '控制进程已退出',
        hint: '隔离控制核心或控制子进程退出，请检查控制日志并重新启动。',
        secondary: false,
      };
    }
    if (lower.includes('control configuration is not ready')) {
      return {
        groupKey: 'control_config_not_ready',
        title: '控制配置未就绪，已拒绝恢复输出',
        hint: '控制进程尚未进入运行状态，请等待就绪后再点击恢复。',
        secondary: false,
      };
    }
    if (cleaned.includes('Marvin SDK 上一帧仍待发送')) {
      return {
        groupKey: 'marvin_sdk_frame_pending',
        title: '双臂底层上一帧仍在发送，跳过本次重复帧',
        hint: '偶发告警通常不影响运行；若连续高频出现，请检查机械臂通信延迟或控制下发频率。',
        secondary: false,
      };
    }
    if (lower.includes('joint command input stop timeout')) {
      return {
        groupKey: 'smoother_input_stop_timeout',
        title: '平滑器目标流中断，已暂停转发（保持当前姿态）',
        hint: '常见于停机、退出遥操作或上游切源时；若在正常运动中出现，说明上游控制指令断流。',
        secondary: false,
      };
    }
    if (lower.includes('short joint command gap')) {
      return {
        groupKey: 'smoother_short_command_gap',
        title: '关节目标帧间隔短暂偏大',
        hint: '上游指令短暂抖动，平滑器已自动过渡。',
        secondary: false,
      };
    }
    if (lower.includes('smoother output and targets cleared immediately')) {
      return {
        groupKey: 'smoother_estop_cleared',
        title: '平滑器已响应急停，立即清空输出目标',
        hint: '收到急停信号后的保护动作，并非平滑器自身故障；具体停机原因请看上游触发急停的来源。',
        secondary: true,
      };
    }
    if (lower.includes('retarget integrator reset to fresh joint feedback')) {
      return {
        groupKey: 'ik_integrator_reset',
        title: '逆解积分器已按当前关节反馈重置',
        hint: '恢复控制或切换控制源后的正常保护动作，用于防止机械臂跳变。',
        secondary: false,
      };
    }
    if (lower.includes('solver reset waiting for fresh feedback')) {
      return {
        groupKey: 'ik_reset_waiting_feedback',
        title: '逆解重置中，等待最新关节反馈',
        hint: '重置期间短暂阻断输出，收到最新关节状态后自动恢复。',
        secondary: false,
      };
    }
    if ((match = cleaned.match(/first action jumps\s+([\d.]+)\s*rad\s+from feedback;\s*limit is\s+([\d.]+)\s*rad(?:;\s*joint=([^\s;]+))?/i))) {
      const jointText = match[3] ? (match[3] + ' ') : '';
      return {
        groupKey: 'inference_first_action_jump',
        title: '推理首帧动作跳变超限，已被拦截（' + jointText + '跳变 ' + Number(match[1]).toFixed(3) + ' rad > 上限 ' + Number(match[2]).toFixed(3) + ' rad）',
        hint: '推理输出首帧目标与当前反馈关节角差距超过安全阈值，推理客户端已锁死停发（fault）。请检查跳变阈值参数、模型输出或机器人当前姿态后重启推理。',
        secondary: false,
      };
    }
    if ((match = cleaned.match(/target too far from measured position:\s*([^\s]+).*?error=([\d.]+)\s+limit=([\d.]+)/i))) {
      return {
        groupKey: 'control_target_error_limit',
        title: '目标关节角偏离当前位置过大，指令被拦截（' + match[1] + ' 偏差 ' + Number(match[2]).toFixed(3) + ' rad > 上限 ' + Number(match[3]).toFixed(3) + ' rad）',
        hint: '中间件安全边界（max_target_error_rad）拒绝了该动作指令。请检查模型输出是否突变或确认目标误差阈值配置。',
        secondary: false,
      };
    }
    if ((match = cleaned.match(/joint limit:\s*([^\s]+)\s+target=([-\d.]+)\s+range=\[([-\d.]+),([-\d.]+)\]/i))) {
      return {
        groupKey: 'control_joint_limit',
        title: '目标关节角超出限位，指令被拦截（' + match[1] + ' 目标 ' + Number(match[2]).toFixed(3) + ' 超出 [' + Number(match[3]).toFixed(2) + ', ' + Number(match[4]).toFixed(2) + ']）',
        hint: '目标值超出 URDF/关节配置允许范围，中间件已拦截该帧。',
        secondary: false,
      };
    }
    if (lower.includes('collision check refused')) {
      return {
        groupKey: 'control_collision_refused',
        title: '自碰撞保护拦截了动作指令',
        hint: '目标姿态触发机械臂自碰撞校验，中间件已拒绝下发。',
        secondary: false,
      };
    }
    if (kind === 'control_clip' || lower.includes('arm velocity clipped')) {
      const clipMatch = cleaned.match(/arm velocity clipped(?:\s*\(\d+x\))?:\s*([^\s]+).*?limit=([\d.]+)\s*rad\/s/i);
      return {
        groupKey: 'control_arm_velocity_clipped',
        title: clipMatch
          ? ('中间件双臂速度限幅已截断动作（' + clipMatch[1] + '，上限 ' + clipMatch[2] + ' rad/s）')
          : '中间件双臂速度限幅已截断动作',
        hint: '输入目标变化速度超过 max_arm_velocity_rad_s，中间件已按上限平滑截断，运动会变慢或滞后。',
        secondary: false,
      };
    }
    if (kind === 'inference_clip' || lower.includes('inference action clipped')) {
      const clipVal = cleaned.match(/max_clip_rad=([\d.]+)/i);
      return {
        groupKey: 'inference_action_clipped',
        title: clipVal ? ('推理端动作幅值已被截断（最大截断 ' + clipVal[1] + ' rad）') : '推理端动作幅值已被截断',
        hint: '推理客户端内部限幅触发了动作截断，实际下发目标小于模型原始预测。',
        secondary: false,
      };
    }
    if (kind === 'inference_pause' || lower.includes('inference feedback pause')) {
      return {
        groupKey: 'inference_feedback_pause',
        title: '推理端等待关节反馈，暂停下发动作',
        hint: '推理客户端因反馈超时或未稳定而暂停输出，请检查关节反馈频率与时钟同步。',
        secondary: false,
      };
    }
    if (kind === 'inference_timing_rejection' || lower.includes('inference ownership timing rejected')) {
      return {
        groupKey: 'inference_timing_rejection',
        title: '推理端拒收控制权消息（时钟偏差或消息过期）',
        hint: '请检查机器人主机与推理机的时间同步（NTP/Chrony）及网络延迟。',
        secondary: false,
      };
    }
    if (kind === 'inference_gate_blocked' || lower.includes('inference output blocked in auto')) {
      return {
        groupKey: 'inference_gate_blocked',
        title: 'AUTO 状态下推理端输出仍被门控阻断',
        hint: '中间件已进入 AUTO 放行，但推理客户端内部门控未放行，请查看下方具体原因。',
        secondary: false,
      };
    }
    if (kind === 'inference_fault' || lower.includes('inference/protocol failure')) {
      return {
        groupKey: 'inference_protocol_failure',
        title: '推理客户端故障停机（' + clip(cleaned.replace(/^inference\/protocol failure:\s*/i, ''), 90) + '）',
        hint: '推理客户端进入 fault 锁定并停止发送动作，请排查上述原因后重启或恢复推理。',
        secondary: false,
      };
    }
    if (lower.includes('stale command') || lower.includes('reordered command')) {
      return {
        groupKey: 'control_stale_command',
        title: '控制指令时间戳过期或乱序，已被拦截',
        hint: '请检查发送端与机器人主机的时间同步或网络抖动（指令有效期 250 ms）。',
        secondary: false,
      };
    }
    if (lower.includes('required feedback missing/stale')) {
      return {
        groupKey: 'control_feedback_stale',
        title: '关节反馈缺失或过期，控制指令被拦截',
        hint: '超过 250 ms 未收到新鲜关节反馈，中间件拒绝下发新目标。',
        secondary: false,
      };
    }
    if (lower.includes('ik rejected') || lower.includes('action jump')) {
      return {
        title: '动作目标跳变或超出安全限制，已被拦截',
        hint: '目标关节角相对当前姿态偏差过大，请检查模型/逆解输出或机器人初始位姿。',
        secondary: false,
      };
    }
    if (kind === 'recording_failed') {
      return {
        title: '数据录制失败',
        hint: '录制过程中出现异常，请展开下方诊断字段查看详细错误。',
        secondary: false,
      };
    }
    if (lower.includes('slam init localization failed')) {
      return {
        title: '底盘 SLAM 定位初始化失败',
        hint: '底盘建图或定位模块启动异常，请检查激光雷达与底盘驱动。',
        secondary: false,
      };
    }
    if (lower.includes('dashboard emergency stop')) {
      return {
        title: '网页控制台手动触发急停',
        hint: '操作员在网页界面点击了急停按钮。',
        secondary: false,
      };
    }
    if (lower === 'middleware startup'
        || lower === 'loading control configuration; explicit resume required'
        || lower === 'middleware stopping; explicit resume required'
        || lower === 'control configuration reload; explicit resume required'
        || lower.startsWith('robot profile applied:')) {
      return {
        title: '系统启停 / 加载配置安全锁定（正常流程）',
        hint: '系统启动、切配置或停止时自动锁定输出，需手动确认恢复，不属于运行故障。',
        secondary: true,
      };
    }
    return null;
  }
  function cardStateInfo(group, errorCleared = false) {
    if (errorCleared) {
      if (Number(group.warnings) > 0) return {cls: 'warning', label: '已清错 · 有告警'};
      return {cls: 'cleared', label: '已清错'};
    }
    if (group.state === 'error') return {cls: 'error', label: '近 5 分钟有报错'};
    if (Number(group.errors) > 0) return {cls: 'history-error', label: '本会话有报错'};
    if (Number(group.warnings) > 0) return {cls: 'warning', label: '仅有告警'};
    return {cls: 'ok', label: '运行正常'};
  }

  // Per-session focus clearing: hides acknowledged errors from top overview
  // cards & sidebar badge while retaining every record in the lower log list.
  const CLEAR_STORAGE_PREFIX = 'hc_diag_cleared_v1:';
  const docClearedStores = new WeakMap();
  const clearListeners = new WeakMap();

  function emptyClearedState() {
    return {allBeforeId: 0, bySubsystem: {}};
  }
  function normalizeClearedState(raw) {
    if (!raw || typeof raw !== 'object') return emptyClearedState();
    const bySubsystem = {};
    if (raw.bySubsystem && typeof raw.bySubsystem === 'object') {
      Object.entries(raw.bySubsystem).forEach(([k, v]) => {
        const num = Number(v);
        if (Number.isFinite(num) && num > 0) bySubsystem[String(k)] = num;
      });
    }
    return {
      allBeforeId: Math.max(0, Number(raw.allBeforeId) || 0),
      bySubsystem,
    };
  }
  function getDocStore(doc) {
    if (!doc) return new Map();
    let store = docClearedStores.get(doc);
    if (!store) {
      store = new Map();
      docClearedStores.set(doc, store);
    }
    return store;
  }
  function getLocalStorage(doc) {
    try {
      const win = (doc && doc.defaultView) || global;
      return win && win.localStorage ? win.localStorage : null;
    } catch (_) {
      return null;
    }
  }
  function loadClearedState(doc, sessionId) {
    if (!sessionId) return emptyClearedState();
    const store = getDocStore(doc);
    if (store.has(sessionId)) return normalizeClearedState(store.get(sessionId));
    const storage = getLocalStorage(doc);
    if (storage) {
      try {
        const raw = storage.getItem(CLEAR_STORAGE_PREFIX + sessionId);
        if (raw) {
          const parsed = normalizeClearedState(JSON.parse(raw));
          store.set(sessionId, parsed);
          return parsed;
        }
      } catch (_) { /* ignore storage errors */ }
    }
    return emptyClearedState();
  }
  function hasAnyCleared(cleared) {
    return Boolean(cleared && (cleared.allBeforeId > 0 || Object.keys(cleared.bySubsystem || {}).length > 0));
  }
  function saveClearedState(doc, sessionId, state) {
    if (!sessionId) return;
    const clean = normalizeClearedState(state);
    getDocStore(doc).set(sessionId, clean);
    const storage = getLocalStorage(doc);
    if (storage) {
      try {
        if (hasAnyCleared(clean)) storage.setItem(CLEAR_STORAGE_PREFIX + sessionId, JSON.stringify(clean));
        else storage.removeItem(CLEAR_STORAGE_PREFIX + sessionId);
      } catch (_) { /* ignore storage errors */ }
    }
    const listeners = doc ? clearListeners.get(doc) : null;
    if (listeners) listeners.forEach(fn => { try { fn(sessionId, clean); } catch (_) { /* noop */ } });
  }
  function subscribeClearedChange(doc, fn) {
    if (!doc || typeof fn !== 'function') return () => {};
    let listeners = clearListeners.get(doc);
    if (!listeners) {
      listeners = new Set();
      clearListeners.set(doc, listeners);
    }
    listeners.add(fn);
    return () => listeners.delete(fn);
  }
  function clearedCutoffForSubsystem(subsystem, cleared) {
    if (!cleared) return 0;
    const subCutoff = subsystem && cleared.bySubsystem ? (Number(cleared.bySubsystem[subsystem]) || 0) : 0;
    return Math.max(Number(cleared.allBeforeId) || 0, subCutoff);
  }
  function isEntryFocusCleared(entry, cleared) {
    if (!entry || !cleared) return false;
    if ((LEVELS[levelOf(entry)] || 0) < 3) return false;
    const id = Number(entry.id) || 0;
    if (id <= 0) return false;
    const cutoff = clearedCutoffForSubsystem(entry.subsystem || 'middleware', cleared);
    return cutoff > 0 && id <= cutoff;
  }
  function isGroupErrorCleared(group, cleared) {
    if (!group || !cleared || !group.latest_error) return false;
    const latestId = Number(group.latest_error.id) || 0;
    if (latestId <= 0) return false;
    const cutoff = clearedCutoffForSubsystem(group.subsystem, cleared);
    return cutoff > 0 && latestId <= cutoff;
  }

  function collapseEntries(entries, clearedState = null) {
    const groups = [];
    const byKey = new Map();
    entries.forEach(entry => {
      const cleaned = cleanDisplayMessage(entry.message).text;
      const explained = explainEntry(entry);
      const pattern = (explained && explained.groupKey) || cleaned;
      const clearedFlag = isEntryFocusCleared(entry, clearedState) ? '1' : '0';
      const key = [levelOf(entry), String(entry.source || ''), String(entry.kind || ''), pattern, clearedFlag].join('\u0000');
      const existing = byKey.get(key);
      if (existing) {
        existing.count += 1;
        existing.occurrences.push(entry);
        if (Number(entry.id) > Number(existing.entry.id)) existing.entry = entry;
        existing.earliestTime = existing.earliestTime || entry.time;
        if (String(entry.time || '') < String(existing.earliestTime || '')) existing.earliestTime = entry.time;
        if (String(entry.time || '') > String(existing.latestTime || '')) existing.latestTime = entry.time;
      } else {
        const group = {
          entry,
          count: 1,
          occurrences: [entry],
          earliestTime: entry.time,
          latestTime: entry.time,
        };
        byKey.set(key, group);
        groups.push(group);
      }
    });
    return groups;
  }
  function recentErrors(summary, cleared = null) {
    return (Array.isArray(summary) ? summary : []).reduce((total, group) => {
      if (cleared && isGroupErrorCleared(group, cleared)) return total;
      return total + (Number(group.recent_errors) || 0);
    }, 0);
  }
  function summaryTotals(summary) {
    return (Array.isArray(summary) ? summary : []).reduce((acc, group) => {
      acc.errors += Number(group.errors) || 0;
      acc.warnings += Number(group.warnings) || 0;
      acc.total += Number(group.total) || 0;
      return acc;
    }, {errors: 0, warnings: 0, total: 0});
  }
  function incidentExplanation(incident) {
    if (incident.relation === 'explicit') return '已通过事件关联定位到直接触发原因；以下事件链按发生顺序排列。';
    return '底层原因尚未确定。以下是已记录的异常和上下文，不将时间接近当作因果。';
  }

  // A generation changes on every session/filter switch. Even a fetch mock or
  // browser that ignores AbortController cannot commit an old response.
  class LogModel {
    constructor() { this.generation = 0; this.reset(''); }
    reset(session) { this.generation += 1; this.session = session; this.cursor = 0; this.entries = []; this.priorityEntries = []; this.incidents = []; this.summary = []; }
    accept(generation, session, response) {
      if (generation !== this.generation || session !== this.session || (response.session_id && response.session_id !== session)) return false;
      this.entries = mergeEntries(this.entries, Array.isArray(response.entries) ? response.entries : []);
      if (Array.isArray(response.priority_entries)) this.priorityEntries = response.priority_entries.slice(0, 50);
      this.cursor = Math.max(this.cursor, Number(response.cursor) || 0);
      if (Array.isArray(response.incidents)) this.incidents = response.incidents.slice(0, 30);
      if (Array.isArray(response.summary)) this.summary = response.summary.slice(0, 32);
      return true;
    }
  }

  function mount(root, options = {}) {
    if (!root || root.dataset.diagnosticLogsMounted) return null;
    root.dataset.diagnosticLogsMounted = '1';
    root.classList.add('diagnostic-logs');
    const doc = root.ownerDocument;
    const fetcher = options.fetch || global.fetch.bind(global);
    const setTimer = options.setTimeout || global.setTimeout.bind(global);
    const clearTimer = options.clearTimeout || global.clearTimeout.bind(global);
    const model = new LogModel();
    let timer = null, requestController = null, busy = false, refreshQueued = false, disposed = false, sessionsLoaded = false, lastSessionsAt = 0, filterTimer = null;
    function element(tag, className, text) {
      const node = doc.createElement(tag);
      if (className) node.className = className;
      if (text != null) node.textContent = String(text);
      return node;
    }
    function labelled(label, control) {
      const node = element('label', 'diagnostic-log-field');
      node.append(element('span', '', label), control);
      return node;
    }

    const heading = element('div', 'diagnostic-log-heading');
    const title = element('div');
    title.append(
      element('h2', '', '运行日志与故障诊断'),
      element('p', '', '点击上方模块卡片可按模块筛选，点「一键清错」仅取消上方聚焦高亮，所有历史报错保留在下方日志列表中。')
    );
    const headingActions = element('div', 'diagnostic-log-heading-actions');
    const clearAllBtn = element('button', 'diagnostic-log-clear-btn', '一键清错');
    clearAllBtn.type = 'button';
    clearAllBtn.hidden = true;
    const restoreBtn = element('button', 'diagnostic-log-restore-btn', '恢复聚焦');
    restoreBtn.type = 'button';
    restoreBtn.hidden = true;
    const download = element('a', 'diagnostic-log-download', '下载本会话日志');
    download.setAttribute('download', '');
    headingActions.append(clearAllBtn, restoreBtn, download);
    heading.append(title, headingActions);

    const overview = element('div', 'diagnostic-log-overview'); overview.setAttribute('aria-label', '各子系统状态');

    const tabs = element('div', 'diagnostic-log-tabs'); tabs.setAttribute('role', 'tablist'); tabs.setAttribute('aria-label', '日志视图切换');
    const tabDefs = [
      {value: 'error', label: '仅看报错'},
      {value: 'warning', label: '错误与警告'},
      {value: 'all', label: '全部日志'},
    ];
    const tabButtons = new Map();
    tabDefs.forEach(def => {
      const btn = element('button', 'diagnostic-log-tab', def.label);
      btn.type = 'button';
      btn.dataset.level = def.value;
      btn.setAttribute('role', 'tab');
      btn.addEventListener('click', () => {
        if (level.value !== def.value) {
          level.value = def.value;
          resetView();
        }
      });
      tabButtons.set(def.value, btn);
      tabs.append(btn);
    });

    const toolbar = element('div', 'diagnostic-log-toolbar');
    const session = element('select'); session.setAttribute('aria-label', '日志会话');
    const level = element('select'); level.setAttribute('aria-label', '日志等级');
    [['warning', '错误与警告'], ['error', '仅错误及严重'], ['all', '全部等级'], ['info', '信息及以上']].forEach(([value, name]) => {
      const option = element('option', '', name); option.value = value; level.append(option);
    });
    level.value = 'warning';
    const subsystem = element('select'); subsystem.setAttribute('aria-label', '日志来源');
    const allSources = element('option', '', '全部模块'); allSources.value = ''; subsystem.append(allSources);
    const query = element('input'); query.type = 'search'; query.placeholder = '搜索原因、模块或关键词'; query.maxLength = 200;
    const priority = element('input'); priority.type = 'checkbox'; priority.checked = true;
    const automatic = element('input'); automatic.type = 'checkbox'; automatic.checked = true;
    const collapse = element('input'); collapse.type = 'checkbox'; collapse.checked = true;
    const refresh = element('button', '', '刷新'); refresh.type = 'button';
    toolbar.append(
      labelled('会话', session),
      labelled('模块', subsystem),
      labelled('等级', level),
      labelled('搜索', query),
      labelled('错误优先', priority),
      labelled('自动刷新', automatic),
      labelled('合并重复', collapse),
      refresh
    );

    const status = element('div', 'diagnostic-log-status', '正在读取日志会话…'); status.setAttribute('role', 'status'); status.setAttribute('aria-live', 'polite');
    const error = element('div', 'diagnostic-log-error'); error.hidden = true; error.setAttribute('role', 'alert');
    const warning = element('div', 'diagnostic-log-warning'); warning.hidden = true;
    const list = element('ol', 'diagnostic-log-list'); list.setAttribute('aria-label', '已保存的日志条目');
    const empty = element('p', 'diagnostic-log-empty', '当前会话没有匹配的日志。'); empty.hidden = true;
    const more = element('button', 'diagnostic-log-more', '加载后续记录'); more.type = 'button'; more.hidden = true;
    root.replaceChildren(heading, overview, tabs, toolbar, status, error, warning, list, empty, more);

    function visible() {
      return !doc.hidden && (!root.getClientRects || root.getClientRects().length > 0);
    }
    function updateDownload() {
      if (model.session) {
        download.href = '/api/logs/export?' + new URLSearchParams({session: model.session});
        download.removeAttribute('aria-disabled');
      } else { download.removeAttribute('href'); download.setAttribute('aria-disabled', 'true'); }
    }
    function updateTabs() {
      const totals = summaryTotals(model.summary);
      const counts = {
        error: totals.errors,
        warning: totals.errors + totals.warnings,
        all: totals.total,
      };
      tabDefs.forEach(def => {
        const btn = tabButtons.get(def.value);
        const active = level.value === def.value;
        btn.classList.toggle('active', active);
        btn.setAttribute('aria-selected', String(active));
        btn.replaceChildren(element('span', '', def.label));
        if (model.summary.length) {
          btn.append(element('span', 'diagnostic-log-tab-count', String(counts[def.value] || 0)));
        }
      });
    }
    function currentCleared() {
      return loadClearedState(doc, model.session);
    }
    function currentMaxKnownId(targetSubsystem = '') {
      let maxId = targetSubsystem ? 0 : (Number(model.cursor) || 0);
      model.entries.forEach(e => {
        if (targetSubsystem && (e.subsystem || 'middleware') !== targetSubsystem) return;
        const id = Number(e && e.id) || 0;
        if (id > maxId) maxId = id;
      });
      model.priorityEntries.forEach(e => {
        if (targetSubsystem && (e.subsystem || 'middleware') !== targetSubsystem) return;
        const id = Number(e && e.id) || 0;
        if (id > maxId) maxId = id;
      });
      model.summary.forEach(g => {
        if (targetSubsystem && g.subsystem !== targetSubsystem) return;
        const eid = Number(g && g.latest_error && g.latest_error.id) || 0;
        const wid = Number(g && g.latest_warning && g.latest_warning.id) || 0;
        if (eid > maxId) maxId = eid;
        if (!targetSubsystem && wid > maxId) maxId = wid;
      });
      return maxId;
    }
    function clearAllFocusErrors() {
      if (!model.session) return;
      const prev = currentCleared();
      saveClearedState(doc, model.session, {
        allBeforeId: Math.max(prev.allBeforeId, currentMaxKnownId()),
        bySubsystem: prev.bySubsystem,
      });
      render();
    }
    function clearSubsystemFocusErrors(targetSubsystem) {
      if (!model.session || !targetSubsystem) return;
      const prev = currentCleared();
      const cutoff = Math.max(Number(prev.bySubsystem[targetSubsystem]) || 0, currentMaxKnownId(targetSubsystem));
      if (cutoff <= 0) return;
      saveClearedState(doc, model.session, {
        allBeforeId: prev.allBeforeId,
        bySubsystem: {...prev.bySubsystem, [targetSubsystem]: cutoff},
      });
      render();
    }
    function restoreClearedErrors() {
      if (!model.session) return;
      saveClearedState(doc, model.session, emptyClearedState());
      render();
    }
    function makeEntry(entry, scope = 'list', foldInfo = null, clearedState = null) {
      const severity = levelOf(entry);
      const isCleared = scope === 'list' && isEntryFocusCleared(entry, clearedState);
      const row = element('li', 'diagnostic-log-entry diagnostic-log-level-' + (Object.hasOwn(LEVELS, severity) ? severity : 'info') + (isCleared ? ' diagnostic-log-entry-cleared' : ''));
      row.dataset.logId = String(entry.id);
      const meta = element('div', 'diagnostic-log-meta');
      meta.append(element('span', 'diagnostic-log-level', LABELS[severity] || severity));
      const srcName = sourceLabel(entry.source);
      const srcTag = element('span', 'diagnostic-log-source', srcName);
      if (entry.source && srcName !== entry.source) srcTag.title = String(entry.source);
      meta.append(srcTag);
      const kindText = kindLabel(entry.kind);
      if (kindText) meta.append(element('span', 'diagnostic-log-kind', kindText));
      if (isCleared) meta.append(element('span', 'diagnostic-log-cleared-tag', '已清错'));
      if (foldInfo && foldInfo.count > 1) {
        meta.append(element('span', 'diagnostic-log-repeat', '重复 ' + foldInfo.count + ' 次'));
        meta.append(element('time', '', formatShortTime(foldInfo.earliestTime) + ' ~ ' + formatTime(foldInfo.latestTime)));
      } else {
        meta.append(element('time', '', formatTime(entry.time)));
      }
      row.append(meta);

      const cleaned = cleanDisplayMessage(entry.message);
      const explained = explainEntry(entry);
      if (explained) {
        row.append(element('div', 'diagnostic-log-headline', explained.title));
        if (explained.hint) row.append(element('p', 'diagnostic-log-hint', explained.hint));
      }
      row.append(element('pre', 'diagnostic-log-message' + (explained ? ' diagnostic-log-message-raw' : ''), cleaned.text));
      if (entry.file) {
        row.append(element('div', 'diagnostic-log-file', clip(entry.file, 300) + (entry.line ? ':' + entry.line : '')));
      }
      if (entry.details && Object.keys(entry.details).length) {
        const detail = element('details'); detail.append(element('summary', '', '查看诊断字段'), element('pre', 'diagnostic-log-details', clip(entry.details)));
        detail.dataset.diagnosticKey = model.session + ':' + scope + ':' + entry.id + ':fields';
        row.append(detail);
      }
      return row;
    }
    function syncSubsystems() {
      const known = new Set(Array.from(subsystem.options).map(option => option.value));
      model.summary.forEach(group => {
        if (known.has(group.subsystem)) return;
        const option = element('option', '', subsystemLabel(group.subsystem)); option.value = group.subsystem; subsystem.append(option);
      });
    }
    function selectSubsystem(name) {
      subsystem.value = subsystem.value === name ? '' : name;
      resetView();
    }
    function renderOverview(cleared) {
      syncSubsystems();
      updateTabs();
      const hasActiveErrorCards = model.summary.some(group => Number(group.errors) > 0 && !isGroupErrorCleared(group, cleared));
      clearAllBtn.hidden = !hasActiveErrorCards;
      restoreBtn.hidden = !hasAnyCleared(cleared);

      overview.replaceChildren(...model.summary.map(group => {
        const hasErrors = Number(group.errors) > 0;
        const errCleared = hasErrors && isGroupErrorCleared(group, cleared);
        const stateInfo = cardStateInfo(group, errCleared);
        const card = element('div', 'diagnostic-log-card diagnostic-log-card-' + stateInfo.cls);
        card.setAttribute('role', 'button');
        card.setAttribute('tabindex', '0');
        card.setAttribute('aria-pressed', String(subsystem.value === group.subsystem));
        const head = element('div', 'diagnostic-log-card-head');
        const headRight = element('div', 'diagnostic-log-card-head-right');
        headRight.append(element('span', 'diagnostic-log-card-state', stateInfo.label));
        if (hasErrors && !errCleared) {
          const subClear = element('button', 'diagnostic-log-card-clear', '清错');
          subClear.type = 'button';
          subClear.title = '清除该模块上方报错高亮（下方日志保留）';
          subClear.addEventListener('click', ev => {
            ev.stopPropagation();
            clearSubsystemFocusErrors(group.subsystem);
          });
          headRight.append(subClear);
        }
        head.append(element('strong', '', subsystemLabel(group.subsystem)), headRight);
        const counts = element('div', 'diagnostic-log-card-counts');
        counts.append(
          element('span', errCleared ? 'diagnostic-log-count-cleared' : 'diagnostic-log-count-error',
            (errCleared ? '历史报错 ' : '报错 ') + (Number(group.errors) || 0)),
          element('span', 'diagnostic-log-count-warning', '告警 ' + (Number(group.warnings) || 0)),
          element('span', '', '共 ' + (Number(group.total) || 0))
        );
        card.append(head, counts);
        const noisySources = Object.entries(group.sources || {})
          .filter(([, stat]) => (Number(stat.errors) || 0) > 0 || (Number(stat.warnings) || 0) > 0)
          .sort((a, b) => ((Number(b[1].errors) || 0) - (Number(a[1].errors) || 0)) || ((Number(b[1].warnings) || 0) - (Number(a[1].warnings) || 0)))
          .slice(0, 4);
        if (noisySources.length) {
          const chips = element('div', 'diagnostic-log-card-sources');
          noisySources.forEach(([src, stat]) => {
            const parts = [];
            if (Number(stat.errors) > 0) parts.push(stat.errors + '错');
            if (Number(stat.warnings) > 0) parts.push(stat.warnings + '警');
            chips.append(element('span', 'diagnostic-log-card-chip' + (!errCleared && Number(stat.errors) > 0 ? ' has-error' : ''), sourceLabel(src) + ' ' + parts.join('/')));
          });
          card.append(chips);
        }
        const latest = errCleared ? (group.latest_warning || group.latest_error) : (group.latest_error || group.latest_warning);
        if (latest) {
          const explained = explainEntry(latest);
          const preview = explained ? explained.title : cleanDisplayMessage(latest.message).text;
          const srcText = sourceLabel(latest.source);
          const subText = subsystemLabel(group.subsystem);
          const showSrc = srcText && !srcText.startsWith(subText);
          const prefix = (errCleared && latest === group.latest_error) ? '已清错 · ' : '';
          card.append(element('p', 'diagnostic-log-card-latest',
            prefix + formatShortTime(latest.time) + (showSrc ? (' · ' + srcText + '：') : ' · ') + clip(preview, 180)));
        }
        card.addEventListener('click', () => selectSubsystem(group.subsystem));
        card.addEventListener('keydown', ev => {
          if (ev.target !== card) return;
          if (ev.key === 'Enter' || ev.key === ' ') { ev.preventDefault(); selectSubsystem(group.subsystem); }
        });
        return card;
      }));
    }
    function updateStatusText(shownCount, rawCount) {
      const modeText = priority.checked ? '错误优先' : '时间倒序';
      const foldText = collapse.checked && rawCount > shownCount ? ('（已将 ' + rawCount + ' 条重复日志合并）') : ('（最多 ' + MAX_ENTRIES + ' 条）');
      status.textContent = '已显示 ' + shownCount + ' 条' + foldText + ' · ' + modeText + ' · 更新于 ' + new Date().toLocaleTimeString('zh-CN', {hour12: false});
    }
    function render() {
      // Preserve the evidence the user is reading across polls.
      const openDetails = new Set(Array.from(root.querySelectorAll('details[data-diagnostic-key]'))
        .filter(node => node.open).map(node => node.dataset.diagnosticKey));
      const scrolls = new Map(Array.from(root.querySelectorAll('[data-diagnostic-scroll-key]'))
        .map(node => [node.dataset.diagnosticScrollKey, node.scrollTop]));
      const cleared = currentCleared();
      renderOverview(cleared);
      list.dataset.diagnosticScrollKey = model.session + ':list';
      const rawShown = visibleEntries(model.entries, model.priorityEntries, priority.checked);
      const renderedRows = collapse.checked
        ? collapseEntries(rawShown, cleared).map(group => makeEntry(group.entry, 'list', group, cleared))
        : rawShown.map(entry => makeEntry(entry, 'list', null, cleared));
      list.replaceChildren(...renderedRows);
      empty.hidden = renderedRows.length !== 0;
      if (renderedRows.length === 0) {
        if (level.value === 'error') empty.textContent = '当前筛选条件下没有错误日志（可切换到「错误与警告」或「全部日志」查看）。';
        else if (level.value === 'warning') empty.textContent = '当前筛选条件下没有错误或警告（可切换到「全部日志」查看运行过程）。';
        else empty.textContent = '当前会话没有匹配的日志。';
      }
      if (sessionsLoaded) updateStatusText(renderedRows.length, rawShown.length);

      root.querySelectorAll('details[data-diagnostic-key]').forEach(node => {
        node.open = openDetails.has(node.dataset.diagnosticKey);
      });
      root.querySelectorAll('[data-diagnostic-scroll-key]').forEach(node => {
        node.scrollTop = scrolls.get(node.dataset.diagnosticScrollKey) || 0;
      });
    }
    function cancelRequest() { if (requestController) requestController.abort(); }
    function resetView() {
      cancelRequest(); model.reset(session.value); more.hidden = true; updateDownload(); render();
      if (busy) refreshQueued = true;
      else void refreshLogs(true);
    }
    async function getJSON(url, signal) {
      const response = await fetcher(url, {signal, headers: {'Accept': 'application/json'}});
      if (!response.ok) throw new Error('日志服务返回 HTTP ' + response.status);
      return response.json();
    }
    async function refreshLogs(force = false) {
      if (disposed) return;
      if (busy) { if (force) refreshQueued = true; return; }
      if (!force && !visible()) return;
      busy = true; refresh.disabled = true; more.disabled = true;
      const controller = new AbortController(); requestController = controller;
      const deadline = setTimer(() => controller.abort(), 8000);
      let generation = model.generation;
      try {
        if (!sessionsLoaded || Date.now() - lastSessionsAt > 15000) {
          const result = await getJSON('/api/logs/sessions', controller.signal);
          if (generation !== model.generation || disposed) return;
          const saved = session.value;
          session.replaceChildren();
          (Array.isArray(result.sessions) ? result.sessions : []).forEach(item => {
            const option = element('option', '', (item.label || item.id) + (item.current ? ' · 当前' : '')); option.value = item.id; session.append(option);
          });
          if (Array.from(session.options).some(item => item.value === saved)) session.value = saved;
          else if (result.current_session_id) session.value = result.current_session_id;
          if (model.session !== session.value) model.reset(session.value);
          generation = model.generation;
          sessionsLoaded = true; lastSessionsAt = Date.now(); updateDownload();
        }
        if (!session.value) { status.textContent = '尚无保存的日志会话。'; empty.hidden = false; return; }
        const requestGeneration = model.generation, requestedSession = model.session;
        const params = new URLSearchParams({session: requestedSession, limit: '200', level: level.value, q: query.value.trim(), priority: priority.checked ? '1' : '0', subsystem: subsystem.value});
        if (model.cursor) params.set('after', model.cursor);
        const result = await getJSON('/api/logs?' + params, controller.signal);
        if (disposed || !model.accept(requestGeneration, requestedSession, result)) return;
        error.hidden = true;
        const warnings = [];
        if (Number(result.dropped) > 0) warnings.push('日志队列曾丢弃 ' + result.dropped + ' 条记录，证据可能不完整。');
        if (result.persistence_error) warnings.push('日志持久化异常：' + clip(result.persistence_error, 1000));
        warning.textContent = warnings.join(' '); warning.hidden = !warnings.length;
        more.hidden = !result.has_more;
        render();
      } catch (failure) {
        if (!disposed && !controller.signal.aborted && generation === model.generation) {
          error.textContent = '读取日志失败：' + failure.message + '。已显示的记录仍保留。'; error.hidden = false;
        } else if (!disposed && controller.signal.aborted && generation === model.generation && visible()) {
          error.textContent = '日志请求已超时或取消，已显示的记录仍保留。'; error.hidden = false;
        }
      } finally {
        clearTimer(deadline); busy = false; refresh.disabled = false; more.disabled = false;
        if (requestController === controller) requestController = null;
        if (timer) clearTimer(timer);
        const queued = refreshQueued; refreshQueued = false;
        if (!disposed && queued) void refreshLogs(true);
        else if (!disposed && automatic.checked) timer = setTimer(() => { timer = null; void refreshLogs(); }, 2000);
      }
    }
    clearAllBtn.addEventListener('click', clearAllFocusErrors);
    restoreBtn.addEventListener('click', restoreClearedErrors);
    session.addEventListener('change', resetView);
    level.addEventListener('change', resetView);
    subsystem.addEventListener('change', resetView);
    priority.addEventListener('change', () => { render(); });
    collapse.addEventListener('change', () => { render(); });
    query.addEventListener('input', () => { if (filterTimer) clearTimer(filterTimer); filterTimer = setTimer(resetView, 300); });
    automatic.addEventListener('change', () => { if (timer) clearTimer(timer); if (automatic.checked) void refreshLogs(); });
    refresh.addEventListener('click', () => { sessionsLoaded = false; void refreshLogs(true); });
    more.addEventListener('click', () => void refreshLogs(true));
    function onVisibility() { if (doc.hidden) { cancelRequest(); if (timer) clearTimer(timer); } else if (automatic.checked) void refreshLogs(); }
    doc.addEventListener('visibilitychange', onVisibility);
    void refreshLogs();
    return {
      refresh: () => refreshLogs(true),
      clearFocus: clearAllFocusErrors,
      restoreFocus: restoreClearedErrors,
      model,
      dispose() { disposed = true; refreshQueued = false; cancelRequest(); if (timer) clearTimer(timer); if (filterTimer) clearTimer(filterTimer); doc.removeEventListener('visibilitychange', onVisibility); }
    };
  }
  // Sidebar badge: polls only the error summary so errors are visible from
  // every page, including when the log page itself is not open.
  function watchBadge(link, options = {}) {
    if (!link) return null;
    const fetcher = options.fetch || global.fetch.bind(global);
    const doc = link.ownerDocument;
    const badge = doc.createElement('span');
    badge.className = 'diagnostic-log-badge'; badge.hidden = true;
    link.append(badge);
    let timer = null;
    let lastSessionId = '';
    let lastSummary = [];
    function applyBadge() {
      const cleared = lastSessionId ? loadClearedState(doc, lastSessionId) : null;
      const count = recentErrors(lastSummary, cleared);
      badge.textContent = count > 99 ? '99+' : String(count);
      badge.hidden = count === 0;
      link.title = count ? '最近 5 分钟有 ' + count + ' 条错误' : '';
    }
    const unsubscribe = subscribeClearedChange(doc, changedSession => {
      if (!lastSessionId || changedSession === lastSessionId) applyBadge();
    });
    async function poll() {
      try {
        if (!doc.hidden) {
          const response = await fetcher('/api/logs?limit=1&level=error&priority=0', {headers: {'Accept': 'application/json'}});
          if (response.ok) {
            const data = await response.json();
            lastSessionId = data.session_id || '';
            lastSummary = Array.isArray(data.summary) ? data.summary : [];
            applyBadge();
          }
        }
      } catch (_) { /* the connection indicator already reports middleware loss */ }
      timer = global.setTimeout(poll, options.interval || 5000);
    }
    void poll();
    return {badge, dispose() { unsubscribe(); if (timer) global.clearTimeout(timer); }};
  }
  const api = {
    mount, watchBadge, recentErrors, subsystemLabel, sourceLabel, kindLabel,
    cleanDisplayMessage, explainEntry, collapseEntries,
    isEntryFocusCleared, isGroupErrorCleared,
    LogModel, sortEntries, mergeEntries, visibleEntries, incidentExplanation, formatTime, clip,
  };
  global.TeleopDiagnosticLogs = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  if (global.document) {
    const boot = () => {
      // The page starts hidden; the router calls panel.refresh() when it opens.
      api.panel = mount(global.document.getElementById('diagnosticLogsPanel'));
      watchBadge(global.document.querySelector('nav a[data-page="logs"]'));
    };
    if (global.document.readyState === 'loading') global.document.addEventListener('DOMContentLoaded', boot, {once: true}); else boot();
  }
})(typeof globalThis !== 'undefined' ? globalThis : this);
