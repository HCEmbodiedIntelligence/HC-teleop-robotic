const $ = (id) => document.getElementById(id);
const $$ = (selector) => [...document.querySelectorAll(selector)];

const pageTitles = { overview: "运行总览", joints: "关节监控", diagnostics: "链路诊断" };
const streamNames = {
  vr: "VR FRAME",
  joints: "JOINT STATE",
  cartesian_targets: "CARTESIAN TARGET",
  backend_candidates: "IK CANDIDATE",
  commands: "FINAL COMMAND",
};
const buttonBits = [[1, "主键"], [2, "副键"], [4, "Grip"], [8, "Trigger"], [16, "Menu"], [32, "摇杆"]];
let socket = null;
let reconnectTimer = null;
let toastTimer = null;
let latestSnapshot = null;

function text(value) {
  return value === null || value === undefined || value === "" ? "—" : String(value);
}

function setText(id, value) {
  const element = $(id);
  if (element) element.textContent = text(value);
}

function number(value, digits = 3) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed.toFixed(digits) : "—";
}

function showToast(message, error = false) {
  const element = $("toast");
  element.textContent = message;
  element.className = error ? "show error" : "show";
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { element.className = ""; }, 3200);
}

function route() {
  const requested = location.hash.slice(1) || "overview";
  const page = pageTitles[requested] ? requested : "overview";
  $$(".page").forEach((element) => element.classList.toggle("hidden", element.id !== page));
  $$("nav a").forEach((element) => element.classList.toggle("active", element.dataset.page === page));
  setText("page-title", pageTitles[page]);
  if (latestSnapshot && page === "joints") renderJoints(latestSnapshot);
  requestAnimationFrame(() => window.scrollTo(0, 0));
}

function metricState(metric, optional = false) {
  if (!metric || !metric.total) return optional ? "idle" : "stale";
  return metric.stale ? "stale" : "live";
}

function renderMetrics(streams) {
  const grid = $("metric-grid");
  grid.replaceChildren();
  Object.entries(streamNames).forEach(([key, label]) => {
    const metric = streams[key] || { hz: 0, age_ms: null, stale: true, total: 0 };
    const optional = ["cartesian_targets", "backend_candidates", "commands"].includes(key);
    const state = metricState(metric, optional);
    const card = document.createElement("article");
    card.className = `metric ${state === "stale" ? "stale" : ""} ${state === "idle" ? "idle" : ""}`;
    const top = document.createElement("div");
    top.className = "metric-top";
    const name = document.createElement("span");
    name.textContent = label;
    const dot = document.createElement("i");
    dot.className = "dot";
    top.append(name, dot);
    const rate = document.createElement("strong");
    rate.textContent = `${number(metric.hz || 0, 1)} Hz`;
    const age = document.createElement("small");
    age.textContent = metric.age_ms === null ? "尚未收到消息" : `${number(metric.age_ms, 0)} ms ago · ${metric.total} frames`;
    card.append(top, rate, age);
    grid.append(card);
  });
}

function setPipelineStage(name, state, label) {
  const stage = document.querySelector(`.pipeline-stage[data-stage="${name}"]`);
  if (!stage) return;
  stage.className = `pipeline-stage ${state}`;
  stage.querySelector(":scope > b").textContent = label;
}

function renderPipeline(snapshot) {
  const streams = snapshot.streams || {};
  const vrLive = metricState(streams.vr) === "live";
  const jointsLive = metricState(streams.joints) === "live";
  const mapperLive = metricState(streams.cartesian_targets, true) === "live";
  const backendLive = metricState(streams.backend_candidates, true) === "live";
  const commandLive = metricState(streams.commands, true) === "live";
  const safety = snapshot.safety || {};

  setPipelineStage("vr", vrLive ? "live" : "error", vrLive ? "在线" : "断流");
  setPipelineStage("mapper", mapperLive ? "live" : (vrLive ? "wait" : "error"), mapperLive ? "目标有效" : (vrLive ? "待离合" : "无输入"));
  setPipelineStage("ik", backendLive ? "live" : (mapperLive ? "error" : "wait"), backendLive ? "解算正常" : (mapperLive ? "无解" : "待目标"));
  const safetyError = safety.fault_latched || safety.estop_active;
  setPipelineStage("arbiter", safetyError ? "error" : (commandLive ? "live" : "wait"), safetyError ? "已锁存" : (commandLive ? "已授权" : (safety.enabled ? "待租约" : "未启用")));
  setPipelineStage("robot", jointsLive ? "live" : "error", jointsLive ? "反馈正常" : "无反馈");

  const allRequired = vrLive && jointsLive && !safetyError;
  setText("pipeline-badge", allRequired ? (commandLive ? "控制链路活动" : "已就绪，等待操作") : "链路未就绪");
  setText("pipeline-source", snapshot.vr?.source_id || safety.active_source);
  setText("pipeline-session", safety.active_session);
  setText("pipeline-groups", Object.keys(snapshot.commands || {}).length);
}

function tracked(value) {
  if (typeof value === "boolean") return { tracked: value, confidence: value ? 1 : 0, pose: {} };
  return value || { tracked: false, confidence: 0, pose: {} };
}

function setTracking(id, confidenceId, value) {
  const state = tracked(value);
  $(id).classList.toggle("good", Boolean(state.tracked));
  setText(confidenceId, `${Math.round((Number(state.confidence) || 0) * 100)}%`);
}

function setBar(id, value) {
  const safe = Math.max(0, Math.min(1, Number(value) || 0));
  $(id).style.width = `${safe * 100}%`;
  setText(`${id}-value`, safe.toFixed(3));
}

function heldButtons(mask) {
  const names = buttonBits.filter(([bit]) => (Number(mask) & bit) !== 0).map(([, name]) => name);
  return names.length ? names.join(" + ") : "无按键";
}

function axis(value) {
  const values = Array.isArray(value) ? value : [0, 0];
  return `${number(values[0])}, ${number(values[1])}`;
}

function position(value) {
  const pose = tracked(value).pose || {};
  if (![pose.x, pose.y, pose.z].every((item) => Number.isFinite(Number(item)))) return "—";
  return `${number(pose.x)} / ${number(pose.y)} / ${number(pose.z)}`;
}

function renderVr(snapshot) {
  const vr = snapshot.vr || {};
  const input = vr.inputs || {};
  const tracking = vr.tracking || {};
  setText("vr-source", vr.source_id ? `${vr.source_id} · ${snapshot.streams?.vr?.total || 0} 帧` : "头显尚未连接");
  setText("vr-protocol", vr.protocol_version ? `协议 v${vr.protocol_version}` : "协议 —");
  setText("vr-sequence", vr.sequence);
  setText("vr-loss", vr.packet_loss_total);
  setText("vr-rate", `${number(snapshot.streams?.vr?.hz || 0, 1)} Hz`);
  setTracking("track-head", "head-confidence", tracking.head);
  setTracking("track-left", "left-confidence", tracking.left);
  setTracking("track-right", "right-confidence", tracking.right);
  ["left", "right"].forEach((side) => {
    const controller = input[side] || {};
    setBar(`${side}-grip`, controller.grip);
    setBar(`${side}-trigger`, controller.trigger);
    setText(`${side}-buttons`, heldButtons(controller.held_mask));
    setText(`${side}-axis`, axis(controller.primary_axis));
    setText(`${side}-position`, position(tracking[side]));
  });
}

function modeName(value) {
  return ({ 1: "POSITION", 2: "VELOCITY", 3: "EFFORT" })[Number(value)] || "UNKNOWN";
}

function renderCommands(commands) {
  const root = $("command-groups");
  const entries = Object.entries(commands || {}).sort(([left], [right]) => left.localeCompare(right));
  setText("command-count", `${entries.length} 组`);
  root.replaceChildren();
  root.className = entries.length ? "command-groups" : "command-groups empty-state";
  if (!entries.length) {
    root.textContent = "尚无最终命令；启用安全状态并建立控制租约后才会产生输出";
    return;
  }
  entries.forEach(([group, command]) => {
    const card = document.createElement("div");
    card.className = "command-card";
    const header = document.createElement("header");
    const title = document.createElement("strong");
    title.textContent = group;
    const sequence = document.createElement("span");
    sequence.textContent = `SEQ ${command.sequence}`;
    header.append(title, sequence);
    const values = document.createElement("code");
    values.textContent = (command.positions || []).map((value) => number(value)).join("  ") || "no position values";
    const meta = document.createElement("div");
    meta.className = "command-meta";
    meta.innerHTML = `<span>${modeName(command.mode)}</span><span>${text(command.source_id)}</span><span>${(command.positions || []).length} joints</span>`;
    card.append(header, values, meta);
    root.append(card);
  });
}

function commandJointMap(commands) {
  const result = new Map();
  Object.entries(commands || {}).forEach(([group, command]) => {
    const names = command.names || [];
    (command.positions || []).forEach((value, index) => {
      if (names[index]) result.set(String(names[index]), { value: Number(value), group });
    });
  });
  return result;
}

function renderJoints(snapshot) {
  const root = $("joint-rows");
  const feedback = new Map((snapshot.joints?.values || []).map((joint) => [String(joint.name), Number(joint.position)]));
  const commands = commandJointMap(snapshot.commands);
  const filter = ($("joint-filter").value || "").trim().toLowerCase();
  const names = [...new Set([...feedback.keys(), ...commands.keys()])].filter((name) => !filter || name.toLowerCase().includes(filter));
  root.replaceChildren();
  setText("joint-count", `${names.length} 个关节`);
  if (!names.length) {
    const row = document.createElement("tr");
    const cell = document.createElement("td");
    cell.colSpan = 7;
    cell.className = "empty-cell";
    cell.textContent = filter ? "没有匹配的关节" : "等待 JointState 消息…";
    row.append(cell);
    root.append(row);
    return;
  }
  names.forEach((name) => {
    const feedbackValue = feedback.has(name) ? feedback.get(name) : NaN;
    const command = commands.get(name);
    const commandValue = command?.value;
    const errorRad = Number.isFinite(commandValue) && Number.isFinite(feedbackValue) ? commandValue - feedbackValue : NaN;
    const errorDeg = errorRad * 180 / Math.PI;
    const row = document.createElement("tr");
    if (Number.isFinite(errorDeg) && Math.abs(errorDeg) > 5) row.className = "error";
    else if (Number.isFinite(errorDeg) && Math.abs(errorDeg) > 2) row.className = "warn";
    const values = [name, command?.group || "—", number(commandValue, 4), number(commandValue * 180 / Math.PI, 2), number(feedbackValue, 4), number(feedbackValue * 180 / Math.PI, 2), number(errorDeg, 2)];
    values.forEach((value) => {
      const cell = document.createElement("td");
      cell.textContent = value;
      row.append(cell);
    });
    root.append(row);
  });
}

function healthTag(metric, optional = false) {
  if (!metric?.total && optional) return '<span class="health-tag idle"><i></i>待操作</span>';
  if (!metric?.total || metric.stale) return '<span class="health-tag bad"><i></i>已超时</span>';
  return '<span class="health-tag"><i></i>正常</span>';
}

function renderStreams(streams) {
  const root = $("stream-rows");
  root.replaceChildren();
  Object.entries(streamNames).forEach(([key, label]) => {
    const metric = streams[key] || {};
    const optional = ["cartesian_targets", "backend_candidates", "commands"].includes(key);
    const row = document.createElement("tr");
    row.innerHTML = `<td>${label}</td><td>${healthTag(metric, optional)}</td><td>${number(metric.hz || 0, 1)} Hz</td><td>${metric.age_ms === null || metric.age_ms === undefined ? "—" : `${number(metric.age_ms, 1)} ms`}</td><td>${number(metric.max_gap_ms || 0, 1)} ms</td><td>${metric.total || 0}</td>`;
    root.append(row);
  });
}

function renderTargets(cartesian) {
  const root = $("target-groups");
  const groups = cartesian?.groups || [];
  setText("target-sequence", cartesian?.sequence === undefined ? "SEQ —" : `SEQ ${cartesian.sequence}`);
  root.replaceChildren();
  root.className = groups.length ? "target-groups" : "target-groups empty-state";
  if (!groups.length) {
    root.textContent = "尚无笛卡尔目标";
    return;
  }
  groups.forEach((target) => {
    const pose = target.pose || {};
    const card = document.createElement("div");
    card.className = "data-card";
    card.innerHTML = `<header><strong>${text(target.name)}</strong><span>${text(target.reference_frame)} → ${text(target.tip_frame)}</span></header><code>XYZ ${number(pose.x)}  ${number(pose.y)}  ${number(pose.z)} · Q ${number(pose.qx)}  ${number(pose.qy)}  ${number(pose.qz)}  ${number(pose.qw)}</code><div class="data-meta"><span>POSE MODE ${target.pose_mode}</span></div>`;
    root.append(card);
  });
}

function renderBackend(candidates) {
  const root = $("backend-groups");
  const entries = Object.entries(candidates || {}).sort(([left], [right]) => left.localeCompare(right));
  root.replaceChildren();
  root.className = entries.length ? "backend-groups" : "backend-groups empty-state";
  if (!entries.length) {
    root.textContent = "尚无 IK 候选命令";
    return;
  }
  entries.forEach(([group, candidate]) => {
    const card = document.createElement("div");
    card.className = "data-card";
    card.innerHTML = `<header><strong>${group}</strong><span>SEQ ${candidate.sequence}</span></header><code>${(candidate.positions || []).map((value) => number(value)).join("  ")}</code><div class="data-meta"><span>${modeName(candidate.control_mode)}</span><span>${(candidate.positions || []).length} joints</span><span>${text(candidate.source_id)}</span></div>`;
    root.append(card);
  });
}

function renderDiagnostics(diagnostics) {
  const statuses = diagnostics?.statuses || {};
  const overview = statuses.control_chain || {};
  const latency = statuses.latency?.values || {};
  const badge = $("diagnostic-health");
  const available = Boolean(diagnostics?.available);
  const level = Math.max(0, ...Object.values(statuses).map((status) => Number(status.level || 0)));
  badge.textContent = !available ? "等待" : (level > 0 ? "警告" : "正常");
  badge.className = `panel-badge ${!available ? "" : (level > 0 ? "warn" : "live")}`;
  const values = overview.values || {};
  setText("diagnostic-count", available ? values.anomaly_count ?? 0 : "—");
  setText("diagnostic-last", values.last_anomaly_code ? `${values.last_anomaly_code} · ${values.last_anomaly_group || "—"} · SEQ ${values.last_anomaly_sequence ?? "—"}` : "无");
  setText("diagnostic-log", values.log_path || "未启用持久化");
  const stages = [
    ["vr_receive_gap", "UDP 网关接收间隔"],
    ["vr_callback_delay", "网关 → ROS 回调"],
    ["vr_to_target", "VR → Mapper"],
    ["target_to_candidate", "Mapper → IK"],
    ["candidate_to_command", "IK → Arbiter"],
    ["vr_to_command", "VR → Command"],
  ];
  const root = $("latency-rows");
  const vectors = $("diagnostic-vectors");
  root.replaceChildren();
  if (!available) {
    const row = document.createElement("tr");
    row.innerHTML = '<td colspan="6" class="empty-cell">等待诊断节点…</td>';
    root.append(row);
    vectors.className = "diagnostic-vectors empty-state";
    vectors.textContent = "等待 IK 跳变量与命令反馈误差…";
    return;
  }
  stages.forEach(([key, label]) => {
    const row = document.createElement("tr");
    const fields = [label, latency[`${key}.samples`] || 0, `${number(latency[`${key}.last_ms`] || 0, 2)} ms`, `${number(latency[`${key}.mean_ms`] || 0, 2)} ms`, `${number(latency[`${key}.p95_ms`] || 0, 2)} ms`, `${number(latency[`${key}.max_ms`] || 0, 2)} ms`];
    fields.forEach((field) => {
      const cell = document.createElement("td");
      cell.textContent = field;
      row.append(cell);
    });
    root.append(row);
  });

  const metrics = Object.entries(latency)
    .filter(([key]) => key.startsWith("candidate_step_rad.") || key.startsWith("feedback_error_rad."))
    .sort(([left], [right]) => left.localeCompare(right));
  vectors.replaceChildren();
  vectors.className = metrics.length ? "diagnostic-vectors" : "diagnostic-vectors empty-state";
  if (!metrics.length) {
    vectors.textContent = "尚无活动手臂的 IK 跳变量与命令反馈误差";
  } else {
    metrics.forEach(([key, value]) => {
      const step = key.startsWith("candidate_step_rad.");
      const group = key.slice(key.indexOf(".") + 1);
      const card = document.createElement("div");
      const title = document.createElement("strong");
      const kind = document.createElement("span");
      const reading = document.createElement("code");
      title.textContent = group;
      kind.textContent = step ? "IK 单帧跳变" : "命令 → 反馈误差";
      reading.textContent = `${number(value, 4)} rad / ${number(Number(value) * 180 / Math.PI, 2)}°`;
      card.append(title, kind, reading);
      vectors.append(card);
    });
  }
}

function renderSafety(safety) {
  const label = String(safety.label || "UNKNOWN");
  const className = label.toLowerCase();
  const pill = $("safety-state");
  pill.className = `status-pill ${className}`;
  pill.innerHTML = `<i></i>${label}`;
  $("safety-banner").className = `safety-banner panel ${className}`;
  setText("safety-reason", safety.reason || "等待安全状态");
  setText("session-label", safety.active_session ? `${safety.active_source} / ${safety.active_session}` : "尚无控制会话");
  setText("safety-enabled", safety.enabled ? "已启用" : "已关闭");
  setText("fault-state", safety.estop_active ? "急停" : (safety.fault_latched ? "已锁存" : "正常"));
}

function renderRuntime(snapshot) {
  setText("runtime-robot", snapshot.robot_id);
  setText("runtime-profile", snapshot.profile);
  setText("runtime-mode", snapshot.mode);
  setText("runtime-namespace", `/robots/${snapshot.robot_id}`);
}

function render(snapshot) {
  latestSnapshot = snapshot;
  const robot = String(snapshot.robot_id || "robot");
  setText("robot-name", robot);
  setText("robot-avatar", robot.slice(0, 3).toUpperCase());
  setText("profile-label", `${snapshot.profile || "profile"} · ${snapshot.mode || "runtime"}`);
  setText("mode-label", String(snapshot.mode || "runtime").toUpperCase());
  setText("uptime", `${snapshot.uptime_sec || 0} s`);
  renderSafety(snapshot.safety || {});
  renderMetrics(snapshot.streams || {});
  renderPipeline(snapshot);
  renderVr(snapshot);
  renderCommands(snapshot.commands || {});
  renderJoints(snapshot);
  renderStreams(snapshot.streams || {});
  renderTargets(snapshot.cartesian || {});
  renderBackend(snapshot.backend_candidates || {});
  renderDiagnostics(snapshot.diagnostics || {});
  renderRuntime(snapshot);
}

function confirmAction(title, message, danger = false) {
  const dialog = $("confirm-dialog");
  setText("dialog-title", title);
  setText("dialog-message", message);
  setText("dialog-symbol", danger ? "!!" : "!");
  $("dialog-confirm").className = danger ? "danger" : "primary";
  dialog.showModal();
  return new Promise((resolve) => {
    dialog.addEventListener("close", () => resolve(dialog.returnValue === "confirm"), { once: true });
  });
}

async function request(path, body) {
  const buttons = $$("button");
  buttons.forEach((button) => { button.disabled = true; });
  try {
    const response = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: body === undefined ? undefined : JSON.stringify(body) });
    const result = await response.json();
    if (!response.ok || !result.success) throw new Error(result.reason || `HTTP ${response.status}`);
    showToast(result.reason || "操作完成");
  } catch (error) {
    showToast(`操作失败：${error.message}`, true);
  } finally {
    buttons.forEach((button) => { button.disabled = false; });
  }
}

function setConnection(online) {
  const element = $("socket-state");
  element.className = online ? "connection online" : "connection offline";
  element.querySelector("span").textContent = online ? "实时状态已连接" : "连接断开，正在重试";
  setText("runtime-websocket", online ? "已连接 · 4 Hz" : "未连接");
}

function connect() {
  clearTimeout(reconnectTimer);
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${scheme}://${location.host}/ws`);
  socket.addEventListener("open", () => setConnection(true));
  socket.addEventListener("message", (event) => {
    try { render(JSON.parse(event.data)); } catch (error) { showToast(`状态解析失败：${error.message}`, true); }
  });
  socket.addEventListener("close", () => {
    setConnection(false);
    reconnectTimer = setTimeout(connect, 1200);
  });
  socket.addEventListener("error", () => socket.close());
}

window.addEventListener("hashchange", route);
$("joint-filter").addEventListener("input", () => latestSnapshot && renderJoints(latestSnapshot));
$("enable-button").addEventListener("click", async () => {
  if (await confirmAction("启用机器人控制", "确认机器人周围无人，并已检查初始姿态、映射方向和关节限位。")) request("/api/v1/safety/enabled", { enabled: true });
});
$("disable-button").addEventListener("click", () => request("/api/v1/safety/enabled", { enabled: false }));
$("reset-button").addEventListener("click", async () => {
  if (await confirmAction("复位故障锁存", "仅在故障原因已经排除后复位。复位不会自动启用控制。", true)) request("/api/v1/safety/reset");
});
route();
connect();
