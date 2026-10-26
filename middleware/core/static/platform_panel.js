/* Platform secrets are submitted once to the local service and never persisted here. */
(() => {
  const el = id => document.getElementById(id);
  let state = { connection: null, recent: null }, selected = '', page = 1, total = 0;
  let tasks = [], requestId = null, refreshGeneration = 0, active = false;
  let connecting = false, reconnecting = false, verificationFailed = false;
  let connectionGeneration = 0;
  let selectionSave = Promise.resolve(), selectionVersion = 0, pendingSelection = 0, selectionError = null;
  let historyTarget = null, historySelected = '', historyPage = 1;
  const post = (path, body = {}) => call(path, { method: 'POST', body: JSON.stringify(body) });
  async function call(path, options = {}) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 20000);
    const timeoutError = () => Object.assign(
      new Error('采集端请求超时（20 秒），请检查采集端与平台服务后重试'),
      { code: 'ROBOT_SERVICE_TIMEOUT', retryable: true });
    try {
      let response;
      try {
        response = await fetch('/api/platform/' + path, { cache: 'no-store', ...options,
          signal: controller.signal,
          headers: { 'Content-Type': 'application/json' } });
      } catch (_) {
        if (controller.signal.aborted) throw timeoutError();
        throw Object.assign(new Error('浏览器无法连接采集端，请检查采集端服务和浏览器网络后重试'),
          { code: 'ROBOT_SERVICE_UNREACHABLE', retryable: true });
      }
      let body;
      try { body = await response.json(); } catch (_) {
        if (controller.signal.aborted) throw timeoutError();
        throw Object.assign(new Error(`采集端返回了无效响应（HTTP ${response.status}），请检查采集端服务后重试`),
          { code: 'ROBOT_SERVICE_RESPONSE_INVALID', retryable: true });
      }
      if (!response.ok) {
        const code = typeof body?.code === 'string' && /^[A-Z][A-Z0-9_]{0,95}$/.test(body.code) ? body.code : '';
        throw Object.assign(new Error(`${body?.reason || '平台请求失败'}${code ? `（${code}）` : ''}`),
          { code, retryable: body?.retryable === true });
      }
      return body;
    } finally { clearTimeout(timer); }
  }
  const message = value => { el('platformMessage').textContent = value; };
  const connectionMessage = (value, failed = false) => {
    el('platformConnectMessage').textContent = value;
    el('platformConnectMessage').classList.toggle('error', failed);
  };
  function selection() {
    if (!state.connection || !selected) return { mode: 'local' };
    const revision = tasks.find(task => task.collection_task_id === selected)?.capture_plan?.revision;
    return { mode: 'platform', connection_id: state.connection.id, task_id: selected,
      ...(revision ? { expected_plan_revision: revision } : {}) };
  }
  window.platformRecordingSelection = async () => {
    await ready;
    await selectionSave;
    if (selectionError) throw selectionError;
    requestId ||= uuid();
    return { ...(state.recording_selection || selection()), request_id: requestId };
  };
  function taskDescription(task) {
    const plan = task?.capture_plan;
    if (!plan) return '这个任务尚未配置采集方案，请在平台补充后使用。';
    const scheme = plan.scheme ? `${plan.scheme.scheme_id} v${plan.scheme.version}` : '未命名方案';
    const optional = Object.values(plan.output?.features || {}).filter(feature => feature.required === false).length;
    return `${scheme} · ${plan.source_format} · 映射 v${plan.mapping_version}` +
      ` · ${optional ? `${optional} 项可选` : '无可选项'}` +
      (task.selectable === false ? ` · 暂不可用：${task.reason || '请在平台检查方案'}` : '');
  }
  function taskOption(task) {
    const plan = task.capture_plan;
    const option = new Option(`${task.name} / ${task.project_name} / ${task.task_code}` +
      (!plan ? ' · 方案待配置' : task.selectable === false ? ` · ${task.reason || '暂不可用'}` : ''), task.collection_task_id);
    option.title = taskDescription(task);
    option.disabled = task.selectable === false;
    return option;
  }
  function renderSelection() {
    const value = state.recording_selection;
    selected = value?.mode === 'platform' && value.connection_id === state.connection?.id ? value.task_id : '';
    if (selected && ![...el('platformTasks').options].some(option => option.value === selected)) {
      el('platformTasks').append(new Option('已选任务（正在验证）', selected));
    }
    el('platformTasks').value = selected;
  }
  function saveSelection() {
    const value = selection(), version = ++selectionVersion;
    requestId = null; selectionError = null; pendingSelection++;
    message('正在自动保存录制任务…');
    // Serialize rapid changes so an older response cannot overwrite the latest
    // choice used by the controller. The browser also waits before starting.
    selectionSave = selectionSave.then(async () => {
      try {
        const result = await post('selection', value);
        state.recording_selection = result.recording_selection;
        if (version === selectionVersion) {
          renderSelection();
          message(selected ? '已关联采集任务，网页与手柄录制共用。' : '未关联平台任务，网页或手柄可直接开始本地录制。');
        }
      } catch (error) {
        if (version === selectionVersion) {
          selectionError = error;
          message(`设置保存失败：${error.message}。手柄仍使用上次保存的设置，请重新选择。`);
        }
      } finally { pendingSelection--; }
    });
    return selectionSave;
  }
  function uuid() {
    // crypto.randomUUID is unavailable on many LAN HTTP origins.
    return Array.from(crypto.getRandomValues(new Uint8Array(16)), b => b.toString(16).padStart(2, '0')).join('');
  }
  window.platformRecordingStopped = () => { requestId = null; void refreshRecords(); };
  window.platformRecordingStatus = recording => {
    active = Boolean(recording?.recording);
    const task = recording?.task_recording?.binding?.task;
    el('platformActive').textContent = active ? `正在录制：${task?.name || '未关联平台任务，保存到本机'}` : '';
    for (const id of ['platformEnabled', 'platformTasks', 'platformAuto', 'platformConnect', 'platformUploadFormat']) {
      const target = el(id);
      if (target) target.disabled = active;
    }
    connectionControls();
    if (!pendingSelection && !selectionError && recording?.recording_selection &&
        JSON.stringify(recording.recording_selection) !== JSON.stringify(state.recording_selection)) {
      state.recording_selection = recording.recording_selection;
      requestId = null; renderSelection();
    }
    if (active) el('platformHistoryDialog').close();
  };
  function syncUploadFormatOptions(localFormat) {
    const formatField = el('platformUploadFormat');
    if (!formatField) return;
    const hint = el('platformFormatHint');
    const mcapOption = [...formatField.options].find(o => o.value === 'mcap');
    const lerobotOption = [...formatField.options].find(o => o.value === 'lerobot');
    if (localFormat === 'mcap') {
      if (mcapOption) { mcapOption.disabled = false; mcapOption.textContent = 'MCAP · 原始话题'; }
      if (lerobotOption) { lerobotOption.disabled = true; lerobotOption.textContent = 'LeRobot · 训练数据集（本地未保存）'; }
      if (formatField.value !== 'mcap') {
        formatField.value = 'mcap';
        if (state.upload_format !== 'mcap') {
          void post('upload-format', { format: 'mcap' }).then(res => { state = res; renderConnection(); }).catch(() => {});
        }
      }
      if (hint) hint.textContent = '本地保存格式为 MCAP，上传格式固定为 MCAP。';
    } else if (localFormat === 'lerobot') {
      if (lerobotOption) { lerobotOption.disabled = false; lerobotOption.textContent = 'LeRobot · 训练数据集'; }
      if (mcapOption) { mcapOption.disabled = true; mcapOption.textContent = 'MCAP · 原始话题（本地未保存）'; }
      if (formatField.value !== 'lerobot') {
        formatField.value = 'lerobot';
        if (state.upload_format !== 'lerobot') {
          void post('upload-format', { format: 'lerobot' }).then(res => { state = res; renderConnection(); }).catch(() => {});
        }
      }
      if (hint) hint.textContent = '本地保存格式为 LeRobot，上传格式固定为 LeRobot。';
    } else { // both
      if (mcapOption) { mcapOption.disabled = false; mcapOption.textContent = 'MCAP · 原始话题'; }
      if (lerobotOption) { lerobotOption.disabled = false; lerobotOption.textContent = 'LeRobot · 训练数据集'; }
      formatField.value = state.upload_format || 'lerobot';
      if (hint) hint.textContent = '本地同时保存 MCAP 与 LeRobot，可任选一种上传格式。';
    }
  }
  window.platformSyncLocalFormat = syncUploadFormatOptions;
  function connectionControls() {
    const enabled = state.uploads_enabled !== false;
    el('platformEnabled').disabled = active;
    el('platformConnect').disabled = !enabled || active || connecting || reconnecting;
    el('platformReconnect').disabled = !enabled || connecting || reconnecting || !state.connection;
    el('platformReconnect').hidden = !state.connection;
    el('platformReconnect').textContent = reconnecting ? '正在重新连接…' : '重新连接';
    el('platformReconnectHint').hidden = !state.connection;
    for (const id of ['platformRefresh', 'platformSearch', 'platformProject', 'platformPrev', 'platformNext']) {
      el(id).disabled = !enabled;
    }
  }
  function renderConnection() {
    const enabled = state.uploads_enabled !== false;
    const connection = state.connection;
    const status = reconnecting ? '正在重新连接…' : verificationFailed ? '连接未恢复' : connection?.verified ? '已连接' : '等待验证';
    el('platformConnection').textContent = !enabled ? '平台上传功能已完全关闭' : connection
      ? `平台：${connection.platform_name} · 机器人：${connection.robot_name} · ${status}`
      : '尚未连接平台';
    el('platformEnabled').checked = enabled;
    el('platformAuto').checked = state.auto_upload === true;
    el('platformAuto').disabled = !enabled || active;
    el('platformTasks').disabled = !enabled || active;
    const localFmt = state.local_format || document.getElementById('topicRecordingFormat')?.value || 'both';
    syncUploadFormatOptions(localFmt);
    el('platformUploadFormat').disabled = !enabled || active;
    connectionControls();
  }
  async function refreshTasks() {
    if (state.uploads_enabled === false) return;
    if (!state.connection) return;
    const generation = ++refreshGeneration;
    const connectionId = state.connection.id;
    try {
      const params = new URLSearchParams({ connection_id: connectionId, q: el('platformSearch').value,
        page: String(page), ...(el('platformProject').value ? { project_id: el('platformProject').value } : {}) });
      const response = await call('tasks?' + params);
      if (generation !== refreshGeneration || connectionId !== state.connection?.id) return;
      tasks = response.items; total = response.total;
      const projectValue = el('platformProject').value;
      el('platformProject').replaceChildren(new Option('全部项目', ''), ...response.projects.map(p => new Option(p.name, p.id)));
      el('platformProject').value = projectValue;
      el('platformTasks').replaceChildren(new Option('请选择采集任务', ''), ...tasks.map(taskOption));
      if (selected && !tasks.some(t => t.collection_task_id === selected)) {
        el('platformTasks').append(new Option('原选择（正在验证）', selected));
      }
      el('platformTasks').value = selected;
      el('platformPage').textContent = `第 ${page} 页 · ${total} 个可选任务`;
      el('platformPrev').disabled = page <= 1;
      el('platformNext').disabled = page * 20 >= total;
      if (selected) {
        try {
          const binding = await post('preflight', { mode: 'platform', connection_id: connectionId, task_id: selected });
          if (generation !== refreshGeneration || connectionId !== state.connection?.id || selected !== binding.task.collection_task_id) return;
          const option = [...el('platformTasks').options].find(o => o.value === selected);
          if (option) option.textContent = `${binding.task.name} / ${binding.task.project_name} / ${binding.task.task_code}`;
          el('platformDescription').textContent = taskDescription(binding.task);
          const savedRevision = state.recording_selection?.expected_plan_revision;
          message(savedRevision && savedRevision !== binding.task.capture_plan?.revision
            ? '任务计划修订已变化，请明确重新选择任务后开始录制。原录制快照保持不变。'
            : '任务已刷新。开始录制时准备并固定本段方案；方案选择不会启动设备发布。');
        } catch (error) {
          if (generation === refreshGeneration) message(`原选择已不可用或验证失败：${error.message}。请选择其他任务，或将任务置空后保存到本机。`);
        }
      } else message('未关联平台任务，网页或手柄可直接开始本地录制。');
    } catch (error) {
      if (generation === refreshGeneration) message(`任务刷新失败：${error.message}。已选任务保留；断网时可将任务置空，照常保存到本机。`);
    }
  }
  async function refreshRecords() {
    if (state.uploads_enabled === false) {
      const body = el('platformRecordings'); body.replaceChildren();
      const row = document.createElement('tr'), cell = document.createElement('td'); cell.colSpan = 8;
      cell.textContent = '上传总开关已关闭，不扫描、不重试、不刷新上传记录。'; row.append(cell); body.append(row);
      return;
    }
    try {
      const result = await call('recordings');
      const rows = result.items.slice().reverse();
      for (const job of result.jobs) {
        if (!rows.some(record => record.recording_id === job.local_recording_id)) rows.push({
          recording_id: 'legacy:' + job.id, legacy_job: job, path: job.source_export_id || '历史上传作业',
          binding: { task: { name: `原任务（${job.task_id}）` } }, state: job.state,
        });
      }
      const labels = { ready: '已整理', queued: '等待上传', upload_pending: '等待上传', recording: '正在录制',
        preparing: '准备录制中', prepared: '方案已固定，等待设备就绪', prepare_failed: '准备失败，请按原请求重试',
        stopping: '正在收尾', finalizing: '整理中', interrupted: '录制中断', needs_attention: '需要处理',
        uploading: '上传中', raw_committed: '已上传', processing: '已上传', ready_upload: '已上传' };
      const processingLabels = { UNKNOWN: '未知', NOT_STARTED: '尚未开始', PENDING: '等待处理',
        PROCESSING: '处理中', DISCOVERING_EPISODES: '识别片段中', READY: '处理完成', FAILED: '处理失败',
        PARTIALLY_FAILED: '部分处理失败', AWAITING_CONFIGURATION: '采集方案待配置', STRUCTURE_MISMATCH: '实际结构与方案不一致',
        AWAITING_SLICE: '等待切片', SLICED: '已切片' };
      const qualityLabels = { UNKNOWN: '未知', PENDING: '等待质检', NOT_EVALUATED: '尚未质检',
        PASS: '通过', RISK: '存在风险', REJECT: '未通过' };
      const body = el('platformRecordings'); body.replaceChildren();
      for (const record of rows) {
        const job = record.legacy_job || result.jobs.find(j => j.local_recording_id === record.recording_id);
        const row = document.createElement('tr');
        const uploaded = job?.server_state === 'COMMITTED';
        const annotation = { NOT_CONFIGURED: '标注待配置', FAILED: '标注创建失败', READY: '标注任务已创建',
          COMPLETED: '标注完成', PENDING: '等待标注', NOT_REQUIRED: '不要求标注', UNKNOWN: '标注状态未知' };
        const dataState = { READABLE: '可读', COMMITTED: '已提交', READY: '就绪', PENDING: '处理中', FAILED: '未完成' };
        const trainingState = { READY: '就绪', NOT_READY: '未就绪', BLOCKED: '待处理', PENDING: '准备中', NOT_REQUESTED: '尚未请求' };
        const receipt = job ? `数据：${dataState[job.data_status] || '状态未知'}；${annotation[job.annotation_status] || '标注状态未知'}；训练交付：${trainingState[job.training_status] || '状态未知'}` : '';
        const formatLabel = { mcap: 'MCAP', lerobot: 'LeRobot v3', mcap_mp4: 'MCAP＋MP4 原始包' }[record.format];
        const values = [record.display_name || record.path.split('/').pop() || '启动中', record.binding?.task?.name || record.preparation?.checked?.task?.name || '未选择任务',
          formatLabel || (job?.kind === 'hc_lerobot' ? 'LeRobot v3' : 'MCAP'),
          uploaded ? '已上传' : labels[job?.state || record.state] || job?.state || record.state,
          processingLabels[job?.processing_state] || job?.processing_state || '尚未开始',
          qualityLabels[job?.quality_state] || job?.quality_state || '尚未质检', [record.error || job?.next_action, receipt].filter(Boolean).join('；')];
        for (const value of values) { const td = document.createElement('td'); td.textContent = value; row.append(td); }
        const action = document.createElement('td'), button = document.createElement('button');
        button.textContent = !record.binding ? '选择任务后上传' : job ? '重试 / 刷新结果' : '上传';
        button.disabled = ['preparing', 'prepared', 'prepare_failed', 'starting', 'recording', 'stopping', 'finalizing', 'interrupted', 'failed', 'deleted'].includes(record.state);
        button.onclick = async () => {
          if (!record.binding) { await openHistory(record.upload_target || record.recording_id, values[0], record.format === 'lerobot' ? 'LEROBOT_V3' : record.format === 'mcap_mp4' ? 'CAPTURE_BUNDLE' : 'MCAP'); return; }
          if (record.legacy_job) {
            try { await post('retry-upload', { job_id: job.id }); await refreshRecords(); }
            catch (error) { message(error.message); }
            return;
          }
          try { await post('upload-recording', { recording_id: record.upload_target || record.recording_id }); await refreshRecords(); }
          catch (error) { message(error.message); }
        };
        action.append(button);
        if (record.binding) {
          const reupload = document.createElement('button');
          reupload.textContent = '另选任务上传';
          reupload.style.marginLeft = '6px';
          reupload.disabled = ['starting', 'recording', 'stopping', 'finalizing', 'deleted'].includes(record.state);
          reupload.onclick = async () => {
            await openHistory(record.source_recording_id || record.recording_id, values[0], (record.upload_format || record.format) === 'lerobot' ? 'LEROBOT_V3' : 'MCAP');
          };
          action.append(reupload);
        }
        row.append(action); body.append(row);
      }
      if (!rows.length) { const row = document.createElement('tr'), cell = document.createElement('td'); cell.colSpan = 8;
        cell.textContent = '录制后会在这里显示整理、上传、平台处理和质检状态。'; row.append(cell); body.append(row); }
    } catch (error) { message(error.message); }
  }
  window.platformRefreshRecords = refreshRecords;
  async function historyTasks() {
    el('platformHistoryUpload').disabled = true;
    el('platformHistoryError').textContent = '正在加载任务…';
    try {
      const result = await call('tasks?' + new URLSearchParams({ connection_id: historyTarget.connection_id,
        source_format: historyTarget.source_format, q: el('platformHistorySearch').value, page: String(historyPage) }));
      const select = el('platformHistoryTask');
      select.replaceChildren(new Option('为此录制选择任务', ''), ...result.items.map(t => new Option(`${t.name} / ${t.project_name} / ${t.task_code}`, t.collection_task_id)));
      if (historySelected && !result.items.some(t => t.collection_task_id === historySelected)) select.append(new Option('已选任务（开始上传时验证）', historySelected));
      select.value = historySelected;
      if (el('platformHistoryPage')) {
        el('platformHistoryPage').textContent = `第 ${historyPage} 页 · 共 ${result.total || result.items.length} 个任务`;
      }
      el('platformHistoryError').textContent = result.items.length ? '' : '没有匹配的任务，请调整搜索条件或在平台创建任务。';
      el('platformHistoryUpload').disabled = !historySelected;
      el('platformHistoryPrev').disabled = historyPage <= 1; el('platformHistoryNext').disabled = historyPage * 20 >= (result.total || result.items.length);
    } catch (error) { el('platformHistoryError').textContent = error.message; }
  }
  async function openHistory(identity, name, source_format = 'MCAP') {
    return openHistoryBatch([{ identity, name, source_format }], source_format);
  }
  async function openHistoryBatch(targets, source_format) {
    if (state.uploads_enabled === false) { message('上传总开关已关闭，请先开启平台上传功能'); return; }
    if (!state.connection) { message('请先连接平台，再为历史录制选择任务'); return; }
    historyTarget = { targets, connection_id: state.connection.id, source_format }; historySelected = ''; historyPage = 1;
    el('platformHistoryName').textContent = targets.length === 1 ? targets[0].name : `已选择 ${targets.length} 个数据集：${targets.map(item => item.name).join('、')}`;
    el('platformHistoryError').textContent = '';
    el('platformHistoryFormat').textContent = `本次上传格式：${source_format === 'LEROBOT_V3' ? 'LeRobot 数据集' : 'MCAP 原始录制'}`;
    el('platformHistorySearch').value = '';
    el('platformHistoryUpload').disabled = true;
    el('platformHistoryUpload').textContent = '选择并上传';
    el('platformHistoryDialog').showModal();
    await historyTasks();
  }
  window.platformUploadHistory = openHistory;
  window.platformUploadBatch = openHistoryBatch;
  async function init() {
    el('platformEnabled').onchange = async () => {
      const enabled = el('platformEnabled').checked;
      el('platformEnabled').disabled = true;
      try {
        state = await post('enabled', { enabled });
        requestId = null; selectionError = null; renderSelection(); renderConnection();
        if (enabled) { await refreshTasks(); await refreshRecords(); }
        else { await refreshRecords(); message('上传功能已关闭；本地录制与保存继续可用。'); }
      } catch (error) { renderConnection(); message(error.message); }
      finally { el('platformEnabled').disabled = active; }
    };
    el('platformReconnect').onclick = async () => {
      if (connecting || reconnecting || !state.connection) return;
      connectionGeneration++;
      reconnecting = true;
      renderConnection();
      connectionMessage('正在使用已保存的连接信息重新连接平台…');
      try {
        const result = await post('verify');
        // Only refresh connection health. Task choices, pending saves and the
        // current recording keep their original state throughout a reconnect.
        state.connection = result.connection;
        state.error = result.error;
        verificationFailed = false;
        connectionMessage('重新连接成功，已沿用保存的连接信息。');
        await refreshTasks();
        await refreshRecords();
      } catch (error) {
        verificationFailed = true;
        connectionMessage(`重新连接失败：${error.message}。已保存的连接信息仍保留。` +
          (error.retryable ? '网络恢复后可再次点击“重新连接”，无需重新粘贴链接。' : ''), true);
      } finally { reconnecting = false; renderConnection(); }
    };
    el('platformConnect').onclick = async () => {
      if (connecting || reconnecting || active) return;
      connectionGeneration++;
      connecting = true;
      connectionControls();
      const link = el('platformLink').value;
      el('platformLink').value = '';
      connectionMessage('正在连接平台…');
      try { await selectionSave; state = await post('connect', { link }); requestId = null; selectionError = null; verificationFailed = false;
        connectionMessage('连接成功，请选择采集任务。');
        renderSelection(); renderConnection(); await refreshTasks(); }
      catch (error) {
        connectionMessage(`连接失败：${error.message}。${state.connection ? '原连接设置已保留。' : ''}` +
          (error.retryable ? '排除故障后，可重新粘贴同一链接重试；若链接已过期，请重新生成。' : ''), true);
      }
      finally { connecting = false; connectionControls(); }
    };
    el('platformRefresh').onclick = () => void refreshTasks();
    el('platformSearch').oninput = () => { page = 1; void refreshTasks(); };
    el('platformProject').onchange = () => { page = 1; void refreshTasks(); };
    el('platformPrev').onclick = () => { page--; void refreshTasks(); };
    el('platformNext').onclick = () => { page++; void refreshTasks(); };
    el('platformTasks').onchange = () => { selected = el('platformTasks').value;
      el('platformDescription').textContent = taskDescription(tasks.find(task => task.collection_task_id === selected));
      return saveSelection(); };
    el('platformUploadFormat').onchange = async () => {
      const field = el('platformUploadFormat'); field.disabled = true;
      try { state = await post('upload-format', { format: field.value }); renderConnection();
        requestId = null; await refreshTasks(); message('上传格式已保存，仅对之后开始的录制生效。');
      } catch (error) { renderConnection(); message(error.message); }
      finally { field.disabled = active; }
    };
    el('platformAuto').onchange = async () => {
      try { state = await post('auto-upload', { enabled: el('platformAuto').checked }); renderConnection(); }
      catch (error) { renderConnection(); message(error.message); }
    };
    el('platformHistoryClose').onclick = () => el('platformHistoryDialog').close();
    el('platformHistorySearch').oninput = () => { historyPage = 1; void historyTasks(); };
    el('platformHistoryPrev').onclick = () => { historyPage--; void historyTasks(); };
    el('platformHistoryNext').onclick = () => { historyPage++; void historyTasks(); };
    el('platformHistoryTask').onchange = () => { historySelected = el('platformHistoryTask').value; el('platformHistoryUpload').disabled = !historySelected; };
    el('platformHistoryUpload').onclick = async () => {
      try {
        if (!historySelected) throw new Error('请明确选择这条录制的任务');
        el('platformHistoryUpload').disabled = true;
        el('platformHistoryClose').disabled = true;
        el('platformHistoryUpload').textContent = '正在提交…';
        const selection = { mode: 'platform', connection_id: historyTarget.connection_id, task_id: historySelected };
        const targets = historyTarget.targets;
        if (targets.length === 1) {
          await post('upload-recording', { recording_id: targets[0].identity, selection,
            source_format: historyTarget.source_format, new_upload: true });
          el('platformHistoryDialog').close();
        } else {
          const result = await post('upload-recordings', { selection,
            recordings: targets.map(item => ({ recording_id: item.identity, source_format: item.source_format })) });
          if (result.failed?.length) {
            el('platformHistoryError').textContent = `已提交 ${result.uploaded?.length || 0} 个，失败 ${result.failed.length} 个：` +
              result.failed.map(item => `${item.recording_id}: ${item.reason}`).join('；');
          } else el('platformHistoryDialog').close();
        }
        await refreshRecords();
      } catch (error) { el('platformHistoryError').textContent = error.message; }
      finally {
        el('platformHistoryUpload').textContent = '选择并上传';
        el('platformHistoryUpload').disabled = !historySelected;
        el('platformHistoryClose').disabled = false;
      }
    };
    const generation = connectionGeneration;
    try {
      const initial = await call('status');
      if (generation === connectionGeneration) {
        state = initial;
        renderSelection(); renderConnection();
        if (state.uploads_enabled !== false && state.connection) {
          try {
            const verified = await post('verify');
            if (generation === connectionGeneration) { state = verified; renderConnection(); }
          } catch (error) {
            if (generation === connectionGeneration) {
              verificationFailed = true;
              renderConnection();
              connectionMessage(`当前连接验证失败：${error.message}。` +
                (error.retryable ? '网络恢复后点击“重新连接”即可，无需重新粘贴链接。' : ''), true);
            }
          }
        }
        if (generation === connectionGeneration) { await refreshTasks(); await refreshRecords(); }
      }
    } catch (error) {
      if (generation === connectionGeneration) {
        el('platformConnection').textContent = '平台连接状态读取失败';
        connectionMessage(error.message, true);
      }
    }
    const visibleRefresh = () => { if (!document.hidden && state.uploads_enabled !== false) { void refreshTasks(); void refreshRecords(); } };
    const timer = setInterval(visibleRefresh, 30000);
    const progressTimer = setInterval(() => { if (!document.hidden && state.uploads_enabled !== false) void refreshRecords(); }, 5000);
    document.addEventListener('visibilitychange', visibleRefresh);
    window.addEventListener('pagehide', () => { clearInterval(timer); clearInterval(progressTimer); refreshGeneration++; }, { once: true });
  }
  const ready = init();
})();
