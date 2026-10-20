/* Platform secrets are submitted once to the local service and never persisted here. */
(() => {
  const el = id => document.getElementById(id);
  let state = { connection: null, recent: null }, selected = '', page = 1, total = 0;
  let tasks = [], requestId = null, refreshGeneration = 0, active = false;
  let selectionSave = Promise.resolve(), selectionVersion = 0, pendingSelection = 0, selectionError = null;
  let choosingTask = false;
  let historyTarget = null, historySelected = '', historyPage = 1;
  const post = (path, body = {}) => call(path, { method: 'POST', body: JSON.stringify(body) });
  async function call(path, options = {}) {
    const response = await fetch('/api/platform/' + path, { cache: 'no-store', ...options,
      headers: { 'Content-Type': 'application/json' } });
    const body = await response.json();
    if (!response.ok) throw new Error(body.reason || body.code || '平台请求失败');
    return body;
  }
  const message = value => { el('platformMessage').textContent = value; };
  const taskPrompt = () => (state.connection ? '请选择采集任务' : '请先连接平台，再选择采集任务') +
    '，选定后自动保存关联；选择前仍使用上次保存的录制设置。';
  function selection() {
    if (el('platformLocal').checked || !state.connection || !selected) return { mode: 'local' };
    return { mode: 'platform', connection_id: state.connection.id, task_id: selected };
  }
  window.platformRecordingSelection = async () => {
    await ready;
    await selectionSave;
    if (selectionError) throw selectionError;
    requestId ||= uuid();
    return { ...selection(), request_id: requestId };
  };
  function renderSelection() {
    const value = state.recording_selection;
    selected = value?.mode === 'platform' && value.connection_id === state.connection?.id ? value.task_id : '';
    if (selected) choosingTask = false;
    el('platformLocal').checked = !selected && !choosingTask;
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
          message(choosingTask ? taskPrompt() : '设置已自动保存，网页与手柄录制共用。');
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
    for (const id of ['platformTasks', 'platformLocal', 'platformAuto', 'platformConnect']) el(id).disabled = active;
    if (!pendingSelection && !selectionError && recording?.recording_selection &&
        JSON.stringify(recording.recording_selection) !== JSON.stringify(state.recording_selection)) {
      state.recording_selection = recording.recording_selection;
      requestId = null; renderSelection();
    }
    if (active) el('platformHistoryDialog').close();
  };
  function renderConnection() {
    const connection = state.connection;
    el('platformConnection').textContent = connection
      ? `平台：${connection.platform_name} · 机器人：${connection.robot_name} · ${connection.verified ? '已连接' : '等待验证'}`
      : '尚未连接平台';
    el('platformAuto').checked = state.auto_upload === true;
  }
  async function refreshTasks() {
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
      el('platformTasks').replaceChildren(new Option('请选择采集任务', ''), ...tasks.map(t => new Option(`${t.name} / ${t.project_name} / ${t.task_code}`, t.collection_task_id)));
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
          el('platformDescription').textContent = binding.task.description || '任务未填写说明';
          message('任务已刷新。开始录制时会再次验证并固定本段任务。');
        } catch (error) {
          if (generation === refreshGeneration) message(`原选择已不可用或验证失败：${error.message}。请选择其他任务，或勾选“暂不关联平台任务”后保存到本机。`);
        }
      } else message(choosingTask ? taskPrompt() : '未关联平台任务，网页或手柄可直接开始本地录制。');
    } catch (error) {
      if (generation === refreshGeneration) message(`任务刷新失败：${error.message}。已选任务保留；断网时可勾选“暂不关联平台任务”，照常保存到本机。`);
    }
  }
  async function refreshRecords() {
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
        stopping: '正在收尾', finalizing: '整理中', interrupted: '录制中断', needs_attention: '需要处理',
        uploading: '上传中', raw_committed: '已上传', processing: '已上传', ready_upload: '已上传' };
      const processingLabels = { UNKNOWN: '未知', NOT_STARTED: '尚未开始', PENDING: '等待处理',
        PROCESSING: '处理中', DISCOVERING_EPISODES: '识别片段中', READY: '处理完成', FAILED: '处理失败',
        PARTIALLY_FAILED: '部分处理失败', AWAITING_SLICE: '等待切片', SLICED: '已切片' };
      const qualityLabels = { UNKNOWN: '未知', PENDING: '等待质检', NOT_EVALUATED: '尚未质检',
        PASS: '通过', RISK: '存在风险', REJECT: '未通过' };
      const body = el('platformRecordings'); body.replaceChildren();
      for (const record of rows) {
        const job = record.legacy_job || result.jobs.find(j => j.local_recording_id === record.recording_id);
        const row = document.createElement('tr');
        const uploaded = job?.server_state === 'COMMITTED';
        const values = [record.path.split('/').pop() || '启动中', record.binding?.task?.name || '未选择任务',
          uploaded ? '已上传' : labels[job?.state || record.state] || job?.state || record.state,
          processingLabels[job?.processing_state] || job?.processing_state || '尚未开始',
          qualityLabels[job?.quality_state] || job?.quality_state || '尚未质检', record.error || job?.next_action || ''];
        for (const value of values) { const td = document.createElement('td'); td.textContent = value; row.append(td); }
        const action = document.createElement('td'), button = document.createElement('button');
        button.textContent = !record.binding ? '选择任务后上传' : job ? '重试 / 刷新结果' : '上传';
        button.disabled = ['starting', 'recording', 'stopping', 'finalizing', 'interrupted', 'failed', 'deleted'].includes(record.state);
        button.onclick = async () => {
          if (!record.binding) { await openHistory(record.recording_id, values[0], record.format === 'lerobot' ? 'LEROBOT_V3' : 'MCAP'); return; }
          if (record.legacy_job) {
            try { await post('retry-upload', { job_id: job.id }); await refreshRecords(); }
            catch (error) { message(error.message); }
            return;
          }
          try { await post('upload-recording', { recording_id: record.recording_id }); await refreshRecords(); }
          catch (error) { message(error.message); }
        };
        action.append(button); row.append(action); body.append(row);
      }
      if (!rows.length) { const row = document.createElement('tr'), cell = document.createElement('td'); cell.colSpan = 7;
        cell.textContent = '录制后会在这里显示整理、上传、平台处理和质检状态。'; row.append(cell); body.append(row); }
    } catch (error) { message(error.message); }
  }
  window.platformRefreshRecords = refreshRecords;
  async function historyTasks() {
    try {
      const result = await call('tasks?' + new URLSearchParams({ connection_id: historyTarget.connection_id,
        source_format: historyTarget.source_format, q: el('platformHistorySearch').value, page: String(historyPage) }));
      const select = el('platformHistoryTask');
      select.replaceChildren(new Option('为此录制选择任务', ''), ...result.items.map(t => new Option(`${t.name} / ${t.project_name} / ${t.task_code}`, t.collection_task_id)));
      if (historySelected && !result.items.some(t => t.collection_task_id === historySelected)) select.append(new Option('已选任务（开始上传时验证）', historySelected));
      select.value = historySelected;
      el('platformHistoryPrev').disabled = historyPage <= 1; el('platformHistoryNext').disabled = historyPage * 20 >= result.total;
    } catch (error) { el('platformHistoryError').textContent = error.message; }
  }
  async function openHistory(identity, name, source_format = 'MCAP') {
    if (!state.connection) { message('请先连接平台，再为历史录制选择任务'); return; }
    historyTarget = { identity, connection_id: state.connection.id, source_format }; historySelected = ''; historyPage = 1;
    el('platformHistoryName').textContent = name; el('platformHistoryError').textContent = '';
    el('platformHistorySearch').value = ''; el('platformHistoryDialog').showModal(); await historyTasks();
  }
  window.platformUploadHistory = openHistory;
  async function init() {
    el('platformConnect').onclick = async () => {
      el('platformConnect').disabled = true;
      const link = el('platformLink').value;
      el('platformLink').value = '';
      try { await selectionSave; state = await post('connect', { link }); requestId = null; selectionError = null; choosingTask = false;
        renderSelection(); renderConnection(); await refreshTasks(); }
      catch (error) { message(`连接失败，原连接已保留：${error.message}。可重新粘贴同一链接重试。`); }
      finally { el('platformConnect').disabled = false; }
    };
    el('platformRefresh').onclick = () => void refreshTasks();
    el('platformSearch').oninput = () => { page = 1; void refreshTasks(); };
    el('platformProject').onchange = () => { page = 1; void refreshTasks(); };
    el('platformPrev').onclick = () => { page--; void refreshTasks(); };
    el('platformNext').onclick = () => { page++; void refreshTasks(); };
    el('platformTasks').onchange = () => { choosingTask = false; selected = el('platformTasks').value; el('platformLocal').checked = !selected;
      el('platformDescription').textContent = tasks.find(task => task.collection_task_id === selected)?.description || '';
      return saveSelection(); };
    el('platformLocal').onchange = () => {
      choosingTask = !el('platformLocal').checked && !selected;
      if (choosingTask) {
        // Wait for an explicit task choice instead of saving local mode and
        // checking the box again when the save or status update completes.
        message(taskPrompt());
        el('platformTasks').focus();
        return selectionSave;
      }
      return saveSelection();
    };
    el('platformAuto').onchange = async () => {
      try { state = await post('auto-upload', { enabled: el('platformAuto').checked }); renderConnection(); }
      catch (error) { renderConnection(); message(error.message); }
    };
    el('platformHistoryClose').onclick = () => el('platformHistoryDialog').close();
    el('platformHistorySearch').oninput = () => { historyPage = 1; void historyTasks(); };
    el('platformHistoryPrev').onclick = () => { historyPage--; void historyTasks(); };
    el('platformHistoryNext').onclick = () => { historyPage++; void historyTasks(); };
    el('platformHistoryTask').onchange = () => { historySelected = el('platformHistoryTask').value; };
    el('platformHistoryUpload').onclick = async () => {
      try {
        if (!historySelected) throw new Error('请明确选择这条录制的任务');
        await post('upload-recording', { recording_id: historyTarget.identity, selection: { mode: 'platform',
          connection_id: historyTarget.connection_id, task_id: historySelected } });
        el('platformHistoryDialog').close(); await refreshRecords();
      } catch (error) { el('platformHistoryError').textContent = error.message; }
    };
    try {
      state = await call('status');
      renderSelection(); renderConnection();
      if (state.connection) { try { state = await post('verify'); renderConnection(); } catch (error) { message(error.message); } }
      await refreshTasks(); await refreshRecords();
    } catch (error) { message(error.message); }
    const visibleRefresh = () => { if (!document.hidden) { void refreshTasks(); void refreshRecords(); } };
    const timer = setInterval(visibleRefresh, 30000);
    const progressTimer = setInterval(() => { if (!document.hidden) void refreshRecords(); }, 5000);
    document.addEventListener('visibilitychange', visibleRefresh);
    window.addEventListener('pagehide', () => { clearInterval(timer); clearInterval(progressTimer); refreshGeneration++; }, { once: true });
  }
  const ready = init();
})();
