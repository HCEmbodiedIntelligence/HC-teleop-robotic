/* Independent upload panel. All server text is rendered via textContent. */
window.mountUploadPanel = function (element) {
  element.classList.add('upload-panel');
  element.innerHTML = `<h2>上传</h2><p data-status></p><p data-binding-state></p>
    <form data-settings>
    <label class="upload-toggle"><input name="enabled" type="checkbox">启用上传</label>
    <label class="upload-toggle"><input name="auto_enqueue" type="checkbox">录制完成后自动入队上传</label>
    <button>保存上传设置</button><p>设置立即生效；不重启 ROS 或遥操作。修改前请停止当前录制。关闭自动入队仍可手动上传。</p></form>
    <form data-binding><h3>新任务默认绑定</h3>
    <label>平台地址<input name="platform_base_url" type="url" required></label>
    <label>平台机器人 ID<input name="robot_id" placeholder="平台签发的机器人 ID，非名称" required></label>
    <label>本地机器人 ID<input name="local_robot_id" required></label>
    <label>平台任务 ID<input name="task_id" placeholder="采集任务完整 ID，非任务名称" required></label>
    <label>凭据引用（env: 或 file:，不填密码）<input name="credential_ref" placeholder="file:/绝对路径/robot-ingest.token" pattern="(?:env:|file:).+" required></label>
    <button data-save-binding>保存默认绑定</button><button type="button" data-check-connection>测试连接</button><p data-connection></p><p>凭据文件路径必须带 file: 前缀；机器人与任务 ID 请从平台复制。连接测试只验证身份与接口，不验证任务权限。</p><p>已入队作业保持原机器人与任务归属。</p></form>
    <form data-enqueue><label>产物 ID<input name="artifact_id" required></label><button>手动入队</button></form>
    <button data-refresh>刷新</button><p data-error role="alert"></p>
    <div class="upload-table"><table><thead><tr><th>产物 / 任务</th><th>采集</th><th>产物</th><th>上传</th><th>进度</th><th>平台处理</th><th>质检</th><th>操作</th></tr></thead><tbody></tbody></table></div>`;
  let disposed = false;
  let refreshing = false;
  let settingsLoaded = false;
  let bindingEdited = false;
  let refreshAgain = false;
  const controller = new AbortController();
  const error = element.querySelector('[data-error]');
  async function call(operation, body) {
    const response = await fetch('/api/uploads' + (operation ? '/' + operation : ''),
      body === undefined ? {signal: controller.signal} : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body), signal: controller.signal});
    const result = await response.json();
    if (!response.ok) throw new Error([result.code, result.action || result.error].filter(Boolean).join('：') || `请求失败 (${response.status})`);
    return result;
  }
  async function run(operation, body) {
    try { error.textContent = ''; await call(operation, body); await refresh(); }
    catch (e) { if (!disposed) error.textContent = e.message; }
  }
  async function refresh() {
    if (disposed) return;
    if (refreshing) { refreshAgain = true; return; }
    refreshing = true;
    try {
      const value = await call('');
      if (disposed) return;
      error.textContent = '';
      element.querySelector('[data-status]').textContent = !value.enabled ? '上传未启用，请勾选下方“启用上传”并保存；本地录制可继续使用' :
        `${value.running ? '上传服务运行中' : '上传服务未运行'}；自动入队${value.auto_enqueue ? '开启' : '关闭'}`;
      const settings = element.querySelector('[data-settings]');
      if (!settingsLoaded) {
        settings.elements.enabled.checked = !!value.enabled;
        settings.elements.auto_enqueue.checked = !!value.auto_enqueue;
        settingsLoaded = true;
      }
      const saveBinding = element.querySelector('[data-save-binding]');
      saveBinding.disabled = !value.enabled;
      saveBinding.title = value.enabled ? '' : '请先勾选启用上传并保存上传设置';
      element.querySelector('[data-enqueue] button').disabled = !value.enabled;
      const bindingForm = element.querySelector('[data-binding]');
      if (!bindingEdited && value.binding_values) {
        for (const [name, fieldValue] of Object.entries(value.binding_values)) {
          const field = bindingForm.elements.namedItem(name);
          if (field) field.value = fieldValue;
        }
      }
      const binding = value.default_binding;
      element.querySelector('[data-binding-state]').textContent = binding
        ? `当前默认绑定：${binding.platform_base_url} / 机器人 ${binding.robot_id} / 任务 ${binding.collection_task_id}`
        : '尚未绑定平台机器人与任务；历史产物需要手动选择入队。';
      const tbody = element.querySelector('tbody'); tbody.replaceChildren();
      if (!(value.jobs || []).length) {
        const row = document.createElement('tr');
        const td = document.createElement('td'); td.colSpan = 8;
        td.textContent = '暂无上传作业。可在数据集管理中选择已封存产物上传。';
        row.append(td); tbody.append(row);
      }
      for (const job of (value.jobs || [])) {
        const row = document.createElement('tr');
        const progress = job.upload_state === 'submitted' ? 100 :
          (job.total_bytes > 0 ? Math.max(0, Math.min(100, Math.floor(100 * (job.uploaded_bytes || 0) / job.total_bytes))) : 0);
        for (const value of [ `${job.artifact_id || job.export_id} / ${job.task_id}`, job.capture_state, job.artifact_state, job.upload_state,
                              `${progress}%`, job.processing_state, job.quality_state ]) {
          const td = document.createElement('td'); td.textContent = value; row.append(td);
        }
        const td = document.createElement('td');
        for (const [label, operation, body] of [ ['重试上传/查询', 'retry', {job_id: job.id}],
          ['恢复暂停', 'retry', {job_id: job.id, resume: true}],
          ['重试处理', 'retry', {job_id: job.id, processing: true}],
          ['取消本地上传', 'cancel', {artifact_id: job.artifact_id}] ]) {
          const button = document.createElement('button'); button.textContent = label;
          button.onclick = () => run(operation, body); td.append(button);
        }
        const detail = document.createElement('p'); detail.textContent = [job.code, job.next_action].filter(Boolean).join('：');
        td.append(detail); row.append(td); tbody.append(row);
      }
      if (value.error || Object.keys(value.scan_errors || {}).length) error.textContent = JSON.stringify({service: value.error, scan: value.scan_errors});
    } catch (e) { if (!disposed) error.textContent = e.message; }
    finally {
      refreshing = false;
      if (refreshAgain && !disposed) { refreshAgain = false; refresh(); }
    }
  }
  for (const [selector, operation] of [['data-binding', 'binding'], ['data-enqueue', 'enqueue']]) {
    const form = element.querySelector(`[${selector}]`);
    form.onsubmit = event => {event.preventDefault(); run(operation, Object.fromEntries(new FormData(form)));};
  }
  element.querySelector('[data-binding]').addEventListener('input', () => { bindingEdited = true; });
  const settingsForm = element.querySelector('[data-settings]');
  settingsForm.onsubmit = async event => {
    event.preventDefault();
    const button = settingsForm.querySelector('button'); button.disabled = true;
    try {
      error.textContent = '';
      await call('settings', {enabled: settingsForm.elements.enabled.checked,
                             auto_enqueue: settingsForm.elements.auto_enqueue.checked});
      settingsLoaded = false;
      await refresh();
    } catch (e) { if (!disposed) error.textContent = e.message; }
    finally { button.disabled = false; }
  };
  element.querySelector('[data-check-connection]').onclick = async event => {
    const button = event.currentTarget;
    const output = element.querySelector('[data-connection]');
    const form = element.querySelector('[data-binding]');
    if (!form.reportValidity()) return;
    button.disabled = true; output.textContent = '正在验证连接与机器人身份…';
    try {
      await call('check-connection', Object.fromEntries(new FormData(form)));
      output.textContent = '连接成功，机器人身份已验证；任务权限及数据处理需实际上传验证。';
    } catch (e) { if (!disposed) output.textContent = e.message; }
    finally { button.disabled = false; }
  };
  element.querySelector('[data-refresh]').onclick = refresh;
  refresh(); const timer = setInterval(refresh, 5000);
  return () => {disposed = true; clearInterval(timer); controller.abort();};
};
