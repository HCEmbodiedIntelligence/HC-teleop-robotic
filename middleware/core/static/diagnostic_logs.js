(function (global) {
  'use strict';
  const MAX_ENTRIES = 500;
  const LEVELS = {critical: 4, fatal: 4, error: 3, warning: 2, warn: 2, info: 1, debug: 0};
  const LABELS = {critical: '严重', fatal: '严重', error: '错误', warning: '警告', warn: '警告', info: '信息', debug: '调试'};
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
  function formatTime(value) {
    const timestamp = typeof value === 'number' ? value * (value < 1e12 ? 1000 : 1) : value;
    const date = new Date(timestamp);
    return Number.isNaN(date.getTime()) ? String(value || '时间未知') : date.toLocaleString('zh-CN', {hour12: false});
  }
  function clip(value, max = 12000) {
    let text;
    if (typeof value === 'string') text = value;
    else { try { text = JSON.stringify(value, null, 2); } catch (_) { text = String(value); } }
    return String(text == null ? '' : text).slice(0, max);
  }
  const SUBSYSTEMS = {middleware: '中间件', control: '控制进程', hardware: '硬件驱动 (HC_X1)', diagnostics: '日志系统'};
  const STATE_LABELS = {error: '近 5 分钟有错误', warning: '有警告或历史错误', ok: '正常'};
  function subsystemLabel(name) { return SUBSYSTEMS[name] || name || '未知'; }
  function recentErrors(summary) {
    return (Array.isArray(summary) ? summary : []).reduce((total, group) => total + (Number(group.recent_errors) || 0), 0);
  }
  function incidentExplanation(incident) {
    if (incident.relation === 'explicit') return '有明确事件关联；以下原因以日志证据为准。';
    return '底层原因尚未确定。以下是已记录的异常和上下文，不将时间接近当作因果。';
  }

  // A generation changes on every session/filter switch. Even a fetch mock or
  // browser that ignores AbortController cannot commit an old response.
  class LogModel {
    constructor() { this.generation = 0; this.reset(''); }
    reset(session) { this.generation += 1; this.session = session; this.cursor = 0; this.entries = []; this.incidents = []; this.summary = []; }
    accept(generation, session, response) {
      if (generation !== this.generation || session !== this.session || (response.session_id && response.session_id !== session)) return false;
      this.entries = mergeEntries(this.entries, Array.isArray(response.entries) ? response.entries : []);
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
    let timer = null, requestController = null, busy = false, disposed = false, sessionsLoaded = false, lastSessionsAt = 0, filterTimer = null;
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
    title.append(element('h2', '', '运行日志与故障溯源'), element('p', '', '查看已保存的日志，按事件链追踪停机前的异常。此处仅查看，不改变机器人状态。'));
    const download = element('a', 'diagnostic-log-download', '下载本会话日志');
    download.setAttribute('download', '');
    heading.append(title, download);
    const toolbar = element('div', 'diagnostic-log-toolbar');
    const session = element('select'); session.setAttribute('aria-label', '日志会话');
    const level = element('select'); level.setAttribute('aria-label', '日志等级');
    [['all', '全部等级'], ['error', '错误及严重'], ['warning', '警告及以上'], ['info', '信息及以上']].forEach(([value, name]) => {
      const option = element('option', '', name); option.value = value; level.append(option);
    });
    const subsystem = element('select'); subsystem.setAttribute('aria-label', '日志来源');
    const allSources = element('option', '', '全部来源'); allSources.value = ''; subsystem.append(allSources);
    const query = element('input'); query.type = 'search'; query.placeholder = '搜索原因、话题或来源'; query.maxLength = 200;
    const priority = element('input'); priority.type = 'checkbox'; priority.checked = true;
    const automatic = element('input'); automatic.type = 'checkbox'; automatic.checked = true;
    const refresh = element('button', '', '刷新'); refresh.type = 'button';
    toolbar.append(labelled('会话', session), labelled('来源', subsystem), labelled('等级', level), labelled('搜索', query), labelled('错误优先', priority), labelled('自动刷新', automatic), refresh);
    const status = element('div', 'diagnostic-log-status', '正在读取日志会话…'); status.setAttribute('role', 'status'); status.setAttribute('aria-live', 'polite');
    const error = element('div', 'diagnostic-log-error'); error.hidden = true; error.setAttribute('role', 'alert');
    const warning = element('div', 'diagnostic-log-warning'); warning.hidden = true;
    const alarm = element('div', 'diagnostic-log-alarm'); alarm.hidden = true; alarm.setAttribute('role', 'alert');
    const overview = element('div', 'diagnostic-log-overview'); overview.setAttribute('aria-label', '各子系统状态');
    const incidents = element('div', 'diagnostic-log-incidents'); incidents.setAttribute('aria-label', '故障链');
    const list = element('ol', 'diagnostic-log-list'); list.setAttribute('aria-label', '已保存的日志条目');
    const empty = element('p', 'diagnostic-log-empty', '当前会话没有匹配的日志。'); empty.hidden = true;
    const more = element('button', 'diagnostic-log-more', '加载后续记录'); more.type = 'button'; more.hidden = true;
    root.replaceChildren(heading, alarm, overview, toolbar, status, error, warning, incidents, list, empty, more);

    function visible() {
      return !doc.hidden && (!root.getClientRects || root.getClientRects().length > 0);
    }
    function updateDownload() {
      if (model.session) {
        download.href = '/api/logs/export?' + new URLSearchParams({session: model.session});
        download.removeAttribute('aria-disabled');
      } else { download.removeAttribute('href'); download.setAttribute('aria-disabled', 'true'); }
    }
    function makeEntry(entry) {
      const severity = levelOf(entry);
      const row = element('li', 'diagnostic-log-entry diagnostic-log-level-' + (Object.hasOwn(LEVELS, severity) ? severity : 'info'));
      row.dataset.logId = String(entry.id);
      const meta = element('div', 'diagnostic-log-meta');
      meta.append(element('span', 'diagnostic-log-level', LABELS[severity] || severity), element('time', '', formatTime(entry.time)), element('span', '', entry.source || '来源未知'));
      if (entry.kind) meta.append(element('span', '', entry.kind));
      row.append(meta, element('pre', 'diagnostic-log-message', clip(entry.message)));
      if (entry.file) row.append(element('div', 'diagnostic-log-file', clip(entry.file, 300) + (entry.line ? ':' + entry.line : '')));
      if (entry.details && Object.keys(entry.details).length) {
        const detail = element('details'); detail.append(element('summary', '', '查看诊断字段'), element('pre', 'diagnostic-log-details', clip(entry.details)));
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
    function renderOverview() {
      syncSubsystems();
      overview.replaceChildren(...model.summary.map(group => {
        const card = element('button', 'diagnostic-log-card diagnostic-log-card-' + (STATE_LABELS[group.state] ? group.state : 'ok'));
        card.type = 'button';
        card.setAttribute('aria-pressed', String(subsystem.value === group.subsystem));
        const head = element('div', 'diagnostic-log-card-head');
        head.append(element('strong', '', subsystemLabel(group.subsystem)), element('span', 'diagnostic-log-card-state', STATE_LABELS[group.state] || ''));
        const counts = element('div', 'diagnostic-log-card-counts');
        counts.append(element('span', 'diagnostic-log-count-error', '错误 ' + (Number(group.errors) || 0)),
                      element('span', 'diagnostic-log-count-warning', '警告 ' + (Number(group.warnings) || 0)),
                      element('span', '', '共 ' + (Number(group.total) || 0)));
        card.append(head, counts);
        if (group.latest_error) {
          card.append(element('p', 'diagnostic-log-card-latest', formatTime(group.latest_error.time) + ' · ' + (group.latest_error.source || '') + '\n' + clip(group.latest_error.message, 240)));
        }
        card.addEventListener('click', () => selectSubsystem(group.subsystem));
        return card;
      }));
      const failing = model.summary.filter(group => group.state === 'error');
      alarm.hidden = !failing.length;
      if (failing.length) {
        alarm.replaceChildren(element('strong', '', '最近 5 分钟有 ' + recentErrors(model.summary) + ' 条错误'));
        failing.forEach(group => {
          const line = element('button', 'diagnostic-log-alarm-line', subsystemLabel(group.subsystem) + ' · ' + (group.latest_error?.source || '') + '：' + clip(group.latest_error?.message || '', 300));
          line.type = 'button';
          line.addEventListener('click', () => { subsystem.value = group.subsystem; level.value = 'error'; resetView(); });
          alarm.append(line);
        });
      }
    }
    function render() {
      renderOverview();
      list.replaceChildren(...sortEntries(model.entries, priority.checked).map(makeEntry));
      empty.hidden = model.entries.length !== 0;
      incidents.replaceChildren();
      model.incidents.forEach(incident => {
        const card = element('details', 'diagnostic-log-incident');
        const summary = element('summary');
        summary.append(element('strong', '', incident.title || '停机或异常事件'), element('span', 'diagnostic-log-relation', incident.relation === 'explicit' ? '明确关联' : '原因待确认'));
        card.append(summary, element('p', 'diagnostic-log-explanation', incidentExplanation(incident)));
        if (incident.root_message) card.append(element('p', 'diagnostic-log-cause', (incident.relation === 'explicit' ? '已记录的直接原因：' : '已记录的原因 / 线索：') + clip(incident.root_message, 2000)));
        const chain = Array.isArray(incident.chain) ? incident.chain.slice(0, 100).sort((a, b) => Number(a.id) - Number(b.id)) : [];
        const first = chain.find(entry => (LEVELS[levelOf(entry)] || 0) >= 2);
        if (first) card.append(element('p', 'diagnostic-log-first', '最早记录的异常（不等同于底层根因）：' + formatTime(first.time) + ' · ' + clip(first.message, 1200)));
        if (chain.length) {
          const timeline = element('ol', 'diagnostic-log-chain');
          chain.forEach(entry => timeline.append(makeEntry(entry)));
          card.append(timeline);
        } else card.append(element('p', '', '尚未保存可核对的事件链，不能据此判断底层原因。'));
        incidents.append(card);
      });
    }
    function cancelRequest() { if (requestController) requestController.abort(); }
    function resetView() {
      cancelRequest(); model.reset(session.value); more.hidden = true; updateDownload(); render();
      if (!busy) void refreshLogs(true);
    }
    async function getJSON(url, signal) {
      const response = await fetcher(url, {signal, headers: {'Accept': 'application/json'}});
      if (!response.ok) throw new Error('日志服务返回 HTTP ' + response.status);
      return response.json();
    }
    async function refreshLogs(force = false) {
      if (disposed || busy || (!force && !visible())) return;
      busy = true; refresh.disabled = true; more.disabled = true;
      const controller = new AbortController(); requestController = controller;
      const deadline = setTimer(() => controller.abort(), 8000);
      const generation = model.generation;
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
        status.textContent = '已显示 ' + model.entries.length + ' 条（最多保留最近 ' + MAX_ENTRIES + ' 条） · ' + (priority.checked ? '错误优先' : '时间倒序') + ' · 更新于 ' + new Date().toLocaleTimeString('zh-CN', {hour12: false});
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
        if (!disposed && automatic.checked) timer = setTimer(() => { timer = null; void refreshLogs(); }, 2000);
      }
    }
    session.addEventListener('change', resetView);
    level.addEventListener('change', resetView);
    subsystem.addEventListener('change', resetView);
    priority.addEventListener('change', () => { render(); });
    query.addEventListener('input', () => { if (filterTimer) clearTimer(filterTimer); filterTimer = setTimer(resetView, 300); });
    automatic.addEventListener('change', () => { if (timer) clearTimer(timer); if (automatic.checked) void refreshLogs(); });
    refresh.addEventListener('click', () => { sessionsLoaded = false; void refreshLogs(true); });
    more.addEventListener('click', () => void refreshLogs(true));
    function onVisibility() { if (doc.hidden) { cancelRequest(); if (timer) clearTimer(timer); } else if (automatic.checked) void refreshLogs(); }
    doc.addEventListener('visibilitychange', onVisibility);
    void refreshLogs();
    return {
      refresh: () => refreshLogs(true), model,
      dispose() { disposed = true; cancelRequest(); if (timer) clearTimer(timer); if (filterTimer) clearTimer(filterTimer); doc.removeEventListener('visibilitychange', onVisibility); }
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
    async function poll() {
      try {
        if (!doc.hidden) {
          const response = await fetcher('/api/logs?limit=1&level=error&priority=0', {headers: {'Accept': 'application/json'}});
          if (response.ok) {
            const count = recentErrors((await response.json()).summary);
            badge.textContent = count > 99 ? '99+' : String(count);
            badge.hidden = count === 0;
            link.title = count ? '最近 5 分钟有 ' + count + ' 条错误' : '';
          }
        }
      } catch (_) { /* the connection indicator already reports middleware loss */ }
      timer = global.setTimeout(poll, options.interval || 5000);
    }
    void poll();
    return {badge, dispose() { if (timer) global.clearTimeout(timer); }};
  }
  const api = {mount, watchBadge, recentErrors, subsystemLabel, LogModel, sortEntries, mergeEntries, incidentExplanation, formatTime, clip};
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
