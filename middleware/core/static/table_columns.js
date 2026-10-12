/* Column widths are explicit so live topic rates never resize neighbouring cells. */
(function (root) {
  function clampWidth(value) {
    return Math.max(56, Math.min(1600, Math.round(Number(value) || 56)));
  }
  function restoreWidths(storage, key, defaults) {
    try {
      const saved = JSON.parse(storage.getItem(key));
      if (Array.isArray(saved) && saved.length === defaults.length && saved.every(value => Number.isFinite(value))) {
        return saved.map(clampWidth);
      }
    } catch (_) { /* Private browsing and malformed preferences use defaults. */ }
    return defaults.map(clampWidth);
  }
  function fitWidths(desired, minimums, available, flexible) {
    const widths = desired.slice();
    let deficit = widths.reduce((a, b) => a + b, 0) - available;
    while (deficit > 0) {
      const eligible = flexible.filter(i => widths[i] > minimums[i]);
      if (!eligible.length) break;
      const share = Math.ceil(deficit / eligible.length);
      for (const i of eligible) {
        const change = Math.min(share, widths[i] - minimums[i], deficit);
        widths[i] -= change; deficit -= change;
      }
    }
    // Fill the available card width, keeping numeric/action columns unchanged.
    const spare = available - widths.reduce((a, b) => a + b, 0);
    if (spare > 0 && flexible.length) {
      const weight = flexible.reduce((sum, i) => sum + desired[i], 0);
      let remaining = spare;
      flexible.forEach((i, index) => {
        const extra = index === flexible.length - 1 ? remaining : Math.floor(spare * desired[i] / weight);
        widths[i] += extra; remaining -= extra;
      });
    }
    return widths;
  }
  function install(table) {
    const cols = Array.from(table.querySelectorAll('colgroup col'));
    const headers = Array.from(table.querySelectorAll('thead th'));
    if (!cols.length || cols.length !== headers.length || table.dataset.columnsReady) return;
    table.dataset.columnsReady = 'true';
    const key = 'hc-topic-columns-v3:' + table.dataset.resizableColumns;
    const defaults = cols.map(col => parseFloat(col.style.width));
    let storage;
    try { storage = root.localStorage; } catch (_) {}
    let widths = restoreWidths(storage, key, defaults);
    let manual = false;
    try { manual = !!storage.getItem(key); } catch (_) {}
    const custom = table.dataset.resizableColumns === 'custom';
    const flexible = custom ? [1, 2, 3] : [1, 2, 3];
    const minimums = custom ? [64, 220, 180, 170, 100, 130, 70, 128] : [64, 120, 220, 170, 100, 150, 166];
    const maximums = custom ? [64, 700, 520, 360, 100, 130, 70, 128] : [64, 180, 700, 360, 100, 150, 166];
    let signature = '';
    let context;
    try { context = table.ownerDocument.createElement('canvas').getContext('2d'); } catch (_) {}
    function autofit(force = false) {
      if (manual && !force) return;
      const available = table.parentElement?.clientWidth || widths.reduce((a, b) => a + b, 0);
      if (!available) return;
      const rows = Array.from(table.querySelectorAll('tbody tr'));
      const content = flexible.map(i => rows.map(row => row.cells?.[i]?.textContent || ''));
      const next = JSON.stringify([available, content]);
      if (next === signature && !force) return;
      signature = next;
      const desired = minimums.slice();
      flexible.forEach((i, column) => {
        let widest = minimums[i];
        for (let r = 0; r < rows.length; r++) {
          const text = content[column][r];
          const cell = rows[r].cells?.[i];
          const style = cell && root.getComputedStyle?.(cell.querySelector('code') || cell);
          if (context) context.font = style ? `${style.fontWeight} ${style.fontSize} ${style.fontFamily}` : '12px sans-serif';
          const length = context ? context.measureText(text).width : Array.from(text).reduce((sum, char) => sum + (/[^\x00-\x7f]/.test(char) ? 12 : 7), 0);
          widest = Math.max(widest, Math.ceil(length + 28));
        }
        desired[i] = Math.min(maximums[i], widest);
      });
      widths = fitWidths(desired, minimums, available, flexible);
      apply();
    }
    function apply() {
      cols.forEach((col, index) => { col.style.width = widths[index] + 'px'; });
      table.style.width = widths.reduce((sum, width) => sum + width, 0) + 'px';
      table.style.minWidth = table.style.width;
      headers.forEach((header, index) => {
        const handle = header.querySelector('.column-resize-handle');
        if (handle) handle.setAttribute('aria-valuenow', widths[index]);
      });
    }
    function save() {
      try { storage.setItem(key, JSON.stringify(widths)); } catch (_) {}
    }
    headers.forEach((header, index) => {
      const handle = table.ownerDocument.createElement('span');
      handle.className = 'column-resize-handle';
      handle.tabIndex = 0;
      handle.title = '自动按内容适配；拖动微调；双击恢复自动列宽';
      handle.setAttribute('role', 'separator');
      handle.setAttribute('aria-orientation', 'vertical');
      handle.setAttribute('aria-label', `调整${header.textContent.trim() || '操作'}列宽`);
      handle.setAttribute('aria-valuemin', 56);
      handle.setAttribute('aria-valuemax', 1600);
      handle.addEventListener('pointerdown', event => {
        if (event.button !== 0) return;
        event.preventDefault();
        manual = true;
        const start = event.clientX, initial = widths[index];
        handle.setPointerCapture(event.pointerId);
        table.ownerDocument.body.classList.add('resizing-table-column');
        handle.classList.add('active');
        const move = current => { widths[index] = clampWidth(initial + current.clientX - start); apply(); };
        const end = () => {
          handle.removeEventListener('pointermove', move);
          handle.removeEventListener('pointerup', end);
          handle.removeEventListener('pointercancel', end);
          handle.removeEventListener('lostpointercapture', end);
          table.ownerDocument.body.classList.remove('resizing-table-column');
          handle.classList.remove('active');
          save();
        };
        handle.addEventListener('pointermove', move);
        handle.addEventListener('pointerup', end);
        handle.addEventListener('pointercancel', end);
        handle.addEventListener('lostpointercapture', end);
      });
      handle.addEventListener('dblclick', () => { manual = false; try { storage.removeItem(key); } catch (_) {} autofit(true); });
      handle.addEventListener('keydown', event => {
        if (!['ArrowLeft', 'ArrowRight', 'Home'].includes(event.key)) return;
        event.preventDefault();
        manual = true;
        widths[index] = event.key === 'Home' ? defaults[index] : clampWidth(widths[index] + (event.key === 'ArrowRight' ? 1 : -1) * (event.shiftKey ? 40 : 10));
        apply(); save();
      });
      header.append(handle);
    });
    apply();
    autofit();
    if (root.MutationObserver) {
      const observer = new root.MutationObserver(() => autofit());
      const body = table.querySelector('tbody');
      if (body) observer.observe(body, {childList: true, characterData: true, subtree: true});
    }
    if (root.ResizeObserver && table.parentElement) {
      new root.ResizeObserver(() => autofit()).observe(table.parentElement);
    }
  }
  if (typeof module !== 'undefined') module.exports = {clampWidth, restoreWidths, fitWidths, install};
  if (root.document) root.document.querySelectorAll('[data-resizable-columns]').forEach(install);
})(typeof window === 'undefined' ? globalThis : window);
