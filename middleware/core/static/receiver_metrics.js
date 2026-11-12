/* Receiver-reported WebRTC telemetry. Render callbacks are not physical display timestamps. */
(function (root) {
  'use strict';
  function start(options) {
    const {pc, video, sessionId, url} = options;
    const intervalMs = Math.max(250, Number(options.intervalMs) || 1000);
    const request = options.fetch || root.fetch.bind(root);
    const clock = options.now || (() => root.performance.now());
    const schedule = options.setTimeout || root.setTimeout.bind(root);
    const cancel = options.clearTimeout || root.clearTimeout.bind(root);
    if (!pc || !video || !sessionId || !url) throw new Error('pc, video, sessionId and url are required');
    let stopped = false, timer = null, callback = null, sequence = 0, lastRender = null, renderedFrames = 0;
    const arrays = {render_gap_ms: [], receive_to_render_ms: [], processing_ms: []};
    function append(name, value) {
      if (Number.isFinite(value) && value >= 0 && value <= 60000) {
        arrays[name].push(value);
        if (arrays[name].length > 240) arrays[name].shift();
      }
    }
    function rendered(now, metadata) {
      if (stopped) return;
      renderedFrames += 1;
      if (lastRender !== null) append('render_gap_ms', now-lastRender);
      lastRender = now;
      // receiveTime and expectedDisplayTime are browser monotonic timestamps;
      // captureTime is intentionally omitted because remote clock accuracy varies.
      if (Number.isFinite(metadata.receiveTime) && Number.isFinite(metadata.expectedDisplayTime)) {
        append('receive_to_render_ms', metadata.expectedDisplayTime-metadata.receiveTime);
      }
      if (Number.isFinite(metadata.processingDuration)) append('processing_ms', metadata.processingDuration*1000);
      callback = video.requestVideoFrameCallback(rendered);
    }
    if (typeof video.requestVideoFrameCallback === 'function') callback = video.requestVideoFrameCallback(rendered);
    async function poll() {
      if (stopped) return;
      try {
        const stats = await pc.getStats();
        if (stopped) return;
        let inbound = null;
        stats.forEach(stat => {
          if (stat.type === 'inbound-rtp' && (stat.kind === 'video' || stat.mediaType === 'video') && !stat.isRemote) {
            if (!inbound || (stat.bytesReceived || 0) > (inbound.bytesReceived || 0)) inbound = stat;
          }
        });
        if (inbound) {
          const payload = {session_id: sessionId, sequence: sequence++, sample_ms: clock(), stats_id: String(inbound.id)};
          if (typeof video.requestVideoFrameCallback === 'function') payload.frames_rendered = renderedFrames;
          const fields = {packets_received: ['packetsReceived', 1], packets_lost: ['packetsLost', 1],
            frames_received: ['framesReceived', 1], frames_decoded: ['framesDecoded', 1],
            frames_dropped: ['framesDropped', 1], freeze_count: ['freezeCount', 1],
            freeze_duration_ms: ['totalFreezesDuration', 1000], jitter_buffer_delay_ms: ['jitterBufferDelay', 1000],
            jitter_buffer_emitted_count: ['jitterBufferEmittedCount', 1], decode_time_ms: ['totalDecodeTime', 1000],
            fec_packets_received: ['fecPacketsReceived', 1], fec_packets_discarded: ['fecPacketsDiscarded', 1]};
          Object.entries(fields).forEach(([name, [source, scale]]) => {
            if (Number.isFinite(inbound[source])) payload[name] = inbound[source]*scale;
          });
          Object.keys(arrays).forEach(name => { payload[name] = arrays[name].splice(0); });
          const response = await request(url, {method: 'POST', headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(payload), signal: root.AbortSignal?.timeout?.(4000)});
          if (response && response.ok === false) throw new Error(`receiver metrics HTTP ${response.status}`);
        }
      } catch (error) {
        if (typeof options.onError === 'function') options.onError(error);
      } finally {
        if (!stopped) timer = schedule(poll, intervalMs);
      }
    }
    timer = schedule(poll, intervalMs);
    return {stop() {
      stopped = true;
      if (timer !== null) cancel(timer);
      if (callback !== null && typeof video.cancelVideoFrameCallback === 'function') video.cancelVideoFrameCallback(callback);
      Object.values(arrays).forEach(values => { values.length = 0; });
    }};
  }
  root.TeleopReceiverMetrics = {start};
})(typeof window !== 'undefined' ? window : globalThis);
