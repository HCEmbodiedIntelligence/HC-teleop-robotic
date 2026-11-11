const assert = require('node:assert/strict');
require('../middleware/core/static/receiver_metrics.js');
(async () => {
  let poll, render, cancelled = false;
  const requests = [];
  const video = {requestVideoFrameCallback(fn) { render = fn; return 1; }, cancelVideoFrameCallback() { cancelled = true; }};
  const tracker = globalThis.TeleopReceiverMetrics.start({
    pc: {async getStats() { return new Map([['v', {type:'inbound-rtp',kind:'video',id:'video',packetsReceived:95,
      packetsLost:5,fecPacketsReceived:3,fecPacketsDiscarded:1,framesDecoded:29,totalDecodeTime:.058,jitterBufferDelay:.29,jitterBufferEmittedCount:29}]]); }},
    video, sessionId:'s', url:'/metrics', now:()=>1000,
    fetch:async (url, opts)=>{requests.push(JSON.parse(opts.body));return {ok:true};},
    setTimeout:fn=>{poll=fn;return 2;}, clearTimeout(){}
  });
  render(10, {receiveTime:1,expectedDisplayTime:10,processingDuration:.002});
  render(30, {receiveTime:15,expectedDisplayTime:30,processingDuration:.003});
  await poll();
  assert.equal(requests[0].session_id, 's');
  assert.equal(requests[0].decode_time_ms, 58);
  assert.deepEqual(requests[0].render_gap_ms, [20]);
  assert.deepEqual(requests[0].receive_to_render_ms, [9,15]);
  assert.equal('frames_received' in requests[0], false);
  assert.equal(requests[0].sequence, 0);
  assert.equal(requests[0].fec_packets_received, 3);
  assert.equal(requests[0].fec_packets_discarded, 1);
  assert.equal(requests[0].frames_rendered, 2);
  await poll();
  assert.equal(requests[1].sequence, 1);
  assert.deepEqual(requests[1].render_gap_ms, []);
  tracker.stop();
  assert.equal(cancelled, true);
  console.log('receiver telemetry mock tests passed');
})().catch(error=>{console.error(error);process.exit(1);});
