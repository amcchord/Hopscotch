// Invoked by check_wifi_load.py after its motors-off/disarmed preflight.
import fs from 'node:fs';
const [host, seconds, output, mode = 'overload', radio = 'require-radio'] = process.argv.slice(2);
const duration = Number(seconds) * 1000;
const readerCount = mode === 'normal' ? 1 : 3;
const sockets = [], counts = Array(readerCount).fill(0), gaps = Array(readerCount).fill(0);
const last = Array(readerCount).fill(0), errors = [], transportErrors = [];
let stopped = false, reconnects = 0;
async function telemetry() {
  const r = await fetch(host + '/api/telemetry', {signal: AbortSignal.timeout(5000)});
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}
const before = await telemetry();
const startedMs = Date.now();
let end = startedMs + duration;
let requests = 0, rcAgeMax = 0, lqMin = 100, imuAgeMax = 0, minHeap = Infinity;
function checkState(s) {
  if (s.drive_armed || s.arm_armed || s.arming || s.balance.active) {
    errors.push('Unexpected armed/active state; stopped load'); end = 0;
  }
  if (s.balance.fault) errors.push('IMU fault');
  if (radio !== 'ignore-radio' && !s.link_up) errors.push('RC link lost');
  rcAgeMax = Math.max(rcAgeMax, s.rc_age_ms);
  lqMin = Math.min(lqMin, s.lq);
  imuAgeMax = Math.max(imuAgeMax, s.balance.imu_age_us);
  minHeap = Math.min(minHeap, s.free_heap);
}
checkState(before);
function connect(i) {
  if (stopped) return;
  const ws = new WebSocket(host.replace('http', 'ws') + '/ws');
  ws.onmessage = e => {
    const s = JSON.parse(e.data); checkState(s);
    const now = Date.now();
    if (last[i]) gaps[i] = Math.max(gaps[i], now - last[i]);
    last[i] = now; counts[i]++;
  };
  ws.onerror = () => transportErrors.push(`WebSocket ${i} error`);
  ws.onclose = () => {
    if (!stopped) { reconnects++; setTimeout(() => connect(i), 1000); }
  };
  sockets[i] = ws;
}
for (let i = 0; i < readerCount; i++) connect(i);
await Promise.all(Array.from({length: mode === 'normal' ? 1 : 3}, (_, i) => (async () => {
  while (Date.now() < end) {
    try {
      const r = await fetch(host + (mode !== 'normal' && i === 0 ? '/' : '/api/telemetry'), {
        signal: AbortSignal.timeout(5000)
      });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      if (mode !== 'normal' && i === 0) await r.text(); else checkState(await r.json());
      requests++;
    } catch (e) { transportErrors.push(e.message); }
    await new Promise(r => setTimeout(r, mode === 'normal' ? 1000 : 100));
  }
})()));
stopped = true;
for (const s of sockets) s.close();
const after = await telemetry();
const result = {
  duration_s: duration / 1000, elapsed_s: (Date.now() - startedMs) / 1000,
  mode, nonreading_websocket_client: mode !== 'normal',
  rc_link_required: radio !== 'ignore-radio',
  websocket_frames: counts, max_frame_gap_ms: gaps, http_requests: requests,
  rc_age_max_ms: rcAgeMax, rc_lq_min: lqMin, imu_age_max_us: imuAgeMax,
  observed_min_heap: minHeap, errors, transport_errors: transportErrors,
  reconnects, before, after
};
fs.writeFileSync(output, JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify({...result, before: undefined, after: undefined, timing: after.timing}));
