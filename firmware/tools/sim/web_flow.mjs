// End-to-end test of the web UI's command flow (dev tool, Node 22+), against `web_sim.exe --fresh`:
// the calibration wizard step by step, then teleoperation with the emulated PAR6, all through the same
// WebSocket the page uses. Takes about 90 s of real time.
//   node tools/sim/web_flow.mjs [ws://127.0.0.1:8080/ws]
const url = process.argv[2] || 'ws://127.0.0.1:8080/ws';
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const ws = new WebSocket(url);
ws.binaryType = 'arraybuffer';
let st = null, frame = null, nextId = 1, fails = 0;
const waiting = new Map();
ws.onmessage = (ev) => {
  if (typeof ev.data !== 'string') {
    const v = new DataView(ev.data), I = [0, 1, 2, 3, 4, 5, 6];
    frame = { len: ev.data.byteLength, ver: v.getUint8(4), mode: v.getUint8(5), flags: v.getUint16(6, true), q: I.map((i) => v.getFloat32(16 + 4 * i, true)),
              cur: I.map((i) => v.getInt16(160 + 2 * i, true)), stage: v.getUint8(181), tf: v.getUint8(182), text: I.slice(0, 6).map((i) => v.getFloat32(236 + 4 * i, true)),
              grip: v.getFloat32(260, true), qF: I.map((i) => v.getFloat32(184 + 4 * i, true)) };
    return;
  }
  const m = JSON.parse(ev.data);
  if (m.t === 's') st = m.r;
  else if (m.t === 'r' && waiting.has(m.id)) { waiting.get(m.id)(m.r); waiting.delete(m.id); }
};
await new Promise((r, j) => { ws.onopen = r; ws.onerror = () => j(new Error('cannot connect to ' + url)); });
const send = (o) => new Promise((r) => { const id = nextId++; waiting.set(id, r); ws.send(JSON.stringify({ ...o, id })); });
const check = (ok, what) => { console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${what}`); if (!ok) fails++; };
const deg = (r) => r * 180 / Math.PI;
async function idle(maxS) {           // wait until no calibration is running
  const t0 = Date.now();
  await sleep(400);
  while (Date.now() - t0 < maxS * 1000) { if (st && st.calib === 'idle') return true; await sleep(200); }
  return false;
}
await send({ cmd: 'ws_rate', frame_hz: 20, status_hz: 5 });
await sleep(600);
console.log('start:', st.calibrated ? 'calibrated' : 'not calibrated', '| mode', st.mode);

console.log('=== zero and directions ===');
check((await send({ cmd: 'mode', mode: 'gravity' })).ok === false, 'gravity is refused before calibration');
await send({ cmd: 'mode', mode: 'limp' });
check((await send({ cmd: 'calib_zero' })).ok, 'calib_zero accepted');
await idle(5);
check(st.cal_flags.every((f) => f & 1) && st.last_calib_ok, 'zero stored: ' + st.last_calib);
check(st.calibrated && (await send({ cmd: 'calib_dir', joint: 1 })).ok === false, 'calibrated after the zero; the directions are fixed');
await sleep(500);
check(st.q_deg.every((v) => Math.abs(v) < 1.0), 'the arm now reads the zero pose: ' + st.q_deg.map((v) => v.toFixed(1)).join(' '));

console.log('=== spring, save, torque ===');
check((await send({ cmd: 'spring', type: 'cable', k: 824, a_mm: 13, b_mm: 20 })).ok && st && true, 'spring Z-078I set');
await sleep(300);
check(st.unsaved === true, 'status shows unsaved changes');
check((await send({ cmd: 'save' })).ok, 'saved');
await sleep(500);
check(st.unsaved === false, 'unsaved flag cleared by save');
check((await send({ cmd: 'mode', mode: 'move' })).ok, 'Hold (move mode) accepted');
await sleep(2000);
check(frame.mode === 2 && (frame.flags & 2) && st.q_deg.slice(0, 6).every((v) => Math.abs(v) < 2), 'the arm holds the zero pose with torque on');

console.log('=== torque constants and spring ===');
for (const j of [3, 2]) {
  check((await send({ cmd: 'calib_torque', joint: j })).ok, `calib_torque J${j} accepted`);
  await idle(20);
  check(st.last_calib_ok && (st.cal_flags[j - 1] & 4), st.last_calib);
}
console.log('  measured Kt:', st.kt.map((v) => v.toFixed(3)).join(' '), '(truth J2 0.397, J3 0.485)');
check(Math.abs(st.kt[1] / 0.3972 - 1) < 0.03 && Math.abs(st.kt[2] / 0.4854 - 1) < 0.03, 'Kt J2/J3 within 3 % of the simulated truth');
await send({ cmd: 'move', q_deg: [0, 45, 45, 0, 0, 0, 0], speed_deg: 20 });
await sleep(4000);
check(Math.abs(st.q_deg[1] - 45) < 2 && Math.abs(st.q_deg[2] - 45) < 2, 'moved to 45 / 45: ' + st.q_deg[1].toFixed(1) + ' / ' + st.q_deg[2].toFixed(1));
check((await send({ cmd: 'calib_spring' })).ok, 'calib_spring accepted');
await idle(20);
check(st.last_calib_ok && Math.abs(st.spring.scale / 1.1 - 1) < 0.05, 'spring scale ' + st.spring.scale.toFixed(3) + ' (truth 1.100): ' + st.last_calib);
check((await send({ cmd: 'save' })).ok, 'saved the calibration');

console.log('=== gravity mode ===');
await send({ cmd: 'mode', mode: 'gravity' });
const q0 = st.q_deg.slice();
await sleep(3000);
const drift = Math.max(...st.q_deg.slice(0, 6).map((v, i) => Math.abs(v - q0[i])));
check(st.mode === 'gravity' && drift < 1.0, 'gravity mode floats the arm, drift ' + drift.toFixed(2) + ' deg in 3 s');

console.log('=== teleoperation with the emulated PAR6 ===');
check((await send({ cmd: 'teleop_start', host: 'par6.sim' })).ok, 'teleop_start accepted');
let coupled = false;
for (let k = 0; k < 75 && !coupled; k++) { await sleep(200); coupled = st.teleop === 'COUPLED'; }
check(coupled, 'paired, aligned, claimed and coupled (stage ' + st.teleop + ', link ' + st.link + ')');
check(Array.isArray(st.follower_deg), 'status carries the follower pose');
if (Array.isArray(st.follower_deg)) {
  const err = Math.max(...st.follower_deg.slice(0, 6).map((v, i) => Math.abs(v - st.q_deg[i])));
  check(err < 3, 'leader and follower agree within ' + err.toFixed(2) + ' deg');
}
check(frame.len === 288 && frame.ver === 2 && frame.stage === 4 && (frame.flags & 256) && frame.grip >= 0, 'version 2 frame while coupled: stage COUPLED, follower fresh, gripper reported (' + frame.grip.toFixed(0) + ' raw)');
const stageIs = async (s, maxS) => { for (let k = 0; k < maxS * 10; k++) { if (st.teleop === s) return true; await sleep(100); } return st.teleop === s; };

console.log('=== teleoperation controls ===');
const sv = await send({ cmd: 'save' });
check(sv.ok === false && /teleoperation/.test(sv.error), 'save is refused while teleoperating: ' + sv.error);
await send({ cmd: 'teleop_hold', on: true });
check(await stageIs('HELD', 2) && st.hold && !st.engaged, 'hold on: HELD (PAR6 holds, the leader is free)');
await send({ cmd: 'teleop_hold', on: false });
check(await stageIs('COUPLED', 2), 'hold off: COUPLED again');
await send({ cmd: 'sim_button', name: 'clutch', down: true });
check(await stageIs('HELD', 2) && st.buttons.clutch_down, 'the clutch button holds PAR6');
await send({ cmd: 'sim_button', name: 'clutch', down: false });
check(await stageIs('COUPLED', 2), 'releasing the clutch re-engages');
check((await send({ cmd: 'teleop_precision', value: 0.5 })).ok && (await sleep(700), Math.abs(st.precision - 0.5) < 1e-6), 'precision 0.5');
check((await send({ cmd: 'teleop_lock', joints: [1, 4] })).ok && (await sleep(700), st.locks === 9), 'locks J1 and J4 (mask 9)');
await send({ cmd: 'teleop_lock', joints: [] });
await send({ cmd: 'teleop_precision', value: 1 });
check((await send({ cmd: 'orient_lock', on: true })).ok && (await send({ cmd: 'guide', type: 'plane_h' })).ok, 'orientation lock and horizontal-plane guide accepted');
await sleep(700);
check(st.orient_lock === true && st.guide === 'plane_h', 'status shows them');
await send({ cmd: 'orient_lock', on: false });
await send({ cmd: 'guide', type: 'none' });
// pull the leader forward until PAR6 meets the emulated table (its J2 stops at 60 deg): t_ext rises
await send({ cmd: 'sim_hand', mode: 'pull', p: [0.30, 0.0, 0.02] });
let extMax = 0, j2Max = 0;
for (let k = 0; k < 30; k++) { await sleep(100); extMax = Math.max(extMax, frame.text[1]); j2Max = Math.max(j2Max, deg(frame.q[1])); }
console.log('  pulled forward: leader J2 up to ' + j2Max.toFixed(1) + ' deg, PAR6 t_ext J2 up to ' + extMax.toFixed(2) + ' Nm');
check(j2Max < 60 || (extMax > 0.2 && (frame.tf & 2)), "PAR6's wall shows as t_ext in the frame once the leader passes 60 deg");
await send({ cmd: 'sim_hand', mode: 'free' });
await sleep(1500);
check((await send({ cmd: 'teleop_realign' })).ok, 'realign accepted');
check(await stageIs('REALIGNING', 2), 'REALIGNING: PAR6 holds while the leader moves back onto it');
check(await stageIs('COUPLED', 15) && st.offset_deg < 2.5, 'realigned and coupled, offset ' + st.offset_deg.toFixed(1) + ' deg');
// replay while coupled: PAR6 follows the leader's replay
await send({ cmd: 'replay_clear' });
const q0r = st.q_deg.slice(0, 7);
const samples = [];
for (let k = 0; k <= 50; k++) for (let i = 0; i < 7; i++) samples.push(Math.round(100 * (q0r[i] + (i === 0 ? 15 * k / 50 : 0))));
let okAdd = true, why = '';
for (let s = 0; s <= 50; s += 20) {
  const r = await send({ cmd: 'replay_add', ...(s ? {} : { dt_ms: 20 }), q_cdeg: samples.slice(7 * s, 7 * Math.min(51, s + 20)) });
  if (!r.ok) { okAdd = false; why = r.error; }
}
const rst = okAdd ? await send({ cmd: 'replay_start', speed: 1 }) : { ok: false, error: why };
check(rst.ok, 'replay loaded (51 samples) and started while coupled' + (rst.ok ? '' : ': ' + rst.error + ' (stage ' + st.teleop + ')'));
for (let k = 0; k < 80; k++) { await sleep(100); if (k > 5 && st.replay.state === 'idle') break; }
await sleep(800);
const fJ1 = Array.isArray(st.follower_deg) ? st.follower_deg[0] : NaN;
check(st.replay.note === 'done' && st.teleop === 'COUPLED' && Math.abs(fJ1 - (q0r[0] + 15)) < 3,
  'the replay ran and PAR6 followed it (follower J1 ' + fJ1.toFixed(1) + ' deg, target ' + (q0r[0] + 15).toFixed(1) + ')');
check((await send({ cmd: 'teleop_stop' })).ok, 'teleop_stop');
await sleep(500);
check(st.teleop === 'OFF' && st.mode === 'gravity', 'back to gravity mode, link ' + st.link);
await send({ cmd: 'mode', mode: 'limp' });
ws.close();
console.log(fails ? 'SOME CHECKS FAILED' : 'ALL CHECKS PASSED');
process.exit(fails ? 1 : 0);
