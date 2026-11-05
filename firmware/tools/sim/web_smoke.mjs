// Web UI smoke test (dev tool, Node 22+): drives headless Chrome or Edge against a running web_sim.exe (or a
// real arm) through the DevTools protocol. It loads the page, checks that it connects, receives frames, the
// status (only when it changes) and the binary health message, that the page's own forward kinematics matches
// the firmware's TCP, the layout (menu, 3D view, bottom panel, inspector, status bar), every page, the search
// box, that no script error occurred, and saves screenshots on desktop and phone sizes, light and dark.
//   node tools/sim/web_smoke.mjs [url=http://127.0.0.1:8080/] [outdir=tools/sim/out/shots]
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';

const url = process.argv[2] || 'http://127.0.0.1:8080/';
const outDir = process.argv[3] || path.join(path.dirname(new URL(import.meta.url).pathname.replace(/^\/(\w:)/, '$1')), 'out', 'shots');
fs.mkdirSync(outDir, { recursive: true });
const browsers = [
  'C:/Program Files/Google/Chrome/Application/chrome.exe',
  'C:/Program Files (x86)/Google/Chrome/Application/chrome.exe',
  'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',
  'C:/Program Files/Microsoft/Edge/Application/msedge.exe',
];
const exe = browsers.find((p) => fs.existsSync(p));
if (!exe) { console.log('no Chrome or Edge found'); process.exit(2); }
const port = 9300 + Math.floor(Math.random() * 500);
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'raft-smoke-'));
const browser = spawn(exe, ['--headless=new', `--remote-debugging-port=${port}`, `--user-data-dir=${profile}`, '--no-first-run',
  '--no-default-browser-check', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--window-size=1440,900', 'about:blank'], { stdio: 'ignore' });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

let target = null;
for (let i = 0; i < 50 && !target; i++) {
  await sleep(200);
  try { target = (await (await fetch(`http://127.0.0.1:${port}/json/list`)).json()).find((t) => t.type === 'page'); } catch (e) { /* not up yet */ }
}
if (!target) { console.log('browser did not start'); browser.kill(); process.exit(2); }
const ws = new WebSocket(target.webSocketDebuggerUrl);
await new Promise((r) => { ws.onopen = r; });
let nextId = 1;
const waiting = new Map();
const errors = [];
ws.onmessage = (ev) => {
  const m = JSON.parse(ev.data);
  if (m.id && waiting.has(m.id)) { waiting.get(m.id)(m); waiting.delete(m.id); return; }
  if (m.method === 'Runtime.exceptionThrown') errors.push(m.params.exceptionDetails.exception?.description || m.params.exceptionDetails.text);
  if (m.method === 'Runtime.consoleAPICalled' && m.params.type === 'error') errors.push(m.params.args.map((a) => a.value || a.description).join(' '));
};
const cdp = (method, params = {}) => new Promise((r) => { const id = nextId++; waiting.set(id, r); ws.send(JSON.stringify({ id, method, params })); });
const evaluate = async (expr) => (await cdp('Runtime.evaluate', { expression: expr, returnByValue: true, awaitPromise: true })).result?.result?.value;
const json = async (expr) => JSON.parse(await evaluate(`JSON.stringify(${expr})`));
const shot = async (name) => {
  const r = await cdp('Page.captureScreenshot', { format: 'png' });
  fs.writeFileSync(path.join(outDir, name + '.png'), Buffer.from(r.result.data, 'base64'));
};
const key = async (k, mods = 0) => {
  const code = k === 'Escape' ? 'Escape' : k === 'Enter' ? 'Enter' : 'Key' + k.toUpperCase();
  const vk = k === 'Escape' ? 27 : k === 'Enter' ? 13 : k.toUpperCase().charCodeAt(0);
  await cdp('Input.dispatchKeyEvent', { type: 'keyDown', key: k, code, windowsVirtualKeyCode: vk, modifiers: mods });
  await cdp('Input.dispatchKeyEvent', { type: 'keyUp', key: k, code, windowsVirtualKeyCode: vk, modifiers: mods });
};
const nav = (p) => evaluate(`document.querySelector('#nav [data-p=${p}]').click()`);

let fails = 0;
const check = (ok, what) => { console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${what}`); if (!ok) fails++; };
await cdp('Runtime.enable');
await cdp('Page.enable');
await cdp('Emulation.setDeviceMetricsOverride', { width: 1440, height: 900, deviceScaleFactor: 1, mobile: false });
await cdp('Page.navigate', { url });
await sleep(3500);
await nav('drive');
await sleep(300);
const st = await json(`{conn: __raft.S.conn, hello: !!__raft.S.hello, sim: __raft.S.hello && __raft.S.hello.sim, frames: __raft.S.hist.length, fps: __raft.S.fps,
  status: !!__raft.S.st, health: __raft.S.hm && __raft.S.hm.loop.seq, hz: __raft.S.hm && __raft.S.hm.loop.hz, modelErr: __raft.S.modelErr, rtt: __raft.S.rtt, rate: __raft.S.rateSent}`);
console.log('page state:', JSON.stringify(st));
check(st.conn === 'live', 'connected over the WebSocket');
check(st.hello, 'received the hello message (model, limits)');
check(st.frames > 30 && st.fps > 10, 'receives leader frames (' + st.frames + ' so far, ' + (st.fps || 0).toFixed(0) + '/s)');
check(st.status, 'receives the status');
check(st.health > 0 && st.hz > 150, 'receives the binary health message (window ' + st.health + ', loop ' + (st.hz || 0).toFixed(1) + ' Hz)');
check(/"status":"change"/.test(st.rate || '') && /"health":true/.test(st.rate || ''), 'asks for the status on change and for health (' + st.rate + ')');
const meshes = await json(`{ ready: __raft.G.ready, tris: __raft.G.tris, failed: __raft.G.failed }`);
check(meshes.ready && meshes.tris > 5000, 'the real arm model is drawn with WebGL (' + meshes.tris + ' triangles' + (meshes.failed ? ', ' + meshes.failed : '') + ')');
check(st.modelErr != null && st.modelErr < 0.01, "page FK matches the firmware's TCP (" + (st.modelErr == null ? '-' : st.modelErr.toFixed(4)) + ' mm)');
const reply = await evaluate(`__raft.send({cmd: 'status'}).then((r) => JSON.stringify(r))`);
check(reply && JSON.parse(reply).ok === true, 'a command round trip through the page');
// the status comes only when it changes: count how often the page's status object is replaced while nothing happens
const quiet = await evaluate(`new Promise((res) => { let n = 0, last = __raft.S.st; const t = setInterval(() => { if (__raft.S.st !== last) { n++; last = __raft.S.st; } }, 50);
  setTimeout(() => { clearInterval(t); res(n); }, 3000); })`);
const changed = await evaluate(`new Promise((res) => { const last = __raft.S.st, t0 = performance.now(); __raft.send({cmd: 'hold_stiffness', value: 1.5});
  const t = setInterval(() => { if (__raft.S.st !== last) { clearInterval(t); res(Math.round(performance.now() - t0)); } }, 10); setTimeout(() => { clearInterval(t); res(-1); }, 2000); })`);
await evaluate(`__raft.send({cmd: 'hold_stiffness', value: 1})`);
check(quiet <= 1 && changed >= 0 && changed < 400, 'status on change: ' + quiet + ' in 3 quiet s, a setting change shows ' + changed + ' ms later');
const named = await evaluate(`__raft.send({cmd: 'set_name', name: 'Bench-A'}).then((r) => new Promise((res) => setTimeout(() => res(JSON.stringify({ r,
  sub: document.querySelector('#brandSub').textContent, title: document.title, field: document.querySelector('#armName').value,
  host: document.querySelector('#armHost').textContent })), 400)))`);
const bad = await evaluate(`__raft.send({cmd: 'set_name', name: 'no spaces'}).then((r) => JSON.stringify(r))`);
await evaluate(`__raft.send({cmd: 'set_name', name: ''})`);
await sleep(400);
const unnamed = await json(`({ sub: document.querySelector('#brandSub').textContent, title: document.title })`);
const nm = JSON.parse(named), bd = JSON.parse(bad);
check(nm.r.ok && nm.r.name === 'bench-a' && nm.r.host === 'raft-bench-a.local' && nm.sub === 'bench-a' && nm.title === 'RAFT bench-a' && nm.field === 'bench-a'
      && /raft-bench-a\.local/.test(nm.host) && !bd.ok && unnamed.sub === 'leader arm' && unnamed.title === 'RAFT leader',
      'arm name: header, tab title and the This arm card follow set_name; a space is refused; empty removes it (' + JSON.stringify({ sub: nm.sub, title: nm.title, host: nm.host }) + ')');
const layout = await json(`(() => { const r = (s) => { const e = document.querySelector(s); if (!e) return null; const b = e.getBoundingClientRect(); return [Math.round(b.left), Math.round(b.top), Math.round(b.width), Math.round(b.height)]; };
  const navB = [...document.querySelectorAll('#nav [data-p]')], nr = document.querySelector('#nav').getBoundingClientRect();
  return { nav: r('#nav'), view: r('#cv'), dock: r('#dock'), insp: r('#insp'), sbar: r('#sbar'), header: r('header.top'), navN: navB.length,
    navOut: navB.filter((b) => b.getBoundingClientRect().bottom > nr.bottom + 1).length, scroll: document.documentElement.scrollHeight, H: innerHeight,
    title: document.querySelector('#iTitle').textContent }; })()`);
console.log('layout:', JSON.stringify(layout));
check(layout.navN === 8 && !layout.navOut && layout.view[2] > 600 && layout.view[3] > 350 && layout.dock[3] > 200 && layout.insp[2] >= 380 &&
  layout.sbar[1] + layout.sbar[3] <= layout.H + 1 && layout.scroll <= layout.H + 1 && layout.title === 'Drive',
  'desktop layout: 8 menu entries, 3D view, bottom panel, inspector, status bar, one screen without page scroll');
const sbar = await json(`{ hz: document.querySelector('#sbHz').textContent, jit: document.querySelector('#sbJit').textContent, bus: document.querySelector('#sbBus').textContent,
  net: document.querySelector('#sbNet').textContent }`);
check(parseFloat(sbar.hz) > 150 && sbar.jit !== '-' && sbar.bus !== '-' && sbar.net !== '-', 'status bar: loop, jitter, bus, network (' + JSON.stringify(sbar) + ')');
const pose = await json(`(() => { const g = (id) => document.querySelector(id).textContent;
  const deg = { x: g('#pp0'), r: g('#pp3'), j2: g('#jA1'), grip: g('#jA6'), u: g('#pose .aU') };
  document.querySelector('#units [data-u=rad]').click();
  const rad = { j2: g('#jA1'), u: g('#pose .aU'), th: g('#jAh') };
  document.querySelector('#units [data-u=deg]').click();
  return { deg, rad, q2: __raft.S.f.q[1], chips: document.querySelectorAll('#chips .chip:not([hidden])').length }; })()`);
check(/^-?\d+\.\d$/.test(pose.deg.x) && /^-?\d+\.\d$/.test(pose.deg.j2) && /^-?\d+\.\d$/.test(pose.deg.grip) && pose.deg.u === 'deg' && pose.rad.u === 'rad' &&
  pose.rad.th === 'rad' && Math.abs(parseFloat(pose.rad.j2) - pose.q2) < 0.002 && pose.chips === 0, 'tool point readout and Joints panel, deg / rad switch, no state chips while idle (' + JSON.stringify(pose) + ')');
const jt = await json(`{ v2: !!__raft.S.f.cur && __raft.S.f.cur.length === 7 && __raft.S.f.volt.length === 7, rows: document.querySelectorAll('#jtab tbody tr').length,
  cmd: document.querySelector('#jC1').textContent, meas: document.querySelector('#jI1').textContent, model: document.querySelector('#jM1').textContent,
  volt: document.querySelector('#jV1').textContent, miss: document.querySelector('#jX1').textContent }`);
check(jt.v2 && jt.rows === 7 && /^-?\d+$/.test(jt.meas) && /^(-?\d+|pos)$/.test(jt.cmd) && /^-?\d+$/.test(jt.model) && /^\d+\.\d$/.test(jt.volt) && jt.miss !== '-',
  'Joints panel from version 2 frames: commanded, measured and model current, voltage, missed reads (' + JSON.stringify(jt) + ')');
await shot('desktop-drive');
// the bottom panel's other tabs
await evaluate(`document.querySelector('.dtabs [data-d=plot]').click()`);
await sleep(600);
const plot = await json(`{ w: document.querySelector('#chart').width, leg: document.querySelectorAll('#chLegend span').length, sel: !document.querySelector('#chSel').hidden }`);
check(plot.w > 300 && plot.leg === 7 && plot.sel, 'Live plot tab: chart, legend and its selector (' + JSON.stringify(plot) + ')');
await shot('desktop-dock-plot');
await evaluate(`document.querySelector('.dtabs [data-d=con]').click()`);
await sleep(300);
const con = await json(`{ lines: document.querySelectorAll('#log div').length, input: !!document.querySelector('#conIn').offsetWidth }`);
check(con.lines >= 2 && con.input, 'Console tab: the log and its input (' + JSON.stringify(con) + ')');
await evaluate(`document.querySelector('.dtabs [data-d=joints]').click()`);

for (const p of ['tele', 'cal', 'servos', 'set', 'plots', 'health']) {
  await nav(p);
  await sleep(500);
  await shot('desktop-' + p);
  if (p === 'plots') {
    await sleep(1200);
    const slow = await json(`{ fps: __raft.S.fps, rate: __raft.S.rateSent, view: document.querySelector('#cv').getBoundingClientRect().width }`);
    check(/"frame_hz":5/.test(slow.rate) && slow.fps < 8 && slow.view === 0, 'Plots: no 3D view, so the page asks for 5 frames a second (' + JSON.stringify(slow) + ')');
    await evaluate(`document.querySelector('#scGo').click()`);
    let sc = null;
    for (let k = 0; k < 40; k++) { await sleep(250); sc = await json(`__raft.SC.data ? { n: __raft.SC.data.n, hz: __raft.SC.data.hz, kv: document.querySelectorAll('#scKv dt').length, leg: document.querySelectorAll('#scLeg span').length } : null`); if (sc) break; }
    await shot('desktop-plots');
    check(sc && sc.n >= 495 && sc.hz === 250 && sc.kv === 3 && sc.leg >= 3, 'Plots: a 2 s capture at 250 Hz from the arm, raw and two filters with their lag (' + JSON.stringify(sc) + ')');
  }
  if (p === 'health') {
    await sleep(2200);                                // a loop window, the servo miss rate and two pings
    await shot('desktop-health');
    const hl = await json(`{ hz: document.querySelector('#hlHz').textContent, kv: document.querySelectorAll('#hlKv dt').length, rows: document.querySelectorAll('#hbTbl tr').length,
      rtt: __raft.HL.rtt.length, net: document.querySelector('#hnKv').textContent.includes('/s of'), bud: document.querySelector('#hlBud i').style.width,
      axis: document.querySelectorAll('#hlAxis span').length, bus: document.querySelectorAll('#bdTbl tr').length, hist: __raft.HL.hist.length,
      noDiag: document.querySelector('#bdState').textContent === 'not on this firmware' }`);
    // web_sim has no bus diagnosis (bus_diag is the ESP32's servo bus)
    check(parseFloat(hl.hz) > 150 && hl.kv === 7 && hl.rows === 8 && hl.rtt >= 1 && hl.net && parseFloat(hl.bud) > 0 && hl.axis === 5 && (hl.bus === 8 || hl.noDiag) && hl.hist >= 2,
      'Health: loop tiles and the tick timeline, servo bus, bus diagnosis, ping, network, history (' + JSON.stringify(hl) + ')');
  }
}
const cards = await json(`{ prec: !!document.querySelector('#tPrec'), locks: document.querySelectorAll('#tLocks input').length,
  guide: document.querySelectorAll('#tGuide option').length, hs: !!document.querySelector('#hsSet'), hsFr: document.querySelector('#hsFr').value,
  stage: document.querySelector('#tStage').textContent, steps: document.querySelectorAll('#p-cal details.step').length, pj: document.querySelectorAll('#pj input').length,
  qh: document.querySelectorAll('.qh').length, jd5: document.querySelector('#jd4').value }`);
check(cards.prec && cards.locks === 6 && cards.guide === 5 && cards.hs && cards.hsFr !== '' && cards.stage === 'OFF' && cards.steps === 7 && cards.pj === 14 && cards.qh >= 12 && cards.jd5 !== '',
  'cards: teleop assist (precision, 6 locks, 5 guides), haptics settings and per-joint values filled from the arm, Calibrate in 7 steps, explanations behind (?) (' + JSON.stringify(cards) + ')');
// the search box: Ctrl K, find a setting, Enter shows it; Esc closes the box without going limp
await nav('drive');
await evaluate(`__raft.send({cmd: 'mode', mode: 'gravity'})`);
await sleep(600);
await key('k', 2);
await sleep(200);
const palOpen = await evaluate(`!document.querySelector('#pal').hidden && document.activeElement.id === 'palIn'`);
await cdp('Input.insertText', { text: 'stiffness' });
await sleep(150);
const first = await evaluate(`document.querySelector('#palList li.on').textContent`);
await shot('desktop-search');
await key('Enter');
await sleep(400);
const found = await json(`{ page: document.querySelector('#iTitle').textContent, closed: document.querySelector('#pal').hidden, flash: !!document.querySelector('.card.flash') }`);
check(palOpen && /Hold/.test(first) && found.page === 'Drive' && found.closed && found.flash, 'search: Ctrl K, "stiffness" finds Drive › Hold, Enter shows the card (' + first + ')');
await key('k', 2);
await sleep(150);
await cdp('Input.insertText', { text: 'go health' });
await sleep(150);
await key('Escape');
await sleep(500);
const esc1 = await json(`{ closed: document.querySelector('#pal').hidden, mode: __raft.S.f.mode }`);
await key('Escape');
await sleep(600);
const esc2 = await evaluate(`__raft.S.f.mode`);
check(esc1.closed && esc1.mode === 1 && esc2 === 0, 'Esc closes the search box and keeps the mode; the next Esc goes limp (' + JSON.stringify(esc1) + ', then mode ' + esc2 + ')');
await key('k', 2);
await sleep(150);
await cdp('Input.insertText', { text: 'go health' });
await sleep(150);
await key('Enter');
await sleep(400);
check(await evaluate(`document.querySelector('#nav [data-p=health]').classList.contains('on')`), 'search: "go health" opens Health');
await nav('drive');
// a move target being edited: the see-through ghost arm
await evaluate(`for (const [j, v] of [[0, 35], [1, 40], [2, 15], [4, 30]]) { const r = document.querySelector('#mvSliders [data-j="' + j + '"]'); r.value = v; r.dispatchEvent(new Event('input')); }`);
await sleep(500);
await shot('desktop-target');
await evaluate(`document.querySelector('#mvClear').click()`);
// servo_info on Servos: a row per servo, then one joint's raw registers
await nav('servos');
await evaluate(`document.querySelector('#svRead').click()`);
await sleep(900);
await evaluate(`document.querySelector('#svTable tr:nth-child(3) button').click()`);
await sleep(700);
const sv = await json(`{ rows: document.querySelectorAll('#svTable tr').length, raw: document.querySelector('#svRaw').textContent.split('\\n').length }`);
check(sv.rows === 8 && sv.raw >= 5, 'servo settings table (' + (sv.rows - 1) + ' servos) and a raw register dump');
await shot('desktop-servos');
// themes
await nav('drive');
await evaluate(`document.querySelector('#theme [data-t=dark]').click()`);
await sleep(600);
await shot('desktop-dark');
await nav('health');
await sleep(1200);
await shot('desktop-dark-health');
await evaluate(`document.querySelector('#theme [data-t=light]').click()`);
await nav('drive');
await sleep(500);
await shot('desktop-light');
// a laptop and a phone
await cdp('Emulation.setDeviceMetricsOverride', { width: 1180, height: 760, deviceScaleFactor: 1, mobile: false });
await sleep(700);
await shot('laptop-drive');
await cdp('Emulation.setDeviceMetricsOverride', { width: 390, height: 844, deviceScaleFactor: 2, mobile: true });
await sleep(900);
const over = await json(`(() => { const W = document.documentElement.clientWidth, out = [];
  const scrolls = (e) => { for (let p = e.parentElement; p; p = p.parentElement) { const o = getComputedStyle(p).overflowX; if (o === 'auto' || o === 'scroll') return true; } return false; };
  for (const e of document.querySelectorAll('body *')) { const r = e.getBoundingClientRect(); if (r.width && r.right > W + 1 && !scrolls(e)) out.push((e.id ? '#' + e.id : e.tagName.toLowerCase() + '.' + e.className) + ' ' + Math.round(r.right)); }
  return { W, scroll: document.documentElement.scrollWidth, out: out.slice(0, 12) }; })()`);
console.log('phone width check:', JSON.stringify(over));
check(over.scroll <= over.W + 1 && !over.out.length, 'no horizontal overflow at 390 px');
await shot('phone-drive');
await nav('cal');
await evaluate(`window.scrollTo(0, 0)`);
await sleep(400);
await shot('phone-cal');
await nav('health');
await sleep(400);
await shot('phone-health');
await cdp('Emulation.setDeviceMetricsOverride', { width: 1440, height: 900, deviceScaleFactor: 1, mobile: false });
await evaluate(`document.querySelector('#theme [data-t=auto]').click()`);
await nav('drive');
await sleep(500);

// simulator extras (web_sim.exe --calibrated): pull the floating arm with Shift-drag, then teleoperate the emulated PAR6
const sim = await json(`{ sim: !!(__raft.S.hello && __raft.S.hello.sim), cal: !!(__raft.S.st && __raft.S.st.calibrated) }`);
if (sim.sim && sim.cal) {
  console.log('simulator extras:');
  await evaluate(`__raft.send({cmd: 'mode', mode: 'gravity'})`);
  await sleep(800);
  const tcp0 = await json(`__raft.S.f.tcp.slice(0, 3)`);
  const c0 = await json(`(() => { const r = document.querySelector('#cv').getBoundingClientRect(); return [Math.round(r.left + r.width / 2), Math.round(r.top + r.height / 2)]; })()`);
  const mouse = (type, x, y) => cdp('Input.dispatchMouseEvent', { type, x, y, button: 'left', buttons: type === 'mouseReleased' ? 0 : 1, clickCount: 1, modifiers: 8 });
  await mouse('mousePressed', c0[0], c0[1]);
  for (let k = 1; k <= 10; k++) { await mouse('mouseMoved', c0[0] + 9 * k, c0[1] - 7 * k); await sleep(60); }
  await sleep(1500);
  await shot('sim-pull');
  const tcp1 = await json(`__raft.S.f.tcp.slice(0, 3)`);
  await mouse('mouseReleased', c0[0] + 90, c0[1] - 70);
  const moved = Math.hypot(tcp1[0] - tcp0[0], tcp1[1] - tcp0[1], tcp1[2] - tcp0[2]) * 1000;
  check(moved > 10, 'Shift-drag pulls the floating arm (TCP moved ' + moved.toFixed(0) + ' mm)');

  await nav('tele');
  await evaluate(`document.querySelector('#piHost').value = 'par6.sim'; document.querySelector('#tStart').click()`);
  let stage = '';
  for (let k = 0; k < 60 && stage !== 'COUPLED'; k++) { await sleep(250); stage = await evaluate(`__raft.S.st && __raft.S.st.teleop`); }
  await sleep(1500);
  await shot('sim-teleop');
  const tl = await json(`{ chip: document.querySelector('#cCoupled').hidden ? '' : document.querySelector('#cCoupled').textContent, nav: document.querySelector('#navTele').textContent,
    coup: document.querySelector('#tCoup').textContent, pipe: document.querySelector('#pipe').className, fol: !!(__raft.S.f.flags & 256) }`);
  check(stage === 'COUPLED' && tl.chip === 'COUPLED' && tl.nav === 'COUPLED' && /°/.test(tl.coup) && /coupled/.test(tl.pipe) && tl.fol,
    'teleoperation couples from the page: header chip, menu badge, coupling error from the frames (' + JSON.stringify(tl) + ')');
  await evaluate(`document.querySelector('#tHold').click()`);
  let held = '';
  for (let k = 0; k < 20 && held !== 'HELD'; k++) { await sleep(150); held = await evaluate(`__raft.S.st.teleop`); }
  await sleep(300);
  const holdUi = await json(`{ chip: document.querySelector('#tStage').textContent, btn: document.querySelector('#tHold').textContent, cls: document.querySelector('#tHold').className }`);
  await shot('sim-teleop-held');
  await evaluate(`document.querySelector('#tHold').click()`);
  let back = '';
  for (let k = 0; k < 20 && back !== 'COUPLED'; k++) { await sleep(150); back = await evaluate(`__raft.S.st.teleop`); }
  check(held === 'HELD' && holdUi.chip === 'HELD' && holdUi.btn === 'Release hold' && /held/.test(holdUi.cls) && back === 'COUPLED', 'Hold button: HELD, then COUPLED again');
  await evaluate(`(() => { const r = document.querySelector('#tPrec'); r.value = 0.3; r.dispatchEvent(new Event('change')); document.querySelector('#tl0').click(); })()`);
  await sleep(1200);
  const asst = await json(`{ p: __raft.S.st.precision, l: __raft.S.st.locks, fp: __raft.S.f.prec, fl: __raft.S.f.locks }`);
  check(Math.abs(asst.p - 0.3) < 1e-6 && asst.l === 1 && Math.abs(asst.fp - 0.3) < 1e-6 && asst.fl === 1, 'precision slider and the J1 lock reach the arm (status and frame)');
  await shot('sim-teleop-assist');
  await evaluate(`document.querySelector('#tl0').click(); (() => { const r = document.querySelector('#tPrec'); r.value = 1; r.dispatchEvent(new Event('change')); })()`);
  await sleep(400);
  await evaluate(`document.querySelector('#tStop').click()`);
  await sleep(500);
  await evaluate(`__raft.send({cmd: 'mode', mode: 'limp'})`);
  await nav('drive');
}
check(errors.length === 0, 'no script errors' + (errors.length ? ': ' + errors.join(' | ') : ''));
console.log('screenshots in', outDir);
console.log(fails ? 'SOME CHECKS FAILED' : 'ALL CHECKS PASSED');
await cdp('Browser.close').catch(() => {});
browser.kill();
setTimeout(() => process.exit(fails ? 1 : 0), 300);
