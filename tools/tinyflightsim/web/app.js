import {Sim, STATE, MODES, FAILSAFES} from './sim.js';
import {View} from './view.js';

const $ = (id) => document.getElementById(id);
const sim = new Sim();
const view = new View($('view'));

const status = $('status');
function setStatus(text, cls = '') {
  status.textContent = text;
  status.className = 'pill ' + cls;
}

const consoleBox = $('console');
const consoleText = $('console-text');
const MAX_CONSOLE = 40000;
function log(text) {
  consoleText.textContent = (consoleText.textContent + text).slice(-MAX_CONSOLE);
  consoleText.scrollTop = consoleText.scrollHeight;
}

sim.addEventListener('status', (e) => {
  const d = e.detail;
  setStatus(d.status, d.status === 'running' ? 'running' : '');
  if (d.build) {
    log(`--- firmware ${d.build}\n`);
  }
});
sim.addEventListener('stdout', (e) => log(e.detail.data));
sim.addEventListener('error', (e) => {
  setStatus(e.detail.source === 'compiler' ? 'build failed' : 'error', 'error');
  log(e.detail.message + '\n');
  consoleBox.hidden = false;
});
sim.addEventListener('exited', (e) => setStatus(`exited ${e.detail.code}`, 'error'));

$('restart').onclick = () => {
  consoleText.textContent = '';
  sim.start();
};
$('pause').onclick = () => {
  const paused = sim.ctrl[0] === 0;
  sim.setSpeed(paused ? Number($('speed').value) : 0);
  $('pause').textContent = paused ? 'Pause' : 'Resume';
};
$('speed').onchange = () => {
  sim.setSpeed(Number($('speed').value));
  $('pause').textContent = 'Pause';
};
$('camera').onchange = () => view.setCamera($('camera').value);
$('console-toggle').onclick = () => { consoleBox.hidden = !consoleBox.hidden; };

const hud = {
  armed: $('hud-armed'), mode: $('hud-mode'), failsafe: $('hud-failsafe'), height: $('hud-height'),
  est: $('hud-est'), speed: $('hud-speed'), battery: $('hud-battery'), time: $('hud-time'),
};

function updateHUD(s) {
  const armed = s[STATE.armed] > 0;
  hud.armed.textContent = !s[STATE.calibrated] ? 'calibrating' : armed ? (s[STATE.landing] ? 'landing' : 'armed') : 'disarmed';
  hud.armed.className = armed ? 'armed' : '';
  hud.mode.textContent = MODES[s[STATE.mode]] ?? '-';
  const fs = s[STATE.failsafe];
  hud.failsafe.textContent = FAILSAFES[fs] ?? '-';
  hud.failsafe.className = fs ? 'warn' : '';
  hud.height.textContent = `${(-s[STATE.pos + 2]).toFixed(2)} m`;
  hud.est.textContent = `${(-s[STATE.est + 2]).toFixed(2)} m${s[STATE.xyValid] ? ' xy' : ''}`;
  const v = Math.hypot(s[STATE.vel], s[STATE.vel + 1]);
  hud.speed.textContent = `${v.toFixed(2)} m/s`;
  hud.battery.textContent = `${s[STATE.battery].toFixed(2)} V`;
  hud.time.textContent = `${s[STATE.time].toFixed(1)} s`;
}

let last = performance.now();
let hudAt = 0;
function frame(now) {
  const dt = Math.min((now - last) / 1000, 0.1);
  last = now;
  view.update(sim.state, STATE, dt);
  if (now - hudAt > 100) {
    hudAt = now;
    updateHUD(sim.state);
  }
  requestAnimationFrame(frame);
}

if (!crossOriginIsolated) {
  setStatus('not isolated', 'error');
  log('This page needs cross origin isolation for SharedArrayBuffer. Open it from the tinyflightsim server.\n');
  consoleBox.hidden = false;
} else {
  sim.start();
}
requestAnimationFrame(frame);

// For the joystick and for poking at the sim from the browser console.
window.tinyflight = {sim, view};
