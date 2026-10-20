// An on-screen M5 Atom Joystick running the stock StampFlyController firmware.
// Controls follow docs/joystick.md and the packets follow docs/protocol.md.

const PACKET_MS = 10; // about 100 packets per second
const ARM_PACKETS = 20; // the arm byte stays 1 for about 20 packets after a press
const STICK_RATE = 3; // keyboard sticks move this much per second
const THROTTLE_RATE = 0.8; // slower, so hover can be found
const RETURN_RATE = 6; // and spring back this fast

// telemetry pulls the fields the joystick shows out of the 120 byte packet, from pkg/input/telemetry.go.
export function readTelemetry(p) {
  const v = new DataView(p.buffer, p.byteOffset, p.byteLength);
  if (p[0] !== 88 || p[1] !== 88) {
    return null;
  }
  return {
    roll: v.getFloat32(10, true),
    pitch: v.getFloat32(14, true),
    yaw: v.getFloat32(18, true),
    voltage: v.getFloat32(58, true),
    altitude: v.getFloat32(98, true),
    altFlag: p[110],
    mode: p[111], // 2 flight, 3 parking, 5 landing
  };
}

export class Joystick {
  constructor(root, sim, mac) {
    this.sim = sim;
    this.mac = mac;
    this.screen = 'setup';
    this.rate = false; // Sport when true, Stable when false
    this.autoAlt = true; // on at power on, from setup() in main.cpp of StampFlyController
    this.armPackets = 0;
    this.sticks = {yaw: 0, throttle: 0, roll: 0, pitch: 0};
    this.keys = new Set();
    this.keyThrottle = 0;
    this.pad = null;
    this.telemetry = null;
    this.telemetrySeq = -1;
    this.telemetryAt = 0;
    this.build(root);
    this.listen();
    this.render();
    this.lastTick = performance.now();
    setInterval(() => this.tick(), PACKET_MS);
  }

  build(root) {
    root.innerHTML = `
      <div class="atom">
        <div class="stick" data-stick="left"><div class="knob"></div></div>
        <div class="middle">
          <div class="buttons">
            <button class="side" data-button="left" title="Left button (Q)">L</button>
            <div class="screen" title="Press the screen (Space)" tabindex="0"></div>
            <button class="side" data-button="right" title="Right button (E)">R</button>
          </div>
          <p class="keys">WASD left stick · arrows right stick · Space screen · Q / E buttons</p>
        </div>
        <div class="stick" data-stick="right"><div class="knob"></div></div>
      </div>`;
    this.screenEl = root.querySelector('.screen');
    this.stickEls = {left: root.querySelector('[data-stick=left]'), right: root.querySelector('[data-stick=right]')};
    this.touch = {left: null, right: null};
  }

  listen() {
    this.screenEl.addEventListener('pointerdown', (e) => { e.preventDefault(); this.pressScreen(); });
    for (const b of document.querySelectorAll('[data-button]')) {
      b.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        b.dataset.button === 'left' ? this.pressLeft() : this.pressRight();
      });
    }
    for (const side of ['left', 'right']) {
      const el = this.stickEls[side];
      el.addEventListener('pointerdown', (e) => {
        el.setPointerCapture(e.pointerId);
        this.touch[side] = {id: e.pointerId};
        this.moveTouch(side, e);
      });
      el.addEventListener('pointermove', (e) => {
        if (this.touch[side]?.id === e.pointerId) {
          this.moveTouch(side, e);
        }
      });
      const end = (e) => {
        if (this.touch[side]?.id === e.pointerId) {
          this.touch[side] = null;
        }
      };
      el.addEventListener('pointerup', end);
      el.addEventListener('pointercancel', end);
    }
    window.addEventListener('keydown', (e) => {
      if (e.target.closest?.('select, input, textarea')) {
        return;
      }
      const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
      if (!e.repeat) {
        if (k === ' ') this.pressScreen();
        if (k === 'q') this.pressLeft();
        if (k === 'e') this.pressRight();
      }
      if ([' ', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(k)) {
        e.preventDefault();
      }
      this.keys.add(k);
    });
    window.addEventListener('keyup', (e) => {
      this.keys.delete(e.key.length === 1 ? e.key.toLowerCase() : e.key);
    });
    window.addEventListener('blur', () => this.keys.clear());
  }

  moveTouch(side, e) {
    const r = this.stickEls[side].getBoundingClientRect();
    const radius = r.width / 2;
    let x = (e.clientX - r.left - radius) / (radius * 0.7);
    let y = (r.top + radius - e.clientY) / (radius * 0.7);
    const len = Math.hypot(x, y);
    if (len > 1) {
      x /= len;
      y /= len;
    }
    this.touch[side].x = x;
    this.touch[side].y = y;
  }

  // Screen press: on setup it goes to running, on running it arms or disarms.
  pressScreen() {
    if (this.screen === 'setup') {
      this.screen = 'running';
    } else {
      this.armPackets = ARM_PACKETS;
    }
    this.render();
  }

  // Left button: on setup it toggles Stable and Sport, on running it goes back to setup and disarms.
  pressLeft() {
    if (this.screen === 'setup') {
      this.rate = !this.rate;
    } else {
      this.screen = 'setup';
      if (this.droneArmed()) {
        this.armPackets = ARM_PACKETS;
      }
    }
    this.render();
  }

  // Right button: on setup it toggles altitude mode.
  pressRight() {
    if (this.screen === 'setup') {
      this.autoAlt = !this.autoAlt;
      this.keyThrottle = 0;
      this.render();
    }
  }

  droneArmed() {
    const t = this.telemetry;
    return t && performance.now() - this.telemetryAt < 500 && (t.mode === 2 || t.mode === 5);
  }

  // keyAxis moves a keyboard stick toward the pressed direction, or back to centre.
  keyAxis(value, plus, minus, dt, spring = true, rate = STICK_RATE) {
    const want = (this.keys.has(plus) ? 1 : 0) - (this.keys.has(minus) ? 1 : 0);
    if (want !== 0) {
      return Math.max(-1, Math.min(1, value + want * rate * dt));
    }
    if (!spring) {
      return value;
    }
    const step = RETURN_RATE * dt;
    return Math.abs(value) <= step ? 0 : value - Math.sign(value) * step;
  }

  readSticks(dt) {
    const s = this.sticks;
    const k = this.keyState ??= {yaw: 0, roll: 0, pitch: 0};
    k.yaw = this.keyAxis(k.yaw, 'd', 'a', dt);
    k.roll = this.keyAxis(k.roll, 'ArrowRight', 'ArrowLeft', dt);
    k.pitch = this.keyAxis(k.pitch, 'ArrowUp', 'ArrowDown', dt);
    // A spring loaded throttle key is hard to hover with in Stabilize and Acro, so it stays put there. X centres it.
    this.keyThrottle = this.keyAxis(this.keyThrottle, 'w', 's', dt, this.autoAlt, THROTTLE_RATE);
    if (this.keys.has('x')) {
      this.keyThrottle = 0;
    }

    const l = this.touch.left, r = this.touch.right;
    const pad = this.readPad();
    s.yaw = l ? l.x : pad?.yaw || k.yaw;
    s.throttle = l ? l.y : pad?.throttle || this.keyThrottle;
    s.roll = r ? r.x : pad?.roll || k.roll;
    const forward = r ? r.y : pad?.forward || k.pitch;
    // Forward on the right stick tips the nose down, which is negative pitch. See docs/joystick.md.
    s.pitch = -forward;
  }

  // readPad uses the first gamepad in the standard layout: A is the screen, LB and RB the side buttons.
  readPad() {
    const pad = navigator.getGamepads?.().find((p) => p && p.mapping === 'standard');
    if (!pad) {
      return null;
    }
    const dz = (v) => (Math.abs(v) < 0.08 ? 0 : v);
    const pressed = pad.buttons.map((b) => b.pressed);
    const was = this.padButtons || [];
    if (pressed[0] && !was[0]) this.pressScreen();
    if (pressed[4] && !was[4]) this.pressLeft();
    if (pressed[5] && !was[5]) this.pressRight();
    this.padButtons = pressed;
    return {yaw: dz(pad.axes[0]), throttle: -dz(pad.axes[1]), roll: dz(pad.axes[2]), forward: -dz(pad.axes[3])};
  }

  packet() {
    const p = new Uint8Array(25);
    const v = new DataView(p.buffer);
    p.set(this.mac.slice(3), 0);
    const s = this.sticks;
    [s.yaw, s.throttle, s.roll, s.pitch].forEach((x, i) => v.setFloat32(3 + 4 * i, x, true));
    p[19] = this.armPackets > 0 ? 1 : 0;
    p[21] = this.rate ? 1 : 0;
    p[22] = this.autoAlt ? 4 : 5;
    let sum = 0;
    for (let i = 0; i < 24; i++) {
      sum += p[i];
    }
    p[24] = sum & 255;
    return p;
  }

  tick() {
    const now = performance.now();
    const dt = Math.min((now - this.lastTick) / 1000, 0.1);
    this.lastTick = now;
    this.readSticks(dt);
    this.sim.sendPacket(this.packet());
    if (this.armPackets > 0) {
      this.armPackets--;
    }
    const seq = this.sim.telemetrySeq();
    if (seq !== this.telemetrySeq) {
      this.telemetrySeq = seq;
      this.telemetry = readTelemetry(this.sim.telemetry());
      this.telemetryAt = now;
      if (this.screen === 'running') {
        this.render();
      }
    }
    this.drawSticks();
  }

  drawSticks() {
    const s = this.sticks;
    const place = (el, x, y) => {
      el.firstElementChild.style.transform = `translate(${x * 35}%, ${-y * 35}%)`;
    };
    place(this.stickEls.left, s.yaw, s.throttle);
    place(this.stickEls.right, s.roll, -s.pitch);
  }

  render() {
    const ctl = this.rate ? '<span class="tile sport">Sport</span>' : '<span class="tile stable">Stable</span>';
    const alt = this.autoAlt ? '<span class="tile auto">Auto</span>' : '<span class="tile manual">Manual</span>';
    if (this.screen === 'setup') {
      this.screenEl.className = 'screen setup';
      this.screenEl.innerHTML = `<div class="title">SETUP</div><div class="tiles">${ctl}${alt}</div><div class="hint">press to fly</div>`;
      return;
    }
    const t = this.telemetry;
    const fresh = t && performance.now() - this.telemetryAt < 500;
    let state = '<span class="lost">NO LINK</span>';
    if (fresh) {
      state = {2: '<span class="armed">ARMED</span>', 5: '<span class="landing">LANDING</span>'}[t.mode] ?? '<span class="safe">SAFE</span>';
    }
    this.screenEl.className = 'screen running';
    this.screenEl.innerHTML = `
      <div class="state">${state}</div>
      <div class="alt">${fresh ? t.altitude.toFixed(2) : '-.--'}<small> m</small></div>
      <div class="row"><span>${fresh ? t.voltage.toFixed(2) : '-.--'} V</span><span>${fresh ? Math.round(t.yaw) : '-'}°</span></div>
      <div class="tiles small">${ctl}${alt}</div>`;
  }
}
