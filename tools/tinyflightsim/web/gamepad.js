// Gamepad reads any gamepad or joystick through a mapping that the Controller panel can change.

const KEY = 'tinyflightsim.pads';
const DEAD_ZONE = 0.08;

// Stick axes, with pitch positive for forward like the right stick of the Atom Joystick.
export const AXES = ['throttle', 'yaw', 'pitch', 'roll'];
export const BUTTONS = ['screen', 'left', 'right'];

// defaultMap follows the standard layout, or AETR for other devices since EdgeTX sends that order.
export function defaultMap(pad) {
  if (pad.mapping === 'standard') {
    return {
      throttle: {axis: 1, invert: true}, yaw: {axis: 0, invert: false},
      pitch: {axis: 3, invert: true}, roll: {axis: 2, invert: false},
      screen: 0, left: 4, right: 5,
    };
  }
  return {
    throttle: {axis: 2, invert: false}, yaw: {axis: 3, invert: false},
    pitch: {axis: 1, invert: false}, roll: {axis: 0, invert: false},
    screen: 0, left: 1, right: 2,
  };
}

export class Gamepad {
  constructor() {
    this.maps = load();
    this.learning = null; // a button name while waiting for a press
    this.held = [];
  }

  pad() {
    return navigator.getGamepads?.().find((p) => p && p.connected) ?? null;
  }

  map(pad) {
    return this.maps[pad.id] ?? defaultMap(pad);
  }

  setMap(pad, m) {
    this.maps[pad.id] = m;
    save(this.maps);
  }

  reset(pad) {
    delete this.maps[pad.id];
    this.learning = null;
    save(this.maps);
  }

  // read returns the sticks from -1 to 1 and which buttons are down, or null with no pad.
  read() {
    const pad = this.pad();
    if (!pad) {
      return null;
    }
    const m = this.map(pad);
    const axis = (a) => {
      const v = (pad.axes[a.axis] ?? 0) * (a.invert ? -1 : 1);
      return Math.abs(v) < DEAD_ZONE ? 0 : v;
    };
    const down = pad.buttons.map((b) => b.pressed);
    const out = {pad, throttle: axis(m.throttle), yaw: axis(m.yaw), pitch: axis(m.pitch), roll: axis(m.roll)};
    if (this.learning) {
      const i = down.findIndex((d, j) => d && !this.held[j]);
      if (i >= 0) {
        this.setMap(pad, {...m, [this.learning]: i});
        this.learning = null;
      }
      out.screen = out.left = out.right = false;
    } else {
      for (const b of BUTTONS) {
        out[b] = !!down[m[b]];
      }
    }
    this.held = down;
    return out;
  }
}

function load() {
  try {
    return JSON.parse(localStorage.getItem(KEY)) ?? {};
  } catch {
    return {};
  }
}

function save(maps) {
  try {
    localStorage.setItem(KEY, JSON.stringify(maps));
  } catch {}
}

// ControllerPanel shows the device, its live axes and the mapping, and lets the pilot change it.
export class ControllerPanel {
  constructor(root, gamepad) {
    this.root = root;
    this.gamepad = gamepad;
    this.padId = undefined;
    setInterval(() => this.refresh(), 100);
  }

  refresh() {
    if (this.root.hidden) {
      return;
    }
    const pad = this.gamepad.pad();
    const id = pad ? `${pad.id}/${pad.axes.length}/${pad.buttons.length}` : null;
    if (id !== this.padId) {
      this.padId = id;
      this.build(pad);
    }
    if (!pad) {
      return;
    }
    pad.axes.forEach((v, i) => {
      const bar = this.bars[i];
      bar.style.left = `${50 + Math.min(v, 0) * 50}%`;
      bar.style.width = `${Math.abs(v) * 50}%`;
    });
    const m = this.gamepad.map(pad);
    for (const b of BUTTONS) {
      const learning = this.gamepad.learning === b;
      this.buttonEls[b].textContent = learning ? 'press a button' : `Button ${m[b]}`;
      this.buttonEls[b].classList.toggle('learning', learning);
    }
  }

  build(pad) {
    if (!pad) {
      this.root.innerHTML = '<p>No gamepad or joystick found. Plug one in and press a button on it.</p>';
      return;
    }
    const m = this.gamepad.map(pad);
    const options = pad.axes.map((_, i) => `<option value="${i}">Axis ${i}</option>`).join('');
    const names = {throttle: 'Throttle', yaw: 'Yaw', pitch: 'Pitch, forward', roll: 'Roll, right'};
    this.root.innerHTML = `
      <p class="device"></p>
      <div class="axes">${pad.axes.map((_, i) => `<span>${i}</span><div class="bar"><div></div></div>`).join('')}</div>
      <div class="map">
        ${AXES.map((a) => `<span>${names[a]}</span><select data-axis="${a}">${options}</select>
          <label><input type="checkbox" data-invert="${a}"> invert</label>`).join('')}
        ${BUTTONS.map((b) => `<span>${b[0].toUpperCase() + b.slice(1)} button</span><button data-learn="${b}"></button><span></span>`).join('')}
      </div>
      <button data-reset>Reset to defaults</button>`;
    this.root.querySelector('.device').textContent = `${pad.id} (${pad.mapping || 'no standard layout'})`;
    this.bars = [...this.root.querySelectorAll('.bar > div')];
    this.buttonEls = {};
    for (const a of AXES) {
      const sel = this.root.querySelector(`[data-axis=${a}]`);
      const inv = this.root.querySelector(`[data-invert=${a}]`);
      sel.value = m[a].axis;
      inv.checked = m[a].invert;
      const change = () => {
        const cur = this.gamepad.map(pad);
        this.gamepad.setMap(pad, {...cur, [a]: {axis: Number(sel.value), invert: inv.checked}});
        sel.blur();
      };
      sel.onchange = change;
      inv.onchange = change;
    }
    for (const b of BUTTONS) {
      const el = this.buttonEls[b] = this.root.querySelector(`[data-learn=${b}]`);
      el.onclick = () => {
        this.gamepad.learning = this.gamepad.learning === b ? null : b;
        el.blur();
      };
    }
    this.root.querySelector('[data-reset]').onclick = (e) => {
      this.gamepad.reset(pad);
      this.padId = undefined;
      e.target.blur();
    };
  }
}
