// Sound plays a motor hum that follows the four motor commands. From sound.js in https://github.com/hybridgroup/jeyzma
import {STATE} from './sim.js';

const KEY = 'tinyflightsim.sound';

export class Sound {
  constructor(sim) {
    this.sim = sim;
    this.audio = null;
    this.motors = [];
    this.on = load();
    const start = () => this.start();
    addEventListener('pointerdown', start);
    addEventListener('keydown', start);
    document.addEventListener('visibilitychange', () => {
      if (this.audio) {
        document.hidden ? this.audio.suspend() : this.audio.resume();
      }
    });
    requestAnimationFrame(() => this.update());
  }

  // start makes the sound on the first touch or key, since browsers block audio before that.
  start() {
    if (this.audio) {
      return;
    }
    const audio = new AudioContext();
    const out = audio.createGain();
    out.gain.value = 0.2;
    out.connect(audio.destination);
    const filter = audio.createBiquadFilter();
    filter.type = 'lowpass';
    filter.frequency.value = 1800;
    filter.connect(out);
    this.motors = [0, 1, 2, 3].map(() => {
      const osc = audio.createOscillator();
      osc.type = 'sawtooth';
      const gain = audio.createGain();
      gain.gain.value = 0;
      osc.connect(gain).connect(filter);
      osc.start();
      return {osc, gain};
    });
    this.audio = audio;
  }

  setOn(on) {
    this.on = on;
    try {
      localStorage.setItem(KEY, on ? '1' : '0');
    } catch {}
  }

  update() {
    if (this.audio) {
      const s = this.sim.state;
      const running = this.on && this.sim.ctrl[0] > 0;
      const t = this.audio.currentTime;
      this.motors.forEach(({osc, gain}, i) => {
        const cmd = running ? Math.max(0, Math.min(1, s[STATE.motors + i])) : 0;
        // A small offset per motor gives the beat of four props.
        osc.frequency.setTargetAtTime(120 + cmd * 520 + i * 2, t, 0.04);
        gain.gain.setTargetAtTime(cmd > 0.01 ? 0.1 + cmd * 0.15 : 0, t, 0.04);
      });
    }
    requestAnimationFrame(() => this.update());
  }
}

function load() {
  try {
    return localStorage.getItem(KEY) !== '0';
  } catch {
    return true;
  }
}
