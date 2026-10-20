// Sim owns the runner worker and the memory it shares with it.

// Shared memory layout. The runner gets a copy in its start message.
const LAYOUT = {
  ctrlInts: 8,
  speed: 0, // percent of real time, 0 is paused
  inputSeq: 1,
  stateSeq: 2,
  telemetrySeq: 3,
  stop: 4, // set to end the run
  input: 32, // byte offset of the joystick packet
  telemetry: 64,
  state: 192,
  stateFloats: 64,
  size: 448,
};

// Offsets into the state record, from the State constants in boards/sim/world.go.
export const STATE = {
  time: 0, pos: 1, vel: 4, quat: 7, motors: 11, led: 15, est: 18,
  armed: 21, mode: 22, failsafe: 23, landing: 24, battery: 25, calibrated: 26, xyValid: 27,
};

export const MODES = ['Acro', 'Stabilize', 'AltHold', 'PosHold'];
export const FAILSAFES = ['none', 'input lost', 'low battery', 'tilt'];

export class Sim extends EventTarget {
  constructor() {
    super();
    this.speed = 100;
    this.worker = null;
    this.share();
  }

  // share makes fresh memory for a run. A stopped worker can run on for a moment, so it must not share it.
  share() {
    this.shared = new SharedArrayBuffer(LAYOUT.size);
    this.ctrl = new Int32Array(this.shared, 0, LAYOUT.ctrlInts);
    this.bytes = new Uint8Array(this.shared);
    this.state = new Float32Array(this.shared, LAYOUT.state, LAYOUT.stateFloats);
  }

  // start powers on the drone, which builds the firmware first if the sources changed.
  start() {
    this.stop();
    this.share();
    Atomics.store(this.ctrl, LAYOUT.speed, this.speed);
    this.worker = new Worker('runner.js');
    this.worker.onmessage = (e) => this.dispatchEvent(new CustomEvent(e.data.type, {detail: e.data}));
    this.worker.onerror = (e) => this.dispatchEvent(new CustomEvent('error', {detail: {message: e.message}}));
    this.worker.postMessage({type: 'start', url: 'api/firmware.wasm', shared: this.shared, layout: LAYOUT});
  }

  stop() {
    if (this.worker) {
      Atomics.store(this.ctrl, LAYOUT.stop, 1);
      Atomics.notify(this.ctrl, LAYOUT.speed);
      this.worker.terminate();
      this.worker = null;
    }
  }

  setSpeed(percent) {
    this.speed = percent;
    Atomics.store(this.ctrl, LAYOUT.speed, percent);
    Atomics.notify(this.ctrl, LAYOUT.speed);
  }

  // sendPacket hands a joystick packet to the drone, as if it came over ESP-NOW.
  sendPacket(p) {
    this.bytes.set(p, LAYOUT.input);
    Atomics.add(this.ctrl, LAYOUT.inputSeq, 1);
  }

  stateSeq() {
    return Atomics.load(this.ctrl, LAYOUT.stateSeq);
  }

  telemetrySeq() {
    return Atomics.load(this.ctrl, LAYOUT.telemetrySeq);
  }

  // telemetry is the latest packet the drone sent back, 120 bytes.
  telemetry() {
    return this.bytes.slice(LAYOUT.telemetry, LAYOUT.telemetry + 120);
  }
}
