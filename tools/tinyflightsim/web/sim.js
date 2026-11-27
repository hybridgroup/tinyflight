// Sim owns the runner worker and the memory it shares with it.

// MAC addresses from boards/sim/world.go.
export const DRONE_MAC = [0x02, 0x54, 0x46, 0x53, 0x49, 0x4D];
export const JOYSTICK_MAC = [0x02, 0x54, 0x46, 0x4A, 0x4F, 0x59];

const TERRAIN = 448;
const TERRAIN_FLOATS = 5 + 100 * 100;
const QUEUE = TERRAIN + 4 * TERRAIN_FLOATS;

// Shared memory layout. The runner gets a copy in its start message.
const LAYOUT = {
  ctrlInts: 8,
  speed: 0, // percent of real time, 0 is paused
  queueWrite: 1, // packets put in the queue, by the page
  stateSeq: 2,
  telemetrySeq: 3,
  stop: 4, // set to end the run
  queueRead: 5, // packets taken from the queue, by the runner
  telemetry: 64, // the last packet the drone sent the joystick
  state: 192,
  stateFloats: 64,
  terrain: TERRAIN, // the record from terrain.js
  terrainFloats: TERRAIN_FLOATS,
  // Packets on their way to the drone. Each slot is the 6 byte source, the length, and the payload from byte 8.
  queue: QUEUE,
  slots: 16,
  slotSize: 48,
  joystick: JOYSTICK_MAC,
  size: QUEUE + 16 * 48,
};

// Offsets into the state record, from the State constants in boards/sim/world.go.
export const STATE = {
  time: 0, pos: 1, vel: 4, quat: 7, motors: 11, led: 15, est: 18,
  armed: 21, mode: 22, failsafe: 23, landing: 24, battery: 25, calibrated: 26, xyValid: 27, owner: 28,
};

// Owners from espnowinput.Owner.
export const OWNERS = ['none', 'joystick', 'ground station'];

export const MODES = ['Acro', 'Stabilize', 'AltHold', 'PosHold'];
export const FAILSAFES = ['none', 'input lost', 'low battery', 'tilt', 'height lost'];

export class Sim extends EventTarget {
  constructor() {
    super();
    this.speed = 100;
    this.worker = null;
    this.terrain = new Float32Array(5);
    this.share();
  }

  // share makes fresh memory for a run. A stopped worker can run on for a moment, so it must not share it.
  share() {
    this.shared = new SharedArrayBuffer(LAYOUT.size);
    this.ctrl = new Int32Array(this.shared, 0, LAYOUT.ctrlInts);
    this.bytes = new Uint8Array(this.shared);
    this.state = new Float32Array(this.shared, LAYOUT.state, LAYOUT.stateFloats);
  }

  // setTerrain takes a record from terrain.js for the next start.
  setTerrain(rec) {
    this.terrain = rec;
  }

  // start powers on the drone, which builds the firmware first if the sources changed.
  start() {
    this.stop();
    this.share();
    new Float32Array(this.shared, LAYOUT.terrain, LAYOUT.terrainFloats).set(this.terrain);
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

  // sendPacket hands a packet from src to the drone, as if it came over ESP-NOW. A full queue drops it, like a busy radio.
  sendPacket(src, p) {
    const L = LAYOUT;
    const w = Atomics.load(this.ctrl, L.queueWrite);
    if (w - Atomics.load(this.ctrl, L.queueRead) >= L.slots || p.length > L.slotSize - 8) {
      return;
    }
    const at = L.queue + (w % L.slots) * L.slotSize;
    this.bytes.set(src, at);
    this.bytes[at + 6] = p.length;
    this.bytes.set(p, at + 8);
    Atomics.store(this.ctrl, L.queueWrite, w + 1);
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
