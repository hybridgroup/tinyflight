'use strict';

// Runs the firmware wasm in a web worker. The WASI host and the pausable clock
// come from worker/runner.js of https://github.com/tinygo-org/playground
// Copyright The TinyGo Authors, BSD 3-clause license.

onmessage = (e) => {
  if (e.data.type === 'start') {
    start(e.data.url, e.data.shared, e.data.layout);
  }
};

async function start(url, shared, layout) {
  postMessage({type: 'status', status: 'building'});
  let res;
  try {
    res = await fetch(url, {cache: 'no-store'});
  } catch (err) {
    postMessage({type: 'error', message: `Could not reach the server: ${err.message}`});
    return;
  }
  if (res.headers.get('Content-Type') !== 'application/wasm') {
    postMessage({type: 'error', source: 'compiler', message: await res.text()});
    return;
  }
  postMessage({type: 'status', status: 'loading', build: res.headers.get('X-Build')});
  const runner = new Runner(shared, layout);
  try {
    await runner.load(res);
  } catch (err) {
    postMessage({type: 'error', message: String(err)});
    return;
  }
  postMessage({type: 'status', status: 'running'});
  try {
    runner.inst.exports._start();
    postMessage({type: 'exited', code: 0});
  } catch (err) {
    if (err === runner.exit) {
      postMessage({type: 'exited', code: runner.exitCode});
    } else {
      postMessage({type: 'error', message: String(err.stack || err)});
    }
  }
}

class Runner {
  constructor(shared, layout) {
    this.layout = layout;
    this.ctrl = new Int32Array(shared, 0, layout.ctrlInts);
    this.bytes = new Uint8Array(shared);
    this.state = new Float32Array(shared, layout.state, layout.stateFloats);
    this.exit = {}; // thrown by proc_exit
    this.decoder = new TextDecoder('utf-8');
    this.clock = new Clock(this.speed());
  }

  speed() {
    return Atomics.load(this.ctrl, this.layout.speed) / 100;
  }

  mem() {
    return new DataView(this.inst.exports.memory.buffer);
  }

  async load(res) {
    const SUCCESS = 0;
    const ERRNO_BADF = 8;
    const ERRNO_NOSYS = 52;
    const L = this.layout;
    const imports = {
      // The subset of WASI preview 1 that TinyGo uses.
      wasi_snapshot_preview1: {
        args_get: () => SUCCESS,
        args_sizes_get: (num, size) => {
          this.mem().setUint32(num, 0, true);
          this.mem().setUint32(size, 0, true);
          return SUCCESS;
        },
        environ_get: () => SUCCESS,
        environ_sizes_get: (num, size) => {
          this.mem().setUint32(num, 0, true);
          this.mem().setUint32(size, 0, true);
          return SUCCESS;
        },
        clock_time_get: (id, precision, retptr) => {
          this.mem().setBigUint64(retptr, BigInt(Math.round(this.clock.now() * 1e6)), true);
          return SUCCESS;
        },
        fd_close: () => ERRNO_NOSYS,
        fd_fdstat_get: () => ERRNO_NOSYS,
        fd_fdstat_set_flags: () => ERRNO_NOSYS,
        fd_prestat_get: (fd) => fd > 2 ? ERRNO_BADF : ERRNO_NOSYS,
        fd_prestat_dir_name: () => ERRNO_NOSYS,
        fd_seek: () => ERRNO_NOSYS,
        fd_write: (fd, iovs, n, nwritten) => this.write(fd, iovs, n, nwritten),
        poll_oneoff: (subIn, eventOut, nsub, retptr) => {
          // Only the single clock subscription that time.Sleep makes.
          const timeout = Number(this.mem().getBigUint64(subIn + 24, true)) / 1e6;
          this.sleep(timeout);
          return SUCCESS;
        },
        proc_exit: (code) => {
          this.exitCode = code;
          throw this.exit;
        },
        sched_yield: () => SUCCESS,
        random_get: (ptr, len) => {
          crypto.getRandomValues(new Uint8Array(this.inst.exports.memory.buffer, ptr, len));
          return SUCCESS;
        },
      },
      // See boards/sim/bridge.go.
      tinyflightsim: {
        read_input: (ptr, len) => {
          this.checkStop();
          const dst = new Uint8Array(this.inst.exports.memory.buffer, ptr, len);
          for (;;) {
            const seq = Atomics.load(this.ctrl, L.inputSeq);
            dst.set(this.bytes.subarray(L.input, L.input + len));
            if (Atomics.load(this.ctrl, L.inputSeq) === seq) {
              return seq;
            }
          }
        },
        publish_state: (ptr, len) => {
          this.state.set(new Float32Array(this.inst.exports.memory.buffer, ptr, len));
          Atomics.add(this.ctrl, L.stateSeq, 1);
        },
        telemetry: (ptr, len) => {
          this.bytes.set(new Uint8Array(this.inst.exports.memory.buffer, ptr, len), L.telemetry);
          Atomics.add(this.ctrl, L.telemetrySeq, 1);
        },
      },
    };
    const result = await WebAssembly.instantiateStreaming(res, imports);
    this.inst = result.instance;
  }

  // sleep blocks for ms of simulated time. It stops while paused and follows speed changes.
  sleep(ms) {
    const L = this.layout;
    let end = this.clock.now() + ms;
    for (;;) {
      this.checkStop();
      const speed = Atomics.load(this.ctrl, L.speed);
      if (speed === 0) {
        this.clock.setRate(0);
        Atomics.wait(this.ctrl, L.speed, 0);
        continue;
      }
      this.clock.setRate(speed / 100);
      const left = end - this.clock.now();
      if (left <= 0) {
        return;
      }
      Atomics.wait(this.ctrl, L.speed, speed, left / this.clock.rate);
    }
  }

  checkStop() {
    if (Atomics.load(this.ctrl, this.layout.stop)) {
      this.exitCode = 0;
      throw this.exit;
    }
  }

  write(fd, iovs, n, nwritten) {
    let total = 0;
    let text = '';
    for (let i = 0; i < n; i++) {
      const ptr = this.mem().getUint32(iovs + i * 8, true);
      const len = this.mem().getUint32(iovs + i * 8 + 4, true);
      total += len;
      text += this.decoder.decode(new Uint8Array(this.inst.exports.memory.buffer, ptr, len));
    }
    if (text !== '') {
      postMessage({type: 'stdout', data: text});
    }
    this.mem().setUint32(nwritten, total, true);
    return 0;
  }
}

// Clock is simulated time in ms since start. It runs at rate times real time.
class Clock {
  constructor(rate) {
    this.base = 0;
    this.origin = performance.now();
    this.rate = rate;
  }

  now() {
    return this.base + (performance.now() - this.origin) * this.rate;
  }

  setRate(rate) {
    if (rate === this.rate) {
      return;
    }
    this.base = this.now();
    this.origin = performance.now();
    this.rate = rate;
  }
}
