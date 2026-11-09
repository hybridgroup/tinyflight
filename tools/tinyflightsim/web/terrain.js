// Random terrain as a height map. The firmware reads the same map through read_terrain, see boards/sim/bridge.go.

export const CELL = 0.2; // m
export const COLS = 100; // along north, TerrainMaxCells in boards/sim/world.go is COLS * ROWS
export const ROWS = 100; // along east
const PAD = 1.5; // m around the takeoff spot that stays flat

// mulberry32, so a seed always gives the same terrain.
function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// randomTerrain makes blocks, steps and rough patches around a flat pad at the origin.
export function randomTerrain(seed) {
  const rnd = rng(seed);
  const between = (a, b) => a + rnd() * (b - a);
  const t = {seed, x0: -COLS * CELL / 2, y0: -ROWS * CELL / 2, cell: CELL, cols: COLS, rows: ROWS, h: new Float32Array(COLS * ROWS)};
  const raise = (x, y, w, d, height) => {
    const i0 = Math.max(0, Math.floor((x - t.x0) / CELL)), i1 = Math.min(COLS, Math.ceil((x + w - t.x0) / CELL));
    const j0 = Math.max(0, Math.floor((y - t.y0) / CELL)), j1 = Math.min(ROWS, Math.ceil((y + d - t.y0) / CELL));
    for (let j = j0; j < j1; j++) {
      for (let i = i0; i < i1; i++) {
        const k = j * COLS + i;
        t.h[k] = Math.max(t.h[k], typeof height === 'function' ? height() : height);
      }
    }
  };
  const spot = (margin) => [between(t.x0, -t.x0 - margin), between(t.y0, -t.y0 - margin)];

  for (let n = 0; n < 30; n++) {
    const [x, y] = spot(1);
    raise(x, y, between(0.4, 1.6), between(0.4, 1.6), Math.round(between(0.1, 0.8) * 20) / 20);
  }
  for (let n = 0; n < 6; n++) {
    const [x, y] = spot(2);
    const steps = 3 + Math.floor(rnd() * 3), rise = between(0.1, 0.15), width = between(0.8, 1.6);
    const alongX = rnd() < 0.5;
    for (let s = 0; s < steps; s++) {
      const h = rise * (s + 1);
      alongX ? raise(x + s * 0.4, y, 0.4, width, h) : raise(x, y + s * 0.4, width, 0.4, h);
    }
  }
  for (let n = 0; n < 6; n++) {
    const [x, y] = spot(2.5);
    const size = between(1, 2.5);
    raise(x, y, size, size, () => rnd() * 0.1);
  }

  for (let j = 0; j < ROWS; j++) {
    for (let i = 0; i < COLS; i++) {
      const x = t.x0 + (i + 0.5) * CELL, y = t.y0 + (j + 0.5) * CELL;
      if (Math.abs(x) < PAD && Math.abs(y) < PAD) {
        t.h[j * COLS + i] = 0;
      }
    }
  }
  return t;
}

// heightAt is the height of the surface at north x and east y, 0 for a flat floor.
export function heightAt(t, x, y) {
  if (!t) {
    return 0;
  }
  const i = Math.floor((x - t.x0) / t.cell), j = Math.floor((y - t.y0) / t.cell);
  if (i < 0 || j < 0 || i >= t.cols || j >= t.rows) {
    return 0;
  }
  return t.h[j * t.cols + i];
}

// record lays the terrain out for World.SetTerrain in boards/sim/world.go.
export function record(t) {
  if (!t) {
    return new Float32Array(5);
  }
  const r = new Float32Array(5 + t.h.length);
  r.set([t.x0, t.y0, t.cell, t.cols, t.rows]);
  r.set(t.h, 5);
  return r;
}
