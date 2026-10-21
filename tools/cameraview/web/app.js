'use strict';

// Colors from cold to hot, an approximation of the ironbow palette used by thermal cameras.
const stops = [
  [0, 0, 0, 0], [0.15, 32, 0, 140], [0.35, 145, 0, 155], [0.55, 225, 60, 30],
  [0.75, 250, 150, 0], [0.9, 255, 225, 60], [1, 255, 255, 255],
];

function color(t) {
  t = Math.min(1, Math.max(0, t));
  for (let i = 1; i < stops.length; i++) {
    const [b, ...hi] = stops[i];
    if (t <= b) {
      const [a, ...lo] = stops[i - 1];
      const k = (t - a) / (b - a);
      return lo.map((v, j) => Math.round(v + (hi[j] - v) * k));
    }
  }
  return stops[stops.length - 1].slice(1);
}

const lut = Array.from({ length: 256 }, (_, i) => color(i / 255));

const image = document.getElementById('image');
const ctx = image.getContext('2d');
const status = document.getElementById('status');
const rangeSel = document.getElementById('range');
const smooth = document.getElementById('smooth');
const flipV = document.getElementById('flip-v');
const flipH = document.getElementById('flip-h');
const cursor = document.getElementById('cursor');
const $ = (id) => document.getElementById(id);

const bar = $('bar').getContext('2d');
for (let y = 0; y < 256; y++) {
  const [r, g, b] = lut[255 - y];
  bar.fillStyle = `rgb(${r},${g},${b})`;
  bar.fillRect(0, y, 16, 1);
}

let last = null;
let times = [];
let mouse = null;
let auto = null;

// The sensor noise is about 0.3 °C, so a narrow auto range turns an empty scene into color noise.
const minSpan = 8;

function range(values) {
  switch (rangeSel.value) {
    case 'room': return [15, 40];
    case 'wide': return [0, 80];
  }
  const lo = Math.min(...values);
  const hi = Math.max(Math.max(...values), lo + minSpan);
  // Ease toward the new range so the colors do not flicker from frame to frame.
  auto = auto ? auto.map((v, i) => v + ([lo, hi][i] - v) * 0.2) : [lo, hi];
  return auto;
}

// view maps a fractional position on the screen to one on the sensor, for a flipped mount.
function view(fx, fy) {
  return [flipH.checked ? 1 - fx : fx, flipV.checked ? 1 - fy : fy];
}

// sample reads the image at fractional pixel coordinates, bilinear when smoothing.
function sample(values, w, h, x, y) {
  if (!smooth.checked) {
    return values[Math.min(h - 1, Math.floor(y + 0.5)) * w + Math.min(w - 1, Math.floor(x + 0.5))];
  }
  x = Math.min(w - 1, Math.max(0, x));
  y = Math.min(h - 1, Math.max(0, y));
  const x0 = Math.floor(x), y0 = Math.floor(y);
  const x1 = Math.min(w - 1, x0 + 1), y1 = Math.min(h - 1, y0 + 1);
  const fx = x - x0, fy = y - y0;
  const top = values[y0 * w + x0] * (1 - fx) + values[y0 * w + x1] * fx;
  const bot = values[y1 * w + x0] * (1 - fx) + values[y1 * w + x1] * fx;
  return top * (1 - fy) + bot * fy;
}

function draw() {
  if (!last) return;
  const { width: w, height: h, values } = last;
  const [lo, hi] = range(values);
  const size = image.width;
  const img = ctx.createImageData(size, size);
  for (let py = 0; py < size; py++) {
    for (let px = 0; px < size; px++) {
      const [fx, fy] = view((px + 0.5) / size, (py + 0.5) / size);
      const v = sample(values, w, h, fx * w - 0.5, fy * h - 0.5);
      const [r, g, b] = lut[Math.round(Math.min(1, Math.max(0, (v - lo) / (hi - lo))) * 255)];
      const i = (py * size + px) * 4;
      img.data[i] = r;
      img.data[i + 1] = g;
      img.data[i + 2] = b;
      img.data[i + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);

  const fmt = (v) => `${v.toFixed(1)} ${last.unit}`;
  $('scale-max').textContent = hi.toFixed(1);
  $('scale-min').textContent = lo.toFixed(1);
  $('hud-min').textContent = fmt(Math.min(...values));
  $('hud-max').textContent = fmt(Math.max(...values));
  $('hud-center').textContent = fmt(sample(values, w, h, (w - 1) / 2, (h - 1) / 2));
  $('hud-seq').textContent = last.seq;
  showCursor();
}

function showCursor() {
  if (!last || !mouse) {
    cursor.hidden = true;
    return;
  }
  const { width: w, height: h, values } = last;
  const [fx, fy] = view(mouse.fx, mouse.fy);
  const v = sample(values, w, h, fx * w - 0.5, fy * h - 0.5);
  cursor.hidden = false;
  cursor.style.left = `${mouse.x}px`;
  cursor.style.top = `${mouse.y}px`;
  cursor.textContent = `${v.toFixed(1)} ${last.unit}`;
}

image.addEventListener('mousemove', (e) => {
  const r = image.getBoundingClientRect();
  mouse = { x: e.clientX - r.left, y: e.clientY - r.top, fx: (e.clientX - r.left) / r.width, fy: (e.clientY - r.top) / r.height };
  showCursor();
});
image.addEventListener('mouseleave', () => {
  mouse = null;
  showCursor();
});
rangeSel.addEventListener('change', () => {
  auto = null;
  draw();
});
smooth.addEventListener('change', draw);

// Remember the flips for this browser, since they follow how the camera is mounted.
for (const box of [flipV, flipH]) {
  try {
    box.checked = localStorage.getItem(box.id) === '1';
  } catch {}
  box.addEventListener('change', () => {
    try {
      localStorage.setItem(box.id, box.checked ? '1' : '0');
    } catch {}
    draw();
  });
}

function setStatus(text, cls) {
  status.textContent = text;
  status.className = `pill ${cls || ''}`;
}

const events = new EventSource('events');
events.onopen = () => setStatus('waiting for frames', 'warn');
events.onerror = () => setStatus('server gone', 'warn');
events.onmessage = (e) => {
  const f = JSON.parse(e.data);
  last = { ...f, values: f.pixels.map((p) => p * f.scale) };
  const now = performance.now();
  times.push(now);
  times = times.filter((t) => now - t < 2000);
  $('hud-fps').textContent = `${(times.length / 2).toFixed(1)} fps`;
  setStatus('live', 'ok');
  draw();
};

// Mark the stream stale when frames stop, for example when the drone is out of range.
setInterval(() => {
  if (times.length && performance.now() - times[times.length - 1] > 1500) {
    setStatus('no frames', 'warn');
    $('hud-fps').textContent = '0 fps';
  }
}, 500);
