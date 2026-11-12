// Viewer of the runs in results/traj (see scripts/viewer.sh): the runs of one scenario overlaid
// in 3D, one plot per series with its theory, a shared time cursor and a table that is legend,
// visibility switch and metrics. Scenarios, variants, engines and contact methods come from
// index.json (tools/viewer/index.py); nothing here names them.
import * as THREE from 'three';
import { OBJLoader } from './vendor/OBJLoader.js';
import { OrbitControls } from './vendor/OrbitControls.js';
import uPlot from './vendor/uPlot.esm.js';
import { DESCRIPTIONS, NAMES, QUANTITIES, STATUSES, THEORY_CLASSES, VARIANTS, VARIANT_GROUPS } from './descriptions.js';

const SCENARIOS = await import('./scenarios.js').then(m => m.SCENARIOS ?? {}, () => ({}));   // [title, theory, class]

const TRAJ = document.documentElement.dataset.traj ?? '../../results/traj/';
const ASSETS = document.documentElement.dataset.assets ?? '../../';
const SPEEDS = [0.1, 0.25, 0.5, 1, 2];
const RENDER_MODES = { solid: 'Solid', edges: 'Solid + edges', wire: 'Wireframe' };
const INK = '#14181C', GRID = '#D3D8DD', PANEL = '#F8F9FA', MUTED = '#5E6770';

const $ = id => document.getElementById(id);
const el = (tag, props = {}, ...children) => {
  const node = Object.assign(document.createElement(tag), props);
  node.append(...children);
  return node;
};
const sceneEl = $('scene'), plotsEl = $('plots');
const nameOf = id => NAMES[id] ?? id.replace(/^./, c => c.toUpperCase());
const words = name => name.replace(/_/g, ' ');
function nf(x, digits = 4) {
  if (x == null || !Number.isFinite(x)) return '—';
  const a = Math.abs(x);
  if (a !== 0 && (a < 1e-4 || a >= 1e6)) return x.toExponential(Math.max(digits - 2, 1));
  return x.toLocaleString('en', { maximumSignificantDigits: digits, useGrouping: false });
}
const pct = x => (x == null || !Number.isFinite(x) ? '—' : `${nf(x * 100, 3)} %`);
const sec = x => x.toFixed(3);
const store = {
  get: k => { try { return localStorage.getItem(k); } catch { return null; } },
  set: (k, v) => { try { localStorage.setItem(k, v); } catch { /* private mode */ } },
};

// ---- colours: hue = contact method, shade and dash = engine ----
// OKLCH to sRGB, so that the shades of the engines are evenly spaced in lightness (the part of a
// colour every kind of colour vision reads).
function oklch(L, C, hueDeg) {
  const h = hueDeg * Math.PI / 180, a = C * Math.cos(h), b = C * Math.sin(h);
  const l = (L + 0.3963377774 * a + 0.2158037573 * b) ** 3;
  const m = (L - 0.1055613458 * a - 0.0638541728 * b) ** 3;
  const s = (L - 0.0894841775 * a - 1.291485548 * b) ** 3;
  const lin = [4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s,
    -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s,
    -0.0041960863 * l - 0.7034186147 * m + 1.707614701 * s];
  return '#' + lin.map(x => {
    const v = x <= 0.0031308 ? 12.92 * x : 1.055 * x ** (1 / 2.4) - 0.055;
    return Math.round(255 * Math.min(Math.max(v, 0), 1)).toString(16).padStart(2, '0');
  }).join('');
}
const HUES = [255, 50, 350, 160, 300, 105, 20, 205];   // blue, orange, magenta, teal, purple, olive, red, cyan
const DASHES = [[], [7, 4], [2, 3], [10, 3, 2, 3], [14, 4]];
let engines = [], methods = [];   // every engine and contact method of the index, sorted
function styleOf(engine, contact) {
  const e = Math.max(engines.indexOf(engine), 0), c = Math.max(methods.indexOf(contact), 0);
  const L = engines.length < 2 ? 0.58 : 0.45 + 0.33 * e / (engines.length - 1);
  return { color: oklch(L, L > 0.7 ? 0.11 : 0.15, HUES[c % HUES.length]), dash: DASHES[e % DASHES.length] };
}
// A sample of the line a run is drawn with in the plots.
const swatch = (color, dash, width = 2) =>
  `<svg class="sw" viewBox="0 0 30 10" aria-hidden="true"><line x1="1" y1="5" x2="29" y2="5" stroke="${color}" stroke-width="${width}" stroke-dasharray="${dash.join(' ')}"/></svg>`;

// ---- 3D ----
const renderer = new THREE.WebGLRenderer({ antialias: true });
sceneEl.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(PANEL);
scene.add(new THREE.HemisphereLight(0xffffff, 0x8a929a, 1.7));
const sun = new THREE.DirectionalLight(0xffffff, 1.3);
sun.position.set(3, -5, 10);
scene.add(sun);
const camera = new THREE.PerspectiveCamera(40, 1, 0.001, 1e5);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.15;
const world = new THREE.Group();   // ground and runs of the current selection
scene.add(world);

const objCache = new Map(), edgeCache = new Map();
function loadObj(path) {
  if (!objCache.has(path)) {
    objCache.set(path, fetch(ASSETS + path).then(r => {
      if (!r.ok) throw new Error(`${path}: ${r.status}`);
      return r.text();
    }).then(text => new OBJLoader().parse(text)));
  }
  return objCache.get(path);
}
// Every body and ground is drawn the same way: translucent faces plus the edges between faces
// that are not coplanar; the rendering switch shows one, the other or both.
let renderMode = RENDER_MODES[store.get('vc-render')] ? store.get('vc-render') : 'edges';
function paint(obj, color, role, layer = 0) {
  const group = new THREE.Group();
  obj.traverse(o => {
    if (!o.isMesh) return;
    const face = new THREE.Mesh(o.geometry, new THREE.MeshStandardMaterial({
      color, flatShading: true, side: THREE.DoubleSide, roughness: 0.8, metalness: 0, transparent: true,
      // Faces sit behind their edges, and each run a little behind the one before: where opaque
      // runs coincide the first one shows, instead of a flicker between them.
      polygonOffset: true, polygonOffsetFactor: 1 + layer, polygonOffsetUnits: 1 + 4 * layer,
    }));
    face.userData = { kind: 'face', role };
    if (!edgeCache.has(o.geometry)) edgeCache.set(o.geometry, new THREE.EdgesGeometry(o.geometry, 1));
    const edge = new THREE.LineSegments(edgeCache.get(o.geometry), new THREE.LineBasicMaterial({ color: role === 'ground' ? 0x9aa2aa : color }));
    edge.userData = { kind: 'edge', role };
    group.add(face, edge);
  });
  return group;
}
// World axes in the lower left corner, turning with the camera (x red, y green, z blue; z is up).
const axesScene = new THREE.Scene(), axesCamera = new THREE.OrthographicCamera(-1.5, 1.5, 1.5, -1.5, 0.1, 10);
axesCamera.up.set(0, 0, 1);
for (const [name, dir, color] of [['x', [1, 0, 0], '#C8372D'], ['y', [0, 1, 0], '#2E8B3C'], ['z', [0, 0, 1], '#1F5FBF']]) {
  const v = new THREE.Vector3(...dir);
  axesScene.add(new THREE.ArrowHelper(v, new THREE.Vector3(), 1, color, 0.28, 0.16));
  const canvas = Object.assign(document.createElement('canvas'), { width: 64, height: 64 });
  const ctx = canvas.getContext('2d');
  Object.assign(ctx, { font: '500 44px Barlow, sans-serif', textAlign: 'center', textBaseline: 'middle', fillStyle: color });
  ctx.fillText(name, 32, 30);
  const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }));
  label.position.copy(v).multiplyScalar(1.28);
  label.scale.setScalar(0.62);
  axesScene.add(label);
}
function drawAxes() {
  const size = 112;
  axesCamera.position.copy(camera.position).sub(controls.target).setLength(4);
  axesCamera.lookAt(0, 0, 0);
  renderer.autoClear = false;
  renderer.clearDepth();
  renderer.setViewport(6, 6, size, size);
  renderer.render(axesScene, axesCamera);
  renderer.autoClear = true;
}
function applyRenderMode() {
  world.traverse(o => {
    const { kind, role } = o.userData;
    if (kind === 'edge') o.visible = renderMode !== 'solid';
    if (kind === 'face') {
      o.visible = renderMode !== 'wire';
      // Overlaid runs stay readable through each other; alone, a solid reads better opaque.
      const opaque = renderMode === 'solid';
      o.material.opacity = role === 'ground' ? (opaque ? 0.9 : 0.4) : opaque ? 0.85 : 0.45;
      o.material.depthWrite = role === 'ground' || opaque;
    }
  });
  document.querySelectorAll('#render button').forEach(b => b.setAttribute('aria-checked', String(b.dataset.mode === renderMode)));
}

// ---- state ----
let index = {};          // scenario -> {description, class, variants: {variant: {segments, dt, perturbed, runs}}}
let entries = [];        // runs of the selection: {id, engine, contact, color, dash, state, why, data, group, speed, ...}
const hidden = new Set(store.get('vc-hidden')?.split(',').filter(Boolean) ?? []);   // run ids, kept across scenarios
let scenario = null, variant = null;
let time = 0, duration = 1, refT = [0];
let playing = false, speed = 1, drawnTime = -1;  // Never starts on its own: the user presses play.
let plots = [];
const fullRange = new Set();   // series whose plot shows the whole range of every curve
let sort = { key: 'contact', dir: 1 }, showOther = false;
let selection = 0;       // counts selections, to drop answers of an older one

const okEntries = () => entries.filter(e => e.state === 'ok');
const shownEntries = () => entries.filter(e => e.state === 'ok' && !hidden.has(e.id));

const finitePose = p => p.length === 7 && p.every(Number.isFinite);
function validate(d) {
  if (!d || !Array.isArray(d.t) || !d.t.length || !Array.isArray(d.bodies)) throw new Error('malformed file');
  if (!d.t.every(Number.isFinite)) throw new Error('not a number in the time');
  for (const b of d.bodies) if (!b.poses.every(finitePose)) throw new Error('not a number in the trajectory');
}

const runCache = new Map();   // url -> promise of the validated run; a trajectory is fetched when first shown
function fetchRun(url) {
  if (!runCache.has(url)) {
    runCache.set(url, fetch(url).then(r => {
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      return r.json();
    }).then(d => { validate(d); return d; }));
  }
  return runCache.get(url);
}

async function loadEntry(run, layer) {
  const entry = { ...run, ...styleOf(run.engine, run.contact), state: 'unavailable', why: 'no data' };
  // A run the battery recorded without a trajectory (timeout, failure) says what happened.
  if (run.status && run.status !== 'ok' && STATUSES[run.status]) entry.why = STATUSES[run.status].why;
  try {
    if (run.file === false) throw new Error('no trajectory file');
    const data = await fetchRun(`${TRAJ}${scenario}/${variant}/${run.id}.json`);
    entry.group = new THREE.Group();
    entry.meshes = [];
    entry.radius = 0;
    for (const b of data.bodies) {
      const mesh = paint(await loadObj(b.surface), entry.color, 'body', layer);
      entry.radius = Math.max(entry.radius, new THREE.Box3().setFromObject(mesh).getBoundingSphere(new THREE.Sphere()).radius);
      entry.meshes.push(mesh);
      entry.group.add(mesh);
    }
    entry.data = data;
    entry.state = 'ok';
  } catch (e) {
    if (!(run.status && run.status !== 'ok' && STATUSES[run.status])) entry.why = `no data (${e.message ?? e})`;
  }
  return entry;
}

// ---- pose at time ----
const q0 = new THREE.Quaternion(), q1 = new THREE.Quaternion();
function locate(ts, t) {
  if (t <= ts[0]) return [0, 0];
  if (t >= ts[ts.length - 1]) return [ts.length - 1, 0];
  let lo = 0, hi = ts.length - 1;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (ts[mid] <= t) lo = mid; else hi = mid; }
  return [lo, (t - ts[lo]) / (ts[lo + 1] - ts[lo])];
}
function poseMesh(mesh, a, b, f) {
  mesh.position.set(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]));
  q0.set(a[4], a[5], a[6], a[3]);
  q1.set(b[4], b[5], b[6], b[3]);
  mesh.quaternion.copy(q0).slerp(q1, f);
}
const median = a => a.sort((x, y) => x - y)[a.length >> 1];
// Poses every shown run at the current time. The anchor the camera rides on is the median over
// the runs of the mean position of their bodies, so a run that flies away does not drag the view.
const anchor = new THREE.Vector3();
let anchored = false;
function applyTime() {
  const means = [[], [], []];
  for (const e of shownEntries()) {
    const [i, f] = locate(e.data.t, time);
    const j = Math.min(i + 1, e.data.t.length - 1);
    const mean = [0, 0, 0];
    e.data.bodies.forEach((b, k) => {
      const mesh = e.meshes[k];
      poseMesh(mesh, b.poses[i], b.poses[j], f);
      mean[0] += mesh.position.x; mean[1] += mesh.position.y; mean[2] += mesh.position.z;
    });
    if (e.meshes.length) mean.forEach((v, k) => means[k].push(v / e.meshes.length));
  }
  anchored = means[0].length > 0;
  if (anchored) anchor.set(...means.map(median));
}

// ---- quantities and units ----
// Unit of a series or scalar from its name: an explicit suffix first, then the quantity. Empty
// when the name does not tell (the bench output has no units).
function unitOf(name) {
  const n = name.toLowerCase().replace('_theory', '');
  if (/_(radps|rad_s)$/.test(n)) return 'rad/s';
  if (/_(mps|m_s)$/.test(n)) return 'm/s';
  if (/_rad$/.test(n)) return 'rad';
  if (/_m$/.test(n)) return 'm';
  if (/ratio|error|decay|gain|energy_drift|^err_/.test(n)) return '';
  if (/time|period|(^|_)t(_|$)/.test(n)) return 's';
  if (/energy/.test(n)) return 'J';
  if (/(^|_)(w|omega|spin|rate)(_|$)/.test(n)) return 'rad/s';
  if (/(^|_)(v|vx|vy|vz|wr|velocity|speed|rebound)(_|$)/.test(n)) return 'm/s';
  if (/angle|theta|tilt|rotation|pitch|roll|amplitude|turn/.test(n)) return 'rad';
  if (/depth|delta|penetration|sink|creep|slip|slide|lateral|compression|travel|displacement|radius|shift|drift/.test(n)) return 'm';
  return '';
}
function quantityOf(name) {
  const [head, ...rest] = name.split('_');
  return QUANTITIES[head] ? QUANTITIES[head] + (rest.length ? ` (${rest.join(' ')})` : '') : words(name).replace(/^./, c => c.toUpperCase());
}

// ---- plots ----
function interp(xs, ys, x) {
  const [i, f] = locate(xs, x), j = Math.min(i + 1, xs.length - 1);
  const a = ys[i], b = ys[j];
  return a == null || b == null ? null : a + f * (b - a);
}

// A y range that fits the theory and the curves in a robust way: the theory's whole range, plus
// the 2nd-98th percentile of each curve that is not wildly wider than it. Returns [lo, hi, entries
// that leave it]. With `full`, the whole range of everything finite.
const finite = a => a.filter(Number.isFinite);
function quantile(sorted, q) {
  const x = q * (sorted.length - 1), i = Math.floor(x);
  return sorted[i] + (x - i) * ((sorted[Math.min(i + 1, sorted.length - 1)] ?? sorted[i]) - sorted[i]);
}
function yRange(theory, curves, full) {
  const span = a => (a.length ? a[a.length - 1] - a[0] : 0);
  const sorted = curves.map(c => ({ ...c, v: finite(c.v).sort((x, y) => x - y) })).filter(c => c.v.length);
  const th = theory ? finite(theory).sort((x, y) => x - y) : [];
  let lo = Infinity, hi = -Infinity;
  const take = (a, b) => { lo = Math.min(lo, a); hi = Math.max(hi, b); };
  if (full) {
    [th, ...sorted.map(c => c.v)].filter(a => a.length).forEach(a => take(a[0], a[a.length - 1]));
  } else {
    const robust = sorted.map(c => [quantile(c.v, 0.02), quantile(c.v, 0.98)]);
    if (th.length) take(th[0], th[th.length - 1]);
    const spans = robust.map(r => r[1] - r[0]).sort((x, y) => x - y);
    const ref = Math.max(th.length ? span(th) : 0, spans[spans.length >> 1] ?? 0);   // theory or the typical curve
    robust.forEach(([a, b]) => { if (b - a <= 4 * Math.max(ref, 1e-12)) take(a, b); });
    if (lo > hi) robust.forEach(([a, b]) => take(a, b));
  }
  if (!(lo <= hi)) return [-1, 1, []];
  const pad = (hi - lo) * 0.08 || Math.max(Math.abs(hi) * 0.1, 1e-9);
  lo -= pad; hi += pad;
  const off = sorted.filter(c => c.v[0] < lo || c.v[c.v.length - 1] > hi).map(c => c.entry);
  return [lo, hi, off];
}

function resizePlots() {
  for (const p of plots) {
    const h = p.root.parentElement;
    if (h.clientHeight && (p.height !== h.clientHeight || p.width !== h.clientWidth)) p.setSize({ width: h.clientWidth, height: h.clientHeight });
  }
  drawnTime = -1;
}

// The time cursor and, on the theory curve, its name.
function drawOverlay(u, theoryIndex) {
  const c = u.ctx, r = uPlot.pxRatio, { left, top, width, height } = u.bbox;
  c.save();
  if (theoryIndex) {
    // At the highest point of the curve; among equals, the one nearest to 60 % of the time.
    const xs = u.data[0], ys = u.data[theoryIndex], want = xs.length * 0.6;
    let i = -1;
    ys.forEach((y, k) => {
      if (y != null && (i < 0 || y > ys[i] + 1e-12 || (y > ys[i] - 1e-12 && Math.abs(k - want) < Math.abs(i - want)))) i = k;
    });
    if (i >= 0) {
      const x = u.valToPos(xs[i], 'x', true), y = u.valToPos(ys[i], 'y', true);
      // Beside the curve at either end of the plot, above it elsewhere (below when there is no room).
      const side = x < left + 40 * r ? 1 : x > left + width - 40 * r ? -1 : 0, above = y - top > 26 * r;
      c.font = `500 ${13 * r}px Barlow, sans-serif`;
      c.textAlign = side > 0 ? 'left' : side < 0 ? 'right' : 'center';
      c.textBaseline = side ? 'middle' : above ? 'bottom' : 'top';
      c.lineWidth = 4 * r; c.strokeStyle = PANEL; c.lineJoin = 'round'; c.fillStyle = INK;
      const tx = x + side * 10 * r;
      const ty = Math.min(Math.max(y, top + 8 * r), top + height - 8 * r) + (side ? 0 : above ? -6 : 6) * r;
      c.strokeText('Theory', tx, ty);
      c.fillText('Theory', tx, ty);
    }
  }
  const x = u.valToPos(time, 'x', true);
  c.strokeStyle = INK; c.lineWidth = r; c.setLineDash([3 * r, 3 * r]);
  c.beginPath(); c.moveTo(x, top); c.lineTo(x, top + height); c.stroke();
  c.restore();
}

function buildPlots() {
  for (const p of plots) p.destroy();
  plots = [];
  plotsEl.innerHTML = '';
  const ok = okEntries();
  if (!ok.length) return;
  const shown = shownEntries();
  const groups = [...new Set(ok.flatMap(e => Object.keys(e.data.series ?? {})).map(n => n.replace(/_theory$/, '')))]
    .filter(g => ok.some(e => e.data.series[g]));
  const xs = refT;
  groups.forEach((g, k) => {
    const series = [{}], data = [xs];
    const curves = [];
    for (const e of shown) {
      const v = e.data.series[g];
      if (!v) continue;
      series.push({ stroke: e.color, width: 1.5, dash: e.dash.length ? e.dash : undefined, points: { show: false } });
      const iv = xs.map(x => interp(e.data.t, v, x));
      data.push(iv);
      curves.push({ entry: e, v: iv });
    }
    // The theory goes last, so that it is drawn over every run: a light halo, then the thick dark line.
    const source = ok.find(e => e.data.series[g + '_theory']?.some(v => v != null));
    const tv = source ? xs.map(x => interp(source.data.t, source.data.series[g + '_theory'], x)) : null;
    if (tv) {
      series.push({ stroke: PANEL, width: 6.5, alpha: 0.85, points: { show: false } }, { stroke: INK, width: 3, points: { show: false } });
      data.push(tv, tv);
    }
    const [lo, hi, off] = yRange(tv, curves, fullRange.has(g));
    const unit = unitOf(g);

    const head = el('div', { className: 'plot-head' },
      el('span', { className: 'q', textContent: quantityOf(g), title: `Series "${g}" of the runs` }),
      el('span', { className: 'unit', textContent: unit ? `[${unit}]` : '' }),
      el('span', { className: 'ref', innerHTML: tv ? `${swatch(INK, [], 3.5)}Theory` : 'no theory curve for this quantity' }));
    if (off.length || fullRange.has(g)) {
      const chip = el('button', {
        className: 'offscale',
        title: fullRange.has(g) ? 'Back to the range of the theory' : `${off.map(e => `${nameOf(e.engine)} ${e.contact}`).join(', ')}: beyond this range. Click to show the whole range.`,
        innerHTML: fullRange.has(g) ? 'Whole range shown' : `Off scale ${off.map(e => swatch(e.color, e.dash)).join('')}`,
      });
      chip.onclick = () => { if (!fullRange.delete(g)) fullRange.add(g); buildPlots(); };
      head.append(chip);
    }
    const canvas = el('div', { className: 'plot-canvas' });
    plotsEl.append(el('div', { className: 'plot' }, head, canvas));

    const axis = { stroke: MUTED, font: '12px Barlow', grid: { stroke: GRID, width: 1 }, ticks: { stroke: GRID, size: 4 } };
    const last = k === groups.length - 1;
    const u = new uPlot({
      width: canvas.clientWidth || 300, height: canvas.clientHeight || 120, legend: { show: false }, series,
      cursor: { show: false }, select: { show: false }, padding: [8, 12, 0, 0],
      scales: { x: { time: false, range: () => [xs[0], xs[xs.length - 1]] }, y: { range: () => [lo, hi] } },
      axes: [
        { ...axis, size: last ? 40 : 24, label: last ? 'Time [s]' : undefined, labelFont: '12px Barlow', labelSize: 16, values: (_, splits) => splits.map(v => nf(v, 6)) },
        { ...axis, size: 58, values: (_, splits) => splits.map(v => nf(v, 5)) },
      ],
      hooks: { draw: [p => drawOverlay(p, tv ? series.length - 1 : 0)] },
    }, data, canvas);
    // Click or drag on a plot to move the time.
    const seekTo = ev => {
      const rect = u.over.getBoundingClientRect();
      setTime(u.posToVal(ev.clientX - rect.left, 'x'));
    };
    u.over.addEventListener('pointerdown', ev => { u.over.setPointerCapture(ev.pointerId); playing = false; syncButtons(); seekTo(ev); });
    u.over.addEventListener('pointermove', ev => { if (ev.buttons & 1) seekTo(ev); });
    plots.push(u);
  });
  resizePlots();
}

// ---- table: legend, visibility and metrics ----
const EYE = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M2 12s3.6-6.5 10-6.5S22 12 22 12s-3.6 6.5-10 6.5S2 12 2 12z" fill="none" stroke="currentColor" stroke-width="1.8"/><circle cx="12" cy="12" r="3"/><path class="slash" d="M4 4l16 16" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/></svg>';

// The scalars of the runs, in three groups: the terms of the error (err_<name>), the measured
// values that have a theory (<x> with <x>_theory, in either word order), and the rest.
function scalarGroups(ok) {
  const keys = [...new Set(ok.flatMap(e => Object.keys(e.data.scalars ?? {})))];
  const theoryOf = {};
  for (const k of keys) {
    const base = k.replace('_theory', '');
    if (k.includes('_theory') && keys.includes(base)) theoryOf[base] = k;
  }
  const terms = keys.filter(k => k.startsWith('err_'));
  const paired = keys.filter(k => k in theoryOf);
  const used = new Set([...terms, ...paired, ...Object.values(theoryOf)]);
  return { terms, paired, theoryOf, other: keys.filter(k => !used.has(k)) };
}
const scalar = (e, k) => e.data?.scalars?.[k] ?? null;
// The error cell: the status the battery recorded when the run did not end in a number.
const failedStatus = e => (e.status && e.status !== 'ok' ? e.status : (e.state === 'ok' && !Number.isFinite(e.data?.error) ? 'nan' : null));
const errorText = e => { const st = failedStatus(e); return st ? (STATUSES[st]?.label ?? st) : pct(e.data?.error); };
function speedText(e) {
  if (!(e.speed > 0)) return '—';
  const v = e.speed >= 100 ? Math.round(e.speed).toString() : e.speed.toLocaleString('en', { maximumSignificantDigits: 2, useGrouping: false });
  return (e.speed_source === 'timing' ? '' : '≈ ') + '×' + v;
}
const speedTitle = e => (!(e.speed > 0) ? 'No timing for this run'
  : `${e.speed >= 1 ? `${nf(e.speed, 3)} times faster than real time` : `${nf(1 / e.speed, 3)} times slower than real time`}. `
    + (e.speed_source === 'timing' ? 'From the timing pass on an idle machine.' : 'Approximate: from the accuracy pass, which runs several scenarios at once.'));

function setHidden(ids, hide) {
  for (const id of ids) if (hide) hidden.add(id); else hidden.delete(id);
  store.set('vc-hidden', [...hidden].join(','));
  for (const e of okEntries()) e.group.visible = !hidden.has(e.id);
  renderTable();
  buildPlots();
  applyTime();
}
function solo(entry) {
  const others = okEntries().filter(e => e !== entry).map(e => e.id);
  const alone = !hidden.has(entry.id) && others.every(id => hidden.has(id));
  if (alone) return setHidden(others, false);
  hidden.delete(entry.id);
  setHidden(others, true);
}

function renderTable() {
  const ok = okEntries();
  const { terms, paired, theoryOf, other } = scalarGroups(ok);
  const theoryValue = k => ok.map(e => scalar(e, k)).find(v => v != null) ?? null;
  const cols = [
    { key: 'engine', label: 'Engine', group: 'run', text: e => nameOf(e.engine), value: e => e.engine },
    { key: 'contact', label: 'Contact method', group: 'run', text: e => nameOf(e.contact), value: e => e.contact },
    { key: 'speed', label: 'Compute speed', group: 'run', num: true, text: speedText, value: e => e.speed ?? null, cellTitle: speedTitle,
      title: 'Simulated time over computing time: ×200 runs 200 times faster than real time, ×0.1 ten times slower. ≈ marks a value taken from the parallel accuracy pass instead of the timing pass.' },
    { key: 'error', label: terms.length ? 'Worst' : 'Error', group: 'error', num: true, text: errorText, value: e => (failedStatus(e) ? null : e.data?.error ?? null),
      title: 'Normalized deviation from the theory' + (terms.length ? ': the worst of the terms to its right' : '') },
    ...terms.map(k => ({ key: k, label: words(k.slice(4)), group: 'error', num: true, text: e => pct(scalar(e, k)), value: e => scalar(e, k),
      worst: e => scalar(e, k) != null && scalar(e, k) === Math.max(...terms.map(t => scalar(e, t) ?? -1)) })),
    ...paired.map(k => ({ key: k, label: words(k.replace(/_(m|rad|mps|radps|m_s|rad_s)$/, '')), unit: unitOf(k), group: 'measured', num: true, text: e => nf(scalar(e, k)), value: e => scalar(e, k), theory: theoryValue(theoryOf[k]) })),
    ...(showOther ? other : []).map(k => ({ key: k, label: words(k.replace(/_(m|rad|mps|radps|m_s|rad_s)$/, '')), unit: unitOf(k), group: 'other', num: true, text: e => nf(scalar(e, k)), value: e => scalar(e, k),
      theory: k.includes('theory') ? theoryValue(k) : null })),
  ];
  const groupLabels = { run: '', error: 'Error', measured: 'Measured', other: 'Other scalars' };
  const groupTitles = { error: 'Normalized deviation from the theory: the columns below are its terms', measured: 'Measured quantities; the Theory row gives the value each should have', other: 'Other scalars of the runs' };

  const head = $('runs').tHead, body = $('runs').tBodies[0];
  head.innerHTML = '';
  body.innerHTML = '';
  const top = head.insertRow(), bottom = head.insertRow();
  top.className = 'groups';
  const anyHidden = ok.some(e => hidden.has(e.id));
  const all = el('button', { className: 'eye', innerHTML: EYE, title: anyHidden ? 'Show every run' : 'Every run is shown', disabled: !anyHidden });
  all.setAttribute('aria-label', 'Show every run');
  all.onclick = () => setHidden(ok.map(e => e.id), false);
  top.append(el('th', { rowSpan: 2, className: 'vis' }, el('div', {}, all)));
  for (const g of Object.keys(groupLabels)) {
    const n = cols.filter(c => c.group === g).length;
    if (!n) continue;
    if (g === 'run') {   // engine and contact method stay in view: their part of the header too
      top.append(el('th', { colSpan: 2, className: 'g-run pin' }));
      if (n > 2) top.append(el('th', { colSpan: n - 2, className: 'g-run' }));
    } else {   // a heading over its columns, bracketed by a rule; a group of one column needs none
      const th = el('th', { colSpan: n, className: `g-${g} span`, title: groupTitles[g] ?? '' });
      if (n > 1) th.append(el('span', { textContent: groupLabels[g] }));
      top.append(th);
    }
  }
  for (const c of cols) {
    const th = el('th', { className: `${c.num ? 'num ' : ''}g-${c.group} c-${c.key}`, title: c.title ?? '' });
    const b = el('button', { className: 'sort', innerHTML: `${c.label}${c.unit ? ` <span class="unit">[${c.unit}]</span>` : ''}` });
    if (sort.key === c.key) th.setAttribute('aria-sort', sort.dir > 0 ? 'ascending' : 'descending');
    b.onclick = () => { sort = { key: c.key, dir: sort.key === c.key ? -sort.dir : 1 }; renderTable(); };
    th.append(b);
    bottom.append(th);
  }

  // The theory: pinned first, whatever the order of the runs.
  const theoryRow = body.insertRow();
  theoryRow.className = 'theory';
  theoryRow.append(el('td', { className: 'vis' }),
    el('td', { colSpan: 2, className: 'c-engine', innerHTML: `<span class="who">${swatch(INK, [], 3.5)}Theory</span>` }));
  for (const c of cols.slice(2)) theoryRow.append(el('td', { className: `${c.num ? 'num ' : ''}g-${c.group}`, textContent: c.theory != null ? nf(c.theory) : '' }));

  // The fastest run and the most accurate one (ties within 0.1 % of the best share the mark).
  const errorOf = e => (failedStatus(e) ? null : e.data?.error ?? null);
  const speeds = ok.map(e => e.speed).filter(v => v > 0), errors = ok.map(errorOf).filter(v => v != null && isFinite(v));
  const bestSpeed = speeds.length > 1 ? Math.max(...speeds) : null, bestError = errors.length > 1 ? Math.min(...errors) : null;
  const best = { speed: e => bestSpeed != null && e.speed >= bestSpeed * 0.999 && 'Fastest',
                 error: e => bestError != null && errorOf(e) != null && errorOf(e) <= bestError * 1.001 + 1e-12 && 'Most accurate' };

  const by = cols.find(c => c.key === sort.key) ?? cols[1];
  const cmp = (a, b) => (a == null ? (b == null ? 0 : 1) : b == null ? -1 : typeof a === 'string' ? a.localeCompare(b) : a - b);
  const rows = [...entries].sort((a, b) => sort.dir * cmp(by.value(a), by.value(b)) || cmp(a.contact, b.contact) || cmp(a.engine, b.engine));
  for (const e of rows) {
    const isOk = e.state === 'ok', off = hidden.has(e.id);
    const tr = body.insertRow();
    tr.className = isOk ? (off ? 'off' : '') : 'missing';
    tr.title = isOk ? '' : e.why;
    const eye = el('button', { className: 'eye', innerHTML: EYE, disabled: !isOk, title: off ? 'Show in the 3D view and the plots' : 'Hide from the 3D view and the plots (Alt-click: show only this run)' });
    eye.setAttribute('aria-pressed', String(isOk && !off));
    eye.setAttribute('aria-label', `Show ${nameOf(e.engine)} ${e.contact}`);
    eye.onclick = ev => (ev.altKey ? solo(e) : setHidden([e.id], !off));
    const only = el('button', { className: 'only', textContent: 'only', disabled: !isOk, title: 'Show only this run; again to show every run' });
    only.onclick = () => solo(e);
    tr.append(el('td', { className: 'vis' }, el('div', {}, eye, only)));
    for (const c of cols) {
      const td = el('td', { className: `${c.num ? 'num ' : ''}g-${c.group} c-${c.key}${c.worst?.(e) ? ' worst' : ''}`, title: c.cellTitle?.(e) ?? '' });
      if (c.key === 'engine') td.innerHTML = `<span class="who">${swatch(e.color, e.dash, 2.5)}${c.text(e)}</span>`;
      else td.textContent = isOk || c.group === 'run' ? c.text(e) : c.key === 'error' ? errorText(e) : '';
      if (c.key === 'error' && failedStatus(e)) { td.classList.add('failedrun'); td.title = STATUSES[failedStatus(e)]?.why ?? ''; }
      const mark = isOk && best[c.key]?.(e);
      if (mark) td.prepend(el('span', { className: 'best', textContent: mark, title: `${mark} of the runs of this scenario and variant` }));
      tr.append(td);
    }
  }

  const note = $('tablenote');
  note.innerHTML = '';
  note.append('Colour is the contact method; shade and dash are the engine. Click a column to sort by it. ');
  if (other.length) {
    const more = el('button', { className: 'link', textContent: showOther ? 'Hide the other scalars' : `Show the other ${other.length} scalars of the runs` });
    more.onclick = () => { showOther = !showOther; renderTable(); };
    note.append(more);
  }
}

// ---- time ----
function updateDuration() {
  const ok = okEntries();
  duration = Math.max(1e-3, ...ok.map(e => e.data.t[e.data.t.length - 1]));
  refT = ok.length ? ok.reduce((a, b) => (b.data.t.length > a.data.t.length ? b : a)).data.t : [0];
  $('seek').max = duration;
  time = Math.min(time, duration);
}
function setTime(t) { time = Math.min(Math.max(t, 0), duration); }
function step(dir) {
  playing = false;
  const [i, f] = locate(refT, time);
  setTime(refT[Math.min(Math.max(f > 1e-6 && dir < 0 ? i : i + dir, 0), refT.length - 1)]);
  syncButtons();
}

// ---- ground and camera ----
const groundBox = new THREE.Box3();
let groundFrame = new THREE.Box3();   // the parts of the ground worth framing
async function addGround(grounds, bodiesBox) {
  groundFrame = new THREE.Box3();
  const diag = bodiesBox.getSize(new THREE.Vector3()).length();
  await Promise.all(grounds.map(async g => {
    const mesh = paint(await loadObj(g.surface), 0xaab1b8, 'ground');
    poseMesh(mesh, g.pose, g.pose, 0);
    mesh.updateMatrixWorld(true);
    groundBox.setFromObject(mesh);
    const size = groundBox.getSize(new THREE.Vector3());
    if (Math.max(size.x, size.y) <= 10) {
      world.add(mesh);
      // A feature (bowl, groove, hole, seat) is framed whole, unless it is far longer than the
      // motion: then only the stretch within reach of the bodies.
      const near = groundBox.clone();
      if (size.length() > 6 * diag) near.intersect(bodiesBox.clone().expandByScalar(Math.max(0.3, diag)));
      if (!near.isEmpty()) groundFrame.union(near);
      return;
    }
    // A slab too large to look at: a patch around the motion, with a grid (10 cm, or 1 m for very long runs).
    const c = bodiesBox.getCenter(new THREE.Vector3());
    const span = Math.max(...['x', 'y'].map(k => bodiesBox.getSize(new THREE.Vector3())[k]));
    const cell = span > 30 ? 1 : 0.1;
    const side = Math.max(Math.ceil((span * 1.3 + 0.4) / cell), 10) * cell;
    const patch = new THREE.Group();
    const plane = new THREE.Mesh(new THREE.PlaneGeometry(side, side), new THREE.MeshBasicMaterial({ color: 0xdde1e5, transparent: true }));
    plane.userData = { kind: 'face', role: 'ground' };
    patch.add(plane);
    const grid = new THREE.GridHelper(side, Math.round(side / cell), 0x7d858e, 0xb4bac1);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = 1e-4;
    grid.userData = { kind: 'edge', role: 'ground' };   // the grid is the slab's wireframe
    patch.add(grid);
    patch.position.set(Math.round(c.x / cell) * cell, Math.round(c.y / cell) * cell, groundBox.max.z - 2e-4);
    world.add(patch);
  }));
}

// Extent of the motion of the shown runs: per axis, the median over runs of each run's extent, so
// a run that blows up does not decide the framing. `margin` adds the size of the bodies.
function motionBox(margin) {
  const mins = [[], [], []], maxs = [[], [], []];
  for (const e of shownEntries()) {
    const box = new THREE.Box3(), pad = margin ? e.radius : 0;
    for (const b of e.data.bodies) {
      for (const p of b.poses) {
        box.expandByPoint(new THREE.Vector3(p[0] - pad, p[1] - pad, p[2] - pad));
        box.expandByPoint(new THREE.Vector3(p[0] + pad, p[1] + pad, p[2] + pad));
      }
    }
    [0, 1, 2].forEach(k => { mins[k].push(box.min.getComponent(k)); maxs[k].push(box.max.getComponent(k)); });
  }
  const box = new THREE.Box3();
  if (mins[0].length) {
    box.min.set(...mins.map(median));
    box.max.set(...maxs.map(median));
  }
  return box;
}
// A run whose bodies travel much further than their size would be a dot in a view of the whole
// path: then the view frames the bodies and rides along with them. Riding only translates the
// camera and its target together, so orbit, pan and zoom keep working meanwhile.
let riding = false;
const lastAnchor = new THREE.Vector3(), VIEW_DIR = new THREE.Vector3(1, -1.4, 0.9).normalize();
function bodiesSize() {   // radius of a sphere around the bodies of a typical run, at the current time
  const sizes = shownEntries().map(e => e.radius + Math.max(0, ...e.meshes.map(m => m.position.distanceTo(anchor))));
  return sizes.length ? median(sizes) : 0;
}
function frameCamera() {
  applyTime();
  const shown = shownEntries();
  if (!shown.length || !anchored) return;
  const path = motionBox(false), size = bodiesSize();
  riding = path.getSize(new THREE.Vector3()).length() > 6 * size;
  let c, radius;
  if (riding) {
    c = anchor.clone();
    radius = size * 2;
  } else {
    const box = motionBox(true).union(groundFrame);
    c = box.getCenter(new THREE.Vector3());
    radius = Math.max(box.getSize(new THREE.Vector3()).length() / 2, 0.05) * 1.05;
  }
  const dist = radius / Math.tan(THREE.MathUtils.degToRad(Math.min(camera.fov, camera.fov * camera.aspect) / 2)) * 0.95;
  camera.position.copy(c).addScaledVector(VIEW_DIR, dist);
  camera.near = dist / 200; camera.far = dist * 200;
  camera.updateProjectionMatrix();
  controls.target.copy(c);
  controls.enableDamping = false;   // drops what is left of the last drag, so the view lands where asked
  controls.update();
  controls.enableDamping = true;
  lastAnchor.copy(anchor);
}

// ---- selection ----
function showMessage(text) {
  $('message').hidden = !text;
  $('message').textContent = text ?? '';
}

function describe() {
  const d = DESCRIPTIONS[scenario] ?? {}, s = SCENARIOS[scenario] ?? [], info = index[scenario] ?? {};
  const fromData = okEntries().map(e => e.data).find(x => x.scenario === scenario) ?? {};
  $('setup').textContent = d.setup ?? fromData.description ?? info.description ?? '';
  $('tests').textContent = d.tests ?? '';
  $('theory').textContent = s[1] ? `Theory: ${s[1]}` : '';
  const cls = fromData.class ?? info.class ?? s[2] ?? null, badge = $('class');
  badge.hidden = !cls;
  if (cls) {
    const c = THEORY_CLASSES[cls];
    badge.innerHTML = `<b>Class ${cls}</b>${c ? ` ${c.short}` : ''}`;
    badge.dataset.tip = c?.long ?? `Class ${cls} of the theory.`;
    badge.setAttribute('aria-label', badge.dataset.tip);
  }
}

async function select() {
  const mine = ++selection;
  const q = new URLSearchParams({ scenario, variant });
  history.replaceState(null, '', `?${q}`);
  describe();
  const loaded = await Promise.all((index[scenario]?.variants[variant]?.runs ?? []).map(loadEntry));
  if (mine !== selection) return;
  world.clear();
  entries = loaded;
  for (const e of okEntries()) { world.add(e.group); e.group.visible = !hidden.has(e.id); }
  time = 0;
  const first = okEntries()[0];
  // A selection where every run is hidden would show nothing: show them all.
  if (first && !shownEntries().length) { hidden.clear(); okEntries().forEach(e => { e.group.visible = true; }); }
  showMessage(first ? null : 'No trajectories for this run. scripts/bench.sh writes them to results/traj.');
  describe();
  updateDuration();
  renderTable();
  buildPlots();
  applyTime();
  fillVariants();
  if (first) {
    await addGround(first.data.grounds, motionBox(true));
    if (mine !== selection) return;
    applyRenderMode();
    frameCamera();
  }
}

// The variants of the scenario, named in plain words, with what the data says of each (segments
// of its curved meshes, time step) and a line about the selected one.
function variantText(name) {
  const known = VARIANTS[name] ?? {}, info = index[scenario]?.variants[name] ?? {};
  const base = name === 'base' ? {} : index[scenario]?.variants.base ?? {};
  const facts = [];
  if (info.segments && info.segments !== base.segments) facts.push(`${info.segments} segments`);
  if (info.dt) { if (info.dt !== base.dt) facts.push(`${nf(info.dt * 1000)} ms`); } else if (known.detail) facts.push(known.detail);
  const label = known.label ?? words(name).replace(/^./, c => c.toUpperCase());
  let note = known.note ?? 'A variant of the run that this viewer has no description for.';
  const curved = Object.values(index[scenario]?.variants ?? {}).some(v => v.segments);
  if (name === 'fine' && index[scenario]?.variants.base && !curved && 'segments' in info && okEntries().length) {
    note += ' This scenario has no curved surface, so its meshes are those of the base run.';
  }
  return { label: facts.length ? `${label} (${facts.join(', ')})` : label, note };
}
function fillVariants() {
  const box = $('variants'), sub = $('variantsub'), names = Object.keys(index[scenario]?.variants ?? {});
  const order = Object.keys(VARIANTS);
  const rank = v => (order.indexOf(v) + 1 || 99);
  names.sort((a, b) => rank(a) - rank(b) || a.localeCompare(b, 'en', { numeric: true }));
  const groups = VARIANT_GROUPS.map(g => ({ ...g, members: names.filter(g.test) })).filter(g => g.members.length);
  const rest = names.filter(v => !groups.some(g => g.members.includes(v)));
  if (rest.length) groups.push({ id: 'other', label: 'Other', members: rest });
  const current = groups.find(g => g.members.includes(variant)) ?? groups[0];
  box.innerHTML = '';
  sub.innerHTML = '';
  for (const g of groups) {
    const b = el('button', { textContent: g.label });
    b.setAttribute('role', 'radio');
    b.setAttribute('aria-checked', String(g === current));
    b.onclick = () => { variant = g.members.includes(variant) ? variant : g.members[0]; fillVariants(); select(); };
    box.appendChild(b);
  }
  sub.hidden = !current || current.members.length < 2;
  for (const v of current?.members ?? []) {
    const b = el('button', { textContent: variantText(v).label });
    b.setAttribute('role', 'radio');
    b.setAttribute('aria-checked', String(v === variant));
    b.onclick = () => { variant = v; fillVariants(); select(); };
    sub.appendChild(b);
  }
  $('variantnote').textContent = variant ? variantText(variant).note : '';
}

function fitTitle() {   // the title is the selector: as wide as its text
  const sel = $('scenario'), cs = getComputedStyle(sel);
  const ctx = (fitTitle.ctx ??= document.createElement('canvas').getContext('2d'));
  ctx.font = `${cs.fontWeight} ${cs.fontSize} ${cs.fontFamily}`;
  sel.style.width = `${Math.ceil(ctx.measureText(sel.selectedOptions[0]?.textContent ?? '').width) + 46}px`;
}
document.fonts?.ready.then(fitTitle);   // measured again once the title's font has loaded
matchMedia('(max-width: 980px)').addEventListener('change', fitTitle);   // its size changes there
function chooseScenario(name) {
  scenario = name;
  $('scenario').value = name;
  fitTitle();
  const names = Object.keys(index[name]?.variants ?? {});
  if (!names.includes(variant)) variant = names.includes('base') ? 'base' : names[0];
  fillVariants();
  return select();
}

// ---- loop ----
let last = performance.now();
function tick(now) {
  requestAnimationFrame(tick);
  const dt = Math.min((now - last) / 1000, 0.1); last = now;
  if (playing && refT.length > 1) {
    time = Math.min(time + dt * speed, duration);
    if (time >= duration) { playing = false; syncButtons(); }   // stops at the end; play starts it again
  }
  const w = sceneEl.clientWidth, h = sceneEl.clientHeight;
  if (w && h && (renderer.domElement.width !== Math.round(w * devicePixelRatio) || renderer.domElement.height !== Math.round(h * devicePixelRatio))) {
    renderer.setPixelRatio(devicePixelRatio);
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  applyTime();
  if (riding && anchored) {
    const d = anchor.clone().sub(lastAnchor);
    camera.position.add(d); controls.target.add(d);
  }
  lastAnchor.copy(anchor);
  controls.update();
  renderer.setViewport(0, 0, w, h);
  renderer.render(scene, camera);
  drawAxes();
  if (time !== drawnTime) {
    for (const p of plots) p.redraw(false);
    drawnTime = time;
    $('seek').value = time;
    $('seek').style.setProperty('--done', `${100 * time / duration}%`);
    $('time').textContent = `${sec(time)} / ${sec(duration)} s`;
  }
}

// ---- wiring ----
function syncButtons() {
  $('play').classList.toggle('playing', playing);
  $('play').setAttribute('aria-label', playing ? 'Pause' : 'Play');
}
for (const [mode, label] of Object.entries(RENDER_MODES)) {
  const b = el('button', { textContent: label });
  b.dataset.mode = mode;
  b.setAttribute('role', 'radio');
  b.onclick = () => { renderMode = mode; store.set('vc-render', mode); applyRenderMode(); };
  $('render').appendChild(b);
}
$('speed').innerHTML = SPEEDS.map(s => `<option value="${s}"${s === speed ? ' selected' : ''}>${s}×</option>`).join('');
$('speed').onchange = e => { speed = parseFloat(e.target.value); };
$('scenario').onchange = e => chooseScenario(e.target.value);
$('play').onclick = () => { playing = !playing; if (playing && time >= duration) setTime(0); syncButtons(); };
$('restart').onclick = () => setTime(0);
$('prev').onclick = () => step(-1);
$('next').onclick = () => step(1);
$('seek').oninput = e => setTime(parseFloat(e.target.value));
$('reset').onclick = frameCamera;
renderer.domElement.addEventListener('dblclick', frameCamera);
controls.addEventListener('start', () => $('view').classList.add('used'));
addEventListener('keydown', e => {
  if (e.ctrlKey || e.metaKey || e.altKey || e.target.closest('select, textarea')) return;
  if (e.key === ' ' && e.target.closest('button')) return;
  if (e.key === ' ') $('play').click();
  else if (e.key === 'ArrowLeft') step(-1);
  else if (e.key === 'ArrowRight') step(1);
  else if (e.key === 'Home') setTime(0);
  else if (e.key === 'End') setTime(duration);
  else if (e.key === 'r' || e.key === 'R') frameCamera();
  else return;
  e.preventDefault();
});
new ResizeObserver(resizePlots).observe(plotsEl);
// What is on screen, for the checks that drive the page from a headless browser.
window.viewerState = () => ({
  scenario, variant, time, duration, riding,
  shown: shownEntries().map(e => e.id),
  bodiesVisible: okEntries().filter(e => e.group.visible).map(e => e.id),
  curves: plots.map(p => p.series.length - 1),
  camera: [...camera.position.toArray(), ...controls.target.toArray()],
});

// ---- index ----
// index.json of tools/viewer/index.py, or the older {scenario: {variant: [run id, ...]}}.
function normalise(raw) {
  const out = {};
  for (const [name, s] of Object.entries(raw.scenarios ?? raw)) {
    const variants = {};
    for (const [v, x] of Object.entries(s.variants ?? s)) {
      const runs = (x.runs ?? x).map(r => (typeof r === 'string' ? { id: r, engine: r.split('-')[0], contact: r.slice(r.indexOf('-') + 1) } : r));
      variants[v] = { ...(Array.isArray(x) ? {} : x), runs };
    }
    out[name] = { description: s.description ?? null, class: s.class ?? null, variants };
  }
  return out;
}
// An older index has no speeds: read them from the battery tables next to the trajectories.
async function legacySpeeds() {
  const ratios = new Map();   // run -> {battery: [speed, ...], timing: [speed, ...]}
  for (const [file, source] of [['battery.csv', 'battery'], ['timing.csv', 'timing']]) {
    const text = await fetch(`${TRAJ}../${file}`).then(r => (r.ok ? r.text() : ''), () => '');
    const [header, ...rows] = text.trim().split('\n').map(l => l.split(','));
    const col = n => header.indexOf(n);
    for (const row of rows) {
      const ratio = parseFloat(row[col('realtime_ratio')]);
      const run = index[row[col('scenario')]]?.variants[row[col('variant')]]?.runs.find(r => r.engine === row[col('engine')] && r.contact === row[col('contact')]);
      if (!run || !(ratio > 0)) continue;
      if (!ratios.has(run)) ratios.set(run, { battery: [], timing: [] });
      ratios.get(run)[source].push(1 / ratio);
    }
  }
  const median = a => { const v = [...a].sort((x, y) => x - y), m = v.length >> 1; return v.length % 2 ? v[m] : (v[m - 1] + v[m]) / 2; };
  for (const [run, r] of ratios) {   // the timing pass wins over the parallel accuracy pass
    const source = r.timing.length ? 'timing' : 'battery';
    Object.assign(run, { speed: median(r[source]), speed_source: source });
  }
}

let raw = {};
try {
  raw = await (await fetch(TRAJ + 'index.json')).json();
} catch (e) {
  raw = {};
}
index = normalise(raw);
if (!raw.version) await legacySpeeds();
const allRuns = Object.values(index).flatMap(s => Object.values(s.variants).flatMap(v => v.runs));
engines = [...new Set(allRuns.map(r => r.engine))].sort();
methods = [...new Set(allRuns.map(r => r.contact))].sort();

// The scenarios, grouped by the class of their theory when the data or scenarios.js tells it.
const names = Object.keys(index);
const titleOf = s => DESCRIPTIONS[s]?.title ?? SCENARIOS[s]?.[0] ?? words(s).replace(/^./, c => c.toUpperCase());
const classOf = s => index[s].class ?? SCENARIOS[s]?.[2] ?? '';
const option = s => `<option value="${s}">${titleOf(s)}</option>`;
const classes = [...new Set(names.map(classOf))].sort();
$('scenario').innerHTML = classes.length < 2 ? names.map(option).join('')
  : classes.map(c => `<optgroup label="${c ? `Class ${c}: ${THEORY_CLASSES[c]?.short ?? ''}` : 'Other scenarios'}">${names.filter(s => classOf(s) === c).map(option).join('')}</optgroup>`).join('');

const params = new URLSearchParams(location.search);   // e.g. ?scenario=bowl&variant=fine
await document.fonts.load('500 32px "Barlow Semi Condensed"');
await document.fonts.load('13px Barlow');
syncButtons();
applyRenderMode();
requestAnimationFrame(tick);
if (names.length) {
  variant = params.get('variant');
  await chooseScenario(names.includes(params.get('scenario')) ? params.get('scenario') : names[0]);
} else {
  showMessage('No trajectories yet. scripts/bench.sh writes them to results/traj.');
}
