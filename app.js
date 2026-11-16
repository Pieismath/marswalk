import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
const LINE_MATS = [];
function fatLine(pts, color, width, opacity = 1) {   // constant on-screen width at any zoom
  const g = new LineGeometry(); g.setPositions(pts.flatMap(p => [p.x, p.y, p.z]));
  const m = new LineMaterial({ color, linewidth: width, transparent: opacity < 1, opacity }); m.resolution.set(innerWidth, innerHeight); LINE_MATS.push(m);
  const l = new Line2(g, m); l.computeLineDistances(); return l;
}
const gltf = new GLTFLoader().setDRACOLoader(new DRACOLoader().setDecoderPath('https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/libs/draco/gltf/'));

// ---------- constants ----------
const HABITAT0 = { lat: 18.4447, lon: 77.4508 };          // Octavia E. Butler Landing (Perseverance)
const DELTA_DEMO = { lat: 18.5461, lon: 77.4206 };        // raised delta deposits ~6 km NNW of habitat, ~140 m higher (demo target)
const RAD_MSV_PER_DAY = 0.64;                              // MSL RAD surface dose equivalent, Hassler et al. 2014
const UNIT = 100;                                          // 1 scene unit = 100 m
const SOL_S = 88775.244;

const $ = (id) => document.getElementById(id);
const fmt = (x, d = 1) => Number(x).toFixed(d);

// ---------- state ----------
const S = {
  meta: null, z: null, W: 0, H: 0, dx: 0, dy: 0, slope: null,
  habitat: { ...HABITAT0 }, target: null, pickMode: 'target',
  ex: 2, haz: false, maxSlope: 15, eva: 7, sci: 1.5, sleep: 6.5, awake: 3, slam: false,
  storms: [], storm: null, lead: 3,
  out: null, back: null, shelter: null, plan: null,
};

// ---------- load data ----------
const [meta, dtmBuf, donki, rover, wmeta, wideBuf, trav] = await Promise.all([
  fetch('data/meta.json').then(r => r.json()),
  fetch('data/jezero_dtm_60m_f32.bin').then(r => r.arrayBuffer()),
  fetch('data/donki_snapshot.json').then(r => r.json()),
  fetch('data/rover_landing_imgs.json').then(r => r.json()),
  fetch('data/wide_meta.json').then(r => r.json()),
  fetch('data/wide_dem_400m_i16.bin').then(r => r.arrayBuffer()),
  fetch('data/m20_traverse.json').then(r => r.json()),
]);
S.meta = meta; S.W = meta.width; S.H = meta.height; S.z = new Float32Array(dtmBuf);
const midLat = (meta.lat_min + meta.lat_max) / 2;
S.dy = meta.cell_m;                                        // N-S cell size (m)
S.dx = meta.cell_m * Math.cos(midLat * Math.PI / 180);     // E-W cell size (equirectangular, true scale at equator)
computeSlope();

// ---------- geo helpers ----------
function ll2cr(lat, lon) {
  return { c: (lon - meta.lon_min) / (meta.lon_max - meta.lon_min) * S.W,
           r: (meta.lat_max - lat) / (meta.lat_max - meta.lat_min) * S.H };
}
function cr2ll(c, r) {
  return { lon: meta.lon_min + c / S.W * (meta.lon_max - meta.lon_min),
           lat: meta.lat_max - r / S.H * (meta.lat_max - meta.lat_min) };
}
function idx(c, r) { return r * S.W + c; }
function zAt(c, r) { // bilinear
  c = Math.min(Math.max(c, 0), S.W - 1.001); r = Math.min(Math.max(r, 0), S.H - 1.001);
  const c0 = Math.floor(c), r0 = Math.floor(r), fc = c - c0, fr = r - r0;
  const a = S.z[idx(c0, r0)], b = S.z[idx(c0 + 1, r0)], d = S.z[idx(c0, r0 + 1)], e = S.z[idx(c0 + 1, r0 + 1)];
  return (a * (1 - fc) + b * fc) * (1 - fr) + (d * (1 - fc) + e * fc) * fr;
}
function computeSlope() {
  const { W, H, z, dx, dy } = S; const s = new Float32Array(W * H);
  for (let r = 0; r < H; r++) for (let c = 0; c < W; c++) {
    const cl = Math.max(c - 1, 0), cr = Math.min(c + 1, W - 1), ru = Math.max(r - 1, 0), rd = Math.min(r + 1, H - 1);
    const gx = (z[idx(cr, r)] - z[idx(cl, r)]) / ((cr - cl) * dx);
    const gy = (z[idx(c, rd)] - z[idx(c, ru)]) / ((rd - ru) * dy);
    s[idx(c, r)] = Math.atan(Math.hypot(gx, gy)) * 180 / Math.PI;
  }
  S.slope = s;
}

// ---------- energy / speed models ----------
// Minetti et al. 2002, metabolic cost of walking vs gradient i (J kg^-1 m^-1)
const minetti = (i) => { i = Math.max(-0.45, Math.min(0.45, i)); return 280.5*i**5 - 58.7*i**4 - 76.8*i**3 + 51.9*i**2 + 19.6*i + 2.5; };
let MIN_C = Infinity; for (let i = -0.45; i <= 0.45; i += 0.005) MIN_C = Math.min(MIN_C, minetti(i));
// Tobler hiking function (km/h) x 0.7 suit factor (assumption)
const speedKmh = (grade) => 6 * Math.exp(-3.5 * Math.abs(grade + 0.05)) * 0.7;

// ---------- A* (energy-optimal, slope-constrained) ----------
function astar(start, goal, maxSlopeDeg) {
  const { W, H, z, dx, dy, slope } = S, N = W * H;
  const g = new Float32Array(N).fill(Infinity), came = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
  const heap = [], fv = new Float32Array(N);
  const push = (n) => { heap.push(n); let i = heap.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (fv[heap[p]] <= fv[heap[i]]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
  const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { const l = 2*i+1, r = l+1; let m = i; if (l < heap.length && fv[heap[l]] < fv[heap[m]]) m = l; if (r < heap.length && fv[heap[r]] < fv[heap[m]]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
  const s = idx(start.c, start.r), t = idx(goal.c, goal.r), tc = goal.c, tr = goal.r;
  const h = (c, r) => Math.hypot((c - tc) * dx, (r - tr) * dy) * MIN_C;
  const maxGrade = Math.tan(maxSlopeDeg * Math.PI / 180);
  g[s] = 0; fv[s] = h(start.c, start.r); push(s);
  const nb = [[1,0],[-1,0],[0,1],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]];
  while (heap.length) {
    const n = pop(); if (closed[n]) continue; closed[n] = 1;
    if (n === t) break;
    const c = n % W, r = (n / W) | 0;
    for (const [oc, or] of nb) {
      const c2 = c + oc, r2 = r + or; if (c2 < 0 || r2 < 0 || c2 >= W || r2 >= H) continue;
      const m = idx(c2, r2); if (closed[m]) continue;
      if (slope[m] > maxSlopeDeg && m !== t) continue;
      const d = Math.hypot(oc * dx, or * dy), dz = z[m] - z[n], grade = dz / d;
      if (Math.abs(grade) > maxGrade) continue;
      const cost = minetti(grade) * Math.hypot(d, dz);
      const ng = g[n] + cost;
      if (ng < g[m]) { g[m] = ng; came[m] = n; fv[m] = ng + h(c2, r2); push(m); }
    }
  }
  if (came[t] === -1 && s !== t) return null;
  const path = []; for (let n = t; n !== -1; n = came[n]) path.push({ c: n % W, r: (n / W) | 0 });
  path.reverse(); return path;
}

function pathStats(path) {
  let dist = 0, gain = 0, loss = 0, hours = 0, energy = 0, maxSl = 0; const cum = [0];
  for (let i = 1; i < path.length; i++) {
    const a = path[i-1], b = path[i];
    const d = Math.hypot((b.c - a.c) * S.dx, (b.r - a.r) * S.dy);
    const dz = S.z[idx(b.c, b.r)] - S.z[idx(a.c, a.r)];
    dist += d; if (dz > 0) gain += dz; else loss -= dz;
    hours += (d / 1000) / speedKmh(dz / d);
    energy += minetti(dz / d) * Math.hypot(d, dz);
    maxSl = Math.max(maxSl, S.slope[idx(b.c, b.r)]);
    cum.push(hours);
  }
  return { dist, gain, loss, hours, energy, maxSl, cum };
}

function straightLineCheck(a, b, maxSlopeDeg) {
  const n = Math.ceil(Math.hypot(b.c - a.c, b.r - a.r) * 2); let bad = 0, maxSl = 0;
  for (let i = 0; i <= n; i++) {
    const c = Math.round(a.c + (b.c - a.c) * i / n), r = Math.round(a.r + (b.r - a.r) * i / n);
    const s = S.slope[idx(c, r)]; maxSl = Math.max(maxSl, s); if (s > maxSlopeDeg) bad++;
  }
  return { fracBad: bad / (n + 1), maxSl, dist: Math.hypot((b.c - a.c) * S.dx, (b.r - a.r) * S.dy) };
}

// ---------- Mars24 local mean solar time ----------
function lmst(date, lonEast) {
  const jdUT = date.getTime() / 86400000 + 2440587.5;
  const jdTT = jdUT + (69.184) / 86400;                 // TT-UTC (37 leap s + 32.184)
  const msd = (jdTT - 2405522.0028779) / 1.0274912517;
  const mtc = ((24 * msd) % 24 + 24) % 24;
  return { sol: Math.floor(msd), h: ((mtc + lonEast / 15) % 24 + 24) % 24 };
}
const hhmm = (h) => `${String(Math.floor(h)).padStart(2,'0')}:${String(Math.floor((h % 1) * 60)).padStart(2,'0')}`;

// ---------- three.js scene ----------
const container = $('scene');
const renderer = new THREE.WebGLRenderer({ antialias: true, logarithmicDepthBuffer: true, preserveDrawingBuffer: true }); // metres-to-100-km zoom range
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;
container.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const SKY_CALM = new THREE.Color(0xc79670), SKY_STORM = new THREE.Color(0x5a0d0a);
scene.fog = new THREE.Fog(SKY_CALM.clone(), 450, 1800);
const camera = new THREE.PerspectiveCamera(48, innerWidth / innerHeight, 0.0005, 12000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.maxPolarAngle = Math.PI * 0.475; controls.minDistance = 0.03; controls.maxDistance = 1400;
scene.add(new THREE.HemisphereLight(0xffe2c8, 0x3a2418, 0.85));
const sun = new THREE.DirectionalLight(0xfff1e0, 1.7); sun.position.set(-300, 180, -160); scene.add(sun);
// sky dome: dusty butterscotch horizon fading to a darker zenith
const skyU = { top: { value: new THREE.Color(0x24120c) }, hor: { value: SKY_CALM.clone() }, sunDir: { value: sun.position.clone().normalize() } };
const sky = new THREE.Mesh(new THREE.SphereGeometry(5000, 64, 48), new THREE.ShaderMaterial({
  side: THREE.BackSide, depthWrite: false, depthTest: false, fog: false, uniforms: skyU,
  vertexShader: 'varying vec3 vP; void main(){ vP = normalize(position); vec4 p = projectionMatrix * modelViewMatrix * vec4(position,1.); gl_Position = vec4(p.xy, p.w * .9999, p.w); }',
  fragmentShader: `uniform vec3 top; uniform vec3 hor; uniform vec3 sunDir; varying vec3 vP;
    void main(){ vec3 n = normalize(vP); float h = max(n.y, 0.); vec3 c = mix(hor, top, pow(smoothstep(.06, .9, h), .7));
      float s = max(dot(n, sunDir), 0.); c += vec3(1.,.85,.65) * (pow(s, 600.) * 1.5 + pow(s, 12.) * .18);
      if (n.y < 0.) c = hor; gl_FragColor = vec4(c, 1.);
      #include <colorspace_fragment>
    }` }));
sky.renderOrder = -1; scene.add(sky);

const worldW = S.W * S.dx / UNIT, worldD = S.H * S.dy / UNIT;
const toWorld = (c, r, lift = 0) => { const x = (c / S.W - 0.5) * worldW, z = (r / S.H - 0.5) * worldD; return new THREE.Vector3(x, surfaceY(x, z) + lift, z); };
// height of the rendered terrain mesh itself (same triangles the GPU draws), so true-scale models sit exactly on the ground
function surfaceY(x, z) {
  const P = geo.attributes.position.array, u = THREE.MathUtils.clamp((x / worldW + 0.5) * SEG_X, 0, SEG_X - 1e-6), v = THREE.MathUtils.clamp((z / worldD + 0.5) * SEG_Z, 0, SEG_Z - 1e-6);
  const i = Math.floor(u), j = Math.floor(v), fu = u - i, fv = v - j, row = SEG_X + 1, Y = (ix, iy) => P[(iy * row + ix) * 3 + 1];
  const ya = Y(i, j), yb = Y(i, j + 1), yc = Y(i + 1, j + 1), yd = Y(i + 1, j);
  return fu + fv <= 1 ? ya + fu * (yd - ya) + fv * (yb - ya) : yc + (1 - fu) * (yb - yc) + (1 - fv) * (yd - yc);
}

// terrain mesh (half-res vertices)
const SEG_X = Math.floor(S.W / 2), SEG_Z = Math.floor(S.H / 2);
const geo = new THREE.PlaneGeometry(worldW, worldD, SEG_X, SEG_Z); geo.rotateX(-Math.PI / 2);
function applyHeights() {
  const p = geo.attributes.position;
  for (let j = 0; j <= SEG_Z; j++) for (let i = 0; i <= SEG_X; i++) {
    const k = j * (SEG_X + 1) + i; const c = i / SEG_X * (S.W - 1), r = j / SEG_Z * (S.H - 1);
    p.setY(k, (zAt(c, r) - meta.z_min) / UNIT * S.ex);
  }
  p.needsUpdate = true; geo.computeVertexNormals();
}
applyHeights();

// texture: CTX ortho (tinted) + hazard overlay on a canvas
const orthoImg = await new Promise((res, rej) => { const im = new Image(); im.onload = () => res(im); im.onerror = rej; im.src = 'data/jezero_ctx_ortho.jpg'; });
const TEXW = 2048, TEXH = Math.round(TEXW * orthoImg.height / orthoImg.width);
const texCanvas = document.createElement('canvas'); texCanvas.width = TEXW; texCanvas.height = TEXH;
const tctx = texCanvas.getContext('2d');
const hazCanvas = document.createElement('canvas'); hazCanvas.width = S.W; hazCanvas.height = S.H;
const tex = new THREE.CanvasTexture(texCanvas); tex.colorSpace = THREE.SRGBColorSpace; tex.anisotropy = 8;
function drawTexture() {
  tctx.globalCompositeOperation = 'source-over'; tctx.globalAlpha = 1;
  tctx.drawImage(orthoImg, 0, 0, TEXW, TEXH);
  tctx.globalCompositeOperation = 'multiply'; tctx.fillStyle = '#e0a27a'; tctx.fillRect(0, 0, TEXW, TEXH); // display tint
  tctx.globalCompositeOperation = 'source-over';
  if (S.haz) {
    const hc = hazCanvas.getContext('2d'), id = hc.createImageData(S.W, S.H);
    for (let k = 0; k < S.W * S.H; k++) {
      const s = S.slope[k]; let a = 0, R = 0, G = 0, B = 0;
      if (s > S.maxSlope) { R = 255; G = 60; B = 50; a = 170; }
      else if (s > S.maxSlope * 0.75) { R = 255; G = 200; B = 60; a = 95; }
      id.data.set([R, G, B, a], k * 4);
    }
    hc.putImageData(id, 0, 0);
    tctx.imageSmoothingEnabled = true; tctx.drawImage(hazCanvas, 0, 0, TEXW, TEXH);
  }
  tex.needsUpdate = true;
}
drawTexture();
const terrain = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ map: tex, roughness: 0.95, metalness: 0 }));
scene.add(terrain);

// ---------- surrounding region (240 km, HRSC-MOLA blend + Viking color) ----------
const MPD = 2 * Math.PI * 3396190 / 360, COSL = Math.cos(midLat * Math.PI / 180);
const lonSpan = meta.lon_max - meta.lon_min, latSpan = meta.lat_max - meta.lat_min;
const wX = (lon) => ((lon - meta.lon_min) / lonSpan - 0.5) * worldW;
let wideMesh = null;
const wZ = (lat) => ((meta.lat_max - lat) / latSpan - 0.5) * worldD;
{
  const WW = wmeta.width, WH = wmeta.height, dpp = wmeta.deg_per_px, wz = new Int16Array(wideBuf);
  const lonL = wmeta.lon_left + dpp / 2, latT = wmeta.lat_top - dpp / 2;
  const sx = wX(lonL), ex = wX(lonL + (WW - 1) * dpp), sz = wZ(latT), ez = wZ(latT - (WH - 1) * dpp);
  const g2 = new THREE.PlaneGeometry(ex - sx, ez - sz, WW - 1, WH - 1); g2.rotateX(-Math.PI / 2); g2.translate((sx + ex) / 2, 0, (sz + ez) / 2);
  const p = g2.attributes.position, BAND = 2500; // m blend band at the seam
  for (let j = 0; j < WH; j++) for (let i = 0; i < WW; i++) {
    const k = j * WW + i, lon = lonL + i * dpp, lat = latT - j * dpp;
    let z = wz[k];
    const c = (lon - meta.lon_min) / lonSpan * S.W, r = (meta.lat_max - lat) / latSpan * S.H;
    const ox = Math.max(0, -c, c - (S.W - 1)) * S.dx, oz = Math.max(0, -r, r - (S.H - 1)) * S.dy, o = Math.hypot(ox, oz);
    if (o === 0) z = zAt(c, r) - (Math.min(c, S.W - 1 - c, r, S.H - 1 - r) * S.dy > 1200 ? 40 : 0); // tucked under the high-res crater
    else if (o < BAND) { const t = o / BAND, w = t * t * (3 - 2 * t); z = zAt(c, r) * (1 - w) + z * w; }
    const curve = o * o / (2 * 3396190);                              // real Mars curvature beyond the crater zone
    p.setY(k, (z - meta.z_min) / UNIT * S.ex - curve / UNIT);
  }
  p.needsUpdate = true; g2.computeVertexNormals();
  const wtex = new THREE.TextureLoader().load('data/wide_tex.jpg'); wtex.colorSpace = THREE.SRGBColorSpace; wtex.anisotropy = 8;
  wideMesh = new THREE.Mesh(g2, new THREE.MeshStandardMaterial({ map: wtex, roughness: 1, metalness: 0, polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 8 })); scene.add(wideMesh);
}

// regional elevation sampler (for the cursor readout)
const WIDE = { W: wmeta.width, H: wmeta.height, dpp: wmeta.deg_per_px, z: new Int16Array(wideBuf) };
function wideZ(lat, lon) { const i = Math.round((lon - wmeta.lon_left) / WIDE.dpp - .5), j = Math.round((wmeta.lat_top - lat) / WIDE.dpp - .5);
  return WIDE.z[Math.min(WIDE.H - 1, Math.max(0, j)) * WIDE.W + Math.min(WIDE.W - 1, Math.max(0, i))]; }

// ---------- elevation colour ramp + contour lines (shader on both terrain meshes) ----------
const yOf = (m) => (m - meta.z_min) / UNIT * S.ex;
const elevU = { uElev: { value: 0 }, uImg: { value: 1 }, uMin: { value: yOf(-3200) }, uMax: { value: yOf(-1200) }, uCI: { value: 50 / UNIT * S.ex } };
function addElev(mat) {
  mat.onBeforeCompile = (sh) => {
    Object.assign(sh.uniforms, elevU);
    sh.vertexShader = 'varying float vH;\n' + sh.vertexShader.replace('#include <begin_vertex>', '#include <begin_vertex>\n  vH = position.y;');
    sh.fragmentShader = `uniform float uElev, uImg, uMin, uMax, uCI; varying float vH;
vec3 ramp(float t) { t = clamp(t, 0., 1.) * 7.;
  vec3 c0=vec3(.106,.165,.549), c1=vec3(.122,.471,.784), c2=vec3(.149,.71,.627), c3=vec3(.486,.804,.31), c4=vec3(.953,.851,.298), c5=vec3(.941,.541,.18), c6=vec3(.839,.235,.173), c7=vec3(1.);
  vec3 c = t<1.?mix(c0,c1,t):t<2.?mix(c1,c2,t-1.):t<3.?mix(c2,c3,t-2.):t<4.?mix(c3,c4,t-3.):t<5.?mix(c4,c5,t-4.):t<6.?mix(c5,c6,t-5.):mix(c6,c7,t-6.);
  return pow(c, vec3(2.2)); }
` + sh.fragmentShader.replace('#include <map_fragment>', `#include <map_fragment>
  float lum = dot(diffuseColor.rgb, vec3(.299,.587,.114));
  if (uImg < .5) diffuseColor.rgb = vec3(.32);
  if (uElev > .5) {
    vec3 rc = ramp((vH - uMin) / (uMax - uMin));
    diffuseColor.rgb = uImg > .5 ? rc * (.35 + 2.2 * lum) : rc;
    float f = vH / uCI; float d = abs(fract(f - .5) - .5) / max(fwidth(f), 1e-4);
    diffuseColor.rgb = mix(diffuseColor.rgb, vec3(1.), (1. - min(d, 1.)) * .5);
  }`);
  };
}
addElev(terrain.material); addElev(wideMesh.material);

// ---------- Perseverance traverse (NASA/JPL MMGIS) ----------
const travGroup = new THREE.Group(); scene.add(travGroup);
{
  const pts = trav.path.map(([lon, lat]) => { const p = ll2cr(lat, lon); return toWorld(p.c, p.r, 0.03); });
  travGroup.add(fatLine(pts, 0xffffff, 2.2, 0.9));
  const wp = trav.waypoints.filter((_, i) => i % 6 === 0).map(w => { const p = ll2cr(w.lat, w.lon); return toWorld(p.c, p.r, 0.04); });
  const g = new THREE.BufferGeometry().setFromPoints(wp);
  travGroup.add(new THREE.Points(g, new THREE.PointsMaterial({ color: 0xffffff, size: 3, sizeAttenuation: false })));
}
// cells Perseverance has actually driven across (±1 cell): ground proven traversable
const TRACK = new Uint8Array(S.W * S.H);
trav.path.forEach(([lon, lat]) => { const p = ll2cr(lat, lon), c0 = Math.round(p.c), r0 = Math.round(p.r);
  for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) { const c = c0 + dc, r = r0 + dr; if (c >= 0 && r >= 0 && c < S.W && r < S.H) TRACK[idx(c, r)] = 1; } });
const onTracks = (path) => path.reduce((n, q) => n + TRACK[idx(q.c, q.r)], 0) / path.length;
const roverDot = new THREE.Mesh(new THREE.SphereGeometry(0.7, 16, 12), new THREE.MeshBasicMaterial({ color: 0xfc3d21 }));
{ const w = trav.waypoints[trav.waypoints.length - 1], p = ll2cr(w.lat, w.lon); roverDot.position.copy(toWorld(p.c, p.r, 0.9)); travGroup.add(roverDot); }
const rover3d = new THREE.Group(); travGroup.add(rover3d);
{ const w = trav.waypoints[trav.waypoints.length - 1], p = ll2cr(w.lat, w.lon); rover3d.position.copy(toWorld(p.c, p.r, 0)); }
gltf.load('assets/models/perseverance.glb', (g) => { rover3d.add(fitModel(g.scene, 0.022, 0)); });   // 2.2 m tall, true scale

// markers
function makeHabitat() {
  const g = new THREE.Group();
  const dome = new THREE.Mesh(new THREE.SphereGeometry(0.08, 24, 12, 0, Math.PI*2, 0, Math.PI/2), new THREE.MeshStandardMaterial({ color: 0xf2f2f2, emissive: 0x332211, metalness: .3, roughness: .4 }));
  const beacon = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 14, 8), new THREE.MeshBasicMaterial({ color: 0x9fffb0, transparent: true, opacity: .8 }));
  beacon.position.y = 7; g.add(dome, beacon); return g;
}
function makePin(color) {
  const g = new THREE.Group();
  const stick = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 12, 8), new THREE.MeshBasicMaterial({ color }));
  stick.position.y = 6; const ball = new THREE.Mesh(new THREE.SphereGeometry(1, 16, 12), new THREE.MeshBasicMaterial({ color }));
  ball.position.y = 12.5; g.add(stick, ball); return g;
}
const habMesh = makeHabitat(); scene.add(habMesh);
const tgtMesh = makePin(0xffb070); tgtMesh.visible = false; scene.add(tgtMesh);
// astronaut figure (EVA suit built from primitives; drawn as a map marker, so it stays readable at any zoom)
function makeAstronaut() {
  const g = new THREE.Group(), body = new THREE.Group(); g.add(body);
  const suit = new THREE.MeshStandardMaterial({ color: 0xf4f4f2, roughness: .55 });
  const grey = new THREE.MeshStandardMaterial({ color: 0x8a9096, roughness: .7 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x3a3d42, roughness: .8 });
  const red = new THREE.MeshStandardMaterial({ color: 0xfc3d21, roughness: .5 });
  const gold = new THREE.MeshStandardMaterial({ color: 0xe0a526, metalness: .95, roughness: .12, emissive: 0x3a2400 });
  const add = (geo, mat, x, y, z, parent = body) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); parent.add(m); return m; };
  add(new THREE.CapsuleGeometry(.3, .5, 6, 14), suit, 0, 1.18, 0);                                  // torso
  add(new THREE.CylinderGeometry(.31, .31, .12, 16), grey, 0, .93, 0);                               // waist ring
  add(new THREE.BoxGeometry(.56, .7, .3), suit, 0, 1.26, -.32);                                      // life-support pack
  add(new THREE.BoxGeometry(.3, .16, .06), dark, 0, 1.22, .28);                                      // chest display
  add(new THREE.SphereGeometry(.26, 24, 18), suit, 0, 1.8, 0);                                       // helmet
  add(new THREE.SphereGeometry(.215, 24, 18, -Math.PI * .42, Math.PI * .84, Math.PI * .22, Math.PI * .42), gold, 0, 1.8, .07); // visor
  const limb = (x, y, len, r, isArm) => { const pivot = new THREE.Group(); pivot.position.set(x, y, 0); body.add(pivot);
    add(new THREE.CapsuleGeometry(r, len, 4, 10), suit, 0, -len / 2 - r * .5, 0, pivot);
    if (isArm) { add(new THREE.CylinderGeometry(r * 1.05, r * 1.05, .07, 10), red, 0, -len * .3, 0, pivot); add(new THREE.SphereGeometry(r * 1.1, 10, 8), grey, 0, -len - r * 1.2, 0, pivot); }
    else add(new THREE.BoxGeometry(r * 2.4, r * 1.4, r * 3.4), dark, 0, -len - r * 1.6, r * .6, pivot);
    return pivot; };
  g.userData = { body, legL: limb(-.14, .88, .55, .12, false), legR: limb(.14, .88, .55, .12, false),
                 armL: limb(-.4, 1.48, .48, .095, true), armR: limb(.4, 1.48, .48, .095, true), phase: 0 };
  g.userData.armL.rotation.z = .12; g.userData.armR.rotation.z = -.12;
  return g;
}
// NASA's official astronaut model (NASA 3D Resources); the primitive figure is only a fallback while it loads
const astro = new THREE.Group(); astro.visible = false; scene.add(astro);
const astroBody = new THREE.Group(); astro.add(astroBody); astro.userData = { body: astroBody, phase: 0 };
function fitModel(obj, height, yaw = 0) {           // scale to a given height, feet on y = 0, centred
  obj.rotation.y = yaw; obj.updateMatrixWorld(true);
  const b = new THREE.Box3().setFromObject(obj), size = b.getSize(new THREE.Vector3());
  const k = height / size.y; obj.scale.multiplyScalar(k); obj.updateMatrixWorld(true);
  const b2 = new THREE.Box3().setFromObject(obj), c = b2.getCenter(new THREE.Vector3());
  obj.position.x -= c.x; obj.position.z -= c.z; obj.position.y -= b2.min.y; return obj;
}
gltf.load('assets/models/astronaut.glb', (g) => { const m = g.scene; m.traverse(o => { if (o.isMesh) { o.castShadow = false; if (o.material) o.material.envMapIntensity = 1; } });
  astroBody.add(fitModel(m, 0.0185, ASTRO_YAW)); /* 1.85 m tall, true scale (1 unit = 100 m) */ }, undefined, () => { const f = makeAstronaut(); f.scale.setScalar(0.0185 / 2); astroBody.add(f); });
const astroGlow = new THREE.PointLight(0xbfefff, 2, 30); astroGlow.position.y = 2.5; astro.add(astroGlow);
let astroLast = new THREE.Vector3(), astroMoving = 0; const ASTRO_YAW = 0;
function poseAstro(dt) {
  const u = astro.userData, dist = camera.position.distanceTo(astro.position);
  const moved = astro.position.distanceTo(astroLast); astroLast.copy(astro.position);
  astroMoving = THREE.MathUtils.lerp(astroMoving, moved > 1e-4 ? 1 : 0, Math.min(1, dt * 6));
  u.phase += dt * (stormOn ? 13 : 8);
  const sw = Math.sin(u.phase) * .55 * astroMoving;
  if (u.legL) { u.legL.rotation.x = sw; u.legR.rotation.x = -sw; u.armL.rotation.x = -sw * .8; u.armR.rotation.x = sw * .8; }
  u.body.rotation.z = Math.sin(u.phase) * .05 * astroMoving;
  u.body.position.y = Math.abs(Math.cos(u.phase)) * .06 * astroMoving; u.body.rotation.x = (stormOn ? .22 : .06) * astroMoving;
}
function faceAlong(curve, u) { const a = curve.getPointAt(Math.min(1, u + .01)), b = curve.getPointAt(Math.max(0, u - .01)); const d = a.sub(b); if (d.lengthSq() > 1e-8) astro.rotation.y = Math.atan2(d.x, d.z); }

let routeObjs = [];
function clearRoutes() { routeObjs.forEach(o => { scene.remove(o); o.geometry?.dispose(); }); routeObjs = []; }
function smooth(path, iters = 3) { // Chaikin corner-cutting on grid path (display only)
  let p = path.map(q => ({ c: q.c, r: q.r }));
  for (let k = 0; k < iters; k++) {
    const n = [p[0]];
    for (let i = 0; i < p.length - 1; i++) { const a = p[i], b = p[i + 1];
      n.push({ c: .75 * a.c + .25 * b.c, r: .75 * a.r + .25 * b.r }, { c: .25 * a.c + .75 * b.c, r: .25 * a.r + .75 * b.r }); }
    n.push(p[p.length - 1]); p = n;
  }
  return p;
}
function tubeFor(path, color, radius = 0.45, opacity = 0.95) {
  const sm = smooth(path); const step = Math.max(1, Math.floor(sm.length / 600));
  const pts = sm.filter((_, i) => i % step === 0 || i === sm.length - 1).map(p => toWorld(p.c, p.r, 0.04));
  if (pts.length < 2) return null;
  const curve = new THREE.CatmullRomCurve3(pts);
  const dense = curve.getSpacedPoints(Math.min(pts.length * 3, 1500)).map(p => new THREE.Vector3(p.x, surfaceY(p.x, p.z) + 0.04, p.z));
  const m = fatLine(dense, color, radius * 14, opacity);
  m.userData.curve = curve; scene.add(m); routeObjs.push(m); return m;
}
function lineFor(a, b, color) {
  const pts = []; const n = 120;
  for (let i = 0; i <= n; i++) pts.push(toWorld(a.c + (b.c - a.c) * i / n, a.r + (b.r - a.r) * i / n, 0.06));
  const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineDashedMaterial({ color, dashSize: 0.8, gapSize: 0.6, transparent: true, opacity: .8 }));
  l.computeLineDistances(); scene.add(l); routeObjs.push(l); return l;
}

// storm particles
const PN = 6000, pGeo = new THREE.BufferGeometry(), pPos = new Float32Array(PN * 3);
for (let i = 0; i < PN; i++) { pPos[i*3] = (Math.random() - .5) * worldW * 2; pPos[i*3+1] = Math.random() * 120; pPos[i*3+2] = (Math.random() - .5) * worldD * 2; }
pGeo.setAttribute('position', new THREE.BufferAttribute(pPos, 3));
const dot = document.createElement('canvas'); dot.width = dot.height = 64;
{ const g = dot.getContext('2d'), gr = g.createRadialGradient(32, 32, 0, 32, 32, 32); gr.addColorStop(0, 'rgba(255,230,200,1)'); gr.addColorStop(.3, 'rgba(255,110,70,.8)'); gr.addColorStop(1, 'rgba(255,60,40,0)'); g.fillStyle = gr; g.fillRect(0, 0, 64, 64); }
const particles = new THREE.Points(pGeo, new THREE.PointsMaterial({ map: new THREE.CanvasTexture(dot), size: 1.6, transparent: true, opacity: 0.9, depthWrite: false, blending: THREE.AdditiveBlending }));
particles.visible = false; scene.add(particles);

// ---------- planning ----------
function cellOf(ll) { const p = ll2cr(ll.lat, ll.lon); return { c: Math.round(p.c), r: Math.round(p.r) }; }
function groundAt(x, z) { return surfaceY(x, z); }
function placeMarkers() {
  const h = cellOf(S.habitat); habMesh.position.copy(toWorld(h.c, h.r));
  if (!anim || anim.kind === 'fly') { astro.position.copy(toWorld(h.c + 0.3, h.r + 0.2)); astro.rotation.y = Math.PI * .8; astro.visible = !!S.target; }
  if (S.target) { const t = cellOf(S.target); tgtMesh.position.copy(toWorld(t.c, t.r)); tgtMesh.visible = true; } else tgtMesh.visible = false;
}

function readiness() {
  const flags = []; let level = 0;
  const awakeEnd = S.awake + (S.plan ? S.plan.totalH : 0);
  if (S.sleep < 5) { level = 2; flags.push(`Only ${S.sleep} h of sleep`); }
  else if (S.sleep < 6) { level = Math.max(level, 1); flags.push(`Short sleep (${S.sleep} h)`); }
  if (awakeEnd > 16) { level = 2; flags.push(`${fmt(awakeEnd)} h awake by EVA end`); }
  else if (awakeEnd > 13) { level = Math.max(level, 1); flags.push(`${fmt(awakeEnd)} h awake by EVA end`); }
  if (S.slam) { level = Math.max(level, 1); flags.push('Recent slam shift: circadian misalignment'); }
  return { level, flags };
}

function plan() {
  clearRoutes(); placeMarkers(); S.shelter = null; setStormVisual(false);
  if (!S.target) { render(); return; }
  const h = cellOf(S.habitat), t = cellOf(S.target);
  const out = astar(h, t, S.maxSlope), back = out && astar(t, h, S.maxSlope);
  S.out = out; S.back = back;
  lineFor(h, t, 0xffffff);
  const base = straightLineCheck(h, t, S.maxSlope);
  if (!out || !back) { S.plan = { fail: true, base }; render(); return; }
  tubeFor(out, 0x4fe3ff, 0.28); tubeFor(back, 0x2a9dbb, 0.2, 0.7);
  const so = pathStats(out), sb = pathStats(back);
  const totalH = so.hours + S.sci + sb.hours;
  S.plan = { so, sb, totalH, base, reserveOK: totalH <= S.eva * 0.8, withinLimit: totalH <= S.eva };
  render();
}

// ---------- storm scenario ----------
function stormInfo() {
  if (!S.storm || !S.plan || S.plan.fail) return null;
  const arrival = new Date(S.storm.arrival);
  const start = new Date(arrival.getTime() - S.lead * 3600e3);
  const tHit = S.lead;                                     // hours into EVA when the CME is predicted to arrive
  const { so, sb, totalH } = S.plan;
  let phase, pos, path;
  if (tHit >= totalH) phase = 'after';
  else if (tHit <= so.hours) { phase = 'outbound'; const i = so.cum.findIndex(h => h >= tHit); pos = S.out[Math.max(0, i)]; }
  else if (tHit <= so.hours + S.sci) { phase = 'science'; pos = S.out[S.out.length - 1]; }
  else { phase = 'return'; const tb = tHit - so.hours - S.sci; const i = sb.cum.findIndex(h => h >= tb); pos = S.back[Math.max(0, i)]; }
  let shelterH = 0;
  if (pos) { path = astar(pos, cellOf(S.habitat), S.maxSlope); if (path) shelterH = pathStats(path).hours; }
  return { arrival, start, tHit, phase, pos, path, shelterH, startLMST: lmst(start, S.habitat.lon), endLMST: lmst(new Date(start.getTime() + totalH * 3600e3), S.habitat.lon) };
}

let stormOn = false;
function setStormVisual(on) {
  stormOn = on; particles.visible = on; $('banner').classList.toggle('show', on); $('vignette').classList.toggle('show', on);
  terrain.material.color.set(on ? 0xffb0a0 : 0xffffff); wideMesh.material.color.set(on ? 0xffb0a0 : 0xffffff);
  skyU.hor.value.copy(on ? SKY_STORM : SKY_CALM); skyU.top.value.set(on ? 0x1a0303 : 0x24120c); scene.fog.color.copy(skyU.hor.value);
}

$('runStorm').onclick = () => {
  if (!S.plan || S.plan.fail) { toast('Pick a reachable target first.'); return; }
  const si = stormInfo(); clearRoutes(); plan();
  if (!si) return;
  if (si.phase === 'after') {
    $('banner').textContent = `Predicted arrival ${si.arrival.toISOString().slice(0,16)}Z falls after the EVA ends. Crew is already inside.`;
    $('banner').classList.add('show'); setTimeout(() => $('banner').classList.remove('show'), 4000); return;
  }
  setStormVisual(true);
  if (si.path) {
    const m = tubeFor(si.path, 0xff3b30, 0.4, 1); S.shelter = si;
    astro.position.copy(toWorld(si.pos.c, si.pos.r, 0)); astro.visible = true; crew.t = si.tHit; crew.mode = 'abort';
    animateAlong(m.userData.curve, 6000);
    flyTo(toWorld(si.pos.c, si.pos.r), 120);
  }
  $('banner').textContent = `Solar storm inbound · CME of ${S.storm.start.slice(0, 10)}, ${S.storm.speed} km/s · hits ${fmt(si.tHit)} h into the walk · ${fmt(si.shelterH)} h back to shelter`;
  render();
};

// ---------- UI rendering ----------
let toastT = 0;
function toast(msg) { const t = $('toast'); t.textContent = msg; t.classList.add('show'); clearTimeout(toastT); toastT = setTimeout(() => t.classList.remove('show'), 2600); }
const LAND = new Date('2021-02-18T20:55:00Z');
const msdOf = (d) => ((d.getTime() / 86400000 + 2440587.5 + 69.184 / 86400) - 2405522.0028779) / 1.0274912517;
function topBar() {
  const now = new Date(), l = lmst(now, S.habitat.lon), loc = (d) => msdOf(d) + S.habitat.lon / 360;
  $('roLmst').textContent = hhmm(l.h); $('roSol').textContent = String(Math.floor(loc(now)) - Math.floor(loc(LAND)));
}
function render() {
  const P = S.plan, rd = readiness();
  topBar();
  $('hint').style.display = S.target ? 'none' : ''; $('result').style.display = S.target ? '' : 'none';
  if (!S.target) return;
  if (P.fail) {
    $('verdict').textContent = 'No safe route'; $('vdot').style.background = 'var(--bad)';
    $('why').textContent = `Every way there crosses ground steeper than ${S.maxSlope}°. ${fmt(P.base.fracBad * 100, 0)}% of the straight line is too steep.`;
    $('kv').innerHTML = ''; drawProfile(); return;
  }
  const si = stormInfo();
  let level = Math.max(rd.level, P.reserveOK ? 0 : (P.withinLimit ? 1 : 2));
  const reasons = [];
  if (!P.withinLimit) reasons.push(`${fmt(P.totalH)} h is over the ${S.eva} h suit limit`);
  else if (!P.reserveOK) reasons.push(`under 20% suit reserve`);
  reasons.push(...rd.flags.map(f => f.charAt(0).toLowerCase() + f.slice(1)));
  if (stormOn && si && si.phase !== 'after') {
    const left = S.eva - si.tHit, ok = si.shelterH <= Math.min(left, 1.0);
    level = ok ? Math.max(level, 1) : 2;
    reasons.unshift(`storm hits during ${si.phase}, ${fmt(si.shelterH)} h from shelter`);
  }
  $('vdot').style.background = ['var(--ok)', 'var(--warn)', 'var(--bad)'][level];
  $('verdict').textContent = ['GO', 'GO · CAUTION', 'NO-GO'][level];
  $('why').textContent = reasons.length ? reasons.join(' · ').replace(/^./, c => c.toUpperCase()) + '.' : 'All checks pass.';
  const rad = (P.totalH / 24) * RAD_MSV_PER_DAY, row = (k, v) => `<tr><td>${k}</td><td>${v}</td></tr>`;
  $('kv').innerHTML = row('Outbound / return', `${fmt(P.so.dist / 1000, 2)} / ${fmt(P.sb.dist / 1000, 2)} km`) +
    row('Suit time (incl. science)', `${fmt(P.totalH)} of ${fmt(S.eva)} h`) + row('Climb out / back', `${fmt(P.so.gain, 0)} / ${fmt(P.sb.gain, 0)} m`) +
    row('Max slope on route', `${fmt(Math.max(P.so.maxSl, P.sb.maxSl))}° (limit ${S.maxSlope}°)`) + row('Walking energy', `${fmt((P.so.energy + P.sb.energy) / 1000, 0)} kJ/kg`) +
    row('Surface dose, calm sun', `${fmt(rad, 3)} mSv`) + row('Straight line too steep', `${fmt(P.base.fracBad * 100, 0)}% (max ${fmt(P.base.maxSl, 0)}°)`) + row('On Perseverance\'s driven tracks', `${fmt(onTracks(S.out) * 100, 0)}% of route`) +
    (si ? row('EVA window (LMST)', `${hhmm(si.startLMST.h)}–${hhmm(si.endLMST.h)}`) : '');
  drawProfile();
}

function drawProfile() {
  const cv = $('profile'), g = cv.getContext('2d'), Wc = cv.width, Hc = cv.height; g.clearRect(0, 0, Wc, Hc);
  if (!S.plan || S.plan.fail || !S.out) return;
  const pts = []; let d = 0;
  for (let i = 0; i < S.out.length; i++) {
    if (i) d += Math.hypot((S.out[i].c - S.out[i-1].c) * S.dx, (S.out[i].r - S.out[i-1].r) * S.dy);
    pts.push({ d, z: S.z[idx(S.out[i].c, S.out[i].r)], s: S.slope[idx(S.out[i].c, S.out[i].r)] });
  }
  const zmin = Math.min(...pts.map(p => p.z)), zmax = Math.max(...pts.map(p => p.z)), T = 30, B = 22, L = 0;
  const X = v => L + v / d * (Wc - L), Y = z => Hc - B - (z - zmin) / Math.max(1, zmax - zmin) * (Hc - B - T);
  g.strokeStyle = 'rgba(255,255,255,.08)'; g.lineWidth = 1;
  for (let k = 0; k <= 4; k++) { const y = T + k * (Hc - B - T) / 4; g.beginPath(); g.moveTo(0, y); g.lineTo(Wc, y); g.stroke(); }
  g.fillStyle = 'rgba(63,213,255,.12)'; g.beginPath(); g.moveTo(X(0), Hc - B); pts.forEach(p => g.lineTo(X(p.d), Y(p.z))); g.lineTo(X(d), Hc - B); g.fill();
  g.lineWidth = 3; g.lineJoin = 'round';
  for (let i = 1; i < pts.length; i++) { const s = pts[i].s;
    g.strokeStyle = s > S.maxSlope ? '#fc3d21' : s > S.maxSlope * .75 ? '#f7c948' : '#3fd5ff';
    g.beginPath(); g.moveTo(X(pts[i-1].d), Y(pts[i-1].z)); g.lineTo(X(pts[i].d), Y(pts[i].z)); g.stroke(); }
  g.fillStyle = '#9aa0a6'; g.font = '500 19px "DM Mono", monospace';
  g.fillText(`${Math.round(zmax)} m`, 0, 20); g.fillText(`0 km`, 0, Hc - 2);
  g.textAlign = 'right'; g.fillText(`${fmt(d / 1000, 1)} km`, Wc, Hc - 2); g.fillText(`ELEVATION PROFILE · OUTBOUND`, Wc, 20); g.textAlign = 'left';
}

// ---------- storms + timeline ----------
const T0 = Date.parse('2024-01-01'), T1 = Date.parse('2026-12-31');
let tlHover = -1;
function loadStorms(list, label) {
  S.storms = list.slice().filter(s => s.arrival).sort((a, b) => a.arrival.localeCompare(b.arrival));
  const fast = S.storms.filter(s => !s.glancing).reduce((m, s) => (!m || (s.speed || 0) > (m.speed || 0)) ? s : m, null);
  S.storm = S.storms.find(s => s.start.startsWith('2026-08-21')) || fast || S.storms[0] || null;
  $('roCme').textContent = String(S.storms.length);
  $('tlSub').textContent = `${S.storms.length} coronal mass ejections predicted to reach Mars · NASA CCMC DONKI · WSA-ENLIL · ${label}`;
  drawTimeline();
}
function drawTimeline() {
  const cv = $('tlc'), dpr = Math.min(devicePixelRatio, 2), w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== w * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
  const g = cv.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, w, h);
  const base = h - 16, X = t => (t - T0) / (T1 - T0) * w, Hs = v => Math.min(1, (v || 300) / 2500) * (base - 6);
  g.strokeStyle = 'rgba(255,255,255,.15)'; g.beginPath(); g.moveTo(0, base + .5); g.lineTo(w, base + .5); g.stroke();
  g.font = '500 10.5px "DM Mono", monospace'; g.fillStyle = '#5f6368';
  for (let y = 2024; y <= 2026; y++) for (let m = 0; m < 12; m++) {
    const x = X(Date.UTC(y, m, 1)); g.fillStyle = m === 0 ? '#9aa0a6' : 'rgba(255,255,255,.12)';
    g.fillRect(x, base, 1, m === 0 ? 8 : 4); if (m === 0) g.fillText(String(y), x + 4, h - 2); else if (m % 3 === 0) { g.fillStyle = '#5f6368'; g.fillText(['', '', '', 'APR', '', '', 'JUL', '', '', 'OCT'][m], x + 3, h - 2); }
  }
  [500, 1000, 2000].forEach(v => { const y = base - Hs(v); g.fillStyle = 'rgba(255,255,255,.07)'; g.fillRect(0, y, w, 1); g.fillStyle = '#5f6368'; g.fillText(`${v} km/s`, 2, y - 3); });
  S.storms.forEach((s, i) => {
    const x = X(Date.parse(s.arrival)), sel = s === S.storm, hv = i === tlHover, v = s.speed || 300;
    const t = Math.min(1, v / 2000);
    g.fillStyle = sel ? '#fc3d21' : hv ? '#fff' : `rgba(${Math.round(160 + 95 * t)},${Math.round(170 - 110 * t)},${Math.round(180 - 140 * t)},${s.glancing ? .35 : .8})`;
    g.fillRect(x - (sel ? 1.5 : .75), base - Hs(v), sel ? 3 : 1.5, Hs(v));
  });
  const now = X(Date.now()); g.fillStyle = '#fff'; g.fillRect(now, 4, 1, base - 4); g.fillText('TODAY', now - 40, 12);
  if (S.storm) { const x = X(Date.parse(S.storm.arrival)), y = base - Hs(S.storm.speed); g.fillStyle = '#fc3d21'; g.beginPath(); g.arc(x, y, 4, 0, 7); g.fill();
    g.fillStyle = '#fff'; g.textAlign = x > w - 220 ? 'right' : 'left'; g.fillText(`SCENARIO · ${S.storm.start.slice(0, 10)} · ${S.storm.speed} km/s`, x + (x > w - 220 ? -8 : 8), Math.max(12, y + 4)); g.textAlign = 'left'; }
}
function tlPick(e) {
  const r = $('tlc').getBoundingClientRect(), x = e.clientX - r.left, w = r.width; let best = -1, bd = 8;
  S.storms.forEach((s, i) => { const d = Math.abs((Date.parse(s.arrival) - T0) / (T1 - T0) * w - x); if (d < bd) { bd = d; best = i; } });
  return best;
}
$('tlc').addEventListener('mousemove', e => {
  const i = tlPick(e); if (i !== tlHover) { tlHover = i; drawTimeline(); }
  const tip = $('tip'); if (i < 0) { tip.style.display = 'none'; return; }
  const s = S.storms[i]; tip.innerHTML = `<b>CME ${s.start.slice(0, 16).replace('T', ' ')}Z</b><br><span class="mono">${s.speed ?? '?'} km/s · Mars arrival ${s.arrival.slice(0, 16).replace('T', ' ')}Z${s.glancing ? ' · glancing' : ''}</span><br><span style="color:#9aa0a6">Click to use as the storm scenario</span>`;
  tip.style.display = 'block'; tip.style.left = Math.min(innerWidth - 300, e.clientX + 12) + 'px'; tip.style.top = (e.clientY - 80) + 'px';
});
$('tlc').addEventListener('mouseleave', () => { tlHover = -1; $('tip').style.display = 'none'; drawTimeline(); });
$('tlc').addEventListener('click', e => { const i = tlPick(e); if (i < 0) return; S.storm = S.storms[i]; drawTimeline();
  if (stormOn) $('runStorm').click(); else { render(); toast(`Storm scenario set: CME of ${S.storm.start.slice(0, 10)}, ${S.storm.speed} km/s`); } });
loadStorms(donki.mars_cmes, `snapshot ${donki.retrieved.slice(0, 10)}`);
$('liveDonki').onclick = async () => {
  const end = new Date(), start = new Date(end.getTime() - 55 * 864e5), f = d => d.toISOString().slice(0, 10);
  toast('Fetching the last 55 days from NASA DONKI…');
  try {
    const j = await fetch(`https://api.nasa.gov/DONKI/CME?startDate=${f(start)}&endDate=${f(end)}&api_key=DEMO_KEY`).then(r => r.json());
    const ev = [];
    for (const c of j) for (const a of (c.cmeAnalyses || [])) for (const e of (a.enlilList || [])) for (const imp of (e.impactList || []))
      if ((imp.location || '').toLowerCase() === 'mars') ev.push({ id: c.activityID, start: c.startTime, link: c.link, arrival: imp.arrivalTime, glancing: imp.isGlancingBlow, speed: a.speed });
    const uniq = Object.values(Object.fromEntries(ev.map(e => [e.id, e])));
    const ids = new Set(donki.mars_cmes.map(s => s.id)); const added = uniq.filter(e => !ids.has(e.id));
    loadStorms(donki.mars_cmes.concat(added), `live ${new Date().toISOString().slice(0, 16)}Z`);
    toast(`DONKI: ${uniq.length} Mars-bound CMEs in the last 55 days, ${added.length} new.`);
  } catch (e) { toast('Live fetch failed (rate limit?). Using the bundled snapshot.'); }
  render();
};
addEventListener('resize', drawTimeline);

// ---------- ground truth images ----------
const imgs = [{ url: 'assets/perseverance-sol2-navcam.jpg', camera: 'NAVCAM_LEFT', sol: 2, local: true }].concat(
  rover.images.slice().sort((a, b) => (a.camera.includes('NAVCAM') ? 0 : 1) - (b.camera.includes('NAVCAM') ? 0 : 1)));
let lbI = 0;
function showLB(i) {
  lbI = (i + imgs.length) % imgs.length; const im = imgs[lbI];
  $('lbImg').src = im.url; $('lbLab').textContent = `Ground truth · ${lbI + 1} of ${imgs.length}`;
  $('lbTitle').textContent = `Perseverance ${im.camera.replaceAll('_', ' ').toLowerCase()}, sol ${im.sol}`;
  $('lbBody').innerHTML = `Raw image taken at Octavia E. Butler Landing, the habitat site in this plan, ${im.sol === 0 ? 'minutes after touchdown' : `on mission sol ${im.sol}`}. Use it to see the real ground at the scale of a footstep: the orbital elevation model is 60 m per pixel. <div style="color:#5f6368;margin-top:10px">NASA/JPL-Caltech</div>`;
}
$('thRover').onclick = () => { showLB(0); $('lb').classList.add('show'); };
$('lbPrev').onclick = () => showLB(lbI - 1); $('lbNext').onclick = () => showLB(lbI + 1);
$('lbClose').onclick = () => $('lb').classList.remove('show');
$('lb').onclick = (e) => { if (e.target === $('lb')) $('lb').classList.remove('show'); };
addEventListener('keydown', (e) => { if (e.key === 'Escape') { $('lb').classList.remove('show'); $('about').classList.remove('show'); } });

// ---------- controls wiring ----------
const bind = (id, key, out, suf = '', dp = 1, replanFlag = true) => $(id).oninput = () => { S[key] = +$(id).value; $(out).textContent = fmt(S[key], dp) + suf; if (id === 'slope') drawTexture(); replanFlag ? plan() : render(); };
bind('slope', 'maxSlope', 'vSlope', '°', 0); bind('eva', 'eva', 'vEva', ' h', 1, false); bind('sci', 'sci', 'vSci', ' h', 2);
bind('sleep', 'sleep', 'vSleep', ' h', 1, false); bind('awake', 'awake', 'vAwake', ' h', 1, false); bind('lead', 'lead', 'vLead', ' h', 1, false);
$('sci').oninput = () => { S.sci = +$('sci').value; $('vSci').textContent = fmt(S.sci, 2) + ' h'; if (S.plan && !S.plan.fail) { S.plan.totalH = S.plan.so.hours + S.sci + S.plan.sb.hours; S.plan.reserveOK = S.plan.totalH <= S.eva * .8; S.plan.withinLimit = S.plan.totalH <= S.eva; } render(); };
$('eva').oninput = () => { S.eva = +$('eva').value; $('vEva').textContent = fmt(S.eva) + ' h'; if (S.plan && !S.plan.fail) { S.plan.reserveOK = S.plan.totalH <= S.eva * .8; S.plan.withinLimit = S.plan.totalH <= S.eva; } render(); };
$('slam').onchange = () => { S.slam = $('slam').checked; render(); };
const LAYERS = { imagery: true, elev: false, slope: false, trav: true, labels: true };
function applyLayers() {
  elevU.uImg.value = LAYERS.imagery ? 1 : 0; elevU.uElev.value = LAYERS.elev ? 1 : 0;
  $('legend').classList.toggle('show', LAYERS.elev);
  if (S.haz !== LAYERS.slope) { S.haz = LAYERS.slope; drawTexture(); }
  travGroup.visible = LAYERS.trav; $('labels').style.display = LAYERS.labels ? '' : 'none';
  document.querySelectorAll('.layer').forEach(el => el.classList.toggle('on', LAYERS[el.dataset.l]));
}
document.querySelectorAll('.layer').forEach(el => el.onclick = () => { LAYERS[el.dataset.l] = !LAYERS[el.dataset.l]; applyLayers(); });
$('demoTarget').onclick = () => { S.target = { ...DELTA_DEMO }; plan(); flyTo(toWorld(...midCell()), 120); };
function midCell() { const h = cellOf(S.habitat), t = cellOf(S.target || S.habitat); return [(h.c + t.c) / 2, (h.r + t.r) / 2]; }
$('aboutBtn').onclick = () => $('about').classList.add('show');
$('closeAbout').onclick = () => $('about').classList.remove('show');
$('about').onclick = (e) => { if (e.target === $('about')) $('about').classList.remove('show'); };
$('homeBtn').onclick = () => { anim = null; flyHome(); };
$('crewBtn').onclick = () => { controls.autoRotate = false; const p = astro.position.clone(); const from = { p: camera.position.clone(), t: controls.target.clone() };
  anim = { kind: 'fly', t0: performance.now(), dur: 2200, from, to: { p: p.clone().add(new THREE.Vector3(0.05, 0.025, 0.07)), t: p.clone().add(new THREE.Vector3(0, 0.009, 0)) } }; };
$('roverBtn').onclick = () => { controls.autoRotate = false; const p = rover3d.position.clone(); const from = { p: camera.position.clone(), t: controls.target.clone() };
  anim = { kind: 'fly', t0: performance.now(), dur: 2200, from, to: { p: p.clone().add(new THREE.Vector3(0.06, 0.03, 0.08)), t: p.clone().add(new THREE.Vector3(0, 0.01, 0)) } }; };

// ---------- picking + cursor readout ----------
const ray = new THREE.Raycaster(), mouse = new THREE.Vector2(); let downAt = null, hoverXY = null;
renderer.domElement.addEventListener('pointerdown', e => downAt = [e.clientX, e.clientY]);
renderer.domElement.addEventListener('pointermove', e => hoverXY = [e.clientX, e.clientY]);
renderer.domElement.addEventListener('pointerup', e => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 5 || e.button !== 0) return;
  mouse.set(e.clientX / innerWidth * 2 - 1, -(e.clientY / innerHeight) * 2 + 1); ray.setFromCamera(mouse, camera);
  const hit = ray.intersectObject(terrain)[0];
  if (!hit) { toast('Routes are planned inside the high-resolution crater zone, near the habitat.'); return; }
  const c = (hit.point.x / worldW + 0.5) * S.W, r = (hit.point.z / worldD + 0.5) * S.H;
  S.target = cr2ll(c, r); plan();
});
function readCursor() {
  if (!hoverXY) return; const [x, y] = hoverXY; hoverXY = null;
  mouse.set(x / innerWidth * 2 - 1, -(y / innerHeight) * 2 + 1); ray.setFromCamera(mouse, camera);
  const hit = ray.intersectObjects([terrain, wideMesh])[0]; if (!hit) return;
  const lon = meta.lon_min + (hit.point.x / worldW + 0.5) * lonSpan, lat = meta.lat_max - (hit.point.z / worldD + 0.5) * latSpan;
  const c = (lon - meta.lon_min) / lonSpan * S.W, r = (meta.lat_max - lat) / latSpan * S.H, inside = c >= 0 && r >= 0 && c < S.W - 1 && r < S.H - 1;
  const z = inside ? zAt(c, r) : wideZ(lat, lon);
  const sl = inside ? `  ·  slope ${fmt(S.slope[idx(Math.round(c), Math.round(r))])}°` : '';
  const hb = cellOf(S.habitat), dist = inside ? `  ·  ${fmt(Math.hypot((c - hb.c) * S.dx, (r - hb.r) * S.dy) / 1000, 2)} km from habitat` : '';
  $('roCursor').textContent = `${fmt(lat, 4)}°N  ${fmt(lon, 4)}°E  ·  ${Math.round(z).toLocaleString()} m${sl}${dist}`;
}
function updateScale() {
  const dist = camera.position.distanceTo(controls.target), mPerPx = 2 * dist * Math.tan(camera.fov * Math.PI / 360) / innerHeight * UNIT;
  const raw = mPerPx * 110, p = Math.pow(10, Math.floor(Math.log10(raw))), n = [1, 2, 5, 10].find(k => k * p >= raw) * p;
  $('scale').style.width = (n / mPerPx) + 'px'; $('scaleTxt').textContent = n >= 1000 ? `${n / 1000} km` : `${n} m`;
  const d = controls.target.clone().sub(camera.position); $('needle').setAttribute('transform', `rotate(${-Math.atan2(d.x, -d.z) * 180 / Math.PI} 20 20)`);
}

// ---------- place labels ----------
const PLACES = [
  { big: true, name: 'Jezero Crater', lat: 18.38, lon: 77.58, lift: 30 },
  { name: 'Habitat', sub: 'Octavia E. Butler Landing · sol 0', lat: HABITAT0.lat, lon: HABITAT0.lon, lift: 6 },
  { name: 'Western delta', sub: 'river deposits, ~3.5 billion yr', lat: 18.505, lon: 77.395, lift: 4 },
];
const lastWP = trav.waypoints[trav.waypoints.length - 1];
PLACES.splice(2, 0, { name: 'Perseverance', sub: `sol ${lastWP.sol} · ${fmt(lastWP.km, 1)} km driven`, lat: lastWP.lat, lon: lastWP.lon, lift: 4 });
PLACES.push(PLACES.shift());
PLACES.forEach(p => { const el = document.createElement('div'); el.className = 'pl' + (p.big ? ' big' : ''); el.innerHTML = `<div class="t">${p.name}${p.sub ? `<small>${p.sub}</small>` : ''}</div><div class="stem"></div>`; $('labels').appendChild(el); p.el = el; });
const crew = { t: 0, km: 0, mode: 'idle' };
const crewLabel = document.createElement('div'); crewLabel.className = 'pl'; crewLabel.innerHTML = '<div class="t" style="border-color:#3fd5ff">Crew<small></small></div><div class="stem"></div>'; $('labels').appendChild(crewLabel);
const tgtLabel = document.createElement('div'); tgtLabel.className = 'pl'; tgtLabel.innerHTML = '<div class="t" style="border-color:#ffb070">EVA target<small></small></div><div class="stem"></div>'; $('labels').appendChild(tgtLabel);
const _v = new THREE.Vector3();
function placeLabels() {
  const put = (el, wp) => { _v.copy(wp).project(camera); const vis = _v.z < 1 && Math.abs(_v.x) < 1.1 && Math.abs(_v.y) < 1.1;
    el.style.display = vis ? '' : 'none'; if (vis) { el.style.left = ((_v.x + 1) / 2 * innerWidth) + 'px'; el.style.top = ((1 - _v.y) / 2 * innerHeight) + 'px'; } };
  const far = camera.position.distanceTo(controls.target) > 260;
  PLACES.forEach(p => { const ll = ll2cr(p.lat, p.lon); const w = new THREE.Vector3(wX(p.lon), (zAt(ll.c, ll.r) - meta.z_min) / UNIT * S.ex + p.lift, wZ(p.lat)); put(p.el, w);
    p.el.classList.toggle('far', far); });
  // declutter: drop lower-priority labels that collide
  const boxes = []; PLACES.forEach(p => { if (p.el.style.display === 'none') return; const r = p.el.getBoundingClientRect();
    if (boxes.some(b => r.left < b.right && r.right > b.left && r.top < b.bottom && r.bottom > b.top)) p.el.style.display = 'none'; else boxes.push(r); });
  if (astro.visible) { put(crewLabel, astro.position.clone().add(new THREE.Vector3(0, Math.max(0.03, camera.position.distanceTo(astro.position) * .045), 0)));
    crewLabel.querySelector('small').textContent = crew.mode === 'out' ? `EVA T+${fmt(crew.t)} h · ${fmt(crew.km, 1)} km out` : crew.mode === 'abort' ? `storm hit at T+${fmt(S.shelter ? S.shelter.tHit : crew.t)} h · now T+${fmt(crew.t)} h` : 'suited up at the habitat · 1.85 m, true scale'; }
  else crewLabel.style.display = 'none';
  if (S.target) { const t = cellOf(S.target); put(tgtLabel, toWorld(t.c, t.r, 14)); tgtLabel.querySelector('small').textContent = S.plan && !S.plan.fail ? `${fmt(S.plan.so.dist / 1000, 1)} km walk · +${Math.round(S.z[idx(t.c, t.r)] - S.z[idx(cellOf(S.habitat).c, cellOf(S.habitat).r)])} m` : ''; }
  else tgtLabel.style.display = 'none';
}

// ---------- camera animation ----------
let anim = null;
function flyTo(target, dist) {
  const from = { p: camera.position.clone(), t: controls.target.clone() };
  const dir = new THREE.Vector3(0.55, 0.75, 0.9).normalize();
  const to = { p: target.clone().add(dir.multiplyScalar(dist)), t: target.clone() };
  anim = { kind: 'fly', t0: performance.now(), dur: 1600, from, to };
}
function animateAlong(curve, dur) { anim = { kind: 'astro', t0: performance.now(), dur, curve }; }
$('fly').onclick = () => {
  const m = routeObjs.find(o => o.userData.curve); if (!m) { toast('Pick a target first.'); return; }
  anim = { kind: 'route', t0: performance.now(), dur: 16000, curve: m.userData.curve }; astro.visible = true; setStormVisual(false); $('banner').classList.remove('show');
};
const HOME = { t: new THREE.Vector3(70, 0, 40), p: new THREE.Vector3(-260, 210, 520) };
function resetView() { camera.position.copy(HOME.p); controls.target.copy(HOME.t); }
function flyHome() { const from = { p: camera.position.clone(), t: controls.target.clone() }; anim = { kind: 'fly', t0: performance.now(), dur: 1800, from, to: { p: HOME.p.clone(), t: HOME.t.clone() } }; }
resetView();
controls.autoRotate = true; controls.autoRotateSpeed = 0.35;
const stopIntro = () => { controls.autoRotate = false; };
renderer.domElement.addEventListener('pointerdown', stopIntro, { once: true });
$('right').addEventListener('click', stopIntro); $('left').addEventListener('click', stopIntro);

// ---------- loop ----------
const clock = new THREE.Clock();
function tick() {
  const dt = clock.getDelta(), now = performance.now();
  if (anim) {
    const u = Math.min(1, (now - anim.t0) / anim.dur), e = u < .5 ? 2*u*u : 1 - (-2*u + 2)**2 / 2;
    if (anim.kind === 'fly') { camera.position.lerpVectors(anim.from.p, anim.to.p, e); controls.target.lerpVectors(anim.from.t, anim.to.t, e); }
    else if (anim.kind === 'route') {
      const p = anim.curve.getPointAt(u), a = anim.curve.getPointAt(Math.min(1, u + 0.02));
      astro.position.set(p.x, groundAt(p.x, p.z), p.z); faceAlong(anim.curve, u);
      crew.t = u * S.plan.so.hours; crew.km = u * S.plan.so.dist / 1000; crew.mode = 'out';
      const back = p.clone().sub(a).setY(0).normalize().multiplyScalar(0.7), aim = new THREE.Vector3(a.x, groundAt(a.x, a.z) + 0.01, a.z);
      const k = 1 - Math.exp(-4 * Math.min(dt, 0.5)); camera.position.lerp(astro.position.clone().add(back).add(new THREE.Vector3(0, 0.22, 0)), k); controls.target.lerp(aim, Math.min(1, k * 2));
    } else if (anim.kind === 'astro') { const p = anim.curve.getPointAt(u); astro.position.set(p.x, groundAt(p.x, p.z), p.z); faceAlong(anim.curve, u); if (S.shelter) { crew.t = S.shelter.tHit + u * S.shelter.shelterH; crew.mode = 'abort'; } }
    if (u >= 1) anim = null;
  }
  if (particles.visible) {
    const p = pGeo.attributes.position;
    for (let i = 0; i < PN; i++) { let y = p.getY(i) - 60 * dt; if (y < 0) y += 120; p.setY(i, y); }
    p.needsUpdate = true;
  }
  habMesh.children[1].material.opacity = 0.5 + 0.4 * Math.sin(now / 300);
  roverDot.scale.setScalar(1 + 0.35 * Math.sin(now / 250));
  roverDot.visible = camera.position.distanceTo(rover3d.position) > 2;
  sky.position.copy(camera.position);
  if (astro.visible) poseAstro(dt);
  if (!anim || anim.kind !== "route") controls.update(); else camera.lookAt(controls.target);
  renderer.render(scene, camera);
  placeLabels(); readCursor(); updateScale();
  requestAnimationFrame(tick);
}
addEventListener('resize', () => { LINE_MATS.forEach(m => m.resolution.set(innerWidth, innerHeight)); camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
placeMarkers(); applyLayers(); render(); setInterval(topBar, 30000); tick();
$('loading').remove();
// URL params for screenshots: ?demo=1&storm=1&still=1&clean=1&elev=1
{ const q = new URLSearchParams(location.search);
  if (q.has('still')) controls.autoRotate = false;
  if (q.has('clean')) document.body.classList.add('clean');
  if (q.has('elev')) { LAYERS.elev = true; applyLayers(); }
  if (q.has('demo')) { S.target = { ...DELTA_DEMO }; plan(); controls.autoRotate = false;
    const m = toWorld(...midCell()); camera.position.copy(m.clone().add(new THREE.Vector3(60, 95, 120))); controls.target.copy(m); }
  if (q.has('storm')) $('runStorm').click();
  if (q.has('rover')) { controls.autoRotate = false; const p = rover3d.position.clone(); controls.target.copy(p).add(new THREE.Vector3(0, .6, 0)); camera.position.copy(p).add(new THREE.Vector3(0.06, 0.03, 0.08)); controls.target.copy(p).add(new THREE.Vector3(0, 0.01, 0)); }
  if (q.has('astro')) { const p = astro.position.clone(); controls.target.copy(p).add(new THREE.Vector3(0, .009, 0)); camera.position.copy(p).add(new THREE.Vector3(0.05, 0.025, 0.07)); } }
window.__S = S; window.__api = { camera, controls, astro, rover3d, astar, pathStats, straightLineCheck, cellOf, cr2ll, ll2cr, plan }; // debug
