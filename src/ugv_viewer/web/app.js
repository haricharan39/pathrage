import * as THREE from 'three';
import { OrbitControls } from '/static/vendor/OrbitControls.js';
import { MSG_MAP_DELTA, MSG_LIVE, MSG_TIER1, msgType, parseMapDelta, parseLive, parseTier1 } from '/static/wire.js';

// ROS convention: Z up, X forward.
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const $ = (id) => document.getElementById(id);
const LIVE_CAP = 20000, TIER1_CAP = 20000, TRAJ_CAP = 50000, PLAN_CAP = 500;

// ---------------------------------------------------------------- scene
const container = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setClearColor(0x12151a);
container.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 400);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.12;
controls.maxPolarAngle = Math.PI * 0.499;

const grid = new THREE.GridHelper(60, 60, 0x3a4658, 0x252d39);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(1));

// ---------------------------------------------------------------- point shader
const POINT_VS = `
  attribute vec3 aColor;
  attribute float aCls;
  uniform float uSize, uScale, uZmin, uZmax, uObstOnly, uBoost;
  uniform int uMode;
  varying vec3 vCol;
  vec3 ramp(float t) {                       // blue -> cyan -> green -> yellow -> red
    vec3 a = vec3(0.10, 0.25, 0.90), b = vec3(0.05, 0.85, 0.90), c = vec3(0.25, 0.85, 0.25),
         d = vec3(0.98, 0.88, 0.15), e = vec3(0.95, 0.20, 0.12);
    if (t < 0.25) return mix(a, b, t / 0.25);
    if (t < 0.50) return mix(b, c, (t - 0.25) / 0.25);
    if (t < 0.75) return mix(c, d, (t - 0.50) / 0.25);
    return mix(d, e, (t - 0.75) / 0.25);
  }
  void main() {
    vec4 mv = modelViewMatrix * vec4(position, 1.0);
    gl_Position = projectionMatrix * mv;
    float h = clamp((position.z - uZmin) / (uZmax - uZmin), 0.0, 1.0);
    vec3 col = aColor;
    if (uMode == 1) col = ramp(h);
    if (uMode == 2) {
      if (aCls > 1.5)      col = vec3(1.00, 0.18, 0.82);
      else if (aCls > 0.5) col = vec3(1.00, 0.29, 0.24);
      else                 col = aColor * 0.55 + vec3(0.04, 0.06, 0.08);
    }
    vCol = mix(col, vec3(1.0), uBoost);
    if (uObstOnly > 0.5 && aCls < 0.5) gl_Position = vec4(2.0, 2.0, 2.0, 1.0);   // clipped away
    gl_PointSize = clamp(uSize * uScale / max(-mv.z, 0.3), 1.5, 28.0);   // uScale = voxel size in pixels at 1 m
  }`;
const POINT_FS = `
  varying vec3 vCol;
  void main() { gl_FragColor = vec4(vCol, 1.0); }`;

function pointMaterial(boost = 0) {
  return new THREE.ShaderMaterial({
    vertexShader: POINT_VS, fragmentShader: POINT_FS,
    uniforms: {
      uSize: { value: 1 }, uScale: { value: 100 }, uZmin: { value: -0.3 }, uZmax: { value: 1.5 },
      uObstOnly: { value: 0 }, uBoost: { value: boost }, uMode: { value: 2 },
    },
  });
}

// ---------------------------------------------------------------- map buffers
const mapMat = pointMaterial(0);
const liveMat = pointMaterial(0.25);
const mapGeo = new THREE.BufferGeometry();
let mapCap = 0, nMap = 0;
let mapPos, mapCol, mapCls;
let dirty = { lo: Infinity, hi: -1 };
const mapPoints = new THREE.Points(mapGeo, mapMat);
mapPoints.frustumCulled = false;
scene.add(mapPoints);

function allocMap(cap) {
  mapCap = cap;
  mapPos = new Float32Array(cap * 3);
  mapCol = new Uint8Array(cap * 3);
  mapCls = new Uint8Array(cap);
  mapGeo.setAttribute('position', new THREE.BufferAttribute(mapPos, 3).setUsage(THREE.DynamicDrawUsage));
  mapGeo.setAttribute('aColor', new THREE.BufferAttribute(mapCol, 3, true).setUsage(THREE.DynamicDrawUsage));
  mapGeo.setAttribute('aCls', new THREE.BufferAttribute(mapCls, 1).setUsage(THREE.DynamicDrawUsage));
  nMap = 0;
  mapGeo.setDrawRange(0, 0);
  dirty = { lo: Infinity, hi: -1 };
}

function flushMap() {
  if (dirty.hi < dirty.lo) return;
  const lo = dirty.lo, count = dirty.hi - dirty.lo + 1;
  const a = mapGeo.attributes;
  a.position.addUpdateRange(lo * 3, count * 3); a.position.needsUpdate = true;
  a.aColor.addUpdateRange(lo * 3, count * 3);   a.aColor.needsUpdate = true;
  a.aCls.addUpdateRange(lo, count);             a.aCls.needsUpdate = true;
  mapGeo.setDrawRange(0, nMap);
  dirty = { lo: Infinity, hi: -1 };
}

function applyMapDelta(buf) {
  const { n, idx, pos, rgb, cls } = parseMapDelta(buf);
  if (n === 0 || mapCap === 0) return;
  let lo = dirty.lo, hi = dirty.hi;
  for (let i = 0; i < n; i++) {
    const j = idx[i];
    if (j >= mapCap) continue;
    mapPos[3 * j] = pos[3 * i]; mapPos[3 * j + 1] = pos[3 * i + 1]; mapPos[3 * j + 2] = pos[3 * i + 2];
    mapCol[3 * j] = rgb[3 * i]; mapCol[3 * j + 1] = rgb[3 * i + 1]; mapCol[3 * j + 2] = rgb[3 * i + 2];
    mapCls[j] = cls[i];
    if (j < lo) lo = j;
    if (j > hi) hi = j;
    if (j >= nMap) nMap = j + 1;
  }
  dirty = { lo, hi };
}

// ---------------------------------------------------------------- live frame + tier-1
const liveGeo = new THREE.BufferGeometry();
const livePos = new Float32Array(LIVE_CAP * 3), liveCol = new Uint8Array(LIVE_CAP * 3), liveCls = new Uint8Array(LIVE_CAP);
liveGeo.setAttribute('position', new THREE.BufferAttribute(livePos, 3).setUsage(THREE.DynamicDrawUsage));
liveGeo.setAttribute('aColor', new THREE.BufferAttribute(liveCol, 3, true).setUsage(THREE.DynamicDrawUsage));
liveGeo.setAttribute('aCls', new THREE.BufferAttribute(liveCls, 1).setUsage(THREE.DynamicDrawUsage));
liveGeo.setDrawRange(0, 0);
const livePoints = new THREE.Points(liveGeo, liveMat);
livePoints.frustumCulled = false;
scene.add(livePoints);

function applyLive(buf) {
  const f = parseLive(buf);
  const n = Math.min(f.n, LIVE_CAP);
  livePos.set(f.pos.subarray(0, 3 * n));
  liveCol.set(f.rgb.subarray(0, 3 * n));
  liveCls.set(f.cls.subarray(0, n));
  liveGeo.setDrawRange(0, n);
  for (const k of ['position', 'aColor', 'aCls']) liveGeo.attributes[k].needsUpdate = true;
}

const t1Geo = new THREE.BufferGeometry();
const t1Pos = new Float32Array(TIER1_CAP * 3);
t1Geo.setAttribute('position', new THREE.BufferAttribute(t1Pos, 3).setUsage(THREE.DynamicDrawUsage));
t1Geo.setDrawRange(0, 0);
const tier1 = new THREE.Points(t1Geo, new THREE.PointsMaterial({ color: 0xff2a1f, size: 0.09, sizeAttenuation: true }));
tier1.frustumCulled = false;
scene.add(tier1);

function applyTier1(buf) {
  const f = parseTier1(buf);
  const n = Math.min(f.n, TIER1_CAP);
  t1Pos.set(f.pos.subarray(0, 3 * n));
  t1Geo.setDrawRange(0, n);
  t1Geo.attributes.position.needsUpdate = true;
}

// ---------------------------------------------------------------- robot
const robot = new THREE.Group();
robot.visible = true;
{
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.60, 0.40, 0.15),
    new THREE.MeshBasicMaterial({ color: 0x4fb3ff, transparent: true, opacity: 0.55 }));
  body.position.set(0, 0, 0.135);
  const edges = new THREE.LineSegments(new THREE.EdgesGeometry(body.geometry),
    new THREE.LineBasicMaterial({ color: 0x9fd8ff }));
  body.add(edges);
  robot.add(body);
  const wheelGeo = new THREE.CylinderGeometry(0.10, 0.10, 0.06, 16);
  const wheelMat = new THREE.MeshBasicMaterial({ color: 0x20252d });
  for (const sx of [-0.2, 0.2]) for (const sy of [-0.25, 0.25]) {
    const w = new THREE.Mesh(wheelGeo, wheelMat);
    w.rotation.x = Math.PI / 2;          // axis along Y
    w.position.set(sx, sy, 0.10);
    robot.add(w);
  }
  const mast = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.16, 0.30),
    new THREE.MeshBasicMaterial({ color: 0xffd23e }));
  mast.position.set(0.27, 0, 0.40);      // stereo camera bar (URDF: camera_x 0.27, ~0.40 m above ground)
  robot.add(mast);
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.3, 0, 0.25), 0.5, 0xffd23e, 0.12, 0.08);
  robot.add(arrow);
}
scene.add(robot);

// camera frustum (80 deg HFOV, 4:3, out to the trusted stereo range)
const frustum = (() => {
  const range = 6, hf = (80 * Math.PI) / 360, vf = Math.atan(Math.tan(hf) * 0.75);
  const w = Math.tan(hf) * range, h = Math.tan(vf) * range;
  const o = [0.27, 0, 0.40];
  const c = [[range, w, h], [range, -w, h], [range, -w, -h], [range, w, -h]].map(([x, y, z]) => [o[0] + x, o[1] + y, o[2] + z]);
  const v = [];
  for (let i = 0; i < 4; i++) { v.push(...o, ...c[i]); v.push(...c[i], ...c[(i + 1) % 4]); }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
  const l = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: 0x4fb3ff, transparent: true, opacity: 0.4 }));
  l.visible = false;
  robot.add(l);
  return l;
})();

// ---------------------------------------------------------------- trajectory, plan, goal
const trajPos = new Float32Array(TRAJ_CAP * 3);
const trajGeo = new THREE.BufferGeometry();
trajGeo.setAttribute('position', new THREE.BufferAttribute(trajPos, 3).setUsage(THREE.DynamicDrawUsage));
trajGeo.setDrawRange(0, 0);
const trajLine = new THREE.Line(trajGeo, new THREE.LineBasicMaterial({ color: 0xffd23e }));
trajLine.frustumCulled = false;
scene.add(trajLine);
let nTraj = 0;

function trajReset(pts) {
  nTraj = 0;
  for (const p of pts) trajAdd(p);
}
function trajAdd(p) {
  if (nTraj >= TRAJ_CAP) {                       // drop the oldest half
    trajPos.copyWithin(0, (TRAJ_CAP / 2) * 3);
    nTraj = TRAJ_CAP / 2;
  }
  trajPos.set(p, nTraj * 3);
  nTraj++;
  trajGeo.setDrawRange(0, nTraj);
  trajGeo.attributes.position.needsUpdate = true;
}

const planPos = new Float32Array(PLAN_CAP * 3);
const planGeo = new THREE.BufferGeometry();
planGeo.setAttribute('position', new THREE.BufferAttribute(planPos, 3).setUsage(THREE.DynamicDrawUsage));
planGeo.setDrawRange(0, 0);
const planLine = new THREE.Line(planGeo, new THREE.LineBasicMaterial({ color: 0x2fd4ff }));
planLine.frustumCulled = false;
scene.add(planLine);

const goal = new THREE.Group();
goal.add(new THREE.Mesh(new THREE.RingGeometry(0.25, 0.32, 32), new THREE.MeshBasicMaterial({ color: 0x5ed38a, side: THREE.DoubleSide })));
{
  const pole = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 1.2)]),
    new THREE.LineBasicMaterial({ color: 0x5ed38a }));
  goal.add(pole);
}
goal.visible = false;
let goalSet = false;
scene.add(goal);

// ---------------------------------------------------------------- websocket
const state = { pose: null, stats: null };
let voxel = 0.05;
function updateScale() {
  // pixels covered by one voxel at 1 m distance
  const h = renderer.getDrawingBufferSize(new THREE.Vector2()).y;
  const px = (voxel * h) / (2 * Math.tan((camera.fov * Math.PI) / 360));
  mapMat.uniforms.uScale.value = px;
  liveMat.uniforms.uScale.value = px;
}
let ws = null;

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => setLink(true);
  ws.onclose = () => { setLink(false); setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    if (typeof ev.data !== 'string') {
      const t = msgType(ev.data);
      if (t === MSG_MAP_DELTA) applyMapDelta(ev.data);
      else if (t === MSG_LIVE) applyLive(ev.data);
      else if (t === MSG_TIER1) applyTier1(ev.data);
      return;
    }
    const m = JSON.parse(ev.data);
    switch (m.type) {
      case 'hello':
        if (m.max_points !== mapCap) allocMap(m.max_points);
        else { nMap = 0; mapGeo.setDrawRange(0, 0); dirty = { lo: Infinity, hi: -1 }; }
        trajReset([]);
        voxel = m.voxel || voxel;
        updateScale();
        break;
      case 'state':
        state.pose = m.pose || state.pose; state.stats = m.stats;
        if (m.pose) {
          robot.position.set(m.pose[0], m.pose[1], m.pose[2]);
          robot.quaternion.set(m.pose[3], m.pose[4], m.pose[5], m.pose[6]);
        }
        break;
      case 'traj_set': trajReset(m.pts); break;
      case 'traj_add': for (const p of m.pts) trajAdd(p); break;
      case 'plan': {
        const n = Math.min(m.pts.length, PLAN_CAP);
        for (let i = 0; i < n; i++) planPos.set(m.pts[i], i * 3);
        planGeo.setDrawRange(0, n); planGeo.attributes.position.needsUpdate = true;
        break;
      }
      case 'goal': goal.position.set(...m.pos); goalSet = true; goal.visible = $('lPlan').checked; break;
      case 'cleared':
        if (m.what === 'map') { nMap = 0; mapGeo.setDrawRange(0, 0); dirty = { lo: Infinity, hi: -1 }; }
        if (m.what === 'traj') { nTraj = 0; trajGeo.setDrawRange(0, 0); }
        break;
    }
  };
}
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }
function setLink(on) {
  $('dotWs').classList.toggle('on', on);
  $('vWs').textContent = on ? 'connected' : 'reconnecting';
}

// ---------------------------------------------------------------- UI wiring
const bind = (id, fn) => { const el = $(id); const h = () => fn(el); el.addEventListener('input', h); h(); };
bind('mode', (e) => { mapMat.uniforms.uMode.value = +e.value; liveMat.uniforms.uMode.value = +e.value; });
bind('obstOnly', (e) => { mapMat.uniforms.uObstOnly.value = e.checked ? 1 : 0; });
bind('size', (e) => {
  const s = +e.value;
  mapMat.uniforms.uSize.value = s; liveMat.uniforms.uSize.value = s * 1.3;
  tier1.material.size = voxel * 1.8 * s;
});
bind('lMap', (e) => { mapPoints.visible = e.checked; });
bind('lLive', (e) => { livePoints.visible = e.checked; });
bind('lTier1', (e) => { tier1.visible = e.checked; });
bind('lTraj', (e) => { trajLine.visible = e.checked; });
bind('lPlan', (e) => { planLine.visible = e.checked; goal.visible = e.checked && goalSet; });
bind('lRobot', (e) => { robot.visible = e.checked; });
bind('lFrustum', (e) => { frustum.visible = e.checked; });
bind('lGrid', (e) => { grid.visible = e.checked; });
$('clearMap').onclick = () => send({ cmd: 'clear_map' });
$('clearTraj').onclick = () => send({ cmd: 'clear_traj' });

// camera presets
const defaultOffset = new THREE.Vector3(-4, -4, 3);
function viewChase() {
  const yaw = new THREE.Euler().setFromQuaternion(robot.quaternion).z;
  const off = new THREE.Vector3(-4.5, 0, 2.6).applyAxisAngle(new THREE.Vector3(0, 0, 1), yaw);
  camera.position.copy(controls.target).add(off);
}
function viewTop() { camera.position.copy(controls.target).add(new THREE.Vector3(0.001, 0, 14)); }
function viewReset() { controls.target.set(0, 0, 0); camera.position.copy(defaultOffset); $('follow').checked = false; }
$('vChase').onclick = viewChase; $('vTop').onclick = viewTop; $('vReset').onclick = viewReset;
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'SELECT' || e.ctrlKey || e.metaKey) return;
  if (e.key === 'f') $('follow').click();
  if (e.key === 'c') viewChase();
  if (e.key === 't') viewTop();
  if (e.key === 'r') viewReset();
});
camera.position.copy(defaultOffset);
camera.up.set(0, 0, 1);
controls.update();

// double-click on the ground -> goal (opt-in)
const raycaster = new THREE.Raycaster();
renderer.domElement.addEventListener('dblclick', (e) => {
  if (!$('goalMode').checked) return;
  const ndc = new THREE.Vector2((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
  const z = state.pose ? state.pose[2] : 0;
  const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -z);
  const hit = new THREE.Vector3();
  if (raycaster.ray.intersectPlane(plane, hit)) send({ cmd: 'goal', x: hit.x, y: hit.y });
});

// ---------------------------------------------------------------- loop + HUD
const follow = $('follow');
const lastRobot = new THREE.Vector3();
let frames = 0, fpsT0 = performance.now(), fps = 0, hadPose = false;

function updateHud() {
  const s = state.stats;
  $('vFps').textContent = fps.toFixed(0);
  if (!s) return;
  $('vPts').textContent = `${s.points.toLocaleString()} / ${(s.max_points / 1000).toFixed(0)}k`;
  $('vRate').textContent = `${s.rate.toFixed(1)} Hz`;
  $('vProc').textContent = `${s.proc_ms.toFixed(0)} ms`;
  $('vFrame').textContent = s.fixed_frame;
  $('dotTf').classList.toggle('on', s.tf_ok);
  $('vTf').textContent = s.tf_ok ? 'ok' : 'waiting';
  if (state.pose) $('vPose').textContent = `${state.pose[0].toFixed(2)}, ${state.pose[1].toFixed(2)}`;
  const warn = $('warn');
  let msg = '';
  if (!s.tf_ok) msg = `No TF to '${s.fixed_frame}' yet. Is the simulation running?`;
  else if (s.rate === 0) msg = 'No /points2 data. Is stereo_image_proc running?';
  else if (s.points >= s.max_points) msg = 'Map is full - new areas are not added.';
  warn.style.display = msg ? 'block' : 'none';
  warn.textContent = msg;
}
setInterval(updateHud, 500);

function loop() {
  requestAnimationFrame(loop);
  flushMap();
  if (state.pose && !hadPose) { lastRobot.copy(robot.position); hadPose = true; }
  if (follow.checked && state.pose) {
    const p = robot.position;
    const d = p.clone().sub(lastRobot);
    controls.target.add(d);
    camera.position.add(d);
    lastRobot.copy(p);
  } else if (state.pose) {
    lastRobot.copy(robot.position);
  }
  // keep the grid under the robot so it never runs out
  grid.position.set(Math.round(controls.target.x), Math.round(controls.target.y), robot.position.z);
  controls.update();
  renderer.render(scene, camera);
  frames++;
  const now = performance.now();
  if (now - fpsT0 > 1000) { fps = (frames * 1000) / (now - fpsT0); frames = 0; fpsT0 = now; }
}

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
  updateScale();
});

updateScale();
connect();
loop();
