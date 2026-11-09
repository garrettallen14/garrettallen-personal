// Sail drone 3D viewer, built on our plate viewer (same renderer, lights, controls, look).
// One source for the public page (gallen.dev/sail-drone/) and the internal copy. Reads window.SD = {glb, json}.
// three.js r160 from the import map. No network calls beyond those two files.
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {toCreasedNormals} from 'three/addons/utils/BufferGeometryUtils.js';

const CFG = window.SD || {glb: 'boat.glb', json: 'boat.json'};
const MUT = new URLSearchParams(location.search).get('mut') || '';   // test only: group = drag carries the part's group; joint = drag keeps joint constraints; orbit = press on a part orbits; nocost = no cost line
const $ = id => document.getElementById(id), V3 = THREE.Vector3, D2R = Math.PI / 180;
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));
const status = $('status');
function setStatus(msg) { if (!msg) { status.style.display = 'none'; return; } status.style.display = 'flex'; status.firstElementChild.textContent = msg; }

// ---------- renderer / scene (plate viewer)
const mount = $('mount');
const renderer = new THREE.WebGLRenderer({antialias: true, alpha: true, powerPreference: 'high-performance'});
renderer.setPixelRatio(devicePixelRatio || 1);
renderer.outputColorSpace = THREE.SRGBColorSpace;
mount.appendChild(renderer.domElement);
const cv = renderer.domElement;
const scene = new THREE.Scene();
scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.55));
const sun = new THREE.DirectionalLight(0xffffff, 1.5); sun.position.set(-120, 380, 220); scene.add(sun);      // plate sun, Y-up frame
const fill = new THREE.DirectionalLight(0xdfe8ff, 0.55); fill.position.set(300, 120, -260); scene.add(fill);
const cam = new THREE.PerspectiveCamera(32, 1, 1, 80000);
const ctl = new OrbitControls(cam, cv);
ctl.enableDamping = false; ctl.zoomSpeed = 1.2; ctl.screenSpacePanning = true;
ctl.touches = {ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN};
let needs = false, frames = 0;
function draw() { if (needs) return; needs = true; requestAnimationFrame(() => { needs = false; renderer.render(scene, cam); frames++; }); }
const redraw = draw, syncSliders = () => {};
ctl.addEventListener('change', draw);
function resize() { const w = innerWidth, h = Math.max(1, innerHeight); renderer.setSize(w, h, false); cam.aspect = w / h; cam.updateProjectionMatrix(); draw(); }
new ResizeObserver(resize).observe(document.body); resize();

// ---------- tweens (camera moves, reset parts); a frame is drawn per tick only while one runs
const tweens = []; let ticking = false;
const ease = t => t < .5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
function tick(now) { for (let i = tweens.length - 1; i >= 0; i--) { const w = tweens[i], t = Math.min(1, (now - w.t0) / w.ms); w.fn(ease(t)); if (t >= 1) { tweens.splice(i, 1); w.res(); } }
  draw(); if (tweens.length) requestAnimationFrame(tick); else ticking = false; }
function tween(ms, fn) { return new Promise(res => { tweens.push({t0: performance.now(), ms, fn, res}); if (!ticking) { ticking = true; requestAnimationFrame(tick); } }); }

// ---------- load
const glbP = new Promise((res, rej) => new GLTFLoader().load(CFG.glb, res, e => setStatus(`Loading boat… ${e.total ? Math.round(e.loaded / e.total * 100) + '%' : (e.loaded / 1e6).toFixed(1) + ' MB'}`), rej));
const [D, G] = await Promise.all([fetch(CFG.json).then(r => r.json()), glbP]);
const byId = Object.fromEntries(D.parts.map(p => [p.id, p])), nodes = {}, meshesOf = {}, colorOf = {};
const cleanName = n => String(n || '').replace(/^#\d+\s*/, '');
const shortName = n => { const c = cleanName(n), m = c.split(/[:(;,]/)[0].trim(); return m.length > 2 ? m : c; };

const all = [], unmapped = [];   // a mesh with no part entry is a model bug: counted, reported by the tests
G.scene.traverse(o => { if (!o.isMesh) return; let id = null; for (let q = o; q; q = q.parent) if (byId[q.name]) { id = q.name; nodes[id] ||= q; break; }
  if (!id) { unmapped.push(o.name || '(unnamed mesh)'); return; }
  const src = Array.isArray(o.material) ? o.material[0] : o.material, col = src && src.color ? src.color.clone() : new THREE.Color(0x9aa2aa);
  let g = o.geometry; if (g.attributes.color) g.deleteAttribute('color'); o.geometry = toCreasedNormals(g, THREE.MathUtils.degToRad(32));
  o.material = new THREE.MeshStandardMaterial({color: col, roughness: 0.62, metalness: 0.0, side: THREE.DoubleSide});
  o.userData.id = id; (meshesOf[id] ||= []).push(o); all.push(o); colorOf[id] ||= '#' + col.getHexString(); });
// every part node hangs straight off the model root, so moving one part never carries another (no parent/child carry)
G.scene.updateMatrixWorld(true); for (const id in nodes) if (nodes[id].parent !== G.scene) G.scene.attach(nodes[id]);
scene.add(G.scene); G.scene.updateMatrixWorld(true);
const box0 = new THREE.Box3().setFromObject(G.scene), C0 = box0.getCenter(new V3()), S0 = box0.getSize(new V3());

// ---------- joints (model contract): each part's joint transforms compose in list order in the GLB view frame;
// follower parts take their table matrix (home pose -> posed) instead; requires = removal order.
function mockJoints() {
  const W = D.wing; if (!W) return [];
  return [{id: 'wing', label: 'Wing angle', type: 'revolute', axis: [0, 1, 0], origin: W.pivot_view, min: W.stops_deg[0], max: W.stops_deg[1], default: 0, unit: 'deg', parts: W.turning}];
}
const JOINTS = (D.joints && D.joints.length ? D.joints : mockJoints()).map(j => ({...j})), jointsMock = !(D.joints && D.joints.length);
const J = Object.fromEntries(JOINTS.map(j => [j.id, j])), jv = {};
for (const j of JOINTS) jv[j.id] = j.default ?? 0;
const jointsOf = {}; for (const j of JOINTS) for (const id of j.parts || []) if (nodes[id]) (jointsOf[id] ||= []).push(j);
const homeW = {}, parentInv = {}, off = {}; let explodeF = 0;
for (const id in nodes) { const n = nodes[id]; n.updateWorldMatrix(true, false); homeW[id] = n.matrixWorld.clone();
  parentInv[id] = n.parent ? n.parent.matrixWorld.clone().invert() : new THREE.Matrix4(); n.matrixAutoUpdate = false; off[id] = {step: new V3(), man: new V3()}; }
const _p = new V3(), _q = new THREE.Quaternion(), _s = new V3(), _p2 = new V3(), _q2 = new THREE.Quaternion(), _s2 = new V3();
function tableAt(table, v) {
  const T = table.slice().sort((a, b) => a[0] - b[0]); let i = 0; while (i < T.length - 2 && v > T[i + 1][0]) i++;
  const a = T[i], b = T[Math.min(i + 1, T.length - 1)], f = b[0] === a[0] ? 0 : Math.max(0, Math.min(1, (v - a[0]) / (b[0] - a[0])));
  const A = new THREE.Matrix4().set(...a[1]), B = new THREE.Matrix4().set(...b[1]);
  A.decompose(_p, _q, _s); B.decompose(_p2, _q2, _s2);
  return new THREE.Matrix4().compose(_p.clone().lerp(_p2, f), _q.clone().slerp(_q2, f), _s.clone().lerp(_s2, f));
}
function jointM(j) {
  if (j.follows) return tableAt(j.follows.table, jv[j.follows.joint] ?? 0);
  const v = jv[j.id], ax = new V3(...(j.axis || [0, 1, 0])).normalize();
  if (j.type === 'prismatic') return new THREE.Matrix4().makeTranslation(ax.multiplyScalar(v));
  const o = new V3(...(j.origin || [0, 0, 0]));
  return new THREE.Matrix4().makeTranslation(o.x, o.y, o.z).multiply(new THREE.Matrix4().makeRotationAxis(ax, (j.unit === 'rad' ? 1 : D2R) * v)).multiply(new THREE.Matrix4().makeTranslation(-o.x, -o.y, -o.z));
}
function placeAll() {
  const M = {};
  for (const id in nodes) { const W = new THREE.Matrix4();
    for (const j of jointsOf[id] || []) W.premultiply(M[j.id] ||= jointM(j));
    const o = off[id], p = byId[id], t = o.step.clone().add(o.man); if (p.explode && explodeF) t.addScaledVector(new V3(...p.explode), explodeF);
    W.premultiply(new THREE.Matrix4().makeTranslation(t.x, t.y, t.z)).multiply(homeW[id]);
    nodes[id].matrix.copy(parentInv[id]).multiply(W); nodes[id].matrixWorldNeedsUpdate = true; }
  scene.updateMatrixWorld(true); redraw();
}
const near = (a, b) => Math.abs(a - b) <= 1e-6 + 1e-3 * Math.abs(b);
const REQ_OFF = MUT === 'requires';                       // test only: removal order ignored (the requires test must go red)
const defOf = id => J[id].default ?? 0;
function blockedBy(id) { if (REQ_OFF) return []; return Object.entries(J[id].requires || {}).filter(([k, v]) => J[k] && !near(jv[k], v)).map(([k]) => k); }
function heldBy(id) { if (REQ_OFF) return []; return JOINTS.filter(d => d.requires && id in d.requires && !near(jv[d.id], defOf(d.id))).map(d => d.id); }
const depthMemo = {};
function depth(id) { if (id in depthMemo) return depthMemo[id]; depthMemo[id] = 0; let d = 0; for (const k of Object.keys(J[id].requires || {})) if (J[k]) d = Math.max(d, depth(k) + 1); return depthMemo[id] = d; }
function setJoint(id, v, force) {
  const j = J[id]; if (!j || j.follows) return false;
  if (!force && (blockedBy(id).length || heldBy(id).length)) { syncSliders(); return false; }
  jv[id] = Math.max(j.min ?? -1e9, Math.min(j.max ?? 1e9, +v)); placeAll(); syncSliders(); return true;
}
const fmtJ = j => (+jv[j.id]).toFixed(j.unit === 'mm' ? 0 : 1) + (j.unit === 'deg' || !j.unit ? '°' : ' ' + j.unit);
const shortLabel = j => String(j.label || j.id).split(' (')[0];
function tweenOne(id, v, ms = 450) { const a = jv[id]; return near(a, v) ? Promise.resolve() : tween(ms, t => setJoint(id, a + (v - a) * t, true)); }
async function satisfy(id) { for (const [k, v] of Object.entries(J[id].requires || {})) { if (!J[k]) continue; await satisfy(k); await release(k, v); await tweenOne(k, v); } }
async function release(id) { for (const d of JOINTS) if (d.requires && id in d.requires && !near(jv[d.id], defOf(d.id))) { await release(d.id); await tweenOne(d.id, defOf(d.id)); } }
async function tweenJoints(target, ms = 650) {         // step state: listed joints to their value, unlisted to default; removal order kept
  const want = {}; for (const j of JOINTS) if (!j.follows) want[j.id] = target && j.id in target ? +target[j.id] : defOf(j.id);
  const ch = Object.keys(want).filter(id => !near(jv[id], want[id])); if (!ch.length) return;
  const back = ch.filter(id => Math.abs(want[id] - defOf(id)) < Math.abs(jv[id] - defOf(id))), fwd = ch.filter(id => !back.includes(id));
  const groups = (ids, desc) => { const m = {}; for (const id of ids) (m[depth(id)] ||= []).push(id); return Object.keys(m).map(Number).sort((a, b) => desc ? b - a : a - b).map(k => m[k]); };
  const phases = [...groups(back, true), ...groups(fwd, false)], per = Math.max(260, ms / phases.length);
  for (const g of phases) { const from = Object.fromEntries(g.map(id => [id, jv[id]])); await tween(per, t => { for (const id of g) setJoint(id, from[id] + (want[id] - from[id]) * t, true); }); }
}
placeAll();


// ---------- camera presets (plate viewer setView, Y-up)
const VDIR = (() => { const v = (D.views || []).find(x => x.id === 'whole'); return v ? new V3(...v.eye).sub(new V3(...v.target)).normalize().toArray() : [0.55, 0.35, 0.75]; })();
const DIRS = {iso: VDIR, top: [0.0001, 1, 0.0001], side: [0, 0.12, 1]};
let lastDir = 'iso';
function viewFor(v, frameBox) {
  if (v === 'fit') v = lastDir;
  const b = frameBox || box0, target = b.getCenter(new V3()), size = b.getSize(new V3());
  const d = new V3(...(DIRS[v] || DIRS.iso)).normalize(); lastDir = DIRS[v] ? v : 'iso';
  // plate viewer's fit assumes a flat plate; a tall boat needs its real silhouette: project the 8 box corners, scale until they fill ~84% of the screen
  let dist = size.length() * 1.5; const pc = cam.clone(), corner = new V3();
  for (let it = 0; it < 4; it++) {
    pc.position.copy(target).addScaledVector(d, dist); pc.lookAt(target); pc.near = dist / 100; pc.far = dist * 10; pc.updateProjectionMatrix(); pc.updateMatrixWorld();
    let ext = 0; for (let k = 0; k < 8; k++) { corner.set(k & 1 ? b.max.x : b.min.x, k & 2 ? b.max.y : b.min.y, k & 4 ? b.max.z : b.min.z).project(pc); ext = Math.max(ext, Math.abs(corner.x), Math.abs(corner.y)); }
    dist *= ext / 0.84;
  }
  return {eye: target.clone().addScaledVector(d, dist), target, dist};
}
function setView(v, frameBox, ms = 0) {
  const f = viewFor(v, frameBox); cam.near = Math.max(0.5, f.dist / 200); cam.far = f.dist * 30; cam.updateProjectionMatrix();
  if (!ms) { cam.position.copy(f.eye); ctl.target.copy(f.target); ctl.update(); draw(); return Promise.resolve(); }
  const e0 = cam.position.clone(), t0 = ctl.target.clone();
  return tween(ms, t => { cam.position.lerpVectors(e0, f.eye, t); ctl.target.lerpVectors(t0, f.target, t); ctl.update(); });
}
function frameOf(id) { const b = new THREE.Box3(); for (const m of meshesOf[id] || []) b.expandByObject(m); return b; }
document.querySelectorAll('#views button[data-v]').forEach(b => b.addEventListener('click', () => setView(b.dataset.v)));

// ---------- selection (plate viewer tint) + hover tooltip + tap label
function tint(id, k) { if (!id || !meshesOf[id]) return; for (const m of meshesOf[id]) m.material.emissive.setRGB(0, k * 0.45, k * 0.2); }
let selected = null, hovered = null;
function hover(id) { if (id === hovered) return; if (hovered && hovered !== selected) tint(hovered, 0); hovered = id; if (id && id !== selected) tint(id, 0.35); draw(); }
const tip = $('tip');
function select(id, ev) {
  if (selected) tint(selected, 0);
  selected = id; if (id) tint(id, 0.7);
  if (hovered && hovered !== selected) tint(hovered, 0.35);
  if (id && ev) showTip(id, ev.clientX, ev.clientY); else hideTip();
  renderList(); draw();
}
function showTip(id, x, y) {
  const L = labelParts(id);
  tip.innerHTML = `<b>${esc(L.line1)}</b>` + (L.line2 ? `<small>${esc(L.line2)}</small>` : '');
  tip.classList.add('lab'); tip.style.left = Math.max(125, Math.min(x, innerWidth - 125)) + 'px'; tip.style.top = Math.max(70, y) + 'px'; tip.style.display = 'block';
}
function hideTip() { tip.style.display = 'none'; tip.classList.remove('lab'); }
// cost shows only when the json carries cost data (the public copy is scrubbed of it, so it shows name + weight)
const COST_ON = MUT !== 'nocost' && D.parts.some(p => 'cost_usd' in p || 'price' in p);
const plainName = p => p.label || (/^[^:]*:\s*small parts/i.test(p.name || "") ? shortName(p.name) + " components" : shortName(p.name));
const grams = p => { const g = p.kg != null ? p.kg * 1000 : null; return g == null ? '' : g >= 1000 ? (g / 1000).toFixed(1) + ' kg' : g >= 10 ? g.toFixed(0) + ' g' : g.toFixed(1) + ' g'; };
function costOf(p) {               // [line-1 cost text, line-2 note]
  if ('cost_usd' in p) {
    if (p.cost_usd == null) return ['price pending', p.cost_note || ''];
    if (+p.cost_usd === 0 && /have|own/i.test(p.cost_note || '')) return ['$0 · you have it', p.cost_note || ''];
    return ['$' + (+p.cost_usd).toFixed(2), p.cost_note || ''];
  }
  const s = String(p.price ?? '').trim();          // older json: the parts-table price string, shown as written
  if (/^HAVE\b/.test(s)) return ['$0 · you have it', ''];
  const m = s.match(/^\$[^(]*/); if (m) return [m[0].trim().slice(0, 40), 'parts-table price, as written'];
  return ['price pending', ''];
}
function labelParts(id) { const p = byId[id], c = COST_ON ? costOf(p) : null; return {line1: [plainName(p), c && c[0], grams(p)].filter(Boolean).join(' · '), line2: c ? c[1] : ''}; }

const JOINT_MUT = MUT === 'joint', GROUP_MUT = MUT === 'group';
function moverFor(id) {
  const js = jointsOf[id] || [];
  const cands = []; for (const j of js) { const d = j.follows ? J[j.follows.joint] : j; if (d && !d.follows && !cands.includes(d)) cands.push(d); }
  for (const j of cands) if (!blockedBy(j.id).length && !heldBy(j.id).length) return {kind: 'joint', j};
  if (cands.length) return null;                        // jointed but locked right now: it just doesn't move
  const e = byId[id].explode; if (e && Math.hypot(...e) > 1) return {kind: 'slide', dir: new V3(...e).normalize(), max: Math.hypot(...e) * 1.2};
  return null;
}
const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
function rayAt(x, y) { const r = cv.getBoundingClientRect(); ndc.set((x - r.left) / r.width * 2 - 1, -(y - r.top) / r.height * 2 + 1); ray.setFromCamera(ndc, cam); return ray.ray.clone(); }
function pick(x, y) { rayAt(x, y); const h = ray.intersectObjects(all.filter(m => m.visible), false)[0]; return h || null; }
function closestT(r, o, d) {        // parameter t on the line o + t d nearest the pointer ray
  const w = o.clone().sub(r.origin), b = d.dot(r.direction), den = 1 - b * b; if (den < 1e-4) return null;
  return (b * w.dot(r.direction) - w.dot(d)) / den;
}
function angleAt(r, o, ax, ref) {    // angle of the ray's hit on the plane through o normal ax, measured from ref
  const den = r.direction.dot(ax); if (Math.abs(den) < 0.08) return null;
  const p = r.origin.clone().addScaledVector(r.direction, o.clone().sub(r.origin).dot(ax) / den).sub(o);
  const u = ref.clone(), v = ax.clone().cross(u); return Math.atan2(p.dot(v), p.dot(u));
}
let drag = null, down = null; const ptrs = new Set();
function startDrag(e, h) {
  const id = h.object.userData.id, m = JOINT_MUT ? moverFor(id) : {kind: 'free'}; if (!m) return {id, none: true};
  const r = rayAt(e.clientX, e.clientY), d = {id, m, x0: e.clientX, y0: e.clientY};
  if (m.kind === 'free') { d.plane = new THREE.Plane().setFromNormalAndCoplanarPoint(cam.getWorldDirection(new V3()), h.point); d.p0 = h.point.clone();
    d.ids = GROUP_MUT ? Object.keys(nodes).filter(k => byId[k].group && byId[k].group === byId[id].group) : [id];
    d.man0 = Object.fromEntries(d.ids.map(k => [k, off[k].man.clone()])); }
  else if (m.kind === 'slide') { d.o = h.point.clone(); d.dir = m.dir; d.t0 = closestT(r, d.o, d.dir) ?? 0; d.man0 = off[id].man.clone(); }
  else if (m.j.type === 'prismatic') { d.o = h.point.clone(); d.dir = new V3(...m.j.axis).normalize(); d.t0 = closestT(r, d.o, d.dir) ?? 0; d.v0 = jv[m.j.id]; }
  else { d.o = new V3(...(m.j.origin || [0, 0, 0])); d.ax = new V3(...(m.j.axis || [0, 1, 0])).normalize(); d.ref = h.point.clone().sub(d.o).projectOnPlane(d.ax).normalize();
    if (d.ref.lengthSq() < 1e-6) d.ref = new V3(1, 0, 0); d.a0 = angleAt(r, d.o, d.ax, d.ref) ?? 0; d.v0 = jv[m.j.id]; }
  return d;
}
function moveDrag(e) {
  const d = drag, m = d.m, r = rayAt(e.clientX, e.clientY);
  if (m.kind === 'free') { const p = r.intersectPlane(d.plane, new V3()); if (p) { p.sub(d.p0); for (const k of d.ids) off[k].man.copy(d.man0[k]).add(p); placeAll(); } return; }
  if (m.kind === 'slide') { const t = closestT(r, d.o, d.dir); if (t == null) return;
    const cur = d.man0.dot(d.dir), nv = Math.max(0, Math.min(m.max, cur + t - d.t0)); off[d.id].man.copy(d.dir).multiplyScalar(nv); placeAll(); return; }
  if (m.j.type === 'prismatic') { const t = closestT(r, d.o, d.dir); if (t == null) return; setJoint(m.j.id, d.v0 + (t - d.t0), true); return; }
  let a = angleAt(r, d.o, d.ax, d.ref); const deg = a == null ? (e.clientX - d.x0) * 0.4 : (a - d.a0) / D2R;
  setJoint(m.j.id, d.v0 + deg, true);
}

// ---------- gestures. Press on a part and drag: that one part follows the pointer (camera-facing plane through the grab point).
// Press on empty space: orbit. Click without moving: label. Two fingers: pinch / pan.
const ORBIT_MUT = MUT === 'orbit';
let replay = false, first = null, downAt = null;
addEventListener('pointerdown', e => {
  if (e.target !== cv || replay) return; ptrs.add(e.pointerId);
  if (ptrs.size > 1) {        // second finger: pinch/pan. If the first finger was moving a part, hand it to the orbit controls.
    if (drag) { drag = null; ctl.enabled = true;
      if (first) { replay = true; cv.dispatchEvent(new PointerEvent('pointerdown', {pointerId: first.pointerId, pointerType: first.pointerType, isPrimary: true, clientX: first.x, clientY: first.y, button: 0, buttons: 1, bubbles: true, cancelable: true})); replay = false; } }
    downAt = null; return; }
  first = {pointerId: e.pointerId, pointerType: e.pointerType, x: e.clientX, y: e.clientY};
  downAt = [e.clientX, e.clientY];
  const h = pick(e.clientX, e.clientY);
  if (h && !ORBIT_MUT) { const d = startDrag(e, h); if (!d.none) { drag = d; ctl.enabled = false; cv.style.cursor = 'grabbing'; } }
}, true);
addEventListener('pointermove', e => {
  if (e.target === cv && e.pointerType !== 'touch' && !e.buttons && !ptrs.size) {        // desktop hover tooltip
    const h = pick(e.clientX, e.clientY), id = h ? h.object.userData.id : null; hover(id); cv.style.cursor = id && !ORBIT_MUT ? 'grab' : '';
    if (id && id !== selected) { tip.classList.remove('lab'); tip.textContent = plainName(byId[id]); tip.style.left = e.clientX + 'px'; tip.style.top = e.clientY + 'px'; tip.style.display = 'block'; }
    else if (!tip.classList.contains('lab')) tip.style.display = 'none';
    return; }
  if (!ptrs.has(e.pointerId)) return; if (first && e.pointerId === first.pointerId) { first.x = e.clientX; first.y = e.clientY; }
  if (drag && downAt && Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4) { if (tip.style.display === 'block') hideTip(); moveDrag(e); }
}, true);
function endPtr(e) {
  if (!ptrs.has(e.pointerId)) return; ptrs.delete(e.pointerId);
  if (drag) cv.style.cursor = 'grab'; drag = null; ctl.enabled = true;
  if (!downAt) return; const moved = Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]); downAt = null;
  if (moved > 4) return;
  const h = pick(e.clientX, e.clientY), id = h ? h.object.userData.id : null;
  select(id && id !== selected ? id : null, e);
}
addEventListener('pointerup', endPtr, true); addEventListener('pointercancel', endPtr, true);
cv.addEventListener('pointerleave', () => { if (!tip.classList.contains('lab')) tip.style.display = 'none'; hover(null); });

// ---------- side panel (List chip): rows grouped by system, tap = select + frame
const SYS = [['hull', 'Hull'], ['fin', 'Fin'], ['wing', 'Mast + wing'], ['rudder', 'Rudder'], ['electronics', 'Electronics'], ['power', 'Power + solar'], ['other', 'Other']];
const GROUP_SYS = {hull: 'hull', tray: 'electronics', fin: 'fin', wing: 'wing', rig: 'wing', masthead: 'wing', stern: 'rudder', plate: 'electronics', window: 'electronics', power: 'power', panels: 'power'};
const sysOf = p => p.system || GROUP_SYS[p.group] || 'other';
function showSide(on) { $('side').classList.toggle('hide', !on); $('sideTab').classList.toggle('hide', on); if (on) renderList(); }
$('sideClose').onclick = () => showSide(false); $('sideOpen').onclick = () => showSide(true);
function renderList() {
  if ($('side').classList.contains('hide')) return;
  const groups = {}; for (const p of D.parts) if (nodes[p.id]) (groups[sysOf(p)] ||= []).push(p);
  const body = $('sideBody'); body.innerHTML = '';
  for (const [k, lab] of SYS) { if (!groups[k]) continue;
    const h = document.createElement('div'); h.className = 'grp'; h.textContent = lab; body.appendChild(h);
    for (const p of groups[k]) { const r = document.createElement('div'); r.className = 'row' + (p.id === selected ? ' sel' : ''); r.dataset.id = p.id;
      r.innerHTML = `<span class="sw" style="background:${esc(colorOf[p.id])}"></span><span class="nm">${esc(plainName(p))}</span><span class="r">${esc(grams(p))}</span>`;
      r.onmouseenter = () => hover(p.id); r.onmouseleave = () => hover(null);
      r.onclick = () => { select(p.id); const b = frameOf(p.id);
        setView(lastDir, b.clone().expandByScalar(Math.max(40, b.getSize(new V3()).length() * 0.6)), 450).then(() => {
          if (selected !== p.id) return; const c = frameOf(p.id).getCenter(new V3()).project(cam); showTip(p.id, (c.x + 1) / 2 * innerWidth, (1 - c.y) / 2 * innerHeight); }); };
      body.appendChild(r); } }
  const s = body.querySelector('.row.sel'); if (s) s.scrollIntoView({block: 'nearest'});
}

// ---------- reset parts: every moved part back home (moves are never saved; a reload shows the real boat)
function resetParts() {
  const m0 = Object.fromEntries(Object.keys(off).map(id => [id, off[id].man.clone()])), j0 = {...jv};
  return tween(450, t => { for (const id in off) off[id].man.copy(m0[id]).multiplyScalar(1 - t);
    for (const j of JOINTS) if (!j.follows) jv[j.id] = j0[j.id] + (defOf(j.id) - j0[j.id]) * t;
    explodeF = 0; placeAll(); }).then(() => { for (const id in off) off[id].man.set(0, 0, 0); for (const j of JOINTS) if (!j.follows) jv[j.id] = defOf(j.id); placeAll(); });
}
$('resetParts').onclick = resetParts;
addEventListener('keydown', e => { const k = e.key; if (k === '0') setView('iso'); else if (k === '1') setView('top'); else if (k === '3') setView('side'); else if (k === 'f' || k === 'F') setView('fit'); });

// ---------- start
setView('iso'); setStatus('');

// ---------- test hooks
window._sd = {
  ready: true, busy: () => tweens.length > 0, parts: () => D.parts.length, meshes: () => all.length,
  cam: () => ({pos: cam.position.toArray(), target: ctl.target.toArray(), dist: cam.position.distanceTo(ctl.target)}),
  jointVal: id => jv[id], joint: (id, v) => setJoint(id, v, true), mover: id => { const m = moverFor(id); return m ? (m.kind === 'joint' ? 'joint:' + m.j.id : m.kind) : null; },
  pickAt: (x, y) => { const h = pick(x, y); return h ? h.object.userData.id : null; },
  worldOf: id => { const b = frameOf(id); return b.isEmpty() ? null : b.getCenter(new V3()).toArray(); },
  project: p => { const v = new V3(...p).project(cam); return [(v.x + 1) / 2 * innerWidth, (1 - v.y) / 2 * innerHeight]; },
  label: () => tip.style.display === 'block' && tip.classList.contains('lab') ? tip.innerText : null, sel: () => selected,
  matrices: () => Object.fromEntries(Object.keys(nodes).map(id => [id, nodes[id].matrixWorld.toArray()])), unmapped: () => unmapped.slice(), costOn: () => COST_ON, labelFor: id => labelParts(id), partIds: () => Object.keys(nodes),
  fps: ms => new Promise(res => { const f0 = frames, t0 = performance.now(); const spin = () => { ctl.autoRotate = true; ctl.autoRotateSpeed = 6; ctl.update(); if (performance.now() - t0 < (ms || 3000)) requestAnimationFrame(spin); else { ctl.autoRotate = false; res((frames - f0) / ((performance.now() - t0) / 1000)); } }; spin(); }),
};
