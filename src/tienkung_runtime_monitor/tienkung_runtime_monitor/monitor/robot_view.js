/* Local, read-only URDF renderer. No remote libraries or inferred joint measurements.
 * Matrices are column-major; URDF origin = T(xyz) Rz(yaw) Ry(pitch) Rx(roll).
 * GPU geometry is 16 bytes/vertex (float xyz + normalized signed-byte normal).
 */
const MAX_TRIANGLES = 2_000_000;
const MAX_STL_BYTES = 96 * 1024 * 1024;
const MAX_PARTS = 2048;
const meshCache = new Map();
let cachedTriangles = 0;
let downloadQueue = Promise.resolve();
const identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const finite = value => typeof value === 'number' && Number.isFinite(value);
function vector(value, fallback) {
  if (value === undefined) return fallback.slice();
  if (!Array.isArray(value) || value.length !== 3 || !value.every(finite)) {
    throw new Error('Expected a finite xyz vector');
  }
  return value;
}
export function multiply(a, b) {
  const out = new Array(16).fill(0);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    for (let k = 0; k < 4; k++) out[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k];
  }
  return out;
}
export function transformPoint(m, p) {
  return [0, 1, 2].map(r => m[r] * p[0] + m[4 + r] * p[1] + m[8 + r] * p[2] + m[12 + r]);
}
function rotation(axis, angle) {
  const length = Math.hypot(...axis);
  if (!(length > 0)) throw new Error('Joint axis has zero length');
  const [x, y, z] = axis.map(v => v / length);
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [t*x*x+c, t*x*y+s*z, t*x*z-s*y, 0,
    t*x*y-s*z, t*y*y+c, t*y*z+s*x, 0,
    t*x*z+s*y, t*y*z-s*x, t*z*z+c, 0, 0, 0, 0, 1];
}
export function originMatrix(xyz, rpy) {
  const p = vector(xyz, [0, 0, 0]), [r, pitch, y] = vector(rpy, [0, 0, 0]);
  const m = multiply(multiply(rotation([0, 0, 1], y), rotation([0, 1, 0], pitch)), rotation([1, 0, 0], r));
  m[12] = p[0]; m[13] = p[1]; m[14] = p[2];
  return m;
}

function compileModel(model) {
  if (!model || !Array.isArray(model.links) || !Array.isArray(model.joints) ||
      model.links.length > MAX_PARTS || model.joints.length > MAX_PARTS) {
    throw new Error('Invalid or oversized URDF model');
  }
  const links = new Set(model.links), names = new Map(), children = new Map(), parents = new Set();
  if (links.size !== model.links.length || [...links].some(n => typeof n !== 'string')) throw new Error('Invalid link names');
  const joints = model.joints.map(j => {
    if (!j || typeof j.name !== 'string' || names.has(j.name) || !links.has(j.parent) ||
        !links.has(j.child) || parents.has(j.child)) throw new Error('Invalid URDF joint graph');
    if (!['fixed', 'revolute', 'continuous', 'prismatic'].includes(j.type)) throw new Error(`Unsupported joint type: ${j.type}`);
    const axis = vector(j.axis, [1, 0, 0]);
    if (j.type !== 'fixed' && Math.hypot(...axis) === 0) throw new Error(`Zero axis: ${j.name}`);
    const item = {...j, axis, origin: originMatrix(j.origin_xyz, j.origin_rpy)};
    if (j.mimic && (typeof j.mimic.joint !== 'string' ||
        !finite(j.mimic.multiplier ?? 1) || !finite(j.mimic.offset ?? 0))) throw new Error(`Invalid mimic: ${j.name}`);
    names.set(j.name, item); parents.add(j.child);
    if (!children.has(j.parent)) children.set(j.parent, []);
    children.get(j.parent).push(item);
    return item;
  });
  const roots = [...links].filter(link => !parents.has(link));
  const ordered = [], visited = new Set(roots), queue = roots.slice();
  for (let i = 0; i < queue.length; i++) for (const joint of children.get(queue[i]) || []) {
    if (visited.has(joint.child)) throw new Error('Cyclic URDF links');
    visited.add(joint.child); queue.push(joint.child); ordered.push(joint);
  }
  if (visited.size !== links.size) throw new Error('Disconnected cycle in URDF links');
  return {joints, names, roots, ordered};
}
function pose(compiled, values, neutral = false) {
  const transforms = new Map(compiled.roots.map(n => [n, identity()]));
  const unknownLinks = new Set(), missing = new Set(), resolved = new Map(), resolving = new Set();
  function valueOf(name) {
    if (resolved.has(name)) return resolved.get(name);
    const j = compiled.names.get(name);
    if (!j || resolving.has(name)) { missing.add(name); return null; }
    resolving.add(name);
    let value = 0;
    if (j.type !== 'fixed' && !neutral) {
      if (j.mimic) {
        const source = valueOf(j.mimic.joint);
        value = source === null ? null : source * (j.mimic.multiplier ?? 1) + (j.mimic.offset ?? 0);
      } else value = Object.hasOwn(values, name) && finite(values[name]) ? values[name] : null;
      if (!finite(value)) { value = null; missing.add(name); }
    } else if (neutral && j.mimic) {
      const source = valueOf(j.mimic.joint);
      value = source === null ? null : source * (j.mimic.multiplier ?? 1) + (j.mimic.offset ?? 0);
    }
    resolving.delete(name); resolved.set(name, value);
    return value;
  }
  for (const j of compiled.ordered) {
    const measured = valueOf(j.name), value = measured ?? 0;
    let motion = identity();
    if (j.type === 'revolute' || j.type === 'continuous') motion = rotation(j.axis, value);
    if (j.type === 'prismatic') {
      const length = Math.hypot(...j.axis);
      for (let k = 0; k < 3; k++) motion[12 + k] = j.axis[k] / length * value;
    }
    transforms.set(j.child, multiply(multiply(transforms.get(j.parent), j.origin), motion));
    if (measured === null || unknownLinks.has(j.parent)) unknownLinks.add(j.child);
  }
  return {transforms, unknownLinks, missingJoints: [...missing].sort()};
}
/** Missing/non-finite measurements use contextual zero ONLY and taint descendants. */
export function forwardKinematics(model, jointValues = {}) {
  return pose(compileModel(model), jointValues || {});
}

function geometryBuffer(triangles) {
  if (!Number.isInteger(triangles) || triangles < 1 || triangles > MAX_TRIANGLES) throw new Error('STL triangle limit exceeded or empty mesh');
  const data = new ArrayBuffer(triangles * 3 * 16);
  return {data, floats: new Float32Array(data), bytes: new Int8Array(data), triangles,
    bounds: {min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity]}};
}
function putTriangle(mesh, index, points) {
  if (points.some(p => p.some(v => !finite(v) || Math.abs(v) > 1e12))) throw new Error('Non-finite or excessive STL coordinate');
  const a = points[0], b = points[1], c = points[2];
  const u = b.map((v, i) => v - a[i]), v = c.map((n, i) => n - a[i]);
  const n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]];
  const length = Math.hypot(...n) || 1;
  for (let k = 0; k < 3; k++) for (let d = 0; d < 3; d++) {
    mesh.floats[(index * 3 + k) * 4 + d] = points[k][d];
    mesh.bytes[(index * 3 + k) * 16 + 12 + d] = Math.round(n[d] / length * 127);
    mesh.bounds.min[d] = Math.min(mesh.bounds.min[d], points[k][d]);
    mesh.bounds.max[d] = Math.max(mesh.bounds.max[d], points[k][d]);
  }
}
/** Strict binary/ASCII STL parsing, including binary headers beginning with solid. */
export function parseSTL(input, triangleLimit = MAX_TRIANGLES) {
  const bytes = input instanceof ArrayBuffer ? new Uint8Array(input) :
    ArrayBuffer.isView(input) ? new Uint8Array(input.buffer, input.byteOffset, input.byteLength) : null;
  if (!bytes || bytes.byteLength > MAX_STL_BYTES) throw new Error('STL byte limit exceeded or invalid input');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const count = bytes.length >= 84 ? view.getUint32(80, true) : 0;
  if (count > 0 && 84 + count * 50 === bytes.length) {
    if (count > triangleLimit) throw new Error('STL triangle limit exceeded');
    const mesh = geometryBuffer(count);
    for (let i = 0; i < count; i++) {
      const points = [0, 1, 2].map(k => [0, 1, 2].map(d => view.getFloat32(84 + i * 50 + 12 + k * 12 + d * 4, true)));
      putTriangle(mesh, i, points);
    }
    return mesh;
  }
  const text = new TextDecoder('utf-8', {fatal: true}).decode(bytes);
  if (!/^\s*solid(?:\s|$)/i.test(text) || !/endsolid(?:\s|$)/i.test(text)) throw new Error('Invalid or truncated STL');
  // Two passes avoid building a second giant JS array for ASCII meshes.
  const number = '[+-]?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][+-]?\\d+)?';
  const facet = new RegExp(`facet\\s+normal\\s+${number}\\s+${number}\\s+${number}\\s+outer\\s+loop\\s+` +
    Array(3).fill(`vertex\\s+(${number})\\s+(${number})\\s+(${number})\\s+`).join('') + 'endloop\\s+endfacet', 'gi');
  let match, triangles = 0;
  while ((match = facet.exec(text))) {
    if (++triangles > Math.min(triangleLimit, MAX_TRIANGLES)) throw new Error('STL triangle limit exceeded');
  }
  if ((text.match(/\bfacet\s+normal\b/gi) || []).length !== triangles ||
      (text.match(/\bvertex\b/gi) || []).length !== triangles * 3) throw new Error('Malformed ASCII STL facets');
  const mesh = geometryBuffer(triangles);
  facet.lastIndex = 0;
  let index = 0;
  while ((match = facet.exec(text))) putTriangle(mesh, index++, [0, 1, 2].map(k => [0, 1, 2].map(d => Number(match[1 + k * 3 + d]))));
  return mesh;
}

async function boundedFetch(url) {
  // Never allow a URDF to introduce a CDN, file URL, or arbitrary network target.
  if (typeof url !== 'string' || !/^\/meshes\/\d+$/.test(url)) throw new Error('Mesh URL must be a local /meshes/N route');
  const response = await fetch(url, {credentials: 'same-origin'});
  if (!response.ok) throw new Error(`Mesh HTTP ${response.status}: ${url}`);
  if (Number(response.headers.get('content-length')) > MAX_STL_BYTES) throw new Error('STL download exceeds byte limit');
  if (!response.body?.getReader) {
    const data = await response.arrayBuffer();
    if (data.byteLength > MAX_STL_BYTES) throw new Error('STL download exceeds byte limit');
    return data;
  }
  const reader = response.body.getReader(), chunks = [];
  let length = 0;
  try {
    while (true) {
      const next = await reader.read();
      if (next.done) break;
      length += next.value.byteLength;
      if (length > MAX_STL_BYTES) throw new Error('STL download exceeds byte limit');
      chunks.push(next.value);
    }
  } catch (error) { await reader.cancel().catch(() => {}); throw error; }
  finally { reader.releaseLock(); }
  const data = new Uint8Array(length);
  let offset = 0;
  for (const chunk of chunks) { data.set(chunk, offset); offset += chunk.byteLength; }
  return data.buffer;
}
function loadMesh(url) {
  if (meshCache.has(url)) {
    const entry = meshCache.get(url);
    meshCache.delete(url); meshCache.set(url, entry);
    return entry.promise;
  }
  const entry = {triangles: 0, promise: null};
  // Serial download/parse bounds transient memory and coalesces requests from all views.
  entry.promise = downloadQueue.then(async () => {
    const mesh = parseSTL(await boundedFetch(url));
    for (const [key, old] of meshCache) {
      if (cachedTriangles + mesh.triangles <= MAX_TRIANGLES && meshCache.size <= 128) break;
      if (key !== url && old.triangles) { cachedTriangles -= old.triangles; meshCache.delete(key); }
    }
    entry.triangles = mesh.triangles; cachedTriangles += mesh.triangles;
    return mesh;
  }).catch(error => { meshCache.delete(url); throw error; });
  downloadQueue = entry.promise.then(() => undefined, () => undefined);
  meshCache.set(url, entry);
  return entry.promise;
}

function primitive(geometry) {
  const triangles = [];
  const add = (a, b, c) => triangles.push([a, b, c]);
  if (geometry.type === 'box') {
    const size = vector(geometry.size, [1, 1, 1]);
    if (size.some(v => v <= 0)) throw new Error('Invalid box size');
    const p = Array.from({length: 8}, (_, i) => size.map((v, d) => (i & (1 << d) ? 1 : -1) * v / 2));
    for (const [a, b, c, d] of [[0,2,3,1],[4,5,7,6],[0,1,5,4],[2,6,7,3],[0,4,6,2],[1,3,7,5]]) {
      add(p[a],p[b],p[c]); add(p[a],p[c],p[d]);
    }
  } else if (geometry.type === 'sphere' || geometry.type === 'cylinder') {
    const r = geometry.radius, length = geometry.length;
    if (!finite(r) || r <= 0 || (geometry.type === 'cylinder' && (!finite(length) || length <= 0))) throw new Error('Invalid primitive dimensions');
    const point = (a, z) => [r * Math.cos(a), r * Math.sin(a), z];
    for (let i = 0; i < 32; i++) {
      const a = i * Math.PI / 16, b = (i + 1) * Math.PI / 16;
      if (geometry.type === 'cylinder') {
        const p = point(a, -length / 2), q = point(b, -length / 2), s = point(a, length / 2), t = point(b, length / 2);
        add(p,q,t); add(p,t,s); add([0,0,-length/2],q,p); add([0,0,length/2],s,t);
      } else for (let j = 0; j < 16; j++) {
        const sphere = (angle, step) => {
          const latitude = -Math.PI / 2 + step * Math.PI / 16;
          return [r*Math.cos(latitude)*Math.cos(angle),r*Math.cos(latitude)*Math.sin(angle),r*Math.sin(latitude)];
        };
        add(sphere(a,j),sphere(b,j),sphere(b,j+1)); add(sphere(a,j),sphere(b,j+1),sphere(a,j+1));
      }
    }
  } else throw new Error(`Unsupported visual geometry: ${geometry.type || 'unknown'}`);
  const mesh = geometryBuffer(triangles.length);
  triangles.forEach((t, i) => putTriangle(mesh, i, t));
  return mesh;
}
function visualTransform(visual) {
  const m = originMatrix(visual.origin_xyz, visual.origin_rpy);
  const scale = vector(visual.geometry?.scale, [1, 1, 1]);
  if (scale.some(v => v === 0)) throw new Error('Zero visual scale');
  for (let c = 0; c < 3; c++) for (let r = 0; r < 3; r++) m[c * 4 + r] *= scale[c];
  return m;
}
function framing(compiled, visuals) {
  const neutral = pose(compiled, {}, true), min = [Infinity,Infinity,Infinity], max = [-Infinity,-Infinity,-Infinity];
  const include = p => p.forEach((v, i) => { min[i] = Math.min(min[i], v); max[i] = Math.max(max[i], v); });
  for (const m of neutral.transforms.values()) include(transformPoint(m, [0,0,0]));
  for (const v of visuals) {
    const m = multiply(neutral.transforms.get(v.link), v.local);
    for (let k = 0; k < 8; k++) include(transformPoint(m, [0,1,2].map(d => v.mesh.bounds[k & (1 << d) ? 'max' : 'min'][d])));
  }
  if (!finite(min[0])) return {center: [0,0,0], radius: 1, floorZ: -1};
  return {center: min.map((v, i) => (v + max[i]) / 2), floorZ: min[2],
    radius: Math.max(0.1, Math.hypot(...max.map((v, i) => v - min[i])) / 2)};
}
const normalize = v => { const n = Math.hypot(...v); return v.map(x => x / n); };
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
export function defaultCamera(camera = 'oblique') {
  return {yaw: camera === 'front' ? 0 : camera === 'side' ? -Math.PI/2 : Math.atan2(-2,1.5),
    pitch: camera === 'front' || camera === 'side' ? 0 : Math.atan2(.8,2.5),
    zoom: 1, pan: [0,0,0]};
}
function cameraAxes(camera) {
  const forward = [Math.cos(camera.pitch)*Math.cos(camera.yaw), Math.cos(camera.pitch)*Math.sin(camera.yaw), Math.sin(camera.pitch)];
  const right = normalize(cross([0,0,1],forward));
  return {forward,right,up:cross(forward,right)};
}
export function cameraMatrix(frame, camera, aspect) {
  // Orthographic REP-103 view; orbit/pan never changes robot or reference data.
  const {forward,right,up} = cameraAxes(camera);
  const halfHeight = frame.radius * 1.12 / Math.min(1, aspect) / camera.zoom, halfWidth = halfHeight * aspect;
  const m = identity(), axes = [right, up, forward], scales = [1/halfWidth,1/halfHeight,-1/(frame.radius*12)];
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) m[c*4+r] = axes[r][c] * scales[r];
    m[12+r] = -axes[r].reduce((sum, v, i) => sum + v * (frame.center[i] + camera.pan[i]), 0) * scales[r];
  }
  return m;
}
// Neutral-pose bounds define a stable visual floor, NOT an estimated contact plane.
export function floorTiles(frame) {
  const tiles = [], cell = frame.radius / 3, z = frame.floorZ ?? -frame.radius;
  for (let x=-6;x<6;x++) for (let y=-6;y<6;y++) {
    const x0=frame.center[0]+x*cell, y0=frame.center[1]+y*cell;
    tiles.push({shade:(x+y+12)%2, points:[[x0,y0,z],[x0+cell,y0,z],[x0+cell,y0+cell,z],[x0,y0+cell,z]]});
  }
  return tiles;
}
function normalMatrix(m) {
  // Inverse transpose for URDF mesh nonuniform/negative scale.
  const a = [m[0],m[1],m[2]], b = [m[4],m[5],m[6]], c = [m[8],m[9],m[10]];
  const x = cross(b,c), y = cross(c,a), z = cross(a,b), det = a.reduce((s,v,i) => s+v*x[i], 0);
  return [...x,...y,...z].map(v => v / det);
}
function rgb(hex) {
  if (typeof hex !== 'string' || !/^#[\da-f]{6}$/i.test(hex)) throw new Error('Color must be #rrggbb');
  return [1,3,5].map(i => parseInt(hex.slice(i,i+2),16)/255);
}
function makeProgram(gl) {
  const shaders = [];
  let program;
  try {
    for (const [type, source] of [[gl.VERTEX_SHADER, `attribute vec3 aPosition; attribute vec3 aNormal;
      uniform mat4 uMvp; uniform mat3 uNormal; varying vec3 vNormal;
      void main(){gl_Position=uMvp*vec4(aPosition,1.0);vNormal=uNormal*aNormal;gl_PointSize=5.0;}`],
    [gl.FRAGMENT_SHADER, `precision mediump float; uniform vec3 uColor; uniform float uLit; varying vec3 vNormal;
      void main(){float light=0.42+0.58*abs(dot(normalize(vNormal+vec3(0.00001)),normalize(vec3(0.4,-0.6,0.9))));
      gl_FragColor=vec4(uColor*mix(1.0,light,uLit),1.0);}`]]) {
      const shader = gl.createShader(type); shaders.push(shader); gl.shaderSource(shader,source); gl.compileShader(shader);
      if (!gl.getShaderParameter(shader,gl.COMPILE_STATUS)) throw new Error('WebGL shader compilation failed');
    }
    program = gl.createProgram(); shaders.forEach(s => gl.attachShader(program,s)); gl.linkProgram(program);
    if (!gl.getProgramParameter(program,gl.LINK_STATUS)) throw new Error('WebGL shader linking failed');
    return {program, position: gl.getAttribLocation(program,'aPosition'), normal: gl.getAttribLocation(program,'aNormal'),
      mvp: gl.getUniformLocation(program,'uMvp'), normals: gl.getUniformLocation(program,'uNormal'),
      color: gl.getUniformLocation(program,'uColor'), lit: gl.getUniformLocation(program,'uLit')};
  } catch (error) { if (program) gl.deleteProgram(program); throw error; }
  finally { shaders.forEach(s => gl.deleteShader(s)); }
}

export class RobotView {
  constructor(canvas, color = '#3987e5') {
    this.canvas = canvas; this.setColor(color);
    this.status = {loading:false,ready:false,warnings:[],missingJoints:[],triangleCount:0,fallback:true};
    this.visuals = []; this.buffers = new Map(); this.generation = 0; this.disposed = false;
    this.frame = {center:[0,0,0],radius:1}; this.baseWarnings = []; this.modelWarnings = [];
    this.last = [{},'oblique','light'];
    this.camera = defaultCamera(); this.cameraPreset = 'oblique';
    this.interactionListeners = []; this.drag = null; this.pendingRender = null;
    this.oldTouchAction = canvas.style.touchAction; this.oldCursor = canvas.style.cursor;
    this.oldTabIndex = canvas.tabIndex;
    canvas.style.touchAction = 'none'; canvas.style.cursor = 'grab'; canvas.tabIndex = 0;
    try {
      this.gl = canvas.getContext('webgl', {alpha:false,antialias:true,depth:true,preserveDrawingBuffer:false});
      if (!this.gl) throw new Error('WebGL unavailable — skeleton only, not the exact robot surface.');
      this.program = makeProgram(this.gl); this.lineBuffer = this.gl.createBuffer();
    } catch (error) {
      this.baseWarnings.push(error.message);
      if (!this.gl) this.context2d = canvas.getContext('2d');
    }
    const document = canvas.ownerDocument;
    if (document && canvas.parentNode) {
      this.notice = document.createElement('div'); this.notice.className = 'robot-view-status';
      this.notice.setAttribute('role','status'); this.notice.style.cssText = 'font-size:11px;line-height:1.4;padding:4px 8px;overflow-wrap:anywhere';
      canvas.insertAdjacentElement('afterend',this.notice);
    }
    this.onLost = event => {
      event.preventDefault(); this.contextLost = true;
      this.baseWarnings.push('WebGL context lost — surface rendering unavailable until restored.');
      this.updateStatus();
    };
    this.onRestored = () => {
      this.contextLost = false; this.buffers.clear();
      this.baseWarnings = this.baseWarnings.filter(w => !w.startsWith('WebGL context lost'));
      try { this.program = makeProgram(this.gl); this.lineBuffer = this.gl.createBuffer(); this.upload(); }
      catch (error) { this.baseWarnings.push(error.message); this.program = null; }
      this.render(...this.last);
    };
    canvas.addEventListener?.('webglcontextlost',this.onLost);
    canvas.addEventListener?.('webglcontextrestored',this.onRestored);
    this.installInteraction();
    this.updateStatus();
  }
  requestRender() {
    if (this.disposed || this.pendingRender !== null) return;
    if (!globalThis.requestAnimationFrame) { this.render(...this.last); return; }
    this.pendingRender = requestAnimationFrame(() => {
      this.pendingRender = null; if (!this.disposed) this.render(...this.last);
    });
  }
  resetCamera(preset = this.cameraPreset) {
    this.cameraPreset = preset; this.camera = defaultCamera(preset); this.last[1] = preset; this.requestRender();
  }
  orbit(dx, dy) {
    this.camera.yaw = (this.camera.yaw - dx*.008) % (2*Math.PI);
    this.camera.pitch = clamp(this.camera.pitch + dy*.008, -Math.PI/2+.03, Math.PI/2-.03);
    this.requestRender();
  }
  pan(dx, dy) {
    const {right,up} = cameraAxes(this.camera);
    const w = this.canvas.clientWidth || 320, h = this.canvas.clientHeight || 300;
    const scale = 2*this.frame.radius*1.12/Math.min(1,w/h)/this.camera.zoom/h;
    this.camera.pan = this.camera.pan.map((v,i) => clamp(v + (-dx*right[i]+dy*up[i])*scale,-10*this.frame.radius,10*this.frame.radius));
    this.requestRender();
  }
  zoomBy(factor) {
    this.camera.zoom = clamp(this.camera.zoom*factor,.2,8); this.requestRender();
  }
  installInteraction() {
    const listen = (type,handler,options) => {
      this.canvas.addEventListener?.(type,handler,options);
      this.interactionListeners.push([type,handler,options]);
    };
    listen('pointerdown',event => {
      if (this.drag || ![0,1,2].includes(event.button)) return;
      event.preventDefault(); this.canvas.focus?.();
      this.drag = {id:event.pointerId,x:event.clientX,y:event.clientY,pan:event.button!==0||event.shiftKey};
      this.canvas.setPointerCapture?.(event.pointerId); this.canvas.style.cursor='grabbing';
    });
    listen('pointermove',event => {
      if (!this.drag || this.drag.id!==event.pointerId) return;
      const dx=event.clientX-this.drag.x,dy=event.clientY-this.drag.y;
      this.drag.x=event.clientX;this.drag.y=event.clientY;
      if(this.drag.pan||event.shiftKey)this.pan(dx,dy);else this.orbit(dx,dy);
    });
    const end = event => {
      if(this.drag?.id!==event.pointerId)return;
      this.drag=null;this.canvas.style.cursor='grab';
      if(this.canvas.hasPointerCapture?.(event.pointerId))this.canvas.releasePointerCapture(event.pointerId);
    };
    listen('pointerup',end);listen('pointercancel',end);listen('lostpointercapture',end);
    listen('contextmenu',event=>event.preventDefault());
    listen('wheel',event=>{
      event.preventDefault();
      const pixels=event.deltaY*(event.deltaMode===1?16:event.deltaMode===2?(this.canvas.clientHeight||300):1);
      this.zoomBy(Math.exp(-clamp(pixels,-500,500)*.0015));
    },{passive:false});
    listen('dblclick',event=>{event.preventDefault();this.resetCamera();});
    listen('keydown',event=>{
      const directions={ArrowLeft:[-12,0],ArrowRight:[12,0],ArrowUp:[0,-12],ArrowDown:[0,12]};
      if(directions[event.key]){event.preventDefault();const [x,y]=directions[event.key];if(event.shiftKey)this.pan(x,y);else this.orbit(x,y);}
      else if(['+','=','-','Home'].includes(event.key)){event.preventDefault();if(event.key==='Home')this.resetCamera();else this.zoomBy(event.key==='-'?1/1.1:1.1);}
    });
  }
  setColor(hex) { this.color = rgb(hex); this.colorHex = hex; }
  updateStatus() {
    this.status.warnings = [...new Set([...this.baseWarnings,...this.modelWarnings])];
    this.status.fallback = !this.program || !!this.contextLost || this.modelWarnings.length > 0 || !this.visuals.length;
    const messages = this.status.loading ? ['Loading local robot meshes…'] : [...this.status.warnings];
    if (this.status.missingJoints.length) messages.push(`Unmeasured joints shown gray in contextual zero pose (not measurements): ${this.status.missingJoints.join(', ')}`);
    const text = messages.join(' ');
    if (this.notice) { this.notice.textContent = text; this.notice.hidden = !text; }
    this.canvas.title = (text ? text + ' ' : '') + '拖动旋转；Shift/右键拖动平移；滚轮缩放；双击/Home 复位。方向键旋转、Shift+方向键平移、+/-缩放。地板仅为固定视觉参考。';
    this.canvas.setAttribute?.('aria-label',this.canvas.title);
  }
  clearBuffers() {
    if (this.gl) for (const buffer of this.buffers.values()) this.gl.deleteBuffer(buffer);
    this.buffers.clear();
  }
  upload() {
    if (!this.program || this.contextLost) return;
    const gl = this.gl;
    for (const visual of this.visuals) if (!this.buffers.has(visual.mesh)) {
      const buffer = gl.createBuffer();
      if (!buffer) throw new Error('GPU mesh allocation failed; skeleton fallback.');
      gl.bindBuffer(gl.ARRAY_BUFFER,buffer); gl.bufferData(gl.ARRAY_BUFFER,visual.mesh.data,gl.STATIC_DRAW);
      if (gl.getError() !== gl.NO_ERROR) { gl.deleteBuffer(buffer); throw new Error('GPU mesh upload failed; skeleton fallback.'); }
      this.buffers.set(visual.mesh,buffer);
    }
  }
  async setModel(urdf) {
    if (this.disposed) return;
    const generation = ++this.generation;
    this.clearBuffers(); this.visuals = []; this.compiled = null; this.modelWarnings = [];
    this.status = {loading:true,ready:false,warnings:[],missingJoints:[],triangleCount:0,fallback:true};
    this.updateStatus();
    try {
      const compiled = compileModel(urdf);
      this.compiled = compiled;
      if (!Array.isArray(urdf.visuals) || urdf.visuals.length > MAX_PARTS) throw new Error('Missing or oversized URDF visuals; skeleton only.');
      const visuals = [], warnings = [];
      let triangleCount = 0;
      for (const visual of urdf.visuals) {
        if (generation !== this.generation || this.disposed) return;
        try {
          if (!urdf.links.includes(visual.link)) throw new Error('Visual refers to an unknown link');
          const local = visualTransform(visual);
          const mesh = visual.geometry?.type === 'mesh' ? await loadMesh(visual.geometry.url) : primitive(visual.geometry || {});
          if (triangleCount + mesh.triangles > MAX_TRIANGLES) throw new Error('Model exceeds 2,000,000 rendered triangles');
          triangleCount += mesh.triangles;
          visuals.push({link:visual.link,local,mesh});
        } catch (error) { warnings.push(`${visual?.link || 'Visual'}: ${error.message}`); }
      }
      if (generation !== this.generation || this.disposed) return;
      this.visuals = visuals; this.modelWarnings = warnings;
      if (!visuals.length) this.modelWarnings.push('No renderable visual meshes — skeleton only, not an exact surface model.');
      else if (warnings.length) this.modelWarnings.push('Incomplete surface model — skeleton overlaid for context.');
      this.frame = framing(compiled, visuals);
      this.status.triangleCount = triangleCount; this.status.ready = true;
      try { this.upload(); } catch (error) { this.clearBuffers(); this.modelWarnings.push(error.message); }
    } catch (error) {
      if (generation !== this.generation || this.disposed) return;
      this.modelWarnings.push(error.message);
      if (this.compiled) this.frame = framing(this.compiled,[]);
    } finally {
      if (generation === this.generation && !this.disposed) {
        this.status.loading = false; this.updateStatus(); this.render(...this.last);
      }
    }
  }
  render(jointValues = {}, camera = 'oblique', theme = 'light') {
    if (this.disposed) return;
    this.last = [jointValues || {},camera,theme];
    const dark = theme === 'dark', background = dark ? '#1a1a19' : '#fcfcfb';
    this.canvas.style.backgroundColor = background;
    if (this.notice) this.notice.style.color = dark ? '#c2c0b6' : '#66645e';
    const ratio = Math.min(globalThis.devicePixelRatio || 1,2);
    const width = Math.max(1,Math.min(4096,Math.round((this.canvas.clientWidth || 320)*ratio)));
    const height = Math.max(1,Math.min(4096,Math.round((this.canvas.clientHeight || 300)*ratio)));
    if (this.canvas.width !== width) this.canvas.width = width;
    if (this.canvas.height !== height) this.canvas.height = height;
    const state = this.compiled ? pose(this.compiled,jointValues || {}) : null;
    this.status.missingJoints = state?.missingJoints || []; this.updateStatus();
    if (camera !== this.cameraPreset) { this.cameraPreset = camera; this.camera = defaultCamera(camera); }
    const vp = cameraMatrix(this.frame,this.camera,width/height);
    const tiles = state ? floorTiles(this.frame) : [];
    const floorColors = dark ? ['#292928','#383835'] : ['#e1e0d9','#c3c2b7'];
    const gray = dark ? [0.47,0.47,0.45] : [0.56,0.56,0.54];
    if (this.gl && this.program && !this.contextLost) {
      const gl = this.gl, p = this.program;
      gl.viewport(0,0,width,height); gl.clearColor(...rgb(background),1);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); gl.enable(gl.DEPTH_TEST); gl.disable(gl.CULL_FACE);
      gl.useProgram(p.program); gl.uniform1f(p.lit,0);
      gl.disableVertexAttribArray(p.normal); gl.vertexAttrib3f(p.normal,0,0,1);
      gl.uniformMatrix4fv(p.mvp,false,new Float32Array(vp));
      gl.uniformMatrix3fv(p.normals,false,new Float32Array([1,0,0,0,1,0,0,0,1]));
      gl.bindBuffer(gl.ARRAY_BUFFER,this.lineBuffer); gl.enableVertexAttribArray(p.position);
      gl.vertexAttribPointer(p.position,3,gl.FLOAT,false,0,0);
      for (const shade of [0,1]) {
        const vertices=[];
        for (const tile of tiles) if(tile.shade===shade) {
          const [a,b,c,d]=tile.points;vertices.push(...a,...b,...c,...a,...c,...d);
        }
        gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(vertices),gl.DYNAMIC_DRAW);
        gl.uniform3fv(p.color,rgb(floorColors[shade]));gl.drawArrays(gl.TRIANGLES,0,vertices.length/3);
      }
      gl.uniform1f(p.lit,1);
      if (state) for (const visual of this.visuals) {
        const buffer = this.buffers.get(visual.mesh);
        if (!buffer) continue;
        const model = multiply(state.transforms.get(visual.link),visual.local);
        gl.bindBuffer(gl.ARRAY_BUFFER,buffer); gl.enableVertexAttribArray(p.position); gl.enableVertexAttribArray(p.normal);
        gl.vertexAttribPointer(p.position,3,gl.FLOAT,false,16,0); gl.vertexAttribPointer(p.normal,3,gl.BYTE,true,16,12);
        gl.uniformMatrix4fv(p.mvp,false,new Float32Array(multiply(vp,model)));
        gl.uniformMatrix3fv(p.normals,false,new Float32Array(normalMatrix(model)));
        gl.uniform3fv(p.color,state.unknownLinks.has(visual.link) ? gray : this.color);
        gl.drawArrays(gl.TRIANGLES,0,visual.mesh.triangles*3);
      }
      if (state && this.status.fallback) {
        gl.disable(gl.DEPTH_TEST); gl.uniform1f(p.lit,0);
        gl.disableVertexAttribArray(p.normal); gl.vertexAttrib3f(p.normal,0,0,1);
        gl.uniformMatrix4fv(p.mvp,false,new Float32Array(vp));
        gl.uniformMatrix3fv(p.normals,false,new Float32Array([1,0,0,0,1,0,0,0,1]));
        gl.bindBuffer(gl.ARRAY_BUFFER,this.lineBuffer); gl.enableVertexAttribArray(p.position);
        gl.vertexAttribPointer(p.position,3,gl.FLOAT,false,0,0);
        for (const unknown of [false,true]) {
          const vertices = [];
          for (const j of this.compiled.ordered) if (state.unknownLinks.has(j.child) === unknown) {
            vertices.push(...transformPoint(state.transforms.get(j.parent),[0,0,0]),...transformPoint(state.transforms.get(j.child),[0,0,0]));
          }
          gl.bufferData(gl.ARRAY_BUFFER,new Float32Array(vertices),gl.DYNAMIC_DRAW);
          gl.uniform3fv(p.color,unknown ? gray : this.color);
          gl.drawArrays(gl.LINES,0,vertices.length/3); gl.drawArrays(gl.POINTS,0,vertices.length/3);
        }
      }
    } else if (this.context2d) {
      const ctx = this.context2d;
      ctx.fillStyle = background; ctx.fillRect(0,0,width,height);
      for (const tile of tiles) {
        ctx.fillStyle=floorColors[tile.shade];ctx.beginPath();
        tile.points.forEach((point,i)=>{const p=transformPoint(vp,point);const x=(p[0]+1)*width/2,y=(1-p[1])*height/2;if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});
        ctx.closePath();ctx.fill();
      }
      if (state) {
        const project = m => { const p = transformPoint(vp,transformPoint(m,[0,0,0])); return [(p[0]+1)*width/2,(1-p[1])*height/2]; };
        ctx.lineWidth = 2*ratio;
        for (const j of this.compiled.ordered) {
          const a = project(state.transforms.get(j.parent)), b = project(state.transforms.get(j.child));
          ctx.strokeStyle = state.unknownLinks.has(j.child) ? (dark ? '#787873' : '#8f8f8a') : this.colorHex;
          ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
          ctx.fillStyle = ctx.strokeStyle; ctx.beginPath(); ctx.arc(...b,3*ratio,0,Math.PI*2); ctx.fill();
        }
      }
      ctx.fillStyle = dark ? '#c2c0b6' : '#66645e'; ctx.font = `${11*ratio}px sans-serif`;
      ctx.fillText('Skeleton only — WebGL surface unavailable',8*ratio,18*ratio);
    }
  }
  dispose() {
    if (this.disposed) return;
    this.disposed = true; ++this.generation; this.clearBuffers();
    if(this.pendingRender!==null)globalThis.cancelAnimationFrame?.(this.pendingRender);
    this.pendingRender=null;
    for(const [type,handler,options] of this.interactionListeners)this.canvas.removeEventListener?.(type,handler,options);
    this.interactionListeners=[];
    if(this.drag && this.canvas.hasPointerCapture?.(this.drag.id))this.canvas.releasePointerCapture(this.drag.id);
    this.drag=null;this.canvas.style.touchAction=this.oldTouchAction;this.canvas.style.cursor=this.oldCursor;
    this.canvas.tabIndex=this.oldTabIndex;
    if (this.gl) { if (this.program) this.gl.deleteProgram(this.program.program); if (this.lineBuffer) this.gl.deleteBuffer(this.lineBuffer); }
    this.canvas.removeEventListener?.('webglcontextlost',this.onLost);
    this.canvas.removeEventListener?.('webglcontextrestored',this.onRestored);
    this.notice?.remove(); this.visuals = []; this.compiled = null;
  }
}
