import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

// Browser .js module without changing the ROS package's module configuration.
const source = await readFile(new URL('../tienkung_runtime_monitor/monitor/robot_view.js', import.meta.url), 'utf8');
const {forwardKinematics, originMatrix, transformPoint, parseSTL, RobotView, defaultCamera, cameraMatrix, floorTiles} =
  await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const close = (a,b) => a.forEach((v,i) => assert.ok(Math.abs(v-b[i]) < 1e-6, `${a} != ${b}`));
const joint = (name,parent,child,extra={}) => ({name,parent,child,type:'revolute',axis:[0,0,1],...extra});
const model = joints => ({links:[...new Set(['root',...joints.flatMap(j => [j.parent,j.child])])],joints,root_links:['root'],visuals:[]});
const point = (fk,link,p=[0,0,0]) => transformPoint(fk.transforms.get(link),p);
const ascii = `solid example
facet normal 0 0 1
 outer loop
  vertex 0 0 0
  vertex 1 0 0
  vertex 0 1 0
 endloop
endfacet
endsolid example`;
const bytes = text => new TextEncoder().encode(text);
function binary() {
  const buffer = new ArrayBuffer(134), view = new DataView(buffer);
  new Uint8Array(buffer).set(bytes('solid binary STL'));
  view.setUint32(80,1,true);
  [0,0,0,1,0,0,0,1,0].forEach((v,i) => view.setFloat32(96+i*4,v,true));
  return buffer;
}
function canvas() {
  const calls = [];
  const ctx = {fillRect(...args){calls.push(['fillRect',this.fillStyle,...args]);},beginPath(){},closePath(){},moveTo(){},lineTo(){},stroke(){},arc(){},fill(){},fillText(...args){calls.push(['text',...args]);}};
  return {calls,style:{},clientWidth:320,clientHeight:300,getContext(type){return type==='2d'?ctx:null;},setAttribute(){}};
}

test('root is identity and fixed joints need no measurements', () => {
  const fk = forwardKinematics(model([joint('fixed','root','child',{type:'fixed',origin_xyz:[1,2,3]})]),{});
  close(point(fk,'root'),[0,0,0]); close(point(fk,'child'),[1,2,3]);
  assert.deepEqual(fk.missingJoints,[]); assert.equal(fk.unknownLinks.size,0);
});
test('URDF origin rotation is Rz Ry Rx, translation is not rotated', () => {
  const m = originMatrix([1,2,3],[Math.PI/2,Math.PI/2,Math.PI/2]);
  close(transformPoint(m,[1,0,0]),[1,2,2]);
  close(transformPoint(m,[0,1,0]),[1,3,3]);
  close(transformPoint(m,[0,0,1]),[2,2,3]);
});
test('parent origin precedes joint rotation; arbitrary normalized axis uses Rodrigues', () => {
  const fk = forwardKinematics(model([
    joint('spin','root','a',{origin_xyz:[1,0,0],origin_rpy:[0,0,Math.PI/2],axis:[0,0,2]}),
    joint('tip','a','b',{type:'fixed',origin_xyz:[1,0,0]})]),{spin:Math.PI/2});
  close(point(fk,'b'),[0,0,0]);
  const arbitrary = forwardKinematics(model([joint('q','root','a',{axis:[1,1,1]})]),{q:Math.PI*2/3});
  close(point(arbitrary,'a',[1,0,0]),[0,1,0]);
});
test('continuous and prismatic axes operate in joint-origin frame', () => {
  const fk = forwardKinematics(model([
    joint('spin','root','a',{type:'continuous'}),
    joint('slide','a','b',{type:'prismatic',axis:[2,0,0]})]),{spin:Math.PI/2,slide:2});
  close(point(fk,'b'),[0,2,0]);
});
test('graph ordering need not be parent-first', () => {
  const fk = forwardKinematics(model([
    joint('tip','a','b',{type:'fixed',origin_xyz:[1,0,0]}),joint('spin','root','a')]),{spin:Math.PI/2});
  close(point(fk,'b'),[0,1,0]);
});
test('missing/null/NaN/infinite/string values taint descendants, not sibling or root', () => {
  const m = model([joint('q','root','a'),joint('fixed','a','b',{type:'fixed'}),joint('s','root','s')]);
  for (const q of [undefined,null,NaN,Infinity,'0']) {
    const fk = forwardKinematics(m,{q,s:0});
    assert.deepEqual(fk.missingJoints,['q']); assert.deepEqual([...fk.unknownLinks],['a','b']);
    close(point(fk,'a',[1,0,0]),[1,0,0]);
  }
  assert.equal(forwardKinematics(m,{q:0,s:0}).unknownLinks.size,0);
});
test('mimic resolves chains by source with multiplier/offset and ignores direct value', () => {
  const m = model([joint('m','root','m',{mimic:{joint:'q',multiplier:-2,offset:0.5}}),
    joint('q','root','q'),joint('chain','m','end',{mimic:{joint:'m'}})]);
  const fk = forwardKinematics(m,{q:0.25,m:7});
  close(point(fk,'m',[1,0,0]),[1,0,0]); assert.deepEqual(fk.missingJoints,[]);
  const missing = forwardKinematics(m,{});
  assert.deepEqual(missing.missingJoints,['chain','m','q']);
  assert.deepEqual([...missing.unknownLinks].sort(),['end','m','q']);
});
test('mimic cycles fail closed and malformed kinematic graphs reject', () => {
  const cyclicMimic = model([joint('a','root','a',{mimic:{joint:'b'}}),joint('b','root','b',{mimic:{joint:'a'}})]);
  assert.deepEqual(forwardKinematics(cyclicMimic,{}).missingJoints,['a','b']);
  assert.throws(() => forwardKinematics(model([joint('x','a','b'),joint('y','b','a')]),{}),/cycle/i);
  assert.throws(() => forwardKinematics(model([joint('q','root','a',{type:'floating'})]),{}),/Unsupported/);
  assert.throws(() => forwardKinematics(model([joint('q','root','a',{axis:[0,0,0]})]),{}),/Zero axis/);
});
test('binary STL with solid header and ASCII STL produce matching packed vertices/normals', () => {
  const a = parseSTL(binary()), b = parseSTL(bytes(ascii));
  assert.equal(a.triangles,1); assert.equal(a.data.byteLength,48);
  assert.deepEqual(a.bounds,{min:[0,0,0],max:[1,1,0]});
  assert.deepEqual(new Uint8Array(a.data),new Uint8Array(b.data));
  assert.equal(a.bytes[14],127);
});
test('STL handles typed-array subviews and scientific notation', () => {
  const buf = new Uint8Array(140); buf.set(new Uint8Array(binary()),3);
  assert.equal(parseSTL(buf.subarray(3,137)).triangles,1);
  assert.equal(parseSTL(bytes(ascii.replace('vertex 1 0 0','vertex 1e+0 -0.0 .0'))).triangles,1);
});
test('STL rejects truncation, malformed facets, nonfinite points, and triangle limits', () => {
  assert.throws(() => parseSTL(new Uint8Array(binary()).subarray(0,130)));
  assert.throws(() => parseSTL(bytes(ascii.replace('vertex 1 0 0','vertex nope 0 0'))));
  assert.throws(() => parseSTL(bytes(ascii.replace('vertex 1 0 0','vertex 1e999 0 0'))));
  const bad = binary(); new DataView(bad).setFloat32(96,NaN,true);
  assert.throws(() => parseSTL(bad),/Non-finite/);
  assert.throws(() => parseSTL(binary(),0),/limit/);
  assert.throws(() => parseSTL(bytes(ascii),0),/limit/);
  assert.throws(() => parseSTL(bytes('solid empty\nendsolid empty')),/empty/);
});
test('no-WebGL gives explicit skeleton warning, correct backgrounds, missing status, and setColor', async () => {
  const c = canvas(), view = new RobotView(c,'#3987e5');
  await view.setModel(model([joint('q','root','a',{origin_xyz:[0,0,1]})]));
  view.render({q:null},'front','dark');
  assert.equal(view.status.fallback,true); assert.match(view.status.warnings.join(' '),/WebGL unavailable/);
  assert.deepEqual(view.status.missingJoints,['q']); assert.equal(c.style.backgroundColor,'#1a1a19');
  assert.ok(c.calls.some(call => call[0]==='text' && call[1].includes('Skeleton only')));
  view.setColor('#d95926'); assert.equal(view.colorHex,'#d95926');
  view.render({q:0},'side','light'); assert.equal(c.style.backgroundColor,'#fcfcfb');
  assert.deepEqual(view.status.missingJoints,[]); assert.throws(() => view.setColor('red'));
  view.dispose(); view.dispose();
});
test('box/sphere/cylinder support, stable neutral frame, and unsupported geometry warnings', async () => {
  const m = model([joint('slide','root','a',{type:'prismatic',axis:[1,0,0]})]);
  m.visuals = [
    {link:'a',geometry:{type:'box',size:[1,2,3]}},
    {link:'root',geometry:{type:'sphere',radius:0.2}},
    {link:'root',geometry:{type:'cylinder',radius:0.2,length:1}}];
  const view = new RobotView(canvas()); await view.setModel(m);
  assert.ok(view.status.triangleCount>12); assert.equal(view.modelWarnings.length,0);
  const frame = JSON.stringify(view.frame);
  view.render({slide:100}); assert.equal(JSON.stringify(view.frame),frame);
  m.visuals.push({link:'root',geometry:{type:'capsule'}}); await view.setModel(m);
  assert.match(view.status.warnings.join(' '),/Unsupported visual geometry/);
  assert.match(view.status.warnings.join(' '),/Incomplete surface model/);
  view.dispose();
});
test('WebGL path uploads packed normals, uses depth, colors unknown meshes gray, and frees buffers', async () => {
  const calls = [], gl = new Proxy({
    COMPILE_STATUS:1,LINK_STATUS:2,NO_ERROR:0,TRIANGLES:3,DEPTH_TEST:4,FLOAT:5,BYTE:6,
    getShaderParameter:()=>true,getProgramParameter:()=>true,getError:()=>0,
    getAttribLocation:(_p,name)=>name==='aPosition'?0:1,getUniformLocation:(_p,name)=>name,
    createShader:()=>({}),createProgram:()=>({}),createBuffer:()=>({}),
  }, {get(target,key){if(key in target)return target[key];return (...args)=>calls.push([key,...args]);}});
  const c = canvas(); c.getContext=type=>type==='webgl'?gl:null;
  const view=new RobotView(c), m=model([joint('q','root','a')]);
  m.visuals=[{link:'a',geometry:{type:'box',size:[1,1,1],scale:[2,1,1]}}];
  await view.setModel(m); view.render({q:null},'front','light');
  assert.equal(view.status.fallback,false);
  assert.ok(calls.some(x=>x[0]==='enable' && x[1]===gl.DEPTH_TEST));
  assert.ok(calls.some(x=>x[0]==='vertexAttribPointer' && x[3]===gl.BYTE && x[4]===true && x[5]===16 && x[6]===12));
  assert.ok(calls.some(x=>x[0]==='drawArrays' && x[1]===gl.TRIANGLES && x[3]===36));
  assert.ok(calls.some(x=>x[0]==='uniform3fv' && x[1]==='uColor' && x[2][0]===0.56));
  view.dispose(); assert.ok(calls.some(x=>x[0]==='deleteBuffer'));
});

test('checker floor has bounded alternating tiles at a stable neutral height', () => {
  const frame={center:[1,2,0],radius:1.5,floorZ:-.8};
  const tiles=floorTiles(frame);
  assert.equal(tiles.length,144);
  assert.equal(tiles.filter(t=>t.shade===0).length,72);
  assert.equal(tiles.filter(t=>t.shade===1).length,72);
  assert.ok(tiles.every(t=>t.points.every(p=>p[2]===-.8)));
  close(tiles[0].points[0],[-2,-1,-.8]);
});
test('manual camera survives new telemetry and theme changes, reset restores selected preset', () => {
  const view=new RobotView(canvas());
  view.render({},'oblique','dark');view.orbit(30,20);view.pan(10,-15);view.zoomBy(1.5);
  const adjusted=JSON.stringify(view.camera);
  view.render({q:1},'oblique','light');assert.equal(JSON.stringify(view.camera),adjusted);
  view.render({},'front','light');assert.deepEqual(view.camera,defaultCamera('front'));
  view.orbit(20,20);view.resetCamera('side');assert.deepEqual(view.camera,defaultCamera('side'));
  view.zoomBy(1e9);assert.equal(view.camera.zoom,8);
  view.zoomBy(1e-20);assert.equal(view.camera.zoom,.2);
  view.orbit(0,1e6);assert.ok(view.camera.pitch<Math.PI/2);
  assert.ok(cameraMatrix(view.frame,view.camera,1).every(Number.isFinite));
  view.dispose();
});
test('pointer capture, wheel, keyboard, cancellation and disposal lifecycle', () => {
  const c=canvas(),listeners=new Map();let captured=null,prevented=0;
  c.addEventListener=(name,fn)=>listeners.set(name,fn);
  c.removeEventListener=name=>listeners.delete(name);
  c.setPointerCapture=id=>{captured=id;};c.hasPointerCapture=id=>captured===id;
  c.releasePointerCapture=()=>{captured=null;};c.focus=()=>{};
  const view=new RobotView(c),event=extra=>({preventDefault(){prevented++;},...extra});
  listeners.get('pointerdown')(event({button:0,pointerId:1,clientX:10,clientY:10}));
  assert.equal(captured,1);const yaw=view.camera.yaw;
  listeners.get('pointermove')(event({pointerId:1,clientX:40,clientY:20}));assert.notEqual(view.camera.yaw,yaw);
  listeners.get('pointercancel')(event({pointerId:1}));assert.equal(view.drag,null);assert.equal(captured,null);
  listeners.get('wheel')(event({deltaY:-100,deltaMode:0}));assert.ok(view.camera.zoom>1);
  listeners.get('keydown')(event({key:'Home'}));assert.deepEqual(view.camera,defaultCamera());
  listeners.get('keydown')(event({key:'ArrowRight',shiftKey:true}));assert.notDeepEqual(view.camera.pan,[0,0,0]);
  assert.ok(prevented>=4);view.dispose();assert.equal(listeners.size,0);
});

test('three concurrent views share a single local mesh download and parsed geometry', async () => {
  const previous = globalThis.fetch; let calls = 0;
  globalThis.fetch = async url => {
    assert.equal(url,'/meshes/981'); calls++;
    return {ok:true,headers:{get(){return '134';}},arrayBuffer:async()=>binary()};
  };
  const m = model([]); m.visuals=[{link:'root',geometry:{type:'mesh',url:'/meshes/981',scale:[1,1,1]}}];
  const views = [0,1,2].map(()=>new RobotView(canvas()));
  try {
    await Promise.all(views.map(v=>v.setModel(m))); assert.equal(calls,1);
    assert.equal(views[0].visuals[0].mesh,views[1].visuals[0].mesh);
    assert.deepEqual(views[0].frame,views[2].frame);
    m.visuals[0].geometry.url='https://example.com/model.stl';
    await views[0].setModel(m); assert.equal(calls,1);
    assert.match(views[0].status.warnings.join(' '),/local \/meshes\/N/);
  } finally { globalThis.fetch=previous; views.forEach(v=>v.dispose()); }
});
