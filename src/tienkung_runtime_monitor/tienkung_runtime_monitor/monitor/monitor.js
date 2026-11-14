import {RobotView} from './robot_view.js';
import {frameAtTime, appendReplayFrames, prefetchNeeded} from './replay_buffer.js';

const $ = id => document.getElementById(id);
const finite = Number.isFinite;
const at = (a, i) => Array.isArray(a) && finite(a[i]) ? a[i] : null;
const fmt = (v, digits = 3) => finite(v) ? v.toFixed(digits) : '—';
const truth = v => v === true ? '✓' : v === false ? '✕' : '—';
const styles = () => getComputedStyle(document.documentElement);
const color = i => styles().getPropertyValue(`--series-${i + 1}`).trim();
let frames = [], metadata = null, generation = null, sequence = 0, snapshot = null;
let paused = false, displayed = [], selectedFrame = null, modelKey = '', views = [];
let lastResponse = 0, disconnected = false;
let replay = null, playing = false, replayBusy = false, replayError = '', replayRequest = 0;
let replayAbort = null, replayWall = 0, replayAnchor = 0, replaySpeed = 1, replayTimer = null;
let scrubbing = false, seekTimer = null, playIntent = 0;
let prefetchAbort = null, prefetchLatency = .5, replayBuffering = false;
let lastDashboardRender = -Infinity, lastHistoryRender = -Infinity;
const frameTime = f => replay ? f.replay_time_sec : f.timestamp;
const names = () => metadata?.joint_order_policy || [];
const config = () => metadata?.config || {};
const index = () => Number($('joint').value) || 0;
const jointCount = f => Math.max(names().length, f?.dof_pos?.length || 0, f?.target?.length || 0);
const nameAt = i => names()[i] || `joint[${i}]（缺少名称映射）`;
function groupFor(i) {
  return ['leg', 'arm', 'head', 'waist'].find(g => config()[`${g}_indices`]?.includes(i));
}
function validActual(f, i) {
  if (f?.motor_feedback?.filtered?.valid?.[i] !== true) return false;
  const group = groupFor(i), age = f.input_age_sec?.[group];
  const timeout = metadata?.runtime?.state_timeout_sec;
  return !!group && finite(age) && age >= 0 && (!finite(timeout) || age <= timeout);
}
function actual(f, i) { return validActual(f, i) ? at(f.dof_pos, i) : null; }
function published(f, i) {
  const id = (metadata?.can_id_by_index || config().can_id_by_index || [])[i];
  return f?.command_published === true && ['leg', 'arm', 'waist'].some(g => f.command?.[g]?.some(c => c.name === id));
}
function reference(f, i) { const a = f?.effective_motion_ref; return a?.length === jointCount(f) + 6 ? at(a, i + 6) : null; }
function tracking(f, i) {
  const a = actual(f, i), t = at(f?.target, i);
  return published(f, i) && f.mode !== 'STOP' && finite(a) && finite(t) ? t - a : null;
}
function current(f, i) { return validActual(f, i) && f.motor_feedback?.raw_valid?.[i] === true ? at(f.motor_feedback.raw_current, i) : null; }
function series(label, slot, read) { return {label, slot, read}; }
const specs = [
  {title:'关节位置跟踪', unit:'rad', series:[series('Reference',0,reference),series('Target',1,(f,i)=>at(f.target,i)),series('Actual',2,actual)]},
  {title:'网络原始输出', unit:'raw action', series:[series('Raw action',1,(f,i)=>at(f.raw_action,i))]},
  {title:'实际关节速度', unit:'rad/s', series:[series('Velocity',2,(f,i)=>validActual(f,i)?at(f.dof_vel,i):null)]},
  {title:'关节力矩估计（非力矩传感器）', unit:'Nm · estimated', series:[series('Torque estimate',2,(f,i)=>validActual(f,i)?at(f.dof_torque_estimated,i):null)]},
  {title:'电机电流（驱动器原始单位）', unit:'current', series:[series('Current',2,current)]},
  {title:'机身姿态', unit:'rad', series:[series('Roll',0,f=>at(f.rpy,0)),series('Pitch',1,f=>at(f.rpy,1))]},
  {title:'推理耗时', unit:'ms', series:[series('Inference',0,f=>f.inference_executed?f.inference_latency_ms:null)]},
  {title:'控制周期 / 处理耗时', unit:'ms', series:[series('Period',0,f=>finite(f.timing?.tick_dt_sec)?f.timing.tick_dt_sec*1000:null),series('Processing',1,f=>f.timing?.processing_ms)]}
];
const charts = specs.map((spec, id) => {
  const card = document.createElement('article'); card.className = 'chart-card';
  const title = document.createElement('h2'); title.textContent = spec.title; card.append(title);
  const legend = document.createElement('div'); legend.className = 'legend';
  if (spec.series.length > 1) spec.series.forEach(s => {
    const item = document.createElement('span'), key = document.createElement('i');
    key.className = 'key'; key.style.background = `var(--series-${s.slot+1})`; item.append(key,document.createTextNode(s.label)); legend.append(item);
  });
  const canvas = document.createElement('canvas'); canvas.className='chart'; canvas.tabIndex=0; canvas.setAttribute('aria-label',spec.title+'，左右方向键查看数据');
  const readout = document.createElement('div'); readout.className='readout'; readout.id=`readout-${id}`; canvas.setAttribute('aria-describedby',readout.id);
  card.append(legend,canvas,readout); $('charts').append(card);
  const chart = {...spec,canvas,readout,hover:null};
  canvas.addEventListener('pointermove', e => { const r=canvas.getBoundingClientRect(); chart.hover=Math.round(Math.max(0,Math.min(1,(e.clientX-r.left-60)/Math.max(1,r.width-180)))*(displayed.length-1)); drawChart(chart); });
  canvas.addEventListener('pointerleave',()=>{chart.hover=null;drawChart(chart);});
  canvas.addEventListener('focus',()=>{chart.hover=displayed.length-1;drawChart(chart);});
  canvas.addEventListener('keydown',e=>{if(!['ArrowLeft','ArrowRight'].includes(e.key))return;e.preventDefault();chart.hover=Math.max(0,Math.min(displayed.length-1,(chart.hover??displayed.length-1)+(e.key==='ArrowLeft'?-1:1)));drawChart(chart);});
  return chart;
});
function canvasContext(canvas) {
  const r=canvas.getBoundingClientRect(), dpr=Math.min(devicePixelRatio||1,2);
  const w=Math.max(1,r.width),h=Math.max(1,r.height);
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);return {ctx,w,h};
}
function drawChart(chart) {
  const {ctx,w,h}=canvasContext(chart.canvas), css=styles(), ink=css.getPropertyValue('--secondary'), text=css.getPropertyValue('--text');
  const left=60,right=120,top=20,bottom=32,pw=Math.max(1,w-left-right),ph=h-top-bottom;
  const values=chart.series.flatMap(s=>displayed.map(f=>s.read(f,index()))).filter(finite);
  let lo=values.length?Math.min(...values):0, hi=values.length?Math.max(...values):1;
  const pad=Math.max((hi-lo)*.1,.01);lo-=pad;hi+=pad;
  const t0=displayed.length?frameTime(displayed[0]):0,t1=displayed.length?frameTime(displayed.at(-1)):t0+1;
  const x=f=>left+(frameTime(f)-t0)/Math.max(.001,t1-t0)*pw,y=v=>top+(hi-v)/(hi-lo)*ph;
  ctx.font='11px system-ui';ctx.fillStyle=ink;ctx.strokeStyle=css.getPropertyValue('--grid');ctx.lineWidth=1;ctx.setLineDash([]);
  for(let k=0;k<5;k++){const value=lo+(hi-lo)*k/4,py=y(value);ctx.beginPath();ctx.moveTo(left,py);ctx.lineTo(left+pw,py);ctx.stroke();ctx.textAlign='right';ctx.fillText(fmt(value,2),left-7,py+4);}
  ctx.textAlign='left';ctx.fillText(chart.unit,left,12);ctx.textAlign='center';ctx.fillText(`${fmt(t0,2)} → ${fmt(t1,2)} s`,left+pw/2,h-8);
  const labels=[];
  chart.series.forEach((s,k)=>{
    ctx.strokeStyle=color(s.slot);ctx.lineWidth=2;ctx.lineJoin='round';ctx.setLineDash($('patterns').checked?[[0],[7,4],[2,4]][s.slot]:[]);ctx.beginPath();let connected=false,last=null,prior=null;
    displayed.forEach(f=>{const v=s.read(f,index());if(!finite(v)){connected=false;prior=null;return;}if(prior&&(frameTime(f)-frameTime(prior)>.15||frameTime(f)<frameTime(prior)))connected=false;
      if(connected)ctx.lineTo(x(f),y(v));else ctx.moveTo(x(f),y(v));connected=true;last={x:x(f),y:y(v),v};prior=f;});ctx.stroke();ctx.setLineDash([]);
    if(last)labels.push({...last,label:s.label,slot:s.slot});
  });
  labels.sort((a,b)=>a.y-b.y);let previous=-20;
  labels.forEach(l=>{const ly=Math.max(top+8,Math.min(h-bottom-3,Math.max(l.y,previous+16)));previous=ly;ctx.strokeStyle=color(l.slot);ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(l.x,l.y);ctx.lineTo(left+pw+7,ly);ctx.stroke();ctx.fillStyle=text;ctx.textAlign='left';ctx.fillText(`${l.label} ${fmt(l.v,2)}`,left+pw+10,ly+4);});
  const frame=displayed[Math.max(0,Math.min(displayed.length-1,chart.hover??displayed.length-1))];
  if(frame){if(chart.hover!==null){ctx.strokeStyle=ink;ctx.beginPath();ctx.moveTo(x(frame),top);ctx.lineTo(x(frame),h-bottom);ctx.stroke();}chart.readout.textContent=`t=${fmt(frameTime(frame),3)} · ${frame.mode}\n`+chart.series.map(s=>`${s.label}: ${fmt(s.read(frame,index()))}`).join('  |  ');}else chart.readout.textContent='等待有效数据';
}
function tableRow(values) { const tr=document.createElement('tr');values.forEach(v=>{const td=document.createElement('td');td.textContent=String(v);tr.append(td);});return tr; }
function populateJoints(f) {
  const count=jointCount(f), signature=JSON.stringify(names())+count;
  if($('joint').dataset.signature===signature)return;
  const prior=index();$('joint').replaceChildren();for(let i=0;i<count;i++){const option=document.createElement('option');option.value=i;option.textContent=`[${i}] ${nameAt(i)}`;$('joint').append(option);}$('joint').value=String(Math.min(prior,Math.max(0,count-1)));$('joint').dataset.signature=signature;
}
function drawGauge(canvas,values,slot) {
  const {ctx,w,h}=canvasContext(canvas);const n=values.length,cols=Math.max(1,Math.ceil(Math.sqrt(n*w/h))),rows=Math.ceil(n/cols),cw=w/cols,ch=h/Math.max(1,rows),radius=Math.min(cw*.3,ch*.28);
  ctx.font='10px system-ui';ctx.textAlign='center';
  values.forEach((q,i)=>{const x=(i%cols+.5)*cw,y=(Math.floor(i/cols)+.45)*ch;ctx.strokeStyle=styles().getPropertyValue('--grid');ctx.lineWidth=1;ctx.beginPath();ctx.arc(x,y,radius,0,2*Math.PI);ctx.stroke();if(finite(q)){ctx.strokeStyle=color(slot);ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+Math.sin(q)*radius,y-Math.cos(q)*radius);ctx.stroke();}ctx.fillStyle=styles().getPropertyValue('--text');ctx.fillText(`${i}: ${fmt(q,2)}`,x,y+radius+14);});
}
function renderPoses(f) {
  const sets=[Array.from({length:jointCount(f)},(_,i)=>reference(f,i)),Array.from({length:jointCount(f)},(_,i)=>at(f.target,i)),Array.from({length:jointCount(f)},(_,i)=>actual(f,i))];
  if(views.length){views.forEach((v,k)=>{v.setColor?.(color(k));v.render(Object.fromEntries(names().map((name,i)=>[name,sets[k][i]])),$('camera').value,document.documentElement.dataset.theme);});}
  else ['reference','target','actual'].forEach((key,k)=>drawGauge($(`pose-${key}`),sets[k],k));
}
function renderHistory() {
  if(!$('history-details').open)return;
  const i=index();$('history-table').replaceChildren(...displayed.slice(-1500).map(f=>tableRow([fmt(frameTime(f)),f.mode,fmt(reference(f,i)),fmt(at(f.target,i)),fmt(actual(f,i)),fmt(at(f.raw_action,i)),fmt(validActual(f,i)?at(f.dof_vel,i):null),fmt(validActual(f,i)?at(f.dof_torque_estimated,i):null),fmt(current(f,i)),fmt(at(f.rpy,0)),fmt(at(f.rpy,1)),fmt(f.inference_executed?f.inference_latency_ms:null),fmt(finite(f.timing?.tick_dt_sec)?f.timing.tick_dt_sec*1000:null)])));
}
function render(options = {}) {
  const playbackTick=options.playbackTick===true,now=performance.now();
  // Model poses remain responsive; expensive charts/DOM refresh at most 5 Hz during playback.
  if(playbackTick&&selectedFrame&&now-lastDashboardRender<200){renderPoses(selectedFrame);return;}
  lastDashboardRender=now;
  if(!frames.length){
    displayed=[];selectedFrame=null;
    ['mode','latency','period','ready','error'].forEach(id=>$(id).textContent='—');
    $('joint-table').replaceChildren();$('history-table').replaceChildren();
    $('inputs').textContent='等待新采样';$('commands').textContent='等待新采样';
    charts.forEach(drawChart);renderPoses({});updateStatus();return;
  }
  if(replay && selectedFrame){const end=frameTime(selectedFrame),start=end-Number($('window').value);displayed=frames.filter(f=>frameTime(f)>=start&&f.replay_frame_index<=selectedFrame.replay_frame_index);}
  else if(!paused){selectedFrame=frames.at(-1);const start=selectedFrame.timestamp-Number($('window').value);displayed=frames.filter(f=>finite(f.timestamp)&&f.timestamp>=start&&f.timestamp<=selectedFrame.timestamp);}
  const f=selectedFrame;if(!f)return;populateJoints(f);const i=index();
  $('mode').textContent=`${f.mode} / ${f.command_published?'已发布':'未发布'}`;
  $('latency').textContent=f.inference_executed?`${fmt(f.inference_latency_ms)} ms`:'未推理';
  $('period').textContent=finite(f.timing?.tick_dt_sec)?`${fmt(f.timing.tick_dt_sec*1000,2)} ms`:'—';
  $('ready').textContent=`${truth(f.state_ready)} state / ${truth(f.motion_ready)} motion`;
  $('error').textContent=`${fmt(tracking(f,i))} rad`;
  $('joint-table').replaceChildren(...Array.from({length:jointCount(f)},(_,j)=>{
    const blocked=config().blocked_action_indices?.includes(j),raw=at(f.raw_action,j),limit=config().clip_actions/config().action_scale;
    const flags=[published(f,j)?'已发布':'未发布',validActual(f,j)?'反馈有效':'反馈无效/过期',blocked?'阻断':null,finite(raw)&&finite(limit)&&Math.abs(raw)>limit?'裁剪':null].filter(Boolean).join(' · ');
    return tableRow([`${nameAt(j)} / ${metadata?.can_id_by_index?.[j]??'—'}`,fmt(reference(f,j)),fmt(raw),fmt(at(f.target,j)),fmt(actual(f,j)),fmt(tracking(f,j)),fmt(validActual(f,j)?at(f.dof_vel,j):null),fmt(validActual(f,j)?at(f.dof_torque_estimated,j):null),flags]);
  }));
  $('inputs').textContent=JSON.stringify({input_age_sec:f.input_age_sec,state_ready:f.state_ready,motion_ready:f.motion_ready,motion_reference_valid:f.motion_reference_valid,motion_reference_state:f.motion_reference_state,timing:f.timing,dropped_records:f.dropped_records,parse_errors:snapshot?.parse_errors,skipped_lines:snapshot?.skipped_lines,logger_stats:snapshot?.logger_stats,source_age_sec:snapshot?.source_age_sec,last_error:snapshot?.last_error},null,2);
  $('commands').textContent=JSON.stringify({command_published:f.command_published,command:f.command},null,2);
  charts.forEach(drawChart);renderPoses(f);
  if(!playbackTick||now-lastHistoryRender>=1000){renderHistory();lastHistoryRender=now;}
  updateStatus();
}
function updateStatus() {
  if(replay){
    const error=replayError||replay.error;
    $('connection').textContent=error?'⚠ 回放错误':replay.status==='indexing'?'◌ 建立索引中':replayBusy?'◌ 加载回放片段':replayBuffering?'◌ 缓冲后续帧':playing?'▶ 离线播放':'Ⅱ 离线已暂停';
    $('freshness').textContent=`历史采样 / ${selectedFrame?.dropped_records??'—'}`;
    $('notice').textContent=error?`⚠ ${error}`:'离线回放：所有姿态、readiness 和命令发布标记均为历史记录，不代表当前机器人状态。';
    if(snapshot?.truncated)$('notice').textContent+=' 当前曲线窗口因帧数或字节上限被截短，不影响跳转到其他时刻。';
    updateReplayControls();return;
  }
  const responseAge=lastResponse?(performance.now()-lastResponse)/1000:Infinity;
  const age=finite(snapshot?.age_sec)?snapshot.age_sec+responseAge:null;
  const stale=disconnected||responseAge>2||!finite(age)||age>1;
  $('connection').textContent=disconnected?'⚠ 服务连接中断':stale?'⚠ 无新采样 / 已过期':paused?'Ⅱ 显示已暂停':'● 数据接收中';
  $('freshness').textContent=`${fmt(age,1)} s / ${selectedFrame?.dropped_records??'—'}`;
  const notices=[];
  if(stale)notices.push('⚠ 没有新鲜采样；保留最后一帧仅供检查，不代表机器人当前姿态。');
  if(paused)notices.push('Ⅱ 页面显示已暂停，后台仍接收；恢复后跳到最新帧。');
  if(snapshot?.truncated)notices.push('⚠ 超出监控历史窗口，较旧帧已省略；完整数据请查磁盘日志。');
  if(!metadata)notices.push('⚠ 缺少 schema v2 metadata，不能可靠映射 URDF 或判定反馈有效性。');
  if(selectedFrame?.motion_reference_valid===false)notices.push('⚠ 原始动作参考无效/过期；有效参考可能为插值、保持帧或默认帧。');
  $('notice').textContent=notices.join('\n')||'只读监控正常接收。readiness 不是安全认证；请独立核对控制模式与实机状态。';
}
async function setupModel(model) {
  const key=JSON.stringify(model);if(key===modelKey)return;modelKey=key;
  views.forEach(v=>v.dispose?.());views=[];
  // A canvas cannot switch between a 2D and WebGL context; replace when model changes.
  ['reference','target','actual'].forEach(k=>{const old=$(`pose-${k}`),fresh=old.cloneNode(false);old.replaceWith(fresh);});
  if(!model){$('pose-note').textContent='未提供 URDF：显示逐关节角度仪表，不代表机器人几何形状。';return;}
  $('pose-note').textContent='加载 URDF / STL；按 joint_order_policy 名称映射。灰色关节表示无有效数据（仅零位上下文），不代表真实测量。';
  const newViews=['reference','target','actual'].map((k,i)=>new RobotView($(`pose-${k}`),color(i)));views=newViews;
  try{await Promise.all(newViews.map(v=>v.setModel(model)));if(modelKey===key){$('pose-note').textContent='URDF 已载入；按 joint_order_policy 名称映射。灰色部位表示缺少有效数据（仅零位上下文）。模型加载警告见各视图下方。';render();}}catch(e){$('pose-note').textContent=`模型加载失败：${e.message}`;}
}
function updateReplayControls() {
  if(!replay)return;
  const ready=replay.status==='ready'&&replay.frame_count>0;
  const total=replay.total_bytes||0,progress=total?100*(replay.indexed_bytes||0)/total:0;
  $('replay-progress').textContent=replay.status==='indexing'?`建立索引 ${fmt(progress,1)}% · 已索引 ${replay.frame_count||0} 帧`:`${replay.frame_count||0} 帧 · ${fmt(replay.duration_sec,3)} s · 时钟：${replay.clock?.source||'日志时间'} · 时钟回退 ${replay.clock?.backward_jumps||0} 次`;
  $('replay-play').disabled=!ready||replayBusy||!!replay.error;
  $('replay-play').textContent=playing?'暂停':'播放';
  $('replay-prev').disabled=!ready||replayBusy||!selectedFrame||selectedFrame.replay_frame_index===0;
  $('replay-next').disabled=!ready||replayBusy||!selectedFrame||selectedFrame.replay_frame_index>=replay.frame_count-1;
  $('replay-jump').disabled=!ready||!!replay.error;
  $('replay-timeline').disabled=!ready||!!replay.error;
  $('replay-timeline').max=String(replay.duration_sec||0);$('replay-seconds').max=String(replay.duration_sec||0);
  if(selectedFrame){
    if(!scrubbing)$('replay-timeline').value=String(selectedFrame.replay_time_sec);
    $('replay-position').textContent=`${fmt(selectedFrame.replay_time_sec,3)} / ${fmt(replay.duration_sec,3)} s · 帧 ${selectedFrame.replay_frame_index+1} / ${replay.frame_count} · ROS timestamp ${fmt(selectedFrame.timestamp,6)} · 后续缓存 ${fmt(Math.max(0,(frames.at(-1)?.replay_time_sec??0)-selectedFrame.replay_time_sec),1)} s${prefetchAbort?'（后台加载中）':''}`;
  }
}
function cancelPrefetch() {prefetchAbort?.abort();prefetchAbort=null;}
function stopReplay() {playIntent++;playing=false;replayBuffering=false;cancelPrefetch();clearTimeout(replayTimer);replayTimer=null;if(selectedFrame)render();else updateStatus();}
async function prefetchReplay() {
  if(prefetchAbort||replayBusy||!playing||!selectedFrame)return;
  const last=frames.at(-1);if(!last||last.replay_frame_index>=replay.frame_count-1)return;
  const ticket=replayRequest,controller=new AbortController();prefetchAbort=controller;
  const started=performance.now(),timeout=setTimeout(()=>controller.abort(),30000);
  // No history: append only unseen frames. Moderate payloads keep JSON parsing off the hot path.
  const params=new URLSearchParams({frame:String(last.replay_frame_index+1),window_sec:'0',ahead_sec:String(Math.min(12,Math.max(4,4*replaySpeed)))});
  try{
    const response=await fetch(`/api/replay?${params}`,{cache:'no-store',signal:controller.signal});
    const data=await response.json();
    if(controller!==prefetchAbort||ticket!==replayRequest)return;
    if(!response.ok)throw Error(data.error||`HTTP ${response.status}`);
    if(data.playback?.status!=='ready'){if(data.playback)replay=data.playback;throw Error(data.playback?.error||'回放预取失败');}
    frames=appendReplayFrames(frames,data.frames||[],selectedFrame.replay_frame_index,selectedFrame.replay_time_sec-Number($('window').value));
    prefetchLatency=.5*prefetchLatency+.5*(performance.now()-started)/1000;
    // Do not touch selectedFrame or the playback clock on successful prefetch.
    replayError='';
  }catch(e){
    if(controller!==prefetchAbort||ticket!==replayRequest)return;
    replayError=e.name==='AbortError'?'后续帧读取超时，请点击播放重试':e.message;stopReplay();
  }finally{clearTimeout(timeout);if(prefetchAbort===controller)prefetchAbort=null;}
}
async function seekReplay(value,byFrame=false,resume=false) {
  if(replay?.status!=='ready'||!finite(value))return;
  clearTimeout(replayTimer);replayTimer=null;cancelPrefetch();replayBuffering=false;
  const intent=playIntent;
  const ticket=++replayRequest;replayAbort?.abort();const controller=new AbortController();replayAbort=controller;
  replayBusy=true;playing=false;replayError='';updateStatus();
  const params=new URLSearchParams({[byFrame?'frame':'time']:String(Math.max(0,byFrame?Math.min(replay.frame_count-1,Math.floor(value)):Math.min(replay.duration_sec,value))),window_sec:$('window').value,ahead_sec:String(Math.min(12,Math.max(4,4*replaySpeed)))});
  const timeout=setTimeout(()=>controller.abort(),30000);
  try{
    const response=await fetch(`/api/replay?${params}`,{cache:'no-store',signal:controller.signal});
    const data=await response.json();if(ticket!==replayRequest)return;
    if(!response.ok)throw Error(data.error||`HTTP ${response.status}`);
    if(data.playback?.status!=='ready'||!data.selected_frame){if(data.playback)replay=data.playback;throw Error(data.playback?.error||'没有可回放的采样');}
    snapshot=data;replay=data.playback;metadata=data.metadata;frames=data.frames||[];selectedFrame=data.selected_frame;
    if(!frames.some(f=>f.replay_frame_index===selectedFrame.replay_frame_index))frames.push(selectedFrame);
    frames.sort((a,b)=>a.replay_frame_index-b.replay_frame_index);
    replayBusy=false;paused=true;disconnected=false;lastResponse=performance.now();
    charts.forEach(c=>c.hover=null);render();
    if(resume&&intent===playIntent&&!document.hidden)startReplay();
  }catch(e){if(ticket!==replayRequest)return;replayBusy=false;replayError=e.name==='AbortError'?'片段读取超时，请重试':e.message;stopReplay();}
  finally{clearTimeout(timeout);}
}
function replayTick() {
  if(!playing||replayBusy||!selectedFrame)return;
  const now=performance.now(),last=frames.at(-1);
  if(replayBuffering){
    if(last.replay_frame_index>selectedFrame.replay_frame_index){replayBuffering=false;replayAnchor=selectedFrame.replay_time_sec;replayWall=now;}
    else {void prefetchReplay();replayTimer=setTimeout(replayTick,33);return;}
  }
  const desired=Math.min(replay.duration_sec,replayAnchor+(now-replayWall)/1000*replaySpeed);
  const chosen=frameAtTime(frames,desired)||selectedFrame;
  if(chosen.replay_frame_index!==selectedFrame.replay_frame_index){selectedFrame=chosen;render({playbackTick:true});}
  if(desired>last.replay_time_sec&&last.replay_frame_index<replay.frame_count-1){
    // Freeze at the last real frame only on genuine underrun, never jump over unseen data.
    replayBuffering=true;replayAnchor=selectedFrame.replay_time_sec;replayWall=now;updateStatus();void prefetchReplay();
  }else if(prefetchNeeded(frames,selectedFrame,replay.frame_count,replaySpeed,prefetchLatency))void prefetchReplay();
  if(desired>=replay.duration_sec&&selectedFrame.replay_frame_index>=replay.frame_count-1){stopReplay();return;}
  replayTimer=setTimeout(replayTick,33);
}
function startReplay() {
  if(replayBusy||!selectedFrame||replay.status!=='ready'||replay.error)return;
  if(selectedFrame.replay_frame_index>=replay.frame_count-1){void seekReplay(0,true,true);return;}
  playing=true;replayError='';replayBuffering=false;replayAnchor=selectedFrame.replay_time_sec;replayWall=performance.now();updateStatus();
  if(prefetchNeeded(frames,selectedFrame,replay.frame_count,replaySpeed,prefetchLatency))void prefetchReplay();
  clearTimeout(replayTimer);replayTimer=setTimeout(replayTick,0);
}
$('replay-play').addEventListener('click',()=>playing?stopReplay():startReplay());
$('replay-prev').addEventListener('click',()=>{stopReplay();void seekReplay(selectedFrame.replay_frame_index-1,true);});
$('replay-next').addEventListener('click',()=>{stopReplay();void seekReplay(selectedFrame.replay_frame_index+1,true);});
$('replay-jump').addEventListener('click',()=>{stopReplay();void seekReplay($('replay-seconds').valueAsNumber);});
$('replay-seconds').addEventListener('keydown',e=>{if(e.key==='Enter')$('replay-jump').click();});
$('replay-speed').addEventListener('change',()=>{replaySpeed=Number($('replay-speed').value);if(playing)startReplay();});
$('replay-timeline').addEventListener('input',()=>{scrubbing=true;stopReplay();$('replay-position').textContent=`跳转预览 ${fmt(Number($('replay-timeline').value),3)} s`;clearTimeout(seekTimer);seekTimer=setTimeout(()=>void seekReplay(Number($('replay-timeline').value)),150);});
$('replay-timeline').addEventListener('change',()=>{scrubbing=false;clearTimeout(seekTimer);void seekReplay(Number($('replay-timeline').value));});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&replay)stopReplay();});
async function poll() {
  try {
    const response=await fetch(`/api/snapshot?after=${sequence}`,{cache:'no-store',signal:AbortSignal.timeout(4000)});
    if(!response.ok)throw Error(`HTTP ${response.status}`);
    const data=await response.json();lastResponse=performance.now();disconnected=false;
    if(data.playback?.mode==='replay'){
      replay=data.playback;snapshot=data;metadata=data.metadata;
      $('replay-controls').hidden=false;$('pause').hidden=true;document.querySelector('h1').textContent='离线运动回放';
      $('metadata').textContent=JSON.stringify(metadata,null,2);$('source').textContent=data.source||'离线文件';
      void setupModel(data.urdf);updateStatus();
      if(replay.status==='ready'){await seekReplay(0,true);return;}
      if(replay.status==='error')return;
      setTimeout(poll,500);return;
    }
    if(generation!==data.generation){generation=data.generation;frames=[];displayed=[];selectedFrame=null;paused=false;$('pause').textContent='暂停显示';}
    snapshot=data;metadata=data.metadata;sequence=data.last_seq||0;
    frames.push(...(data.frames||[]).filter(f=>finite(f.timestamp)));frames=frames.slice(-1500);
    $('metadata').textContent=JSON.stringify(metadata,null,2);$('source').textContent=data.source||'等待日志文件';
    void setupModel(data.urdf);render();updateStatus();
  }catch(e){disconnected=true;$('connection').textContent=`⚠ 连接失败：${e.message}`;updateStatus();}
  setTimeout(poll,200);
}
$('pause').addEventListener('click',()=>{paused=!paused;$('pause').textContent=paused?'恢复实时':'暂停显示';render();});
$('theme').addEventListener('change',()=>{document.documentElement.dataset.theme=$('theme').value;render();});
['joint','patterns'].forEach(id=>$(id).addEventListener('change',render));
$('window').addEventListener('change',()=>{if(replay&&selectedFrame)void seekReplay(selectedFrame.replay_frame_index,true,playing);else render();});
const resetCameras=()=>{views.forEach(view=>view.resetCamera($('camera').value));render();};
$('camera').addEventListener('change',resetCameras);
$('reset-camera').addEventListener('click',resetCameras);
$('history-details').addEventListener('toggle',renderHistory);window.addEventListener('resize',render);
setInterval(updateStatus,500);void poll();
