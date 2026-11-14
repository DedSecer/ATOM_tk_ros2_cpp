import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source=await readFile(new URL('../tienkung_runtime_monitor/monitor/replay_buffer.js',import.meta.url),'utf8');
const {frameAtTime,appendReplayFrames,prefetchNeeded,MAX_REPLAY_FRAMES}=await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const frame=(i,time=i*.02)=>({replay_frame_index:i,replay_time_sec:time});
test('time lookup returns most recent real frame including duplicate times',()=>{
  const frames=[frame(0,0),frame(1,1),frame(2,1),frame(3,2)];
  assert.equal(frameAtTime(frames,1).replay_frame_index,2);
  assert.equal(frameAtTime(frames,1.9).replay_frame_index,2);
  assert.equal(frameAtTime(frames,10).replay_frame_index,3);
  assert.equal(frameAtTime([],0),null);
});
test('prefetch begins before exhaustion, scales with speed and latency, stops at EOF',()=>{
  const frames=[frame(0,0),frame(100,2)];
  assert.equal(prefetchNeeded(frames,frame(0,0),200,1,.1),false);
  assert.equal(prefetchNeeded(frames,frame(75,1.5),200,1,.1),true);
  assert.equal(prefetchNeeded(frames,frame(0,0),200,4,.1),true);
  assert.equal(prefetchNeeded(frames,frame(0,0),200,1,1),true);
  assert.equal(prefetchNeeded(frames,frame(75,1.5),101,1,1),false);
});
test('incremental append preserves selection and retains history without duplicate frames',()=>{
  const existing=Array.from({length:100},(_,i)=>frame(i));
  const selected=existing[75];
  const merged=appendReplayFrames(existing,[frame(100),frame(101)],75,1);
  assert.equal(merged[0].replay_frame_index,50);
  assert.equal(merged.at(-1).replay_frame_index,101);
  assert.equal(merged.find(f=>f.replay_frame_index===75),selected);
  assert.equal(existing.length,100);
  assert.throws(()=>appendReplayFrames(existing,[frame(102)],75,0),/不连续/);
  assert.throws(()=>appendReplayFrames(existing,[frame(100),frame(102)],75,0),/顺序异常/);
});
test('cache remains bounded and cannot silently evict selected or future frames',()=>{
  const existing=Array.from({length:MAX_REPLAY_FRAMES},(_,i)=>frame(i));
  const merged=appendReplayFrames(existing,[frame(MAX_REPLAY_FRAMES)],1000,0);
  assert.equal(merged.length,MAX_REPLAY_FRAMES);
  assert.equal(merged[0].replay_frame_index,1);
  assert.throws(()=>appendReplayFrames(existing,[frame(MAX_REPLAY_FRAMES)],0,0),/上限/);
});
