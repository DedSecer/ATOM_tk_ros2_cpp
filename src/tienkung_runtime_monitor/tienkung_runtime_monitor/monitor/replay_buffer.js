// Bounded, contiguous replay cache; frame identity, not timestamp, distinguishes duplicates.
export const MAX_REPLAY_FRAMES = 3000;
export function frameAtTime(frames, seconds) {
  let low=0, high=frames.length;
  while(low<high){const middle=(low+high)>>>1;if(frames[middle].replay_time_sec<=seconds)low=middle+1;else high=middle;}
  return frames[Math.max(0,low-1)] || null;
}
export function appendReplayFrames(existing, incoming, selectedIndex, historyStart) {
  if(!incoming.length)throw Error('回放预取没有返回新帧');
  const last=existing.at(-1);
  if(last&&incoming[0].replay_frame_index!==last.replay_frame_index+1)throw Error('回放缓存不连续，请重新定位');
  for(let i=1;i<incoming.length;i++){
    if(incoming[i].replay_frame_index!==incoming[i-1].replay_frame_index+1 || incoming[i].replay_time_sec<incoming[i-1].replay_time_sec)throw Error('回放片段顺序异常');
  }
  const joined=existing.concat(incoming);
  // Retain the selected sample even during underrun; discard only obsolete history.
  let start=0;
  while(start<joined.length && joined[start].replay_frame_index<selectedIndex &&
      (joined[start].replay_time_sec<historyStart || joined.length-start>MAX_REPLAY_FRAMES))start++;
  if(joined.length-start>MAX_REPLAY_FRAMES)throw Error('回放预取超过缓存上限');
  return joined.slice(start);
}
export function prefetchNeeded(frames, selected, frameCount, speed, latencySec) {
  const last=frames.at(-1);
  if(!last||!selected||last.replay_frame_index>=frameCount-1)return false;
  const secondsLeft=(last.replay_time_sec-selected.replay_time_sec)/speed;
  return secondsLeft<=Math.max(1,Math.min(5,latencySec*3+.3));
}
