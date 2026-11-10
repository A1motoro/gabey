/* Pure event navigation: no DOM, simulation decisions or browser timers. */
(function (root) {
  'use strict';
  const EPS = 1e-8;
  function buildGroups(data) {
    const events = [];
    for (const [system, side] of Object.entries(data.systems)) {
      const onset = side.summary.brake_onset_t;
      if (onset != null) events.push({id:system+':first_brake', t:onset, kind:'first_brake', system, label:side.label+' first issues BRAKE'});
      if (side.first_collision_t != null) events.push({id:system+':collision', t:side.first_collision_t, kind:'collision', system, label:side.label+' reaches contact; panel freezes'});
    }
    events.sort((a,b)=>a.t-b.t || a.id.localeCompare(b.id));
    const groups=[];
    for (const event of events) {
      const last=groups[groups.length-1];
      if (last && Math.abs(last.t-event.t)<=EPS) last.events.push(event);
      else groups.push({t:event.t, events:[event]});
    }
    return groups;
  }
  function next(groups, t) {return groups.find(g=>g.t>t+EPS) || null;}
  function previous(groups, t) {return [...groups].reverse().find(g=>g.t<t-EPS) || null;}
  function crossed(groups, from, to) {
    if (to<from) return null;
    return groups.find(g=>g.t>from+EPS && g.t<=to+EPS) || null;
  }
  function pauseTarget(groups, from, to, mode) {
    if (mode === 'full') return null;
    if (mode !== 'events') throw new RangeError('Unknown playback mode');
    return crossed(groups, from, to);
  }
  function latest(groups, t) {return [...groups].reverse().find(g=>g.t<=t+EPS) || null;}
  const api={EPS,buildGroups,next,previous,crossed,latest,pauseTarget};
  root.DemoEvents=api;
  if(typeof module!=='undefined' && module.exports) module.exports=api;
})(typeof window!=='undefined'?window:globalThis);
