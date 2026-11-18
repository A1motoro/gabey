/* Presentation-only UI. Python still owns prediction, risk and policy. */
(function () {
  'use strict';
  const byId = id => document.getElementById(id);
  const eventModel = window.DemoEvents;
  let api, groups = [], loading = true, zeroAcknowledged = false;
  const eventButtons = ['event-prev', 'event-next', 'event-cv', 'event-reactive', 'event-collision'];
  const text = (element, value) => { if (element.textContent !== value) element.textContent = value; };
  const seconds = t => t.toFixed(2) + ' s';
  const selectedMode = () => byId('mode-events').checked ? 'events' : 'full';
  function updateModeHint() {
    text(byId('playback-mode-note'), selectedMode() === 'events'
      ? 'Pauses at first brakes and contact. Click Play to continue.'
      : 'Plays through key events. Collided panels still freeze.');
  }

  function jump(group) { if (group && !loading) api.seek(group.t); }
  function shortcut(kind, system) {
    return groups.find(g => g.events.some(e => e.kind === kind && (!system || e.system === system))) || null;
  }
  function updateNavigation(state) {
    const ready = Boolean(state.data) && !loading;
    const targets = {
      'event-prev': eventModel.previous(groups, state.time),
      'event-next': eventModel.next(groups, state.time),
      'event-cv': shortcut('first_brake', 'cv'),
      'event-reactive': shortcut('first_brake', 'reactive'),
      'event-collision': shortcut('collision')
    };
    const labels = {
      'event-cv': 'Predictive first brake',
      'event-reactive': 'Reactive first brake',
      'event-collision': 'First collision'
    };
    for (const id of eventButtons) {
      const button = byId(id), target = targets[id];
      button.disabled = !ready || !target;
      if (labels[id]) {
        text(button, labels[id] + (target ? ' · ' + seconds(target.t) : ' · none'));
        button.title = target ? 'Jump to ' + seconds(target.t) + ' and pause' : 'This event did not occur in the run';
      }
    }
  }
  function updateParameters() {
    text(byId('parameters-toggle'), 'Adjust parameters · gap ' + Number(byId('gap_d').value).toFixed(1) +
      ' m · relative speed ' + Number(byId('rel_v').value).toFixed(1) + ' m/s');
  }

  window.demoPresentation = {
    attach(controller) {
      api = controller;
      byId('event-prev').addEventListener('click', () => jump(eventModel.previous(groups, api.state().time)));
      byId('event-next').addEventListener('click', () => jump(eventModel.next(groups, api.state().time)));
      byId('event-cv').addEventListener('click', () => jump(shortcut('first_brake', 'cv')));
      byId('event-reactive').addEventListener('click', () => jump(shortcut('first_brake', 'reactive')));
      byId('event-collision').addEventListener('click', () => jump(shortcut('collision')));
      ['mode-events','mode-full'].forEach(id => byId(id).addEventListener('change', () => { updateModeHint(); api.render(); }));
      document.addEventListener('keydown', event => {
        if (event.ctrlKey || event.metaKey || event.altKey || event.target.closest('input,select,textarea,[contenteditable="true"]')) return;
        if (event.code === 'Space' && !event.target.closest('button,summary,a')) { event.preventDefault(); api.toggle(); }
        if (event.key.toLowerCase() === 'n') jump(eventModel.next(groups, api.state().time));
        if (event.key.toLowerCase() === 'p') jump(eventModel.previous(groups, api.state().time));
      });
    },
    setLoading(value) {
      loading = value;
      if (value) {
        eventButtons.forEach(id => { byId(id).disabled = true; });
        text(byId('event-message'), 'Computing the scenario…');
        text(byId('event-next-label'), '');
      }
    },
    onData(data) {
      groups = eventModel.buildGroups(data);
      zeroAcknowledged = false;
      updateParameters();
    },
    onParameters: updateParameters,
    onSeek() { zeroAcknowledged = true; },
    onReplay() { zeroAcknowledged = false; },
    onPlay(t) {
      // A time-zero event pauses once, and resumes normally on the next Play.
      if (selectedMode() === 'events' && !zeroAcknowledged && groups[0]?.t === 0 && t === 0) {
        zeroAcknowledged = true;
        api.pause();
        api.render();
      }
    },
    advance(from, to) {
      const event = eventModel.pauseTarget(groups, from, to, selectedMode());
      return event ? {time:event.t, pause:true} : null;
    },
    render(state) {
      if (loading) return;
      updateNavigation(state);
      const current = eventModel.latest(groups, state.time);
      const next = eventModel.next(groups, state.time);
      text(byId('event-message'), current
        ? seconds(current.t) + ' — ' + current.events.map(e => e.label).join(' · ')
        : groups.length ? 'Ready. Play or jump to a key event.' : 'No braking or collision events in this run.');
      text(byId('event-next-label'), next
        ? 'Next: ' + seconds(next.t) + ' · ' + next.events.map(e => e.label).join(' · ')
        : groups.length ? 'No further key events.' : 'Playback remains available.');
    },
    drawRoad({name, k, f, center, panelTime, data, helpers}) {
      const {canvasContext, car, rounded, sample, fmt} = helpers;
      const [c, w, h] = canvasContext(byId(name + '-road'));
      const side = data.systems[name], lane = data.lane_width;
      // Fit both panels at the same scale before contact. A frozen side queries
      // this function at its own immutable contact time, so camera/zoom freeze too.
      let windowMetres = 24;
      for (const key of ['reactive', 'cv']) {
        const s = data.systems[key], t = Math.min(panelTime, s.first_collision_t ?? 10);
        const other = data.systems[key === 'cv' ? 'reactive' : 'cv'];
        const ownX = sample(data.t, s.x, t);
        const origin = other.first_collision_t != null && other.first_collision_t < t
          ? ownX : (sample(data.t, data.systems.reactive.x, t) + sample(data.t, data.systems.cv.x, t)) / 2;
        const points = [ownX, ...data.agents.map(a => sample(a.t, a.x, t))];
        for (const x of points) windowMetres = Math.max(windowMetres, (x - origin + 4) / .7, (origin - x + 4) / .3);
      }
      const scale = Math.min(w / windowMetres, (h - 60) / (2 * lane));
      const cy = h / 2 + lane * scale / 2;
      const xy = (x, y) => [w * .3 + (x - center) * scale, cy - y * scale];
      const upper = xy(0, lane * 1.5)[1], lower = xy(0, -lane / 2)[1];
      c.fillStyle = '#dce3d7'; c.fillRect(0, 0, w, h);
      c.fillStyle = '#434f4d'; c.fillRect(0, upper, w, lower - upper);
      c.fillStyle = '#758174'; c.fillRect(0, upper - 3, w, 3); c.fillRect(0, lower, w, 3);
      const lines = data.road_lines ?? [[[-1000,-lane/2],[10000,-lane/2]],[[-1000,lane/2],[10000,lane/2]],[[-1000,lane*1.5],[10000,lane*1.5]]];
      lines.forEach((line, i) => {
        c.beginPath(); line.forEach(([x,y], j) => { const p = xy(x,y); if (j) c.lineTo(...p); else c.moveTo(...p); });
        c.strokeStyle = '#e3e5d6'; c.lineWidth = 1.6; c.setLineDash(i === 1 ? [14,14] : []); c.stroke();
      });
      c.setLineDash([]); c.font = '12px sans-serif'; c.textAlign = 'left'; c.fillStyle = '#f0f3e8';
      c.fillText('ADJACENT LANE', 12, upper + 18); c.fillText('EGO LANE', 12, lower - 12);
      const start = center - w * .3 / scale, tick = windowMetres < 40 ? 5 : 10;
      c.textAlign = 'center'; c.fillStyle = '#3f573d';
      for (let x = Math.ceil(start / tick) * tick; x < start + w / scale; x += tick) {
        const px = xy(x,0)[0]; if (px < 24 || px > w - 24) continue; c.fillText(x + ' m', px, h - 5);
      }
      const frames = side.frames[k] || [];
      if (name === 'cv' && byId('prediction').checked) for (const frame of frames) {
        if (!frame.predicted.length) continue;
        const o = frame.observed; c.beginPath(); c.moveTo(...xy(o.x.at(-1),o.y.at(-1)));
        frame.predicted.forEach(p => c.lineTo(...xy(...p)));
        c.strokeStyle = '#f3bc72'; c.lineWidth = 2.3; c.setLineDash([6,5]); c.stroke(); c.setLineDash([]);
      }
      const next = Math.min(k + 1, side.x.length - 1);
      const ego = xy(side.x[k] + (side.x[next] - side.x[k]) * f, side.y[k] + (side.y[next] - side.y[k]) * f);
      const finite = frames.filter(t => t.considered && t.ttc != null).sort((a,b) => a.ttc - b.ttc)[0];
      const agent = finite ? data.agents.find(a => a.agent_id === finite.agent_id) :
        [...data.agents].sort((a,b) => Math.hypot(sample(a.t,a.x,panelTime)-sample(data.t,side.x,panelTime), sample(a.t,a.y,panelTime)-sample(data.t,side.y,panelTime)) -
          Math.hypot(sample(b.t,b.x,panelTime)-sample(data.t,side.x,panelTime), sample(b.t,b.y,panelTime)-sample(data.t,side.y,panelTime)))[0];
      if (agent) {
        const target = xy(sample(agent.t,agent.x,panelTime), sample(agent.t,agent.y,panelTime));
        c.beginPath(); c.moveTo(...ego); c.lineTo(...target); c.strokeStyle = '#d7e9e9'; c.lineWidth = 2; c.setLineDash([5,5]); c.stroke(); c.setLineDash([]);
        const frame = frames.find(t => t.agent_id === agent.agent_id);
        const hit = side.first_collision_t != null && panelTime >= side.first_collision_t;
        let label;
        if (hit) label = 'Contact at ' + seconds(side.first_collision_t) + ' · frozen';
        else if (frame?.considered === false) label = side.label === 'Reactive' ? 'Outside ego lane · not assessed' : 'Not assessed';
        else if (frame?.ttc != null) label = 'Assessed TTC  ' + fmt(frame.ttc,2) + ' s';
        else label = 'No assessed conflict';
        c.font = (frame?.ttc != null && !hit ? '600 17px' : '600 13px') + ' sans-serif';
        const boxWidth = Math.min(w - 20, c.measureText(label).width + 24);
        const midX = (ego[0] + target[0]) / 2, midY = (ego[1] + target[1]) / 2;
        const labelX = Math.max(boxWidth / 2 + 8, Math.min(w - boxWidth / 2 - 8, midX));
        c.beginPath(); c.moveTo(midX,midY); c.lineTo(labelX,34); c.strokeStyle = '#a9c2c0'; c.lineWidth = 1; c.stroke();
        const warning = hit || (k < side.executed_steps && side.action[k] === 'brake');
        rounded(c,labelX-boxWidth/2,7,boxWidth,28,5,warning?'#ffede6':'#f4f7ed');
        c.fillStyle = warning ? '#9b2f25' : '#203f39'; c.textAlign = 'center'; c.fillText(label,labelX,26);
      }
      for (const a of data.agents) car(c,...xy(sample(a.t,a.x,panelTime),sample(a.t,a.y,panelTime)),sample(a.t,a.heading,panelTime),scale,'orange',false,'AGENT',{fontSize:13,width:60,above:true});
      const hit = side.first_collision_t != null && panelTime >= side.first_collision_t;
      car(c,...ego,side.heading[k],scale,'blue',!hit && k < side.executed_steps && side.action[k] === 'brake','EGO',{fontSize:13,width:48});
    }
  };
})();
