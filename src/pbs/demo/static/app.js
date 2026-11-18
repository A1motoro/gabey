'use strict';
const $=id=>document.getElementById(id), names={reactive:'Reactive',cv:'Predictive · CV'};
let config,data,time=0,playing=false,previous=null,busy=false,worldWidth=60,summaryShown=false;
const controls={gap_d:['Initial gap','m',1],rel_v:['Relative speed · ego − agent','m/s',.5],cut_in_t:['Cut-in starts','s',.1],lc_T:['Lane-change duration','s',.1]};
const mix=(a,b,f)=>a+(b-a)*f,fmt=(v,d=1)=>v==null?'—':v.toFixed(d);
function selection(){return Object.fromEntries(Object.keys(controls).map(k=>[k,Number($(k).value)]));}
function updateField(k){
 $(k+'-value').textContent=`${Number($(k).value).toFixed(1)} ${controls[k][1]}`;
 const relative=Number($('rel_v').value),gap=Number($('gap_d').value),ego=config.fixed.v_ego;
 const catchUp=relative>0?`${fmt(gap/relative,2)} s to catch up without braking`:'no catch-up at constant speeds';
 $('scenario-explanation').textContent=`Inputs: ego ${fmt(ego)} m/s · agent longitudinal speed ${fmt(ego-relative)} m/s · ${catchUp}.`;
}
function setParams(p){for(const k of Object.keys(controls)){$(k).value=p[k];updateField(k);}}
function pause(){playing=false;$('play').textContent='Play';}
function play(){if(!data||busy)return;if(time>=10){time=0;window.demoPresentation?.onReplay();}playing=true;$('play').textContent='Pause';window.demoPresentation?.onPlay(time);}
async function run(){
 pause();busy=true;$('error').hidden=true;window.demoPresentation?.setLoading(true);
 for(const el of $('parameters').querySelectorAll('input,select'))el.disabled=true;
 for(const id of ['run','play','replay','step','timeline'])$(id).disabled=true;
 $('run').textContent='Simulating…';
 try{
  const response=await fetch('/api/simulate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(selection())});
  const result=await response.json();if(!response.ok)throw new Error(result.error);
  data=result;time=0;$('dirty').hidden=true;
  $('source-note').textContent=`${data.source.label||'Reference scenario'} · agent trajectory replay · closed-loop ego braking${data.road_lines==null?' · schematic road':''}`;
  const chosen=config.presets[$('preset').value];$('scene-name').textContent=Object.keys(controls).every(k=>Number($(k).value)===chosen.params[k])?chosen.label:'Custom cut-in';
  for(const n of ['reactive','cv']){const s=data.systems[n];names[n]=s.label;$(n+'-title').textContent=s.label;$(n+'-legend').textContent=s.label;$(n+'-road').setAttribute('aria-label',s.label+' vehicle and cut-in road animation');s.frames=Array.from({length:data.t.length},()=>[]);for(const frame of s.trace)s.frames[Math.round(frame.t/data.fixed.dt)].push(frame);}
  let front=0,back=0;
  data.t.forEach(t=>{for(const n of ['reactive','cv']){const s=data.systems[n],pt=Math.min(t,s.first_collision_t??10),c=panelCenter(n,pt),d=sample(data.t,s.x,pt)-c;front=Math.max(front,d);back=Math.min(back,d);for(const a of data.agents){const ad=sample(a.t,a.x,pt)-c;front=Math.max(front,ad);back=Math.min(back,ad);}}});
  worldWidth=Math.max(55,(front+10)/.7,(-back+8)/.3);
  window.demoPresentation?.onData(data);
 }catch(error){$('error').textContent=error.message+' Adjust parameters or retry.';$('error').hidden=false;}
 finally{for(const el of $('parameters').querySelectorAll('input,select'))el.disabled=false;busy=false;$('run').textContent='Run simulation';$('run').disabled=false;for(const id of ['play','replay','step','timeline'])$(id).disabled=!data;window.demoPresentation?.setLoading(false);render();}
}
function sample(ts,vs,t){let lo=0,hi=ts.length-1;while(lo+1<hi){const m=(lo+hi)>>1;if(ts[m]<=t)lo=m;else hi=m;}return mix(vs[lo],vs[hi],Math.max(0,Math.min(1,(t-ts[lo])/(ts[hi]-ts[lo]||1))));}
function panelCenter(name,t){
 const other=data.systems[name==='cv'?'reactive':'cv'];
 if(other.first_collision_t!=null&&other.first_collision_t<t)return sample(data.t,data.systems[name].x,t);
 return (sample(data.t,data.systems.reactive.x,t)+sample(data.t,data.systems.cv.x,t))/2;
}
function canvasContext(canvas){const r=canvas.getBoundingClientRect(),dpr=devicePixelRatio||1;if(canvas.width!==Math.round(r.width*dpr)||canvas.height!==Math.round(r.height*dpr)){canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);}const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,r.width,r.height);return[ctx,r.width,r.height];}
function rounded(c,x,y,w,h,r,fill){c.fillStyle=fill;c.beginPath();c.roundRect(x,y,w,h,r);c.fill();}
function car(c,x,y,angle,scale,color,brake,label,options={}){
 c.save();c.translate(x,y);c.rotate(-angle);const footprint=data.fixed.r_safe/2.5,l=2.2*scale*footprint,w=1.05*scale*footprint;
 c.shadowColor='#17252150';c.shadowBlur=7;c.shadowOffsetX=3;c.shadowOffsetY=5;
 rounded(c,-l/2,-w/2+3,l,w,4,color==='blue'?'#175581':'#9e501f');c.shadowColor='transparent';
 rounded(c,-l/2,-w/2,l,w-1,4,color==='blue'?'#318ac5':'#e69b4a');
 rounded(c,-l*.22,-w*.38,l*.47,w*.72,3,color==='blue'?'#80bdd8':'#f1c184');
 rounded(c,l*.14,-w*.33,l*.11,w*.62,2,'#244a53');rounded(c,-l*.22,-w*.31,l*.09,w*.58,2,'#35515a');
 c.fillStyle=brake?'#ff302c':'#9a3430';c.fillRect(-l/2-1,-w*.37,3,w*.24);c.fillRect(-l/2-1,w*.15,3,w*.24);
 c.fillStyle='#fff5cd';c.fillRect(l/2-2,-w*.37,2,w*.22);c.fillRect(l/2-2,w*.15,2,w*.22);c.restore();
 const font=options.fontSize??10,boxWidth=options.width??44,boxHeight=font+6,labelY=options.above?y-w/2-boxHeight-6:y+w/2+6;rounded(c,x-boxWidth/2,labelY,boxWidth,boxHeight,3,'#eff3e9');c.fillStyle='#243a35';c.textAlign='center';c.font=`600 ${font}px sans-serif`;c.fillText(label,x,labelY+font+2);
}
function drawRoad(name,k,f,center,panelTime){
 if(window.demoPresentation)return window.demoPresentation.drawRoad({name,k,f,center,panelTime,data,helpers:{canvasContext,car,rounded,sample,fmt}});
 const[c,w,h]=canvasContext($(name+'-road')),s=data.systems[name],scale=w/worldWidth,cy=h*.63,lane=data.lane_width;
 const xy=(x,y)=>[w*.3+(x-center)*scale,cy-y*scale];
 c.fillStyle='#dce3d7';c.fillRect(0,0,w,h);const upper=xy(0,lane*1.5)[1],lower=xy(0,-lane*.5)[1];
 c.fillStyle='#434f4d';c.fillRect(0,upper,w,lower-upper);c.fillStyle='#758174';c.fillRect(0,upper-4,w,4);c.fillRect(0,lower,w,4);
 const lines=data.road_lines??[[[-1000,-lane/2],[10000,-lane/2]],[[-1000,lane/2],[10000,lane/2]],[[-1000,lane*1.5],[10000,lane*1.5]]];
 lines.forEach((line,i)=>{c.beginPath();line.forEach(([x,y],j)=>{const p=xy(x,y);if(j)c.lineTo(...p);else c.moveTo(...p);});c.strokeStyle='#e3e5d6';c.lineWidth=1.5;c.setLineDash(i===1?[12,13]:[]);c.stroke();});c.setLineDash([]);
 c.font='10px sans-serif';c.textAlign='left';c.fillStyle='#556652';c.fillText('ADJACENT LANE',14,Math.max(18,upper-15));c.fillText('EGO LANE  →',14,Math.min(h-12,lower+29));
 const start=center-worldWidth*.3;for(let x=Math.ceil(start/10)*10;x<start+worldWidth;x+=10){const px=xy(x,0)[0];c.strokeStyle='#9eae98';c.beginPath();c.moveTo(px,h-19);c.lineTo(px,h-15);c.stroke();c.fillStyle='#53664f';c.textAlign='center';c.fillText(x+' m',px,h-3);}
 if(name==='cv'&&$('prediction').checked)for(const trace of s.frames[k]){const points=trace.predicted;if(!points.length)continue;const o=trace.observed;c.beginPath();c.moveTo(...xy(o.x.at(-1),o.y.at(-1)));points.forEach(p=>c.lineTo(...xy(...p)));c.strokeStyle='#f3bc72';c.lineWidth=2;c.setLineDash([5,5]);c.stroke();c.setLineDash([]);const p=xy(...points.at(-1));c.beginPath();c.arc(...p,3,0,Math.PI*2);c.stroke();}
 for(const a of data.agents)car(c,...xy(sample(a.t,a.x,panelTime),sample(a.t,a.y,panelTime)),sample(a.t,a.heading,panelTime),scale,'orange',false,'AGENT');
 const next=Math.min(k+1,s.x.length-1);car(c,...xy(mix(s.x[k],s.x[next],f),mix(s.y[k],s.y[next],f)),s.heading[k],scale,'blue',k<s.executed_steps&&s.action[k]==='brake','EGO');
}
function drawChart(k){
 const[c,w,h]=canvasContext($('chart')),left=35,right=w-10,top=13,bottom=h-23,x=t=>left+t/10*(right-left),y=v=>bottom-v/16*(bottom-top);
 c.font='10px sans-serif';c.lineWidth=1;
 for(const v of[0,5,10,15]){c.strokeStyle='#d0d7cd';c.beginPath();c.moveTo(left,y(v));c.lineTo(right,y(v));c.stroke();c.fillStyle='#526360';c.textAlign='right';c.fillText(v,left-8,y(v)+3);}
 for(const t of[0,2,4,6,8,10]){c.fillStyle='#526360';c.textAlign='center';c.fillText(t+' s',x(t),h-5);}c.textAlign='left';c.fillText('m/s',0,10);
 for(const n of['reactive','cv']){c.strokeStyle=n==='cv'?'#176c59':'#80669b';c.lineWidth=2;c.setLineDash(n==='reactive'?[5,3]:[]);c.beginPath();const s=data.systems[n],until=Math.min(time,s.first_collision_t??10);for(let i=0;i<=k&&data.t[i]<=until;i++){const p=[x(data.t[i]),y(s.v[i])];if(i)c.lineTo(...p);else c.moveTo(...p);}c.lineTo(x(until),y(sample(data.t,s.v,until)));c.stroke();const onset=s.summary.brake_onset_t;if(onset!=null&&onset<=until+1e-8){c.setLineDash([2,3]);c.lineWidth=1;c.beginPath();c.moveTo(x(onset),top);c.lineTo(x(onset),bottom);c.stroke();}}
 c.setLineDash([]);c.strokeStyle='#9c5b27';c.lineWidth=1;c.beginPath();c.moveTo(x(time),top);c.lineTo(x(time),bottom);c.stroke();
}
function render(){
 if(!data)return;
 const k=Math.min(Math.floor(time/data.fixed.dt+1e-8),data.t.length-1);
 for(const n of['reactive','cv']){
   const s=data.systems[n],panelTime=Math.min(time,s.first_collision_t??10),hit=s.first_collision_t!=null&&time>=s.first_collision_t;
   const position=Math.min(panelTime/data.fixed.dt,data.t.length-1),j=Math.min(Math.floor(position+1e-8),data.t.length-1),f=Math.max(0,position-j);
   const center=panelCenter(n,panelTime);
   const finished=j>=s.executed_steps,brake=!hit&&!finished&&s.action[j]==='brake',b=$(n+'-action');
   b.textContent=hit?'COLLISION · FROZEN':finished?'FINISHED':brake?'BRAKE':'CRUISE';
   b.className='action'+(brake||hit?' brake':'')+(finished&&!hit?' finished':'');
   const onset=s.summary.brake_onset_t;
   $(n+'-status').textContent=`First brake: ${onset!=null&&onset<=panelTime+1e-8?fmt(onset,2)+' s':'—'} · panel t = ${fmt(panelTime,2)} s`;
   const count=hit?s.summary.brake_count:s.braking.count[j],duration=hit?s.summary.brake_duration_s:s.braking.duration_s[j];
   $(n+'-metrics').innerHTML=`<div><small>Ego speed</small><strong>${fmt(sample(data.t,s.v,panelTime))}</strong><em>m/s</em></div><div><small>Estimated TTC</small><strong>${fmt(s.ttc[j])}</strong><em>s</em></div><div><small>Center distance</small><strong>${fmt(hit?s.summary.min_dist:s.distance[j])}</strong><em>m</em></div><div title="BRAKE command episodes / cumulative commanded time"><small>Brake events · time</small><strong>${count}</strong><em> / ${fmt(duration,2)} s</em></div>`;
   drawRoad(n,j,f,center,panelTime);
 }
 $('timeline').value=time;$('clock').textContent=`${time.toFixed(2)} / 10.0 s`;const p=data.params;$('phase').textContent=time>=10?'Run complete':time<p.cut_in_t?'Before cut-in':time<p.cut_in_t+p.lc_T?'Agent changing lanes':'Agent in ego lane';drawChart(k);
 if(time>=10&&!summaryShown){
   const rows=['reactive','cv'].map(n=>{
     const s=data.systems[n].summary;
     return `<tr><td>${names[n]}</td><td>${s.brake_onset_t==null?'Not triggered':fmt(s.brake_onset_t)+' s'}</td><td>${s.brake_count}</td><td>${fmt(s.brake_duration_s,2)} s</td><td>${fmt(s.min_dist,2)} m</td><td>${s.collision?'Yes':'No'}</td><td>${fmt(s.metrics_until_t,2)} s</td></tr>`;
   }).join('');
   const events=['reactive','cv'].flatMap(n=>data.systems[n].braking.episodes.map(e=>`<tr><td>${names[n]}</td><td>${fmt(e.start_t,2)} s</td><td>${e.release_t==null?'Held at panel end':fmt(e.release_t,2)+' s'}</td><td>${fmt(e.duration_s,2)} s</td></tr>`)).join('');
   $('summary').innerHTML='<table><thead><tr><th>System</th><th>First brake</th><th>Events</th><th>Brake time</th><th>Min. distance</th><th>Collision</th><th>Panel end</th></tr></thead><tbody>'+rows+'</tbody></table>'+`<span class="lead">${data.lead_time_s==null?'Lead time unavailable: one or both systems did not brake.':data.lead_time_s>=0?'Predictive brakes '+fmt(data.lead_time_s,2)+' s earlier.':'Predictive brakes '+fmt(-data.lead_time_s,2)+' s later.'}</span>`+(events?'<details class="brake-log"><summary>Brake / release event log</summary><p>One event = entry into BRAKE. Each side is measured until contact or run end.</p><table><thead><tr><th>System</th><th>Start</th><th>Release</th><th>Duration</th></tr></thead><tbody>'+events+'</tbody></table></details>':'');
   summaryShown=true;
 }
 else if(time<10){if(summaryShown)$('summary').textContent='Play to the end to compare braking onset and clearance.';summaryShown=false;}
 window.demoPresentation?.render({data,time,playing,busy});
}
function tick(stamp){if(playing&&previous!=null){const target=Math.min(10,time+Math.min((stamp-previous)/1000,.1)*Number($('speed').value)),stop=window.demoPresentation?.advance(time,target);time=stop?.time??target;if(stop?.pause||time>=10)pause();render();}previous=stamp;requestAnimationFrame(tick);}
async function init(){try{
 const response=await fetch('/api/config');if(!response.ok)throw new Error('Could not load configuration.');config=await response.json();
 $('preset').innerHTML=Object.entries(config.presets).map(([key,p])=>`<option value="${key}">${p.label}</option>`).join('');
 $('fields').innerHTML=Object.entries(controls).map(([k,[label,unit,step]])=>`<div class="field"><label for="${k}">${label}<output id="${k}-value" for="${k}"></output></label><input type="range" id="${k}" min="${config.ranges[k][0]}" max="${config.ranges[k][1]}" step="${step}"></div>`).join('');setParams(config.defaults);
 for(const k of Object.keys(controls))$(k).addEventListener('input',()=>{updateField(k);$('dirty').hidden=false;window.demoPresentation?.onParameters();});
 $('preset').addEventListener('change',()=>{setParams(config.presets[$('preset').value].params);$('dirty').hidden=false;window.demoPresentation?.onParameters();});
 $('parameters').addEventListener('submit',e=>{e.preventDefault();run();});$('play').addEventListener('click',()=>{if(playing)pause();else play();render();});
 $('replay').addEventListener('click',()=>{time=0;window.demoPresentation?.onReplay();play();render();});$('step').addEventListener('click',()=>{pause();time=Math.min(10,(Math.floor(time/.1+1e-6)+1)*.1);window.demoPresentation?.onSeek(time);render();});
 $('timeline').addEventListener('input',()=>{pause();time=Number($('timeline').value);window.demoPresentation?.onSeek(time);render();});$('prediction').addEventListener('change',render);window.addEventListener('resize',render);document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});new ResizeObserver(render).observe($('reactive-road'));
 await run();requestAnimationFrame(tick);
 }catch(error){$('error').textContent=error.message+' Reload the page to retry.';$('error').hidden=false;$('run').disabled=true;}}
window.demoPresentation?.attach({
 state:()=>({data,time,playing,busy}),
 seek:t=>{pause();time=Math.max(0,Math.min(10,t));window.demoPresentation.onSeek(time);render();},
 pause,render,
 toggle:()=>{if(playing)pause();else play();render();}
});
init();
