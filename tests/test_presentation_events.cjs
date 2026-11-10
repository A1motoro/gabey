const test = require('node:test');
const assert = require('node:assert/strict');
const events = require('../src/pbs/demo/static/presentation-events.js');
const side = (label, onset, contact=null) => ({label,summary:{brake_onset_t:onset},first_collision_t:contact});
const fixture = () => ({systems:{reactive:side('Reactive',3,3.12),cv:side('Predictive · CV',1.7)}});

test('first brake and contact events are chronological and retain the correct side', () => {
  const groups=events.buildGroups(fixture());
  assert.deepEqual(groups.map(g=>g.t),[1.7,3,3.12]);
  assert.equal(groups[0].events[0].system,'cv');
  assert.equal(groups[2].events[0].kind,'collision');
});
test('coincident events are one stop containing all descriptions', () => {
  const groups=events.buildGroups({systems:{reactive:side('R',2,2),cv:side('P',2)}});
  assert.equal(groups.length,1);
  assert.equal(groups[0].events.length,3);
  assert.equal(events.crossed(groups,1.9,2.1).t,2);
  assert.equal(events.crossed(groups,2,2.1),null);
});
test('large playback jumps stop at the first crossed event and resume beyond it', () => {
  const groups=events.buildGroups(fixture());
  assert.equal(events.crossed(groups,1.6,3.5).t,1.7);
  assert.equal(events.crossed(groups,1.7,3.5).t,3);
  assert.equal(events.crossed(groups,3,3.5).t,3.12);
  assert.equal(events.crossed(groups,3.12,8),null);
});
test('backwards seeks and replay make earlier events reachable again', () => {
  const groups=events.buildGroups(fixture());
  assert.equal(events.previous(groups,3.12).t,3);
  assert.equal(events.previous(groups,1.7),null);
  assert.equal(events.next(groups,0).t,1.7);
  assert.equal(events.crossed(groups,8,0),null);
  assert.equal(events.crossed(groups,0,2).t,1.7);
  assert.equal(events.latest(groups,2).t,1.7);
});
test('runs without braking or contact have no fake stops', () => {
  const groups=events.buildGroups({systems:{reactive:side('R',null),cv:side('P',null)}});
  assert.deepEqual(groups,[]);
  assert.equal(events.next(groups,0),null);
  assert.equal(events.previous(groups,10),null);
  assert.equal(events.latest(groups,10),null);
});
test('time-zero contact is available for the initial pause and explicit navigation', () => {
  const groups=events.buildGroups({systems:{reactive:side('R',null,0),cv:side('P',null)}});
  assert.equal(groups[0].t,0);
  assert.equal(events.latest(groups,0).t,0);
  assert.equal(events.next(groups,0),null);
});

test('full process mode passes every key event without stopping', () => {
  const groups=events.buildGroups(fixture());
  assert.equal(events.pauseTarget(groups,0,10,'full'),null);
  assert.equal(events.pauseTarget(groups,1.6,3.5,'full'),null);
});
test('event mode stops at the next event and mode switches do not revisit passed events', () => {
  const groups=events.buildGroups(fixture());
  assert.equal(events.pauseTarget(groups,0,10,'events').t,1.7);
  assert.equal(events.pauseTarget(groups,1.7,10,'events').t,3);
  assert.equal(events.pauseTarget(groups,3.01,10,'events').t,3.12);
  assert.equal(events.pauseTarget(groups,3.12,10,'events'),null);
});
test('unknown playback modes fail instead of silently changing playback', () => {
  assert.throws(()=>events.pauseTarget(events.buildGroups(fixture()),0,10,'other'),RangeError);
});
