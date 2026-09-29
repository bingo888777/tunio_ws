// node test/test_registration.js (Node.js) 또는 VM으로 동일 소스를 실행한다.
const assert = require('node:assert/strict');
const Tracker = require('../web/registration.js');
const state = (phase, stamp, job_id = 'new', current_string = 6) => ({phase, stamp, job_id, current_string});
const fresh = () => { const t = new Tracker(); t.begin(state(0, 10, '')); return t; };
const success = t => t.message.includes('재등록·복귀 완료');
let t = fresh();
t.reply(true); t.observe(state(0, 10, '')); assert(t.busy); assert(!success(t));
t.observe(state(12, 11)); t.observe(state(7, 12)); t.observe(state(0, 13, ''));
assert(success(t)); assert(!t.busy);
// 응답보다 모든 상태가 먼저 와도 접수 확인 전에는 완료가 아니다.
t = fresh(); [state(12,11),state(7,12),state(0,13,'')].forEach(s=>t.observe(s));
assert(!success(t)); t.reply(true); assert(success(t));
// 같은 버퍼가 있어도 접수 거부이면 완료가 아니다.
t = fresh(); t.observe(state(12,11)); t.observe(state(7,12)); t.observe(state(0,13,''));
t.reply(false,'진행 중'); assert(!success(t)); assert(t.message.includes('거부'));
for (const phase of [10,11]) {
  t=fresh(); t.reply(true); t.observe(state(12,11)); t.observe(state(phase,12));
  t.observe(state(0,13,'')); assert(!success(t)); assert(!t.busy);
}
t=fresh(); t.reply(true); t.observe(state(12,11)); t.observe(state(0,12,'')); assert(!success(t));
t=fresh(); t.reply(true); t.observe(state(12,11)); t.disconnect();
t.observe(state(7,12)); t.observe(state(0,13,'')); assert(!success(t));
t=fresh(); t.reply(true); t.observe(state(12,11)); t.observe(state(12,12,'other')); assert(!t.busy); assert(!success(t));
t=fresh(); t.reply(true); t.observe(state(12,11)); t.observe(state(9,12)); assert(t.message.includes('일시정지')); assert(t.busy);
t=new Tracker(); t.observe(state(0,13,'')); assert(!success(t));
console.log('재등록 상태 추적 10개 시나리오 통과');
