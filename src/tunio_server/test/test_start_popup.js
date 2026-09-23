const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(__dirname + '/../web/app.js', 'utf8');
const begin = source.indexOf('async function sendCommand(name)');
const end = source.indexOf('function renderSettingsLock', begin);
const alerts = [], toasts = [];
let accepted = false;
const context = {
  commandPending:false, streamReady:true, lastState:{phase:0},
  registration:{busy:false}, isRunning:()=>false, renderButtons:()=>{},
  fetch:async()=>({ok:true,json:async()=>({accepted,message:'페그 재등록을 먼저 실행하세요'})}),
  window:{alert:msg=>alerts.push(msg)}, toast:msg=>toasts.push(msg),
};
vm.createContext(context);
vm.runInContext(source.slice(begin,end),context);
(async()=>{
  await context.sendCommand('start');
  assert.equal(alerts.length,1);
  assert(alerts[0].includes('재등록을 먼저'));
  assert.equal(toasts.length,0);
  assert.equal(context.commandPending,false);
  accepted=true;
  await context.sendCommand('start');
  assert.equal(alerts.length,1);
  assert.equal(toasts.length,1);
  assert(context.registration.message.includes('다음 조율'));
  console.log('시작 거부 팝업·접수 안내·다음 조율 재등록 문구 통과');
})();
