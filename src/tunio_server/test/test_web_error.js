// node src/tunio_server/test/test_web_error.js
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(__dirname + '/../web/app.js', 'utf8');
new vm.Script(source);
const begin = source.indexOf('function renderJobError(state) {');
const end = source.indexOf('\nfunction renderLive(state)', begin);
const element = {hidden: true, textContent: ''};
const context = {PHASE: {ERROR: 11}, $: id => {
  assert.equal(id, 'job-error'); return element;
}};
vm.createContext(context);
vm.runInContext(source.slice(begin, end), context);
const message = '6번 페그 등록값 없음 — 페그 재등록을 먼저 실행하세요';
context.renderJobError({phase:11, error_message:message});
assert.equal(element.hidden, false);
assert(element.textContent.includes(message));
// 새로고침/재연결로 같은 상태를 받아도 안내가 남는다.
context.renderJobError({phase:11, error_message:message});
assert.equal(element.hidden, false);
context.renderJobError({phase:12, error_message:''});
assert.equal(element.hidden, true);
assert.equal(element.textContent, '');
// 오류 문구를 HTML로 해석하지 않는다.
context.renderJobError({phase:11, error_message:'<img onerror=alert(1)>'});
assert.equal(element.textContent, '작업 중단: <img onerror=alert(1)>');
context.renderJobError(null);
assert.equal(element.hidden, true);
console.log('오류 안내 표시·재수신·초기화·텍스트 렌더링 통과');
