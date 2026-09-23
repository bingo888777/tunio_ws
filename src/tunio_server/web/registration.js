'use strict';

// #98: 접수와 완료는 다르다. 응답보다 먼저 온 SSE도 보관한다.
// 새로고침/구독 누락만으로 등록 완료를 추정하지 않는다.
class RegistrationTracker {
  constructor() {
    this.busy = false;
    this.message = '사람이 페그를 돌렸다면 재등록 후 시작하세요. 재등록은 등록·복귀만 수행합니다.';
  }
  begin(state) {
    this.busy = true;
    this.accepted = false;
    this.baseline = state.stamp;
    this.events = [];
    this.job = null;
    this.returned = false;
    this.gap = false;
    this.message = '페그 재등록 요청 중…';
  }
  reply(accepted, message) {
    if (!accepted) {
      this.busy = false;
      this.events = [];
      this.message = '재등록 거부 — ' + (message || '사유 없음');
      return;
    }
    this.accepted = true;
    this.message = '재등록 접수 — 등록·복귀 완료를 기다립니다.';
    const events = this.events;
    this.events = [];
    events.forEach(s => this.observe(s));
  }
  observe(s) {
    if (!this.busy || !s || !(s.stamp > this.baseline)) return;
    if (!this.accepted) { this.events.push(s); return; }
    if (s.phase === 12 && s.job_id) {
      if (this.job && this.job !== s.job_id) { this.unknown('다른 작업 감지 — 재등록 결과 확인 필요'); return; }
      this.job = s.job_id;
      this.message = `${s.current_string}번 페그 재등록 중 — 완료 후 대기 자세로 복귀합니다.`;
    } else if (s.job_id === this.job && this.job) {
      if (s.phase === 7) {
        this.returned = true;
        this.message = '페그 등록 종료 — 대기 자세 복귀 중';
      } else if (s.phase === 11 || s.phase === 10) {
        this.busy = false;
        this.message = s.phase === 11 ? '재등록 실패 — 로봇 로그를 확인하세요.' : '재등록 취소 — 완료되지 않았습니다.';
      } else if (s.phase === 9) {
        this.message = '페그 재등록 일시정지';
      }
    } else if (s.phase === 0 && !s.job_id) {
      this.busy = false;
      this.message = this.job && this.returned && !this.gap
        ? '페그 재등록·복귀 완료 — 시작을 눌러 조율하세요.'
        : '대기 상태 — 재등록 완료 경로를 확인하지 못했습니다. 로그를 확인하세요.';
    }
  }
  disconnect() {
    if (!this.busy) return;
    this.gap = true;
    this.message = '연결 끊김 — 재등록 완료 여부 확인 필요. 자동 재요청하지 않습니다.';
  }
  unknown(message) {
    this.busy = false;
    this.events = [];
    this.message = message;
  }
}
if (typeof module !== 'undefined') module.exports = RegistrationTracker;
