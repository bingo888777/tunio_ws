'use strict';

// TUNIO 관제 웹 — 화면 로직.
// 상태는 SSE(/api/events)로 받고, 명령·설정·이력은 fetch 로 주고받는다.
// 열거값의 뜻은 서버(web_node.py)가 라벨까지 같이 내려주므로 여기서 다시 해석하지 않는다.
// 다만 버튼 활성 판단에는 단계 번호가 필요해 상수를 둔다 — tunio_interfaces 의 TuningState 와 같다.

const PHASE = {
  IDLE: 0, MEASURE_READY: 1, PLUCK: 2, ANALYZE: 3, ADJUST_PEG: 4,
  VERIFY: 5, STRUM: 6, RETURN: 7, DONE: 8, PAUSED: 9, CANCELED: 10, ERROR: 11,
};

// 대상 줄과 이름 (CLAUDE.md 4절). 6→5→2→1 순서로 처리한다.
const STRINGS = [
  { n: 6, name: 'E2' },
  { n: 5, name: 'A2' },
  { n: 2, name: 'B3' },
  { n: 1, name: 'E4' },
];

// 기타 그림용 6줄. 4·3번은 조율하지 않지만 빠지면 기타로 안 보인다.
// thick 은 현 굵기(저음현이 굵다). TARGET 은 STRINGS 에 있는 줄만 true.
const FRETBOARD = [
  { n: 6, name: 'E2', thick: 4.2 },
  { n: 5, name: 'A2', thick: 3.6 },
  { n: 4, name: 'D3', thick: 3.0 },
  { n: 3, name: 'G3', thick: 2.5 },
  { n: 2, name: 'B3', thick: 2.0 },
  { n: 1, name: 'E4', thick: 1.6 },
];
const TARGET_STRINGS = new Set(STRINGS.map((s) => s.n));

// 미터가 보여주는 센트 범위. 실제 튜너들이 대개 ±50 을 쓴다.
const METER_RANGE = 50;

const SETTING_FIELDS = [
  'tolerance_cents', 'max_adjust_count', 'max_measure_retry',
  'measure_timeout_sec', 'onset_timeout_sec',
];

const $ = (id) => document.getElementById(id);

let lastState = null;
let defaults = null;
let selectedJob = null;

// ─────────────────────────────── 표시 헬퍼

function hz(v) {
  return (v === null || v === undefined) ? '—' : Number(v).toFixed(2);
}

function cents(v) {
  if (v === null || v === undefined) return '—';
  const n = Number(v);
  return (n > 0 ? '+' : '') + n.toFixed(1);
}

function phaseClass(phase) {
  if (phase === PHASE.DONE) return 'badge badge-ok';
  if (phase === PHASE.ERROR) return 'badge badge-bad';
  if (phase === PHASE.PAUSED || phase === PHASE.CANCELED) return 'badge badge-warn';
  if (phase === PHASE.IDLE) return 'badge badge-idle';
  return 'badge badge-run';
}

function overallClass(v) {
  const cls = ['badge-idle', 'badge-ok', 'badge-warn', 'badge-bad'][v];
  return 'badge ' + (cls || 'badge-idle');
}

function stringName(n) {
  const s = STRINGS.find((x) => x.n === n);
  return s ? `${n}번 ${s.name}` : `${n}번`;
}

function whenText(epoch) {
  if (!epoch) return '—';
  const d = new Date(epoch * 1000);
  const p = (x) => String(x).padStart(2, '0');
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ` +
         `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
}

let toastTimer = null;
function toast(message, ok) {
  const el = $('toast');
  el.textContent = message;
  el.className = 'toast show ' + (ok ? 'ok' : 'bad');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.className = 'toast'; }, 6000);
}

// ─────────────────────────────── 실시간 화면

function isRunning(state) {
  if (!state) return false;
  return (state.phase >= PHASE.MEASURE_READY && state.phase <= PHASE.RETURN)
      || state.phase === PHASE.PAUSED;
}

// 센트 오차 = 1200 * log2(측정 / 목표) (CLAUDE.md 4절)
function centsOf(measured, target) {
  if (!measured || !target || measured <= 0 || target <= 0) return null;
  return 1200 * Math.log2(measured / target);
}

function centsText(v) {
  if (v === null || !isFinite(v)) return '—';
  const r = Math.round(v * 10) / 10;
  return (r > 0 ? '+' : '') + r.toFixed(1);
}

// 기타 헤드스톡을 그린다. 줄 하나가 <g class="gstring">, 상태는 class 로만 바꾼다.
function buildGuitar() {
  const W = 260, TOP = 22, GAP = 30, PEG_X = 42, NUT_X = 92, END_X = 236;
  const H = TOP + GAP * (FRETBOARD.length - 1) + 26;

  const parts = FRETBOARD.map((st, i) => {
    const y = TOP + GAP * i;
    return `<g class="gstring" data-n="${st.n}">
      <line class="gs-line" x1="${NUT_X}" y1="${y}" x2="${END_X}" y2="${y}"
            stroke-width="${st.thick}" stroke-linecap="round"></line>
      <line class="gs-line" x1="${PEG_X + 9}" y1="${y}" x2="${NUT_X}" y2="${y}"
            stroke-width="1.4"></line>
      <circle class="gs-peg" cx="${PEG_X}" cy="${y}" r="9" stroke-width="2"></circle>
      <text class="gs-label" x="${PEG_X}" y="${y + 4.5}" text-anchor="middle">${st.n}</text>
      <text class="gs-note" x="${END_X}" y="${y - 7}" text-anchor="end">${st.name}</text>
    </g>`;
  }).join('');

  return `<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="기타 줄 상태">
    <rect x="${NUT_X - 6}" y="${TOP - 16}" width="6"
          height="${GAP * (FRETBOARD.length - 1) + 32}" rx="2" fill="#3a4450"></rect>
    ${parts}
  </svg>`;
}

function renderGuitar(state) {
  const wrap = $('guitar');
  if (!wrap.firstChild) wrap.innerHTML = buildGuitar();

  const byNumber = new Map();
  if (state) state.results.forEach((r) => byNumber.set(r.string_number, r));
  const running = isRunning(state);

  wrap.querySelectorAll('.gstring').forEach((g) => {
    const n = Number(g.dataset.n);
    let cls = 'gstring';
    if (!TARGET_STRINGS.has(n)) {
      cls += ' off';                       // 4·3번 — 조율하지 않는다
    } else if (state && state.current_string === n && running) {
      cls += ' current';
    } else {
      const r = byNumber.get(n);
      if (r && r.result === 1) cls += ' ok';
      else if (r && r.result === 2) cls += ' bad';
      else cls += ' wait';
    }
    g.setAttribute('class', cls);
  });
}

// 튜너 바늘과 큰 음이름.
function renderTuner(state) {
  const noteEl = $('now-note');
  const octEl = $('now-octave');
  const needle = $('needle');
  const centsEl = $('now-cents');
  const zone = document.querySelector('.meter-zone');

  const tol = state && state.active_settings
    ? Number(state.active_settings.tolerance_cents) : null;

  // 허용 구간을 미터 위에 녹색 띠로 그린다
  if (tol && tol > 0) {
    zone.style.width = Math.min(100, (tol / METER_RANGE) * 100) + '%';
  } else {
    zone.style.width = '0';
  }

  const no = state ? state.current_string : 0;
  $('now-stringno').textContent = no ? `${no}번 줄` : '—';
  $('now-adjust').textContent = no ? `조절 ${state.adjust_count}회` : '—';

  if (!no) {
    noteEl.textContent = '—';
    noteEl.className = 'note';
    octEl.textContent = '';
    needle.className = 'needle';
    needle.style.transform = 'translateX(0)';
    centsEl.textContent = '—';
    centsEl.className = 'cents-big';
    $('now-measured').textContent = '—';
    $('now-target').textContent = '—';
    return;
  }

  // 음이름은 표에서 직접 꺼낸다. "E2" -> 음 'E' + 옥타브 '2'
  const info = FRETBOARD.find((x) => x.n === no);
  const note = info ? info.name.slice(0, -1) : '—';
  octEl.textContent = info ? info.name.slice(-1) : '';
  $('now-target').textContent = hz(state.target_hz);

  if (!state.measurement_valid) {
    noteEl.textContent = '측정 불가';
    noteEl.className = 'note invalid';
    octEl.textContent = '';
    needle.className = 'needle';
    needle.style.transform = 'translateX(0)';
    centsEl.textContent = '—';
    centsEl.className = 'cents-big';
    $('now-measured').textContent = '—';
    return;
  }

  noteEl.textContent = note;
  $('now-measured').textContent = hz(state.measured_hz);

  const c = centsOf(state.measured_hz, state.target_hz);
  centsEl.textContent = centsText(c);

  if (c === null) {
    noteEl.className = 'note';
    needle.className = 'needle';
    needle.style.transform = 'translateX(0)';
    centsEl.className = 'cents-big';
    return;
  }

  const inTune = tol ? Math.abs(c) <= tol : false;
  // 트랙 절반 폭만큼 좌우로 민다. ±50센트를 넘으면 끝에 붙는다.
  const clamped = Math.max(-METER_RANGE, Math.min(METER_RANGE, c));
  const half = needle.parentElement.clientWidth / 2;
  needle.style.transform = `translateX(${(clamped / METER_RANGE) * half}px)`;

  noteEl.className = 'note' + (inTune ? ' ok' : '');
  needle.className = 'needle ' + (inTune ? 'ok' : (Math.abs(c) > METER_RANGE ? 'off' : 'live'));
  centsEl.className = 'cents-big ' + (inTune ? 'ok' : (c > 0 ? 'high' : 'low'));
}

function renderLive(state) {
  $('job-id').textContent = state && state.job_id ? state.job_id : '작업 없음';

  const phaseEl = $('phase');
  phaseEl.textContent = state ? state.phase_label : '상태 없음';
  phaseEl.className = state ? phaseClass(state.phase) : 'badge badge-wait';

  const overallEl = $('overall');
  overallEl.textContent = state ? state.overall_label : '—';
  overallEl.className = state ? overallClass(state.overall_result) : 'badge badge-idle';

  renderGuitar(state);
  renderTuner(state);
  renderStringCards(state);
  renderButtons(state);
  renderSettingsLock(state);
}

function renderStringCards(state) {
  const wrap = $('strings');
  const byNumber = new Map();
  if (state) state.results.forEach((r) => byNumber.set(r.string_number, r));

  wrap.innerHTML = '';
  STRINGS.forEach(({ n, name }) => {
    const r = byNumber.get(n);
    const card = document.createElement('div');

    let cls = 'scard';
    if (state && state.current_string === n && isRunning(state)) cls += ' current';
    if (r && r.result === 1) cls += ' ok';
    if (r && r.result === 2) cls += ' bad';
    card.className = cls;

    // 지금 이 줄을 조율 중인가. 그렇다면 아직 확정되지 않은 값 대신
    // 흘러들어오는 측정값을 보여준다. tuned_hz 는 줄이 끝나야 채워지므로,
    // 그것만 쓰면 진행 중에는 카드가 '측정 불가'로 굳어 보인다.
    const live = state && state.current_string === n && isRunning(state);
    const note = live ? '진행 중' : (r ? r.result_label : '대기');

    const rows = [];
    rows.push(['목표', `${hz(r ? r.target_hz : null)} Hz`, false]);
    if (live) {
      rows.push(['측정', state.measurement_valid ? `${hz(state.measured_hz)} Hz` : '측정 불가',
                 !state.measurement_valid]);
      const c = state.measurement_valid
        ? centsOf(state.measured_hz, state.target_hz) : null;
      rows.push(['오차', c === null ? '—' : `${centsText(c)} 센트`, c === null]);
      rows.push(['조절 횟수', String(state.adjust_count), false]);
    } else if (r) {
      rows.push(['조율 측정', r.tuned_hz === null ? '측정 불가' : `${hz(r.tuned_hz)} Hz`,
                 r.tuned_hz === null]);
      rows.push(['검증 측정', r.verified_hz === null ? '—' : `${hz(r.verified_hz)} Hz`,
                 r.verified_hz === null]);
      rows.push(['최종 오차', `${cents(r.final_error_cents)} 센트`, false]);
      rows.push(['조절 횟수', String(r.adjust_count), false]);
      if (r.result === 2 && r.failure_label) rows.push(['실패 사유', r.failure_label, true]);
    }

    card.innerHTML = `
      <div class="scard-head">
        <span class="scard-name">${n}번 <span class="scard-note">${name}</span></span>
        <span class="${r && r.result === 1 ? 'badge badge-ok'
                       : r && r.result === 2 ? 'badge badge-bad' : 'badge badge-idle'}">${note}</span>
      </div>
      <div class="rows">
        ${rows.map(([k, v, dim]) =>
          `<div class="row"><span class="k">${k}</span><span class="v${dim ? ' dim' : ''}">${v}</span></div>`
        ).join('')}
      </div>`;
    wrap.appendChild(card);
  });
}

function renderButtons(state) {
  const running = isRunning(state);
  const paused = state && state.phase === PHASE.PAUSED;
  $('btn-start').disabled = running;
  $('btn-pause').disabled = !running || paused;
  $('btn-resume').disabled = !paused;
  $('btn-cancel').disabled = !running;
}

// ─────────────────────────────── 명령

async function sendCommand(name) {
  try {
    const res = await fetch(`/api/command/${name}`, { method: 'POST' });
    const data = await res.json();
    if (!res.ok) { toast(data.error || '명령 실패', false); return; }
    toast(data.message || (data.accepted ? '접수' : '거부'), data.accepted);
  } catch (e) {
    toast(`명령을 보내지 못했다: ${e}`, false);
  }
}

// ─────────────────────────────── 설정

function renderSettingsLock(state) {
  const locked = isRunning(state);
  const badge = $('settings-lock');
  badge.textContent = locked ? '작업 중 — 변경 잠김' : '변경 가능';
  badge.className = locked ? 'badge badge-warn' : 'badge badge-idle';
  SETTING_FIELDS.forEach((f) => { $(f).disabled = locked; });
  $('btn-settings-save').disabled = locked;
  $('btn-settings-default').disabled = locked;
}

function fillSettings(values) {
  SETTING_FIELDS.forEach((f) => { $(f).value = values[f]; });
}

async function loadSettings() {
  try {
    const res = await fetch('/api/settings');
    const data = await res.json();
    if (!res.ok) { toast(data.error || '설정을 읽지 못했다', false); return; }
    defaults = data.defaults;
    fillSettings(data.settings);
  } catch (e) {
    toast(`설정을 읽지 못했다: ${e}`, false);
  }
}

async function saveSettings() {
  const values = {};
  for (const f of SETTING_FIELDS) {
    const raw = $(f).value.trim();
    if (raw === '') { toast('빈 칸이 있다', false); return; }
    const n = Number(raw);
    if (!Number.isFinite(n)) { toast(`${f} 값이 숫자가 아니다`, false); return; }
    values[f] = n;
  }
  // task_manager._settings_violations() 와 같은 조건. 여기서 먼저 걸러 왕복을 줄인다.
  if (values.tolerance_cents <= 0) { toast('허용 센트는 0보다 커야 한다', false); return; }
  if (values.measure_timeout_sec <= 0) { toast('측정 제한 시간은 0보다 커야 한다', false); return; }
  if (values.onset_timeout_sec <= 0) { toast('onset 제한 시간은 0보다 커야 한다', false); return; }
  for (const f of ['max_adjust_count', 'max_measure_retry']) {
    if (!Number.isInteger(values[f]) || values[f] < 0 || values[f] > 255) {
      toast(`${f} 는 0~255 정수여야 한다`, false);
      return;
    }
  }

  try {
    const res = await fetch('/api/settings', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(values),
    });
    const data = await res.json();
    if (!res.ok) { toast(data.error || '설정 저장 실패', false); return; }
    toast(data.message || (data.accepted ? '저장' : '거부'), data.accepted);
    if (data.accepted) loadSettings();
  } catch (e) {
    toast(`설정을 저장하지 못했다: ${e}`, false);
  }
}

// ─────────────────────────────── 이력

async function loadHistory() {
  const list = $('history-list');
  list.innerHTML = '<p class="empty">읽는 중…</p>';
  try {
    const res = await fetch('/api/history');
    const data = await res.json();
    if (!res.ok) {
      list.innerHTML = `<p class="empty">${data.error || '이력을 읽지 못했다'}</p>`;
      return;
    }
    if (!data.found || data.jobs.length === 0) {
      // GetHistory 응답에는 사유 필드가 없다. DB 를 못 열어도 found=false 로만 오므로
      // '기록이 없음'과 '조회 실패'를 화면에서 구분할 수 없다. 확인 경로를 같이 적어 둔다.
      list.innerHTML = '<p class="empty">저장된 작업이 없다.<br>' +
        '<span class="hint">저장한 적이 있는데도 비어 보이면 result_store 로그를 확인한다 — ' +
        'get_history 응답에 사유 필드가 없어 DB 오류와 구분할 수 없다.</span></p>';
      return;
    }
    list.innerHTML = '';
    data.jobs.forEach((j) => {
      const b = document.createElement('button');
      b.className = 'hitem' + (selectedJob === j.job_id ? ' active' : '');
      b.innerHTML = `<span class="when">${whenText(j.finished_at)}</span>` +
                    `<span class="${overallClass(j.overall_result)}">${j.overall_label}</span>`;
      b.addEventListener('click', () => { selectedJob = j.job_id; loadHistory(); loadDetail(j); });
      list.appendChild(b);
    });
  } catch (e) {
    list.innerHTML = `<p class="empty">이력을 읽지 못했다: ${e}</p>`;
  }
}

async function loadDetail(job) {
  const box = $('history-detail');
  box.innerHTML = '<p class="empty">읽는 중…</p>';
  try {
    const res = await fetch(`/api/history/${encodeURIComponent(job.job_id)}`);
    const data = await res.json();
    if (!res.ok || !data.found) {
      box.innerHTML = `<p class="empty">${data.error || '상세 기록이 없다'}</p>`;
      return;
    }
    const s = data.settings;
    box.innerHTML = `
      <h3>${job.job_id}</h3>
      <p class="hint">${whenText(job.finished_at)} 종료 · ${job.overall_label}</p>
      <h4>줄별 기록</h4>
      <div class="dlist">
        ${data.strings.map((r) => `
          <div class="dline">
            <span>${stringName(r.string_number)}</span>
            <span class="v">${r.tuned_hz === null ? '측정 불가' : hz(r.tuned_hz) + ' Hz'}
              / 검증 ${r.verified_hz === null ? '—' : hz(r.verified_hz) + ' Hz'}
              / ${cents(r.final_error_cents)} 센트
              / 조절 ${r.adjust_count}회
              / ${r.result_label}${r.failure_label ? ' (' + r.failure_label + ')' : ''}</span>
          </div>`).join('')}
      </div>
      <h4>그때 적용된 설정</h4>
      <div class="dlist">
        <div class="dline"><span>허용 센트</span><span class="v">${s.tolerance_cents}</span></div>
        <div class="dline"><span>최대 조절</span><span class="v">${s.max_adjust_count}회</span></div>
        <div class="dline"><span>측정 재시도</span><span class="v">${s.max_measure_retry}회</span></div>
        <div class="dline"><span>측정 제한</span><span class="v">${s.measure_timeout_sec}초</span></div>
        <div class="dline"><span>onset 제한</span><span class="v">${s.onset_timeout_sec}초</span></div>
      </div>`;
  } catch (e) {
    box.innerHTML = `<p class="empty">상세를 읽지 못했다: ${e}</p>`;
  }
}

// ─────────────────────────────── SSE 연결

function connect() {
  const link = $('link');
  const es = new EventSource('/api/events');

  es.onopen = () => {
    link.textContent = '연결됨';
    link.className = 'badge badge-ok';
  };

  es.onmessage = (ev) => {
    lastState = JSON.parse(ev.data);
    renderLive(lastState);
  };

  es.onerror = () => {
    // EventSource 는 스스로 재접속한다. 그 사이 상태 표시만 바꾼다.
    link.textContent = '연결 끊김 — 재접속 중';
    link.className = 'badge badge-bad';
  };
}

// ─────────────────────────────── 시작

function initTabs() {
  document.querySelectorAll('.tab').forEach((tab) => {
    tab.addEventListener('click', () => {
      document.querySelectorAll('.tab').forEach((t) => t.classList.remove('active'));
      document.querySelectorAll('.panel').forEach((p) => p.classList.remove('active'));
      tab.classList.add('active');
      $(`tab-${tab.dataset.tab}`).classList.add('active');
      if (tab.dataset.tab === 'history') loadHistory();
      if (tab.dataset.tab === 'settings') loadSettings();
    });
  });
}

function init() {
  initTabs();
  $('btn-start').addEventListener('click', () => sendCommand('start'));
  $('btn-pause').addEventListener('click', () => sendCommand('pause'));
  $('btn-resume').addEventListener('click', () => sendCommand('resume'));
  $('btn-cancel').addEventListener('click', () => sendCommand('cancel'));
  $('btn-history-reload').addEventListener('click', loadHistory);
  $('btn-settings-reload').addEventListener('click', loadSettings);
  $('btn-settings-save').addEventListener('click', saveSettings);
  $('btn-settings-default').addEventListener('click', () => {
    if (!defaults) { toast('기본값을 아직 읽지 못했다', false); return; }
    fillSettings(defaults);
    toast('기본값을 채웠다. 저장을 눌러야 반영된다.', true);
  });

  renderLive(null);
  connect();
  loadSettings();
}

document.addEventListener('DOMContentLoaded', init);
