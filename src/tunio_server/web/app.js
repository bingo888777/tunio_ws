'use strict';

// TUNIO 관제 웹 — 화면 로직.
// 상태는 SSE(/api/events)로 받고, 명령·설정·이력은 fetch 로 주고받는다.
// 열거값의 뜻은 서버(web_node.py)가 라벨까지 같이 내려주므로 여기서 다시 해석하지 않는다.
// 다만 버튼 활성 판단에는 단계 번호가 필요해 상수를 둔다 — tunio_interfaces 의 TuningState 와 같다.

const PHASE = {
  IDLE: 0, MEASURE_READY: 1, PLUCK: 2, ANALYZE: 3, ADJUST_PEG: 4,
  VERIFY: 5, STRUM: 6, RETURN: 7, DONE: 8, PAUSED: 9, CANCELED: 10, ERROR: 11,
  REGISTER_PEG: 12, PREPARE_PLUCK: 13,
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
let streamReady = false;
let commandPending = false;
const registration = new RegistrationTracker();
let defaults = null;
let selectedJob = null;
let historyJobs = [];        // 서버에서 받은 이력 원본. 검색은 이 위에서 한다

// 줄이 성공·실패로 확정되는 순간을 잡아 잠깐 강조한다.
// 서버는 판정이 끝나면 바로 다음 줄로 넘어가므로, 그냥 두면 결과가 화면에
// 나타나기도 전에 지나간다. 서버에 대기를 넣으면 실제 로봇 동작 시간이 늘어나므로
// 화면에서만 붙잡는다.
const DONE_HOLD_MS = 2600;
const prevResult = new Map();   // 줄 번호 -> 직전 result
const justDone = new Map();     // 줄 번호 -> 강조가 끝나는 시각
let holdTimer = null;

// results 를 훑어 '방금 확정된' 줄을 표시하고, 강조가 끝날 때 다시 그리도록 예약한다.
function markJustDone(state) {
  const now = Date.now();
  if (state && state.results) {
    state.results.forEach((r) => {
      const before = prevResult.get(r.string_number);
      // RESULT_PENDING(0) 에서 성공(1)·실패(2) 로 넘어간 순간만 잡는다
      if (before === 0 && r.result !== 0) justDone.set(r.string_number, now + DONE_HOLD_MS);
      prevResult.set(r.string_number, r.result);
    });
  }
  // 만료된 것 정리
  let next = 0;
  justDone.forEach((until, n) => {
    if (until <= now) justDone.delete(n);
    else next = next === 0 ? until : Math.min(next, until);
  });
  // 남은 강조가 끝나는 시점에 한 번 더 그린다. 그때까지 새 상태가 안 올 수 있다.
  if (holdTimer) { clearTimeout(holdTimer); holdTimer = null; }
  if (next > 0) holdTimer = setTimeout(() => renderLive(lastState), next - now + 30);
}

function isJustDone(n) {
  const until = justDone.get(n);
  return until !== undefined && until > Date.now();
}

// 검색용으로 넉넉히 받는다. GetHistory.limit 은 uint16 이다.
const HISTORY_FETCH_LIMIT = 200;
const THEME_KEY = 'tunio-theme';

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

// job_id 는 task_manager 가 작업을 '시작'할 때 만든 시각 문자열이다 ("20260919_125643").
// finished_at 은 '끝난' 시각이라 둘이 몇 초에서 몇 분까지 벌어진다. 목록과 검색은
// 시작 시각(job_id)으로 통일하고, 종료 시각은 상세에서만 보여준다.
function startedText(jobId) {
  const m = /^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$/.exec(jobId || '');
  if (!m) return jobId || '—';
  return `${m[1]}-${m[2]}-${m[3]} ${m[4]}:${m[5]}:${m[6]}`;
}

// job_id 를 epoch 초로. 소요 시간 계산에 쓴다. 로컬 시간으로 해석한다
// (task_manager 가 로컬 시각으로 만든 문자열이다).
function startedEpoch(jobId) {
  const m = /^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$/.exec(jobId || '');
  if (!m) return null;
  const d = new Date(+m[1], +m[2] - 1, +m[3], +m[4], +m[5], +m[6]);
  return isNaN(d.getTime()) ? null : d.getTime() / 1000;
}

function durationText(jobId, finishedEpoch) {
  const a = startedEpoch(jobId);
  if (a === null || !finishedEpoch) return null;
  const sec = Math.max(0, Math.round(finishedEpoch - a));
  if (sec < 60) return `${sec}초`;
  return `${Math.floor(sec / 60)}분 ${String(sec % 60).padStart(2, '0')}초`;
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
      || state.phase === PHASE.PAUSED || state.phase === PHASE.REGISTER_PEG
      || state.phase === PHASE.PREPARE_PLUCK;
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
  // 기타 그림은 CSS 배경(guitar2.svg)으로 깔고, 줄 6개만 SVG 로 얹는다.
  //
  // 줄 하나는 꺾인 선이다 — 페그에서 너트로 비스듬히 들어와, 너트부터
  // 브릿지까지 일자로 내려간다. 실제 기타가 그렇게 감긴다.
  //
  //   페그(x, y)
  //      \
  //       \  헤드 구간 (비스듬히)
  //        \
  //   너트 ●────────  여기서부터 수직
  //        │
  //        │  지판 구간
  //        │
  //  브릿지 ●
  //
  // 좌표는 2026-09-20 화면에서 직접 찍어 맞춘 값이다 (?coord=1 모드).
  // 페그는 헤드 좌우로 갈려 균등 간격이 아니라 6개를 따로 둔다.
  const VB_W = 584.35, VB_H = 1513.5;
  const X0 = 259.35, X_STEP = 13.13;     // 너트~브릿지 구간의 x
  const NUT_Y = 259.9;
  const BRIDGE_Y = 320;      // 헤드 화면 아래 끝. 지판으로 이어지는 느낌만 준다

  // 줄 순서(6→1)대로. 6·5·4 는 헤드 왼쪽, 3·2·1 은 오른쪽에 감긴다.
  const PEGS = {
    6: [260.4, 186.7], 5: [259.4, 141.5], 4: [256.1, 93.1],
    3: [325.0, 93.1], 2: [322.8, 140.4], 1: [320.7, 186.7],
  };

  // 줄 번호는 줄이 감기는 축 위가 아니라 그 바깥에 놓는다. 축 위에 두면
  // 줄·원·숫자가 한 자리에 겹쳐 읽기 어렵다. 헤드 중심에서 멀어지는 쪽으로 민다.
  const HEAD_CX = (PEGS[6][0] + PEGS[1][0]) / 2;   // 헤드 좌우 중심
  const LABEL_OUT = 26;                             // 바깥으로 미는 거리

  const parts = FRETBOARD.map((st, i) => {
    const x = X0 + X_STEP * i;           // 지판 위 x
    const [px, py] = PEGS[st.n];         // 줄이 감기는 축
    const lx = px + (px < HEAD_CX ? -LABEL_OUT : LABEL_OUT);   // 번호는 바깥으로
    const w = st.thick * 1.1;

    return `<g class="gstring" data-n="${st.n}">
      <polyline class="gs-hit" points="${px},${py} ${x},${NUT_Y} ${x},${BRIDGE_Y}"></polyline>
      <polyline class="gs-line" points="${px},${py} ${x},${NUT_Y} ${x},${BRIDGE_Y}"
                stroke-width="${w}" stroke-linecap="round" stroke-linejoin="round"></polyline>
      <circle class="gs-peg" cx="${lx}" cy="${py}" r="11" stroke-width="2"></circle>
      <text class="gs-label" x="${lx}" y="${py + 4.5}" text-anchor="middle">${st.n}</text>
      <text class="gs-note" x="${x}" y="${BRIDGE_Y + (i % 2 ? 48 : 26)}"
            text-anchor="middle">${st.name}</text>
      <line class="gs-tick" x1="${x}" y1="${BRIDGE_Y + 4}" x2="${x}"
            y2="${BRIDGE_Y + (i % 2 ? 36 : 14)}"></line>
    </g>`;
  }).join('');

  // 헤드 구간만 보여 준다. 바디까지 넣으면 세로로 너무 길어져 화면이 빈다.
  // guitar_head.svg 의 viewBox 와 같은 값이라 그림과 줄이 정확히 겹친다.
  return `<svg class="guitar-overlay" viewBox="200 0 185 320"
         role="img" aria-label="기타 헤드와 줄 상태">${parts}</svg>`;
}

// 대상 4줄(6·5·2·1)의 진행을 점으로 보여 준다. 기타 그림만으로는
// '몇 번째를 하고 있나' 가 안 읽힌다.
function renderProgress(state) {
  const wrap = $('progress-dots');
  const byNumber = new Map();
  if (state) state.results.forEach((r) => byNumber.set(r.string_number, r));
  const running = isRunning(state);

  if (wrap.children.length !== STRINGS.length) {
    wrap.innerHTML = STRINGS.map(() => '<i class="pdot"></i>').join('');
  }

  let done = 0;
  STRINGS.forEach(({ n, name }, i) => {
    const dot = wrap.children[i];
    const r = byNumber.get(n);
    let cls = 'pdot';
    if (state && state.current_string === n && running) cls += ' current';
    else if (r && r.result === 1) cls += ' ok';
    else if (r && r.result === 2) cls += ' bad';
    dot.className = cls;
    dot.title = `${n}번 ${name}`;
    if (r && r.result !== 0) done += 1;
  });

  $('progress-count').textContent = `${done} / ${STRINGS.length}`;
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
    if (isJustDone(n)) cls += ' just-done';
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
    noteEl.textContent = [PHASE.REGISTER_PEG, PHASE.PREPARE_PLUCK, PHASE.RETURN].includes(state.phase) ? state.phase_label : '측정 불가';
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

let lastJobErrorKey = null;

function renderJobError(state) {
  const message = state && state.phase === PHASE.ERROR ? state.error_message : '';
  if (!state) return; // 연결 대기만으로 같은 오류의 팝업을 반복하지 않는다.
  if (state.phase !== PHASE.ERROR) {
    lastJobErrorKey = null;
    return;
  }
  if (!message) return;
  const key = JSON.stringify([state.job_id, message]);
  if (key === lastJobErrorKey) return;
  lastJobErrorKey = key;
  window.alert(`작업을 진행할 수 없습니다.\n\n${message}`);
}

function renderLive(state) {
  renderJobError(state);
  markJustDone(state);
  $('job-id').textContent = state && state.job_id ? state.job_id : '작업 없음';

  const phaseEl = $('phase');
  phaseEl.textContent = state ? state.phase_label : '상태 없음';
  phaseEl.className = state ? phaseClass(state.phase) : 'badge badge-wait';

  const overallEl = $('overall');
  overallEl.textContent = state ? state.overall_label : '—';
  overallEl.className = state ? overallClass(state.overall_result) : 'badge badge-idle';

  renderProgress(state);
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
    if (isJustDone(n)) cls += ' just-done';
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
  const blocked = !streamReady || !state || commandPending;
  $('btn-start').disabled = blocked || running || registration.busy;
  $('btn-reregister').disabled = blocked || running || registration.busy;
  $('registration-status').textContent = registration.message;
  $('btn-pause').disabled = blocked || !running || paused;
  $('btn-resume').disabled = blocked || !paused;
  $('btn-cancel').disabled = blocked || !running;
}

// ─────────────────────────────── 명령

async function sendCommand(name) {
  if (commandPending || !streamReady || !lastState) return;
  const registering = name === 'reregister_pegs';
  if ((registering || name === 'start') && (isRunning(lastState) || registration.busy)) return;
  commandPending = true;
  if (registering) registration.begin(lastState);
  renderButtons(lastState);
  try {
    const res = await fetch(`/api/command/${name}`, { method: 'POST' });
    const data = await res.json();
    if (registering) {
      if (res.ok) registration.reply(data.accepted, data.message);
      else registration.unknown(data.error || '접수 여부 확인 불가');
    }
    toast(data.error || data.message || (data.accepted ? '접수' : '거부'), res.ok && data.accepted);
  } catch (e) {
    if (registering) registration.unknown('응답 유실 — 접수 여부를 확인할 수 없습니다. 상태와 로그를 확인하세요.');
    toast(`명령을 보내지 못했다: ${e}`, false);
  } finally {
    commandPending = false;
    renderButtons(lastState);
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
    const res = await fetch(`/api/history?limit=${HISTORY_FETCH_LIMIT}`);
    const data = await res.json();
    if (!res.ok) {
      historyJobs = [];
      list.innerHTML = `<p class="empty">${data.error || '이력을 읽지 못했다'}</p>`;
      $('history-hint').textContent = '';
      return;
    }
    historyJobs = (data.found && data.jobs) ? data.jobs : [];
    renderHistoryList();
  } catch (e) {
    historyJobs = [];
    list.innerHTML = `<p class="empty">이력을 읽지 못했다: ${e}</p>`;
    $('history-hint').textContent = '';
  }
}

// 검색·필터는 서버가 아니라 받아 온 목록 위에서 한다.
// GetHistory 에 검색 조건 필드가 없어서인데, 최근 몇백 건이면 화면에서 거르는 편이 빠르다.
// 검색은 시작 시각 기준으로만 한다. 원본 job_id("20260919_125643")와
// 보기 좋게 푼 형태("2026-09-19 12:56:43") 둘 다 맞춘다 — 화면에 보이는 대로 쳐도
// 걸리고, 작업 번호를 그대로 붙여 넣어도 걸린다.
function matchesQuery(job, q) {
  if (!q) return true;
  const id = (job.job_id || '').toLowerCase();
  return id.includes(q) || startedText(job.job_id).toLowerCase().includes(q);
}

function renderHistoryList() {
  const list = $('history-list');
  const hint = $('history-hint');
  const q = $('history-q').value.trim().toLowerCase();
  const f = $('history-filter').value;

  if (historyJobs.length === 0) {
    // GetHistory 응답에는 사유 필드가 없다. DB 를 못 열어도 found=false 로만 오므로
    // '기록이 없음'과 '조회 실패'를 화면에서 구분할 수 없다. 확인 경로를 같이 적어 둔다.
    list.innerHTML = '<p class="empty">저장된 작업이 없다.<br>' +
      '<span class="hint">저장한 적이 있는데도 비어 보이면 result_store 로그를 확인한다 — ' +
      'get_history 응답에 사유 필드가 없어 DB 오류와 구분할 수 없다.</span></p>';
    hint.textContent = '';
    return;
  }

  const rows = historyJobs
    .filter((j) => (f === '' || String(j.overall_result) === f))
    .filter((j) => matchesQuery(j, q));

  hint.textContent = (q || f)
    ? `${historyJobs.length}건 중 ${rows.length}건`
    : `${historyJobs.length}건`;

  if (rows.length === 0) {
    list.innerHTML = '<p class="empty">검색 조건에 맞는 작업이 없다.</p>';
    return;
  }

  list.innerHTML = '';
  rows.forEach((j) => {
    const b = document.createElement('button');
    b.className = 'hitem' + (selectedJob === j.job_id ? ' active' : '');
    b.innerHTML = `<span class="when">${startedText(j.job_id)}</span>` +
                  `<span class="${overallClass(j.overall_result)}">${j.overall_label}</span>`;
    b.addEventListener('click', () => {
      selectedJob = j.job_id;
      renderHistoryList();     // 선택 표시만 갱신. 다시 읽지 않는다
      loadDetail(j);
    });
    list.appendChild(b);
  });
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
      <h3>${startedText(job.job_id)}</h3>
      <p class="hint">
        시작 ${startedText(job.job_id)} · 종료 ${whenText(job.finished_at)}${
          durationText(job.job_id, job.finished_at)
            ? ' · 소요 ' + durationText(job.job_id, job.finished_at) : ''
        }<br>작업 번호 <code>${job.job_id}</code> · ${job.overall_label}
      </p>
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
    streamReady = true;
    registration.observe(lastState);
    renderLive(lastState);
  };

  es.onerror = () => {
    streamReady = false;
    registration.disconnect();
    renderButtons(lastState);
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

function applyTheme(t) {
  document.documentElement.setAttribute('data-theme', t);
  const btn = $('btn-theme');
  if (btn) {
    btn.textContent = (t === 'dark') ? '☀' : '☾';
    btn.title = (t === 'dark') ? '밝은 화면으로' : '어두운 화면으로';
  }
  // 사생활 보호 모드나 저장소 차단 환경에서는 실패할 수 있다. 화면은 그대로 동작한다.
  try { localStorage.setItem(THEME_KEY, t); } catch (e) { /* 무시 */ }
}

function initTheme() {
  let t = 'light';
  try { t = localStorage.getItem(THEME_KEY) || 'light'; } catch (e) { /* 무시 */ }
  applyTheme(t);
  $('btn-theme').addEventListener('click', () => {
    const now = document.documentElement.getAttribute('data-theme');
    applyTheme(now === 'dark' ? 'light' : 'dark');
  });
}

// ?coord=1 로 열면 기타 그림 위에서 좌표를 찍어 읽을 수 있다.
// 일러스트를 바꿨을 때 줄 위치를 다시 잡으려고 둔 것이다. 평소에는 꺼져 있다.
//
// 찍는 순서: 페그 6개(6→1) → 너트 → 브릿지.
// 페그는 헤드 좌우로 갈려 있어 균등 간격이 아니므로 6개를 다 찍는다.
// 너트와 브릿지는 6줄 공통이라 y 만 쓴다(x 는 클릭해도 무시).
const COORD_STEPS = [
  { key: 'peg6', label: '6번 페그 (제일 굵은 줄이 감긴 곳)' },
  { key: 'peg5', label: '5번 페그' },
  { key: 'peg4', label: '4번 페그' },
  { key: 'peg3', label: '3번 페그' },
  { key: 'peg2', label: '2번 페그' },
  { key: 'peg1', label: '1번 페그 (제일 가는 줄)' },
  { key: 'nut', label: '너트 — 줄이 지판으로 들어가기 시작하는 가로선' },
  { key: 'bridge', label: '브릿지 — 줄이 끝나는 곳' },
];

function initCoordMode() {
  if (new URLSearchParams(location.search).get('coord') !== '1') return;

  const wrap = $('guitar');
  wrap.classList.add('coord-on');

  const box = document.createElement('div');
  box.className = 'coordbox';
  document.body.appendChild(box);

  const got = [];

  // 화면 픽셀 -> SVG viewBox 좌표
  const toSvg = (ev) => {
    const svg = wrap.querySelector('.guitar-overlay');
    const pt = svg.createSVGPoint();
    pt.x = ev.clientX;
    pt.y = ev.clientY;
    return pt.matrixTransform(svg.getScreenCTM().inverse());
  };

  const draw = () => {
    const step = COORD_STEPS[got.length];
    if (step) {
      const done = got.map((g, i) =>
        `<div class="done">${COORD_STEPS[i].key} = ${g.x.toFixed(1)}, ${g.y.toFixed(1)}</div>`
      ).join('');
      box.innerHTML =
        `<b>좌표 읽기 ${got.length + 1} / ${COORD_STEPS.length}</b>` +
        `<div class="ask">${step.label}<br>를 클릭하세요</div>` +
        done +
        `<hr><span class="now"></span>` +
        (got.length ? '<br><button class="undo">← 하나 취소</button>' : '');
    } else {
      const g = Object.fromEntries(got.map((v, i) => [COORD_STEPS[i].key, v]));
      const pegs = [6, 5, 4, 3, 2, 1]
        .map((n) => `  ${n}번: [${g['peg' + n].x.toFixed(1)}, ${g['peg' + n].y.toFixed(1)}]`)
        .join('<br>');
      box.innerHTML =
        `<b>다 읽었습니다</b><hr>` +
        `<div class="out">페그<br>${pegs}<br><br>` +
        `너트 Y = ${g.nut.y.toFixed(1)}<br>` +
        `브릿지 Y = ${g.bridge.y.toFixed(1)}</div>` +
        `<hr><span class="hint">위 숫자를 그대로 알려주세요.<br>` +
        `다시 하려면 새로고침</span>`;
      wrap.classList.remove('coord-on');
    }
    const now = box.querySelector('.now');
    if (now) now.textContent = box.dataset.now || '';
    const undo = box.querySelector('.undo');
    if (undo) undo.addEventListener('click', (e) => { e.stopPropagation(); got.pop(); draw(); });
  };

  wrap.addEventListener('mousemove', (ev) => {
    const p = toSvg(ev);
    box.dataset.now = `X ${p.x.toFixed(1)}   Y ${p.y.toFixed(1)}`;
    const now = box.querySelector('.now');
    if (now) now.textContent = box.dataset.now;
  });

  wrap.addEventListener('click', (ev) => {
    if (got.length >= COORD_STEPS.length) return;
    got.push(toSvg(ev));
    draw();
  });

  draw();
}

function init() {
  initCoordMode();
  initTheme();
  initTabs();
  $('btn-reregister').addEventListener('click', () => sendCommand('reregister_pegs'));
  $('btn-start').addEventListener('click', () => sendCommand('start'));
  $('btn-pause').addEventListener('click', () => sendCommand('pause'));
  $('btn-resume').addEventListener('click', () => sendCommand('resume'));
  $('btn-cancel').addEventListener('click', () => sendCommand('cancel'));
  $('btn-history-reload').addEventListener('click', loadHistory);
  $('history-q').addEventListener('input', renderHistoryList);
  $('history-filter').addEventListener('change', renderHistoryList);
  $('btn-history-clear').addEventListener('click', () => {
    $('history-q').value = '';
    $('history-filter').value = '';
    renderHistoryList();
  });
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
