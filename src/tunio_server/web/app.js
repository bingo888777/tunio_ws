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
  if (phase === PHASE.REGISTER_PEG) return 'badge badge-gold';
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

// ─────────────────────────────── 기타 (전체 + 헤드 확대)

// guitar.png 의 좌표계(611×1506, 원본에서 투명 여백을 잘라낸 크기). 그림을 SVG 안의
// <image> 로 넣고 줄을 같은 좌표로 얹어서, viewBox 하나만 바꾸면 그림과 줄이 함께 확대된다.
const GUITAR_VB = [0, 0, 611, 1506];
// 헤드 확대 구간. 튜닝 손잡이 6개와 너트, 지판 윗부분까지 들어온다
const HEAD_VB = [165, 0, 280, 440];
// 전체 보기에서 커서가 이 안에 들어오면 헤드로 확대한다 (헤드 근처 여유 포함)
const HEAD_HOVER = { x0: 150, x1: 460, y1: 300 };
const ZOOM_MS = 380;

// 기타 위 줄을 그린다. 줄 하나가 <g class="gstring">, 상태는 class 로만 바꾼다.
function buildGuitar() {
  // 줄 하나는 꺾인 선이다 — 페그 기둥에서 너트로 비스듬히 들어와, 너트부터 새들까지
  // 지판을 따라 내려간다(새들 쪽 간격이 조금 넓다). 그림에 그려진 줄 위에 정확히 겹친다.
  //
  //   페그 기둥(x, y)
  //      \
  //       \  헤드 구간 (비스듬히)
  //        \
  //   너트 ●────────  여기서부터 지판
  //        │
  //  새들 ●
  //
  // 좌표는 guitar.png 의 픽셀에서 잡았다 (2026-09-27). 그림에 그려진 줄의 밝은 픽셀을
  // 구간마다(기둥→너트, 너트→새들) 따라가 직선을 맞췄다 — 잔차 0.5px 이내.
  // 처음엔 기둥 중심에서 7px 안쪽으로 어림잡았는데, 확대하면 2·1번이 3px 어긋나 보였다.
  // 3+3 헤드라 바깥 줄(6·1)이 너트에 가까운 아래 기둥, 안쪽 줄(4·3)이 위 기둥에 감긴다.
  const NUT_Y = 253;
  const NUT_X = { 6: 273.3, 5: 285.1, 4: 297.4, 3: 309.8, 2: 321.8, 1: 333.4 };
  const SADDLE_Y = 1224;
  const SADDLE_X = { 6: 259.0, 5: 278.1, 4: 296.9, 3: 316.6, 2: 333.9, 1: 352.6 };
  // 줄이 기둥을 떠나는 자리 (맞춘 직선이 기둥 중심 높이를 지나는 x)
  const POSTS = {
    4: [266.3, 86.6], 5: [266.6, 145.1], 6: [268.1, 201.3],
    3: [339.3, 86.5], 2: [340.0, 145.5], 1: [338.0, 201.1],
  };
  // 번호 원은 사람이 실제로 돌리는 바깥 튜닝 손잡이 위에 둔다. 손잡이는 지름 44 원이고
  // 헤드가 아래로 좁아지는 만큼 아래 손잡이일수록 안쪽으로 들어와 있다 — 바깥 가장자리에서
  // 반지름만큼 들어온 자리가 중심이다 (그림 위에 원을 겹쳐 그려 확인했다)
  const KNOBS = {
    4: [210, 92], 5: [213, 149], 6: [215.5, 206.5],
    3: [397, 92], 2: [394, 149], 1: [390.5, 206.5],
  };

  const parts = FRETBOARD.map((st) => {
    const [px, py] = POSTS[st.n];
    const [kx, ky] = KNOBS[st.n];
    const w = st.thick * 1.1;
    const pts = `${px},${py} ${NUT_X[st.n]},${NUT_Y} ${SADDLE_X[st.n]},${SADDLE_Y}`;
    // 음이름은 그림에 싣지 않는다. 새들 쪽 줄 간격이 19 라 어떻게 놓아도 겹친다.
    // 이름은 오른쪽 줄별 진행에 있고, 그림은 번호 원과 줄 색으로 읽는다.
    return `<g class="gstring" data-n="${st.n}">
      <polyline class="gs-hit" points="${pts}"></polyline>
      <polyline class="gs-line" points="${pts}" stroke-width="${w}"
                stroke-linecap="round" stroke-linejoin="round"></polyline>
      <circle class="gs-peg" cx="${kx}" cy="${ky}"></circle>
      <text class="gs-label" x="${kx}" y="${ky}" text-anchor="middle"
            dominant-baseline="central">${st.n}</text>
      <title>${st.n}번 ${st.name}</title>
    </g>`;
  }).join('');

  return `<svg class="guitar-overlay" viewBox="${GUITAR_VB.join(' ')}"
       preserveAspectRatio="xMidYMid meet" role="img" aria-label="기타와 줄 상태">
    <image href="guitar.png" x="0" y="0" width="${GUITAR_VB[2]}" height="${GUITAR_VB[3]}"></image>
    ${parts}
  </svg>`;
}

// 확대 상태. hover 는 커서로 잠깐, pinned 는 🔍·클릭·탭으로 고정
const zoom = { hover: false, pinned: false, anim: null, vb: GUITAR_VB.slice() };

function setViewBox(svg, vb) {
  zoom.vb = vb;
  svg.setAttribute('viewBox', vb.map((v) => v.toFixed(2)).join(' '));
}

function applyZoom() {
  const wrap = $('guitar');
  const svg = wrap.querySelector('.guitar-overlay');
  if (!svg) return;
  const on = zoom.hover || zoom.pinned;
  const to = on ? HEAD_VB : GUITAR_VB;
  wrap.classList.toggle('zoomed', on);
  const btn = $('btn-zoom');
  btn.setAttribute('aria-pressed', String(zoom.pinned));
  btn.title = zoom.pinned ? '확대 고정 풀기' : '헤드 확대 고정';
  $('guitar-cap').textContent = zoom.pinned
    ? '헤드 확대 고정됨 · 🔍 로 풀기'
    : on ? '헤드 확대 중 · 벗어나면 전체로'
      : '헤드에 커서를 올리면 확대 · 4·3번 줄은 조율하지 않음';

  if (zoom.anim) cancelAnimationFrame(zoom.anim);
  const from = zoom.vb.slice();
  if (from.every((v, i) => Math.abs(v - to[i]) < 0.01)) return;
  if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
    setViewBox(svg, to.slice());
    return;
  }
  const t0 = performance.now();
  const ease = (t) => 1 - Math.pow(1 - t, 3);
  const step = (now) => {
    const t = Math.min(1, (now - t0) / ZOOM_MS);
    const k = ease(t);
    setViewBox(svg, from.map((v, i) => v + (to[i] - v) * k));
    zoom.anim = t < 1 ? requestAnimationFrame(step) : null;
  };
  zoom.anim = requestAnimationFrame(step);
}

function initGuitarZoom() {
  const wrap = $('guitar');
  if (!wrap.firstChild) wrap.innerHTML = buildGuitar();
  const svg = wrap.querySelector('.guitar-overlay');
  const canHover = window.matchMedia('(hover: hover)').matches;

  // 커서 위치를 '전체 기타' 좌표로 바꿔 헤드 근처인지 본다. 확대 중에는 좌표계가
  // 바뀌므로 판정하지 않고, 무대 밖으로 나가야 풀린다.
  const nearHead = (ev) => {
    const r = svg.getBoundingClientRect();
    const scale = Math.min(r.width / GUITAR_VB[2], r.height / GUITAR_VB[3]);
    const ox = r.left + (r.width - GUITAR_VB[2] * scale) / 2;
    const oy = r.top + (r.height - GUITAR_VB[3] * scale) / 2;
    const x = (ev.clientX - ox) / scale, y = (ev.clientY - oy) / scale;
    return x >= HEAD_HOVER.x0 && x <= HEAD_HOVER.x1 && y <= HEAD_HOVER.y1;
  };

  if (canHover) {
    wrap.addEventListener('mousemove', (ev) => {
      if (wrap.classList.contains('coord-on') || zoom.hover || zoom.pinned) return;
      if (nearHead(ev)) { zoom.hover = true; applyZoom(); }
    });
    wrap.addEventListener('mouseleave', () => {
      if (!zoom.hover) return;
      zoom.hover = false;
      applyZoom();
    });
  }
  const toggle = () => {
    if (wrap.classList.contains('coord-on')) return;
    zoom.pinned = !zoom.pinned;
    if (!zoom.pinned && canHover) zoom.hover = false;
    applyZoom();
  };
  wrap.addEventListener('click', toggle);
  wrap.addEventListener('keydown', (ev) => {
    if (ev.key === 'Enter' || ev.key === ' ') { ev.preventDefault(); toggle(); }
  });
  $('btn-zoom').addEventListener('click', toggle);
  applyZoom();
}

// 끝난 줄 수. 색의 뜻은 카드 머리의 범례가 보여 준다.
function renderProgress(state) {
  const done = state ? state.results.filter((r) => r.result !== 0).length : 0;
  $('progress-count').textContent = `${done} / ${STRINGS.length}`;
}

// 지금 줄의 색. 조율 중 파랑 · 등록 금색 · 일시정지 호박색
function currentClass(state) {
  if (state.phase === PHASE.REGISTER_PEG) return ' current registering';
  if (state.phase === PHASE.PAUSED) return ' current paused';
  return ' current';
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
      cls += currentClass(state);
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

// ─────────────────────────────── 튜너 게이지

// 부채꼴 게이지. 중심 (160,160), ±50센트가 ±75도다. 반원(±90도)으로 두면 양끝 눈금이
// 바닥선에 붙어 숫자가 묻힌다. 바늘은 테두리 쪽 짧은 막대만 그린다 —
// 가운데를 비워 둬야 큰 음이름이 가려지지 않는다.
const G_CX = 160, G_CY = 160, G_SPAN = 75;

function polar(r, c) {
  const th = (c / METER_RANGE) * (G_SPAN * Math.PI / 180);
  return [G_CX + r * Math.sin(th), G_CY - r * Math.cos(th)];
}

function arcPath(r, c0, c1) {
  const [x0, y0] = polar(r, c0);
  const [x1, y1] = polar(r, c1);
  return `M ${x0.toFixed(2)} ${y0.toFixed(2)} A ${r} ${r} 0 0 1 ${x1.toFixed(2)} ${y1.toFixed(2)}`;
}

function buildGauge() {
  const parts = [];
  for (let c = -METER_RANGE; c <= METER_RANGE; c += 5) {
    const major = c % 25 === 0;
    const [x0, y0] = polar(major ? 110 : 118, c);
    const [x1, y1] = polar(132, c);
    const cls = c === 0 ? 'g-tick zero' : major ? 'g-tick major' : 'g-tick';
    parts.push(`<line class="${cls}" x1="${x0.toFixed(1)}" y1="${y0.toFixed(1)}"
                 x2="${x1.toFixed(1)}" y2="${y1.toFixed(1)}"></line>`);
    if (major && c !== 0) {          // 0 은 굵은 눈금으로 충분하다. 숫자는 음이름과 겹친다
      const [tx, ty] = polar(97, c);
      parts.push(`<text class="g-num" x="${tx.toFixed(1)}" y="${(ty + 4).toFixed(1)}">${
        c > 0 ? '+' + c : c}</text>`);
    }
  }
  $('gauge').innerHTML = `
    <path class="g-face" d="M 8 ${G_CY} A 152 152 0 0 1 312 ${G_CY} Z"></path>
    <path id="g-zone" class="g-zone" d=""></path>
    ${parts.join('')}
    <g class="g-sign flat">
      <circle cx="31" cy="147" r="12.5"></circle>
      <text x="31" y="147" dominant-baseline="central">♭</text>
      <text class="g-sign-cap" x="31" y="173">낮음</text>
    </g>
    <g class="g-sign sharp">
      <circle cx="289" cy="147" r="12.5"></circle>
      <text x="289" y="147" dominant-baseline="central">♯</text>
      <text class="g-sign-cap" x="289" y="173">높음</text>
    </g>
    <g id="g-needle" class="g-needle idle">
      <line x1="${G_CX}" y1="${G_CY - 100}" x2="${G_CX}" y2="${G_CY - 140}"></line>
      <circle class="g-hub" cx="${G_CX}" cy="${G_CY - 140}" r="6"></circle>
    </g>`;
}

function setNeedle(cls, c) {
  const needle = $('g-needle');
  needle.setAttribute('class', 'g-needle ' + cls);
  const v = c === null ? 0 : Math.max(-METER_RANGE, Math.min(METER_RANGE, c));
  needle.style.transform = `rotate(${(v / METER_RANGE) * G_SPAN}deg)`;
}

// 예상 페그 회전. 계산은 web_node 가 task_manager 와 같은 adjust_plan 으로 한다.
// 음이 높으면 백래시 때문에 '풀기 → 조임' 두 단계다 (CLAUDE.md 3.2).
function renderHint(state) {
  const el = $('now-hint');
  const hint = state && state.adjust_hint;
  if (!hint) { el.hidden = true; el.innerHTML = ''; return; }
  el.hidden = false;
  if (!hint.steps.length) {
    el.innerHTML = `<span class="k">예상 페그 회전</span><span class="dim">${hint.note || '계산 불가'}</span>`;
    return;
  }
  // 각도만 보여 준다. 회전 방향 기호(↻↺)는 쓰지 않는다 — 3+3 헤드는 왼쪽(6·5·4)과
  // 오른쪽(3·2·1) 페그의 조이는 방향이 반대라 화살표 하나로는 맞게 그릴 수 없다.
  const parts = hint.steps.map((st) => `
    <span class="step ${st.direction > 0 ? 'tight' : 'loose'}">${st.label}
      <b>${st.deg.toFixed(1)}°</b></span>`);
  el.innerHTML = `<span class="k">예상 페그 회전</span>${parts.join('<span class="arrow">→</span>')}`;
}

// 튜너 바늘과 큰 음이름.
function renderTuner(state) {
  renderHint(state);
  if (!$('g-needle')) buildGauge();
  const noteEl = $('now-note');
  const octEl = $('now-octave');
  const centsEl = $('now-cents');

  const tol = state && state.active_settings
    ? Number(state.active_settings.tolerance_cents) : null;

  // 허용 구간을 게이지 테두리에 초록 띠로 그린다
  const zc = tol && tol > 0 ? Math.min(tol, METER_RANGE) : 0;
  $('g-zone').setAttribute('d', zc ? arcPath(125, -zc, zc) : '');

  const no = state ? state.current_string : 0;
  $('now-stringno').textContent = no ? `${no}번 줄` : '대기 중';
  const maxAdj = state && state.active_settings ? state.active_settings.max_adjust_count : null;
  $('now-adjust').textContent = no
    ? `페그 조절 ${state.adjust_count}${maxAdj !== null ? ' / ' + maxAdj : ''}회` : '';

  // 값이 없을 때는 센트 줄을 통째로 숨긴다. '—' 를 굵은 고정폭으로 찍으면 막대처럼 보인다
  const centsWrap = centsEl.parentElement;
  const blank = (label) => {
    noteEl.textContent = label;
    noteEl.className = 'note invalid';
    octEl.textContent = '';
    setNeedle('idle', null);
    centsEl.textContent = '';
    centsEl.className = 'cents-big';
    centsWrap.hidden = true;
    $('now-measured').textContent = '—';
  };
  centsWrap.hidden = false;

  if (!no) {
    blank(state && state.phase === PHASE.DONE ? '조율 완료' : '대기');
    $('now-target').textContent = '—';
    return;
  }

  // 음이름은 표에서 직접 꺼낸다. "E2" -> 음 'E' + 옥타브 '2'
  const info = FRETBOARD.find((x) => x.n === no);
  const note = info ? info.name.slice(0, -1) : '—';
  $('now-target').textContent = hz(state.target_hz);

  if (!state.measurement_valid) {
    // 측정값이 없을 때는 '측정 불가' 대신 지금 하는 일을 보여 준다
    const busy = [PHASE.REGISTER_PEG, PHASE.PREPARE_PLUCK, PHASE.MEASURE_READY, PHASE.PLUCK,
      PHASE.ANALYZE, PHASE.ADJUST_PEG, PHASE.VERIFY, PHASE.RETURN, PHASE.PAUSED]
      .includes(state.phase);
    blank(busy ? `${state.phase_label}…` : '측정 불가');
    return;
  }

  noteEl.textContent = note;
  octEl.textContent = info ? info.name.slice(-1) : '';
  $('now-measured').textContent = hz(state.measured_hz);

  const c = centsOf(state.measured_hz, state.target_hz);
  centsEl.textContent = centsText(c);

  if (c === null) {
    noteEl.className = 'note';
    setNeedle('idle', null);
    centsEl.className = 'cents-big';
    return;
  }

  const inTune = tol ? Math.abs(c) <= tol : false;
  noteEl.className = 'note' + (inTune ? ' ok' : '');
  setNeedle(inTune ? 'ok' : (Math.abs(c) > METER_RANGE ? 'off' : ''), c);
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

// 지금 줄에서 보여 줄 단계. task_manager._measure_once 와 _adjust_peg 의 순서다
const STEPS = [
  [PHASE.PREPARE_PLUCK, '발현 준비'],
  [PHASE.MEASURE_READY, '측정 준비'],
  [PHASE.PLUCK, '발현'],
  [PHASE.ANALYZE, '분석'],
  [PHASE.ADJUST_PEG, '페그 조절'],
];

// 오차 미니 막대. 가운데가 0, 초록 띠가 허용 구간이다.
function centsBar(c, tol) {
  const has = c !== null && c !== undefined && isFinite(c);
  const clamped = has ? Math.max(-METER_RANGE, Math.min(METER_RANGE, c)) : 0;
  const zone = tol ? Math.min(100, (tol / METER_RANGE) * 100) : 0;
  const inTune = has && tol ? Math.abs(c) <= tol : false;
  return `<div class="cbar">
    <div class="cbar-track">
      <div class="cbar-zone" style="width:${zone}%"></div>
      <div class="cbar-mid"></div>
      ${has ? `<div class="cbar-dot ${inTune ? 'ok' : 'off'}"
               style="left:${50 + (clamped / METER_RANGE) * 50}%"></div>` : ''}
    </div>
    <span class="cbar-val${has ? '' : ' dim'}">${has ? centsText(c) + ' 센트' : '오차 —'}</span>
  </div>`;
}

// 줄별 진행. 줄마다 세로 블록 하나. 색은 줄이 아니라 상태를 뜻한다 (대기·진행·성공·실패).
// 지금 줄은 단계를 세로로 펼친다.
function renderStringCards(state) {
  const wrap = $('strings');
  const byNumber = new Map();
  if (state) state.results.forEach((r) => byNumber.set(r.string_number, r));
  const running = isRunning(state);
  const tol = state && state.active_settings ? Number(state.active_settings.tolerance_cents) : null;
  const maxAdj = state && state.active_settings ? state.active_settings.max_adjust_count : null;

  wrap.innerHTML = STRINGS.map(({ n, name }) => {
    const r = byNumber.get(n);
    // 지금 이 줄을 조율 중이면 확정값 대신 흘러드는 측정값을 보여 준다.
    // tuned_hz 는 줄이 끝나야 채워져서, 그것만 쓰면 진행 중엔 '측정 불가'로 굳어 보인다.
    const live = state && state.current_string === n && running;
    const registering = live && state.phase === PHASE.REGISTER_PEG;

    let cls = 'sblock';
    if (live) cls += currentClass(state);
    else if (r && r.result === 1) cls += ' ok';
    else if (r && r.result === 2) cls += ' bad';
    else cls += ' wait';
    if (isJustDone(n)) cls += ' just-done';

    let badge = ['badge-idle', '대기'];
    if (registering) badge = ['badge-gold', '등록 중'];
    else if (live && state.phase === PHASE.PAUSED) badge = ['badge-warn', '일시정지'];
    else if (live && state.phase === PHASE.VERIFY) badge = ['badge-run', '최종 검증'];
    else if (live) badge = ['badge-run', '진행 중'];
    else if (r && r.result === 1) badge = ['badge-ok', r.result_label];
    else if (r && r.result === 2) badge = ['badge-bad', r.result_label];

    // 부제 — 진행 전엔 목표, 진행 중엔 측정, 끝났으면 마지막 측정(검증이 있으면 검증)
    let sub = `목표 ${hz(r ? r.target_hz : null)} Hz`;
    if (live && state.measurement_valid) sub = `측정 ${hz(state.measured_hz)} Hz`;
    else if (!live && r && r.result !== 0) {
      const v = r.verified_hz !== null ? r.verified_hz : r.tuned_hz;
      sub = v === null ? '측정 불가' : `${r.verified_hz !== null ? '검증' : '측정'} ${hz(v)} Hz`;
    }

    let c = null;
    if (live && state.measurement_valid) c = centsOf(state.measured_hz, state.target_hz);
    else if (!live && r && r.result !== 0 && (r.tuned_hz !== null || r.verified_hz !== null)) {
      c = r.final_error_cents;
    }

    const adj = live ? state.adjust_count : (r ? r.adjust_count : 0);

    let steps = '';
    if (registering) {
      steps = '<div class="ssteps"><div class="sstep active"><i></i><span>페그 자세 등록</span></div></div>';
    } else if (live) {
      const at = STEPS.findIndex(([p]) => p === state.phase);
      steps = `<div class="ssteps">${STEPS.map(([, label], i) => {
        const k = at < 0 ? '' : i < at ? ' done' : i === at ? ' active' : '';
        return `<div class="sstep${k}"><i></i><span>${label}</span></div>`;
      }).join('')}</div>`;
    }
    const fail = !live && r && r.result === 2 && r.failure_label
      ? `<div class="sfail">사유 · ${r.failure_label}</div>` : '';

    return `<div class="${cls}">
      <div class="sb-top">
        <span class="sno">${n}</span>
        <span class="badge sbadge ${badge[0]}">${badge[1]}</span>
      </div>
      <div class="sname"><b>${name}</b><span>${sub}</span></div>
      ${centsBar(c, tol)}
      <div class="sadj"><span>페그 조절</span><b>${adj}${live && maxAdj !== null ? ' / ' + maxAdj : ''}회</b></div>
      ${steps}${fail}
    </div>`;
  }).join('');
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
    if (name === 'start' && (!res.ok || !data.accepted)) {
      window.alert(data.error || data.message || '시작 요청이 거부되었습니다.');
    } else {
      if (name === 'start' && data.accepted) {
        registration.message = '이번 조율에 등록값을 사용합니다. 다음 조율 전에는 다시 등록하세요.';
      }
      toast(data.error || data.message || (data.accepted ? '접수' : '거부'), res.ok && data.accepted);
    }
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
  initGuitarZoom();
  buildGauge();
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
