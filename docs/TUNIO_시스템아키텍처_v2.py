#!/usr/bin/env python3
"""TUNIO 시스템 아키텍처 v2 — 노드 그래프 SVG 생성기.

왜 손으로 SVG 를 그리는가
  graphviz 는 노드 안의 포트로 선을 끌면 표를 가로지르고, 직교 라우팅을
  쓰면 라벨을 못 그린다. ROS 아키텍처는 '어느 노드의 어느 인터페이스가
  무엇과 이어지고 무슨 데이터가 흐르는가'가 핵심이라 포트와 배선을
  직접 통제해야 한다.

고치는 법
  NODES / WIRES 만 고치면 된다. 좌표는 위에서 아래로 쌓아 계산한다.

    python3 docs/TUNIO_시스템아키텍처_v2.py docs/TUNIO_시스템아키텍처_v2.svg

  ⚠️ **PNG 는 이 스크립트가 만들지 않는다.** SVG 를 고쳤으면 PNG 도 손으로
     다시 뽑아야 한다. 안 하면 PNG 만 옛날 그림으로 남는다.

    rsvg-convert -w 3280 docs/TUNIO_시스템아키텍처_v2.svg \
        -o docs/TUNIO_시스템아키텍처_v2.png
"""
import io

W, H = 1640, 1190
PAD = 26

C = {
    'bg': '#FAFBF9', 'grid': '#EDEFEC',
    'ink': '#121815', 'ink2': '#3E4A44', 'muted': '#76817B',
    'card': '#FFFFFF', 'cardline': '#D2D9D4',
    'robot': '#B3761F', 'robot_bg': '#FDF6EC', 'robot_soft': '#F4DFBE',
    'server': '#20669F', 'server_bg': '#EFF5FC', 'server_soft': '#CFE2F4',
    'mic': '#17755A', 'mic_bg': '#EEF8F3', 'mic_soft': '#C9E9DA',
    'ext': '#78838C', 'ext_bg': '#F2F4F5', 'ext_soft': '#E7EAEC',
    'act': '#1F7A9C', 'srv': '#8A5AA8', 'top': '#0F7A56', 'raw': '#8A6A33',
}
KIND = {'act': ('Action', C['act']), 'srv': ('Service', C['srv']),
        'top': ('Topic', C['top']), 'raw': ('비ROS', C['raw'])}

PORT_H, HEAD_H, CARD_PAD = 24, 50, 8


def tw(t, px):
    """한글은 라틴 문자보다 넓다. 대략적인 문자열 폭(px)."""
    k = sum(1 for c in t if ord(c) > 0x1100)
    return (len(t) - k) * px * 0.55 + k * px * 1.0


class Node:
    def __init__(self, key, title, sub, x, y, w, ins, outs, theme):
        self.key, self.title, self.sub = key, title, sub
        self.x, self.y, self.w = x, y, w
        self.ins, self.outs = ins, outs
        self.theme = theme
        self.h = HEAD_H + CARD_PAD * 2 + max(len(ins), len(outs)) * PORT_H + 12

    def port_y(self, side, i):
        return self.y + HEAD_H + CARD_PAD + 11 + i * PORT_H

    def pin(self, name):
        for i, p in enumerate(self.ins):
            if p[0] == name:
                return (self.x, self.port_y('in', i))
        raise KeyError(f'{self.key} in {name}')

    def pout(self, name):
        for i, p in enumerate(self.outs):
            if p[0] == name:
                return (self.x + self.w, self.port_y('out', i))
        raise KeyError(f'{self.key} out {name}')


# ── 노드 정의 ────────────────────────────────────────────────
# (포트키, 표시명, 종류, 타입명)
NODES = {}


def N(*a, **k):
    n = Node(*a, **k)
    NODES[n.key] = n
    return n


N('brow', '브라우저', '서버 PC localhost', 40, 470, 195,
  ins=[('http', 'HTTP · SSE', 'raw', '화면 갱신')],
  outs=[('post', 'POST', 'raw', '버튼 명령')], theme='ext')

N('web', 'web_node', '관제 웹 · 표준 http.server 를 별도 스레드에서', 300, 400, 340,
  ins=[('state', '/tuning/state', 'top', 'TuningState 구독'),
       ('post', '브라우저 POST', 'raw', '')],
  outs=[('cmd', '/tuning/start·pause·resume·cancel', 'srv', 'CommandTuning'),
        ('set', '/tuning/get_settings · set_settings', 'srv', 'Get/SetSettings'),
        ('hist', '/tuning/get_history', 'srv', 'GetHistory'),
        ('http', 'HTTP · SSE', 'raw', '')], theme='server')

N('tm', 'task_manager', '6→5→2→1 · 센트 판정 · 재시도 · 페그 각도 · 백래시', 690, 390, 340,
  ins=[('cmd', '/tuning/start·pause·resume·cancel', 'srv', 'CommandTuning'),
       ('set', '/tuning/get_settings · set_settings', 'srv', 'Get/SetSettings')],
  outs=[('state', '/tuning/state', 'top', 'TuningState 발행'),
        ('meas', '/audio/measure_pitch', 'act', 'MeasurePitch'),
        ('play', '/robot/play_string', 'act', 'PlayString'),
        ('peg', '/robot/adjust_peg', 'act', 'AdjustPeg'),
        ('strum', '/robot/strum_all', 'act', 'StrumAll'),
        ('save', '/tuning/save_job', 'srv', 'SaveJob')], theme='server')

N('rs', 'result_store', 'SQLite — jobs · string_results · settings_snapshot', 300, 672, 340,
  ins=[('save', '/tuning/save_job', 'srv', 'SaveJob'),
       ('hist', '/tuning/get_history', 'srv', 'GetHistory')],
  outs=[('db', 'PARTIAL·FAILED 도 저장', 'raw', '')], theme='server')

N('aa', 'audio_analyzer', 'onset 검출 → FFT 유도 + 협대역 YIN · 관문 20dB / 12센트',
  1180, 118, 350,
  ins=[('meas', '/audio/measure_pitch', 'act', 'MeasurePitch'),
       ('pcm', 'USB 외장 마이크', 'raw', '44.1kHz · OS 향상 해제')],
  outs=[('to', 'onset 부터 타임아웃', 'raw', '')], theme='mic')

N('rc', 'robot_controller', '팔이 하나라 동시 실행은 거부 · dry_run 기본값 true',
  1180, 892, 350,
  ins=[('play', '/robot/play_string', 'act', 'PlayString'),
       ('peg', '/robot/adjust_peg', 'act', 'AdjustPeg'),
       ('strum', '/robot/strum_all', 'act', 'StrumAll')],
  outs=[('dsr', '/dsr01/dsr_controller2/…', 'raw', 'DSR_ROBOT2 · 유선 LAN · M0609+RG2')],
  theme='robot')

# ── 그룹 ─────────────────────────────────────────────────────
GROUPS = [
    ('③ 마이크 PC', '정정인', 'mic', ['aa'], 60, 30),
    ('② 서버 PC', '정동휘', 'server', ['web', 'tm', 'rs'], 60, 30),
    ('① 로봇 제어 PC', '정묵 · 성현', 'robot', ['rc'], 60, 30),
]

# ── 배선 (from노드,포트) → (to노드,포트), 채널x, 데이터 설명 ──
WIRES = [
    ('brow', 'post', 'web', 'post', None),
    ('web', 'http', 'brow', 'http', (270, 600)),
    ('web', 'cmd', 'tm', 'cmd', None),
    ('web', 'set', 'tm', 'set', None),
    ('tm', 'state', 'web', 'state', (1062, 628)),
    ('web', 'hist', 'rs', 'hist', (668, 640)),
    ('tm', 'save', 'rs', 'save', (1050, 652)),
    ('tm', 'meas', 'aa', 'meas', 1074),
    ('tm', 'play', 'rc', 'play', 1086),
    ('tm', 'peg', 'rc', 'peg', 1098),
    ('tm', 'strum', 'rc', 'strum', 1110),
]

# 데이터 설명 — (x, y, 색키, 제목, 줄들)
NOTES = [
    (1140, 296, 'act', '/audio/measure_pitch 로 오가는 데이터', [
        'Goal    job_id · string_number · measurement_id',
        '        target_hz · timeout_sec · onset_timeout_sec',
        'Feedback  준비 완료 (onset 대기 시작)',
        'Result  success · frequency_hz · failure_reason']),
    (1140, 736, 'act', '/robot/* 로 오가는 데이터', [
        'play_string   Goal  string_number',
        'adjust_peg    Goal  string_number · direction(±1) · angle_deg',
        'strum_all     Goal  string_numbers[]',
        '공통 Result   success · message']),
    (44, 880, 'top', '/tuning/state 로 흐르는 데이터', [
        'phase · 현재 줄 · StringResult[4] · 갱신 시각',
        'QoS  reliable · transient local · depth 1',
        '변화가 있을 때만 발행한다 (주기 갱신이 아니다)']),
    (460, 880, 'srv', '/tuning/* 서비스', [
        'start · pause · resume · cancel  →  CommandTuning 하나를',
        '                                    네 이름이 공유한다',
        'get_history   job_id 비면 목록, 채우면 줄별 상세',
        'save_job      JobSummary + StringResult[4] + 설정 스냅샷']),
]



# ── 그리기 ───────────────────────────────────────────────────
def esc(t):
    return t.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def group_box(g):
    title, who, theme, keys, padx, pady = g
    xs = [NODES[k].x for k in keys]
    ys = [NODES[k].y for k in keys]
    xe = [NODES[k].x + NODES[k].w for k in keys]
    ye = [NODES[k].y + NODES[k].h for k in keys]
    x, y = min(xs) - padx, min(ys) - pady - 22
    w, h = max(xe) - x + padx, max(ye) - y + pady
    col, bg = C[theme], C[theme + '_bg']
    s = f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="{bg}" stroke="{col}" stroke-width="1.8"/>'
    s += f'<rect x="{x}" y="{y}" width="{w}" height="30" rx="12" fill="{col}"/>'
    s += f'<rect x="{x}" y="{y+18}" width="{w}" height="12" fill="{col}"/>'
    s += (f'<text x="{x+14}" y="{y+20}" class="gt">{esc(title)}</text>'
          f'<text x="{x+14+len(title)*13.5}" y="{y+20}" class="gw">{esc(who)}</text>')
    return s


def card(n):
    col, soft = C[n.theme], C[n.theme + '_soft']
    s = f'<rect x="{n.x}" y="{n.y}" width="{n.w}" height="{n.h}" rx="7" fill="{C["card"]}" stroke="{C["cardline"]}" stroke-width="1.2"/>'
    s += f'<path d="M{n.x} {n.y+HEAD_H} v-{HEAD_H-7} a7 7 0 0 1 7-7 h{n.w-14} a7 7 0 0 1 7 7 v{HEAD_H-7} z" fill="{soft}"/>'
    s += f'<line x1="{n.x}" y1="{n.y+HEAD_H}" x2="{n.x+n.w}" y2="{n.y+HEAD_H}" stroke="{C["cardline"]}" stroke-width="1.2"/>'
    s += f'<rect x="{n.x}" y="{n.y+4}" width="4" height="{n.h-8}" fill="{col}" rx="2"/>'
    s += f'<text x="{n.x+13}" y="{n.y+22}" class="nt" fill="{C["ink"]}">{esc(n.title)}</text>'
    if n.sub:
        s += f'<text x="{n.x+13}" y="{n.y+39}" class="ns">{esc(n.sub)}</text>'
    for i, (k, label, kind, typ) in enumerate(n.ins):
        cy = n.port_y('in', i)
        kc = KIND[kind][1]
        s += f'<circle cx="{n.x}" cy="{cy}" r="4.5" fill="{kc}" stroke="{C["card"]}" stroke-width="1.6"/>'
        s += f'<text x="{n.x+11}" y="{cy+3.6}" class="pl">{esc(label)}</text>'
        if typ:
            s += f'<text x="{n.x+11}" y="{cy+13.5}" class="pt" fill="{kc}">{esc(typ)}</text>'
    for i, (k, label, kind, typ) in enumerate(n.outs):
        cy = n.port_y('out', i)
        kc = KIND[kind][1]
        s += f'<circle cx="{n.x+n.w}" cy="{cy}" r="4.5" fill="{kc}" stroke="{C["card"]}" stroke-width="1.6"/>'
        s += f'<text x="{n.x+n.w-11}" y="{cy+3.6}" class="pl" text-anchor="end">{esc(label)}</text>'
        if typ:
            s += f'<text x="{n.x+n.w-11}" y="{cy+13.5}" class="pt" text-anchor="end" fill="{kc}">{esc(typ)}</text>'
    return s


def wire(w):
    fk, fp, tk, tp, route = w
    a, b = NODES[fk], NODES[tk]
    x1, y1 = a.pout(fp)
    x2, y2 = b.pin(tp)
    kind = dict((p[0], p[2]) for p in a.outs)[fp]
    col = KIND[kind][1]
    if route is None:
        mx = (x1 + x2) / 2
        d = f'M{x1} {y1} H{mx} V{y2} H{x2}'
    elif isinstance(route, tuple):
        cx, cy = route                     # 카드 사이로 우회한다
        bx = x2 - 30
        d = f'M{x1} {y1} H{cx} V{cy} H{bx} V{y2} H{x2}'
    else:
        d = f'M{x1} {y1} H{route} V{y2} H{x2}'
    return (f'<path d="{d}" fill="none" stroke="{col}" stroke-width="1.8" opacity="0.8" '
            f'stroke-linejoin="round" marker-end="url(#ah_{kind})"/>')


def note(n):
    x, y, kindkey, title, lines = n
    col = KIND[kindkey][1]
    wdt = max([tw(title, 11.5)] + [tw(li, 10.5) for li in lines]) + 30
    hgt = 26 + len(lines) * 14 + 8
    s = (f'<rect x="{x}" y="{y}" width="{wdt:.0f}" height="{hgt}" rx="6" fill="{C["card"]}" '
         f'stroke="{col}" stroke-width="1.2" opacity="0.97"/>')
    s += f'<rect x="{x}" y="{y}" width="4" height="{hgt}" rx="2" fill="{col}"/>'
    s += f'<text x="{x+13}" y="{y+17}" class="ntt" fill="{col}">{esc(title)}</text>'
    for j, li in enumerate(lines):
        s += f'<text x="{x+13}" y="{y+34+j*14}" class="wn" fill="{C["ink2"]}">{esc(li)}</text>'
    return s


def build():
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}">']
    o.append('<defs>')
    for k, (_, c) in KIND.items():
        o.append(f'<marker id="ah_{k}" markerWidth="9" markerHeight="9" refX="7.5" refY="3.2" '
                 f'orient="auto"><path d="M0 0 L7.5 3.2 L0 6.4 z" fill="{c}"/></marker>')
    o.append('<pattern id="grid" width="26" height="26" patternUnits="userSpaceOnUse">'
             f'<path d="M26 0 H0 V26" fill="none" stroke="{C["grid"]}" stroke-width="1"/></pattern>')
    o.append('</defs>')
    o.append('<style>'
             'text{font-family:"Noto Sans CJK KR","Malgun Gothic",sans-serif}'
             f'.gt{{font-size:15px;font-weight:700;fill:#fff}}'
             f'.gw{{font-size:12px;fill:#ffffffcc}}'
             f'.nt{{font-size:15px;font-weight:700}}'
             f'.ns{{font-size:10.5px;fill:{C["muted"]}}}'
             f'.pl{{font-size:11px;font-weight:600;fill:{C["ink2"]}}}'
             f'.pt{{font-size:9.5px;font-weight:600}}'
             f'.wn{{font-size:10.5px;font-family:"IBM Plex Mono",monospace}}'f'.ntt{{font-size:11.5px;font-weight:700}}'
             f'.t1{{font-size:27px;font-weight:700;fill:{C["ink"]}}}'
             f'.t2{{font-size:12.5px;fill:{C["muted"]}}}'
             f'.lg{{font-size:11px;fill:{C["ink2"]};font-weight:600}}'
             '</style>')
    o.append(f'<rect width="{W}" height="{H}" fill="{C["bg"]}"/>')
    o.append(f'<rect width="{W}" height="{H}" fill="url(#grid)" opacity="0.55"/>')
    o.append(f'<text x="{W/2}" y="46" class="t1" text-anchor="middle">TUNIO 시스템 아키텍처 v2</text>')
    o.append(f'<text x="{W/2}" y="68" class="t2" text-anchor="middle">'
             '2026-09-21 · PC 3대 · ROS 2 Jazzy · ROS_DOMAIN_ID 10 · 선 위 상자는 그 인터페이스로 오가는 데이터</text>')
    for g in GROUPS:
        o.append(group_box(g))
    for w in WIRES:
        o.append(wire(w))
    for n in NODES.values():
        o.append(card(n))
    for nt in NOTES:
        o.append(note(nt))
    # 범례
    lx, ly = 48, H - 46
    o.append(f'<rect x="{lx-14}" y="{ly-24}" width="470" height="40" rx="7" fill="{C["card"]}" stroke="{C["cardline"]}"/>')
    for i, (k, (nm, c)) in enumerate(KIND.items()):
        x = lx + i * 116
        o.append(f'<circle cx="{x}" cy="{ly-4}" r="4.5" fill="{c}"/>')
        o.append(f'<text x="{x+11}" y="{ly-0.6}" class="lg">{nm}</text>')
    o.append(f'<text x="{lx+478}" y="{ly-0.6}" class="t2">'
             '3 PC 는 서버 PC 의 Fast DDS Discovery Server(UDP 11811)를 거쳐 서로를 찾는다 · '
             'sim 경로: fake_hardware · replay_wav</text>')
    o.append('</svg>')
    return '\n'.join(o)


if __name__ == '__main__':
    import sys
    io.open(sys.argv[1], 'w', encoding='utf-8').write(build())
    print('생성:', sys.argv[1])
