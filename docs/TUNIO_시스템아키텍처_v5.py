#!/usr/bin/env python3
"""v4 템플릿을 계승한 v5 — 2026-09-30, main c6ec065 기준.
v4 이후: 관제 웹 개편(#107) · 취소/일시정지 즉시 정지·재개는 줄 처음부터(#111) ·
피크 폭 확인/재집기/보충 대기(#88·#115) · TuningState robot_notice/error_message ·
PlayString/PreparePluck resumed·pick_refill_required · 예상 페그 회전(web_node → task_manager 파라미터).

실행: python3 docs/TUNIO_시스템아키텍처_v5.py [출력.svg]
  SVG 만 만든다. PNG 는 rsvg/gi 가 있으면 같이, 없으면(맥) headless Chrome 으로 따로 뽑는다:
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --window-size=2140,1600 \
      --force-device-scale-factor=2 --screenshot=docs/TUNIO_시스템아키텍처_v5.png docs/TUNIO_시스템아키텍처_v5.svg
"""

W, H = 2140, 1600
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

PORT_H, HEAD_H, CARD_PAD = 28, 50, 8


def tw(t, px):
    """한글은 라틴 문자보다 넓다. 대략적인 문자열 폭(px)."""
    k = sum(1 for c in t if ord(c) > 0x1100)
    return (len(t) - k) * px * 0.64 + k * px * 1.0


class Node:
    def __init__(self, key, title, sub, x, y, w, ins, outs, theme):
        self.key, self.title, self.sub = key, title, sub
        self.x, self.y, self.w = x, y, w
        self.ins, self.outs = ins, outs
        self.theme = theme
        self.h = HEAD_H + CARD_PAD * 2 + (len(ins) + len(outs)) * PORT_H + 12

    def port_y(self, side, i):
        return self.y + HEAD_H + CARD_PAD + 11 + (i + (len(self.ins) if side == 'out' else 0)) * PORT_H

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


N('brow', '브라우저', 'localhost:8080 · 관제 웹 (9/29 개편)', 40, 330, 210,
  ins=[('http', 'HTTP · SSE', 'raw', '기타 그림 · 튜너 · 줄별 블록 · 로봇 알림')],
  outs=[('post', 'POST /api/command/*', 'raw', '버튼 요청')], theme='ext')
N('web', 'web_node', 'HTTP 요청 ↔ ROS 서비스 / SSE 상태 전달', 340, 240, 450,
  ins=[('state', '/tuning/state', 'top', 'TuningState 구독'),
       ('post', '브라우저 POST', 'raw', 'HTTP')],
  outs=[('cmd', '/tuning/start · pause · resume · cancel', 'srv', 'CommandTuning'),
        ('reg', '/tuning/reregister_pegs', 'srv', 'CommandTuning · 접수 ≠ 완료'),
        ('set', '/tuning/get_settings · set_settings', 'srv', 'GetSettings / SetSettings'),
        ('hist', '/tuning/get_history', 'srv', 'GetHistory'),
        ('param', '/task_manager/get_parameters', 'srv', 'GetParameters · 캘리브레이션 경로 → 예상 페그 회전'),
        ('http', 'HTTP · SSE /api/events', 'raw', 'JSON 상태 + adjust_hint(예상 페그 각도)')], theme='server')
N('tm', 'task_manager', '순서 · 센트 판정 · 재시도 · 백래시 · 검증 · 정지/재개 · 저장', 900, 240, 500,
  ins=[('cmd', '/tuning/start · pause · resume · cancel', 'srv', 'CommandTuning'),
       ('reg', '/tuning/reregister_pegs', 'srv', 'CommandTuning · 등록 전용 작업'),
       ('set', '/tuning/get_settings · set_settings', 'srv', 'GetSettings / SetSettings'),
       ('param', '/task_manager/get_parameters', 'srv', 'rcl_interfaces GetParameters (rclpy 기본)')],
  outs=[('state', '/tuning/state', 'top', 'TuningState · robot_notice · error_message · 단계 0~13'),
        ('meas', '/audio/measure_pitch', 'act', 'MeasurePitch'),
        ('register', '/robot/register_peg', 'act', 'RegisterPeg(force) · 재등록 버튼 · 재개 전 강제 재등록'),
        ('prepare', '/robot/prepare_pluck', 'act', 'PreparePluck(resumed) → preparation_id · pick_refill_required'),
        ('play', '/robot/play_string', 'act', 'PlayString(resumed · preparation_id) · warning Feedback'),
        ('peg', '/robot/adjust_peg', 'act', 'AdjustPeg'),
        ('home', '/robot/return_home', 'act', 'ReturnHome · 완료 / 취소 뒤 / 재등록 전'),
        ('strum', '/robot/strum_all [호출 보류]', 'act', 'StrumAll · 정상 조율에서 제외 (#104 넥 닦기로 대체 예정)'),
        ('save', '/tuning/save_job', 'srv', 'SaveJob')], theme='server')
N('rs', 'result_store', 'SQLite: jobs · string_results · settings_snapshot', 340, 850, 450,
  ins=[('save', '/tuning/save_job', 'srv', 'SaveJob'),
       ('hist', '/tuning/get_history', 'srv', 'GetHistory')],
  outs=[('db', 'SQLite 저장 · 조회', 'raw', '~/.tunio/tunio.db · 재등록·중단 회차는 저장하지 않음')], theme='server')
N('aa', 'audio_analyzer', 'recorder 연속 입력 → onset → FFT + YIN', 900, 850, 500,
  ins=[('meas', '/audio/measure_pitch', 'act', 'MeasurePitch'),
       ('pcm', 'USB 마이크 PCM', 'raw', '실제 장치 샘플레이트 · OS 향상 해제')],
  outs=[('analysis', '측정 신뢰성 검사', 'raw', '돌출 20dB / 창 일치 12센트 · 과도음 0.5초 제외 (잠정)')], theme='mic')
N('rc', 'robot_controller', '등록 캐시 · 피크 폭 확인/재집기 · 이동/발현/회전 · 즉시 정지·접촉 해제', 1580, 240, 480,
  ins=[('register', '/robot/register_peg', 'act', 'RegisterPeg'),
       ('prepare', '/robot/prepare_pluck', 'act', 'PreparePluck'),
       ('play', '/robot/play_string', 'act', 'PlayString'),
       ('peg', '/robot/adjust_peg', 'act', 'AdjustPeg'),
       ('home', '/robot/return_home', 'act', 'ReturnHome'),
       ('strum', '/robot/strum_all [호환 경로]', 'act', 'StrumAll')],
  outs=[('dsr', '/dsr01/dsr_controller2/…', 'srv', 'DSR_ROBOT2 래퍼 이동·I/O + move_stop · check_motion (즉시 정지)'),
        ('modbus', 'Compute Box Modbus TCP', 'raw', '그리퍼 폭 읽기 전용 (258+9) · 피크 13.3mm↑ / 페그 물림 판정')], theme='robot')
N('driver', 'dsr_controller2 · Compute Box', '외부 의존성 · 이 저장소에 복사하지 않음', 1580, 850, 480,
  ins=[('dsr', '/dsr01/dsr_controller2/…', 'srv', '로봇 이동·상태·I/O 서비스 (유선 LAN 192.168.1.100)'),
       ('modbus', 'Modbus TCP 192.168.1.1', 'raw', '홀딩 레지스터 258~275 읽기만')],
  outs=[('hw', 'M0609 + RG2', 'raw', '그리퍼는 Digital I/O 프리셋(open 38 / close 15 / peg 21mm)')], theme='robot')
GROUPS = [
    ('① 관제·오디오 PC', '마이크 · 서버 · 웹', 'server', ['web','tm','rs','aa'], 44, 30),
    ('② 로봇 제어 PC', '로봇 노드 · 드라이버', 'robot', ['rc','driver'], 44, 30),
]
WIRES = [
    ('brow','post','web','post',None),
    ('web','http','brow','http',(810,750)),
    ('web','cmd','tm','cmd',820),
    ('web','reg','tm','reg',840),
    ('web','set','tm','set',860),
    ('web','param','tm','param',880),
    ('tm','state','web','state',(1435,770)),
    ('web','hist','rs','hist',(825,800)),
    ('tm','save','rs','save',(1450,1150)),
    ('tm','meas','aa','meas',(1470,800)),
    ('tm','register','rc','register',1490),
    ('tm','prepare','rc','prepare',1502),
    ('tm','play','rc','play',1514),
    ('tm','peg','rc','peg',1526),
    ('tm','home','rc','home',1538),
    ('tm','strum','rc','strum',1550),
    ('rc','dsr','driver','dsr',(2090,780)),
    ('rc','modbus','driver','modbus',(2110,800)),
]
NOTES = [
 (45,1260,'top','Topic | /tuning/state',[
   'TuningState: phase · 현재 줄 · 유효 측정값 · StringResult[]',
   '  + robot_notice(피크 없음 등 로봇 알림) · error_message(중단 사유)',
   'QoS: reliable · transient local · depth 1',
   '단계 12 페그 등록 / 13 발현 준비 · 9 일시정지(보충 대기 포함)']),
 (580,1260,'srv','Service | 페그 재등록 · 정지/재개',[
   'CommandTuning {} → accepted · message (접수 ≠ 완료)',
   '재등록: ReturnHome → RegisterPeg(force) 6→5→2→1 → ReturnHome → IDLE',
   '  조율 1회마다 필요 · 시작 허가 1회 발급 · 측정·저장 없음',
   '취소·일시정지: 로봇 즉시 정지(MoveStop→IDLE) · 접촉만 풀고 대기',
   '재개: 그 줄을 처음부터 (조절·등록 중이었으면 강제 재등록)']),
 (1110,1260,'act','Action | 발현 준비와 측정',[
   'PreparePluck(string_number · resumed) → preparation_id',
   '  피크 폭 확인 → 없으면 재집기 2회 → 홈 → pick_refill_required',
   'MeasurePitch: job_id · string_number · measurement_id',
   '  target_hz · timeout_sec · onset_timeout_sec → ready feedback',
   'PlayString(string_number · resumed · preparation_id) · 후퇴 후 폭 재확인',
   '측정 Result: success · frequency_hz · failure_reason']),
 (1650,1260,'act','Action | 로봇 결과와 복귀',[
   'RegisterPeg: string_number · force → 각도 캐시(peg_state.json)',
   'AdjustPeg: string_number · direction · angle_deg',
   '  백래시 마무리(풀기→조임)는 서버가 결정',
   'ReturnHome: 빈 Goal → 피크 반환 · 대기 자세',
   '로봇 성공 Result 확인 후에만 후속 명령',
   'StrumAll 점선: 정의 유지 / 정상 호출 보류']),
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
            f'stroke-dasharray="{'7 5' if fp == 'strum' else 'none'}" '
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
    o.append(f'<text x="{W/2}" y="46" class="t1" text-anchor="middle">TUNIO 시스템 아키텍처 v5</text>')
    o.append(f'<text x="{W/2}" y="68" class="t2" text-anchor="middle">'
             '2026-09-30 · PC 2대 · ROS 2 Jazzy · 관제 웹 개편 / 즉시 정지·재개 / 피크 폭 확인·보충 대기 · 포트별 통신 종류와 메시지 타입</text>')
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
    o.append(f'<text x="{lx+478}" y="{ly-8}" class="t2">'
             '화살표: Action/Service 요청 방향 · 응답/Feedback은 같은 인터페이스로 돌아온다 · '
             'Topic: 발행 → 구독</text>')
    o.append(f'<text x="{lx+478}" y="{ly+6}" class="t2">'
             'PC 간 DDS: /robot/register_peg · prepare_pluck · play_string · adjust_peg · return_home — '
             '/audio/measure_pitch 는 한 PC 안 · 기존 v2~v4 보존</text>')
    o.append('</svg>')
    return '\n'.join(o)


if __name__ == '__main__':
    import sys
    from pathlib import Path
    try:
        import gi
        gi.require_version('Rsvg', '2.0')
        from gi.repository import Rsvg
        import cairo
    except (ImportError, ValueError):
        dest = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_suffix('.svg')
        dest.write_text(build(), encoding='utf-8')
        print('생성:', dest, '(PNG 는 rsvg/gi 가 없어 건너뜀 — 머리말의 Chrome 명령으로)')
        sys.exit(0)
    dest = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_suffix('.svg')
    dest.write_text(build(), encoding='utf-8')
    print('생성:', dest)
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, W * 2, H * 2)
    ctx = cairo.Context(surface)
    ctx.scale(2, 2)
    viewport = Rsvg.Rectangle()
    viewport.x, viewport.y, viewport.width, viewport.height = 0, 0, W, H
    Rsvg.Handle.new_from_file(str(dest)).render_document(ctx, viewport)
    surface.write_to_png(str(dest.with_suffix('.png')))
    print('생성:', dest, dest.with_suffix('.png'))
