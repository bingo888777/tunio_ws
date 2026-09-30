#!/usr/bin/env python3
"""TUNIO 작업 흐름 v3 — 가로 16:9 (시연 영상·프레젠테이션용). 정상 흐름만. 2026-09-30

세로 상세본(TUNIO_작업흐름_v3.dot)과 같은 흐름을 1920×1080 안에 3줄 뱀 모양으로 놓는다.
graphviz 로는 가로 10000px 가 넘어가서 손으로 배치한다. 좌표는 6열 × 3행 격자.

실행: python3 docs/TUNIO_작업흐름_v3_가로.py            → docs/TUNIO_작업흐름_v3_가로.svg
PNG:  rsvg-convert -w 3840 docs/TUNIO_작업흐름_v3_가로.svg -o docs/TUNIO_작업흐름_v3_가로.png
      (맥: headless Chrome --window-size=1920,1080 --force-device-scale-factor=2 --screenshot=…)
"""
from pathlib import Path

W, H = 1920, 1080
COLS = [200, 500, 800, 1110, 1420, 1720]      # 열 중심 x
ROWS = [250, 570, 880]                          # 행 중심 y
BW, BH = 256, 118                               # 상자
DW, DH = 300, 190                               # 마름모

C = {
    'ink': '#1E2724', 'sub': '#4B5652', 'edge': '#5F6B65', 'lab': '#3E4A44',
    'grey': ('#EEF0EE', '#8E99A2'), 'purple': ('#E3DDF3', '#8C7AB8'),
    'green': ('#D2EDE0', '#2E9070'), 'amber': ('#FAE7C6', '#C9873A'),
    'red': ('#FADEDE', '#C25A5A'), 'blue': ('#D9E9F8', '#3E7FB8'),
}
REGIONS = [  # (제목, 부제, 색, x0, y0, x1, y1)
    ('준비 · 페그 재등록', '조율마다 1회 · 관제 웹 버튼', '#8C7AB8', '#F5F3FB', 40, 130, 950, 400),
    ('조율 루프', '줄별 반복 6 → 5 → 2 → 1 · 줄당 페그 조절 최대 5회', '#B8A050', '#FBF9EE', 960, 130, 1880, 1010),
    ('마무리', '', '#3E7FB8', '#EEF5FC', 40, 450, 950, 700),
]
# key: (col,row, 제목, 부제 줄들, 색, 모양)
NODES = {
    'prep':  (0, 0, '사람 준비', ['기타 거치 · 피크 꽂기', '페그 손잡이 눕히기'], 'grey', 'box'),
    'reg':   (1, 0, '페그 각도 등록', ['줄마다 페그를 물어 기록', '6 → 5 → 2 → 1'], 'purple', 'box'),
    'start': (2, 0, '시작', ['관제 웹 버튼'], 'grey', 'box'),
    'pick':  (3, 0, '피크 집기', ['거치대에서 집고', '줄 위로 이동'], 'grey', 'box'),
    'mic':   (4, 0, '마이크 준비', ['ready 뒤에만 발현'], 'green', 'box'),
    'pluck': (5, 0, '줄 튕기기', ['튕기고 즉시 후퇴'], 'green', 'box'),
    'meas':  (5, 1, '주파수 측정', ['FFT + YIN · 약 2초'], 'green', 'box'),
    'judge': (4, 1, '허용 오차 이내?', ['±10 센트'], 'amber', 'diamond'),
    'done':  (3, 1, '줄 완료', ['다음 줄로 (피크 집기부터)'], 'green', 'box'),
    'peg':   (4, 2, '페그 조절', ['피크 반환 → 오차만큼 회전', '높으면 풀었다 조여 올림'], 'red', 'box'),
    'ver':   (2, 1, '최종 검증', ['성공한 줄을 다시 측정', '(다른 줄 조이면 틀어짐)'], 'green', 'box'),
    'home':  (1, 1, '복귀', ['피크 반환 · 대기 자세'], 'blue', 'box'),
    'save':  (0, 1, '결과 저장', ['줄별 판정 · 이력 탭'], 'blue', 'box'),
}


def center(k):
    c, r = NODES[k][0], NODES[k][1]
    return COLS[c], ROWS[r]


def edge_pt(k, side):
    x, y = center(k)
    shape = NODES[k][5]
    w, h = (DW, DH) if shape == 'diamond' else (BW, BH)
    return {'l': (x - w / 2, y), 'r': (x + w / 2, y), 't': (x, y - h / 2), 'b': (x, y + h / 2)}[side]


def esc(t):
    return t.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def node(k):
    c, r, title, subs, col, shape = NODES[k]
    x, y = COLS[c], ROWS[r]
    fill, line = C[col]
    if shape == 'diamond':
        pts = f'{x},{y-DH/2} {x+DW/2},{y} {x},{y+DH/2} {x-DW/2},{y}'
        s = f'<polygon points="{pts}" fill="{fill}" stroke="{line}" stroke-width="3"/>'
    else:
        s = (f'<rect x="{x-BW/2}" y="{y-BH/2}" width="{BW}" height="{BH}" rx="16" '
             f'fill="{fill}" stroke="{line}" stroke-width="3"/>')
    ty = y - (len(subs) * 11) + 2
    s += f'<text x="{x}" y="{ty}" class="t" text-anchor="middle">{esc(title)}</text>'
    for i, li in enumerate(subs):
        s += f'<text x="{x}" y="{ty + 26 + i * 21}" class="s" text-anchor="middle">{esc(li)}</text>'
    return s


def arrow(d, label=None, lx=None, ly=None, dashed=False):
    s = (f'<path d="{d}" fill="none" stroke="{C["edge"]}" stroke-width="3.2" '
         f'stroke-linejoin="round" marker-end="url(#ah)"{" stroke-dasharray=\"10 7\"" if dashed else ""}/>')
    if label:
        s += (f'<rect x="{lx-len(label)*9.5}" y="{ly-15}" width="{len(label)*19}" height="26" rx="6" fill="white" opacity="0.92"/>'
              f'<text x="{lx}" y="{ly+4}" class="l" text-anchor="middle">{esc(label)}</text>')
    return s


def straight(a, sa, b, sb, label=None):
    x1, y1 = edge_pt(a, sa)
    x2, y2 = edge_pt(b, sb)
    return arrow(f'M{x1} {y1} L{x2} {y2}', label, (x1 + x2) / 2, (y1 + y2) / 2 - 16)


def build():
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}">',
         '<defs><marker id="ah" markerWidth="12" markerHeight="12" refX="9" refY="4" orient="auto">'
         f'<path d="M0 0 L9 4 L0 8 z" fill="{C["edge"]}"/></marker></defs>',
         '<style>text{font-family:"Noto Sans CJK KR","Apple SD Gothic Neo","Malgun Gothic",sans-serif}'
         f'.t{{font-size:25px;font-weight:700;fill:{C["ink"]}}}'
         f'.s{{font-size:16.5px;fill:{C["sub"]}}}'
         f'.l{{font-size:17px;font-weight:600;fill:{C["lab"]}}}'
         '.rt{font-size:26px;font-weight:800}.rs{font-size:16px;fill:#6B7571}'
         f'.title{{font-size:34px;font-weight:800;fill:{C["ink"]}}}.sub{{font-size:17px;fill:#6B7571}}</style>',
         f'<rect width="{W}" height="{H}" fill="white"/>',
         f'<text x="40" y="66" class="title">TUNIO 작업 흐름</text>',
         f'<text x="330" y="66" class="sub">로봇이 줄을 튕겨 소리로 재고, 페그를 돌려 맞춘다 · 정상 흐름 · 2026-09-30</text>']
    for title, sub, col, bg, x0, y0, x1, y1 in REGIONS:
        o.append(f'<rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" rx="22" fill="{bg}" stroke="{col}" stroke-width="2.5"/>')
        o.append(f'<text x="{x0+26}" y="{y0+40}" class="rt" fill="{col}">{esc(title)}</text>')
        if sub:
            o.append(f'<text x="{x0+26+len(title)*27+14}" y="{y0+40}" class="rs">{esc(sub)}</text>')
    # 화살표 — 1행 왼→오
    o.append(straight('prep', 'r', 'reg', 'l'))
    o.append(straight('reg', 'r', 'start', 'l'))
    o.append(straight('start', 'r', 'pick', 'l'))
    o.append(straight('pick', 'r', 'mic', 'l'))
    o.append(straight('mic', 'r', 'pluck', 'l'))
    o.append(straight('pluck', 'b', 'meas', 't'))
    # 2행 오→왼
    o.append(straight('meas', 'l', 'judge', 'r'))
    o.append(straight('judge', 'l', 'done', 'r', '예'))
    o.append(straight('done', 'l', 'ver', 'r', '4줄 끝'))
    o.append(straight('ver', 'l', 'home', 'r'))
    o.append(straight('home', 'l', 'save', 'r'))
    # 판정 → 페그 조절 (아래)
    o.append(straight('judge', 'b', 'peg', 't', '아니오'))
    # 페그 조절 → 피크 집기 (왼쪽 위로 되돌아감): 2행 상자 사이 틈(x=1265)으로 올라간다
    px, py = edge_pt('peg', 'l')
    tx, ty = edge_pt('pick', 'b')
    gap = 1265
    o.append(arrow(f'M{px} {py} H{gap} V{ty+38} H{tx} V{ty}', '다시 측정 · 최대 5회', gap, 760))
    # 줄 완료 → 다음 줄 (점선으로 피크 집기로)
    dx, dy = edge_pt('done', 't')
    o.append(arrow(f'M{dx} {dy} V{ty+18} H{tx-70} V{ty}', None, dashed=True))
    # 빈 왼쪽 아래 — 영상 설명용 캡션
    cx, cy = 40, 745
    o.append(f'<rect x="{cx}" y="{cy}" width="910" height="265" rx="22" fill="#FAFAF8" stroke="#C9CFCB" stroke-width="2"/>')
    cap = [('#2E9070', '초록', '마이크가 재고 판정한다 — 조율 피드백은 힘이 아니라 소리'),
           ('#C25A5A', '빨강', '페그 조절 — 각도는 줄별 실측값으로 계산, 마지막은 항상 조이는 방향'),
           ('#8E99A2', '회색', '로봇 이동·사람 준비 · 파랑은 저장·복귀'),
           ('#C9873A', '언제든', '일시정지·취소 → 로봇 즉시 정지. 재개는 그 줄을 처음부터')]
    for i, (col, k, txt) in enumerate(cap):
        y = cy + 52 + i * 52
        o.append(f'<rect x="{cx+30}" y="{y-20}" width="22" height="22" rx="6" fill="{col}"/>')
        o.append(f'<text x="{cx+66}" y="{y-2}" class="t" style="font-size:21px">{esc(k)}</text>')
        o.append(f'<text x="{cx+156}" y="{y-2}" class="s" style="font-size:19px">{esc(txt)}</text>')
    for k in NODES:
        o.append(node(k))
    o.append('</svg>')
    return '\n'.join(o)


if __name__ == '__main__':
    import sys
    dest = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_suffix('.svg')
    dest.write_text(build(), encoding='utf-8')
    print('생성:', dest)
