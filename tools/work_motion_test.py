#!/usr/bin/env python3
"""PR #110 이동 함수를 수치 IK 대역으로 실행하는 오프라인 메시 테스트. 로봇 연결 없음."""
import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

from peg_transfer_sim import CollisionModel, ik, transforms
import zone_motion_test as Z

MP, TP, ROOT = Z.MP, Z.TP, Z.ROOT
SEEDS = {2: [0, 40, 80, 0, 100, 0], 7: [170, -40, -80, 0, -100, 0]}


def task_at(q):
    t = transforms(np.radians(q))[-1]
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)  # ZYZ 특이점의 동등 Euler 표현
        angles = Rotation.from_matrix(t[:3, :3]).as_euler('ZYZ', degrees=True)
    return [*Z.K.fk(q)['tcp'], *angles]


class Offline:
    """이동 명령을 좌표 기록으로만 바꾼다. 공식 두산 sol/시간 보간 재현은 아니다."""

    def __init__(self, start, sol):
        self.q, self.sol = list(start), sol
        self.knots = [list(start)]
        self.commands = []

    def ikin(self, task, sol):
        seeds = ([self.q] if sol == self.sol else []) + [SEEDS[sol]]
        choices = ik(task, seeds=seeds)
        choices = [q for q in choices if q[2] * SEEDS[sol][2] > 0]
        if not choices:
            raise ValueError(f'수치 IK 후보 없음: sol 대역={sol}, TCP={task}')
        return min(choices, key=lambda q: np.linalg.norm(q - self.q))

    def movej(self, q, motion):
        self.commands.append({'kind': 'movej', 'goal': list(q)})
        self.q = list(q)
        self.sol = 7 if q[2] < 0 else 2
        self.knots.append(list(q))

    def movel(self, task, vel, acc):
        start = task_at(self.q)
        a = Rotation.from_euler('ZYZ', start[3:], degrees=True)
        b = Rotation.from_euler('ZYZ', task[3:], degrees=True)
        if (a.inv() * b).magnitude() > 1e-5:
            raise ValueError('테스트 대역 movel은 자세가 일정한 직선만 지원합니다')
        self.commands.append({'kind': 'movel', 'goal': list(task)})
        count = max(1, math.ceil(math.dist(start[:3], task[:3]) / 3))
        for t in np.linspace(0, 1, count + 1)[1:]:
            p = [*(np.array(start[:3]) * (1-t) + np.array(task[:3]) * t), *task[3:]]
            self.q = list(self.ikin(p, self.sol))
            self.knots.append(list(self.q))

    def get_current_posj(self):
        return list(self.q)

    def get_current_solution_space(self):
        return self.sol


def plan(points, scene, peg, side, start_kind, high_only=False):
    sol = 7 if side == 'left' else 2
    floor, margin = MP._peg_transfer_floor(points.motion)
    holder = list(points.pick_holder().task)
    holder[2] = max(holder[2] + points.motion['pick_up_mm'], floor + 2*margin)
    pluck = list(points.pluck(peg).task)
    pluck[2] = max(pluck[2] + points.motion.get('pluck_up_mm', 20), floor + 2*margin)
    task = pluck if scene == 'pluck-to-holder' else holder
    if scene == 'peg-to-holder':
        task = TP.backed_off(points.peg(peg).task, points.motion.get('approach_mm', 100))
        sol = points.peg(peg).sol
    if scene == 'transition':
        sol = 2  # 이번에 적용한 오른쪽→왼쪽만. 역방향은 이 도구에서 자동 생성하지 않는다.
    candidates = ik(task, seeds=[SEEDS[sol]])
    if not candidates:
        raise ValueError('시작 자세의 수치 IK를 찾지 못했습니다')
    start = (points.frame('standby').joints if start_kind == 'standby' else candidates[0])
    sim = Offline(start, sol)
    with ExitStack() as stack:
        stack.enter_context(patch.dict(sys.modules, {'DR_common2': SimpleNamespace(posx=list)}))
        stack.enter_context(patch.multiple(
            MP, _api=sim, read_posx=lambda: (task_at(sim.q), sim.sol),
            _movej=sim.movej, _movel_base=sim.movel))
        if scene == 'transition':
            pose, _ = MP._j6_pose(points.peg(peg), points.motion)
            if high_only:
                path = MP._pick_to_peg_approach(
                    pose, 'left', points.route(), points.motion, execute=False)
                for q in path['joint_plan']:
                    sim.movej(q, points.motion)
            else:
                MP._pick_to_peg_approach(pose, 'left', points.route(), points.motion)
        else:
            target = pluck if scene == 'holder-to-pluck' else holder
            MP._transfer_above_floor(target, points.motion)
    return sim


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scene', choices=['transition', 'holder-to-pluck',
                                       'pluck-to-holder', 'peg-to-holder'], default='transition')
    ap.add_argument('--peg', type=int, choices=[6, 5, 2, 1], default=6)
    ap.add_argument('--side', choices=['right', 'left'], default='right', help='거치대/발현 시작 팔 자세')
    ap.add_argument('--start', choices=['work', 'standby'], default='work', help='전환 출발 자세')
    ap.add_argument('--yaml', type=Path, default=ROOT/'src/tunio_robot/config/teach_points.yaml')
    ap.add_argument('--model', type=Path, default=ROOT/'docs/simulation/work_motion_model.json')
    ap.add_argument('--step-deg', type=float, default=.5)
    ap.add_argument('--check', action='store_true', help='창 없이 검사. 충돌/미확정이면 종료 코드 1')
    ap.add_argument('--output', type=Path, help='명령·관절 경로·검사 결과 JSON')
    ap.add_argument('--snapshot', type=Path)
    ap.add_argument('--first-hit', action='store_true')
    ap.add_argument('--high-only', action='store_true', help='전환의 높은 접근점까지만 검사/표시')
    ap.add_argument('--ui-check', action='store_true', help=argparse.SUPPRESS)
    args = ap.parse_args()
    if not math.isfinite(args.step_deg) or not 0 < args.step_deg <= 1:
        ap.error('--step-deg은 0 초과 1 이하여야 합니다')
    if args.scene == 'transition' and args.side != 'right':
        ap.error('새 전환은 오른쪽 출발만 지원합니다. --side right를 사용하세요')
    if args.scene == 'peg-to-holder':
        args.side = TP.side_of(args.peg)
    if args.scene == 'transition' and args.peg not in (6, 5):
        ap.error('새 전환은 6·5번만 적용했습니다. 역방향은 이번 검사 대상이 아닙니다')
    if args.high_only and args.scene != 'transition':
        ap.error('--high-only는 transition에서만 사용합니다')
    if args.start == 'standby' and args.scene != 'transition':
        ap.error('--start standby는 transition에서만 사용합니다')
    try:
        points = TP.load(args.yaml)
        sim = plan(points, args.scene, args.peg, args.side, args.start, args.high_only)
        config = json.loads(args.model.read_text())
        tool = dict(config['tool_model'])
        tool['urdf'] = str(ROOT / tool['urdf'])
        model = CollisionModel(Path(config['description']), Path(config['srdf']),
                               include_never=True, tool_model=tool)
        print('M0609/RG2 메시 검사 중… (재생 중에는 다시 검사하지 않습니다)', flush=True)
        result = Z.inspect(sim.knots, 0, args.step_deg, points.motion)
        Z.inspect_mesh(model, result, 0)
    except (ImportError, OSError, ValueError, MP.MotionError) as exc:
        ap.error(f'{exc}\n환경 준비: bash tools/setup_zone_motion.sh')
    summary = dict(scene=args.scene, peg=args.peg, side=args.side, start=args.start,
                   high_only=args.high_only,
                   samples=len(result['samples']), step_deg=args.step_deg,
                   box_links=sorted({n for _, links in result['mesh_checks'] for n in links}),
                   self_pairs=sorted({p for pairs, _ in result['mesh_checks'] for p in pairs}),
                   tcp_blocked=sum(v != 0 for v in result['tcp_status']),
                   joint_errors=result['errors'],
                   near_straight_j1_deg=Z.j1_turn_near_straight(sim.knots))
    first = next((i for i, hit in enumerate(result['hit'])
                  if hit or result['self_hit'][i]), None)
    summary['first_hit_tcp_mm'] = None if first is None else result['tcp'][first].tolist()
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(dict(summary=summary, commands=sim.commands,
                                               knots=sim.knots), ensure_ascii=False, indent=2))
    if args.check:
        return int(bool(summary['box_links'] or summary['self_pairs']
                        or summary['tcp_blocked'] or summary['joint_errors']))
    from zone_motion_vtk import show
    args.branch = f'{args.scene} / {args.side} / {args.start}'
    args.via_index, args.via, args.padding_mm = 0, None, 0
    args.j3_first, args.blend_fraction = False, 0
    args.readonly = True
    args.path_label = '적용 코드 경로'
    args.precomputed = result
    args.target_pose_changed = True
    args.target_pose_note = ('소스 이동 함수 재사용 · 수치 IK 대역 · 실물 명령 없음\n'
                             + ('높은 접근점까지만 검사' if args.high_only else '국소 하강 포함'))
    show(args, points.motion, sim.knots, sim.knots, model)
    return 0


if __name__ == '__main__':
    sys.exit(main())
