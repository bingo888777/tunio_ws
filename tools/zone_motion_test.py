#!/usr/bin/env python3
"""ROS 없는 경유 동작 시각화 테스트. 실행/한계: docs/simulation/금지구역_경유_테스트.md."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tunio_robot'))
from tunio_robot import kinematics as K, motion_primitives as MP, teach_points as TP  # noqa: E402

LOW = np.array([230., -230., 0.])
HIGH = np.array([630., 180., 200.])


def intersects(a, b, low, high):
    """선분/AABB slab 검사. 접촉도 침범으로 취급한다."""
    enter, leave = 0., 1.
    for x, y, lo, hi in zip(a, b, low, high):
        d = y - x
        if abs(d) < 1e-12:
            if x < lo or x > hi:
                return False
        else:
            t, u = sorted(((lo - x) / d, (hi - x) / d))
            enter, leave = max(enter, t), min(leave, u)
            if enter > leave:
                return False
    return True


def zone_bounds(padding):
    """박스 수평 범위에서는 윗면 아래로 통과하지 않는다(바닥 아래 우회도 금지)."""
    low, high = LOW-padding, HIGH+padding
    low[2] = -np.inf
    return low, high


def extra_box_bounds(extra_boxes, padding=0):
    """추가 장애물은 Base mm 두 꼭짓점. 원래 구역과 달리 유한한 상자다."""
    out = []
    for raw in extra_boxes:
        values = np.asarray(raw, dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError('추가 상자는 유한한 좌표 X1 Y1 Z1 X2 Y2 Z2가 필요합니다')
        low, high = np.minimum(values[:3], values[3:]), np.maximum(values[:3], values[3:])
        if np.any(low == high):
            raise ValueError('추가 상자의 X/Y/Z 크기는 모두 0보다 커야 합니다')
        out.append((low-padding, high+padding))
    return out


def tcp_interval(start, goal, padding, depth=0, boxes=None):
    """관절 선형 구간 TCP 검사: 0=비침범 확인, 1=침범, 2=판정 여유 부족.

    중간 TCP와 회전 변위 상한으로 구간 전체를 감싼다. 애매하면 세분화하고,
    끝까지 입증하지 못한 구간은 통과시키지 않는다.
    """
    if boxes is None:
        boxes = [zone_bounds(padding)]
    mid = (np.asarray(start)+np.asarray(goal))/2
    xyz = np.array(K.fk(mid)['tcp'])
    if any(intersects(xyz, xyz, low, high) for low, high in boxes):
        return 1
    reach = sum(math.dist((0, 0, 0), xyz) for _, xyz in K._CHAIN)*1000 + K.DEFAULT_TCP_MM
    radius = reach*np.radians(np.abs(np.asarray(goal)-start)).sum()/2
    if all(np.any(xyz+radius < low) or np.any(xyz-radius > high) for low, high in boxes):
        return 0
    if depth >= 12:
        return 2
    left = tcp_interval(start, mid, padding, depth+1, boxes)
    return left or tcp_interval(mid, goal, padding, depth+1, boxes)


def mesh_zone(model, padding):
    """무한 아래 기둥 대신 로봇 모델의 최대 도달 범위 아래까지 덮는 FCL 상자."""
    import fcl
    low, high = zone_bounds(padding)
    reach = sum(math.dist((0, 0, 0), xyz) for _, xyz in K._CHAIN)*1000
    mesh_radius = max(np.linalg.norm(v, axis=1).max()*1000 for _, _, v, _ in model.meshes)
    low[2] = -reach-mesh_radius-padding-1
    return fcl.CollisionObject(fcl.Box(*((high-low)/1000)),
                               fcl.Transform(np.eye(3), (high+low)/2000))


def minimum_elbow_bend(knots):
    """관절 선형 구간 전체의 |J3| 최솟값. 부호 전환은 표본 사이여도 0이다."""
    values = np.asarray(knots, dtype=float)
    if values.ndim != 2 or values.shape[1] != 6 or not len(values) or not np.isfinite(values).all():
        raise ValueError('유한한 관절각 6개씩 필요합니다')
    j3 = values[:, 2]
    if np.any(j3[:-1]*j3[1:] <= 0):
        return 0.
    return float(np.abs(j3).min())


def j1_turn_near_straight(knots, elbow_limit=15.):
    """|J3| <= 15도인 구간의 J1 이동량. 관절 선형 구간을 정확히 잘라 더한다."""
    minimum_elbow_bend(knots)  # 공통 입력 검증
    total = 0.
    for a, b in zip(knots, knots[1:]):
        delta = b[2]-a[2]
        if delta == 0:
            fraction = float(abs(a[2]) <= elbow_limit)
        else:
            low, high = sorted(((-elbow_limit-a[2])/delta, (elbow_limit-a[2])/delta))
            fraction = max(0., min(1., high)-max(0., low))
        total += abs(b[0]-a[0])*fraction
    return float(total)


def blend_knots(knots, fraction, step=.25):
    """경유점 모서리를 관절 공간의 2차 Bezier로 둥글게 한 비교용 다각선.

    두산 movesj/radius 보간 재현이 아니다. 실제 제어기 궤적은 별도 검사해야 한다.
    """
    minimum_elbow_bend(knots)
    if not math.isfinite(fraction) or not 0 <= fraction <= .45 or not 0 < step <= 1:
        raise ValueError('blend fraction은 0~0.45, step은 0 초과 1 이하여야 합니다')
    values = np.asarray(knots, dtype=float)
    if fraction == 0 or len(values) < 3:
        return values.tolist()
    out = [values[0]]
    for previous, corner, following in zip(values, values[1:], values[2:]):
        entry = corner-fraction*(corner-previous)
        leave = corner+fraction*(following-corner)
        count = max(2, math.ceil(2*max(np.max(abs(corner-entry)),
                                      np.max(abs(leave-corner)))/step))
        for t in np.linspace(0, 1, count+1):
            out.append((1-t)**2*entry+2*t*(1-t)*corner+t*t*leave)
    out.append(values[-1])
    return np.asarray(out).tolist()


def inspect(knots, padding, step, motion, extra_boxes=()):
    knots = np.asarray(knots, dtype=float)
    if knots.ndim != 2 or knots.shape[1] != 6 or not np.isfinite(knots).all():
        raise ValueError('유한한 관절각 6개씩 필요합니다')
    if not math.isfinite(padding) or padding < 0 or not 0 < step <= 1:
        raise ValueError('padding >= 0, 0 < step <= 1 이어야 합니다')
    samples = []
    for a, b in zip(knots, knots[1:]):
        n = max(1, math.ceil(max(abs(b - a)) / step))
        if n + len(samples) > 10000:
            raise ValueError('표본 10000개 초과: 관절각/step을 확인하세요')
        samples.extend(np.linspace(a, b, n, endpoint=False))
    samples.append(knots[-1])
    frames, hit, reach = [], [], []
    boxes = [zone_bounds(padding), *extra_box_bounds(extra_boxes, padding)]
    for q in samples:
        f = K.fk(q)
        p = np.array([[0., 0., 0.], *f['origins'], f['tcp']])
        # ponytail: 축 연결선+상자 여유의 근사. 외형 판정은 URDF/RG2 메시로 교체해야 한다.
        segments = list(zip(p, p[1:])) + [(p[4], f['wrist_casing'])]
        hit.append(any(intersects(a, b, low, high) for low, high in boxes for a, b in segments))
        frames.append((p, np.array(f['wrist_casing'])))
        reach.append(math.dist(f['origins'][1], f['origins'][3]))
    tcp = np.array([p[-1] for p, _ in frames])
    errors = []
    if np.any(knots[:, 0] < -60) or np.any(knots[:, 0] > 240):
        errors.append('J1은 -60~240도 안이어야 합니다')
    if np.any(np.abs(knots[:, 2]) > 150):
        errors.append('M0609 모델의 J3 ±150도 한계 초과')
    tcp_status = [int(any(intersects(tcp[0], tcp[0], low, high) for low, high in boxes))]
    tcp_status.extend(tcp_interval(a, b, padding, boxes=boxes) for a, b in zip(samples, samples[1:]))
    hit = [h or status != 0 for h, status in zip(hit, tcp_status)]
    limit = float(motion.get('j6_recover_at_deg', 280))
    if np.max(abs(knots[:, 5])) >= limit:
        errors.append(f'J6 회복 기준 ±{limit:g}도 도달')
    return dict(frames=frames, hit=hit, tcp=tcp, tcp_status=tcp_status, samples=np.array(samples),
                reach=reach, errors=errors,
                length=float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum()),
                travel=float(np.abs(np.diff(knots, axis=0)).sum()))


def self_check():
    assert j1_turn_near_straight([[0, 0, 30, 0, 0, 0], [60, 0, -30, 0, 0, 0]]) == 30
    assert j1_turn_near_straight([[0]*6, [60, 0, 0, 0, 0, 0]]) == 60
    assert j1_turn_near_straight([[0, 0, 37, 0, 0, 0], [60, 0, 37, 0, 0, 0],
                                 [60, 0, -79, 0, 0, 0]]) == 0
    assert minimum_elbow_bend([[0, 0, 37, 0, 0, 0], [0, 0, -79, 0, 0, 0]]) == 0
    assert minimum_elbow_bend([[0, 0, 37, 0, 0, 0], [0, 0, 79, 0, 0, 0]]) == 37
    assert minimum_elbow_bend([[0, 0, 37, 0, 0, 0], [0, 0, 20, 0, 0, 0]]) == 20
    assert intersects([0, 0, 100], [800, 0, 100], LOW, HIGH)
    assert intersects([230, -230, 0], [230, -230, 0], LOW, HIGH)
    assert not intersects([0, 0, 201], [800, 0, 201], LOW, HIGH)
    assert not intersects([0, 0, 0], [0, 0, 0], LOW, HIGH)
    motion = dict(pick_to_peg_min_z_mm=214, pick_to_peg_margin_mm=5)
    r = inspect([[90, 0, 0, 0, 0, 0]] * 2, 0, 1, motion)
    assert not any(r['hit']) and r['length'] == 0
    assert len(r['samples']) == 2
    try:
        inspect([[float('nan')] * 6] * 2, 0, 1, motion)
    except ValueError:
        pass
    else:
        raise AssertionError('invalid joint accepted')
    low, high = zone_bounds(0)
    assert not intersects([100, 0, 172], [100, 0, 172], low, high)
    assert intersects([400, 0, -10], [400, 0, -10], low, high)
    assert not intersects([400, 0, 201], [400, 0, 201], low, high)
    q = [0, 40, 40, 0, 100, 0]
    assert tcp_interval(q, q, 0) == 1
    a, b = [-30, *q[1:]], [30, *q[1:]]
    assert not intersects(K.fk(a)['tcp'], K.fk(a)['tcp'], low, high)
    assert not intersects(K.fk(b)['tcp'], K.fk(b)['tcp'], low, high)
    assert tcp_interval(a, b, 0) == 1  # 양 끝은 밖이어도 중간 TCP가 박스를 가로지른다
    assert tcp_interval([0]*6, [0]*6, 0) == 0
    # 양끝 TCP는 추가 상자 밖이지만 중간 호가 상자를 통과하는 경우.
    a, b = [-20, 0, 0, 0, 90, 0], [20, 0, 0, 0, 90, 0]
    mid = np.array(K.fk([0, 0, 0, 0, 90, 0])['tcp'])
    extra = [np.r_[mid-5, mid+5].tolist()]
    boxes = extra_box_bounds(extra)
    assert all(not intersects(K.fk(q)['tcp'], K.fk(q)['tcp'], *boxes[0]) for q in [a, b])
    assert tcp_interval(a, b, 0, boxes=boxes) == 1
    assert any(inspect([a, b], 0, 1, {}, extra)['tcp_status'])
    assert not any(inspect([a, b], 0, 1, {})['tcp_status'])
    assert np.allclose(extra_box_bounds([extra[0][3:]+extra[0][:3]])[0], boxes[0])
    for invalid in [[0]*6, [0, 0, 0, 1, 1, float('nan')], [0]*5]:
        try:
            extra_box_bounds([invalid])
        except ValueError:
            pass
        else:
            raise AssertionError('invalid obstacle accepted')
    corner = [[0]*6, [10, 0, 0, 0, 0, 0], [10, 10, 0, 0, 0, 0]]
    rounded = blend_knots(corner, .2, 1)
    assert rounded[0] == corner[0] and rounded[-1] == corner[-1]
    assert [9.5, .5, 0, 0, 0, 0] in rounded
    assert corner[1] not in rounded and blend_knots(corner, 0) == corner
    assert np.all(np.array(rounded) >= 0) and np.all(np.array(rounded) <= 10)
    print('zone_motion_test: self-check passed')


def split_j3_first(knots):
    """각 구간에서 J3만 먼저 목표각으로 이동한 뒤 나머지 축을 움직인다."""
    values = np.asarray(knots, dtype=float)
    if values.ndim != 2 or values.shape[1] != 6 or not len(values) or not np.isfinite(values).all():
        raise ValueError('유한한 관절각 6개씩 필요합니다')
    out = [list(knots[0])]
    for goal in knots[1:]:
        mid = out[-1].copy()
        mid[2] = goal[2]
        if mid != out[-1]:
            out.append(mid)
        if list(goal) != out[-1]:
            out.append(list(goal))
    return out


def envelope(model, samples):
    """메시+TCP의 Base 축 정렬 상자. 선형 극값은 convex hull로 정확히 보존된다."""
    from peg_transfer_sim import transforms
    from scipy.spatial import ConvexHull, QhullError
    if not hasattr(model, '_envelope_meshes'):
        model._envelope_meshes = []
        for link, _, vertices, _ in model.meshes:
            try:
                hull = vertices[ConvexHull(vertices).vertices]
            except QhullError:
                hull = vertices
            model._envelope_meshes.append((link, hull))
    low, high = np.full(3, np.inf), np.full(3, -np.inf)
    for q in samples:
        frames = transforms(np.radians(q))
        # 한 링크에 여러 collision 요소가 있을 수 있다. 순서 대신 link index를 쓴다.
        for link, vertices in model._envelope_meshes:
            t = frames[link]
            v = (vertices @ t[:3, :3].T+t[:3, 3])*1000
            low, high = np.minimum(low, v.min(axis=0)), np.maximum(high, v.max(axis=0))
        t = frames[-1]
        tcp = (t[:3, 3]+.208*t[:3, 2])*1000
        low, high = np.minimum(low, tcp), np.maximum(high, tcp)
    return dict(low_mm=low.tolist(), high_mm=high.tolist(), span_mm=(high-low).tolist(),
                enclosing_box_m3=float(np.prod(high-low)/1e9))


def inspect_mesh(model, result, padding, extra_boxes=()):
    """표본마다 전체 충돌 메시 검사. 표시에도 같은 model.meshes를 사용한다."""
    import fcl
    boxes = [mesh_zone(model, padding)]
    for low, high in extra_box_bounds(extra_boxes, padding):
        boxes.append(fcl.CollisionObject(fcl.Box(*((high-low)/1000)),
                                        fcl.Transform(np.eye(3), (high+low)/2000)))
    checks = []
    for q in result['samples']:
        pairs, _ = model.check(q, distances=False)
        links = {name for _, name, obj in model.objects
                 if any(fcl.collide(obj, box, fcl.CollisionRequest(), fcl.CollisionResult())
                        for box in boxes)}
        checks.append((pairs, links))
    result['mesh_checks'] = checks
    result['hit'] = [bool(links) or status != 0
                     for (_, links), status in zip(checks, result['tcp_status'])]
    result['self_hit'] = [bool(pairs) for pairs, _ in checks]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--peg', type=int, choices=[6, 5, 2, 1], default=6)
    ap.add_argument('--branch', choices=['current', 'opposite'], default='current')
    ap.add_argument('--data', type=Path, default=ROOT/'docs/simulation/peg_transfer_direct.json')
    ap.add_argument('--yaml', type=Path, default=ROOT/'src/tunio_robot/config/teach_points.yaml')
    ap.add_argument('--description', type=Path, help='dsr_description2 경로. 기본은 JSON 기록')
    ap.add_argument('--srdf', type=Path, help='SRDF 경로. 기본은 JSON 기록')
    ap.add_argument('--skeleton', action='store_true', help='메시 없이 연결선만 보는 구형 모드')
    ap.add_argument('--padding-mm', type=float, default=0, help='금지 상자를 확장하는 여유 mm')
    ap.add_argument('--extra-box-mm', type=float, nargs=6, action='append', default=[],
                    help='추가 장애물 X1 Y1 Z1 X2 Y2 Z2 (Base mm), 여러 번 지정 가능')
    ap.add_argument('--blend-fraction', type=float, help='관절 모서리 완화 비율 0~0.45; 실제 두산 보간과 별개')
    ap.add_argument('--step-deg', type=float, default=1)
    ap.add_argument('--via', type=float, nargs=6, help='시험 경유 관절각 J1..J6')
    ap.add_argument('--via-index', type=int, default=0, help='편집할 경유점 번호(0부터)')
    ap.add_argument('--j3-first', action='store_true', help='구간마다 J3 먼저 이동')
    ap.add_argument('--snapshot', type=Path, help='화면을 PNG로 저장한 뒤 종료')
    ap.add_argument('--first-hit', action='store_true', help='첫 충돌 표본부터 표시')
    ap.add_argument('--self-check', action='store_true')
    ap.add_argument('--ui-check', action='store_true', help=argparse.SUPPRESS)
    args = ap.parse_args()
    if not math.isfinite(args.padding_mm) or not 0 <= args.padding_mm <= 10000:
        ap.error('--padding-mm은 0~10000 사이의 유한한 값이어야 합니다')
    if not 0 < args.step_deg <= 1:
        ap.error('--step-deg은 0 초과 1 이하여야 합니다')
    if args.self_check:
        self_check()
        assert split_j3_first([[0]*6, [10, 20, 30, 40, 50, 60]]) == [
            [0]*6, [0, 0, 30, 0, 0, 0], [10, 20, 30, 40, 50, 60]]
        assert split_j3_first([[0]*6, [0]*6]) == [[0]*6]
        try:
            split_j3_first([[0]*6, [0, 1]])
        except ValueError:
            pass
        else:
            raise AssertionError('invalid split input accepted')
        if args.skeleton:
            return
    report = json.loads(args.data.read_text())
    try:
        args.extra_boxes = report.get('extra_boxes_mm', []) + args.extra_box_mm
        extra_box_bounds(args.extra_boxes)
    except (ValueError, TypeError) as exc:
        ap.error(str(exc))
    if args.blend_fraction is None:
        args.blend_fraction = float(report.get('blend_fraction', 0))
    if not math.isfinite(args.blend_fraction) or not 0 <= args.blend_fraction <= .45:
        ap.error('--blend-fraction은 0~0.45 사이여야 합니다')
    model = None
    if not args.skeleton:
        try:
            from peg_transfer_sim import CollisionModel, transforms
            description = args.description or Path(report['description'])
            srdf = args.srdf or Path(report['srdf'])
            model = CollisionModel(description, srdf, include_never=True, tool_model=report.get('tool_model'))
        except (ImportError, KeyError, FileNotFoundError) as exc:
            ap.error(f'메시 로드 실패: {exc}. 환경 복구: bash tools/setup_zone_motion.sh. '
                     '실행은 .venv-zone/bin/python tools/zone_motion_test.py를 사용하세요. '
                     '모델 경로는 --description/--srdf와 JSON의 tool_model을 확인하세요.')
    if args.self_check:
        assert model.check([0, 90, 150, 0, 90, 0], distances=False)[0]
        assert not model.check([0]*6, distances=False)[0]
        assert len(model.meshes) == len(model.objects)
        pose = [0, 90, 150, 0, 90, 0]
        result = inspect([pose, pose], 0, 1, dict(pick_to_peg_min_z_mm=214, pick_to_peg_margin_mm=5))
        inspect_mesh(model, result, 0)
        assert all(result['self_hit'])
        assert np.allclose([t[:3, 3]*1000 for t in transforms(np.radians(pose))[1:]],
                           K.fk(pose)['origins'])
        import fcl
        control = fcl.CollisionObject(fcl.Sphere(.001), fcl.Transform(np.eye(3), [.4, 0, .1]))
        box = fcl.CollisionObject(fcl.Box(*((HIGH-LOW)/1000)),
                                  fcl.Transform(np.eye(3), (HIGH+LOW)/2000))
        assert fcl.collide(control, box, fcl.CollisionRequest(), fcl.CollisionResult())
        # TCP가 닿지 않는 링크 표면의 추가 장애물도 메시 검사에서 잡혀야 한다.
        clear = inspect([[0]*6]*2, 0, 1, {})
        inspect_mesh(model, clear, 0)
        assert not any(clear['hit'])
        link, _, vertices, _ = next(mesh for mesh in model.meshes if mesh[1] == 'link_2')
        frame = transforms(np.zeros(6))[link]
        center = (frame[:3, :3] @ vertices[0] + frame[:3, 3])*1000
        extra = [np.r_[center-5, center+5].tolist()]
        blocked = inspect([[0]*6]*2, 0, 1, {}, extra)
        assert not any(blocked['tcp_status'])
        inspect_mesh(model, blocked, 0, extra)
        assert all('link_2' in links for _, links in blocked['mesh_checks'])
        if model.tool_model:
            pairs = model.check([0, 0, 0, 0, 180, 0], distances=False)[0]
            assert any('rg2_' in ' '.join(pair) for pair in pairs), 'RG2–손목 양성 대조 실패'
            assert len([name for _, name, _, _ in model.meshes if name.startswith('rg2_')]) == 7
        print('mesh positive/negative controls: passed')
        return
    points = TP.load(args.yaml)
    row = next((r for r in report['paths']
                if r['peg'] == args.peg and r['holder_branch'] == args.branch), None)
    if row is None:
        ap.error('자료에 해당 peg/branch 경로가 없습니다')
    start, goal = row['start'], row['goal']
    args.goal_joints_changed = bool(row.get('goal_joints_changed', False))
    args.target_pose_changed = bool(row.get('target_pose_changed', False))
    args.target_pose_note = row.get('target_pose_note', '')
    args.rejected_reason = report.get('rejected_reason') or row.get('rejected_reason', '')
    if args.rejected_reason:
        print(f'폐기 후보 — {args.rejected_reason}', flush=True)
    args.keep_elbow_bent = bool(report.get('keep_elbow_bent', False))
    args.fold_turn = bool(report.get('fold_turn', False))
    via = points.route().resolve(start)
    required = MP._peg_route_required(start[0], TP.side_of(args.peg), points.motion)
    baseline_knots = row.get('baseline_knots', [start, via, goal] if required else [start, goal])
    knots = row.get('knots', [start, via, goal])
    if len(knots) < 3:
        knots = [start, [(a+b)/2 for a, b in zip(start, goal)], goal]
    if not 0 <= args.via_index < len(knots)-2:
        ap.error('--via-index가 경유점 개수를 벗어났습니다')
    from zone_motion_vtk import show
    show(args, points.motion, knots, baseline_knots, model)


if __name__ == '__main__':
    main()
