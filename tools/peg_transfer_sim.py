#!/usr/bin/env python3
"""오프라인 피크→페그 직행 검사. ROS/두산 연결·이동 명령은 사용하지 않는다.

실행 예: python tools/peg_transfer_sim.py --description /path/to/dsr_description2 \
    --srdf /path/to/dsr.srdf.xacro --output /tmp/peg_transfer.json
별도 환경 의존성: numpy scipy trimesh python-fcl pycollada pyyaml.
수치 IK는 두산 sol 번호를 재현하지 않는다. 교시된 갈래의 J1/J3/J5 부호로
후보를 고른다. 관절 선형 보간 표본 검사이며 실기 궤적/충돌 인증이 아니다.
"""
import argparse
import itertools
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import fcl
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/tunio_robot'))
from tunio_robot import kinematics as K, teach_points as TP  # noqa: E402

CHAIN = [np.array(K._homog(K._rpy(*rpy), xyz)) for rpy, xyz in K._CHAIN]


def transforms(q):
    t = np.eye(4)
    out = [t.copy()]
    for h, angle in zip(CHAIN, q):
        c, s = np.cos(angle), np.sin(angle)
        z = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        t = t @ h @ z
        out.append(t.copy())
    return out


def ik(pose):
    """고정 초기값의 수치 해 탐색. 유한한 탐색이므로 모든 해 발견을 보장하지 않는다."""
    target = Rotation.from_euler('ZYZ', pose[3:], degrees=True).as_matrix()
    xyz = np.array(pose[:3]) / 1000

    def residual(q):
        t = transforms(q)[-1]
        tcp = t[:3, 3] + t[:3, 2] * .208
        return np.r_[tcp - xyz, Rotation.from_matrix(target.T @ t[:3, :3]).as_rotvec()]

    solutions = []
    for a, b, c, d in itertools.product([0, 180], [-80, 80], [-90, 90], [0, 90, -90, 180]):
        result = least_squares(residual, np.radians([a, b, c, d, 90, 0]), max_nfev=160,
                               gtol=1e-10, ftol=1e-10, xtol=1e-10)
        if np.linalg.norm(residual(result.x)) > 1e-6:
            continue
        q = (np.degrees(result.x) + 180) % 360 - 180
        if q[0] < -60:
            q[0] += 360  # 오프라인 후보의 표현만 선택. 실제 현재각은 환산하지 않는다.
        q[np.abs(q) < 1e-5] = 0
        if not any(np.max(np.abs(q - other)) < .01 for other in solutions):
            solutions.append(q)
    return solutions


class CollisionModel:
    def __init__(self, description, srdf):
        root = ET.parse(description / 'urdf/m0609.white.urdf').getroot()
        self.objects = []
        for index, name in enumerate(['base_link'] + [f'link_{i}' for i in range(1, 7)]):
            link = root.find(f"link[@name='{name}']")
            for collision in link.findall('collision'):
                mesh = collision.find('geometry/mesh')
                path = description / mesh.attrib['filename'].split('dsr_description2/')[1]
                shape = trimesh.load(path, force='mesh')
                shape.apply_scale([float(v) for v in mesh.attrib.get('scale', '1 1 1').split()])
                origin = collision.find('origin')
                xyz = [float(v) for v in origin.attrib.get('xyz', '0 0 0').split()]
                rpy = [float(v) for v in origin.attrib.get('rpy', '0 0 0').split()]
                shape.apply_transform(np.array(K._homog(K._rpy(*rpy), xyz)))
                model = fcl.BVHModel()
                model.beginModel(len(shape.vertices), len(shape.faces))
                model.addSubModel(np.asarray(shape.vertices), np.asarray(shape.faces))
                model.endModel()
                self.objects.append((index, name, fcl.CollisionObject(model)))
        self.excluded = {frozenset((e.attrib['link1'], e.attrib['link2']))
                         for e in ET.parse(srdf).getroot().findall('disable_collisions')}
        self.pairs = [(a, b) for a, b in itertools.combinations(self.objects, 2)
                      if a[1] != b[1] and frozenset((a[1], b[1])) not in self.excluded]

    def check(self, q):
        frames = transforms(np.radians(q))
        for index, _, obj in self.objects:
            t = frames[index]
            obj.setTransform(fcl.Transform(t[:3, :3], t[:3, 3]))
        hit, closest = set(), (float('inf'), None)
        for a, b in self.pairs:
            pair = (a[1], b[1])
            if fcl.collide(a[2], b[2], fcl.CollisionRequest(), fcl.CollisionResult()):
                hit.add(pair)
            distance = fcl.distance(a[2], b[2], fcl.DistanceRequest(), fcl.DistanceResult())
            if distance < closest[0]:
                closest = (distance, pair)
        return hit, closest


def inspect_path(start, goal, model, step, floor, margin, j6_limit=280):
    count = max(1, int(np.ceil(np.max(np.abs(goal - start)) / step)))
    minimum = float('inf')
    min_tcp = float('inf')
    lowest_part = None
    lowest_fraction = None
    collisions = set()
    clearance = float('inf')
    closest = None
    for t in np.linspace(0, 1, count + 1):
        q = start + (goal - start) * t
        geometry = K.fk(q)
        positions = dict(zip(['TCP', 'J3', 'J4', 'J5', 'J6', 'wrist_casing'],
                             [geometry['tcp'], *geometry['origins'][2:], geometry['wrist_casing']]))
        part = min(positions, key=lambda key: positions[key][2])
        if positions[part][2] < minimum:
            minimum, lowest_part, lowest_fraction = positions[part][2], part, float(t)
        min_tcp = min(min_tcp, geometry['tcp'][2])
        hits, near = model.check(q)
        collisions.update(hits)
        if near[0] < clearance:
            clearance, closest = near
    return {'samples': count + 1, 'min_height_mm': minimum,
            'height_ok': minimum >= floor + margin, 'min_height_part': lowest_part,
            'min_height_fraction': lowest_fraction, 'min_tcp_height_mm': min_tcp,
            'j1_ok': all(-60 <= j[0] <= 240 for j in (start, goal)),
            'j6_ok': bool(max(abs(start[5]), abs(goal[5])) < j6_limit),
            'collision_pairs': sorted(collisions), 'min_clearance_mm': clearance * 1000,
            'closest_pair': closest}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--description', type=Path, required=True)
    parser.add_argument('--srdf', type=Path, required=True)
    parser.add_argument('--yaml', type=Path, default=Path('src/tunio_robot/config/teach_points.yaml'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--step-deg', type=float, default=.5)
    args = parser.parse_args()
    if not 0 < args.step_deg <= 1:
        parser.error('--step-deg must be in (0, 1]')
    points = TP.load(args.yaml)
    floor = float(points.motion['pick_to_peg_min_z_mm'])
    margin = float(points.motion['pick_to_peg_margin_mm'])
    holder = points.pick_holder().task.copy()
    holder[2] = max(holder[2] + points.motion['pick_up_mm'], floor + 2 * margin)
    holders = ik(holder)
    # 현재 사용 갈래와 반대쪽 J1 갈래를 비교. 다른 sol로 자동 변경하지 않는다.
    starts = {'current': next(q for q in holders if q[0] < 90 and q[2] > 0 and q[4] > 0),
              'opposite': next(q for q in holders if q[0] > 90 and q[2] < 0 and q[4] < 0)}
    model = CollisionModel(args.description, args.srdf)
    # 검출기가 실제 겹침을 잡는지 알려진 접힌 모델 자세로 검증한다.
    assert model.check([0, 90, 150, 0, 90, 0])[0], "collision positive control failed"
    assert not model.check([0] * 6)[0], "collision negative control failed"
    j6_limit = points.motion['j6_recover_at_deg']
    results = []
    for number in [6, 5, 2, 1]:
        pose = points.peg(number)
        high = TP.backed_off(pose.task, points.motion['approach_mm'])
        high[2] = max(holder[2], high[2])
        goals = ik(high)
        left = number in (6, 5)
        if left:
            goal = next(q for q in goals if 90 < q[0] < 240 and q[2] < 0 and q[4] < 0)
        else:
            goal = next(q for q in goals if q[0] < 90 and q[2] > 0 and q[4] > 0)
        for name, start in starts.items():
            row = {'peg': number, 'holder_branch': name, 'start': start.tolist(),
                   'goal': goal.tolist(), 'high_tcp': high,
                   **inspect_path(start, goal, model, args.step_deg, floor, margin, j6_limit)}
            results.append(row)
            print(number, name, json.dumps(row, ensure_ascii=False), flush=True)
    report = {'scope': 'holder lifted -> high approach only; nominal peg roll=0',
              'excluded': 'RG2, cables, guitar, holder, descent/insertion, actual controller trajectory',
              'ik': 'numerical model; not Doosan sol validation', 'step_deg': args.step_deg,
              'description': str(args.description), 'srdf': str(args.srdf),
              'collision_exclusions': [sorted(v) for v in model.excluded], 'paths': results}
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
