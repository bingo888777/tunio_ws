#!/usr/bin/env python3
"""
교시점 미세 조정 — Base 축으로 mm 단위씩 밀어 보고 그 자리를 적는다.

2026-09-21 통합 실기에서 나온 두 항목이 이 도구를 쓴다 (#56).
  · 피크 잡는 자리가 높다 — 거치대 Z 를 내려야 한다
  · 발현 접촉점이 깊다 — 피크가 기타 넥에 부딪힌 채로 더 밀려 들어가,
    피크가 그리퍼 안쪽으로 파고든다. 접촉점 Z 를 올려야 한다

**팔이 움직인다. 비상정지를 손 닿는 곳에 두고 시작하라.**

교시점 **위** pick_up_mm(거치대) 또는 pluck_up_mm(발현) 만큼 떨어진 자리로 먼저
가고, 거기서 사람이 확인한 뒤에 내려간다. 내려간 뒤로는 한 번에 한 축씩,
기본 1mm 씩만 민다. 전부 Base 기준 직선(movel)이라 팔 모양이 안 바뀐다.

명령
  z -1        Base Z 로 -1mm (축 공백 거리)
  z-1         붙여 써도 된다
  y +0.5      Base Y 로 +0.5mm
  step 0.5    기본 거리를 0.5mm 로 (그 뒤로는 'z-' 'z+' 만 쳐도 된다)
  z-          기본 거리만큼 -
  p           지금 손끝 좌표와 교시값 대비 차이 찍기 (안 움직인다)
  u           직전 이동 되돌리기
  up          교시점 위 대기 자리로 올라가기
  down        교시점(지금 누적된 값)으로 내려가기
  save        지금 자리를 yaml 에 적는다 (.bak 를 남긴다)
  q           끝내기. 적용 안 한 변화가 있으면 yaml 조각을 찍어 준다

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/point_tune.py --point pick.holder
  python3 tools/point_tune.py --point pluck.6
  python3 tools/point_tune.py --point pluck.6 --step 0.5 --speed 20
"""
import argparse
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..", "src", "tunio_robot",
                            "config", "teach_points.yaml")

AXES = {"x": 0, "y": 1, "z": 2}
CMD = re.compile(r"^([xyz])\s*([+-]?\d*\.?\d*)$")


class _Say:
    """motion_primitives.connect 가 기대하는 로거."""

    def info(self, m):
        print(f"  {m}")

    def warn(self, m):
        print(f"  ! {m}")

    def error(self, m):
        print(f"  X {m}")


def fmt(vals, w=8, d=2):
    return "[" + ", ".join(f"{v:{w}.{d}f}" for v in vals) + "]"


def _resolve(points, name):
    """교시점 이름을 Pose 와 '위로 얼마나 떨어져 접근하는지' 로 푼다."""
    if name == "pick.holder":
        return points.pick_holder(), float(points.motion.get("pick_up_mm", 50.0))
    m = re.fullmatch(r"pluck\.([1256])", name)
    if m:
        return points.pluck(int(m.group(1))), float(points.motion.get("pluck_up_mm", 20.0))
    raise SystemExit(f"모르는 교시점이다: {name!r} (pick.holder 또는 pluck.6 같은 형식)")


def _write_back(path, name, task):
    """
    yaml 의 그 교시점 블록에서 x: 줄 하나만 바꾼다.

    주석이 근거를 잔뜩 싣고 있어서 통째로 다시 쓰지 않는다 — PyYAML 로 덤프하면
    그 주석이 전부 날아간다. 블록을 찾아 첫 x: 줄만 갈아 끼우고 .bak 를 남긴다.
    """
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()

    if name == "pick.holder":
        head = [r"^pick:\s*$", r"^  holder:\s*$"]
    else:
        head = [r"^pluck:\s*$", rf"^  {name.split('.')[1]}:\s*$"]

    i, depth = 0, 0
    while i < len(lines) and depth < len(head):
        if re.match(head[depth], lines[i]):
            depth += 1
        i += 1
    if depth < len(head):
        raise SystemExit(f"yaml 에서 {name} 블록을 못 찾았다. 손으로 적어라")

    while i < len(lines):
        line = lines[i]
        if line.strip() and not line.startswith("    "):
            break                                   # 블록이 끝났다
        m = re.match(r"^(\s*x:\s*)\[[^\]]*\](.*)$", line)
        if m:
            lines[i] = f"{m.group(1)}[{', '.join(f'{v:.2f}' for v in task)}]{m.group(2)}\n"
            shutil.copyfile(path, path + ".bak")
            with open(path, "w", encoding="utf-8") as f:
                f.writelines(lines)
            return i + 1
        i += 1
    raise SystemExit(f"{name} 블록 안에 x: 줄이 없다. 손으로 적어라")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--point", required=True,
                    help="pick.holder 또는 pluck.6 / pluck.5 / pluck.2 / pluck.1")
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--step", type=float, default=1.0, help="기본 이동 거리(mm)")
    ap.add_argument("--speed", type=float, default=20.0,
                    help="설정 속도의 %%. 기본 20 — 교시점 근처라 느리게")
    ap.add_argument("--confirm-over", type=float, default=5.0,
                    help="이보다 큰 이동은 매번 확인한다(mm)")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))
    pose, up_mm = _resolve(points, args.point)
    taught = list(pose.task)

    # 이 도구는 교시점 바로 옆에서 논다. 배율을 타면 안 된다.
    motion = dict(points.motion)
    motion["speed_scale"] = 1.0
    scale = max(0.05, args.speed / 100.0)
    v = [x * scale for x in points.motion.get("travel_vel", [50.0, 30.0])]
    a = [x * scale for x in points.motion.get("travel_acc", [100.0, 60.0])]

    print()
    print("=" * 70)
    print(f"교시점 미세 조정 — {args.point}")
    print("=" * 70)
    print(f"  교시값 {fmt(taught)}")
    print(f"  위 대기 자리는 Base +Z {up_mm:.0f}mm")
    print(f"  속도 설정값의 {args.speed:.0f}% (직선 {v[0]:.1f} mm/s), "
          f"기본 거리 {args.step:.2f}mm")
    print("  **팔이 움직인다.** 비상정지를 손 닿는 곳에.")
    print()

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_point_tune", namespace=args.id)
    DR_init.__dsr__node = node

    from tunio_robot import motion_primitives as MP
    robot_cfg = dict(points.robot)
    robot_cfg["id"], robot_cfg["model"] = args.id, args.model
    print("  서비스를 기다린다 (최대 30초) …")
    MP.connect(robot_cfg, points.gripper, logger=_Say())
    print(f"  TCP 확인됨 — tool='{MP._read_named('tool')}' "
          f"tcp='{MP._read_named('tcp')}'\n")

    from DR_common2 import posx
    from DSR_ROBOT2 import get_current_posx, movel
    import DSR_ROBOT2 as DR

    cur = list(taught)                          # 지금 목표로 삼고 있는 좌표
    last = None
    dirty = False

    def here():
        p = get_current_posx()
        p = p[0] if isinstance(p, (list, tuple)) and len(p) == 2 else p
        return [float(x) for x in list(p)[:6]]

    def go(target, why):
        ret = movel(posx(list(target)), vel=v, acc=a, ref=DR.DR_BASE)
        if ret == -1:
            print(f"  X {why} 가 거부됐다 (-1). 안 움직였다")
            return False
        return True

    def above_of(target):
        t = list(target)
        t[2] += up_mm
        return t

    def show():
        now = here()
        d = [now[i] - taught[i] for i in range(3)]
        print(f"  손끝 {fmt(now)}")
        print(f"  교시값 대비  X {d[0]:+.2f}  Y {d[1]:+.2f}  Z {d[2]:+.2f} mm")

    try:
        ans = input(f"  먼저 교시점 위 {up_mm:.0f}mm 로 간다. 진행? [y/N] ").strip().lower()
        if ans != "y":
            print("  그만둔다.")
            return 0
        if not go(above_of(cur), "위 대기 자리로 이동"):
            return 1
        show()
        ans = input("  이제 교시점으로 내려간다. 진행? [y/N] ").strip().lower()
        if ans == "y":
            go(cur, "교시점으로 하강")
            show()

        print("\n  명령: 'z -1' / 'z-' / step 0.5 / p / u / up / down / save / q\n")
        while True:
            try:
                line = input("  > ").strip().lower()
            except EOFError:
                break
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break
            if line == "p":
                show()
                continue
            if line == "up":
                if go(above_of(cur), "위로"):
                    show()
                continue
            if line == "down":
                if go(cur, "아래로"):
                    show()
                continue
            if line == "u":
                if last is None:
                    print("  되돌릴 게 없다")
                    continue
                axis, delta = last
                cur[axis] -= delta
                if go(cur, "되돌리기"):
                    last = None
                    show()
                continue
            if line.startswith("step"):
                try:
                    args.step = abs(float(line.split()[1]))
                    print(f"  기본 거리 {args.step:.2f}mm")
                except (IndexError, ValueError):
                    print("  step 0.5 처럼 쓴다")
                continue
            if line == "save":
                n = _write_back(os.path.abspath(args.yaml), args.point, cur)
                print(f"  적었다 — {args.yaml}:{n}  (원본은 .bak)")
                print(f"    x: [{', '.join(f'{x:.2f}' for x in cur)}]")
                dirty = False
                continue

            m = CMD.match(line.replace(" ", ""))
            if not m:
                print("  못 알아듣겠다. 'z -1' / 'z-' / step / p / u / up / down / save / q")
                continue
            axis = AXES[m.group(1)]
            raw = m.group(2)
            if raw in ("", "+", "-"):
                delta = args.step * (-1.0 if raw == "-" else 1.0)
            else:
                delta = float(raw)
            if abs(delta) > args.confirm_over:
                ok = input(f"  {abs(delta):.2f}mm 는 크다. 진행? [y/N] ").strip().lower()
                if ok != "y":
                    continue
            cur[axis] += delta
            if go(cur, f"{m.group(1).upper()} {delta:+.2f}mm"):
                last, dirty = (axis, delta), True
                show()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print()
    if dirty:
        d = [cur[i] - taught[i] for i in range(3)]
        print("  적용 안 한 변화가 있다. yaml 에 적으려면 —")
        print(f"    {args.point} 의  x: [{', '.join(f'{x:.2f}' for x in cur)}]")
        print(f"    교시값 대비 X {d[0]:+.2f}  Y {d[1]:+.2f}  Z {d[2]:+.2f} mm")
        print("  (다음에 save 를 치면 도구가 직접 적는다)")
    else:
        print("  바뀐 것 없음.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
