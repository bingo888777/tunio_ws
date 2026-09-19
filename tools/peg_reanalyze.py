"""저장된 peg*.json 을 새 로직으로 다시 분석한다."""
import json, sys
import numpy as np

src = open("tools/peg_cal.py", encoding="utf-8").read()
ns = {"__name__": "peg_cal_reanalyze"}
exec(compile(src, "peg_cal", "exec"), ns)

for path in sys.argv[1:]:
    d = json.load(open(path, encoding="utf-8"))
    rounds = [[(a, b) for a, b in pts] for pts in d["점"]]
    segs = ns["all_segments"](rounds)
    val, how = ns["value_at_target"](segs, d["목표_hz"])
    v = np.array([x[1] for x in segs])
    print(f"\n{path}  —  {d['줄']}번 {d['음']} (목표 {d['목표_hz']:.4f} Hz)")
    print(f"  구간 {len(segs)}개  범위 {v.min():.3f}~{v.max():.3f}  흩어짐 {(v.max()-v.min())/v.mean()*100:.0f}%")
    print(f"  목표 지점: {val:.3f} 센트/도   k = {1/val:.3f} deg/cent   ({how})")
    if "센트per도" in d:
        old = d["센트per도"]
        print(f"  (예전 요약: {old} 센트/도 = k {1/old:.3f} — 장력 의존성을 반영 안 한 값)")
