"""실행 기록 저장·조회. 로그가 실측을 날리지 않게 하는 것이 요점이다."""
import os

from tunio_robot import runlog


def test_저장하고_다시_읽는다(tmp_path):
    p = runlog.save("turn_peg6_loosen", {"peg": 6, "gripped": True}, str(tmp_path))
    assert p is not None
    body = runlog.load(p)
    assert body["peg"] == 6 and body["gripped"] is True
    assert "saved_at" in body


def test_저장에_실패해도_예외를_안_올린다():
    """로그 때문에 실측이 날아가면 안 된다."""
    assert runlog.save("x", {}, "/이런/경로는/없다/그리고/못/만든다") is None


def test_원본_payload_를_안_건드린다(tmp_path):
    payload = {"peg": 6}
    runlog.save("a", payload, str(tmp_path))
    assert "saved_at" not in payload


def test_조건에_맞는_가장_최근_것을_찾는다(tmp_path):
    d = str(tmp_path)
    for name, body in [("2026-09-19_100000_a.json", {"peg": 6, "gripped": True}),
                       ("2026-09-19_110000_b.json", {"peg": 6, "gripped": False}),
                       ("2026-09-19_120000_c.json", {"peg": 6, "gripped": False}),
                       ("2026-09-19_130000_d.json", {"peg": 1, "gripped": False})]:
        runlog.save(name[:-5], body, d)
        os.rename(os.path.join(d, sorted(os.listdir(d))[-1]),
                  os.path.join(d, name))
    path, body = runlog.latest(d, lambda b: b.get("peg") == 6
                               and b.get("gripped") is False)
    assert body is not None and os.path.basename(path).endswith("_c.json")


def test_없는_디렉터리면_조용히_비운다():
    assert runlog.latest("/없는/경로") == (None, None)


def test_조건에_맞는_게_없으면_비운다(tmp_path):
    runlog.save("a", {"peg": 6}, str(tmp_path))
    assert runlog.latest(str(tmp_path), lambda b: b.get("peg") == 99) == (None, None)


def test_저장된_6번_회전_기록을_읽을_수_있다():
    """2026-09-19 실측. 허공 값과 견줄 기준이라 형식이 깨지면 안 된다."""
    d = os.path.join(os.path.dirname(__file__), "..", "..", "..",
                     "docs", "robot_data")
    path, body = runlog.latest(d, lambda b: b.get("tool") == "peg_turn_check"
                               and b.get("peg") == 6 and b.get("gripped") is True)
    assert body is not None, "6번 파지 회전 기록이 없다"
    assert len(body["steps"]) == 12
    assert abs(body["summary"]["step_from_rest"] + 0.438) < 0.01
