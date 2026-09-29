#!/usr/bin/env python3
"""Qt/VTK 실제 버튼 회귀 검사와 프레임 시간 측정. 실행 인자는 뷰어와 같다."""
import statistics
import sys
import time

from PyQt5 import QtCore, QtTest



def check_rejected_candidate():
    """폐기 자료의 충돌 0건을 새 탐색의 유효 기준으로 재사용하지 못해야 한다."""
    import json
    import subprocess
    import tempfile
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    data = root/'docs/simulation/zone_bent_approach_peg6.json'
    report = json.loads(data.read_text())
    assert report['rejected_reason'] and report['paths']
    assert all(row['rejected_reason'] for row in report['paths'])
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory)/'result.json'
        result = subprocess.run([sys.executable, str(root/'tools/zone_space_search.py'),
                                 '--data', str(data), '--output', str(output)],
                                capture_output=True, text=True, timeout=10)
        assert result.returncode == 2 and report['rejected_reason'] in result.stderr, result.stderr
        assert not output.exists(), '폐기 자료에서 새 후보 생성됨'
    print('폐기 후보 재사용 차단 검사: passed', flush=True)

def check_window(app, window, state, slider, play, route, sequence, actors, render_window):
    from zone_motion_vtk import framebuffer
    QtTest.QTest.qWait(300)
    pixels = framebuffer(render_window)
    assert pixels.std() > 10, "VTK 프레임버퍼가 비어 있습니다"
    idle = state['renders']
    wall, cpu = time.perf_counter(), time.process_time()
    QtTest.QTest.qWait(400)
    idle_cpu = 100*(time.process_time()-cpu)/(time.perf_counter()-wall)
    assert state['renders'] == idle, '정지 중 반복 렌더링'
    meshes = [(a.GetMapper().GetInput().GetMTime(),
               a.GetMapper().GetInput().GetNumberOfCells()) for a in actors]
    checks = state['checks']
    QtTest.QTest.mouseClick(play, QtCore.Qt.LeftButton)
    wall, cpu = time.perf_counter(), time.process_time()
    QtTest.QTest.qWait(1000)
    play_cpu = 100*(time.process_time()-cpu)/(time.perf_counter()-wall)
    assert state['play'] and slider.value() > 0, '재생 클릭 실패'
    QtTest.QTest.mouseClick(play, QtCore.Qt.LeftButton)
    assert not state['play'], '일시정지 클릭 실패'
    count = state['renders']
    QtTest.QTest.qWait(400)
    assert state['renders'] == count, '정지 후 렌더링 지속'
    times = []
    for i in range(1, 21):
        start = time.perf_counter()
        slider.setValue(i % (slider.maximum()+1))
        times.append((time.perf_counter()-start)*1000)
    assert state['checks'] == checks, '프레임 재생 중 충돌 검사를 다시 수행함'
    assert meshes == [(a.GetMapper().GetInput().GetMTime(),
                       a.GetMapper().GetInput().GetNumberOfCells()) for a in actors], '메시 재생성'
    if route.isEnabled():
        route.setCurrentIndex(1)
        assert state['checks'] > checks, '경로 전환 실패'
        route.setCurrentIndex(0)
        sequence.setCurrentIndex(1)
        assert any(state['result']['tcp_status']), 'J3 선행 TCP 침범 회귀'
    else:
        assert not sequence.isEnabled(), '적용 코드 재생은 편집할 수 없어야 한다'
    print(f'GUI passed: pause/play/controls; mesh unchanged; triangles={sum(n for _, n in meshes)}; '
          f'frame median={statistics.median(times):.1f}ms, max={max(times):.1f}ms; '
          f'CPU idle={idle_cpu:.1f}%, playing={play_cpu:.1f}% (one-core equivalent)', flush=True)


if __name__ == '__main__':
    import work_motion_test
    sys.argv.append('--ui-check')
    sys.exit(work_motion_test.main())
