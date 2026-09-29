"""VTK 표시 전용. 충돌 메시를 GPU 버퍼에 유지하고 링크 행렬만 갱신한다."""
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

from peg_transfer_sim import transforms
import zone_motion_test as Z


def poly_actor(vertices, triangles=None):
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(np.asarray(vertices, dtype=float), deep=True))
    poly = vtk.vtkPolyData()
    poly.SetPoints(points)
    cells = vtk.vtkCellArray()
    if triangles is None:
        cells.InsertNextCell(len(vertices))
        for i in range(len(vertices)):
            cells.InsertCellPoint(i)
        poly.SetLines(cells)
    else:
        connectivity = np.column_stack((np.full(len(triangles), 3), triangles)).astype(np.int64)
        cells.SetCells(len(triangles), numpy_to_vtkIdTypeArray(connectivity.ravel(), deep=True))
        poly.SetPolys(cells)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor


def framebuffer(render_window):
    """Qt grab은 네이티브 OpenGL 자식 창을 담지 못하므로 VTK에서 직접 읽는다."""
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(render_window)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()
    capture.Update()
    output = capture.GetOutput()
    width, height, _ = output.GetDimensions()
    return vtk_to_numpy(output.GetPointData().GetScalars()).reshape(height, width, 3)[::-1].copy()


def show(args, motion, knots, baseline_knots, model):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = QtWidgets.QWidget()
    window.setWindowTitle(f'M0609 경로 테스트 — {args.peg}번 / {args.branch} — VTK')
    window.resize(1320, 850)
    layout = QtWidgets.QHBoxLayout(window)
    view = QVTKRenderWindowInteractor(window)
    layout.addWidget(view, 1)
    panel = QtWidgets.QWidget()
    scroll = QtWidgets.QScrollArea()
    scroll.setFixedWidth(360)
    scroll.setWidgetResizable(True)
    scroll.setWidget(panel)
    layout.addWidget(scroll)
    controls = QtWidgets.QVBoxLayout(panel)
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(.96, .97, .99)
    render_window = view.GetRenderWindow()
    render_window.SetMultiSamples(0)
    render_window.AddRenderer(renderer)
    view.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
    actors, matrices = [], []
    if model:
        for _, _, vertices, triangles in model.meshes:
            actor = poly_actor(vertices*1000, triangles)
            renderer.AddActor(actor)
            matrix = vtk.vtkMatrix4x4()
            actor.SetUserMatrix(matrix)
            actors.append(actor)
            matrices.append(matrix)
    extra_boxes = getattr(args, 'extra_boxes', [])
    visible_boxes = [(Z.LOW, Z.HIGH), *Z.extra_box_bounds(extra_boxes)]
    for index, (low, high) in enumerate(visible_boxes):
        box = vtk.vtkCubeSource()
        box.SetCenter(*((low+high)/2))
        box.SetXLength(high[0]-low[0])
        box.SetYLength(high[1]-low[1])
        box.SetZLength(high[2]-low[2])
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(box.GetOutputPort())
        box_actor = vtk.vtkActor()
        box_actor.SetMapper(mapper)
        box_actor.GetProperty().SetColor(*((1, .15, .15) if index == 0 else (1, .55, .05)))
        box_actor.GetProperty().SetOpacity(.16 if index == 0 else .3)
        box_actor.GetProperty().EdgeVisibilityOn()
        renderer.AddActor(box_actor)
    sphere = vtk.vtkSphereSource()
    sphere.SetRadius(6)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(sphere.GetOutputPort())
    tcp_actor = vtk.vtkActor()
    tcp_actor.SetMapper(mapper)
    renderer.AddActor(tcp_actor)
    axes = vtk.vtkAxesActor()
    axes.SetTotalLength(100, 100, 100)
    for caption in [axes.GetXAxisCaptionActor2D(), axes.GetYAxisCaptionActor2D(),
                    axes.GetZAxisCaptionActor2D()]:
        caption.SetWidth(.04)
        caption.SetHeight(.025)
        caption.GetCaptionTextProperty().SetColor(.1, .1, .1)
    renderer.AddActor(axes)

    note = QtWidgets.QLabel('M0609 충돌 메시 전체 사용\n빨강: 구역 침범 · 자홍: 자기충돌\n'
                           '초록 점: TCP · 빨간 점: TCP 구간 차단\n'
                           '빨간 박스 XY 범위에서는 Z>200mm\n'
                           + (f'추가 장애물: {len(extra_boxes)}개 (주황)\n기타 형상은 추가 상자 범위만 검사\n'
                              if extra_boxes else '기타 목·헤드·옆 페그 형상 미포함\n') +
                           (f"RG2 {model.tool_model['width_mm']:g}mm 메시 포함 · 골무/케이블 제외"
                            if model and model.tool_model else 'RG2·피크·케이블 미포함')
                           +' / 실물 명령 없음')
    note.setWordWrap(True)
    rejection = getattr(args, 'rejected_reason', '')
    if rejection:
        warning = QtWidgets.QLabel('폐기 후보 — '+rejection)
        warning.setWordWrap(True)
        warning.setStyleSheet('color: #b00020; font-weight: bold')
        controls.addWidget(warning)
    if getattr(args, 'target_pose_changed', False):
        note.setText(note.text()+'\n※ 후보: '+(getattr(args, 'target_pose_note', '') or '기존과 다른 도착 위치·방향'))
    elif getattr(args, 'goal_joints_changed', False):
        note.setText(note.text()+'\n※ 기존 대비 도착 관절각 변경')
    if getattr(args, 'blend_fraction', 0):
        note.setText(note.text()+f'\n후보 관절 모서리 {args.blend_fraction:g} 완화 · 두산 보간 재현 아님')
    if getattr(args, 'fold_turn', False):
        note.setText(note.text()+'\n분할 시험 · 도착측 완전 펼침은 남음')
    controls.addWidget(note)
    route = QtWidgets.QComboBox()
    route.addItems([getattr(args, 'path_label', None) or
                    ('폐기 후보 경로' if rejection else '후보 경로'), '기존 경로'])
    sequence = QtWidgets.QComboBox()
    sequence.addItems(['동시 이동', 'J3 먼저'])
    sequence.setCurrentIndex(int(args.j3_first))
    controls.addWidget(route)
    controls.addWidget(sequence)
    via = knots[args.via_index+1]
    default_text = ', '.join(f'{v:.3f}' for v in via)
    controls.addWidget(QtWidgets.QLabel(f'경유점 {args.via_index+1} / J1~J6 (도)'))
    edit = QtWidgets.QLineEdit(', '.join(map(str, args.via)) if args.via else default_text)
    edit.setFont(QtGui.QFont("monospace", 8))
    edit.setCursorPosition(0)
    controls.addWidget(edit)
    controls.addWidget(QtWidgets.QLabel('박스 확장 여유 (mm)'))
    padding = QtWidgets.QDoubleSpinBox()
    padding.setRange(0, 10000)
    padding.setValue(args.padding_mm)
    controls.addWidget(padding)
    apply_button = QtWidgets.QPushButton('경로 다시 검사')
    controls.addWidget(apply_button)
    status = QtWidgets.QLabel()
    status.setWordWrap(True)
    slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    controls.addWidget(slider)
    play = QtWidgets.QPushButton('재생')
    first = QtWidgets.QPushButton('첫 침범 / 충돌로')
    fit = QtWidgets.QPushButton('화면 맞춤')
    for button in [play, first, fit]:
        controls.addWidget(button)
    controls.addWidget(QtWidgets.QLabel('마우스: 회전 / 휠: 확대\n10fps 상한 · 실제 제어기의 시간 아님'))
    controls.addWidget(status, 1)
    timer = QtCore.QTimer(window)
    timer.setSingleShot(True)
    timer.setInterval(100)
    state = dict(play=False, result=None, trail=None, skeleton=None, renders=0, checks=0)

    def pause():
        state['play'] = False
        timer.stop()
        play.setText('재생')

    def draw(index):
        r = state['result']
        if r is None:
            return
        pairs, links = r.get('mesh_checks', [(set(), set())]*len(r['samples']))[index]
        involved = {name for pair in pairs for name in pair}
        if model:
            frames = transforms(np.radians(r['samples'][index]))
            for actor, matrix, (link, name, _, _) in zip(actors, matrices, model.meshes):
                t = frames[link].copy()
                t[:3, 3] *= 1000
                matrix.DeepCopy(t.ravel())
                color = (1, .5, 0) if name in links and name in involved else (
                    (1, .1, .1) if name in links else (1, 0, 1) if name in involved else
                    (.35+.055*link, .50+.045*link, .62+.035*link))
                actor.GetProperty().SetColor(*color)
        else:
            if state['skeleton']:
                renderer.RemoveActor(state['skeleton'])
            state['skeleton'] = poly_actor(r['frames'][index][0])
            renderer.AddActor(state['skeleton'])
        tcp_actor.SetPosition(*r['tcp'][index])
        tcp_actor.GetProperty().SetColor(*((1, 0, 0) if r['tcp_status'][index] else (0, .8, .2)))
        tcp_state = ['비침범 확인', '침범', '미확정 — 차단'][r['tcp_status'][index]]
        status.setText(
            f"{route.currentText()} / {sequence.currentText()}\n표본 {index+1}/{len(r['samples'])}\n\n"
            f"현재 구역 충돌: {', '.join(sorted(links)) or '없음'}\n"
            f"현재 자기충돌: {'; '.join('/'.join(p) for p in sorted(pairs)) or '없음'}\n"
            f"TCP 구간: {tcp_state}\n\n"
            f"전체 구역/TCP 차단: {sum(r['hit'])}\n"
            f"전체 자기충돌: {sum(r.get('self_hit', [])) if model else '검사 안 함'}\n"
            f"관절 제한: {', '.join(r['errors']) or '통과'}\n\n"
            f"TCP 경로: {r['length']:.0f}mm\n"
            +("팔+TCP 범위 X/Y/Z (mm):\n"
              + ' / '.join(f'{v:.0f}' for v in r['envelope']['span_mm'])+'\n'
              if 'envelope' in r else '')
            +
            f"팔꿈치 굽힘(0°=펴짐): {r['minimum_elbow_bend']:.1f}° 이상\n"
            f"J3 ±15° 구간의 J1 회전: {r['near_straight_turn']:.1f}°\n"
            '현재 J1~J6:\n'+', '.join(f'{v:+.1f}' for v in r['samples'][index])
            +f"\n\n검사 간격: {args.step_deg:g}°\n메시 표본 사이 충돌·실물 안전은 보증하지 않음")
        render_window.Render()
        state['renders'] += 1

    def update():
        pause()
        status.setText('전체 경로 검사 중…')
        panel.setEnabled(False)
        app.processEvents()
        try:
            q = list(via) if edit.text() == default_text else [float(v) for v in edit.text().split(',')]
            selected = [list(v) for v in knots]
            selected[args.via_index+1] = q
            if route.currentIndex():
                selected = baseline_knots
            if sequence.currentIndex():
                selected = Z.split_j3_first(selected)
            if not route.currentIndex() and getattr(args, 'blend_fraction', 0):
                selected = Z.blend_knots(selected, args.blend_fraction, min(args.step_deg, .25))
            cached = getattr(args, 'precomputed', None)
            r = cached if cached is not None else Z.inspect(
                selected, padding.value(), args.step_deg, motion, extra_boxes)
            r['minimum_elbow_bend'] = Z.minimum_elbow_bend(selected)
            r['near_straight_turn'] = Z.j1_turn_near_straight(selected)
            if (getattr(args, 'keep_elbow_bent', False)
                    and r['minimum_elbow_bend'] < abs(selected[0][2])-1e-6):
                r['errors'].append('시작보다 팔꿈치 펴짐 — 후보 제외')
            if model:
                if cached is None:
                    Z.inspect_mesh(model, r, padding.value(), extra_boxes)
                r['envelope'] = Z.envelope(model, r['samples'])
            state['checks'] += 1
        except (ValueError, OverflowError) as exc:
            status.setText(f'입력 오류: {exc}\n이전 경로 화면을 유지합니다.')
            for widget in [play, first, slider]:
                widget.setEnabled(state['result'] is not None)
            return
        finally:
            panel.setEnabled(True)
        state['result'] = r
        for widget in [play, first, slider]:
            widget.setEnabled(True)
        if state['trail']:
            renderer.RemoveActor(state['trail'])
        state['trail'] = poly_actor(r['tcp'])
        state['trail'].GetProperty().SetColor(.05, .45, .7)
        state['trail'].GetProperty().SetLineWidth(2)
        renderer.AddActor(state['trail'])
        slider.blockSignals(True)
        slider.setRange(0, len(r['samples'])-1)
        slider.setValue(0)
        slider.blockSignals(False)
        draw(0)
        print(f"VTK: samples={len(r['samples'])}, blocked={sum(r['hit'])}, "
              f"self={sum(r.get('self_hit', [])) if model else 'unchecked'}", flush=True)

    def tick():
        if state['play']:
            size = len(state['result']['samples'])
            slider.setValue((slider.value()+max(1, size//120)) % size)
            timer.start()  # Render가 끝난 뒤 예약해 UI 이벤트가 밀리지 않게 한다.

    def toggle_play():
        if state['play']:
            pause()
        else:
            state['play'] = True
            play.setText('일시정지')
            timer.start()

    def first_hit():
        pause()
        r = state['result']
        found = next((i for i, blocked in enumerate(r['hit'])
                      if blocked or r.get('self_hit', [False]*len(r['hit']))[i]), None)
        if found is not None:
            slider.setValue(found)

    def fit_scene():
        r = state['result']
        if r is not None and 'envelope' in r:
            lo = np.minimum(r['envelope']['low_mm'], Z.LOW)
            hi = np.maximum(r['envelope']['high_mm'], Z.HIGH)
            for low, high in visible_boxes[1:]:
                lo, hi = np.minimum(lo, low), np.maximum(hi, high)
            bounds = np.column_stack((lo, hi)).ravel()
            renderer.ResetCamera(*bounds)
            renderer.ResetCameraClippingRange(*bounds)
        else:
            renderer.ResetCamera()
        render_window.Render()

    slider.valueChanged.connect(draw)
    play.clicked.connect(toggle_play)
    first.clicked.connect(first_hit)
    fit.clicked.connect(fit_scene)
    apply_button.clicked.connect(update)
    edit.returnPressed.connect(update)
    route.currentIndexChanged.connect(update)
    sequence.currentIndexChanged.connect(update)
    timer.timeout.connect(tick)
    window.show()
    view.Initialize()
    camera = renderer.GetActiveCamera()
    camera.SetPosition(1500, -1600, 1200)
    camera.SetFocalPoint(300, -100, 300)
    camera.SetViewUp(0, 0, 1)
    update()
    fit_scene()
    if getattr(args, 'readonly', False):
        for widget in [route, sequence, edit, padding, apply_button]:
            widget.setEnabled(False)
    if args.first_hit:
        first_hit()

    def finish():
        pause()
        view.Finalize()

    app.aboutToQuit.connect(finish)
    if args.ui_check:
        from zone_motion_ui_check import check_window
        check_window(app, window, state, slider, play, route, sequence, actors, render_window)
        window.close()
        finish()
    elif args.snapshot:
        if not args.first_hit:
            slider.setValue(slider.maximum()//2)
        app.processEvents()
        pixels = framebuffer(render_window)
        height, width, _ = pixels.shape
        image = QtGui.QImage(pixels.data, width, height, pixels.strides[0], QtGui.QImage.Format_RGB888)
        shot = window.grab()
        painter = QtGui.QPainter(shot)
        painter.drawImage(view.geometry(), image)
        painter.end()
        if not shot.save(str(args.snapshot)):
            raise RuntimeError('화면 PNG 저장 실패')
        window.close()
        finish()
    else:
        app.exec_()
