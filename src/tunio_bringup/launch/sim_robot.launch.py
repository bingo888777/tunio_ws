"""
2단계 · 마이크만 진짜 — 사람이 직접 줄을 튕겨 측정을 검증한다.

USB 마이크로 실제 소리를 재고, 실제 판정 로직이 돈다. 로봇만 가짜다.
로봇 동작(pluck_string, strum_all)이 아직 구현 전이라 이 단계가 필요하다.

  진짜: audio_analyzer · task_manager · result_store · web_node
  가짜: fake_hardware (serve_measure=false — 로봇 자리만 대신한다)

**사람이 할 일**: 화면에서 시작을 누르면 로봇이 튕긴 척하고 바로 마이크가 듣기
시작한다. 그때 해당 줄을 손으로 튕기면 된다. 어느 줄인지는 화면 기타 그림에 뜬다.
onset 제한 시간(기본 10초) 안에 튕기지 않으면 '소리 부족'으로 처리된다.

페그도 사람이 돌린다. 화면에 조절 지시가 뜨면 그 방향으로 돌려 주면 되는데,
가짜 로봇은 각도만 받고 흘려보내므로 **얼마나 돌릴지는 사람이 감으로 한다.**
이 단계의 목적은 페그 정확도가 아니라 **측정이 맞는지**다.

⚠ 캘리브레이션은 sim 파일을 쓴다. 실측값이 아니라 지어낸 값이지만, 없으면
  task_manager 가 조절 단계에서 PHASE_ERROR 로 멈춰 측정 검증을 이어갈 수 없다.

  ros2 launch tunio_bringup sim_robot.launch.py

옵션
  device:=2       마이크 장치 번호. -1 이면 시스템 기본 입력
                  (tools/mic_level.py 로 어느 장치가 소리를 받는지 먼저 확인할 것)
  port:=9000      웹 포트
  db:=<경로>      SQLite 파일 위치

⚠ OS 의 마이크 향상 기능(노이즈 제거·자동 음량·에코 제거)을 반드시 끌 것 (기획 2.6).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tunio_server')
    sim_cal = os.path.join(share, 'config', 'peg_calibration.sim.yaml')

    device = LaunchConfiguration('device')
    port = LaunchConfiguration('port')
    db = LaunchConfiguration('db')

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='-1',
                              description='마이크 장치 번호. -1 은 시스템 기본 입력'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('db', default_value=os.path.expanduser('~/.tunio/tunio.db')),

        LogInfo(msg='[sim_robot] 마이크는 진짜다. 화면에 뜬 줄을 사람이 직접 튕겨라.'),
        LogInfo(msg='[sim_robot] OS 의 마이크 향상 기능(노이즈 제거·자동 음량)을 껐는지 확인할 것.'),

        Node(package='tunio_audio', executable='audio_analyzer', name='audio_analyzer',
             output='screen', parameters=[{'device': device}]),

        # 로봇 자리만 대신한다. /audio/measure_pitch 는 진짜 audio_analyzer 가 받는다.
        Node(package='tunio_server', executable='fake_hardware', name='fake_hardware',
             output='screen',
             parameters=[{'serve_measure': False, 'serve_robot': True,
                          'play_delay_sec': 1.0, 'adjust_delay_sec': 2.0,
                          'strum_delay_sec': 1.0}]),

        Node(package='tunio_server', executable='task_manager', name='task_manager',
             output='screen', parameters=[{'peg_calibration_file': sim_cal}]),
        Node(package='tunio_server', executable='result_store', name='result_store',
             output='screen', parameters=[{'db_path': db}]),
        Node(package='tunio_server', executable='web_node', name='web_node',
             output='screen', parameters=[{'port': port}]),
    ])
