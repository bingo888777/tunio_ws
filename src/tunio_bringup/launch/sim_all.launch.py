"""
1단계 · 전부 가짜 — 웹 화면과 서버 흐름 확인용.

로봇도 마이크도 없이 조율 전 과정이 돈다. 페그를 돌리면 음정이 실제로 움직여
조절 횟수가 늘고 오차가 줄어드는 것을 화면에서 볼 수 있다.

  진짜: task_manager · result_store · web_node
  가짜: fake_hardware (mode=sim 이 로봇과 마이크를 모두 대신한다)

⚠ 여기 쓰는 각도-센트 상수는 실측값이 아니라 지어낸 숫자다
  (config/peg_calibration.sim.yaml). 수렴한다고 해서 실제로 수렴한다는 뜻이
  아니다. 화면이 제대로 그려지는지만 본다.

  ros2 launch tunio_bringup sim_all.launch.py

옵션
  speed:=fast     지연을 줄여 빠르게 돌린다 (결과만 볼 때)
  port:=9000      웹 포트
  db:=<경로>      SQLite 파일 위치
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tunio_server')
    sim_cal = os.path.join(share, 'config', 'peg_calibration.sim.yaml')

    speed = LaunchConfiguration('speed')
    port = LaunchConfiguration('port')
    db = LaunchConfiguration('db')
    is_fast = PythonExpression(["'", speed, "' == 'fast'"])

    # 실제 동작에 가까운 지연. measure 2.2초만 근거가 있고(기획 2.3) 나머지는 추정이다.
    real_delays = {
        'ready_delay_sec': 0.3, 'measure_delay_sec': 2.2,
        'play_delay_sec': 3.0, 'adjust_delay_sec': 4.0, 'strum_delay_sec': 3.0,
    }
    fast_delays = {k: 0.05 for k in real_delays}

    def fake(delays, cond):
        return Node(
            package='tunio_server', executable='fake_hardware', name='fake_hardware',
            output='screen', condition=cond,
            parameters=[dict(mode='sim', serve_measure=True, serve_robot=True, **delays)],
        )

    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='real',
                              description='real | fast — fast 는 지연을 없애 빠르게 돌린다'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('db', default_value=os.path.expanduser('~/.tunio/tunio.db')),

        LogInfo(msg='[sim_all] 전부 가짜다. 여기서 수렴하는 것은 조율 성능의 근거가 아니다.'),

        fake(real_delays, UnlessCondition(is_fast)),
        fake(fast_delays, IfCondition(is_fast)),

        Node(package='tunio_server', executable='task_manager', name='task_manager',
             output='screen', parameters=[{'peg_calibration_file': sim_cal}]),
        Node(package='tunio_server', executable='result_store', name='result_store',
             output='screen', parameters=[{'db_path': db}]),
        Node(package='tunio_server', executable='web_node', name='web_node',
             output='screen', parameters=[{'port': port}]),
    ])
