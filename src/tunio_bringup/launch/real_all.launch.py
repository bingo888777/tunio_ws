"""
3단계 · 전부 진짜 — 최종 산출물.

가짜 노드가 하나도 없다. 실제 로봇이 움직이고 실제 마이크가 듣는다.

  진짜: robot_controller · audio_analyzer · task_manager · result_store · web_node

**시작 전에 반드시 확인할 것**

1. 페그 캘리브레이션 실측이 끝났는가
   config/peg_calibration.yaml 의 measured 가 true 여야 한다. false 면 조절이
   필요해지는 순간 PHASE_ERROR 로 멈춘다. 실측 전에 지어낸 각도로 페그를 돌리지
   않으려고 일부러 막아 둔 것이다. 값을 넣는 방법은 docs/페그_실측_가이드.md 참조.

2. 교시가 끝났는가
   tunio_robot 의 config/teach_points.yaml 에서 쓰려는 자세들의 taught 가 true 여야
   한다. 피크 거치대(pick.holder), 페그 4개, 발현 자세 4개, frames.standby 가 필요하다.

3. 그리퍼 인수를 줬는가
   gripper:=rg2 를 반드시 준다. 기본값이 rg6 라서 한계값이 120N·160mm 로 잡히고
   RG2 가 못 내는 값을 걸러 주지 못한다 (CLAUDE.md 3.6).

4. 마이크 향상 기능을 껐는가
   OS 의 노이즈 제거·자동 음량·에코 제거를 전부 끈다 (기획 2.6).

  ros2 launch tunio_bringup real_all.launch.py

⚠ dry_run 기본값이 true 다. 로봇을 실제로 움직이려면 dry_run:=false 를 줘야 한다.
  실수로 켜자마자 움직이는 것보다 안 움직이는 쪽이 안전하기 때문이다.

⚠ 3 PC 분산이 본래 구성이다. 이 파일은 한 PC 에 다 물렸을 때 쓴다.
  분산할 때는 각 PC 에서 server.launch.py / robot.launch.py / audio.launch.py 를
  따로 띄운다. ROS_DOMAIN_ID 를 맞추는 것을 잊지 말 것.

옵션
  dry_run:=false  실제로 로봇을 움직인다 (기본 true — 움직이지 않는다)
  gripper:=rg2    그리퍼 종류. 반드시 rg2
  device:=2       마이크 장치 번호
  port:=9000      웹 포트
  db:=<경로>      SQLite 파일 위치
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dry_run = LaunchConfiguration('dry_run')
    gripper = LaunchConfiguration('gripper')
    device = LaunchConfiguration('device')
    port = LaunchConfiguration('port')
    db = LaunchConfiguration('db')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='true 면 로봇을 움직이지 않고 좌표 조회까지만 한다'),
        DeclareLaunchArgument('gripper', default_value='rg2',
                              description='반드시 rg2. 기본값 rg6 는 한계값이 달라 위험하다'),
        DeclareLaunchArgument('device', default_value='-1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('db', default_value=os.path.expanduser('~/.tunio/tunio.db')),

        LogInfo(msg='[real_all] 가짜 노드가 없다. 페그 실측과 교시가 끝났는지 확인했는가.'),
        LogInfo(msg=['[real_all] dry_run=', dry_run,
                     ' (true 면 로봇은 움직이지 않는다)']),

        Node(package='tunio_robot', executable='robot_controller', name='robot_controller',
             output='screen',
             parameters=[{'dry_run': dry_run, 'gripper': gripper}]),
        Node(package='tunio_audio', executable='audio_analyzer', name='audio_analyzer',
             output='screen', parameters=[{'device': device}]),

        # 캘리브레이션 파일을 지정하지 않는다. 기본값인 peg_calibration.yaml 을 읽고,
        # measured 가 false 면 조절 단계에서 멈춘다. 그게 의도한 동작이다.
        Node(package='tunio_server', executable='task_manager', name='task_manager',
             output='screen'),
        Node(package='tunio_server', executable='result_store', name='result_store',
             output='screen', parameters=[{'db_path': db}]),
        Node(package='tunio_server', executable='web_node', name='web_node',
             output='screen', parameters=[{'port': port}]),
    ])
