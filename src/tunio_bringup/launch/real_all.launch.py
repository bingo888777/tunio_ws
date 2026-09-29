"""
3단계 · 전부 진짜 — 최종 산출물.

가짜 노드가 하나도 없다. 실제 로봇이 움직이고 실제 마이크가 듣는다.

  진짜: robot_controller · audio_analyzer · task_manager · result_store · web_node

**시작 전에 반드시 확인할 것**

1. 페그 캘리브레이션 실측이 끝났는가
   config/peg_calibration.yaml 의 measured 가 true 여야 한다. false 면 조절이
   필요해지는 순간 PHASE_ERROR 로 멈춘다. 실측 전에 지어낸 각도로 페그를 돌리지
   않으려고 일부러 막아 둔 것이다. 각 값의 뜻과 형식은 그 yaml 파일 주석에 있고,
   재는 방법은 docs/주파수분석_테스트가이드.md(마이크·분석 담당) 를 따른다.

2. 교시가 끝났는가
   tunio_robot 의 config/teach_points.yaml 에서 쓰려는 자세들의 taught 가 true 여야
   한다. 피크 거치대(pick.holder), 페그 4개, 발현 자세 4개, frames.standby 가 필요하다.

3. ⚠ gripper 인수는 지금 아무 일도 하지 않는다 (2026-09-22 확인)
   robot_controller 가 'gripper' 파라미터를 선언하지도 읽지도 않는다. 그리퍼
   한계값·프리셋은 teach_points.yaml 의 gripper 절에서 온다. 예전에 여기 적혀
   있던 '반드시 rg2 를 준다, 기본값 rg6 는 위험하다'는 **틀린 안내였다** —
   준 적이 없어도 rg6 로 돌지 않는다. 인수는 남겨 두되 믿지 말 것.
   확인할 것은 teach_points.yaml 의 gripper.presets 가 웹에서 설정한 값과
   같은지다 (CLAUDE.md 3.6).

4. 마이크 향상 기능을 껐는가
   OS 의 노이즈 제거·자동 음량·에코 제거를 전부 끈다 (기획 2.6).

  ros2 launch tunio_bringup real_all.launch.py

⚠ dry_run 기본값이 true 다. 로봇을 실제로 움직이려면 dry_run:=false 를 줘야 한다.
  실수로 켜자마자 움직이는 것보다 안 움직이는 쪽이 안전하기 때문이다.

⚠ 2 PC 분산이 본래 구성이다 (2026-09-22 변경, 이전에는 3대). 이 파일은 한 PC 에
  다 물렸을 때 쓴다. 분산할 때는 —
    정정인 PC     ros2 launch tunio_bringup station.launch.py   (마이크+서버+웹)
    로봇 제어 PC  ros2 launch tunio_robot robot.launch.py
  ROS_DOMAIN_ID 를 맞추는 것을 잊지 말 것. docs/TUNIO_시스템아키텍처_v3.md 참조.

옵션
  dry_run:=false  실제로 로봇을 움직인다 (기본 true — 움직이지 않는다)
  gripper:=rg2    그리퍼 종류. 반드시 rg2
  device:=2       마이크 장치 번호
  speed_scale:=1.0          이동 속도 배율. 0 이면 teach_points.yaml 값
  peg_register_force:=true  페그 진입을 힘 제어로 (기본 false, #51)
  verify_pick:=true         집은 뒤 폭으로 피크 확인 (기본 false, 실기 검증 전 #88)
  port:=9000      웹 포트
  db:=<경로>      SQLite 파일 위치
  save_on_error_exit:=true
                  중간에 끊긴 작업(PHASE_ERROR)도 DB 에 저장한다. 기본 false.
                  **true/false 로 적는다. 1/0 은 안 된다** — launch 가 INTEGER 로
                  추론해서 노드가 InvalidParameterTypeException 으로 죽는다.
                  true/True/yes 는 되고 1 만 안 된다.
                  실측 회차에서 원인을 보려고 켜는 것이다. 시연에서는 켜지 말 것 —
                  '중간에 끊긴 작업은 저장하지 않는다'가 확정 사항이다 (CLAUDE.md 13절).
                  켜면 처리 못 한 줄이 RESULT_PENDING 이라 전체 판정이
                  OVERALL_PENDING('판정 전')으로 들어간다
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
    speed_scale = LaunchConfiguration('speed_scale')
    peg_register_force = LaunchConfiguration('peg_register_force')
    verify_pick = LaunchConfiguration('verify_pick')
    port = LaunchConfiguration('port')
    db = LaunchConfiguration('db')
    save_on_error = LaunchConfiguration('save_on_error_exit')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='true 면 로봇을 움직이지 않고 좌표 조회까지만 한다'),
        DeclareLaunchArgument('gripper', default_value='rg2',
                              description='반드시 rg2. 기본값 rg6 는 한계값이 달라 위험하다'),
        DeclareLaunchArgument('device', default_value='-1'),
        # 실기에서 속도를 바꿔 가며 보려고 뺀다. 0 이면 teach_points.yaml 의
        # motion.speed_scale을 그대로 쓴다. 1.0 을 주면 예전 속도다
        DeclareLaunchArgument('speed_scale', default_value='0.0'),
        # 페그 진입을 힘 제어로 할지. 기본 false — 실물 검증이 안 끝났다 (#51)
        DeclareLaunchArgument('peg_register_force', default_value='false'),
        DeclareLaunchArgument('verify_pick', default_value='false'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('db', default_value=os.path.expanduser('~/.tunio/tunio.db')),
        DeclareLaunchArgument(
            'save_on_error_exit', default_value='false',
            description='끊긴 작업도 저장한다. 실측 회차용. 시연에서는 false'),

        LogInfo(msg='[real_all] 가짜 노드가 없다. 페그 실측과 교시가 끝났는지 확인했는가.'),
        LogInfo(msg=['[real_all] dry_run=', dry_run,
                     ' (true 면 로봇은 움직이지 않는다)']),

        Node(package='tunio_robot', executable='robot_controller', name='robot_controller',
             output='screen',
             parameters=[{'dry_run': dry_run, 'gripper': gripper,
                          'speed_scale': speed_scale,
                          'peg_register_force': peg_register_force,
                          'verify_pick': verify_pick}]),
        Node(package='tunio_audio', executable='audio_analyzer', name='audio_analyzer',
             output='screen', parameters=[{'device': device}]),

        # 캘리브레이션 파일을 지정하지 않는다. 기본값인 peg_calibration.yaml 을 읽고,
        # measured 가 false 면 조절 단계에서 멈춘다. 그게 의도한 동작이다.
        Node(package='tunio_server', executable='task_manager', name='task_manager',
             output='screen',
             parameters=[{'save_on_error_exit': save_on_error}]),
        Node(package='tunio_server', executable='result_store', name='result_store',
             output='screen', parameters=[{'db_path': db}]),
        Node(package='tunio_server', executable='web_node', name='web_node',
             output='screen', parameters=[{'port': port}]),
    ])
