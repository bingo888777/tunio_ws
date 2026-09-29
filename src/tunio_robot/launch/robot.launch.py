"""
로봇 제어 PC 런치 — robot_controller 하나.

PC 2대 구성에서 이 파일이 로봇 PC 의 진입점이다 (2026-09-22).
반대편은 `ros2 launch tunio_bringup station.launch.py` 다.

  ros2 launch tunio_robot robot.launch.py                    # 안 움직인다 (dry_run 기본 true)
  ros2 launch tunio_robot robot.launch.py dry_run:=false     # 실제로 움직인다

⚠ **dry_run 기본값이 true 다.** 실수로 켜자마자 움직이는 것보다 안 움직이는
  쪽이 안전하다. 실기에서는 반드시 dry_run:=false 를 준다.

이 런치를 띄우기 전에 두산 브링업이 먼저 떠 있어야 한다.

  ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100

옵션
  dry_run:=false            실제로 로봇을 움직인다 (기본 true)
  speed_scale:=1.0          이동 속도 배율. 0 이면 teach_points.yaml 값(2.0)
  peg_register_force:=true  페그 진입을 힘 제어로 (기본 false, #51)

ℹ 그리퍼 종류는 인자가 아니다. 한계값·프리셋은 teach_points.yaml 의 gripper 절에서
  온다. real_all.launch.py 의 gripper:=rg2 는 노드가 선언하지 않는 파라미터라
  아무 일도 하지 않는다 — 여기서는 아예 두지 않는다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dry_run = LaunchConfiguration('dry_run')
    speed_scale = LaunchConfiguration('speed_scale')
    peg_register_force = LaunchConfiguration('peg_register_force')
    verify_pick = LaunchConfiguration('verify_pick')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='true 면 로봇을 움직이지 않고 좌표 조회까지만 한다'),
        DeclareLaunchArgument('speed_scale', default_value='0.0',
                              description='이동 속도 배율. 0 이면 teach_points.yaml 값'),
        DeclareLaunchArgument('peg_register_force', default_value='false',
                              description='페그 진입을 힘 제어로 할지 (#51)'),
        DeclareLaunchArgument('verify_pick', default_value='false',
                              description='집은 뒤 Modbus 폭으로 피크를 확인할지. '
                                          '실기 검증 전이라 기본 false (#88 리뷰)'),

        LogInfo(msg=['[robot] dry_run=', dry_run,
                     ' (true 면 로봇은 움직이지 않는다)']),

        Node(package='tunio_robot', executable='robot_controller',
             name='robot_controller', output='screen',
             parameters=[{'dry_run': dry_run,
                          'speed_scale': speed_scale,
                          'peg_register_force': peg_register_force,
                          'verify_pick': verify_pick}]),
    ])
