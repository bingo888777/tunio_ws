"""
정정인 PC 런치 — 마이크 + 서버 + 관제 웹. PC 2대 구성의 한쪽이다.

2026-09-22 에 PC 3대에서 2대로 바뀌었다. 마이크 PC 와 서버 PC 가 한 대로
합쳐졌고, 로봇 제어 PC 만 따로 남는다. 노드도 인터페이스도 안 바뀌었다 —
**어느 PC 에서 도는가만 바뀌었다.** 그래서 이 파일은 기존 server.launch.py 와
audio.launch.py 를 그대로 include 한다. 노드 정의를 복사하지 않는 이유는,
복사하면 원본이 바뀔 때 여기만 옛날 설정으로 남기 때문이다.

  # 이 PC (정정인)
  ros2 launch tunio_bringup station.launch.py

  # 다른 PC (로봇 제어)
  ros2 launch tunio_robot robot.launch.py dry_run:=false

두 PC 는 같은 ROS_DOMAIN_ID 를 봐야 서로 보인다. 강의실 Wi-Fi 가 멀티캐스트를
막으면 Fast DDS Discovery Server 를 쓴다 — 서버는 **이 PC** 에서 띄운다
(예전에는 서버 PC 였다).

  # 이 PC
  fastdds discovery -i 0 -l 0.0.0.0 -p 11811
  export ROS_DISCOVERY_SERVER=127.0.0.1:11811
  # 로봇 PC
  export ROS_DISCOVERY_SERVER=<이 PC 의 IP>:11811

옵션
  device:=2        마이크 장치 번호 (기본 -1 = OS 기본 입력)
  port:=9000       관제 웹 포트
  open_browser:=false
  db:=<경로>       SQLite 파일 위치

⚠ 로봇은 여기서 안 띄운다. 로봇 제어 PC 에서 따로 띄운다.
  한 PC 에 전부 물려 시험하려면 real_all.launch.py 를 쓴다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')
    port = LaunchConfiguration('port')
    open_browser = LaunchConfiguration('open_browser')
    db = LaunchConfiguration('db')

    server_launch = os.path.join(
        get_package_share_directory('tunio_server'), 'launch', 'server.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='-1',
                              description='마이크 장치 번호. -1 이면 OS 기본 입력'),
        DeclareLaunchArgument('port', default_value='8080',
                              description='관제 웹 포트. http://localhost:<port>'),
        DeclareLaunchArgument('open_browser', default_value='true'),
        DeclareLaunchArgument('db', default_value=os.path.expanduser('~/.tunio/tunio.db')),

        LogInfo(msg='[station] 마이크 + 서버 + 관제 웹. 로봇은 로봇 제어 PC 에서 따로 띄운다.'),
        LogInfo(msg=['[station] 관제 웹 http://localhost:', port]),

        # server.launch.py 를 그대로 쓴다 (task_manager · result_store · web_node)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(server_launch),
            launch_arguments={'port': port, 'open_browser': open_browser,
                              'db': db}.items()),

        # audio.launch.py 는 인자를 안 받아서(device 고정) 여기서 직접 띄운다.
        # 장치 번호를 실기에서 바꿔 가며 봐야 하기 때문이다.
        Node(package='tunio_audio', executable='audio_analyzer', name='audio_analyzer',
             output='screen', parameters=[{'device': device}]),
    ])
