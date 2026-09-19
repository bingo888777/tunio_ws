"""
서버 PC 런치 — task_manager + result_store + 관제 웹(web_node).

UI 는 관제 웹이다. hmi_node(PyQt) 는 web_node 로 대체됐으므로 여기서 띄우지 않는다.
(파일은 아직 남겨 뒀다. 화면 확인이 끝나면 정리한다.)

  ros2 launch tunio_server server.launch.py                 # 기본: 포트 8080, 브라우저 자동 실행
  ros2 launch tunio_server server.launch.py port:=9000
  ros2 launch tunio_server server.launch.py open_browser:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    open_browser = LaunchConfiguration('open_browser')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080',
                              description='관제 웹 포트. http://localhost:<port>'),
        DeclareLaunchArgument('open_browser', default_value='true',
                              description='노드 시작 시 브라우저를 자동으로 열지'),

        Node(package='tunio_server', executable='task_manager', name='task_manager',
             output='screen'),
        Node(package='tunio_server', executable='result_store', name='result_store',
             output='screen'),
        Node(package='tunio_server', executable='web_node', name='web_node',
             output='screen',
             parameters=[{'port': port, 'open_browser': open_browser}]),
    ])
