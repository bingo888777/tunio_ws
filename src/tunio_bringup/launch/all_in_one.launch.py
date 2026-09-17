from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    def inc(pkg, f):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), "launch", f)))
    # 단일 PC 테스트용. 실제로는 PC별로 각 런치를 따로 실행.
    return LaunchDescription([
        inc("tunio_server", "server.launch.py"),
        inc("tunio_robot", "robot.launch.py"),
        inc("tunio_audio", "audio.launch.py"),
    ])
