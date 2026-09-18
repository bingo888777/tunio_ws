from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='tunio_audio', executable='audio_analyzer',
             name='audio_analyzer', output='screen'),
    ])
