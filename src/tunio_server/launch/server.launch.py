from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='tunio_server', executable='task_manager', name='task_manager', output='screen'),
        Node(package='tunio_server', executable='result_store', name='result_store', output='screen'),
        Node(package='tunio_server', executable='hmi_node', name='hmi_node', output='screen'),
    ])
