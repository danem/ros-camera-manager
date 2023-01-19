from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_camera_manager',
            namespace='camera_manager',
            executable='server.py',
            name='camera_manager_server'
        )
    ])