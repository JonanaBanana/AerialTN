from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lightwarelidar2',
            executable='sf30d',
            name='sf30d',
            parameters=[{
                'port': '/dev/ttyACM0',
                'frame_id': 'sf30d',
            }],
            output='screen',
        )
    ])
