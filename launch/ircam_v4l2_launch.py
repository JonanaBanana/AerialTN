from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[{
                'video_device': '/dev/video2',
                'image_size': [640, 512],
                'pixel_format': 'YUYV',
            }],
            remappings=[
                ('image_raw', 'ircam'),
                ('image_raw/ffmpeg', 'ircam/ffmpeg'),
            ],
        )
    ])