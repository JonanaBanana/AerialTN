import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('aerial_tn')
    default_params = os.path.join(pkg_share, 'config', 'ircam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('device', default_value='/dev/video3'),
        DeclareLaunchArgument('encoding', default_value='yuyv'),
        DeclareLaunchArgument('v4l2_buffers', default_value='8'),

        Node(
            package='aerial_tn',
            executable='ircam_node',
            name='ir_v4l2_camera',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'device': LaunchConfiguration('device'),
                    'encoding': LaunchConfiguration('encoding'),
                    'v4l2_buffers': LaunchConfiguration('v4l2_buffers'),
                },
            ],
        ),
    ])