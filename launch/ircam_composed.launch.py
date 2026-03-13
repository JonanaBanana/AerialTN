import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode



def generate_launch_description():
    #     pkg_share = get_package_share_directory('aerial_tn')
    #     default_params = os.path.join(pkg_share, 'config', 'ircam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('device',   default_value='/dev/video2'),
        DeclareLaunchArgument('encoding', default_value='yuyv'),

        ComposableNodeContainer(
            name='ircam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[

                ComposableNode(
                    package='aerial_tn',
                    plugin='ir_v4l2_camera::IrCameraComponent',
                    name='ircam',
                    parameters=[{
                        'device':       LaunchConfiguration('device'),
                        'encoding':     LaunchConfiguration('encoding'),
                        'width':        640,
                        'height':       512,
                        'fps':          30,
                        'v4l2_buffers': 8,
                        'frame_id':     'ir_camera',
                        'queue_depth':  2,
                    }],
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                    ],
                ),

                ComposableNode(
                    package='aerial_tn',
                    plugin='ir_v4l2_camera::IrCameraH264Component',
                    name='ircam_h264',
                    parameters=[{
                        'input_topic':    '/ircam/raw_image',
                        'output_topic':   '/ircam/h264',
                        'bitrate':        2000000,
                        'preset':         'ultrafast',
                    }],
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                    ],
                ),
            ],
        ),
    ])
