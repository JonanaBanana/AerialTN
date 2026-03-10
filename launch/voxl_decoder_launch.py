from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aerial_tn',
            executable='voxl_decoder',
            name='decoder_low_light_down',
            parameters=[{
                'input_topic': 'low_light_down_misp_encoded',
                'output_topic': 'low_light_down_misp_decoded',
                'frame_id': 'low_light_down',
                'encoding': 'hevc' #h265
            }]
            ),
        Node(
            package='aerial_tn',
            executable='voxl_decoder',
            name='decoder_hires_front',
            parameters=[{
                'input_topic': 'hires_front_small_encoded',
                'output_topic': 'hires_front_small_decoded',
                'frame_id': 'hires_front',
                'encoding': 'hevc' #h265
            }]
            ),
        Node(
            package='aerial_tn',
            executable='voxl_decoder',
            name='decoder_tracking_down',
            parameters=[{
                'input_topic': 'tracking_down_misp_encoded',
                'output_topic': 'tracking_down_misp_decoded',
                'frame_id': 'tracking_down',
                'encoding': 'hevc' #h265
            }]
            ),
        Node(
            package='aerial_tn',
            executable='voxl_decoder',
            name='decoder_tracking_front',
            parameters=[{
                'input_topic': 'tracking_front_misp_encoded',
                'output_topic': 'tracking_front_misp_decoded',
                'frame_id': 'tracking_front',
                'encoding': 'hevc' #h265
            }]
            )
    ])
    