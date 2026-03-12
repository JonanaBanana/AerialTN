from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aerial_tn',
            executable='voxl_h265_decoder',
            name='decoder_low_light_down',
            parameters=[{
                'input_topic': '/low_light_down_misp_encoded',
                'output_topic': '/low_light_down_misp_decoded',
                'frame_id': 'low_light_down',
                'live_stream': False,   #enable only when streaming over wifi, 
                                        #as it will provide additional checks to increase stream stability
                'convert_to_bgr': False #enable only when you want to visualize in rviz. 
                                        #enabling doubles computation per frame, but allows bgr8 output instead of yuv420p.e
            }]
            ),
        Node(
            package='aerial_tn',
            executable='voxl_h265_decoder',
            name='decoder_tracking_down',
            parameters=[{
                'input_topic': '/tracking_down_misp_encoded',
                'output_topic': '/tracking_down_misp_decoded',
                'frame_id': 'tracking_down',
                'live_stream': False, 
                'convert_to_bgr': False
            }]
            ),
        Node(
            package='aerial_tn',
            executable='voxl_h265_decoder',
            name='decoder_tracking_front',
            parameters=[{
                'input_topic': '/tracking_front_misp_encoded',
                'output_topic': '/tracking_front_misp_decoded',
                'frame_id': 'tracking_front',
                'live_stream': False, 
                'convert_to_bgr': False
            }]
            )
    ])
    