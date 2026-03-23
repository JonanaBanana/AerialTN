from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

# We do not need to decode ircam here, as it is available from the v4l2 publisher
# as a composed node. For offboard decoding, use AerialTN_Utility.
def generate_launch_description():
    pkg_dir = get_package_share_directory('aerial_tn')

    return LaunchDescription([
        Node(
            package='aerial_tn',
            executable='voxl_h264_decoder',
            name='decoder_low_light_down_node',
            parameters=[{
                'input_topic': '/low_light_down_misp_encoded',
                'output_topic': '/low_light_down_misp_decoded',
                'frame_id': 'low_light_down',
                'live_stream': False,   #enable only when streaming over wifi, 
                                        #as it will provide additional checks to increase stream stability
                'convert_to_bgr': False #enable only when you want to visualize in rviz. 
                                        #enabling doubles computation per frame, but allows bgr8 output instead of yuv420p.
            }]
            ),
        Node(
            package='aerial_tn',
            executable='voxl_h264_decoder',
            name='decoder_tracking_down_node',
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
            executable='voxl_h264_decoder',
            name='decoder_tracking_front_node',
            parameters=[{
                'input_topic': '/tracking_front_misp_encoded',
                'output_topic': '/tracking_front_misp_decoded',
                'frame_id': 'tracking_front',
                'live_stream': False, 
                'convert_to_bgr': False
            }]
            )
    ])
    