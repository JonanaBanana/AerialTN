from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from datetime import datetime


## MCAP compression allows for similar compression as standard ros2 bag zstd compression, but the bagfiles can be played as is and requires no mamual uncompression.
def generate_launch_description():

    # Timestamped output folder
    bag_name = 'voxlbag_mcap_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join('/bagfiles', bag_name)

    topics = [
        '/ircam',
        '/low_light_down_gray',
        '/tracking_down',
        '/tracking_front',
        '/hires_front_small_grey',
        '/imu_apps',
        '/fmu/out/vehicle_gps_position',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_odometry'
    ]

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--output', output_dir,
                '--storage', 'mcap',
                '--storage-preset-profile', 'zstd_fast'
            ] + topics,
            output='screen',
            shell=False,
        )
    ])