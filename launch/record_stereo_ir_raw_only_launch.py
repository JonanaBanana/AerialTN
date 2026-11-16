from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bag_name = 'voxlbag_raw_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join('/bagfiles', bag_name)

    topics = [
        '/ircam_narrow/raw_image',
        '/ircam_wide/raw_image',
        '/imu_apps',
        '/fmu/out/vehicle_odometry',
        '/vvhub_aligned_vio',
        '/alt_raw',
        '/alt_smoothed',
        '/px4/baro/altitude',
        '/px4/baro/pressure',
        '/px4/baro/temperature',
        '/px4/mag'
    ]

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--output', output_dir,
                '--max-bag-size', '300000000', # 300 MB splits
            ] + topics,
            output='screen',
            shell=False,
        )
    ])
