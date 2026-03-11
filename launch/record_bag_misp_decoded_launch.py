from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from datetime import datetime

def generate_launch_description():

    # Timestamped output folder
    bag_name = 'voxlbag_decoded_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join('/bagfiles', bag_name)

    topics = [
        '/low_light_down_misp_decoded',
        '/tracking_down_misp_decoded',
        '/tracking_front_misp_decoded',
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
                '--max-bag-size', '500000000',   # 5GB bag size (splits bag when exceeding)
            ] + topics,
            output='screen',
            shell=False,
        )
    ])