from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bag_name = 'voxlbag_encoded_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join('/bagfiles', bag_name)

    qos_override = os.path.join(
        get_package_share_directory('aerial_tn'),
        'config', 'qos_override_full_monty.yaml')

    topics = [
        '/low_light_down_misp_encoded',
        '/tracking_down_misp_encoded',
        '/tracking_front_misp_encoded',
        '/hires_front_misp_encoded',
        '/ircam/h264',
        '/imu_apps',
        '/fmu/out/vehicle_gps_position',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_odometry',
    ]

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--output', output_dir,
                '--max-bag-size', '5000000000',
                '--qos-profile-overrides-path', qos_override,
            ] + topics,
            output='screen',
            shell=False,
        )
    ])