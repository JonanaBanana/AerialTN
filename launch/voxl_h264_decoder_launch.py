from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import tempfile
import yaml
from datetime import datetime

def generate_launch_description():

    # Timestamped output folder
    bag_name = 'voxlbag_encoded_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join('/bagfiles', bag_name)

    # Encoded camera topics that require TRANSIENT_LOCAL to guarantee the
    # SPS/PPS parameter set packet is captured at the start of the bag.
    # Without this, ros2 bag record uses VOLATILE and misses the SPS/PPS
    # if recording starts after voxl-mpa-to-ros2, making the bag undecodable.
    encoded_topics = [
        '/low_light_down_misp_encoded',
        '/tracking_down_misp_encoded',
        '/tracking_front_misp_encoded',
    ]

    other_topics = [
        '/imu_apps',
        '/fmu/out/vehicle_gps_position',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_odometry',
    ]

    # Build QoS override yaml for the encoded topics only.
    # Other topics use the default QoS so we don't override them.
    qos_overrides = {
        topic: {
            'reliability':  'reliable',
            'durability':   'transient_local',
            'history':      'keep_last',
            'depth':        10,
        }
        for topic in encoded_topics
    }

    # Write to a temp file — it persists for the lifetime of the process.
    qos_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False)
    yaml.dump(qos_overrides, qos_file)
    qos_file.flush()

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--output', output_dir,
                '--max-bag-size', '5000000000',
                '--qos-profile-overrides-path', qos_file.name,
            ] + encoded_topics + other_topics,
            output='screen',
            shell=False,
        )
    ])