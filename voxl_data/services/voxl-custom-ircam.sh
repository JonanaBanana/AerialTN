#!/bin/bash
source /ros2_ws/install/setup.bash
exec ros2 launch ircam_driver stereo_ircam_composed.launch.py
