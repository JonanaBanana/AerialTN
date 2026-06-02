# AerialTN

ROS2 package for drone-specific code. Currently provides an H.264 decoder node for encoded video streams from VOXL cameras, usable both live and from a ROS2 bag.

**Platform:** Ubuntu 22.04 / ROS2 Humble

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select aerial_tn
source install/setup.bash
```

## H.264 Decoder

Decodes H.264-encoded image topics (`sensor_msgs/CompressedImage`) into raw frames (`sensor_msgs/Image`).

**Node:** `voxl_h264_decoder`

| Parameter | Type | Default | Description |
|---|---|---|---|
| `input_topic` | string | — | Encoded input topic |
| `output_topic` | string | — | Decoded output topic |
| `frame_id` | string | — | Frame ID for output messages |
| `live_stream` | bool | `false` | Enable extra stability checks for live WiFi streams |
| `convert_to_bgr` | bool | `false` | Output `bgr8` instead of `yuv420p` (doubles CPU cost, needed for RViz) |

### Launch

The provided launch file decodes three camera streams (low_light_down, tracking_down, tracking_front):

```bash
ros2 launch aerial_tn decoder_launch.py
```

To decode a single stream directly:

```bash
ros2 run aerial_tn voxl_h264_decoder --ros-args \
  -p input_topic:=/your/encoded/topic \
  -p output_topic:=/your/decoded/topic \
  -p frame_id:=camera \
  -p live_stream:=false \
  -p convert_to_bgr:=false
```
