# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS1 Noetic package integrating YOLOv8 object detection with camera inputs (OpenNI2 RGB-D, USB cameras). Runs in Docker with GPU acceleration.

## Build & Run Commands

### Docker (Primary Method)
```bash
# Build image
docker compose build

# Run container (launches yolo26.launch by default)
docker compose up

# Run with specific launch file
docker compose run --rm yolo26 /bin/bash -lc "roslaunch yolo26_ros1 xtion_yolo26.launch"
```

### Inside Container (Manual Build)
```bash
source /opt/ros/noetic/setup.bash
cd /catkin_ws
catkin_make
source devel/setup.bash
```

### Launch Files
- `yolo26.launch` - Default, expects `/camera/image_preprocessed` topic
- `xtion_yolo26.launch` - Includes OpenNI2 driver, uses `/camera/rgb/image_raw`
- `usbcam_yolo26.launch` - Launches USB camera node + YOLO

## Architecture

```
catkin_ws/src/yolo26_ros1/
├── scripts/yolo26_ros1_node.py  # Main ROS node
├── launch/                       # Launch configurations
├── package.xml                   # ROS package metadata
└── CMakeLists.txt               # Catkin build config
```

### Node: yolo26_ros1_node

Single ROS node that:
1. Subscribes to image topic (configurable)
2. Runs YOLO inference via Ultralytics
3. Publishes annotated images to `/yolo/image`

**ROS Parameters:**
- `~image` - Input topic (default: `/camera/rgb/image_raw`)
- `~out_image` - Output topic (default: `/yolo/image`)
- `~model` - Model path (default: from `YOLO_MODEL` env)
- `~conf` - Confidence threshold (default: 0.25)
- `~device` - GPU device: `""` (auto), `"0"` (GPU:0), `"cpu"`

### Environment Variables (compose.yaml)
- `YOLO_MODEL` - Path to weights file
- `YOLO_CONF` - Detection confidence threshold
- `YOLO_DEVICE` - GPU selection

## Key Implementation Details

- Python interpreter: `/opt/venv/bin/python` (venv isolates ultralytics from system packages)
- Model weights stored in `./weights/` (mounted to `/workspace/weights/`)
- Uses `buff_size=2**24` on subscriber to prevent message drops
- Queue size=1 for latest-frame-only processing
