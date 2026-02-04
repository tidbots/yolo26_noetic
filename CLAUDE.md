# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS1 Noetic package integrating YOLOv8 object detection with camera inputs (OpenNI2 RGB-D, USB cameras). Features include ByteTrack/BotSort tracking, bounding box smoothing, hysteresis filtering, and Detection2DArray output. Runs in Docker with GPU acceleration.

## Build & Run Commands

### Docker (Primary Method)
```bash
# Build image
docker compose build

# Run with yolo profile
docker compose --profile yolo up

# Run with webcam profile (USB camera + YOLO)
docker compose --profile yolo --profile webcam up

# Run with tracking enabled
docker compose --profile yolo run --rm yolo26_ros1 bash -lc \
  "roslaunch yolo26_ros1 yolo26.launch model:=/models/best.pt tracking:=true"
```

### Inside Container (Manual Build)
```bash
source /opt/ros/noetic/setup.bash
cd /catkin_ws
catkin_make
source devel/setup.bash
```

### Launch Files
- `yolo26.launch` - Main launch file with all parameters
- `xtion_yolo26.launch` - Includes OpenNI2 driver
- `usbcam_yolo26.launch` - Launches USB camera node + YOLO

### Launch Arguments
```bash
# Example with tracking
roslaunch yolo26_ros1 yolo26.launch \
  model:=/models/best.pt \
  image:=/camera/image_raw \
  tracking:=true \
  smoothing:=15 \
  appear_frames:=3 \
  disappear_frames:=5
```

## Architecture

```
catkin_ws/src/yolo26_ros1/
├── scripts/yolo26_ros1_node.py  # Main ROS node
├── launch/                       # Launch configurations
├── config/yolo26.yaml           # Default parameters
├── package.xml                   # ROS package metadata
└── CMakeLists.txt               # Catkin build config
```

### Node: yolo26_ros1_node

**Subscriptions:**
- `~image_topic` (sensor_msgs/Image or CompressedImage)

**Publications:**
- `~detections_topic` (vision_msgs/Detection2DArray) - Detections with tracking IDs
- `~debug_image_topic` (sensor_msgs/Image) - Annotated visualization

**Key Features:**
- Timer-based inference at configurable rate (default 15Hz)
- ByteTrack/BotSort object tracking
- Bounding box smoothing (moving average)
- Hysteresis filtering (appear/disappear frame thresholds)
- Class locking after track confirmation
- Supports raw and compressed image transport

### ROS Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~model_path` | env YOLO_MODEL | Path to .pt weights |
| `~device` | "0" | GPU device or "cpu" |
| `~conf_thres` | 0.25 | Confidence threshold |
| `~iou_thres` | 0.45 | IoU NMS threshold |
| `~process_rate_hz` | 15.0 | Inference rate (Hz) |
| `~enable_tracking` | false | Enable ByteTrack |
| `~tracker` | "bytetrack.yaml" | Tracker config |
| `~smoothing_window` | 15 | Frames for moving average |
| `~appear_frames` | 3 | Frames to confirm appearance |
| `~disappear_frames` | 5 | Frames to confirm disappearance |
| `~image_transport` | "raw" | "raw" or "compressed" |
| `~classes_yaml` | "" | Path to class names YAML |

### TrackState Class

Manages per-object tracking state:
- `bbox_history` - Deque for smoothed bounding boxes
- `conf_history` - Deque for smoothed confidence
- `class_history` - Deque for stable class determination
- `appear_count` / `disappear_count` - Hysteresis counters
- `locked_class` - Class locked after confirmation

## Key Implementation Details

- Python interpreter: `/opt/venv/bin/python` (venv isolates ultralytics)
- Model weights: `./models/` (mounted to `/models/`)
- Class names: `./models/classes.yaml`
- Uses `buff_size=2**24` on subscriber to prevent message drops
- Timer-based inference decouples image receipt from processing
