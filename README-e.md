# yolo26_noetic

ROS1 Noetic package for YOLOv8 object detection with ByteTrack/BotSort tracking support.

## Features

- YOLOv8 object detection via Ultralytics
- ByteTrack/BotSort object tracking
- Bounding box smoothing (moving average)
- Hysteresis filtering (appear/disappear thresholds)
- Detection2DArray (vision_msgs) output
- Compressed image transport support
- Direct camera capture with OpenCV
- OpenCV window display for debugging
- GPU acceleration with Docker

## Requirements

- Docker & Docker Compose
- NVIDIA GPU with drivers installed
- nvidia-container-toolkit

## Build

```bash
docker compose --profile yolo build
```

## Run

```bash
# Basic mode (no tracking, requires external image topic)
docker compose --profile yolo up

# Tracking enabled mode
docker compose --profile yolo-tracking up

# OpenCV display mode (for debugging, direct camera capture)
xhost +local:docker
docker compose --profile yolo-display up

# Tracking + OpenCV display mode
xhost +local:docker
docker compose --profile yolo-tracking-display up

# With USB camera
docker compose --profile yolo --profile webcam up
docker compose --profile yolo-tracking --profile webcam up
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `model` | env YOLO_MODEL | Path to .pt weights |
| `image` | /camera/image_raw | Input image topic |
| `device` | 0 | GPU device or "cpu" |
| `conf` | 0.25 | Confidence threshold |
| `tracking` | false | Enable ByteTrack |
| `smoothing` | 15 | Smoothing window (frames) |
| `appear_frames` | 3 | Frames to confirm appearance |
| `disappear_frames` | 5 | Frames to confirm disappearance |
| `cv_display` | false | OpenCV window display |
| `direct_camera` | false | Direct camera capture with OpenCV |
| `camera_device` | /dev/video0 | Camera device |
| `camera_width` | 1280 | Camera resolution (width) |
| `camera_height` | 720 | Camera resolution (height) |

### Example with tracking

```bash
docker compose --profile yolo run --rm yolo26_ros1 bash -lc "roslaunch yolo26_ros1 yolo26.launch model:=/models/yolo26n.pt tracking:=true"
```

### OpenCV Window Display (for debugging)

Run with direct camera capture and OpenCV window display:

```bash
# Allow X11 access
xhost +local:docker

# Run
docker compose --profile yolo run --rm yolo26_ros1 bash -lc "roslaunch yolo26_ros1 yolo26.launch model:=/models/yolo26n.pt cv_display:=true direct_camera:=true"
```

- Press `q` or `ESC` to exit the node
- Requires X11 forwarding (run `xhost +local:docker` beforehand)

## Topics

### Subscriptions
- `/camera/image_raw` (sensor_msgs/Image)

### Publications
- `/yolo26/detections` (vision_msgs/Detection2DArray)
- `/yolo26/debug_image` (sensor_msgs/Image)

## Model Weights

Place your model weights in `./models/` directory:
- `yolo26n.pt` - YOLOv8n model (default)
- `classes.yaml` - Class name mapping

## License

Apache 2.0
