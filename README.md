# yolo26_noetic

ROS1 Noetic package for YOLOv8 object detection with ByteTrack/BotSort tracking support.

## Features

- YOLOv8 object detection via Ultralytics
- ByteTrack/BotSort object tracking
- Bounding box smoothing (moving average)
- Hysteresis filtering (appear/disappear thresholds)
- Detection2DArray (vision_msgs) output
- Compressed image transport support
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
# Basic (requires external image topic)
docker compose --profile yolo up

# With USB camera
docker compose --profile yolo --profile webcam up
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

### Example with tracking

```bash
docker compose --profile yolo run --rm yolo26_ros1 bash -lc \
  "roslaunch yolo26_ros1 yolo26.launch model:=/models/best.pt tracking:=true"
```

## Topics

### Subscriptions
- `/camera/image_raw` (sensor_msgs/Image)

### Publications
- `/yolo26/detections` (vision_msgs/Detection2DArray)
- `/yolo26/debug_image` (sensor_msgs/Image)

## Model Weights

Place your model weights in `./models/` directory:
- `best.pt` - Your trained model
- `classes.yaml` - Class name mapping

## License

Apache 2.0
