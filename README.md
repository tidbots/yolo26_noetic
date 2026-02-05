# yolo26_noetic

YOLOv8物体検出とByteTrack/BotSortトラッキングをサポートするROS1 Noeticパッケージです。

## 機能

- Ultralyticsを使用したYOLOv8物体検出
- ByteTrack/BotSortによるオブジェクトトラッキング
- バウンディングボックスのスムージング（移動平均）
- ヒステリシスフィルタリング（出現/消失の閾値）
- Detection2DArray（vision_msgs）出力
- 圧縮画像転送のサポート
- OpenCVによる直接カメラキャプチャ
- OpenCVウィンドウによるデバッグ表示
- DockerによるGPUアクセラレーション

## 必要条件

- Docker & Docker Compose
- NVIDIAドライバがインストールされたNVIDIA GPU
- nvidia-container-toolkit

## ビルド

```bash
docker compose --profile yolo build
```

## 実行

```bash
# === ROSトピック入力（外部カメラノードが必要） ===
docker compose --profile yolo up                    # トラッキングなし
docker compose --profile yolo-tracking up           # トラッキング有効

# === 直接カメラキャプチャ（usb_cam不要） ===
docker compose --profile yolo-direct up             # トラッキングなし
docker compose --profile yolo-tracking-direct up    # トラッキング有効

# === OpenCV表示 + 直接カメラキャプチャ（デバッグ用） ===
xhost +local:docker
docker compose --profile yolo-display up            # トラッキングなし
docker compose --profile yolo-tracking-display up   # トラッキング有効

# === USBカメラノードと組み合わせ（カメラによっては非対応） ===
docker compose --profile yolo --profile webcam up
docker compose --profile yolo-tracking --profile webcam up
```

## 起動引数

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `model` | 環境変数 YOLO_MODEL | .pt重みファイルのパス |
| `image` | /camera/image_raw | 入力画像トピック |
| `device` | 0 | GPUデバイスまたは"cpu" |
| `conf` | 0.25 | 信頼度の閾値 |
| `tracking` | false | ByteTrackを有効化 |
| `smoothing` | 15 | スムージングウィンドウ（フレーム数） |
| `appear_frames` | 3 | 出現確認に必要なフレーム数 |
| `disappear_frames` | 5 | 消失確認に必要なフレーム数 |
| `cv_display` | false | OpenCVウィンドウ表示 |
| `direct_camera` | false | OpenCVで直接カメラキャプチャ |
| `camera_device` | /dev/video0 | カメラデバイス |
| `camera_width` | 1280 | カメラ解像度（幅） |
| `camera_height` | 720 | カメラ解像度（高さ） |

### トラッキング有効時の例

```bash
docker compose --profile yolo run --rm yolo26_ros1 bash -lc "roslaunch yolo26_ros1 yolo26.launch model:=/models/yolo26n.pt tracking:=true"
```

### OpenCVウィンドウ表示（デバッグ用）

直接カメラキャプチャとOpenCVウィンドウ表示を有効にして実行：

```bash
# X11アクセスを許可
xhost +local:docker

# 実行
docker compose --profile yolo run --rm yolo26_ros1 bash -lc "roslaunch yolo26_ros1 yolo26.launch model:=/models/yolo26n.pt cv_display:=true direct_camera:=true"
```

- `q`キーまたは`ESC`キーでノードを終了できます
- X11転送が必要です（`xhost +local:docker`を事前に実行）

## トピック

### 購読トピック
- `/camera/image_raw` (sensor_msgs/Image)

### 配信トピック
- `/yolo26/detections` (vision_msgs/Detection2DArray)
- `/yolo26/debug_image` (sensor_msgs/Image)

## モデルの重みファイル

`./models/`ディレクトリにモデルの重みファイルを配置してください：
- `yolo26n.pt` - YOLOv8nモデル（デフォルト）
- `classes.yaml` - クラス名のマッピング

## ライセンス

Apache 2.0
