# ws_autonomy

## Recorded-video camera

The `camera_publisher` package publishes the downloaded driving video as
`sensor_msgs/msg/Image` messages on `/image_raw`. The video plays at its native
15 FPS and repeats by default.

```bash
cd /home/juwnoo/test/ws_autonomy
colcon build --symlink-install
source install/setup.bash
ros2 launch camera_publisher video_camera.launch.py
```

Useful launch overrides:

```bash
# Do not repeat after the last frame.
ros2 launch camera_publisher video_camera.launch.py loop:=false

# Use a different file or publish rate.
ros2 launch camera_publisher video_camera.launch.py \\
  video_path:=/path/to/video.mp4 fps:=10.0
```

The compose configuration mounts `/home/juwnoo/Downloads` into the container
read-only, so restart the container once after changing `docker-compose.yml`.

## YOLO line detection with Intel Arc GPU

`line_detection` subscribes to `/image_raw`, loads
`/home/juwnoo/Downloads/roadfinal_best.pt`, and publishes detections on
`/yolo/detections` as `vision_msgs/msg/Detection2DArray`.

The first run converts the PyTorch model to OpenVINO IR under `models/`. Later
runs reuse that export and request the Intel GPU (`device:=GPU`). If the GPU
runtime is unavailable, the node logs the reason and falls back to CPU so the
pipeline remains usable.

After the Docker image is rebuilt, run the camera and detector in separate
container terminals:

```bash
ros2 launch camera_publisher video_camera.launch.py
ros2 launch line_detection yolo_line_detection.launch.py
```
