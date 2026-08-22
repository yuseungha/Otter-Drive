# Runtime architecture

```text
BRIO camera
  -> /camera/front/image_raw
  -> YOLO lane detector (lane1/lane2)
  -> /lane/center_error, /lane/valid, /lane/confidence
  -> lane controller (DRY-RUN by default)
  -> actuation monitor
  -> serial bridge only after explicit live-hardware approval
```

The host launches a Jetson CUDA 12.6 container. The repository is mounted at
the same absolute path so ROS install scripts and model paths remain stable.
Colcon build, install, test, and log output is isolated under `.colcon/` and is
never treated as source code.
