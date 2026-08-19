# KMU IRE center-priority lane planner

`kmu_ire_track` is an isolated variant of the existing `kmu_track`
segmentation pipeline. It keeps the same model, topics, controller,
visualizer, safety gate, and hardware limits while changing only path-source
priority:

1. A detected `center` mask (the yellow center marking) defines the target.
2. Two `lane` masks validate the center target when available.
3. The midpoint of two `lane` masks is used only when no center marking is
   available for that scan row.

The original `kmu_track` package and `scripts/run_seg_lane.sh` remain the
legacy boundary-first implementation. Do not run the legacy and IRE launch
files at the same time because they intentionally use the same ROS topics.

Build and verify:

```bash
source /opt/ros/humble/setup.bash
source .colcon/install/setup.bash
colcon --log-base .colcon/log build \
  --base-paths src/laptop src/jetson \
  --build-base .colcon/build \
  --install-base .colcon/install \
  --packages-select kmu_ire_track \
  --symlink-install
./scripts/run_ire_seg_lane.sh --check
```

Preview a recording without hardware output:

```bash
./scripts/run_ire_seg_lane.sh --video /absolute/path/to/video.mp4 --display
```

The low-latency path captures at 1920x1080 but runs inference at 768 pixels,
keeps only the newest frame, and skips all image overlays when display is
disabled. Use `--headless` to force this mode even if `.env` enables display.
Live modes are headless by default; pass `--display` only for diagnostics.
Live camera capture and YOLO run in one process, so the 1920x1080 BGR frame is
not serialized through a ROS image topic. Recorded-video launches still use a
depth-one ROS image subscription because the video source is a separate node.

Live actuation remains guarded by the same `.env` hardware confirmation and
serial limits as the legacy pipeline:

```bash
./scripts/run_ire_seg_lane.sh --video-live /absolute/path/to/video.mp4
```

For the real-camera path, use the dedicated guarded runner. Its
preflight requires the stable camera and Arduino by-id devices, rejects device
and local ROS conflicts, and isolates the graph with `ROS_LOCALHOST_ONLY=1`.
Live output additionally requires explicit approval in the current shell and
keeps the confirmed throttle `0..700` and steering `-650..650` limits:

```bash
./scripts/run_ire_seg_lane.sh --camera-live-check --headless
KMU_DRIVE_APPROVED=true ./scripts/run_ire_seg_lane.sh --camera-live --headless
```

Keep the driven wheels airborne. Confirm steering direction and then obscure
the marking to verify LOST returns steering to neutral. Do not use
`--video-live` for a real camera.

Emergency stop from another terminal on the same host:

```bash
source /opt/ros/humble/setup.bash
source /home/sandi/KMU_AutoDriving/.colcon/install/setup.bash
ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1 ros2 topic pub --once \
  /vehicle/estop std_msgs/msg/Bool '{data: true}'
```
