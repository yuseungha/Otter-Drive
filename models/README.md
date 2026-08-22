# Models

Model binaries are not committed to normal Git history. The original detector
is `road_best.pt`. The segmentation planner uses `lane_seg_v3_e37.pt`. Their
expected SHA-256 values and class metadata are recorded in
`../configs/model_manifest.yaml` and `.env.example`.

Before competition use:

```bash
sha256sum models/road_best.pt
sha256sum models/lane_seg_v3_e37.pt
```

`road_best.pt` is an Ultralytics detection model with `lane1`/`lane2` classes.
`lane_seg_v3_e37.pt` is an Ultralytics segmentation model with `center`/`lane`
classes. They require different ROS inference nodes and are not interchangeable.

`lane_seg_v3_e37.pt` is a yolo11n-seg checkpoint at **epoch 37 of 100** with
optimizer state still inside it, trained on the `realmerge-3` dataset. It was
chosen over `center_lane_best.pt` for inference speed: yolo11n against
yolov8m, 17.5 MB against 52.3 MB. It has **not** passed the track-video gate
(12 of 30 sampled frames produced a usable `center` mask, threshold 15), so
verify masks on the real camera before relying on it. The previous model is
still recorded in the manifest as `segmentation_lane_planner_v8m`; roll back
by setting `KMU_SEG_MODEL_PATH` and `KMU_SEG_MODEL_SHA256`.
