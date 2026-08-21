# Models

Model binaries are not committed to normal Git history. The original detector
is `road_best.pt`. The segmentation planner uses `center_lane_best.pt`. Their
expected SHA-256 values and class metadata are recorded in
`../configs/model_manifest.yaml` and `.env.example`.

Before competition use:

```bash
sha256sum models/road_best.pt
sha256sum models/center_lane_best.pt
```

`road_best.pt` is an Ultralytics detection model with `lane1`/`lane2` classes.
`center_lane_best.pt` is an Ultralytics segmentation model with `center`/`lane`
classes. They require different ROS inference nodes and are not interchangeable.
