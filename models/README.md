# Models

Model binaries are not committed to normal Git history. The active model is
`road_best.pt`; its expected SHA-256 and class metadata are recorded in
`../configs/model_manifest.yaml` and `.env.example`.

Before competition use:

```bash
sha256sum models/road_best.pt
```

The expected classes are `lane1` and `lane2` and the expected task is
Ultralytics object detection.
