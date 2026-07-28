# YOLO 라인 검출 개발 가이드

## 구성

| 구성 요소 | 역할 | ROS 2 토픽 |
| --- | --- | --- |
| `camera_publisher` | 다운로드한 주행 영상을 프레임으로 발행 | `/image_raw` |
| `line_detection` | `roadfinal_best.pt` 기반 YOLO 라인/객체 검출 | `/yolo/detections` |

입력 영상과 모델은 각각 아래 경로를 사용한다.

```text
/home/juwnoo/Downloads/국민대학교 자율주행 스튜디오 트랙 영상 - 자이트론 (480p, h264).mp4
/home/juwnoo/Downloads/road_best.pt
```

Docker Compose는 다운로드 폴더를 컨테이너에 읽기 전용으로 마운트한다.

## 개발 환경과 Jetson 전환

현재 Galaxy Book5 Pro 개발 환경은 CPU 추론을 기본으로 사용한다. Intel Arc GPU용 런타임은
구성하지 않는다. 모델 및 노드 인터페이스는 Jetson Orin Nano 전환을 고려해 유지한다.

Jetson에서 NVIDIA JetPack과 CUDA PyTorch 환경을 준비하고 TensorRT `.engine` 또는 PyTorch `.pt` 모델 경로를 지정한 뒤 다음 옵션으로 실행하면 된다.

```bash
ros2 launch line_detection yolo_line_detection.launch.py device:=cuda:0 model_path:=/models/roadfinal_best.engine
```

## 빌드와 실행

호스트에서 컨테이너를 최신 이미지로 다시 만들고 접속한다.

```bash
cd /home/juwnoo/test
docker compose up -d --force-recreate
./enter_container.sh
```

컨테이너에서 워크스페이스를 빌드한다.

```bash
cd /home/juwnoo/test/ws_autonomy
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

두 터미널에서 다음 노드를 실행한다.

```bash
# 터미널 1: 녹화 영상 카메라
ros2 launch camera_publisher video_camera.launch.py

# 터미널 2: YOLO CPU 추론
ros2 launch line_detection yolo_line_detection.launch.py
```

## 런치 옵션

```bash
# 신뢰도 임계값을 0.40으로 변경
ros2 launch line_detection yolo_line_detection.launch.py confidence:=0.40

# Jetson Orin Nano에서 CUDA GPU 사용
ros2 launch line_detection yolo_line_detection.launch.py device:=cuda:0
```

`/yolo/detections`의 메시지 형식은 `vision_msgs/msg/Detection2DArray`다. Segmentation 모델이면 `/yolo/line_points`에 정규화된 polygon 좌표도 발행하며, `/yolo/inference_ms`는 프레임별 추론 시간을 발행한다.
