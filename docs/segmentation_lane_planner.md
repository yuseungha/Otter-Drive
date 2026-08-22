# Segmentation lane planner

`models/lane_seg_v3_e37.pt`는 Ultralytics 세그멘테이션 모델이며 클래스는
`center`와 `lane`입니다. 기존 `road_best.pt`의 `lane1`/`lane2` 바운딩박스
노드와 모델 인터페이스가 다르므로 전용 `yolo_seg_lane_detector` 노드가
처리합니다.

## 경로 산출

1. 각 이미지 높이의 50–96% 구간을 여러 가로 밴드로 샘플링합니다.
2. 두 `lane` 마스크가 있으면 두 경계의 중점을 목표점으로 사용합니다.
3. `center` 마스크는 경계 중점과의 일관성 검사와 경계 누락 시 보강에 씁니다.
4. 최소 3개 목표점과 이미지 높이 15% 이상의 관측 구간이 있을 때만 1·2차
   경로를 맞춥니다.
5. 74% 높이의 횡오차와 원근 방향의 heading error를 기존 `lane_control`에
   전달합니다.

한쪽 `lane`만 있고 `center`가 없으면 임의 차선 폭을 적용하지 않습니다.
플래너는 `valid=false`를 발행하고 기존 제어기의 hold → decay → stop 절차를
사용합니다.

## 안전한 최초 실행

패키지를 다시 빌드한 뒤 모델과 CUDA 호환성을 확인합니다.

```bash
cd /home/sandi/KMU_AutoDriving
./scripts/setup_jetson.sh
./scripts/run_seg_lane.sh --check
```

기본 녹화 영상과 카메라 검증은 모두 `/rc_car/drive_cmd_preview`만 발행합니다.

```bash
./scripts/run_seg_lane.sh --video /absolute/path/to/video.mp4
./scripts/run_seg_lane.sh --camera
```

화면 HUD가 필요하면 `.env`에서 `KMU_DISPLAY=true`로 설정합니다. 주요 진단
토픽은 `/lane/lane_overlay`, `/lane/lane_geometry`, `/lane/center_error`,
`/lane/heading_error`, `/lane/confidence`, `/lane/valid`입니다.

## 녹화영상 실차 주행

공중 구동 시험을 마친 차량에서만 `.env`의 Arduino 경로와
`KMU_HARDWARE_CONFIRMED=true`를 확인한 뒤 명시적인 live 모드를 사용합니다.

```bash
./scripts/run_seg_lane.sh --video-live /absolute/path/to/video.mp4
```

이 모드는 호스트에서 안전 시리얼 브리지를 실행하고 컨테이너의 CV/YOLO
결과만 ROS로 전달합니다. 검증된 출력 한계는 전진 `0..700`, 조향
`-650..650`이며 차선주행에는 후진을 허용하지 않습니다. 입력 지연, 차선
완전 유실, 종료 또는 deadman 해제 시 `0,0`을 보낸 뒤 STOP으로 닫힙니다.

## 초기 현장 튜닝

- `confidence_threshold`: 마스크 오검출과 누락의 균형
- `scan_rows`, `look_ahead_ratio`: 카메라 장착 각도에 맞는 전방 주시점
- `center_consistency_tol`: 중앙 점선과 경계 중점의 허용 차이
- `kp`, `kd`, `k_heading`: 조향 응답. 공중 조향 방향 검증 전에는 변경 금지

실차 주행 전에는 영상 전체 구간의 검출률과 조향 부호를 preview 모드에서 먼저
확인하고, 비상 정지 수단과 차량 주변 안전거리를 확보해야 합니다.
