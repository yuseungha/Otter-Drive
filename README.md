# KMU AutoDriving

Jetson Orin에서 실행하는 대회용 ROS 2 Humble 작업공간입니다. 현재 기준선은
Logitech BRIO 기반 YOLO 차선 인식과 RPLIDAR 기반 라바콘 중앙 경로 인식입니다.
두 경로 모두 기본 실행에서는 차량 출력이 차단됩니다.

## 최초 준비

```bash
cd /home/sandi/KMU_AutoDriving
cp .env.example .env
./scripts/setup_jetson.sh
./scripts/run_competition.sh --check
```

`.env`에서 카메라와 모델 경로를 확인하십시오. `.env`는 Git에 저장하지 않습니다.

## 실행

카메라와 YOLO/제어 토픽만 실행하며 Arduino 출력은 차단합니다.

```bash
./scripts/run_competition.sh --dry-run
```

녹화 영상 검증:

```bash
./scripts/run_competition.sh --video /absolute/path/to/video.mp4
```

새 `center`/`lane` 세그멘테이션 모델로 기본 차선 플래너를 검증할 때는 전용
드라이런을 사용합니다. 두 실행 모두 Arduino bridge를 시작하지 않습니다.

```bash
./scripts/run_seg_lane.sh --check
./scripts/run_seg_lane.sh --video /absolute/path/to/video.mp4
./scripts/run_seg_lane.sh --camera
```

이 플래너는 양쪽 `lane` 마스크의 중점을 우선 추종하고 `center` 마스크로
보강합니다. 한쪽 경계만 보이고 중앙선도 없으면 차선 폭을 추측하지 않고
`/lane/valid=false`를 내보냅니다. 상세 구조와 초기 튜닝 항목은
`docs/segmentation_lane_planner.md`에 있습니다.

RPLIDAR와 라바콘 플래너만 실행하며 모터·ESC·Arduino bridge는 시작하지 않습니다.

```bash
./scripts/run_competition.sh --cone-dry-run
```

다른 터미널에서 `/scan`, `/tf`, `/tf_static`을 rosbag으로 녹화할 수 있습니다.
기본 저장 위치는 `data/lidar/rosbags/lidar_<날짜-시간>`이며 `Ctrl+C`로 안전하게
종료합니다.

```bash
./scripts/record_lidar_rosbag.sh

# 30초 후 자동 종료
./scripts/record_lidar_rosbag.sh --duration 30
```

녹화 전 연결만 확인하려면 `./scripts/record_lidar_rosbag.sh --check`를 사용합니다.
전체 옵션은 `./scripts/record_lidar_rosbag.sh --help`와
[`docs/lidar_rosbag_recording.md`](docs/lidar_rosbag_recording.md)에서 볼 수 있습니다.

LiDAR 장착 좌표와 코스 실측값은 `configs/cone/`에 있으며, 실행 전에
`.env`의 `KMU_LIDAR_DEVICE`를 실제 by-id 장치 경로와 대조해야 합니다.

실차 출력은 `docs/competition_runbook.md`의 하드웨어 검증을 완료하고 `.env`의
`KMU_HARDWARE_CONFIRMED=true`를 명시한 경우에만 허용됩니다.

```bash
./scripts/run_competition.sh --live
```

기본값은 항상 DRY-RUN입니다. `Ctrl+C` 시 Docker 컨테이너와 ROS launch를 함께
종료하며 실행 로그는 `logs/<날짜-시간>/competition.log`에 저장합니다.

## 소스 구분

- `src/common`: 장비 비의존 공통 코드의 향후 분리 위치
- `src/laptop`: 영상 재생 및 노트북 운영 도구의 향후 분리 위치
- `src/jetson/kmu_track`: 차선 인식·제어·bringup ROS 패키지
- `src/jetson/rc_car_teleop`: Arduino 직렬 브리지
- `src/jetson/lidar_cone_planner`: 라바콘 검출·중앙 경로·안전 제어 패키지
- `src/jetson/rplidar_ros`: Slamtec RPLIDAR ROS 2 드라이버
- `configs`: 장비 및 알고리즘 운영 설정
- `models`: 로컬 모델 저장 위치. 모델 바이너리는 Git에서 제외

빌드·테스트 생성물은 숨김 경로 `.colcon/`에만 저장되고 Git에서 제외됩니다.
실행 로그는 `logs/`에 저장되므로 프로젝트 최상위에는 운영에 필요한 폴더만
유지됩니다. 수신 당시 파일의 해시 기록은 `docs/transfer_manifest.received.sha256`에
보관합니다.

호스트의 사용자 Python 패키지는 사용하지 않습니다. Jetson 런타임은
`sandikookmin:cuda126` 컨테이너의 CUDA 12.6용 PyTorch를 사용합니다.
