# Cone navigation 자료 구성

## 실행 소스

- `src/jetson/lidar_cone_planner`: fail-closed 라바콘 인식 및 중앙 경로 플래너
- `src/jetson/rplidar_ros`: RPLIDAR A1M8 드라이버
- `configs/cone`: 2026-08-18 실측 LiDAR TF와 0.80 m 코스 설정

안전한 인식 전용 실행은 프로젝트 루트에서 다음 명령을 사용합니다.

```bash
./scripts/run_competition.sh --cone-dry-run
```

이 모드는 `cone_lidar_cv.launch.py`만 실행하며 Pure Pursuit, Arduino 직렬 bridge,
모터 및 ESC 출력을 시작하지 않습니다.

## 검증 자료

- `data/cone/images`: OpenCV BEV 캡처 사본
- `data/cone/metrics`: 실시간·재생 검증 지표 사본
- `data/cone/rosbags`: 11개 실측 장면 rosbag 사본
- `data/cone/video`: 시뮬레이션 영상 사본
- `logs/cone`: 빌드·실행·단절 시험 로그 사본
- `lidar_zeroing_cone_planner_2026-08-18.md`: 실측 보정 및 검증 기록

대용량 데이터와 실행 로그는 Git에서 제외됩니다. 원본은
`/home/sandi/KMU_AutoDrive/cone_original/home_root`에 보관합니다.
