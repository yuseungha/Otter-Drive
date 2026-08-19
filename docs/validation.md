# Validation record

검증일: 2026-08-19 (Asia/Seoul)

- `setup_jetson.sh`: ROS 2 패키지 빌드 성공
- `colcon test`: 112개 테스트 통과, 실패 및 오류 0개
- `run_competition.sh --check`: CUDA 및 YOLO 모델 사전 점검 성공
- Logitech BRIO DRY-RUN: 640x480 입력 및 `lane/valid=true` 확인
- DRY-RUN 중 Arduino 직렬 브리지는 실행하지 않음
- `lidar_cone_planner`, `rplidar_ros` 포함 4개 ROS 패키지 빌드 성공
- Cone 패키지 테스트: 96개 실행 결과 정상, launch 전용 1개 테스트 skip
- Cone 원본·사본 이미지, 로그, 지표, 영상 및 11개 rosbag 세트 바이트 일치
- `synthetic_validation all`: 직선·좌회전 시나리오 완주, 충돌 0회
  - 직선 진행 1.816 m, 유효 경로 비율 0.993
  - 좌회전 진행 1.098 m, 유효 경로 비율 0.993

현재 환경에는 `ackermann_msgs`가 없어 cone 제어기는 arming을 거부합니다. 대회
스크립트의 `--cone-dry-run`은 해당 제어기와 모든 actuator bridge를 시작하지 않고
RPLIDAR, 정적 TF, 플래너, viewer만 구성합니다.

이 기록은 당시 환경의 결과이며 장비 연결이나 의존성이 바뀐 경우
`setup_jetson.sh`와 `run_competition.sh --check`를 다시 실행해야 합니다.
