# Validation record

검증일: 2026-08-19 (Asia/Seoul)

- `setup_jetson.sh`: ROS 2 패키지 빌드 성공
- `colcon test`: 112개 테스트 통과, 실패 및 오류 0개
- `run_competition.sh --check`: CUDA 및 YOLO 모델 사전 점검 성공
- Logitech BRIO DRY-RUN: 640x480 입력 및 `lane/valid=true` 확인
- DRY-RUN 중 Arduino 직렬 브리지는 실행하지 않음

이 기록은 당시 환경의 결과이며 장비 연결이나 의존성이 바뀐 경우
`setup_jetson.sh`와 `run_competition.sh --check`를 다시 실행해야 합니다.
