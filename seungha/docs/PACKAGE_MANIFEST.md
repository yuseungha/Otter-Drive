# 패키지 구성표

스냅샷 기준일: 2026-07-29

## 포함

- `ros2_ws/src/kmu_track`
  - 신호등·좌회전 화살표 검출
  - 신호등 상태 기반 모터 안전 게이트
  - USB 카메라, 미리보기 서버, 기존 미션·차선 노드
  - 설정, launch, 단위 테스트
- `ros2_ws/src/rc_car_teleop`
  - 키보드 수동 입력
  - Arduino 직렬 브리지
  - launch, 단위 시험, 기어·서보 안내
- `ros2_ws/src/laptop_teleop`
  - 웹 수동 입력과 단일 운전자·heartbeat 상태 머신
  - 웹 UI, launch, 실행·검증 스크립트
  - Arduino 안전 펌웨어와 조향 피드백 도구
- `ros2_ws/tools/traffic_light_camera_test.py`
- `windows_launchers`
  - 카메라 시험, DRY-RUN, 실차, 기존 수동 주행, Arduino 업로드 실행기
- `docs/guides`
  - 노트북 실행 가이드와 100 m 첫 랩 가이드

## 의도적으로 제외

- `build/`, `install/`, `log/`: 다시 생성되는 ROS 2 산출물
- `.venv/`, `__pycache__/`, `.pytest_cache/`: 로컬 Python 환경·캐시
- 영상, rosbag, 학습 가중치: 용량이 크고 이번 알고리즘 스냅샷에 필요하지 않음
- `hardware_confirmed.env`: 특정 차량의 LIVE 승인 상태
- Git 인증 정보, SSH 키, 토큰: 저장소에 포함하지 않음

## ZIP 생성 원칙

배포 ZIP에는 현재 커밋 대상 파일을 넣되 `.git`과 ZIP 자신은 넣지 않습니다.
압축을 푼 뒤 `ros2_ws`에서 바로 `colcon build`할 수 있는 구조를 유지합니다.
