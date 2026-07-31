# 검증 결과

검증일: 2026-07-29  
환경: WSL Ubuntu 22.04, ROS 2 Humble, Python 3.10.12

## 수행 명령

```bash
source /opt/ros/humble/setup.bash
cd laptop/ros2_ws
export PYTHONNOUSERSITE=1
colcon build --symlink-install \
  --packages-select kmu_track rc_car_teleop laptop_teleop
colcon test \
  --packages-select kmu_track rc_car_teleop laptop_teleop \
  --event-handlers console_direct+
colcon test-result --verbose
```

## 결과

```text
Build: 3 packages finished
kmu_track: 37 passed
rc_car_teleop: 3 passed
laptop_teleop: 5 passed
Summary: 45 tests, 0 errors, 0 failures, 0 skipped
```

추가 확인:

- Python 소스 `compileall`: 성공
- 원본 72개 패키지 파일과 업로드용 복사본의 SHA-256 비교: 불일치 0개
- 실제 `hardware_confirmed.env`: 제외
- 토큰·API 키 형태 문자열: 발견되지 않음

## 검증 범위 밖

- 실제 경기장 영상의 ROI·HSV 임계값
- Jetson 카메라 지연과 프레임 손실
- Arduino USB 재연결의 실기 타이밍
- 조향 방향·ADC 중앙값·ESC 중립값
- 지상 주행과 물리 비상정지
