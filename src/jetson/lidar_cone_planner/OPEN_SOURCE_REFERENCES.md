# Open-source references

확인일: 2026-08-13

이 패키지는 아래 프로젝트의 공개된 알고리즘과 인터페이스를 비교해 설계했지만,
소스 파일이나 모델을 복사해 포함하지 않았습니다. 따라서 런타임 의존성은 ROS 2
메시지와 NumPy뿐이며, 패키지 자체 라이선스는 Apache-2.0입니다.

| 프로젝트 | 라이선스 | 참고한 부분 | 현재 구현과의 차이 |
|---|---|---|---|
| [papalotis/ft-fsd-path-planning](https://github.com/papalotis/ft-fsd-path-planning) | MIT | 색을 모르는 콘 정렬, 한쪽 누락 시 가상 콘, 중앙 곡선 | 글로벌 SLAM 지도·SciPy·Numba 전체 패키지를 넣지 않고, 실제 양측 anchor 이후 최대 2개 tail만 NumPy로 복원 |
| [ajtudela/laser_segmentation](https://github.com/ajtudela/laser_segmentation) | Apache-2.0 | 거리에 따라 달라지는 scan jump threshold | 별도 ROS 노드 의존성 없이 A1 cluster 안에 작은 거리 적응식과 1-beam hole bridge 구현 |
| [harmony-eu/obstacle_detector_2](https://github.com/harmony-eu/obstacle_detector_2) | BSD-3-Clause | 2D 원형 물체와 시간축 추적 구조 | 원 fitting을 복사하지 않고 cone 폭·각폭·깊이 필터와 연속-hit EMA tracker 사용 |
| [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit) | MIT | Pure Pursuit 기본식, 속도 연동 lookahead | 경로·상태 stamp pairing, confidence·정지거리·곡률 속도 제한과 즉시 정지 추가 |
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller) | Apache-2.0 | regulated speed와 입력 경로 관리 방식 | Nav2 costmap 없이 짧은 local cone path 전용이며, 과거 경로 fallback을 허용하지 않음 |
| [QUT-Motorsport/QUTMS_Driverless](https://github.com/QUT-Motorsport/QUTMS_Driverless) | MIT | perception→planning→control의 ROS 2 토픽 분리 | 3D PointCloud2 대신 RPLIDAR A1 2D LaserScan을 직접 처리 |

## 채택하지 않은 부분

- `ft-fsd-path-planning` 전체 의존성: 차량의 글로벌 pose와 cone SLAM map을 요구하고
  SciPy·Numba·scikit-learn 및 최초 JIT 시간이 현재 로컬 A1 플래너에 과합니다.
- 이전 경로 fallback: 센서나 TF가 끊겼을 때 마지막 경로를 재사용하면 실차가 계속
  움직일 수 있어 의도적으로 사용하지 않습니다.
- GPL 저장소의 구현 코드: Urinay와 일부 Formula Student 스택은 구조 비교에만
  사용했으며 코드나 파라미터를 가져오지 않았습니다.
- 처음부터 단일 경계만 보이는 상태의 가상 코스: 색이 없는 local scan만으로 좌우를
  확정할 수 없어 기본적으로 금지합니다.

## A/B 비교 방법

`config/cone_planner.yaml`의 `enable_single_side_fallback`만 바꾸면 같은 rosbag에서
기존 양측 전용 방식과 hybrid 방식을 비교할 수 있습니다. 최소 비교 지표는 다음입니다.

- 유효 경로 비율과 false-valid 비율
- 실제 코스 중앙 대비 횡방향 RMSE
- 프레임 간 path lateral/heading jump
- `OK_VIRTUAL` 비율과 연속 길이
- 처리시간 p50/p95와 scan 주기 초과 횟수
- 빈 Path·USB 분리·TF 오류 후 정지까지 걸린 시간
