# LiDAR 데이터 rosbag 녹화 방법

이 문서는 ROS 2 Humble에서 RPLIDAR의 `sensor_msgs/msg/LaserScan` 데이터를
rosbag으로 저장하는 방법을 설명한다. 프로젝트에서 사용하는 기본 LiDAR 토픽은
`/scan`이다.

## 녹화 스크립트

녹화에는 다음 스크립트를 사용한다.

```text
scripts/record_lidar_rosbag.sh
```

스크립트는 다음 작업을 자동으로 수행한다.

- `/opt/ros/humble/setup.bash`와 프로젝트 빌드 환경 로드
- `.env`의 `KMU_ROS_DOMAIN_ID` 적용(기본값 `86`)
- 녹화 전 LiDAR 메시지 수신 여부 확인
- `LaserScan` 또는 `PointCloud2` 타입 검사
- `/scan`, `/tf`, `/tf_static` 녹화
- sqlite3 저장 및 zstd 파일 압축
- 녹화 종료 후 bag 정보와 LiDAR 메시지 수 검사

## 기본 실행 순서

첫 번째 터미널에서 모터와 Arduino bridge를 사용하지 않는 LiDAR 인식 모드를
실행한다.

```bash
cd /home/sandi/KMU_AutoDriving
./scripts/run_competition.sh --cone-dry-run
```

두 번째 터미널에서 rosbag 녹화를 시작한다.

```bash
cd /home/sandi/KMU_AutoDriving
./scripts/record_lidar_rosbag.sh
```

시간 제한을 지정하지 않으면 `Ctrl+C`를 누를 때까지 녹화한다. 종료 시 rosbag이
캐시 데이터를 저장하고 `metadata.yaml`을 생성할 때까지 기다린다.

## 시간과 저장 경로 지정

30초 동안 녹화한다.

```bash
./scripts/record_lidar_rosbag.sh --duration 30
```

저장 경로를 직접 지정한다. 상대 경로는 프로젝트 루트를 기준으로 해석한다.

```bash
./scripts/record_lidar_rosbag.sh \
  --duration 30 \
  --output data/lidar/rosbags/cone_course_01
```

저장 경로를 생략하면 다음 형식의 디렉터리가 자동 생성된다.

```text
data/lidar/rosbags/lidar_YYYYMMDD-HHMMSS
```

이미 존재하는 경로에는 덮어쓰지 않는다.

## 연결 확인

bag을 만들지 않고 LiDAR 메시지와 타입만 확인한다.

```bash
./scripts/record_lidar_rosbag.sh --check
```

정상이라면 다음과 같은 결과가 출력된다.

```text
LIDAR_TOPIC=/scan
LIDAR_TYPE=sensor_msgs/msg/LaserScan
ROS_DOMAIN_ID=86
LIDAR_ROSBAG_CHECK=OK
```

## 다른 LiDAR 토픽 녹화

토픽이 `/scan`이 아닌 경우 절대 토픽 이름을 지정한다.

```bash
./scripts/record_lidar_rosbag.sh --topic /lidar/scan
```

스크립트는 `sensor_msgs/msg/LaserScan`과 `sensor_msgs/msg/PointCloud2`를 지원한다.

## 선택 옵션

```text
-t, --topic TOPIC       LiDAR 토픽 지정(기본값: /scan)
-o, --output PATH       rosbag 저장 디렉터리 지정
-d, --duration SEC      지정한 시간이 지나면 자동 종료
    --wait-timeout SEC  첫 LiDAR 메시지를 기다릴 시간(기본값: 15초)
    --without-tf        /tf와 /tf_static을 제외하고 LiDAR 토픽만 저장
    --no-compression    zstd 압축 비활성화
    --check             연결만 확인하고 종료
-h, --help              전체 도움말 출력
```

## 녹화 결과 확인 및 재생

녹화 정보를 확인한다.

```bash
source /opt/ros/humble/setup.bash
ros2 bag info data/lidar/rosbags/lidar_YYYYMMDD-HHMMSS
```

녹화 데이터를 재생한다.

```bash
source /opt/ros/humble/setup.bash
ros2 bag play data/lidar/rosbags/lidar_YYYYMMDD-HHMMSS
```

재생 중 `/scan`을 확인하려면 다른 터미널에서 다음 명령을 실행한다.

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /scan
```

## 오류 확인

`no message received on /scan` 오류가 발생하면 다음 항목을 확인한다.

1. `./scripts/run_competition.sh --cone-dry-run`이 실행 중인지 확인한다.
2. `ros2 topic list -t`에 `/scan [sensor_msgs/msg/LaserScan]`이 표시되는지 확인한다.
3. 녹화 터미널과 LiDAR 실행 환경의 `ROS_DOMAIN_ID`가 같은지 확인한다.
4. `.env`의 `KMU_LIDAR_DEVICE`가 실제 `/dev/serial/by-id/...` 장치인지 확인한다.

스크립트는 LiDAR 메시지가 한 개도 저장되지 않은 빈 bag을 성공으로 처리하지
않는다. 녹화 종료 후 `LIDAR_MESSAGES`와 `LIDAR_ROSBAG_SAVED`가 출력되어야 정상이다.

## 검증 결과

실제 장치를 구동하지 않고 별도 ROS 도메인에서 가짜 `LaserScan`을 10 Hz로
발행해 통합 시험했다. 2초 제한 시험에서 `/scan` 15개 메시지가 zstd 압축 bag에
저장되고 `metadata.yaml` 및 `ros2 bag info`로 정상 확인됐다.
