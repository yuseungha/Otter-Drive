# KMU AutoDriving

Jetson Orin에서 실행하는 대회용 ROS 2 Humble 작업공간입니다. 현재 기준선은
Logitech BRIO 입력, YOLO `lane1`/`lane2` 차선 인식, 안전한 차선 제어 DRY-RUN입니다.

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
- `configs`: 장비 및 알고리즘 운영 설정
- `models`: 로컬 모델 저장 위치. 모델 바이너리는 Git에서 제외

빌드·테스트 생성물은 숨김 경로 `.colcon/`에만 저장되고 Git에서 제외됩니다.
실행 로그는 `logs/`에 저장되므로 프로젝트 최상위에는 운영에 필요한 폴더만
유지됩니다. 수신 당시 파일의 해시 기록은 `docs/transfer_manifest.received.sha256`에
보관합니다.

호스트의 사용자 Python 패키지는 사용하지 않습니다. Jetson 런타임은
`sandikookmin:cuda126` 컨테이너의 CUDA 12.6용 PyTorch를 사용합니다.
