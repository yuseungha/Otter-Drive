# html 파일로 UI 만들어서 조종하면 어떨까 하는 생각에 만들어봤는데 응답속도 느려서 안 쓸 예정.
# 다음에 왔을 때 이거 없애는 것부터 할 예정

# Laptop teleoperation 

노트북 브라우저의 키보드 입력을 Jetson ROS 2 토픽으로 보내고, 기존
`rc_car_teleop/serial_bridge`가 Arduino로 전달합니다.

## 안전 계층

1. 브라우저는 키를 누르는 동안 20 Hz heartbeat를 보냅니다.
2. `web_teleop`은 0.25초 동안 heartbeat가 없으면 ARM을 해제합니다.
3. `serial_bridge`는 0.30초 동안 ROS 명령이 없으면 Arduino 송신을 멈춥니다.
4. Arduino 펌웨어는 0.40초 동안 시리얼 명령이 없으면 ESC와 조향을 정지합니다.
5. 실차 실행 스크립트는 `config/hardware_confirmed.env`가 `YES`일 때만 열립니다.

## DRY-RUN

Jetson의 `sanditest` 컨테이너에서 빌드한 후 다음을 실행합니다.

```bash
bash /home/sandi/ros2_ws/src/laptop_teleop/scripts/run_dry_run.sh
```

노트북에서는 SSH 포트포워딩 후 `http://127.0.0.1:8765`를 엽니다.
루트 폴더의 `수동주행_드라이런.cmd`가 이 과정을 자동으로 실행합니다.

## 실차 실행 전 확인 필수

- 실제 보드가 Mega 2560인지 Uno R3인지
- ESC signal 핀
- 조향 모터 드라이버 ENA, IN1, IN2 핀
- 조향 피드백 ADC 핀
- 조향 왼쪽/중앙/오른쪽 ADC 실측값
- 조향 모터 방향
- ESC 중립, 저속 전진, 저속 후진 pulse
- 물리 비상정지 또는 구동 전원 즉시 차단 수단

확인 전에는 펌웨어의 `HARDWARE_PROFILE_VERIFIED`와
`config/hardware_confirmed.env`를 활성화하지 않습니다.

