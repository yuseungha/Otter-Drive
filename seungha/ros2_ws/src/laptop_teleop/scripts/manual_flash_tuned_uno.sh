#!/usr/bin/env bash
set -eo pipefail

port=/dev/ttyACM0
hex_file=/tmp/rc_car_safe_70_build/rc_car_controller_safe_tuned.ino.hex
tuned_source=/home/sandi/Arduino/rc_car_controller_safe_tuned/rc_car_controller_safe_tuned.ino
active_source=/home/sandi/Arduino/rc_car_controller_safe/rc_car_controller_safe.ino

if [[ ! -e "$port" ]]; then
  echo "Arduino port not found: $port" >&2
  exit 1
fi
if [[ ! -s "$hex_file" ]]; then
  echo "Compiled HEX not found: $hex_file" >&2
  exit 1
fi

echo
echo '============================================================'
echo ' Arduino Uno 수동 RESET 업로드'
echo ' 목표: 전진 1850us / 후진 1150us / 중립 1500us (70%)'
echo '============================================================'
echo
echo '1) 차량 바퀴를 띄우고 가능하면 구동 배터리를 분리하세요.'
echo '2) Arduino의 RESET 버튼을 계속 누르세요.'
read -r -p '3) RESET을 누른 상태에서 이 창의 Enter를 누르세요: '

echo
echo '>>> 지금 RESET 버튼에서 손을 떼세요!'
sleep 0.20

if avrdude \
    -p atmega328p \
    -c arduino \
    -P "$port" \
    -b 115200 \
    -D \
    -U "flash:w:$hex_file:i"; then
  cp -p "$tuned_source" "$active_source"
  echo
  echo 'UPLOAD SUCCESS'
  echo '활성 소스도 1850us / 1150us 값으로 갱신했습니다.'
else
  echo
  echo 'UPLOAD FAILED - 기존 펌웨어와 활성 소스는 유지됩니다.' >&2
  exit 1
fi
