#!/usr/bin/env bash
set -eo pipefail

python3 /home/sandi/ros2_ws/src/laptop_teleop/scripts/reset_arduino_dtr.py

exec avrdude \
  -p atmega328p \
  -c arduino \
  -P /dev/ttyACM0 \
  -b 115200 \
  -D \
  -U flash:w:/tmp/rc_car_safe_70_build/rc_car_controller_safe_tuned.ino.hex:i
