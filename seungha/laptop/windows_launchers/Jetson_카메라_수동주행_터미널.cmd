@echo off
chcp 65001 >nul
title Jetson 카메라 안전 수동주행 - DRY RUN
echo Jetson 터미널 수동주행을 시작합니다.
echo W/S: 전후진, A/D: 조향, Space: 속도 정지, C: 센터, X: 전체 정지
echo 현재는 DRY-RUN이며 실제 모터로 출력되지 않습니다.
echo.
wsl.exe -d Ubuntu-22.04 -- ssh -t jetson-car "docker exec -it sanditest bash -lc 'source /opt/ros/humble/install/setup.bash; source /root/ros2_ws/install/setup.bash; ros2 run rc_car_teleop keyboard_teleop --ros-args -r /rc_car/drive_cmd:=/rc_car/manual_drive_cmd'"
echo.
echo 수동주행 터미널이 종료되었습니다. 안전 게이트가 중립 명령을 유지합니다.
pause
