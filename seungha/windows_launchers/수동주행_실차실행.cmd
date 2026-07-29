@echo off
chcp 65001 >nul
title Jetson RC Car 수동주행 LIVE

echo 실차 수동주행 서버를 시작합니다.
echo 하드웨어 확인 전에는 Jetson의 안전 잠금에 의해 실행이 차단됩니다.
start "Jetson LIVE 서버" powershell.exe -NoExit -Command "ssh -tt -L 8765:127.0.0.1:8765 jetson-car bash /home/sandi/ros2_ws/src/laptop_teleop/scripts/run_live.sh"
timeout /t 6 /nobreak >nul
start "" "http://127.0.0.1:8765"

