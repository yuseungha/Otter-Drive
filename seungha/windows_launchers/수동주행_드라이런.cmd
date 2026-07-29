@echo off
chcp 65001 >nul
title Jetson RC Car 수동주행 DRY-RUN

echo 모터 출력이 없는 DRY-RUN 조종기를 시작합니다.
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0수동주행_드라이런.ps1"
exit /b %errorlevel%
