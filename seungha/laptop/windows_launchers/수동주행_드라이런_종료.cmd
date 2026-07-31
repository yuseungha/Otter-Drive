@echo off
chcp 65001 >nul
title Jetson RC Car DRY-RUN 종료

echo DRY-RUN 조종기와 Jetson 서버를 안전하게 종료합니다.
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0수동주행_드라이런_종료.ps1"
exit /b %errorlevel%

