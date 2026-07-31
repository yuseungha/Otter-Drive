@echo off
chcp 65001 >nul
title 기존 RC Car 수동주행

echo 기존 수동주행 알고리즘을 새 PowerShell 창에서 실행합니다.
echo 바퀴를 지면에서 띄운 뒤 조작하세요.

powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0기존_수동주행_실행.ps1"
exit /b

