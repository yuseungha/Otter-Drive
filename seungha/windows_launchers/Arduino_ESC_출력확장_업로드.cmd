@echo off
chcp 65001 >nul
title Arduino ESC 출력 확장 업로드

powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0Arduino_ESC_출력확장_업로드.ps1"
exit /b

