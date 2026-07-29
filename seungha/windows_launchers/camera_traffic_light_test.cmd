@echo off
setlocal
cd /d "%~dp0"

set "CAMERA_PYTHON=%~dp0.venv\Scripts\python.exe"
if not exist "%CAMERA_PYTHON%" (
  echo [ERROR] Camera test environment is not ready.
  echo Run: python -m venv .venv
  echo Then: .venv\Scripts\python.exe -m pip install -r requirements-camera.txt
  pause
  exit /b 1
)

"%CAMERA_PYTHON%" "%~dp0ros2_ws\tools\traffic_light_camera_test.py" %*
if errorlevel 1 pause
