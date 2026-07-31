$Host.UI.RawUI.WindowTitle = 'Jetson Camera-Guarded Manual Drive - DRY RUN'

Write-Host 'Jetson camera-guarded manual driving terminal' -ForegroundColor Cyan
Write-Host 'W/S: throttle, A/D: steering, Space: throttle stop, C: center, X: full stop'
Write-Host 'DRY-RUN: commands are previewed, but are not sent to the physical motor.' -ForegroundColor Yellow
Write-Host ''

$remoteCommand = "docker exec -it sanditest bash -lc 'source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run rc_car_teleop keyboard_teleop --ros-args -r /rc_car/drive_cmd:=/rc_car/manual_drive_cmd'"

& wsl.exe -d Ubuntu-22.04 -- ssh -tt jetson-car $remoteCommand

Write-Host ''
Write-Host 'Keyboard teleop ended. The safety gate will keep a neutral command.' -ForegroundColor Yellow
Read-Host 'Press Enter to close this window'
