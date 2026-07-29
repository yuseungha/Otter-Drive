$Host.UI.RawUI.WindowTitle = 'Jetson Camera-Guarded Manual Drive - LIVE HARDWARE'

Write-Host 'LIVE HARDWARE OUTPUT' -ForegroundColor Red -BackgroundColor Black
Write-Host 'W/S: throttle, A/D: steering, Space: throttle stop, C: center, X: software full stop'
Write-Host 'Arduino ESC output is limited to 70%: 1150-1850 us.' -ForegroundColor Yellow
Write-Host 'If control is abnormal, cut physical motor power immediately.' -ForegroundColor Yellow
Write-Host ''

$remoteCommand = "docker exec -it sanditest bash -lc 'source /opt/ros/humble/install/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run rc_car_teleop keyboard_teleop --ros-args -r /rc_car/drive_cmd:=/rc_car/manual_drive_cmd'"

& wsl.exe -d Ubuntu-22.04 -- ssh -tt jetson-car $remoteCommand

Write-Host ''
Write-Host 'Keyboard teleop ended. The safety gate should hold neutral.' -ForegroundColor Yellow
Read-Host 'Press Enter to close this window'
