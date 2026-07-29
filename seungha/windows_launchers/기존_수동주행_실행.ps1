$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()

$taskInteractiveCommand = @'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
Write-Host 'Jetson 기존 수동주행 알고리즘에 연결합니다...' -ForegroundColor Cyan
& ssh -tt jetson-car 'docker exec -it sanditest bash /root/ros2_ws/src/laptop_teleop/scripts/container_existing_manual.sh'
$taskExitCode = $LASTEXITCODE
if ($taskExitCode -ne 0) {
    Write-Host ''
    Write-Host "수동주행 실행 실패 (exit=$taskExitCode)" -ForegroundColor Red
    Read-Host 'Enter를 누르면 닫힙니다'
} else {
    Write-Host ''
    Write-Host '안전하게 종료되었습니다.' -ForegroundColor Green
    Start-Sleep -Seconds 3
}
'@

$taskBytes = [System.Text.Encoding]::Unicode.GetBytes($taskInteractiveCommand)
$taskEncoded = [Convert]::ToBase64String($taskBytes)
Start-Process powershell.exe -ArgumentList @(
    '-NoLogo',
    '-NoProfile',
    '-EncodedCommand',
    $taskEncoded
)

