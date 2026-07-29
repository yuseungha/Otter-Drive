$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()

$taskUploadCommand = @'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()
& ssh -tt jetson-car 'bash /home/sandi/ros2_ws/src/laptop_teleop/scripts/manual_flash_tuned_uno.sh'
$taskExitCode = $LASTEXITCODE
if ($taskExitCode -ne 0) {
    Write-Host ''
    Write-Host "업로드 실패 (exit=$taskExitCode)" -ForegroundColor Red
} else {
    Write-Host ''
    Write-Host '업로드가 완료되었습니다.' -ForegroundColor Green
}
Read-Host '결과를 확인한 뒤 Enter를 누르면 닫힙니다'
'@

$taskBytes = [System.Text.Encoding]::Unicode.GetBytes($taskUploadCommand)
$taskEncoded = [Convert]::ToBase64String($taskBytes)
Start-Process powershell.exe -ArgumentList @(
    '-NoLogo',
    '-NoProfile',
    '-EncodedCommand',
    $taskEncoded
)

