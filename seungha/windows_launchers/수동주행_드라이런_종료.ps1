$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()

try {
    & ssh jetson-car 'bash /home/sandi/ros2_ws/src/laptop_teleop/scripts/stop_dry_run.sh'
    if ($LASTEXITCODE -ne 0) {
        throw 'Jetson DRY-RUN 서버 종료에 실패했습니다.'
    }
    Write-Host 'DRY-RUN 서버를 안전하게 종료했습니다.' -ForegroundColor Green
    Start-Sleep -Seconds 2
} catch {
    Write-Host "종료 실패: $($_.Exception.Message)" -ForegroundColor Red
    Read-Host 'Enter를 누르면 닫힙니다'
    exit 1
}

