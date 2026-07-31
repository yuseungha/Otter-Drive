$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new()

Write-Host 'Jetson DRY-RUN 서버를 확인하고 있습니다...' -ForegroundColor Cyan

try {
    $taskIpOutput = (& ssh jetson-car 'hostname -I').Trim()
    if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($taskIpOutput)) {
        throw 'Jetson IP를 확인하지 못했습니다.'
    }
    $taskJetsonIp = ($taskIpOutput -split '\s+')[0]

    & ssh jetson-car 'bash /home/sandi/ros2_ws/src/laptop_teleop/scripts/start_dry_run_detached.sh'
    if ($LASTEXITCODE -ne 0) {
        throw 'Jetson에서 DRY-RUN 서버를 시작하지 못했습니다.'
    }

    $taskUrl = "http://${taskJetsonIp}:8765"
    $taskReady = $false
    for ($taskAttempt = 0; $taskAttempt -lt 20; $taskAttempt++) {
        try {
            $null = Invoke-RestMethod -Uri "$taskUrl/api/status" -TimeoutSec 1
            $taskReady = $true
            break
        } catch {
            Start-Sleep -Milliseconds 500
        }
    }

    if (-not $taskReady) {
        throw "조종기 서버가 준비되지 않았습니다: $taskUrl"
    }

    Start-Process $taskUrl
    Write-Host ''
    Write-Host '브라우저 조종 화면을 열었습니다.' -ForegroundColor Green
    Write-Host 'W/A/S/D는 반드시 브라우저 화면을 클릭한 뒤 누르세요.' -ForegroundColor Yellow
    Write-Host '이 명령 창에는 조작키를 입력하지 않습니다.' -ForegroundColor Yellow
    Start-Sleep -Seconds 3
} catch {
    Write-Host ''
    Write-Host "실행 실패: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host 'Jetson 전원, Wi-Fi, SSH 접속 상태를 확인하세요.' -ForegroundColor Yellow
    Read-Host 'Enter를 누르면 닫힙니다'
    exit 1
}

