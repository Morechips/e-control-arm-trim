param(
    [switch]$ConfirmHardwareReady,
    [string]$OpenOcdExecutable = 'D:\Tool\xpack-openocd-0.12.0-7\bin\openocd.exe'
)

$ErrorActionPreference = 'Stop'
if (-not $ConfirmHardwareReady) {
    throw 'Refusing to flash. Support the arm, lift the wheels, verify SWD wiring and board power, then rerun with -ConfirmHardwareReady.'
}

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$configuration = Join-Path $projectRoot 'openocd\horco-cmsis-dap-stm32f407.cfg'
$firmware = Join-Path $projectRoot 'firmware_direct\stm32f407_bt_oled.elf'
if (-not (Test-Path -LiteralPath $OpenOcdExecutable -PathType Leaf)) {
    throw "OpenOCD not found: $OpenOcdExecutable"
}
if (-not (Test-Path -LiteralPath $configuration -PathType Leaf)) {
    throw "OpenOCD configuration not found: $configuration"
}
if (-not (Test-Path -LiteralPath $firmware -PathType Leaf)) {
    throw 'Firmware not found. Run scripts\build_firmware.ps1 first.'
}

Push-Location $projectRoot
try {
    $flashSucceeded = $false
    $lastOpenOcdExitCode = -1
    for ($attempt = 1; $attempt -le 3; ++$attempt) {
        Write-Output "OpenOCD flash attempt $attempt/3"
        & $OpenOcdExecutable -f '.\openocd\horco-cmsis-dap-stm32f407.cfg' `
            -c 'program firmware_direct/stm32f407_bt_oled.elf verify reset exit'
        $lastOpenOcdExitCode = $LASTEXITCODE
        if ($lastOpenOcdExitCode -eq 0) {
            $flashSucceeded = $true
            break
        }
        if ($attempt -lt 3) {
            Write-Warning 'OpenOCD failed; waiting 1 second before retrying the CMSIS-DAP connection.'
            Start-Sleep -Seconds 1
        }
    }
    if (-not $flashSucceeded) {
        throw "OpenOCD failed after 3 attempts; last exit code $lastOpenOcdExitCode. Re-enumerate the CMSIS-DAP USB device before retrying."
    }
} finally {
    Pop-Location
}
