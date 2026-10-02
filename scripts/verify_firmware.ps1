param(
    [string]$OpenOcdExecutable = 'D:\Tool\xpack-openocd-0.12.0-7\bin\openocd.exe'
)

$ErrorActionPreference = 'Stop'
$trimVerifyRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$trimVerifyImage = Join-Path $trimVerifyRoot 'firmware_direct\stm32f407_bt_oled.hex'
$trimVerifyConfiguration = Join-Path $trimVerifyRoot 'openocd\horco-cmsis-dap-stm32f407.cfg'
$trimVerifyCommandsFile = Join-Path $trimVerifyRoot 'openocd\verify-current-firmware.cfg'
foreach ($trimVerifyPath in @($OpenOcdExecutable, $trimVerifyImage, $trimVerifyConfiguration, $trimVerifyCommandsFile)) {
    if (-not (Test-Path -LiteralPath $trimVerifyPath -PathType Leaf)) {
        throw "Required file not found: $trimVerifyPath"
    }
}
Write-Output "Reference image: $trimVerifyImage"
Write-Output ('SHA256: ' + (Get-FileHash -LiteralPath $trimVerifyImage -Algorithm SHA256).Hash)
Write-Output 'Verification only; no flash erase/write. Park the car before checking.'
Write-Output 'The MCU is briefly halted, then resumed even if image comparison fails.'

Push-Location $trimVerifyRoot
try {
    # Pass a file instead of a quoted Tcl command through native argv.
    # OpenOCD writes informational output to stderr as well as stdout.
    $trimVerifyNativePreference = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $trimVerifyOutput = & $OpenOcdExecutable -f '.\openocd\horco-cmsis-dap-stm32f407.cfg' -f '.\openocd\verify-current-firmware.cfg' 2>&1
        $trimVerifyExitCode = $LASTEXITCODE
    } finally { $ErrorActionPreference = $trimVerifyNativePreference }
    $trimVerifyLines = @($trimVerifyOutput | ForEach-Object { $_.ToString() })
    $trimVerifyLines | Write-Output
    if ($trimVerifyExitCode -ne 0) {
        throw "OpenOCD connection/verification failed (exit $trimVerifyExitCode). This does not establish which firmware is installed."
    }
    if ($trimVerifyLines -notcontains 'CURRENT_FIRMWARE_VERIFIED') {
        throw 'Board image does not match the reference, or verification did not complete. No firmware was written.'
    }
} finally { Pop-Location }
