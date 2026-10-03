$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$armOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit_IT -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic -Itests/arm -ICore/Inc Core/Src/arm_control.c Core/Src/servo.c tests/arm/test_arm.c -o $buildDir/test_arm.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm host test compile failed' }
    & (Join-Path $buildDir 'test_arm.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Arm host tests failed' }
} finally {
    $env:PATH = $armOriginalPath
    Pop-Location
}

