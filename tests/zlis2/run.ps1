$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$zlis2OriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -DUART_TEST_HAS_HAL_UART_Transmit_IT -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_AbortTransmit -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic -Itests/zlis2 -ICore/Inc Core/Src/servo.c tests/zlis2/test_zlis2.c -o $buildDir/test_zlis2.exe
    if ($LASTEXITCODE -ne 0) { throw 'ZLIS2 host test compile failed' }
    & (Join-Path $buildDir 'test_zlis2.exe')
    if ($LASTEXITCODE -ne 0) { throw 'ZLIS2 host tests failed' }
} finally {
    $env:PATH = $zlis2OriginalPath
    Pop-Location
}
