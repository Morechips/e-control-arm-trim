$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$trimInputOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build-local' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Receive_IT -DARM_TRIM_ENABLE=1 -ffunction-sections -fdata-sections '-Wl,--gc-sections' -std=c11 -Wall -Wextra -Werror -pedantic -Itests/servo -ICore/Inc tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c Core/Src/bluetooth_driver.c tests/arm_trim_input/test_trim_packets.c -o (Join-Path $buildDir 'test_trim_packets.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Trim packet compile failed' }
    & (Join-Path $buildDir 'test_trim_packets.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Trim packet tests failed' }
    & gcc -DARM_TRIM_ENABLE=1 -std=c11 -Wall -Wextra -Werror -pedantic -Itests/servo -ICore/Inc Core/Src/arm_trim_input.c tests/arm_trim_input/test_trim_frontend.c -lm -o (Join-Path $buildDir 'test_trim_frontend.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Trim frontend compile failed' }
    & (Join-Path $buildDir 'test_trim_frontend.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Trim frontend tests failed' }
} finally { $env:PATH = $trimInputOriginalPath; Pop-Location }
