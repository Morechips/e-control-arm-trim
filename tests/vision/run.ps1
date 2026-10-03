$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$visionOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc Core/Src/vision_data.c tests/vision/test_vision_data.c -o $buildDir/test_vision_data.exe
    if ($LASTEXITCODE -ne 0) { throw 'Vision host test compile failed' }
    & (Join-Path $buildDir 'test_vision_data.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Vision host tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ICore/Inc tests/vision/test_detect_data.c -o $buildDir/test_detect_data.exe
    if ($LASTEXITCODE -ne 0) { throw 'MaxiCam data test compile failed' }
    & (Join-Path $buildDir 'test_detect_data.exe')
    if ($LASTEXITCODE -ne 0) { throw 'MaxiCam data tests failed' }
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc Core/Src/maxicam.c tests/vision/test_maxicam.c -o $buildDir/test_maxicam.exe
    if ($LASTEXITCODE -ne 0) { throw 'MaxiCam UART test compile failed' }
    & (Join-Path $buildDir 'test_maxicam.exe')
    if ($LASTEXITCODE -ne 0) { throw 'MaxiCam UART tests failed' }
} finally {
    $env:PATH = $visionOriginalPath
    Pop-Location
}
