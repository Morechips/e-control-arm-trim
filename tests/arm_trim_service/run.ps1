$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$serviceOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build-local' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/zlis2 -ICore/Inc Core/Src/arm_trim_service.c Core/Src/arm_trim.c Core/Src/arm_collision.c Core/Src/arm_kinematics.c Core/Src/arm_trim_project.c tests/arm_trim_service/test_arm_trim_service.c -lm -o $buildDir/test_arm_trim_service.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim service compile failed' }
    & (Join-Path $buildDir 'test_arm_trim_service.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim service tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ffunction-sections -fdata-sections '-Wl,--gc-sections' -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit_IT -DUART_TEST_HAS_HAL_UART_AbortTransmit -Itests/zlis2 -ICore/Inc tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c Core/Src/servo.c Core/Src/arm_trim_service.c Core/Src/arm_trim.c Core/Src/arm_collision.c Core/Src/arm_kinematics.c Core/Src/arm_trim_project.c tests/arm_trim_service/test_arm_trim_pipeline.c -lm -o $buildDir/test_arm_trim_pipeline.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim pipeline compile failed' }
    & (Join-Path $buildDir 'test_arm_trim_pipeline.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Arm trim pipeline tests failed' }
} finally { $env:PATH = $serviceOriginalPath; Pop-Location }
