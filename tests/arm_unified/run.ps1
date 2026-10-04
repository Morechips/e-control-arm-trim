$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build-local' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -ffunction-sections -fdata-sections '-Wl,--gc-sections' `
        -DARM_TRIM_ENABLE=1 -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Receive_IT `
        -DUART_TEST_HAS_HAL_UART_Transmit_IT -DUART_TEST_HAS_HAL_UART_AbortTransmit `
        -Itests/servo -ICore/Inc tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c `
        Core/Src/servo.c Core/Src/bluetooth_driver.c Core/Src/arm_trim_input.c Core/Src/arm_trim_service.c `
        Core/Src/arm_trim_project.c Core/Src/arm_trim.c Core/Src/arm_collision.c Core/Src/arm_kinematics.c `
        tests/arm_unified/test_unified.c -lm -o (Join-Path $buildDir 'test_arm_unified.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Unified integration compile failed' }
    & (Join-Path $buildDir 'test_arm_unified.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Unified integration tests failed' }
} finally { Pop-Location }
