$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$armBtOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit -DUART_TEST_HAS_HAL_UART_Transmit_IT -DUART_TEST_HAS_HAL_UART_Receive_IT -DUART_TEST_HAS_HAL_UART_AbortTransmit -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic -Itests/arm_bt -ICore/Inc Core/Src/arm_tuner.c Core/Src/arm_control.c Core/Src/arm_kinematics.c Core/Src/arm_kinematics_legacy_project.c Core/Src/servo.c Core/Src/bluetooth_driver.c Core/Src/serial_io.c Core/Src/pid_tuner.c tests/arm_bt/test_arm_tuner.c -lm -o $buildDir/test_arm_bt.exe
    if ($LASTEXITCODE -ne 0) { throw 'Arm Bluetooth test compile failed' }
    & (Join-Path $buildDir 'test_arm_bt.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Arm Bluetooth tests failed' }
} finally { $env:PATH = $armBtOriginalPath; Pop-Location }

