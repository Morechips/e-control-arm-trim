$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$servoOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    foreach ($inputMode in @(0, 1)) {
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit_IT -DUART_TEST_HAS_HAL_UART_Receive_IT -DUART_TEST_HAS_HAL_UART_AbortTransmit -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic "-DCAR_TEST_INPUTS_ENABLE=$inputMode" -Itests/servo -ICore/Inc Core/Src/servo.c Core/Src/board_inputs.c Core/Src/start_button.c Core/Src/bluetooth_driver.c Core/Src/serial_io.c Core/Src/pid_tuner.c tests/servo/test_bluetooth_servo.c -lm -o $buildDir/test_bluetooth_servo.exe
    if ($LASTEXITCODE -ne 0) { throw 'Servo direct test compile failed' }
    & (Join-Path $buildDir 'test_bluetooth_servo.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Servo direct tests failed' }
    }
} finally { $env:PATH = $servoOriginalPath; Pop-Location }
