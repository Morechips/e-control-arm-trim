$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $carOriginalPath = $env:PATH
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    # Output directory is overridable so the repository's generated `build`
    # folder can stay untouched (see README: host regressions).
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    # This suite drives the bench buttons (PE0/PC1/PE4), so they stay compiled in.
    $carFlags = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic',
        '-DCAR_TEST_INPUTS_ENABLE=1', '-Itests/car', '-ICore/Inc')
    # Keep the integration source list in one place, including Servo and board inputs.
    $carSources = @(
        'tests/car/test_car.c', 'Core/Src/serial_io.c', 'Core/Src/bluetooth_driver.c',
        'Core/Src/pid_tuner.c', 'Core/Src/motor_driver.c', 'Core/Src/emm42_driver.c',
        'Core/Src/mecanum.c', 'Core/Src/mecanum_test.c', 'Core/Src/car_control.c',
        'Core/Src/remote_heading.c', 'Core/Src/servo.c',
        'Core/Src/turn_right.c', 'Core/Src/heading_control.c',
        'Core/Src/jy61.c', 'tests/car/mock_jy61_transport.c', 'tests/car/mock_maxicam.c',
        'Core/Src/uart_bridge.c', 'Core/Src/board_inputs.c', 'Core/Src/start_button.c')
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc tests/car/test_remote_heading.c Core/Src/remote_heading.c Core/Src/heading_control.c Core/Src/jy61.c tests/car/mock_jy61_transport.c -lm -o $buildDir/test_remote_heading.exe
    if ($LASTEXITCODE -ne 0) { throw 'Remote heading unit compile failed' }
    & (Join-Path $buildDir 'test_remote_heading.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Remote heading unit tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc tests/car/test_start_button.c Core/Src/start_button.c -o $buildDir/test_start_button.exe
    if ($LASTEXITCODE -ne 0) { throw 'Start button test compile failed' }
    & (Join-Path $buildDir 'test_start_button.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Start button tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/display -ICore/Inc Core/Src/ssd1306.c tests/display/test_display.c -o "$buildDir/test_display.exe"
    if ($LASTEXITCODE -ne 0) { throw 'OLED test compile failed' }
    & (Join-Path $buildDir 'test_display.exe')
    if ($LASTEXITCODE -ne 0) { throw 'OLED tests failed' }
    foreach ($inputMode in @(0, 1)) {
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic "-DCAR_TEST_INPUTS_ENABLE=$inputMode" -Itests/car -ICore/Inc Core/Src/laser.c Core/Src/board_inputs.c Core/Src/start_button.c tests/car/test_laser_inputs.c -o "$buildDir/test_laser_inputs_$inputMode.exe"
        if ($LASTEXITCODE -ne 0) { throw 'Laser input test compile failed' }
        & (Join-Path $buildDir "test_laser_inputs_$inputMode.exe")
        if ($LASTEXITCODE -ne 0) { throw 'Laser input tests failed' }
    }
    foreach ($inputMode in @(0, 1)) {
        & gcc -std=c11 -Wall -Wextra -Werror -pedantic "-DCAR_TEST_INPUTS_ENABLE=$inputMode" -Itests/car -ICore/Inc Core/Src/board_inputs.c Core/Src/start_button.c Core/Src/laser.c tests/car/test_board_inputs.c -o "$buildDir/test_board_inputs_$inputMode.exe"
        if ($LASTEXITCODE -ne 0) { throw 'Board input compile failed' }
        & (Join-Path $buildDir "test_board_inputs_$inputMode.exe")
        if ($LASTEXITCODE -ne 0) { throw 'Board input tests failed' }
    }
    foreach ($carMode in @(0, 1)) {
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit -DUART_TEST_HAS_HAL_UART_Transmit_IT -DUART_TEST_HAS_HAL_UART_Receive_IT -DUART_TEST_HAS_HAL_UART_AbortTransmit -DUART_TEST_HAS_HAL_UART_AbortReceive -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c @carFlags "-DCAR_PD10_STANDALONE_TEST=$carMode" @carSources -o "$buildDir/test_car_$carMode.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Host test compile failed' }
    & (Join-Path $buildDir "test_car_$carMode.exe")
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
    }
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit -DUART_TEST_HAS_HAL_UART_Transmit_IT -DUART_TEST_HAS_HAL_UART_Receive_IT -DUART_TEST_HAS_HAL_UART_AbortTransmit -DUART_TEST_HAS_HAL_UART_AbortReceive -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c @carFlags -DCAR_MECANUM_TEST_MODE=1 @carSources -o $buildDir/test_mecanum_mode.exe
    if ($LASTEXITCODE -ne 0) { throw 'Mecanum test mode compile failed' }
    & (Join-Path $buildDir 'test_mecanum_mode.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Mecanum test mode failed' }
    & gcc -DUART_TEST_HAS_HAL_UART_Transmit_IT -DUART_TEST_HAS_HAL_UART_Receive_IT -DUART_TEST_HAS_HAL_UART_AbortReceive -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc tests/car/test_usart2_bridge.c tests/car/mock_maxicam.c Core/Src/uart_bridge.c -o $buildDir/test_usart2_bridge.exe
    if ($LASTEXITCODE -ne 0) { throw 'USART2 bridge test compile failed' }
    & (Join-Path $buildDir 'test_usart2_bridge.exe')
    if ($LASTEXITCODE -ne 0) { throw 'USART2 bridge tests failed' }
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc tests/car/test_heading.c tests/car/mock_jy61_transport.c Core/Src/jy61.c Core/Src/heading_control.c -o $buildDir/test_heading.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading unit tests compile failed' }
    & (Join-Path $buildDir 'test_heading.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Heading unit tests failed' }
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc tests/car/test_turn_right.c tests/car/mock_jy61_transport.c Core/Src/jy61.c Core/Src/heading_control.c Core/Src/turn_right.c -o $buildDir/test_turn_right.exe
    if ($LASTEXITCODE -ne 0) { throw 'Right turn test compile failed' }
    & (Join-Path $buildDir 'test_turn_right.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Right turn tests failed' }
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit -DUART_TEST_HAS_HAL_UART_Transmit_IT -DUART_TEST_HAS_HAL_UART_Receive_IT -DUART_TEST_HAS_HAL_UART_AbortTransmit -DUART_TEST_HAS_HAL_UART_AbortReceive -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c @carFlags -DCAR_HEADING_TEST_MODE=1 @carSources -o $buildDir/test_heading_car.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading car tests compile failed' }
    & (Join-Path $buildDir 'test_heading_car.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Heading car tests failed' }
} finally { $env:PATH = $carOriginalPath; Pop-Location }
