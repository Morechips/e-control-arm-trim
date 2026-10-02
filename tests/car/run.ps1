$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $carOriginalPath = $env:PATH
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    New-Item -ItemType Directory -Force build | Out-Null
    foreach ($carMode in @(0, 1)) {
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic "-DCAR_PD10_STANDALONE_TEST=$carMode" -Itests/car -ICore/Inc tests/car/test_car.c Core/Src/serial_io.c Core/Src/bluetooth_driver.c Core/Src/pid_tuner.c Core/Src/motor_driver.c Core/Src/emm42_driver.c Core/Src/mecanum.c Core/Src/mecanum_test.c Core/Src/car_control.c Core/Src/servo_remote.c Core/Src/servo_pose.c Core/Src/turn_right.c Core/Src/heading_control.c Core/Src/jy61.c tests/car/mock_jy61_transport.c tests/car/mock_maxicam.c Core/Src/uart_bridge.c -o "build/test_car_$carMode.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Host test compile failed' }
    & "./build/test_car_$carMode.exe"
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
    }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -DCAR_MECANUM_TEST_MODE=1 -Itests/car -ICore/Inc tests/car/test_car.c Core/Src/serial_io.c Core/Src/bluetooth_driver.c Core/Src/pid_tuner.c Core/Src/motor_driver.c Core/Src/emm42_driver.c Core/Src/mecanum.c Core/Src/mecanum_test.c Core/Src/car_control.c Core/Src/servo_remote.c Core/Src/servo_pose.c Core/Src/turn_right.c Core/Src/heading_control.c Core/Src/jy61.c tests/car/mock_jy61_transport.c tests/car/mock_maxicam.c Core/Src/uart_bridge.c -o build/test_mecanum_mode.exe
    if ($LASTEXITCODE -ne 0) { throw 'Mecanum test mode compile failed' }
    & ./build/test_mecanum_mode.exe
    if ($LASTEXITCODE -ne 0) { throw 'Mecanum test mode failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc tests/car/test_usart2_bridge.c tests/car/mock_maxicam.c Core/Src/uart_bridge.c -o build/test_usart2_bridge.exe
    if ($LASTEXITCODE -ne 0) { throw 'USART2 bridge test compile failed' }
    & ./build/test_usart2_bridge.exe
    if ($LASTEXITCODE -ne 0) { throw 'USART2 bridge tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc tests/car/test_heading.c tests/car/mock_jy61_transport.c Core/Src/jy61.c Core/Src/heading_control.c -o build/test_heading.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading unit tests compile failed' }
    & ./build/test_heading.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading unit tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/car -ICore/Inc tests/car/test_turn_right.c tests/car/mock_jy61_transport.c Core/Src/jy61.c Core/Src/heading_control.c Core/Src/turn_right.c -o build/test_turn_right.exe
    if ($LASTEXITCODE -ne 0) { throw 'Right turn test compile failed' }
    & ./build/test_turn_right.exe
    if ($LASTEXITCODE -ne 0) { throw 'Right turn tests failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -pedantic -DCAR_HEADING_TEST_MODE=1 -Itests/car -ICore/Inc tests/car/test_car.c Core/Src/serial_io.c Core/Src/bluetooth_driver.c Core/Src/pid_tuner.c Core/Src/motor_driver.c Core/Src/emm42_driver.c Core/Src/mecanum.c Core/Src/mecanum_test.c Core/Src/car_control.c Core/Src/servo_remote.c Core/Src/servo_pose.c Core/Src/turn_right.c Core/Src/heading_control.c Core/Src/jy61.c tests/car/mock_jy61_transport.c tests/car/mock_maxicam.c Core/Src/uart_bridge.c -o build/test_heading_car.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading car tests compile failed' }
    & ./build/test_heading_car.exe
    if ($LASTEXITCODE -ne 0) { throw 'Heading car tests failed' }
} finally { $env:PATH = $carOriginalPath; Pop-Location }
