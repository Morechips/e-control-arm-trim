$ErrorActionPreference = 'Stop'
Push-Location (Join-Path $PSScriptRoot '../..')
$routeOriginalPath = $env:PATH
try {
    $env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
    $buildDir = if ($env:CAR_TEST_BUILD_DIR) { $env:CAR_TEST_BUILD_DIR } else { 'build' }
    New-Item -ItemType Directory -Force $buildDir | Out-Null
    & gcc -DUART_TEST_HAS_HAL_GetTick -DUART_TEST_HAS_HAL_UART_Transmit -ffunction-sections -fdata-sections '-Wl,--gc-sections' tests/uart_hal_defaults.c Core/Src/uart_driver.c Core/Src/uart_tx_queue.c -std=c11 -Wall -Wextra -Werror -pedantic -DCAR_TEST_INPUTS_ENABLE=1 -Itests/car -ICore/Inc `
        Core/Src/action_fsm.c Core/Src/mission_fsm.c Core/Src/route_fsm.c Core/Src/maxicam.c Core/Src/laser.c Core/Src/board_inputs.c Core/Src/start_button.c tests/route/test_route_fsm.c `
        -o $buildDir/test_route_fsm.exe
    if ($LASTEXITCODE -ne 0) { throw 'Route FSM host test compile failed' }
    & (Join-Path $buildDir 'test_route_fsm.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Route FSM host tests failed' }
} finally {
    $env:PATH = $routeOriginalPath
    Pop-Location
}
